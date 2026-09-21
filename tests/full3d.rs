// SPDX-License-Identifier: Apache-2.0
//! R16 — expanding a routed edge into a full three-dimensional path.
//!
//! 836 edges from 69 designs: 633 that gain points, 143 converted without changing, and **60 the
//! pass skips**, which must come out byte-identical.
//!
//! ⚠️ Three behaviours the corpus cannot witness are pinned by constructed cases below, each with
//! the limitation asserted so a later capture that does reach one fails loudly.

use serde_json::Value;
use vyges_grt::{convert_edge_to_full_3d, Edge3D, Point3D, RouteType};

const NUM_LAYERS: usize = 12;

struct Case {
    design: String,
    net_id: usize,
    edge_id: usize,
    len: i32,
    routelen_in: i32,
    type_in: i32,
    npts_in: usize,
    before: Vec<Point3D>,
    after: Vec<Point3D>,
    routelen_out: i32,
    type_out: i32,
}

fn route_type(code: i32) -> RouteType {
    match code {
        0 => RouteType::NoRoute,
        1 => RouteType::LRoute,
        2 => RouteType::ZRoute,
        _ => RouteType::MazeRoute,
    }
}

fn cases() -> Vec<Case> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/full3d.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let pts = |v: &Value| -> Vec<Point3D> {
        v.as_array().expect("pts").iter().map(|p| {
            let a = p.as_array().expect("triple");
            Point3D {
                x: a[0].as_i64().expect("x") as i16,
                y: a[1].as_i64().expect("y") as i16,
                layer: a[2].as_i64().expect("l") as i16,
            }
        }).collect()
    };
    v["edges"].as_array().expect("edges").iter().map(|c| {
        let i = |k: &str| c[k].as_i64().unwrap_or_else(|| panic!("{k}"));
        Case {
            design: c["design"].as_str().expect("design").to_string(),
            net_id: i("net_id") as usize,
            edge_id: i("edge_id") as usize,
            len: i("len") as i32,
            routelen_in: i("routelen_in") as i32,
            type_in: i("type_in") as i32,
            npts_in: i("npts_in") as usize,
            before: pts(&c["before"]),
            after: pts(&c["after"]),
            routelen_out: i("routelen_out") as i32,
            type_out: i("type_out") as i32,
        }
    }).collect()
}

fn run(c: &Case) -> Edge3D {
    let mut e = Edge3D {
        len: c.len,
        route_type: route_type(c.type_in),
        routelen: c.routelen_in,
        grids: c.before.clone(),
    };
    convert_edge_to_full_3d(&mut e, NUM_LAYERS);
    e
}

#[test]
fn the_expanded_paths_match_the_reference() {
    let all = cases();
    assert!(all.len() >= 700, "corpus too thin: {}", all.len());
    let (mut grew, mut same, mut skipped) = (0usize, 0usize, 0usize);

    for c in &all {
        let got = run(c);
        assert_eq!(
            got.grids, c.after,
            "points of net {} edge {} on {} ({} steps in)",
            c.net_id, c.edge_id, c.design, c.routelen_in
        );
        assert_eq!(
            got.routelen, c.routelen_out,
            "step count of net {} edge {} on {}", c.net_id, c.edge_id, c.design
        );
        assert_eq!(
            got.route_type, route_type(c.type_out),
            "route type of net {} edge {} on {}", c.net_id, c.edge_id, c.design
        );
        if c.len <= 0 { skipped += 1 } else if c.after.len() > c.before.len() { grew += 1 } else { same += 1 }
    }
    assert!(grew >= 400, "too few edges gain points: {grew}");
    // ⚠️ An edge that qualifies but needs no insertion still goes through the walk and is
    // rewritten, so it is not the same case as one that is skipped.
    assert!(same >= 50, "too few edges are converted without changing: {same}");
    assert!(skipped >= 20, "too few edges are skipped: {skipped}");
}

/// ⛔ Every layer transition must appear, in both directions and across more than one layer.
#[test]
fn the_corpus_exercises_every_kind_of_layer_step() {
    let all = cases();
    let (mut up, mut down, mut jump) = (0usize, 0usize, 0usize);
    for c in &all {
        for w in c.before.windows(2) {
            let (a, b) = (w[0].layer, w[1].layer);
            if b > a { up += 1 }
            if b < a { down += 1 }
            if (b - a).abs() > 1 { jump += 1 }
        }
    }
    assert!(up >= 300, "too few upward layer steps: {up}");
    assert!(down >= 300, "too few downward layer steps: {down}");
    // ⚠️ Without a multi-layer step the inserted run is always a single point and the loop
    // bounds are never tested.
    assert!(jump >= 50, "too few steps crossing more than one layer: {jump}");
}

/// ⛔ An edge the pass skips must come out **byte-identical** — points, step count and type.
#[test]
fn a_skipped_edge_is_untouched() {
    let all = cases();
    let mut skipped = 0usize;
    for c in all.iter().filter(|c| c.len <= 0) {
        let got = run(c);
        assert_eq!(got.grids, c.before, "net {} edge {} on {} was rewritten",
                   c.net_id, c.edge_id, c.design);
        assert_eq!(got.routelen, c.routelen_in, "its step count moved");
        assert_eq!(got.route_type, route_type(c.type_in), "its route type was stamped");
        skipped += 1;
    }
    assert!(skipped >= 20, "no skipped edge in the corpus: {skipped}");
}

// ─── What the corpus cannot witness ─────────────────────────────────────────────────────────

/// ⛔ **Every captured edge already enters as a maze route**, so the stamp this pass applies is
/// invisible in the golden: an implementation that never set the type would score the same.
///
/// ⚠️ Asserted as the limitation. The constructed case below is what actually pins the stamp.
#[test]
fn no_captured_edge_enters_with_another_route_type() {
    let all = cases();
    let other = all.iter().filter(|c| c.type_in != 3).count();
    assert_eq!(
        other, 0,
        "{other} captured edges now enter with another route type — the stamp is witnessed by \
         the corpus and no longer needs a constructed case"
    );
    assert!(all.len() >= 700, "corpus too thin to call the stamp unwitnessed");
}

/// ⛔ The route type is stamped on any edge that qualifies, whatever it was before.
#[test]
fn a_converted_edge_is_stamped_a_maze_route() {
    let mut e = Edge3D {
        len: 5,
        route_type: RouteType::LRoute,
        routelen: 1,
        grids: vec![Point3D { x: 0, y: 0, layer: 2 }, Point3D { x: 1, y: 0, layer: 2 }],
    };
    convert_edge_to_full_3d(&mut e, NUM_LAYERS);
    assert_eq!(e.route_type, RouteType::MazeRoute, "the type must be stamped");
    // ⚠️ And nothing else moved: no layer change means no inserted point.
    assert_eq!(e.grids.len(), 2);
    assert_eq!(e.routelen, 1);
}

/// ⛔ **Every captured skipped edge has a length of exactly zero**, never negative, so the
/// comparison could be `== 0` for all the corpus can tell.
#[test]
fn no_captured_edge_has_a_negative_length() {
    let all = cases();
    let negative = all.iter().filter(|c| c.len < 0).count();
    assert_eq!(
        negative, 0,
        "{negative} captured edges now have a negative length — the comparison is witnessed and \
         the constructed case below is redundant"
    );
}

/// ⛔ **A length of exactly zero is skipped**, and this is the case the corpus cannot decide.
///
/// ⚠️ All 60 skipped edges in the golden hold a single point and no steps, so converting them
/// would be a no-op — the gate could read `>= 0` for all the corpus can tell. Only an edge with
/// zero length **and** something to expand separates the two, and no design produces one.
#[test]
fn a_length_of_exactly_zero_is_skipped_though_there_is_work_to_do() {
    let before = vec![
        Point3D { x: 2, y: 2, layer: 1 },
        Point3D { x: 5, y: 2, layer: 4 },
    ];
    let mut e = Edge3D {
        len: 0,
        route_type: RouteType::ZRoute,
        routelen: 1,
        grids: before.clone(),
    };
    convert_edge_to_full_3d(&mut e, NUM_LAYERS);
    assert_eq!(
        e.grids, before,
        "a zero length must be skipped — the gate is 'greater than zero', not 'not negative'"
    );
    assert_eq!(e.routelen, 1, "its step count must not move");
    assert_eq!(e.route_type, RouteType::ZRoute, "and it must not be stamped");
}

/// ⚠️ Every skipped edge in the corpus is one where conversion would change nothing anyway —
/// asserted, because it is what makes the constructed case above necessary rather than redundant.
#[test]
fn no_captured_skipped_edge_would_change_if_converted() {
    let all = cases();
    let mut checked = 0usize;
    for c in all.iter().filter(|c| c.len <= 0) {
        assert!(
            c.routelen_in == 0 && c.before.len() == 1,
            "net {} edge {} on {} is skipped and HAS work to do ({} steps, {} points) — the \
             gate's boundary is now witnessed by the corpus",
            c.net_id, c.edge_id, c.design, c.routelen_in, c.before.len()
        );
        checked += 1;
    }
    assert!(checked >= 20, "no skipped edge to check: {checked}");
}

/// ⛔ A negative length is skipped too: the gate is "greater than zero", not "not zero".
#[test]
fn a_negative_length_is_skipped_as_well() {
    let before = vec![
        Point3D { x: 0, y: 0, layer: 1 },
        Point3D { x: 0, y: 0, layer: 4 },
    ];
    let mut e = Edge3D {
        len: -1,
        route_type: RouteType::ZRoute,
        routelen: 1,
        grids: before.clone(),
    };
    convert_edge_to_full_3d(&mut e, NUM_LAYERS);
    assert_eq!(e.grids, before, "a negative length must be skipped, not converted");
    assert_eq!(e.route_type, RouteType::ZRoute, "and must not be stamped");
}

/// ⛔ **No captured edge carries points past its step count**, so the golden cannot show which of
/// the two is the authority.
#[test]
fn no_captured_edge_carries_points_past_its_step_count() {
    let all = cases();
    let extra = all.iter().filter(|c| c.npts_in > c.routelen_in.max(0) as usize + 1).count();
    assert_eq!(
        extra, 0,
        "{extra} captured edges now carry points past the step count — the authority is \
         witnessed and the constructed case below is redundant"
    );
}

/// ⛔ The step count is the authority. The walk reads that many steps and one final point;
/// anything beyond is dropped rather than copied.
#[test]
fn points_past_the_step_count_are_dropped() {
    let mut e = Edge3D {
        len: 5,
        route_type: RouteType::MazeRoute,
        routelen: 1,
        grids: vec![
            Point3D { x: 0, y: 0, layer: 2 },
            Point3D { x: 1, y: 0, layer: 2 },
            // Past the step count — the reference never reads these.
            Point3D { x: 9, y: 9, layer: 9 },
            Point3D { x: 8, y: 8, layer: 8 },
        ],
    };
    convert_edge_to_full_3d(&mut e, NUM_LAYERS);
    assert_eq!(
        e.grids,
        vec![Point3D { x: 0, y: 0, layer: 2 }, Point3D { x: 1, y: 0, layer: 2 }],
        "points past the step count were copied instead of dropped"
    );
    assert_eq!(e.routelen, 1);
}

/// ⛔ **The inserted points take the NEXT point's coordinates**, so a layer change becomes a move
/// in the plane and then a stack of vias at the destination — never a stack at the origin.
#[test]
fn the_inserted_points_sit_at_the_destination() {
    let mut e = Edge3D {
        len: 5,
        route_type: RouteType::MazeRoute,
        routelen: 1,
        grids: vec![
            Point3D { x: 0, y: 0, layer: 1 },
            Point3D { x: 7, y: 0, layer: 4 },
        ],
    };
    convert_edge_to_full_3d(&mut e, NUM_LAYERS);
    assert_eq!(
        e.grids,
        vec![
            Point3D { x: 0, y: 0, layer: 1 },
            // ⛔ At x=7 already, still on the origin's layer.
            Point3D { x: 7, y: 0, layer: 1 },
            Point3D { x: 7, y: 0, layer: 2 },
            Point3D { x: 7, y: 0, layer: 3 },
            Point3D { x: 7, y: 0, layer: 4 },
        ],
        "the via stack must sit at the destination, on the destination's coordinates"
    );
    assert_eq!(e.routelen, 4, "the step count is recomputed from the expanded path");
}

/// ⛔ Descending is the mirror of ascending, with the same coordinates rule.
#[test]
fn a_descending_layer_change_expands_the_same_way() {
    let mut e = Edge3D {
        len: 5,
        route_type: RouteType::MazeRoute,
        routelen: 1,
        grids: vec![
            Point3D { x: 0, y: 3, layer: 4 },
            Point3D { x: 0, y: 6, layer: 1 },
        ],
    };
    convert_edge_to_full_3d(&mut e, NUM_LAYERS);
    assert_eq!(
        e.grids,
        vec![
            Point3D { x: 0, y: 3, layer: 4 },
            Point3D { x: 0, y: 6, layer: 4 },
            Point3D { x: 0, y: 6, layer: 3 },
            Point3D { x: 0, y: 6, layer: 2 },
            Point3D { x: 0, y: 6, layer: 1 },
        ]
    );
}
