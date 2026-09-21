// SPDX-License-Identifier: Apache-2.0
//! R19 — the three-dimensional checks and the via bookkeeping.
//!
//! Four goldens from 71 designs: `threedvia.json` (6 runs), `overflow3d.json` (10 grids, **7 of
//! them actually overflowing**), `checkroute.json` (600 nets) and `pincoverage.json` (600 nets,
//! 2,240 pins).
//!
//! ⛔ **The checker finds nothing and the pin-coverage pass adds nothing, on every captured
//! design.** Both are asserted as absences with the margin measured, because a checker that has
//! never fired is a checker whose conditions are untested — so each condition is also driven by a
//! constructed case.

use serde_json::Value;
use vyges_grt::{
    check_route_3d, ensure_pin_coverage, get_overflow_3d, three_d_via, Cell3D, Point3D,
    RouteDefect, RoutedEdge, RoutedNode,
};

fn load(name: &str) -> Vec<Value> {
    let path = format!("{}/examples/grt_gate/{name}.json", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["records"].as_array().expect("records").clone()
}

fn pts(v: &Value) -> Vec<Point3D> {
    v.as_array().expect("pts").iter().map(|p| {
        let a = p.as_array().expect("triple");
        Point3D {
            x: a[0].as_i64().expect("x") as i16,
            y: a[1].as_i64().expect("y") as i16,
            layer: a[2].as_i64().expect("l") as i16,
        }
    }).collect()
}

fn edge(v: &Value) -> RoutedEdge {
    RoutedEdge {
        len: v["len"].as_i64().expect("len") as i32,
        routelen: v["routelen"].as_i64().expect("rl") as i32,
        n1: v.get("n1").and_then(|x| x.as_u64()).unwrap_or(0) as usize,
        n2: v.get("n2").and_then(|x| x.as_u64()).unwrap_or(0) as usize,
        grids: pts(&v["grids"]),
    }
}

// ─── The via count ──────────────────────────────────────────────────────────────────────────

#[test]
fn the_via_count_matches_the_reference() {
    let all = load("threedvia");
    assert!(!all.is_empty(), "no records");
    let (mut runs, mut edges, mut with_vias) = (0usize, 0usize, 0usize);

    for r in &all {
        let es: Vec<RoutedEdge> = r["edges"].as_array().expect("edges").iter()
            .map(edge).collect();
        let want = r["want_vias"].as_i64().expect("vias") as i32;
        assert_eq!(
            three_d_via(&es), want,
            "via count on {}", r["design"].as_str().expect("design")
        );
        runs += 1;
        edges += es.len();
        with_vias += usize::from(want > 0);
    }
    assert!(edges >= 10, "too few edges: {edges}");
    assert!(with_vias >= 3, "too few runs with any via: {with_vias}");
    assert_eq!(runs, all.len());
}

/// ⛔ An edge with no length is not counted, however many layer changes it has — the same gate
/// `ConvertToFull3DType2` uses, and the opposite of the checker's.
#[test]
fn an_edge_without_length_contributes_no_vias() {
    let stack = vec![
        Point3D { x: 0, y: 0, layer: 1 },
        Point3D { x: 0, y: 0, layer: 2 },
        Point3D { x: 0, y: 0, layer: 3 },
    ];
    let counted = RoutedEdge { len: 1, routelen: 2, n1: 0, n2: 0, grids: stack.clone() };
    let skipped = RoutedEdge { len: 0, routelen: 2, n1: 0, n2: 0, grids: stack };
    assert_eq!(three_d_via(&[counted]), 2);
    assert_eq!(three_d_via(&[skipped]), 0, "a zero length must contribute nothing");
}

// ─── The overflow ───────────────────────────────────────────────────────────────────────────

#[test]
fn the_three_dimensional_overflow_matches_the_reference() {
    let all = load("overflow3d");
    assert!(!all.is_empty(), "no records");
    let (mut cells, mut overflowing) = (0usize, 0usize);

    for r in &all {
        let cs: Vec<Cell3D> = r["cells"].as_array().expect("cells").iter().map(|c| {
            let a = c.as_array().expect("row");
            // ⚠️ The row is [direction, layer, x, y, usage, capacity]. Reading usage at
            // index 3 picks up the y coordinate instead, which scored every grid as massively
            // overflowing — the first version of this test did exactly that.
            Cell3D {
                horizontal: a[0].as_str().expect("dir") == "H",
                usage: a[4].as_i64().expect("usage") as i32,
                capacity: a[5].as_i64().expect("cap") as i32,
            }
        }).collect();
        let got = get_overflow_3d(&cs);
        let design = r["design"].as_str().expect("design");
        assert_eq!(got.horizontal, r["want_h"].as_i64().expect("h") as i32,
                   "horizontal overflow on {design}");
        assert_eq!(got.vertical, r["want_v"].as_i64().expect("v") as i32,
                   "vertical overflow on {design}");
        assert_eq!(got.max_horizontal, r["want_max_h"].as_i64().expect("mh") as i32,
                   "worst horizontal cell on {design}");
        assert_eq!(got.max_vertical, r["want_max_v"].as_i64().expect("mv") as i32,
                   "worst vertical cell on {design}");
        assert_eq!(got.total, r["want_total"].as_i64().expect("t") as i32,
                   "total overflow on {design}");
        // ⛔ The number the reference RETURNS, which is not the congestion.
        assert_eq!(got.total_usage, r["want_usage"].as_i64().expect("u") as i32,
                   "total usage on {design}");
        cells += cs.len();
        overflowing += usize::from(got.total > 0);
    }
    assert!(cells >= 10_000, "too few cells: {cells}");
    // ⛔ Without an overflowing grid every figure but the usage is zero and proves nothing.
    assert!(overflowing >= 5, "too few overflowing grids: {overflowing}");
}

/// ⚠️ A cell exactly at capacity is not overflowing, and usage counts even where overflow does
/// not.
#[test]
fn the_overflow_threshold_is_strict_and_usage_counts_everywhere() {
    let cells = [
        Cell3D { horizontal: true, usage: 5, capacity: 5 },
        Cell3D { horizontal: true, usage: 7, capacity: 5 },
        Cell3D { horizontal: false, usage: 9, capacity: 5 },
        Cell3D { horizontal: false, usage: 1, capacity: 5 },
    ];
    let got = get_overflow_3d(&cells);
    assert_eq!(got.horizontal, 2, "only the cell above capacity counts");
    assert_eq!(got.vertical, 4);
    assert_eq!(got.max_horizontal, 2);
    assert_eq!(got.max_vertical, 4);
    assert_eq!(got.total, 6);
    assert_eq!(got.total_usage, 22, "usage counts every cell, overflowing or not");
}

// ─── The route checker ──────────────────────────────────────────────────────────────────────

#[test]
fn the_checker_finds_nothing_on_any_captured_net() {
    let all = load("checkroute");
    assert!(all.len() >= 400, "corpus too thin: {}", all.len());
    let (mut nets, mut edges) = (0usize, 0usize);

    for r in &all {
        let nodes: Vec<RoutedNode> = r["nodes"].as_array().expect("nodes").iter().map(|n| {
            let pl = n["pin_layer"].as_i64().expect("pl");
            RoutedNode {
                x: n["x"].as_i64().expect("x") as i16,
                y: n["y"].as_i64().expect("y") as i16,
                bot_layer: n["botL"].as_i64().expect("b") as i16,
                top_layer: n["topL"].as_i64().expect("t") as i16,
                // ⚠️ The capture writes -1 for a Steiner node, which carries no pin.
                pin_layer: if pl >= 0 { Some(pl as i16) } else { None },
            }
        }).collect();
        let es: Vec<RoutedEdge> = r["edges"].as_array().expect("edges").iter()
            .map(edge).collect();

        let defects = check_route_3d(&nodes, &es);
        assert!(
            defects.is_empty(),
            "the checker reported {:?} on {} net {} — either the routing is wrong or our \
             checker is",
            defects, r["design"].as_str().expect("design"), r["net_id"]
        );
        nets += 1;
        edges += es.len();
    }
    assert!(edges >= 1_000, "too few edges checked: {edges}");
    assert!(nets >= 400);
}

/// ⛔ Every condition the checker tests, driven on purpose — a checker that has only ever
/// returned nothing is a checker whose conditions are untested.
#[test]
fn every_defect_the_checker_knows_can_be_provoked() {
    let node = |x, y, b, t, pin| RoutedNode {
        x, y, bot_layer: b, top_layer: t, pin_layer: pin,
    };
    let p = |x, y, l| Point3D { x, y, layer: l };

    // A pin below its node's range, and one above it.
    let nodes = vec![node(0, 0, 2, 4, Some(1)), node(9, 9, 2, 4, Some(5))];
    assert_eq!(
        check_route_3d(&nodes, &[]),
        vec![RouteDefect::FloatingPin { node: 0 }, RouteDefect::FloatingPin { node: 1 }]
    );

    // ⚠️ A Steiner node is never floating, whatever its range.
    let nodes = vec![node(0, 0, 4, 2, None)];
    assert!(check_route_3d(&nodes, &[]).is_empty(), "a node with no pin cannot float");

    // A route that starts and ends somewhere other than its nodes.
    let nodes = vec![node(0, 0, 0, 4, None), node(5, 0, 0, 4, None)];
    let edges = vec![RoutedEdge {
        len: 5, routelen: 1, n1: 0, n2: 1,
        grids: vec![p(1, 1, 0), p(4, 0, 0)],
    }];
    let got = check_route_3d(&nodes, &edges);
    assert!(got.contains(&RouteDefect::StartsElsewhere { edge: 0 }));
    assert!(got.contains(&RouteDefect::EndsElsewhere { edge: 0 }));

    // A step that jumps two cells, and one that moves in two axes at once.
    let nodes = vec![node(0, 0, 0, 4, None), node(3, 1, 0, 4, None)];
    let edges = vec![RoutedEdge {
        len: 5, routelen: 2, n1: 0, n2: 1,
        grids: vec![p(0, 0, 0), p(2, 0, 0), p(3, 1, 0)],
    }];
    let got = check_route_3d(&nodes, &edges);
    assert!(got.contains(&RouteDefect::NotAPath { edge: 0, step: 0, distance: 2 }));
    assert!(got.contains(&RouteDefect::NotAPath { edge: 0, step: 1, distance: 2 }));

    // ⚠️ A step that changes layer only is a legal path step, not a defect.
    let edges = vec![RoutedEdge {
        len: 5, routelen: 1, n1: 0, n2: 0,
        grids: vec![p(0, 0, 0), p(0, 0, 1)],
    }];
    let nodes = vec![node(0, 0, 0, 4, None)];
    assert!(check_route_3d(&nodes, &edges).is_empty(), "a via is one step");

    // A negative layer, including on the LAST point, which the step loop never reaches.
    let edges = vec![RoutedEdge {
        len: 5, routelen: 1, n1: 0, n2: 0,
        grids: vec![p(0, 0, 0), p(0, 0, -1)],
    }];
    let got = check_route_3d(&nodes, &edges);
    assert!(
        got.contains(&RouteDefect::NegativeLayer { edge: 0, point: 1, layer: -1 }),
        "the layer check must include the final point"
    );
}

/// ⛔ The checker's edge gate is `len == 0`, so an edge with a **negative** length is checked
/// where the via count would skip it.
#[test]
fn the_checker_examines_an_edge_with_a_negative_length() {
    let nodes = vec![RoutedNode { x: 0, y: 0, bot_layer: 0, top_layer: 4, pin_layer: None }];
    let edges = vec![RoutedEdge {
        len: -1, routelen: 1, n1: 0, n2: 0,
        grids: vec![Point3D { x: 0, y: 0, layer: 0 }, Point3D { x: 0, y: 0, layer: -3 }],
    }];
    assert!(
        !check_route_3d(&nodes, &edges).is_empty(),
        "a negative length must still be checked"
    );
    assert_eq!(three_d_via(&edges), 0, "but must not be counted for vias");
}

// ─── Pin coverage ───────────────────────────────────────────────────────────────────────────

#[test]
fn the_pin_coverage_pass_adds_nothing_on_any_captured_net() {
    let all = load("pincoverage");
    assert!(all.len() >= 400, "corpus too thin: {}", all.len());
    let (mut pins, mut nets) = (0usize, 0usize);

    for r in &all {
        let terminals: Vec<RoutedNode> = r["terminals"].as_array().expect("t").iter()
            .map(|t| RoutedNode {
                x: t["x"].as_i64().expect("x") as i16,
                y: t["y"].as_i64().expect("y") as i16,
                bot_layer: t["botL"].as_i64().expect("b") as i16,
                top_layer: t["topL"].as_i64().expect("t") as i16,
                pin_layer: None,
            }).collect();
        let es: Vec<RoutedEdge> = r["edges"].as_array().expect("edges").iter()
            .map(edge).collect();
        let num_layers = r["num_layers"].as_i64().expect("nl") as i16;

        let added = ensure_pin_coverage(&terminals, &es, num_layers);
        let want = r["added"].as_array().expect("added").len();
        assert_eq!(
            added.len(), want,
            "{} net {} added {} stacks, the reference added {}",
            r["design"].as_str().expect("design"), r["net_id"], added.len(), want
        );
        pins += terminals.len();
        nets += 1;
    }
    assert!(pins >= 1_500, "too few pins: {pins}");
    assert!(nets >= 400);
}

/// ⛔ **No captured pin comes close to needing a stack** — 0 of 2,240, and no pin's position is
/// even missed by its net's routing. Layer assignment already covers every pin, so this pass is a
/// safety net that never catches anything.
///
/// ⚠️ Asserted as the limitation, with the constructed cases below covering the rules.
#[test]
fn no_captured_pin_is_left_uncovered() {
    let all = load("pincoverage");
    let (mut pins, mut untouched, mut would_add) = (0usize, 0usize, 0usize);

    for r in &all {
        let num_layers = r["num_layers"].as_i64().expect("nl") as i16;
        let mut range = std::collections::BTreeMap::new();
        let terms: Vec<(i16, i16, i16)> = r["terminals"].as_array().expect("t").iter()
            .map(|t| (t["x"].as_i64().expect("x") as i16,
                      t["y"].as_i64().expect("y") as i16,
                      t["botL"].as_i64().expect("b") as i16)).collect();
        for (x, y, _) in &terms {
            range.insert((*x, *y), (num_layers, -1i16));
        }
        for e in r["edges"].as_array().expect("edges") {
            let ed = edge(e);
            if !(ed.len > 0 || ed.routelen > 0) {
                continue;
            }
            for i in 0..=ed.routelen.max(0) as usize {
                let g = ed.grids[i];
                if let Some(v) = range.get_mut(&(g.x, g.y)) {
                    v.0 = v.0.min(g.layer);
                    v.1 = v.1.max(g.layer);
                }
            }
        }
        for (x, y, bot) in &terms {
            pins += 1;
            let (lo, hi) = range[&(*x, *y)];
            if lo == num_layers && hi == -1 {
                untouched += 1;
            }
            if *bot < lo || *bot > hi {
                would_add += 1;
            }
        }
    }
    assert!(pins >= 1_500, "too few pins to say anything: {pins}");
    assert_eq!(untouched, 0, "{untouched} pins now sit where no edge goes");
    assert_eq!(
        would_add, 0,
        "{would_add} pins now need a via stack — the pass fires on a real design and should be \
         pinned by a captured case rather than a constructed one"
    );
}

/// ⛔ A pin whose position no edge visits keeps the **inverted** starting range, so it is judged
/// un-covered and gets a stack running to the layer count.
#[test]
fn a_pin_no_edge_reaches_gets_a_stack_to_the_layer_count() {
    let pin = RoutedNode { x: 4, y: 4, bot_layer: 1, top_layer: 1, pin_layer: None };
    let added = ensure_pin_coverage(&[pin], &[], 8);
    assert_eq!(added.len(), 1, "an unreached pin must get a stack");
    assert_eq!(added[0].routelen, 7, "from its own layer up to the layer count");
    assert_eq!(added[0].grids.first().expect("first").layer, 1);
    assert_eq!(added[0].grids.last().expect("last").layer, 8);
}

/// ⛔ A pin **above** the covered range gets a stack downwards, from the range's top to the pin.
#[test]
fn a_pin_above_the_covered_range_gets_a_downward_stack() {
    let pin = RoutedNode { x: 0, y: 0, bot_layer: 5, top_layer: 5, pin_layer: None };
    let edges = vec![RoutedEdge {
        len: 1, routelen: 1, n1: 0, n2: 0,
        grids: vec![Point3D { x: 0, y: 0, layer: 1 }, Point3D { x: 1, y: 0, layer: 2 }],
    }];
    let added = ensure_pin_coverage(&[pin], &edges, 8);
    assert_eq!(added.len(), 1);
    assert_eq!(added[0].routelen, 4, "from the range's top, 1, up to the pin at 5");
    assert_eq!(added[0].grids.first().expect("f").layer, 1);
    assert_eq!(added[0].grids.last().expect("l").layer, 5);
}

/// ⛔ A pin already inside the covered range gets nothing, and only the pin's BOTTOM layer is
/// tested — a top layer outside the range is not a reason to add anything.
#[test]
fn a_covered_pin_gets_nothing_and_only_its_bottom_layer_is_tested() {
    let edges = vec![RoutedEdge {
        len: 1, routelen: 2, n1: 0, n2: 0,
        grids: vec![
            Point3D { x: 0, y: 0, layer: 1 },
            Point3D { x: 0, y: 0, layer: 2 },
            Point3D { x: 0, y: 0, layer: 3 },
        ],
    }];
    // Bottom inside the range, top far outside it.
    let pin = RoutedNode { x: 0, y: 0, bot_layer: 2, top_layer: 7, pin_layer: None };
    assert!(
        ensure_pin_coverage(&[pin], &edges, 8).is_empty(),
        "only the bottom layer decides"
    );
}

/// ⛔ The pass's edge gate admits an edge with a positive length **or** real steps, so an edge
/// with no length still contributes its points to the covered range — where the via count would
/// ignore it entirely.
#[test]
fn the_coverage_gate_admits_an_edge_the_via_count_ignores() {
    let edges = vec![RoutedEdge {
        len: 0, routelen: 1, n1: 0, n2: 0,
        grids: vec![Point3D { x: 0, y: 0, layer: 3 }, Point3D { x: 0, y: 0, layer: 4 }],
    }];
    let pin = RoutedNode { x: 0, y: 0, bot_layer: 3, top_layer: 3, pin_layer: None };
    assert!(
        ensure_pin_coverage(&[pin], &edges, 8).is_empty(),
        "the zero-length edge must still count towards coverage"
    );
    assert_eq!(three_d_via(&edges), 0, "though it contributes no vias");
}

/// ⛔ A step that **moves and changes layer at once** is not a path step, and only the layer term
/// catches it.
///
/// ⚠️ Added because dropping that term survived everything else: every captured layer change is a
/// pure via, where the position terms are zero, and every captured move keeps its layer. No
/// captured step does both, so the term was carried by nothing.
#[test]
fn a_step_that_moves_and_changes_layer_is_reported() {
    let nodes = vec![
        RoutedNode { x: 0, y: 0, bot_layer: 0, top_layer: 4, pin_layer: None },
        RoutedNode { x: 1, y: 0, bot_layer: 0, top_layer: 4, pin_layer: None },
    ];
    let edges = vec![RoutedEdge {
        len: 5, routelen: 1, n1: 0, n2: 1,
        grids: vec![Point3D { x: 0, y: 0, layer: 2 }, Point3D { x: 1, y: 0, layer: 3 }],
    }];
    assert_eq!(
        check_route_3d(&nodes, &edges),
        vec![RouteDefect::NotAPath { edge: 0, step: 0, distance: 2 }],
        "a diagonal in space and layer must be caught by the layer term"
    );
}

/// ⚠️ **One mutation survives this file and is equivalent, not untested.** Relaxing the overflow
/// test from "above capacity" to "at or above" changes nothing arithmetically: a cell exactly at
/// capacity contributes an overflow of zero, which adds nothing to either running total and
/// cannot raise either maximum, both of which start at zero.
///
/// ⛔ It is not equivalent in the reference, where the same branch also emits a congestion report
/// per cell — so relaxing it there would print a line for every cell that merely reaches
/// capacity. We do not model the logger, so no test here can distinguish them.
#[test]
fn a_cell_exactly_at_capacity_contributes_nothing_either_way() {
    let at_capacity = [Cell3D { horizontal: true, usage: 5, capacity: 5 }];
    let got = get_overflow_3d(&at_capacity);
    assert_eq!(got.horizontal, 0);
    assert_eq!(got.max_horizontal, 0);
    assert_eq!(got.total, 0);
    // ⚠️ But its usage still counts, which is the part that is NOT equivalent.
    assert_eq!(got.total_usage, 5);
}
