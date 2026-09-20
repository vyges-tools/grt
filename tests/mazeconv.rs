// SPDX-License-Identifier: Apache-2.0
//! R11 — expanding each edge's symbolic route into explicit grid points.
//!
//! 3,903 records from four designs, checked **point by point** rather than by length. Every one
//! of the eight (shape, turn, y-ordering) combinations is present, and the test asserts that:
//! each shape branches on the y-ordering, so a corpus missing one arm would validate it against
//! nothing.
//!
//! ⛔ **The point buffer is sized from the length the edge carried on entry**, while the number
//! of points is fixed by the geometry. Every shape writes exactly `manhattan + 1` points, so the
//! two agree only while the stored length already equals the Manhattan distance. That is checked
//! here as a property of the captured data rather than assumed — it is what makes the reference's
//! `resize` safe.

use serde_json::Value;
use vyges_grt::estimate::LShape;
use vyges_grt::{convert_to_mazeroute, RoutePt, SymbolicRoute};

struct Case {
    design: String,
    x1: i32, y1: i32, x2: i32, y2: i32,
    len_on_entry: i32,
    shape: SymbolicRoute,
    bucket: String,
    points: Vec<RoutePt>,
    len: i32,
    routelen: i32,
    buf_size: usize,
}

fn cases() -> Vec<Case> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/mazeconv.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["calls"].as_array().expect("calls").iter().map(|c| {
        let i = |k: &str| c[k].as_i64().unwrap_or_else(|| panic!("{k}")) as i32;
        let (y1, y2) = (i("y1"), i("y2"));
        let kind = c["shape"].as_str().expect("shape");
        let x_first = c["x_first"].as_bool().expect("x_first");
        let hvh = c["hvh"].as_bool().expect("hvh");
        let shape = match kind {
            "none" => SymbolicRoute::NoRoute,
            "l" => SymbolicRoute::L(if x_first { LShape::XFirst } else { LShape::YFirst }),
            "z" => SymbolicRoute::Z { hvh, z_point: i("z_point") },
            other => panic!("unknown shape {other}"),
        };
        let turn = match kind {
            "none" => String::new(),
            "l" => format!("/{}", if x_first { "xfirst" } else { "yfirst" }),
            _ => format!("/{}", if hvh { "hvh" } else { "vhv" }),
        };
        let order = if kind == "none" {
            String::new()
        } else if y1 <= y2 {
            "/y1<=y2".to_string()
        } else {
            "/y1>y2".to_string()
        };
        Case {
            design: c["design"].as_str().expect("design").to_string(),
            x1: i("x1"), y1, x2: i("x2"), y2,
            len_on_entry: i("len_on_entry"),
            shape,
            bucket: format!("{kind}{turn}{order}"),
            points: c["points"].as_array().expect("points").iter()
                .map(|p| RoutePt {
                    x: p[0].as_i64().expect("x") as i32,
                    y: p[1].as_i64().expect("y") as i32,
                    layer: 0,
                }).collect(),
            len: i("len"),
            routelen: i("routelen"),
            buf_size: c["buf_size"].as_u64().expect("buf_size") as usize,
        }
    }).collect()
}

#[test]
fn expansions_match_the_reference_point_by_point() {
    let cases = cases();
    assert!(cases.len() >= 3000, "corpus too thin: {}", cases.len());
    for c in &cases {
        let got = convert_to_mazeroute((c.x1, c.y1), (c.x2, c.y2), c.len_on_entry, c.shape);
        assert_eq!(
            got.grids, c.points,
            "points differ on {} {} edge ({},{})-({},{})",
            c.design, c.bucket, c.x1, c.y1, c.x2, c.y2
        );
        assert_eq!(got.len, c.len, "len on {} {}", c.design, c.bucket);
        assert_eq!(got.routelen, c.routelen, "routelen on {} {}", c.design, c.bucket);
    }
}

/// ⚠️ Every shape branches on the y-ordering and the corpus must reach both sides of each, or
/// half the expansion logic is decided by nothing.
#[test]
fn every_shape_and_ordering_is_represented() {
    let cases = cases();
    let mut seen: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
    for c in &cases {
        *seen.entry(c.bucket.as_str()).or_default() += 1;
    }
    for want in [
        "none",
        "l/xfirst/y1<=y2", "l/xfirst/y1>y2", "l/yfirst/y1<=y2", "l/yfirst/y1>y2",
        "z/hvh/y1<=y2", "z/hvh/y1>y2", "z/vhv/y1<=y2", "z/vhv/y1>y2",
    ] {
        let n = seen.get(want).copied().unwrap_or(0);
        assert!(n >= 50, "bucket {want} has only {n} records");
    }
}

/// The reference sizes the buffer from the entry length and fills it from the geometry.
///
/// ⛔ Those are different quantities, and they agree only because the stored length is already
/// the Manhattan distance. Asserted over the whole corpus, because if it ever failed the
/// reference would be writing past the end of its own buffer or leaving stale points behind.
#[test]
fn the_buffer_size_always_equals_the_points_written() {
    for c in &cases() {
        let manhattan = (c.x1 - c.x2).abs() + (c.y1 - c.y2).abs();
        assert_eq!(
            c.len_on_entry, manhattan,
            "entry length is not the Manhattan distance on {} {}", c.design, c.bucket
        );
        assert_eq!(
            c.buf_size, c.points.len(),
            "buffer and point count disagree on {} {}", c.design, c.bucket
        );
    }
}

/// ⚠️ `routelen` is the length the edge had on ENTRY, not the number of steps written, and for a
/// degenerate edge the reference writes zero and then overwrites it. Worth pinning separately:
/// the two agree everywhere except where they do not.
#[test]
fn a_degenerate_edge_keeps_one_point_and_zero_length() {
    let got = convert_to_mazeroute((7, 9), (7, 9), 0, SymbolicRoute::NoRoute);
    assert_eq!(got.grids, vec![RoutePt { x: 7, y: 9, layer: 0 }]);
    assert_eq!(got.len, 0);
    assert_eq!(got.routelen, 0, "the zero written inside is overwritten by the entry length");
}

// ---------------------------------------------------------------------------
// The two calls the stage makes after every edge has been expanded.
// ---------------------------------------------------------------------------

use vyges_grt::estimate::EstimateGrid;
use vyges_grt::{check_2d_edges_usage, UsageViolation};

/// Folding the estimate into the committed demand **adds** and leaves the estimate in place.
///
/// ⚠️ Worth pinning: moving it instead would look identical on a first fold and diverge on the
/// second, and the router folds more than once.
#[test]
fn folding_the_estimate_adds_and_does_not_move_it() {
    let mut grid = EstimateGrid::new(6, 6);
    grid.update_h(1, 3, 2, 4.0);
    grid.update_v(2, 1, 4, 7.0);

    assert_eq!(grid.usage_h(1, 2), 0.0, "committed demand starts empty");
    grid.add_est_usage_to_usage();
    assert_eq!(grid.usage_h(1, 2), 4.0);
    assert_eq!(grid.usage_v(2, 1), 7.0);
    assert_eq!(grid.h(1, 2), 4.0, "the estimate is left in place, not moved");

    // A second fold adds the same estimate again, which is exactly what "add" means here.
    grid.add_est_usage_to_usage();
    assert_eq!(grid.usage_h(1, 2), 8.0);
    assert_eq!(grid.h(1, 2), 4.0);
}

/// The runaway-usage check fires strictly above a whole multiple of the capacity.
///
/// ⛔ **Constructed, because no shipped design triggers it.** It is an error path — the reference
/// aborts on the first offending edge — so the only way to pin the boundary is to build one.
#[test]
fn the_usage_check_fires_only_strictly_above_the_limit() {
    let mut grid = EstimateGrid::new(5, 5);
    // 100x a capacity of 2 is 200: exactly on the limit must pass.
    grid.update_h(1, 2, 3, 200.0);
    grid.add_est_usage_to_usage();
    assert_eq!(check_2d_edges_usage(&grid, 2, 2), vec![], "exactly on the limit passes");

    grid.update_h(1, 2, 3, 0.5);
    grid.add_est_usage_to_usage();
    let found = check_2d_edges_usage(&grid, 2, 2);
    assert_eq!(
        found,
        vec![UsageViolation { x: 1, y: 3, horizontal: true, usage: 400.5, limit: 200 }],
        "a hair over the limit is reported"
    );

    // ⚠️ The two directions are checked against their own capacities, not a shared one.
    let mut v = EstimateGrid::new(5, 5);
    v.update_v(2, 1, 2, 300.0);
    v.add_est_usage_to_usage();
    assert_eq!(check_2d_edges_usage(&v, 9999, 3), vec![], "300 sits exactly on 100x3");
    assert_eq!(check_2d_edges_usage(&v, 9999, 2).len(), 1, "but is over 100x2");
    // and a horizontal capacity of 9999 must not excuse the vertical edge, nor vice versa
    assert!(
        check_2d_edges_usage(&v, 1, 9999).iter().all(|u| u.horizontal),
        "a vertical edge must not be judged against the horizontal capacity"
    );
}
