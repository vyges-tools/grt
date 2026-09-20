// SPDX-License-Identifier: Apache-2.0
//! R14 driver — how far the search may stray from the edge it is re-routing.
//!
//! 2,410 regions from four designs across **14 branch buckets**: the allowance capped by the
//! caller's limit or by the edge's own route length, the net critical or not, and the region
//! clamped at either grid edge or neither.
//!
//! ⚠️ The two intermediates are carried as well as the result, so a mismatch attributes to the
//! allowance or to the clamping rather than to "the region".

use serde_json::Value;
use vyges_grt::maze_edge_region;

struct Region {
    design: String,
    n1: (i32, i32),
    n2: (i32, i32),
    expand: i32,
    iter: i32,
    routelen: i32,
    is_critical: bool,
    grid: (i32, i32),
    enlarge: i32,
    decrease: i32,
    want: (i32, i32, i32, i32),
}

fn regions() -> Vec<Region> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/region.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["regions"].as_array().expect("regions").iter().map(|r| {
        let i = |k: &str| r[k].as_i64().unwrap_or_else(|| panic!("{k}")) as i32;
        Region {
            design: r["design"].as_str().expect("design").to_string(),
            n1: (i("n1x"), i("n1y")),
            n2: (i("n2x"), i("n2y")),
            expand: i("expand"),
            iter: i("iter"),
            routelen: i("routelen"),
            is_critical: r["is_critical"].as_bool().expect("crit"),
            grid: (i("x_grid"), i("y_grid")),
            enlarge: i("enlarge"),
            decrease: i("decrease"),
            want: (i("x1"), i("x2"), i("y1"), i("y2")),
        }
    }).collect()
}

#[test]
fn search_regions_match_the_reference() {
    let rows = regions();
    assert!(rows.len() >= 2000, "corpus too thin: {}", rows.len());
    for r in &rows {
        let got = maze_edge_region(
            r.n1, r.n2, r.expand, r.iter, r.routelen, r.is_critical, r.grid,
        );
        assert_eq!(
            got, r.want,
            "region on {} for edge {:?}-{:?} at iteration {}",
            r.design, r.n1, r.n2, r.iter
        );
    }
}

/// The allowance itself, separately from the clamping.
///
/// ⛔ **It is capped by the edge's CURRENT route length**, not by the distance between its
/// endpoints — an edge already routed the long way round gets a wider search than a short one
/// between the same points.
#[test]
fn the_allowance_matches_the_reference() {
    let rows = regions();
    let (mut by_expand, mut by_routelen) = (0usize, 0usize);
    for r in &rows {
        let derived = (r.iter / 6 + 3) * r.routelen;
        let enlarge = r.expand.min(derived);
        assert_eq!(enlarge, r.enlarge, "allowance on {} at iteration {}", r.design, r.iter);
        if r.expand <= derived { by_expand += 1 } else { by_routelen += 1 }
    }
    // ⚠️ Both caps must bind somewhere, or the minimum is decided by one side only.
    assert!(by_expand >= 200, "the caller's limit never binds: {by_expand}");
    assert!(by_routelen >= 200, "the route length never binds: {by_routelen}");
}

/// ⛔ **The critical-net shrink is reached but never bites.**
///
/// A net marked critical narrows its own search region — but only from the seventh iteration
/// onward, because the shrink is `(iter / 7) * 5`. Measured across the corpus: critical nets
/// appear **only at iterations 3, 4 and 5**, so the shrink is **zero on all 302 of them**.
///
/// ⟹ The branch is live — it is entered 302 times — and its effect never is. Those are different
/// claims, and the corpus can only support the first.
#[test]
fn the_critical_shrink_is_reached_but_always_zero() {
    let rows = regions();
    let critical: Vec<&Region> = rows.iter().filter(|r| r.is_critical).collect();
    assert!(
        critical.len() >= 50,
        "too few critical nets to say anything: {}", critical.len()
    );
    for r in &critical {
        assert_eq!(
            r.decrease, ((r.iter / 7) * 5).min(r.enlarge / 2),
            "shrink on {} at iteration {}", r.design, r.iter
        );
        assert_eq!(
            r.decrease, 0,
            "a captured critical net finally shrank, at iteration {} — the corpus has moved",
            r.iter
        );
        assert!(r.iter < 7, "a critical net appeared at iteration {}", r.iter);
    }
    // A non-critical net never shrinks either.
    for r in rows.iter().filter(|r| !r.is_critical) {
        assert_eq!(r.decrease, 0, "a non-critical net must not shrink, on {}", r.design);
    }
}

/// What the shrink does when it does bite — constructed, since no design reaches it.
///
/// ⚠️ It is applied **inwards on every side** and capped at half the allowance, so a net under
/// timing pressure is kept close to its existing path and the region can never invert.
#[test]
fn a_critical_net_narrows_its_region_from_the_seventh_iteration() {
    let plain = maze_edge_region((40, 40), (60, 40), 9999, 7, 4, false, (999, 999));
    let crit = maze_edge_region((40, 40), (60, 40), 9999, 7, 4, true, (999, 999));
    // Allowance is (7/6 + 3) * 4 = 16; shrink is min((7/7)*5, 8) = 5.
    assert_eq!(plain, (24, 76, 24, 56));
    assert_eq!(crit, (29, 71, 29, 51), "narrower on every side, by the shrink");

    // ⛔ Capped at half the allowance, so the region cannot invert however late the iteration.
    let late = maze_edge_region((40, 40), (60, 40), 9999, 700, 1, true, (999, 999));
    assert!(late.0 <= late.1 && late.2 <= late.3, "the region inverted: {late:?}");
    let enlarge = 9999i32.min((700 / 6 + 3) * 1);
    // ⚠️ and still clamped to the grid afterwards, which is what keeps it non-negative.
    assert_eq!(late.0, (40 - enlarge + enlarge / 2).max(0), "capped at half, then clamped");
}

/// ⚠️ The two divisions step on different cadences, and both are integer divisions.
#[test]
fn the_two_cadences_step_at_different_iterations() {
    // The allowance steps every sixth iteration.
    let at = |iter: i32| maze_edge_region((5, 5), (5, 5), 9999, iter, 1, false, (999, 999));
    assert_eq!(at(0), at(5), "iterations 0 and 5 give the same allowance");
    assert_ne!(at(5), at(6), "but 6 steps it");

    // ⚠️ The shrink steps every seventh. Iterations 6 and 7 share an allowance step
    // (`iter / 6 == 1` for both), so any difference between them is the shrink alone.
    let crit = |iter: i32| maze_edge_region((40, 40), (60, 40), 9999, iter, 4, true, (999, 999));
    let plain = |iter: i32| maze_edge_region((40, 40), (60, 40), 9999, iter, 4, false, (999, 999));
    assert_eq!(crit(6), plain(6), "before the seventh iteration a critical net is no different");
    assert_ne!(crit(7), plain(7), "from the seventh it is narrower");
    assert_eq!(plain(6), plain(7), "and the allowance itself has not moved between the two");
}

/// ⛔ An edge exactly **at** the threshold is skipped, not routed.
///
/// ⚠️ **Constructed, and it has to be**: the region trace only fires for edges that already
/// passed this gate, so no captured record sits on the boundary. The comparison is strict, so the
/// threshold names the longest edge the router will leave alone.
#[test]
fn an_edge_exactly_at_the_threshold_is_skipped() {
    use vyges_grt::maze_edge_is_long_enough;

    // A span of 10, against thresholds either side of it.
    assert_eq!(maze_edge_is_long_enough((3, 3), (13, 3), 9), Some(10), "over the threshold");
    assert_eq!(maze_edge_is_long_enough((3, 3), (13, 3), 10), None, "exactly at it is skipped");
    assert_eq!(maze_edge_is_long_enough((3, 3), (13, 3), 11), None, "and under it");

    // ⚠️ The length is the Manhattan distance recomputed from the endpoints, so a bend counts.
    assert_eq!(maze_edge_is_long_enough((3, 3), (9, 7), 9), Some(10));
    // A degenerate edge is never long enough, whatever the threshold.
    assert_eq!(maze_edge_is_long_enough((3, 3), (3, 3), 0), None);
}
