// SPDX-License-Identifier: Apache-2.0
//! R10 — `newrouteZ_edge`, the two-terminal Z re-route.
//!
//! Golden `zedge.json`, both cost modes: per call past the early returns, the grid patch the cost
//! loops read (after the rip-up), the chosen column, and the same patch after the commit. `gate`
//! counts every call and which early return it took.

use std::collections::HashMap;

use serde_json::Value;
use vyges_grt::estimate::EstimateGrid;
use vyges_grt::ripup_route::RoutedShape;
use vyges_grt::{newroute_z_edge, route_z_edge_after_ripup};

fn golden() -> Value {
    read(&format!("{}/examples/grt_gate/zedge.json", env!("CARGO_MANIFEST_DIR")))
}

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn f(v: &Value) -> f64 {
    f64::from_bits(v.as_u64().expect("bits"))
}

/// A captured patch: per (dir, x, y), (estimated usage, blockage).
type Patch = HashMap<(char, i32, i32), (f64, u16)>;

fn patch(v: &Value) -> Patch {
    v.as_array().expect("patch").iter().map(|c| {
        let dir = c[0].as_str().expect("dir").chars().next().expect("dir");
        let (est, red) = (f(&c[3]), f(&c[4]));
        let blockage = red - est;
        assert_eq!(blockage, blockage.trunc(), "blockage is integral");
        ((dir, int(&c[1]) as i32, int(&c[2]) as i32), (est, blockage as u16))
    }).collect()
}

/// Replay each record; returns the number replayed.
fn replay(g: &Value) -> usize {
    let records = g["records"].as_array().expect("records");
    for r in records {
        let (x1, y1, x2, y2) =
            (int(&r["x1"]) as i32, int(&r["y1"]) as i32, int(&r["x2"]) as i32, int(&r["y2"]) as i32);
        let before = patch(&r["before"]);
        let after = patch(&r["after"]);
        let span = (x2.max(x1).max(y1).max(y2) + 2) as usize;
        let mut grid = EstimateGrid::new(span, span);
        for (&(d, x, y), &(est, _)) in &before {
            if d == 'V' {
                grid.update_v(x, y, y + 1, est);
            } else {
                grid.update_h(x, x + 1, y, est);
            }
        }
        let red = |d: char| {
            let before = before.clone();
            move |x: usize, y: usize| before.get(&(d, x as i32, y as i32)).map_or(0, |c| c.1)
        };
        let (red_v, red_h) = (red('V'), red('H'));
        let z = route_z_edge_after_ripup(
            &mut grid,
            (x1, y1),
            (x2, y2),
            int(&r["ec"]) as i8,
            f(&r["vlb"]) as f32,
            f(&r["hlb"]) as f32,
            &red_v,
            &red_h,
        );
        let who = format!("{} ({x1},{y1})-({x2},{y2})", r["design"]);
        assert_eq!(z, int(&r["z"]) as i32, "{who}: column");
        for (&(d, x, y), &(est, _)) in &after {
            let got = if d == 'V' { grid.v(x as usize, y as usize) } else { grid.h(x as usize, y as usize) };
            assert_eq!(got.to_bits(), est.to_bits(), "{who}: {d} ({x},{y}) after the commit");
        }
    }
    records.len()
}

/// Every captured decision: the column, and every cell of the patch after the commit.
#[test]
fn z_edge_decisions_match_the_reference() {
    let n = replay(&golden());
    assert!(n >= 50, "too few decisions: {n}");
}

/// GRT_ZEDGE_FULL=/path/to/zedge-all.json cargo test --test zedge -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn z_edge_decisions_match_the_reference_exhaustively() {
    let path = std::env::var("GRT_ZEDGE_FULL").expect("set GRT_ZEDGE_FULL");
    eprintln!("exhaustive: {} decisions", replay(&read(&path)));
}

/// ⚠️ Both early returns repeat checks the live caller already made — measured: neither fires.
#[test]
fn neither_early_return_fires_on_the_live_path() {
    let g = golden();
    let gate = &g["gate"];
    let calls = int(&gate["calls"]);
    assert!(calls >= 100, "too few calls: {calls}");
    assert_eq!(gate.get("len_le_0").map_or(0, int), 0);
    assert_eq!(gate.get("straight").map_or(0, int), 0);
    assert_eq!(int(&gate["routed"]), calls);
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn none(_: usize, _: usize) -> u16 {
    0
}

/// ⛔ The candidate columns INCLUDE x2 — a Z whose middle segment runs up the far end.
#[test]
fn the_far_column_is_a_candidate() {
    let mut grid = EstimateGrid::new(8, 8);
    // Load every vertical column except x2 = 3 far over capacity.
    for x in 0..3 {
        grid.update_v(x, 0, 1, 100.0);
    }
    let z = route_z_edge_after_ripup(&mut grid, (0, 0), (3, 1), 1, 1.0, 1.0, &none, &none);
    assert_eq!(z, 3);
}

/// ⛔ On an exact cost tie the test cost decides, and it is live here — unlike `newroute_z`.
/// Everything under capacity, so every column costs 0. Empty, column 0 wins (its test cost holds
/// the whole top row's slack); fill row y2 almost to capacity and that slack shrinks, and column
/// 1 wins instead.
#[test]
fn the_tie_break_is_live() {
    let mut empty = EstimateGrid::new(8, 8);
    assert_eq!(route_z_edge_after_ripup(&mut empty, (0, 0), (2, 1), 1, 4.0, 4.0, &none, &none), 0);
    let mut grid = EstimateGrid::new(8, 8);
    grid.update_h(0, 2, 1, 3.75);
    assert_eq!(route_z_edge_after_ripup(&mut grid, (0, 0), (2, 1), 1, 4.0, 4.0, &none, &none), 1);
}

/// ⛔ The boundary tie-break is NOT a running total. Row y2 at 3 of 4 gives each later column a
/// delta of -3; carried forward those would sum to -32 at column 8 and it would win. Not carried,
/// column 0 (holding row y2's whole -8) wins. Costs are all 0, so only the tie-break decides.
#[test]
fn the_boundary_tie_break_is_not_carried_forward() {
    let mut grid = EstimateGrid::new(12, 12);
    grid.update_h(0, 8, 1, 3.0);
    let z = route_z_edge_after_ripup(&mut grid, (0, 0), (8, 1), 1, 4.0, 4.0, &none, &none);
    assert_eq!(z, 0);
}

/// Early returns write nothing and rip nothing up.
#[test]
fn early_returns_leave_the_grid_alone() {
    let mut grid = EstimateGrid::new(8, 8);
    grid.update_h(0, 1, 0, 2.0);
    let prior = RoutedShape::L { x_first: true };
    assert_eq!(newroute_z_edge(&mut grid, 0, (0, 0), (3, 1), &prior, 1, 1.0, 1.0, &none, &none), None);
    assert_eq!(newroute_z_edge(&mut grid, 3, (0, 0), (3, 0), &prior, 1, 1.0, 1.0, &none, &none), None);
    assert_eq!(grid.h(0, 0), 2.0);
}

/// The commit: row y1 up to the column, the column itself, row y2 from the column — half-open.
#[test]
fn the_commit_charges_both_rows_and_the_column() {
    let mut grid = EstimateGrid::new(8, 8);
    grid.update_v(0, 0, 1, 50.0);
    grid.update_v(1, 0, 1, 50.0);
    let z = route_z_edge_after_ripup(&mut grid, (0, 0), (3, 2), 2, 1.0, 1.0, &none, &none);
    assert_eq!(z, 2);
    assert_eq!((grid.h(0, 0), grid.h(1, 0), grid.h(2, 0)), (2.0, 2.0, 0.0));
    assert_eq!((grid.v(2, 0), grid.v(2, 1)), (2.0, 2.0));
    assert_eq!((grid.h(1, 2), grid.h(2, 2)), (0.0, 2.0));
}
