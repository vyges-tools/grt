// SPDX-License-Identifier: Apache-2.0
//! R15 — cutting loops out of routed paths.
//!
//! A maze route can revisit a cell: the search is over a grid, not a tree, and nothing in it
//! forbids a path that doubles back. This stage finds a repeated point, removes everything between
//! the two visits, gives back exactly the demand that stretch was charged, and **restarts the
//! scan**.
//!
//! ⛔ **No captured path contains a loop.** 750 edges kept from 17,542 scanned, and not one has a
//! repeated point. The corpus therefore decides only that the scan finds nothing on a loop-free
//! path and leaves it untouched — which is worth having, and is not the same as validating the
//! removal.
//!
//! ⟹ The removal, the give-back and the restart are pinned by **constructed** cases, and the
//! absence is asserted so a recapture that finds a loop fails loudly.

use serde_json::Value;
use vyges_grt::estimate::EstimateGrid;
use vyges_grt::remove_loops;

struct Edge {
    design: String,
    edge_cost: i8,
    before: Vec<(i32, i32)>,
    after: Vec<(i32, i32)>,
    routelen_before: usize,
    routelen_after: usize,
    loops: usize,
}

fn edges() -> Vec<Edge> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/removeloops.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let pts = |v: &Value| -> Vec<(i32, i32)> {
        v.as_array().expect("points").iter()
            .map(|p| (p[0].as_i64().expect("x") as i32, p[1].as_i64().expect("y") as i32))
            .collect()
    };
    v["edges"].as_array().expect("edges").iter().map(|e| Edge {
        design: e["design"].as_str().expect("design").to_string(),
        edge_cost: e["edge_cost"].as_i64().expect("ec") as i8,
        before: pts(&e["before"]),
        after: pts(&e["after"]),
        routelen_before: e["routelen_before"].as_i64().expect("rlb").max(0) as usize,
        routelen_after: e["routelen_after"].as_i64().expect("rla").max(0) as usize,
        loops: e["loops"].as_u64().expect("loops") as usize,
    }).collect()
}

#[test]
fn loop_free_paths_are_left_alone() {
    let all = edges();
    assert!(all.len() >= 500, "corpus too thin: {}", all.len());
    for e in &all {
        let mut grid = EstimateGrid::new(4, 4);
        let before = grid.clone();
        let mut grids = e.before.clone();
        let mut routelen = e.routelen_before;
        let removed = remove_loops(&mut grid, &mut grids, &mut routelen, e.edge_cost);

        assert_eq!(removed, e.loops, "loops removed on {}", e.design);
        assert_eq!(routelen, e.routelen_after, "length on {}", e.design);
        assert_eq!(&grids[..=routelen], &e.after[..], "path on {}", e.design);
        // ⛔ A scan that finds nothing must also charge nothing back.
        assert_eq!(grid, before, "demand was touched on a loop-free path, on {}", e.design);
    }
}

/// ⛔ Asserted as the absence it is, so a recapture containing a loop fails loudly.
#[test]
fn no_captured_path_contains_a_loop() {
    for e in &edges() {
        let mut seen = std::collections::HashSet::new();
        for p in &e.before[..=e.routelen_before] {
            assert!(
                seen.insert(*p),
                "{} now has a path revisiting {p:?} — loop removal is reachable after all, and \
                 the notes on it need revisiting", e.design
            );
        }
        assert_eq!(e.loops, 0);
    }
}

/// A loop is cut out, and exactly its demand is given back.
///
/// ⛔ **Constructed: no captured path loops.** The path below leaves a cell, wanders, and comes
/// back to it — so the stretch between the two visits is charged and must be un-charged.
#[test]
fn a_loop_is_cut_and_its_demand_given_back() {
    let mut grid = EstimateGrid::new(8, 8);
    // (1,1) -> (2,1) -> (2,2) -> (1,2) -> (1,1) -> (1,0): a square, then onward.
    let mut grids = vec![(1, 1), (2, 1), (2, 2), (1, 2), (1, 1), (1, 0)];
    let mut routelen = 5usize;
    // Charge the whole path first, as the router would have.
    vyges_grt::charge_route(&mut grid, &grids, 1);
    let charged = grid.clone();

    let removed = remove_loops(&mut grid, &mut grids, &mut routelen, 1);

    assert_eq!(removed, 1, "one loop");
    assert_eq!(routelen, 1, "four steps of the square are gone");
    assert_eq!(&grids[..=routelen], &[(1, 1), (1, 0)], "only the straight remainder is left");

    // ⛔ Exactly the square's four edges are given back — the final step is not.
    assert_ne!(grid, charged);
    assert_eq!(grid.usage_v(1, 0), 1.0, "the step beyond the loop keeps its demand");
    assert_eq!(grid.usage_h(1, 1), 0.0, "and the loop's own edges are back to nothing");
    assert_eq!(grid.usage_v(2, 1), 0.0);
    assert_eq!(grid.usage_h(1, 2), 0.0);
    // ⛔ Including the step that closes the loop — the last one, which is easy to leave charged.
    assert_eq!(grid.usage_v(1, 1), 0.0, "the closing step must be given back too");
}

/// ⚠️ A zero-length step charges nothing, so nothing is given back for it either.
#[test]
fn a_repeated_point_with_no_movement_gives_nothing_back() {
    let mut grid = EstimateGrid::new(8, 8);
    // The same cell twice in a row: a loop of length one that crosses no edge.
    let mut grids = vec![(3, 3), (3, 3), (4, 3)];
    let mut routelen = 2usize;
    let before = grid.clone();

    let removed = remove_loops(&mut grid, &mut grids, &mut routelen, 1);

    assert_eq!(removed, 1);
    assert_eq!(routelen, 1);
    assert_eq!(&grids[..=routelen], &[(3, 3), (4, 3)]);
    assert_eq!(grid, before, "a step that crossed no edge gives nothing back");
}

/// ⛔ The scan restarts from the beginning, so a path with several loops is fully cleaned.
#[test]
fn several_loops_are_all_removed() {
    let mut grid = EstimateGrid::new(12, 12);
    // Two separate square detours along one path.
    let mut grids = vec![
        (1, 1), (2, 1), (2, 2), (1, 2), (1, 1),
        (1, 0), (2, 0), (2, 1), (1, 1), (1, 0),
    ];
    let mut routelen = 9usize;
    vyges_grt::charge_route(&mut grid, &grids, 1);

    let removed = remove_loops(&mut grid, &mut grids, &mut routelen, 1);

    assert!(removed >= 2, "both loops must be found, got {removed}");
    // ⛔ The exact remainder, not merely "no duplicates" — a scan that continued instead of
    // restarting can still end duplicate-free while having cut a different pair.
    assert_eq!(&grids[..=routelen], &[(1, 1), (1, 0)], "the straight remainder");
    assert_eq!(routelen, 1);
}

/// ⛔ Continuing the scan instead of restarting misses a loop that compaction moved **below** the
/// index.
///
/// Removing a stretch shifts everything after it down. A duplicate pair that sat beyond the index
/// can land entirely before it, and a scan that carries on from where it was never looks there
/// again.
///
/// ⚠️ **Constructed, and it had to be built deliberately**: the obvious two-loop path is cleaned
/// correctly either way, because its second loop happens to stay above the index.
#[test]
fn restarting_the_scan_catches_a_loop_that_compaction_moved_down() {
    let mut grid = EstimateGrid::new(8, 8);
    // A square at the start, then a second detour far enough along that removing the first
    // drags it below where a continuing scan would resume.
    let mut grids = vec![
        (1, 1), (2, 1), (2, 2), (1, 2), (1, 1),
        (0, 1), (0, 0), (1, 0), (0, 0),
    ];
    let mut routelen = 8usize;
    vyges_grt::charge_route(&mut grid, &grids, 1);

    let removed = remove_loops(&mut grid, &mut grids, &mut routelen, 1);

    assert_eq!(removed, 2, "both loops, which needs the restart");
    assert_eq!(
        &grids[..=routelen], &[(1, 1), (0, 1), (0, 0)],
        "a scan that carried on would leave (0,0) twice"
    );
    let kept = &grids[..=routelen];
    let mut seen = std::collections::HashSet::new();
    for p in kept {
        assert!(seen.insert(*p), "a loop survived: {p:?} appears twice");
    }
}

/// 🔑 At most one earlier point can match, so first-versus-last cannot be observed.
///
/// The scan restarts after every removal, so when it reaches an index no two earlier points are
/// equal — if they were, it would have stopped at the second of them. Two matches for the same
/// index would mean exactly that. So taking the last match rather than the first is a mutation
/// nothing can kill, and that is a property of the algorithm rather than of the corpus.
#[test]
fn at_most_one_earlier_point_can_match() {
    // Three visits to one cell: the scan stops at the second, never seeing the third.
    let mut grid = EstimateGrid::new(8, 8);
    let mut grids = vec![(2, 2), (3, 2), (2, 2), (3, 2), (2, 2)];
    let mut routelen = 4usize;
    let removed = remove_loops(&mut grid, &mut grids, &mut routelen, 1);
    assert!(removed >= 1);
    assert_eq!(&grids[..=routelen], &[(2, 2)], "everything collapses onto the first visit");

    // And on any loop-free prefix, no index has two earlier matches — which is what makes the
    // distinction unobservable.
    let path = [(0, 0), (1, 0), (1, 1), (0, 1)];
    for i in 1..path.len() {
        let matches = path[..i].iter().filter(|p| **p == path[i]).count();
        assert!(matches <= 1, "index {i} has {matches} earlier matches");
    }
}
