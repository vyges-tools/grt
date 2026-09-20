// SPDX-License-Identifier: Apache-2.0
//! R14 piece 3 — seeding both search frontiers from the subtrees the edge separates.
//!
//! 1,059 setups from four designs, each carrying the net's whole tree and both frontiers **in
//! push order**.
//!
//! ⛔ **Push order is the behaviour.** Every seed is given a distance of zero, so the heap that
//! follows is entirely ties and pop order is decided by insertion alone. Comparing the frontiers
//! as sets would pass an implementation that searches in a different order.
//!
//! ⚠️ 309 of the setups have at least one point of the subtree **outside** the search region, so
//! the region test is decided by the corpus rather than assumed. That count is asserted.

use serde_json::Value;
use vyges_grt::{setup_heap, Heaps, MazeEdge, MazeNode};

struct Setup {
    design: String,
    num_terminals: usize,
    edge_id: usize,
    region: (i32, i32, i32, i32),
    nodes: Vec<MazeNode>,
    edges: Vec<MazeEdge>,
    src: Vec<(i32, i32)>,
    dest: Vec<(i32, i32)>,
}

fn pts(v: &Value) -> Vec<(i32, i32)> {
    v.as_array().expect("points").iter()
        .map(|p| (p[0].as_i64().expect("x") as i32, p[1].as_i64().expect("y") as i32))
        .collect()
}

fn setups() -> Vec<Setup> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/setupheap.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["setups"].as_array().expect("setups").iter().map(|s| {
        let r = s["region"].as_array().expect("region");
        let n = |i: usize| r[i].as_i64().expect("bound") as i32;
        Setup {
            design: s["design"].as_str().expect("design").to_string(),
            num_terminals: s["num_terminals"].as_u64().expect("terms") as usize,
            edge_id: s["edge_id"].as_u64().expect("edge") as usize,
            region: (n(0), n(1), n(2), n(3)),
            nodes: s["nodes"].as_array().expect("nodes").iter().map(|d| MazeNode {
                x: d["x"].as_i64().expect("x") as i32,
                y: d["y"].as_i64().expect("y") as i32,
                neighbours: d["neighbours"].as_array().expect("nbrs").iter()
                    .map(|p| (p[0].as_u64().expect("nbr") as usize,
                              p[1].as_u64().expect("edge") as usize))
                    .collect(),
                stack_alias: 0,
            }).collect(),
            edges: s["edges"].as_array().expect("edges").iter().map(|e| MazeEdge {
                n1: e["n1"].as_u64().expect("n1") as usize,
                n2: e["n2"].as_u64().expect("n2") as usize,
                routelen: e["routelen"].as_u64().expect("rl") as usize,
                grids: pts(&e["grids"]),
            }).collect(),
            src: pts(&s["src"]),
            dest: pts(&s["dest"]),
        }
    }).collect()
}

fn run(s: &Setup) -> Heaps {
    setup_heap(s.num_terminals, &s.nodes, &s.edges, s.edge_id, s.region)
}

#[test]
fn the_frontiers_match_the_reference_in_push_order() {
    let setups = setups();
    assert!(setups.len() >= 800, "corpus too thin: {}", setups.len());
    let (mut two_pin, mut multi) = (0usize, 0usize);

    for s in &setups {
        let got = run(s);
        assert_eq!(
            got.src, s.src,
            "source frontier on {} net edge {} ({} terminals)",
            s.design, s.edge_id, s.num_terminals
        );
        assert_eq!(got.dest, s.dest, "destination frontier on {}", s.design);
        if s.num_terminals == 2 { two_pin += 1 } else { multi += 1 }
    }
    assert!(two_pin >= 100, "too few two-pin nets: {two_pin}");
    assert!(multi >= 400, "too few multi-pin nets: {multi}");
}

/// ⛔ The two frontiers must never share a point, or the search would start already finished.
#[test]
fn the_two_frontiers_are_disjoint() {
    for s in &setups() {
        let got = run(s);
        let src: std::collections::HashSet<_> = got.src.iter().collect();
        for p in &got.dest {
            assert!(
                !src.contains(p),
                "point {p:?} is in both frontiers on {} edge {}", s.design, s.edge_id
            );
        }
    }
}

/// ⚠️ Points outside the search region are not seeded, and the corpus must contain some — or the
/// region test is decided by nothing.
#[test]
fn points_outside_the_region_are_not_seeded() {
    let setups = setups();
    let mut with_rejects = 0usize;
    for s in &setups {
        let (x1, x2, y1, y2) = s.region;
        let got = run(s);
        for (x, y) in got.src.iter().chain(got.dest.iter()) {
            assert!(
                *x >= x1 && *x <= x2 && *y >= y1 && *y <= y2,
                "seed ({x},{y}) is outside the region on {}", s.design
            );
        }
        // Did this setup actually have something to reject?
        let inside = |x: i32, y: i32| x >= x1 && x <= x2 && y >= y1 && y <= y2;
        let outside = s.nodes.iter().any(|n| !inside(n.x, n.y))
            || s.edges.iter().flat_map(|e| &e.grids).any(|(x, y)| !inside(*x, *y));
        with_rejects += usize::from(outside);
    }
    assert!(
        with_rejects >= 100,
        "only {with_rejects} setups have anything outside the region — the test is near-vacuous"
    );
}

/// A two-pin net seeds exactly its two endpoints, with no traversal at all.
#[test]
fn a_two_pin_net_seeds_only_its_endpoints() {
    let setups = setups();
    let mut checked = 0usize;
    for s in setups.iter().filter(|s| s.num_terminals == 2) {
        let got = run(s);
        let e = &s.edges[s.edge_id];
        assert_eq!(got.src, vec![(s.nodes[e.n1].x, s.nodes[e.n1].y)]);
        assert_eq!(got.dest, vec![(s.nodes[e.n2].x, s.nodes[e.n2].y)]);
        assert!(got.corr_edge.is_empty(), "no edge attribution without a traversal");
        checked += 1;
    }
    assert!(checked >= 100, "too few two-pin nets: {checked}");
}
