// SPDX-License-Identifier: Apache-2.0
//! R14 piece 6 — splicing the new path back into the net's tree.
//!
//! 1,500 surgeries from four designs. When a re-routed edge's endpoint ends up somewhere new, the
//! tree has to be rebuilt around it, and there are two shapes:
//!
//! - the node landed **on one of its own edges** — that edge is re-cut at the new position and
//!   its sibling takes over the remainder;
//! - the node landed **on a different edge** — its own two edges merge into one, and the edge it
//!   landed on splits in two.
//!
//! ⛔ In the second shape the three edge slots are **recycled, not created**: the merged edge is
//! written into the slot of the edge about to be split. That only works because every point list
//! is copied out first, and the test drives the real functions so that ordering is exercised.
//!
//! ⚠️ All four of the first shape's orientation combinations are present and asserted. What is
//! **not** present is a never-routed input edge: every captured input is a real route, so
//! `copy_grids`' single-point branch is pinned by a constructed case instead.

use serde_json::Value;
use vyges_grt::{copy_grids, update_route_type1, update_route_type2, MazeNode, SurgeryEdge};

struct Surgery {
    design: String,
    kind: u8,
    n1: usize,
    a1: usize,
    a2: usize,
    c1: usize,
    c2: usize,
    e1: (i32, i32),
    edge_n1a1: usize,
    edge_n1a2: usize,
    edge_c1c2: usize,
    nodes: Vec<MazeNode>,
    before: Vec<Option<SurgeryEdge>>,
    after: Vec<Option<SurgeryEdge>>,
}

fn edge_from(v: &Value) -> SurgeryEdge {
    SurgeryEdge {
        n1: v["n1"].as_u64().expect("n1") as usize,
        n2: v["n2"].as_u64().expect("n2") as usize,
        n1a: 0,
        n2a: 0,
        routelen: v["routelen"].as_i64().expect("rl").max(0) as usize,
        len: v["len"].as_i64().expect("len") as i32,
        is_maze_route: v["is_maze_route"].as_bool().expect("type"),
        grids: v["grids"].as_array().expect("grids").iter()
            .map(|p| (p[0].as_i64().expect("x") as i32, p[1].as_i64().expect("y") as i32))
            .collect(),
    }
}

fn surgeries() -> Vec<Surgery> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/surgery.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["surgeries"].as_array().expect("surgeries").iter().map(|s| {
        let u = |k: &str| s[k].as_u64().unwrap_or(0) as usize;
        let node_map = s["nodes"].as_object().expect("nodes");
        let max_node = node_map.keys().map(|k| k.parse::<usize>().expect("id")).max().unwrap_or(0);
        let mut nodes: Vec<MazeNode> = (0..=max_node)
            .map(|_| MazeNode { x: 0, y: 0, neighbours: Vec::new(), stack_alias: 0 })
            .collect();
        for (k, p) in node_map {
            let i = k.parse::<usize>().expect("id");
            nodes[i].x = p[0].as_i64().expect("x") as i32;
            nodes[i].y = p[1].as_i64().expect("y") as i32;
        }
        let slots = |key: &str| -> Vec<Option<SurgeryEdge>> {
            let m = s[key].as_object().expect("edges");
            let max = m.keys().map(|k| k.parse::<usize>().expect("id")).max().unwrap_or(0);
            let mut out: Vec<Option<SurgeryEdge>> = vec![None; max + 1];
            for (k, e) in m {
                out[k.parse::<usize>().expect("id")] = Some(edge_from(e));
            }
            out
        };
        let e1 = s["e1"].as_array().expect("e1");
        Surgery {
            design: s["design"].as_str().expect("design").to_string(),
            kind: s["kind"].as_u64().expect("kind") as u8,
            n1: u("n1"), a1: u("a1"), a2: u("a2"), c1: u("c1"), c2: u("c2"),
            e1: (e1[0].as_i64().expect("x") as i32, e1[1].as_i64().expect("y") as i32),
            edge_n1a1: u("edge_n1a1"), edge_n1a2: u("edge_n1a2"), edge_c1c2: u("edge_c1c2"),
            nodes,
            before: slots("before"),
            after: slots("after"),
        }
    }).collect()
}

/// Run one captured surgery over a slot table holding exactly the edges it touches.
fn run(s: &Surgery) -> Vec<Option<SurgeryEdge>> {
    let n = s.before.len().max(s.after.len());
    let mut edges: Vec<SurgeryEdge> = (0..n)
        .map(|i| s.before.get(i).and_then(|e| e.clone()).unwrap_or(SurgeryEdge {
            n1: 0, n2: 0, n1a: 0, n2a: 0, routelen: 0, len: 0, is_maze_route: false, grids: vec![(0, 0)],
        }))
        .collect();

    if s.kind == 1 {
        update_route_type1(&s.nodes, s.n1, s.a1, s.a2, s.e1, &mut edges,
                           s.edge_n1a1, s.edge_n1a2).expect("type 1 succeeds");
    } else {
        update_route_type2(&s.nodes, s.n1, s.a1, s.a2, s.c1, s.c2, s.e1, &mut edges,
                           s.edge_n1a1, s.edge_n1a2, s.edge_c1c2).expect("type 2 succeeds");
    }
    edges.into_iter().map(Some).collect()
}

#[test]
fn surgeries_match_the_reference() {
    let all = surgeries();
    assert!(all.len() >= 1000, "corpus too thin: {}", all.len());
    let (mut t1, mut t2) = (0usize, 0usize);

    for s in &all {
        let got = run(s);
        for (slot, want) in s.after.iter().enumerate() {
            let Some(want) = want else { continue };
            let got = got[slot].as_ref().expect("slot present");
            assert_eq!(
                got.grids, want.grids,
                "grids of edge {slot} after type {} on {}", s.kind, s.design
            );
            assert_eq!(got.routelen, want.routelen, "routelen of edge {slot} on {}", s.design);
            assert_eq!(got.len, want.len, "length of edge {slot} on {}", s.design);
            // ⚠️ The second variant leaves endpoints to its caller, so they are only compared
            // where the function itself sets them.
            if s.kind == 1 {
                assert_eq!((got.n1, got.n2), (want.n1, want.n2),
                           "endpoints of edge {slot} on {}", s.design);
            }
        }
        if s.kind == 1 { t1 += 1 } else { t2 += 1 }
    }
    assert!(t1 >= 500, "too few of the first shape: {t1}");
    assert!(t2 >= 300, "too few of the second shape: {t2}");
}

/// ⚠️ Both orientation decisions must be seen both ways, or half the endpoint assignments are
/// never made in anger.
#[test]
fn every_orientation_combination_is_present() {
    let all = surgeries();
    let mut seen: std::collections::BTreeSet<(bool, bool)> = Default::default();
    for s in all.iter().filter(|s| s.kind == 1) {
        seen.insert((s.nodes[s.a1].x <= s.e1.0, s.e1.0 <= s.nodes[s.a2].x));
    }
    assert_eq!(seen.len(), 4, "only {} orientation combinations: {seen:?}", seen.len());
}

/// ⛔ The merged edge is written into the slot of the edge that is about to be split.
///
/// The split then reads points that no longer exist in that slot — it works only because every
/// list was copied out first. Asserted on the captured data: the two rewritten halves must still
/// join at the position the node moved to.
#[test]
fn the_recycled_slot_is_copied_before_it_is_overwritten() {
    let all = surgeries();
    let mut checked = 0usize;
    for s in all.iter().filter(|s| s.kind == 2) {
        let got = run(s);
        let head = got[s.edge_n1a1].as_ref().expect("head");
        let tail = got[s.edge_n1a2].as_ref().expect("tail");
        assert_eq!(
            *head.grids.last().expect("non-empty"), s.e1,
            "the first half must end where the node moved to, on {}", s.design
        );
        assert_eq!(
            *tail.grids.first().expect("non-empty"), s.e1,
            "and the second half must start there, on {}", s.design
        );
        checked += 1;
    }
    assert!(checked >= 300, "too few of the second shape: {checked}");
}

/// A never-routed edge yields a single point, not an empty list.
///
/// ⛔ **Constructed: every captured input is a real route**, so the corpus cannot reach this arm —
/// and an empty list would make the joins below it produce a shorter path with no error.
#[test]
fn an_unrouted_edge_yields_one_point() {
    let nodes = vec![
        MazeNode { x: 4, y: 9, neighbours: Vec::new(), stack_alias: 0 },
        MazeNode { x: 7, y: 9, neighbours: Vec::new(), stack_alias: 0 },
    ];
    let edges = vec![SurgeryEdge {
        n1: 0, n2: 1, n1a: 0, n2a: 1, routelen: 0, len: 0, is_maze_route: false, grids: vec![(99, 99)],
    }];
    // Asked from either end, the answer is that end's own coordinates — not the stored point.
    assert_eq!(copy_grids(&nodes, 0, &edges, 0), vec![(4, 9)]);
    assert_eq!(copy_grids(&nodes, 1, &edges, 0), vec![(7, 9)]);
}

/// Points are handed back starting from whichever endpoint is asked for.
#[test]
fn copying_reverses_when_asked_from_the_far_end() {
    let nodes = vec![
        MazeNode { x: 1, y: 1, neighbours: Vec::new(), stack_alias: 0 },
        MazeNode { x: 3, y: 1, neighbours: Vec::new(), stack_alias: 0 },
    ];
    let edges = vec![SurgeryEdge {
        n1: 0, n2: 1, n1a: 0, n2a: 1, routelen: 2, len: 2, is_maze_route: true,
        grids: vec![(1, 1), (2, 1), (3, 1)],
    }];
    assert_eq!(copy_grids(&nodes, 0, &edges, 0), vec![(1, 1), (2, 1), (3, 1)]);
    assert_eq!(copy_grids(&nodes, 1, &edges, 0), vec![(3, 1), (2, 1), (1, 1)]);

    // ⚠️ Only the routed prefix is taken: a buffer longer than the route is not copied whole.
    let long = vec![SurgeryEdge {
        n1: 0, n2: 1, n1a: 0, n2a: 1, routelen: 1, len: 1, is_maze_route: true,
        grids: vec![(1, 1), (2, 1), (9, 9)],
    }];
    assert_eq!(copy_grids(&nodes, 0, &long, 0), vec![(1, 1), (2, 1)]);
}
