// SPDX-License-Identifier: Apache-2.0
//! R14 driver — giving a pin a stand-in that can move in its place.
//!
//! 100 splits from four designs, across all four shapes: the pin holding two or three neighbours,
//! and the caller sitting first in its list or not.
//!
//! A pin cannot be relocated, so when the router wants its position to change a duplicate node is
//! created at the same coordinates and joined to it by a **zero-length edge**. The duplicate takes
//! over the pin's connections and moves instead.
//!
//! ⛔ **The whole tree is compared, before and after** — every node's neighbours and every edge's
//! endpoints — not just the node that was added. A split that adds the right node and re-points
//! one neighbour too few leaves a tree that is still a tree, and still wrong.

use serde_json::Value;
use vyges_grt::{split_edge, MazeNode, SurgeryEdge};

struct Split {
    design: String,
    n1: usize,
    n2: usize,
    edge: usize,
    before_nodes: Vec<MazeNode>,
    before_edges: Vec<SurgeryEdge>,
    after_nodes: Vec<MazeNode>,
    after_edges: Vec<SurgeryEdge>,
    returned: usize,
}

fn node_from(v: &Value) -> MazeNode {
    MazeNode {
        x: v["x"].as_i64().expect("x") as i32,
        y: v["y"].as_i64().expect("y") as i32,
        stack_alias: v["stack_alias"].as_u64().expect("alias") as usize,
        neighbours: v["neighbours"].as_array().expect("nbrs").iter()
            .map(|p| (p[0].as_u64().expect("n") as usize, p[1].as_u64().expect("e") as usize))
            .collect(),
    }
}

fn edge_from(v: &Value) -> SurgeryEdge {
    let u = |k: &str| v[k].as_u64().unwrap_or_else(|| panic!("{k}")) as usize;
    SurgeryEdge {
        n1: u("n1"), n2: u("n2"), n1a: u("n1a"), n2a: u("n2a"),
        routelen: u("routelen"),
        len: v["len"].as_i64().expect("len") as i32,
        is_maze_route: v["is_maze_route"].as_bool().expect("type"),
        grids: v["grids"].as_array().expect("grids").iter()
            .map(|p| (p[0].as_i64().expect("x") as i32, p[1].as_i64().expect("y") as i32))
            .collect(),
    }
}

fn splits() -> Vec<Split> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/splitedge.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let nodes = |v: &Value| v.as_array().expect("nodes").iter().map(node_from).collect();
    let edges = |v: &Value| v.as_array().expect("edges").iter().map(edge_from).collect();
    v["splits"].as_array().expect("splits").iter().map(|s| Split {
        design: s["design"].as_str().expect("design").to_string(),
        n1: s["n1"].as_u64().expect("n1") as usize,
        n2: s["n2"].as_u64().expect("n2") as usize,
        edge: s["edge"].as_u64().expect("edge") as usize,
        before_nodes: nodes(&s["before_nodes"]),
        before_edges: edges(&s["before_edges"]),
        after_nodes: nodes(&s["after_nodes"]),
        after_edges: edges(&s["after_edges"]),
        returned: s["returned"].as_u64().expect("ret") as usize,
    }).collect()
}

#[test]
fn splits_rebuild_the_tree_exactly_as_the_reference_does() {
    let all = splits();
    assert!(all.len() >= 80, "corpus too thin: {}", all.len());

    for s in &all {
        let mut nodes = s.before_nodes.clone();
        let mut edges = s.before_edges.clone();
        let got = split_edge(&mut nodes, &mut edges, s.n1, s.n2, s.edge);

        assert_eq!(got, s.returned, "the new node's index on {}", s.design);
        assert_eq!(
            nodes.len(), s.after_nodes.len(),
            "node count on {}", s.design
        );
        for (i, (got, want)) in nodes.iter().zip(&s.after_nodes).enumerate() {
            assert_eq!(
                (got.x, got.y, got.stack_alias), (want.x, want.y, want.stack_alias),
                "node {i} position or alias on {}", s.design
            );
            assert_eq!(got.neighbours, want.neighbours, "node {i} neighbours on {}", s.design);
        }
        assert_eq!(edges, s.after_edges, "edges on {}", s.design);
    }
}

/// ⛔ The stand-in sits exactly on the pin, joined by an edge of no length.
#[test]
fn the_stand_in_is_coincident_with_the_pin() {
    for s in &splits() {
        let mut nodes = s.before_nodes.clone();
        let mut edges = s.before_edges.clone();
        let new_id = split_edge(&mut nodes, &mut edges, s.n1, s.n2, s.edge);

        assert_eq!(
            (nodes[new_id].x, nodes[new_id].y), (nodes[s.n2].x, nodes[s.n2].y),
            "the stand-in must sit on the pin, on {}", s.design
        );
        // ⛔ It inherits the pin's alias rather than taking its own identity, so the two share
        // connection state exactly as coincident nodes do elsewhere in this engine.
        assert_eq!(
            nodes[new_id].stack_alias, s.before_nodes[s.n2].stack_alias,
            "the stand-in must inherit the pin's alias, on {}", s.design
        );
        let joining = edges.last().expect("the new edge");
        assert_eq!(joining.len, 0, "the joining edge has no length, on {}", s.design);
        assert_eq!(joining.routelen, 0);
        assert_eq!(joining.grids, vec![(nodes[s.n2].x, nodes[s.n2].y)]);
    }
}

/// ⚠️ The pin ends with one fewer neighbour than it began — the caller is dropped outright.
#[test]
fn the_pin_loses_a_neighbour() {
    let all = splits();
    let (mut two, mut three) = (0usize, 0usize);
    for s in &all {
        let before = s.before_nodes[s.n2].neighbours.len();
        let after = s.after_nodes[s.n2].neighbours.len();
        assert_eq!(after, before - 1, "the pin's degree on {}", s.design);
        assert!(
            !s.after_nodes[s.n2].neighbours.iter().any(|(v, _)| *v == s.n1),
            "the caller must no longer be a neighbour of the pin, on {}", s.design
        );
        match before {
            2 => two += 1,
            3 => three += 1,
            other => panic!("unexpected pin degree {other} on {}", s.design),
        }
    }
    // ⚠️ Both degrees must appear, since the rebuild walks the list and the shorter one exercises
    // a different path through it.
    assert!(two >= 20 && three >= 20, "degrees are lopsided: {two} of two, {three} of three");
}
