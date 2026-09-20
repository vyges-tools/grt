// SPDX-License-Identifier: Apache-2.0
//! R14 driver — re-pointing the tree after a node has moved onto a different edge.
//!
//! The surgery rewrites three edges' points; this is the other half — their endpoints, and the
//! adjacency of the **five** nodes involved. 509 rewirings from four designs.
//!
//! ⛔ **301 of them have the four surrounding nodes NOT all distinct** — a neighbour of the moved
//! node is also an endpoint of the edge it landed on. Each of the four patches replaces the
//! **first** matching entry, so when two of them touch the same node the order they run in
//! decides the answer. That count is asserted, because a corpus of only distinct cases would say
//! nothing about it.

use serde_json::Value;
use vyges_grt::{rewire_after_type2, MazeNode, SurgeryEdge};

struct Rewire {
    design: String,
    n1: usize,
    n2: usize,
    a1: usize,
    a2: usize,
    c1: usize,
    c2: usize,
    edge_n1n2: usize,
    edge_n1a1: usize,
    edge_n1a2: usize,
    edge_c1c2: usize,
    before_nodes: Vec<Vec<(usize, usize)>>,
    after_nodes: Vec<Vec<(usize, usize)>>,
    before_edges: Vec<(usize, usize)>,
    after_edges: Vec<(usize, usize)>,
}

fn rewires() -> Vec<Rewire> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/rewire.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let nodes = |v: &Value| -> Vec<Vec<(usize, usize)>> {
        v.as_array().expect("nodes").iter().map(|n| {
            n.as_array().expect("list").iter()
                .map(|p| (p[0].as_u64().expect("n") as usize, p[1].as_u64().expect("e") as usize))
                .collect()
        }).collect()
    };
    let edges = |v: &Value| -> Vec<(usize, usize)> {
        v.as_array().expect("edges").iter()
            .map(|e| (e[0].as_u64().expect("n1") as usize, e[1].as_u64().expect("n2") as usize))
            .collect()
    };
    v["rewires"].as_array().expect("rewires").iter().map(|r| {
        let u = |k: &str| r[k].as_u64().unwrap_or_else(|| panic!("{k}")) as usize;
        Rewire {
            design: r["design"].as_str().expect("design").to_string(),
            n1: u("n1"), n2: u("n2"), a1: u("a1"), a2: u("a2"), c1: u("c1"), c2: u("c2"),
            edge_n1n2: u("edge_n1n2"), edge_n1a1: u("edge_n1a1"),
            edge_n1a2: u("edge_n1a2"), edge_c1c2: u("edge_c1c2"),
            before_nodes: nodes(&r["before_nodes"]),
            after_nodes: nodes(&r["after_nodes"]),
            before_edges: edges(&r["before_edges"]),
            after_edges: edges(&r["after_edges"]),
        }
    }).collect()
}

fn run(r: &Rewire) -> (Vec<MazeNode>, Vec<SurgeryEdge>) {
    let mut nodes: Vec<MazeNode> = r.before_nodes.iter().map(|nbrs| MazeNode {
        x: 0, y: 0, stack_alias: 0, neighbours: nbrs.clone(),
    }).collect();
    let mut edges: Vec<SurgeryEdge> = r.before_edges.iter().map(|&(n1, n2)| SurgeryEdge {
        n1, n2, n1a: 0, n2a: 0, routelen: 0, grids: vec![(0, 0)],
        is_maze_route: true, len: 0,
    }).collect();
    rewire_after_type2(
        &mut nodes, &mut edges, r.n1, r.n2, r.a1, r.a2, r.c1, r.c2,
        r.edge_n1n2, r.edge_n1a1, r.edge_n1a2, r.edge_c1c2,
    );
    (nodes, edges)
}

#[test]
fn rewirings_match_the_reference() {
    let all = rewires();
    assert!(all.len() >= 400, "corpus too thin: {}", all.len());

    for r in &all {
        let (nodes, edges) = run(r);
        for (i, (got, want)) in nodes.iter().zip(&r.after_nodes).enumerate() {
            assert_eq!(
                got.neighbours, *want,
                "node {i} neighbours on {} (moved node {}, landed on {}-{})",
                r.design, r.n1, r.c1, r.c2
            );
        }
        let got_edges: Vec<(usize, usize)> = edges.iter().map(|e| (e.n1, e.n2)).collect();
        assert_eq!(got_edges, r.after_edges, "edge endpoints on {}", r.design);
    }
}

/// ⛔ The patch order decides the answer when two of the four touch the same node.
#[test]
fn the_corpus_reaches_the_overlapping_case() {
    let all = rewires();
    let shared = all.iter()
        .filter(|r| {
            let set: std::collections::HashSet<usize> = [r.a1, r.a2, r.c1, r.c2].into();
            set.len() < 4
        })
        .count();
    assert!(
        shared >= 100,
        "only {shared} rewirings have overlapping surrounding nodes — the patch order is untested"
    );
}

/// ⚠️ The moved node's list is rebuilt wholesale, and always to the same three entries.
#[test]
fn the_moved_node_keeps_its_far_endpoint_and_takes_the_edge_it_landed_on() {
    for r in &rewires() {
        let (nodes, _) = run(r);
        assert_eq!(
            nodes[r.n1].neighbours,
            vec![(r.n2, r.edge_n1n2), (r.c1, r.edge_n1a1), (r.c2, r.edge_n1a2)],
            "the moved node's list on {}", r.design
        );
        // ⛔ Its former neighbours are joined to each other, through the slot the edge it landed
        // on used to occupy.
        let (n1a, n2a) = (r.after_edges[r.edge_c1c2].0, r.after_edges[r.edge_c1c2].1);
        assert_eq!((n1a, n2a), (r.a1, r.a2), "the merged edge's endpoints on {}", r.design);
    }
}
