// SPDX-License-Identifier: Apache-2.0
//! R18d — `setupHeap3D` / `addNeighborPoints`, seeding the 3D search.
//!
//! Golden `setupheap3d.json`, both cost modes: per call the tree as it stood (after the rip-up),
//! the edge, the region, and both heaps in push order with the `corr_edge` value at each seed.

use std::collections::HashMap;

use serde_json::Value;
use vyges_grt::{setup_heap_3d, Cell3, Point3D, SeedEdge, SeedNode};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn golden() -> Value {
    read(&format!("{}/examples/grt_gate/setupheap3d.json", env!("CARGO_MANIFEST_DIR")))
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn cells(v: &Value) -> Vec<(Cell3, i64)> {
    v.as_array().expect("heap").iter().map(|c| {
        ((int(&c[0]) as i16, int(&c[1]) as i16, int(&c[2]) as i16), int(&c[3]))
    }).collect()
}

fn replay(g: &Value) -> usize {
    let records = g["records"].as_array().expect("records");
    for r in records {
        let who = format!("{} edge {}", r["design"], r["edge"]);
        let nodes: Vec<SeedNode> = r["nodes"].as_array().expect("nodes").iter().map(|n| SeedNode {
            x: int(&n["x"]) as i16,
            y: int(&n["y"]) as i16,
            neighbours: n["nbr"].as_array().expect("nbr").iter()
                .map(|p| (int(&p[0]) as usize, int(&p[1]) as usize)).collect(),
            stack_alias: int(&n["alias"]) as usize,
            bot_layer: int(&n["bot"]) as i16,
            top_layer: int(&n["top"]) as i16,
        }).collect();
        let edges: Vec<SeedEdge> = r["edges"].as_array().expect("edges").iter().map(|e| SeedEdge {
            n1: int(&e["n1"]) as usize,
            n2: int(&e["n2"]) as usize,
            routelen: int(&e["routelen"]) as i32,
            maze_route: int(&e["type"]) == 3,
            grids: e["grids"].as_array().expect("grids").iter().map(|p| Point3D {
                x: int(&p[0]) as i16,
                y: int(&p[1]) as i16,
                layer: int(&p[2]) as i16,
            }).collect(),
        }).collect();
        let access = match r["access"].as_array() {
            Some(a) => (int(&a[0]) as i16, int(&a[1]) as i16),
            None => (-1, -1),
        };
        let reg: Vec<i32> = r["region"].as_array().expect("region").iter().map(|v| int(v) as i32).collect();
        let h = setup_heap_3d(
            int(&r["terms"]) as usize, &nodes, &edges, int(&r["edge"]) as usize, access,
            (reg[0], reg[1], reg[2], reg[3]),
        );
        let (src, dst) = (cells(&r["src"]), cells(&r["dst"]));
        assert_eq!(h.src, src.iter().map(|c| c.0).collect::<Vec<_>>(), "{who}: source push order");
        assert_eq!(h.dest, dst.iter().map(|c| c.0).collect::<Vec<_>>(), "{who}: dest push order");
        // Every cell our walk wrote must hold the reference's final value (last write wins).
        let mut last: HashMap<Cell3, usize> = HashMap::new();
        for &(cell, edge) in &h.corr_edge {
            last.insert(cell, edge);
        }
        for &(cell, want) in src.iter().chain(&dst) {
            if let Some(&got) = last.get(&cell) {
                assert_eq!(got as i64, want, "{who}: corr_edge at {cell:?}");
            }
        }
    }
    records.len()
}

/// Every captured call: both heaps in push order, and `corr_edge` at every cell written.
#[test]
fn setup_heap_3d_matches_the_reference() {
    let n = replay(&golden());
    assert!(n >= 80, "too few calls: {n}");
}

/// GRT_SETUPHEAP3D_FULL=/path/to/m3d-all.json cargo test --test setupheap3d -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn setup_heap_3d_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_SETUPHEAP3D_FULL").expect("set GRT_SETUPHEAP3D_FULL");
    eprintln!("exhaustive: {} calls", replay(&read(&path)));
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn p(x: i16, y: i16, layer: i16) -> Point3D {
    Point3D { x, y, layer }
}

fn node(x: i16, y: i16, nbrs: &[(usize, usize)], alias: usize, bot: i16, top: i16) -> SeedNode {
    SeedNode { x, y, neighbours: nbrs.to_vec(), stack_alias: alias, bot_layer: bot, top_layer: top }
}

/// ⛔ A two-pin net seeds ONE cell per side, at the pin's access layer — not the node's range.
#[test]
fn a_two_pin_net_seeds_only_the_access_layers() {
    let nodes = vec![node(0, 0, &[(1, 0)], 0, 1, 4), node(3, 0, &[(0, 0)], 1, 2, 2)];
    let edges = vec![SeedEdge { n1: 0, n2: 1, routelen: 3, maze_route: true, grids: vec![] }];
    let h = setup_heap_3d(2, &nodes, &edges, 0, (3, 2), (0, 0, 0, 0));
    assert_eq!((h.src, h.dest), (vec![(3, 0, 0)], vec![(2, 0, 3)]));
}

/// A four-node net: the rip-up edge is 0 (nodes 0-1). Node 0's side reaches node 2 over edge 1,
/// a maze route with an interior point; node 1's side reaches node 3 over edge 2.
fn four_node_net(edge1_maze: bool) -> (Vec<SeedNode>, Vec<SeedEdge>) {
    let nodes = vec![
        node(0, 0, &[(1, 0), (2, 1)], 0, 1, 2), // start: range from ITS alias (itself)
        node(5, 0, &[(0, 0), (3, 2)], 1, 1, 1),
        node(0, 3, &[(0, 1)], 0, 0, 0),         // aliased to node 0: seeded at node 0's range
        node(9, 9, &[(1, 2)], 3, 1, 1),         // outside the region
    ];
    let edges = vec![
        SeedEdge { n1: 0, n2: 1, routelen: 5, maze_route: true, grids: vec![] },
        SeedEdge {
            n1: 0,
            n2: 2,
            routelen: 3,
            maze_route: edge1_maze,
            grids: vec![p(0, 0, 1), p(0, 1, 1), p(0, 2, 1), p(0, 3, 1)],
        },
        SeedEdge { n1: 1, n2: 3, routelen: 2, maze_route: true, grids: vec![p(5, 0, 1), p(9, 0, 1), p(9, 9, 1)] },
    ];
    (nodes, edges)
}

/// ⛔ The start node takes every layer of its alias's range with no region test; a neighbour takes
/// its ALIAS's range; interior points take their own layer; out-of-region cells are not seeded —
/// but the walk still passes through them.
#[test]
fn nodes_seed_their_alias_range_and_interiors_their_own_layer() {
    let (nodes, edges) = four_node_net(true);
    let h = setup_heap_3d(3, &nodes, &edges, 0, (-1, -1), (0, 5, 0, 5));
    assert_eq!(h.src, vec![(1, 0, 0), (2, 0, 0), (1, 3, 0), (2, 3, 0), (1, 1, 0), (1, 2, 0)]);
    assert_eq!(h.dest, vec![(1, 0, 5)]);
    // The start node's own cells are never recorded in corr_edge.
    assert!(h.corr_edge.iter().all(|(c, _)| *c != (1, 0, 0) && *c != (2, 0, 0)));
}

/// ⛔ A counted edge that is NOT a maze route still seeds its far node but skips its interior —
/// where the 2D setup would abort. Never captured: every counted edge is a maze route by then.
#[test]
fn a_non_maze_edge_seeds_its_node_but_not_its_interior() {
    let (nodes, edges) = four_node_net(false);
    let h = setup_heap_3d(3, &nodes, &edges, 0, (-1, -1), (0, 5, 0, 5));
    assert_eq!(h.src, vec![(1, 0, 0), (2, 0, 0), (1, 3, 0), (2, 3, 0)]);
}

/// ⛔ An edge with no steps is not counted — its far node is not seeded — but the walk still goes
/// through it to the nodes beyond.
#[test]
fn an_edge_without_steps_is_walked_through_but_not_seeded() {
    let (nodes, mut edges) = four_node_net(true);
    edges[1].routelen = 0;
    let h = setup_heap_3d(3, &nodes, &edges, 0, (-1, -1), (0, 5, 0, 5));
    assert_eq!(h.src, vec![(1, 0, 0), (2, 0, 0)]);
}
