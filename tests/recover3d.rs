// SPDX-License-Identifier: Apache-2.0
//! R18h — `recoverEdge`, putting back an edge the 3D search could not re-route.
//!
//! Golden `recover3d.json`: BOTH calls the corpus makes (zero-distance crossings on
//! `overlapping_edges`; the search-ran-dry path never fires), each with the whole net tree before
//! and after and every planar step's usage before and after.

use serde_json::Value;
use vyges_grt::{recover_edge, Node3D, NodeConnections, Point3D, RecoverZeroLength, RouteType, SurgEdge3D, Tree3D};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn conn(v: &Value) -> NodeConnections {
    let mut c = NodeConnections {
        e_id: [0; 10],
        heights: [0; 10],
        con_cnt: int(&v["con"]) as i16,
        bot_layer: int(&v["bot"]) as i16,
        top_layer: int(&v["top"]) as i16,
        l_id: int(&v["lid"]) as i32,
        h_id: int(&v["hid"]) as i32,
    };
    for (i, p) in v["list"].as_array().expect("list").iter().enumerate() {
        c.e_id[i] = int(&p[0]) as i32;
        c.heights[i] = int(&p[1]) as i16;
    }
    c
}

fn edge(v: &Value) -> SurgEdge3D {
    SurgEdge3D {
        n1: int(&v["n1"]) as usize,
        n2: int(&v["n2"]) as usize,
        n1a: int(&v["n1a"]) as usize,
        n2a: int(&v["n2a"]) as usize,
        len: int(&v["len"]) as i32,
        route_type: [RouteType::NoRoute, RouteType::LRoute, RouteType::ZRoute, RouteType::MazeRoute][int(&v["type"]) as usize],
        routelen: int(&v["rl"]) as i32,
        grids: v["grids"].as_array().expect("g").iter().map(|p| Point3D {
            x: int(&p[0]) as i16,
            y: int(&p[1]) as i16,
            layer: int(&p[2]) as i16,
        }).collect(),
    }
}

/// Both captured recoveries: every node's registration after, every edge untouched, and the usage
/// requests matching what the reference applied (+1 per planar step for these edge costs).
#[test]
fn recover_edge_matches_the_reference() {
    let g = read(&format!("{}/examples/grt_gate/recover3d.json", env!("CARGO_MANIFEST_DIR")));
    let records = g["records"].as_array().expect("records");
    assert_eq!(records.len(), 2, "the corpus makes exactly two recoverEdge calls");
    for r in records {
        let who = format!("{} net {} edge {}", r["design"], r["net"], r["edge"]);
        let mut tree = Tree3D {
            num_terminals: 0,
            num_layers: 0,
            pin_layers: vec![],
            nodes: r["B"]["n"].as_array().expect("n").iter().map(|n| Node3D {
                x: 0, y: 0, stack_alias: 0, assigned: int(&n["as"]) == 1, status: 0, conn: conn(n), nbr: vec![],
            }).collect(),
            edges: r["B"]["e"].as_array().expect("e").iter().map(edge).collect(),
        };
        let usage = recover_edge(&mut tree, int(&r["edge"]) as usize).unwrap_or_else(|_| panic!("{who}: zero length"));
        for (i, (got, w)) in tree.nodes.iter().zip(r["A"]["n"].as_array().expect("n")).enumerate() {
            let want = conn(w);
            let k = want.con_cnt as usize;
            assert_eq!(
                (got.assigned, got.conn.con_cnt, got.conn.bot_layer, got.conn.top_layer, got.conn.l_id, got.conn.h_id),
                (int(&w["as"]) == 1, want.con_cnt, want.bot_layer, want.top_layer, want.l_id, want.h_id),
                "{who}: node {i}"
            );
            assert_eq!(&got.conn.e_id[..k], &want.e_id[..k], "{who}: node {i} edge list");
            assert_eq!(&got.conn.heights[..k], &want.heights[..k], "{who}: node {i} heights");
        }
        for (i, (got, w)) in tree.edges.iter().zip(r["A"]["e"].as_array().expect("e")).enumerate() {
            assert_eq!(got, &edge(w), "{who}: edge {i} must be untouched");
        }
        let steps = r["steps"].as_array().expect("steps");
        assert_eq!(usage.len(), steps.len(), "{who}: planar steps");
        for (u, s) in usage.iter().zip(steps) {
            let horizontal = s[0].as_str() == Some("H");
            assert_eq!(*u, (horizontal, int(&s[1]) as i16, int(&s[2]) as i16, int(&s[3]) as i16), "{who}: usage cell");
            assert!(int(&s[5]) > int(&s[4]) && int(&s[7]) > int(&s[6]), "{who}: usage was charged");
        }
    }
}

fn node(con: &[(i32, i16)], bot: i16, top: i16) -> Node3D {
    let mut c = NodeConnections { e_id: [0; 10], heights: [0; 10], con_cnt: con.len() as i16, bot_layer: bot, top_layer: top, l_id: 7, h_id: 7 };
    for (i, &(e, h)) in con.iter().enumerate() {
        c.e_id[i] = e;
        c.heights[i] = h;
    }
    Node3D { x: 0, y: 0, stack_alias: 0, assigned: false, status: 0, conn: c, nbr: vec![] }
}

fn p(x: i16, y: i16, layer: i16) -> Point3D {
    Point3D { x, y, layer }
}

/// ⛔ Re-registration is strict at both ends; ⚠️ a step moving in both x and y is SKIPPED (the
/// rip-up it undoes would have aborted on it); vias request nothing.
#[test]
fn recovery_registers_strictly_and_skips_vias_and_diagonals() {
    let mut tree = Tree3D {
        num_terminals: 0,
        num_layers: 6,
        pin_layers: vec![],
        nodes: vec![node(&[(3, 2)], 2, 2), node(&[], 6, -1)],
        edges: vec![SurgEdge3D {
            n1: 0, n2: 1, n1a: 0, n2a: 1, len: 4, route_type: RouteType::MazeRoute, routelen: 3,
            grids: vec![p(0, 0, 2), p(1, 0, 2), p(1, 0, 3), p(2, 1, 3)],
        }],
    };
    let usage = recover_edge(&mut tree, 0).expect("ok");
    assert_eq!(usage, vec![(true, 2, 0, 0)], "one planar step; the via and the diagonal request nothing");
    // End 1 arrives at layer 2 = its bottom and top: strict, so neither id moves.
    assert_eq!((tree.nodes[0].conn.l_id, tree.nodes[0].conn.h_id, tree.nodes[0].conn.con_cnt), (7, 7, 2));
    // End 2 was empty (6, -1): layer 3 becomes both extremes.
    let c = tree.nodes[1].conn;
    assert_eq!((c.bot_layer, c.top_layer, c.l_id, c.h_id), (3, 3, 0, 0));
    assert!(tree.nodes[0].assigned && tree.nodes[1].assigned);
}

/// ⛔ A zero-length edge is fatal here (GRT-206), where the rip-up it undoes merely declines it.
#[test]
fn recovering_a_zero_length_edge_is_fatal() {
    let mut tree = Tree3D {
        num_terminals: 0, num_layers: 6, pin_layers: vec![],
        nodes: vec![node(&[], 6, -1), node(&[], 6, -1)],
        edges: vec![SurgEdge3D { n1: 0, n2: 1, n1a: 0, n2a: 1, len: 0, route_type: RouteType::MazeRoute, routelen: 0, grids: vec![p(0, 0, 1)] }],
    };
    assert_eq!(recover_edge(&mut tree, 0), Err(RecoverZeroLength));
}
