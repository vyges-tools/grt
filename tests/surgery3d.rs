// SPDX-License-Identifier: Apache-2.0
//! R18g3 — the 3D tree surgery's wiring, end to end on the whole tree.
//!
//! Golden `surgery3d.json`, both cost modes: per surgery (sampled, or any with a split / shift),
//! the whole tree before and after, the backtrace it acts on, the two `corr_edge` lookups, and the
//! usage requests, retry additions and shift flags.

use std::collections::HashMap;

use serde_json::Value;
use vyges_grt::{
    tree_surgery_3d, Backtrace3D, Node3D, NodeConnections, Point3D, RouteType, SurgEdge3D, Tree3D,
};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn pts(v: &Value) -> Vec<Point3D> {
    v.as_array().expect("pts").iter().map(|p| Point3D {
        x: int(&p[0]) as i16,
        y: int(&p[1]) as i16,
        layer: int(&p[2]) as i16,
    }).collect()
}

fn route_type(t: i64) -> RouteType {
    [RouteType::NoRoute, RouteType::LRoute, RouteType::ZRoute, RouteType::MazeRoute][t as usize]
}

fn node(v: &Value) -> Node3D {
    let mut conn = NodeConnections {
        e_id: [0; 10],
        heights: [0; 10],
        con_cnt: int(&v["con"]) as i16,
        bot_layer: int(&v["bot"]) as i16,
        top_layer: int(&v["top"]) as i16,
        l_id: int(&v["lid"]) as i32,
        h_id: int(&v["hid"]) as i32,
    };
    for (i, p) in v["list"].as_array().expect("list").iter().enumerate() {
        conn.e_id[i] = int(&p[0]) as i32;
        conn.heights[i] = int(&p[1]) as i16;
    }
    Node3D {
        x: int(&v["x"]) as i16,
        y: int(&v["y"]) as i16,
        stack_alias: int(&v["al"]) as usize,
        assigned: int(&v["as"]) == 1,
        status: int(&v["st"]) as i16,
        conn,
        nbr: v["nb"].as_array().expect("nb").iter().map(|p| (int(&p[0]) as usize, int(&p[1]) as usize)).collect(),
    }
}

fn edge(v: &Value) -> SurgEdge3D {
    SurgEdge3D {
        n1: int(&v["n1"]) as usize,
        n2: int(&v["n2"]) as usize,
        n1a: int(&v["n1a"]) as usize,
        n2a: int(&v["n2a"]) as usize,
        len: int(&v["len"]) as i32,
        route_type: route_type(int(&v["type"])),
        routelen: int(&v["rl"]) as i32,
        grids: pts(&v["grids"]),
    }
}

fn replay(g: &Value) -> usize {
    let records = g["records"].as_array().expect("records");
    for r in records {
        let who = format!("{} net {} edge {}", r["design"], r["net"], r["edge"]);
        let mut tree = Tree3D {
            num_terminals: int(&r["terms"]) as usize,
            num_layers: int(&r["layers"]) as i16,
            pin_layers: r["pinl"].as_array().expect("pinl").iter().map(|v| int(v) as i16).collect(),
            nodes: r["B"]["n"].as_array().expect("n").iter().map(node).collect(),
            edges: r["B"]["e"].as_array().expect("e").iter().map(edge).collect(),
        };
        let edge_id = int(&r["edge"]) as usize;
        let e = &tree.edges[edge_id];
        let orig = (
            (tree.nodes[e.n1].x, tree.nodes[e.n1].y),
            (tree.nodes[e.n2].x, tree.nodes[e.n2].y),
        );
        let aliases = (e.n1a, e.n2a);
        let grids = pts(&r["grids"]);
        let bt = Backtrace3D {
            head_room: int(&r["headroom"]) as usize,
            orig_layer: int(&r["origL"]) as i16,
            last_layer: int(&r["lastL"]) as i16,
            grids: grids.clone(),
        };
        // The two lookups the reference made, keyed by the cell it read.
        let look = &r["look"];
        let mut corr_at: HashMap<(i16, i16, i16), usize> = HashMap::new();
        let (first, last) = (grids[0], grids[grids.len() - 1]);
        if int(&look["corr1"]) >= 0 {
            corr_at.insert((bt.orig_layer, first.y, first.x), int(&look["corr1"]) as usize);
        }
        if int(&look["corr2"]) >= 0 {
            corr_at.insert((int(&look["origL2"]) as i16, last.y, last.x), int(&look["corr2"]) as usize);
        }
        let corr = |l: i16, y: i16, x: i16| corr_at.get(&(l, y, x)).copied().unwrap_or(0);

        let out = tree_surgery_3d(&mut tree, edge_id, &bt, orig, aliases, &corr)
            .unwrap_or_else(|e| panic!("{who}: {e:?}"));

        assert_eq!((out.n1_shift, out.n2_shift), (int(&r["shift1"]) == 1, int(&r["shift2"]) == 1), "{who}: shifts");
        let want_retry: Vec<usize> = r["retry"].as_array().expect("retry").iter().map(|v| int(v) as usize).collect();
        assert_eq!(out.retry, want_retry, "{who}: retry list");
        let want_usage: Vec<(bool, i16, i16, i16)> = r["usage"].as_array().expect("usage").iter().map(|u| {
            (u[0].as_str() == Some("H"), int(&u[1]) as i16, int(&u[2]) as i16, int(&u[3]) as i16)
        }).collect();
        assert_eq!(out.usage, want_usage, "{who}: usage requests");

        let an = r["A"]["n"].as_array().expect("n");
        let ae = r["A"]["e"].as_array().expect("e");
        assert_eq!(tree.nodes.len(), an.len(), "{who}: node count");
        assert_eq!(tree.edges.len(), ae.len(), "{who}: edge count");
        for (i, (got, w)) in tree.nodes.iter().zip(an).enumerate() {
            let want = node(w);
            let k = want.conn.con_cnt as usize;
            assert_eq!(
                (got.x, got.y, got.stack_alias, got.assigned, got.status, got.nbr.clone()),
                (want.x, want.y, want.stack_alias, want.assigned, want.status, want.nbr.clone()),
                "{who}: node {i}"
            );
            assert_eq!(
                (got.conn.con_cnt, got.conn.bot_layer, got.conn.top_layer, got.conn.l_id, got.conn.h_id),
                (want.conn.con_cnt, want.conn.bot_layer, want.conn.top_layer, want.conn.l_id, want.conn.h_id),
                "{who}: node {i} layers"
            );
            assert_eq!(&got.conn.e_id[..k], &want.conn.e_id[..k], "{who}: node {i} edge list");
            assert_eq!(&got.conn.heights[..k], &want.conn.heights[..k], "{who}: node {i} heights");
        }
        for (i, (got, w)) in tree.edges.iter().zip(ae).enumerate() {
            assert_eq!(got, &edge(w), "{who}: edge {i}");
        }
    }
    records.len()
}

/// Every captured surgery: the whole tree after, and what the surgery reported.
#[test]
fn tree_surgery_3d_matches_the_reference() {
    let g = read(&format!("{}/examples/grt_gate/surgery3d.json", env!("CARGO_MANIFEST_DIR")));
    let n = replay(&g);
    assert!(n >= 200, "too few surgeries: {n}");
}

/// GRT_SURGERY3D_FULL=/path/to/g3-all.json cargo test --test surgery3d -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn tree_surgery_3d_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_SURGERY3D_FULL").expect("set GRT_SURGERY3D_FULL");
    eprintln!("exhaustive: {} surgeries", replay(&read(&path)));
}
