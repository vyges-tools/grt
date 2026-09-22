// SPDX-License-Identifier: Apache-2.0
//! R18i — the whole 3D maze pass, end to end.
//!
//! Golden `pass3d.json`: whole `mazeRouteMSMDOrder3D` calls — the router state and every net at
//! entry, and every net's tree and both usage grids at exit. The replay runs the assembled pass
//! (driver R18a over R18b–h) and compares EVERYTHING it leaves.

use std::collections::HashMap;

use serde_json::Value;
use vyges_grt::{
    counts_for_grt183, maze_route_3d_pass, Maze3DCall, Recovery, Node3D, NodeConnections, PassGrid, PassNet, PassParams, Point3D,
    RouteType, SurgEdge3D, Tree3D,
};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn tree(v: &Value, terms: usize, layers: i16, pinl: &[i16]) -> Tree3D {
    let nodes = v["n"].as_array().expect("n").iter().map(|n| {
        let f: Vec<i64> = (0..10).map(|i| int(&n[i])).collect();
        let mut conn = NodeConnections {
            e_id: [0; 10], heights: [0; 10], con_cnt: f[9] as i16,
            bot_layer: f[5] as i16, top_layer: f[6] as i16, l_id: f[7] as i32, h_id: f[8] as i32,
        };
        for (i, p) in n[11].as_array().expect("list").iter().enumerate() {
            conn.e_id[i] = int(&p[0]) as i32;
            conn.heights[i] = int(&p[1]) as i16;
        }
        Node3D {
            x: f[0] as i16, y: f[1] as i16, stack_alias: f[2] as usize, assigned: f[3] == 1,
            status: f[4] as i16, conn,
            nbr: n[10].as_array().expect("nb").iter().map(|p| (int(&p[0]) as usize, int(&p[1]) as usize)).collect(),
        }
    }).collect();
    let edges = v["e"].as_array().expect("e").iter().map(|e| SurgEdge3D {
        n1: int(&e[0]) as usize, n2: int(&e[1]) as usize, n1a: int(&e[2]) as usize, n2a: int(&e[3]) as usize,
        len: int(&e[4]) as i32,
        route_type: [RouteType::NoRoute, RouteType::LRoute, RouteType::ZRoute, RouteType::MazeRoute][int(&e[5]) as usize],
        routelen: int(&e[6]) as i32,
        grids: e[7].as_array().expect("g").iter().map(|p| Point3D { x: int(&p[0]) as i16, y: int(&p[1]) as i16, layer: int(&p[2]) as i16 }).collect(),
    }).collect();
    Tree3D { num_terminals: terms, num_layers: layers, pin_layers: pinl.to_vec(), nodes, edges }
}

/// Flatten the per-row grid captures into the pass's dense arrays.
fn grid3(side: &Value, key: &str, layers: usize, rows: usize) -> (Vec<i32>, Vec<i32>) {
    let (mut u, mut c) = (Vec::new(), Vec::new());
    for l in 0..layers {
        for y in 0..rows {
            for p in side[key][format!("{l},{y}")].as_array().expect("row") {
                u.push(int(&p[0]) as i32);
                c.push(int(&p[1]) as i32);
            }
        }
    }
    (u, c)
}

fn grid2(side: &Value, key: &str, rows: usize) -> Vec<i32> {
    (0..rows).flat_map(|y| side[key][y.to_string()].as_array().expect("row").iter().map(|v| int(v) as i32).collect::<Vec<_>>()).collect()
}

fn replay(g: &Value) -> usize {
    let calls = g["calls"].as_array().expect("calls");
    for c in calls {
        let who = format!("{} ub={}", c["design"], c["ub"]);
        let (layers, xg, yg) = (int(&c["layers"]) as usize, int(&c["xg"]) as usize, int(&c["yg"]) as usize);
        let (h3u, h3c) = grid3(&c["in"], "H3", layers, yg);
        let (v3u, v3c) = grid3(&c["in"], "V3", layers, yg - 1);
        let mut grid = PassGrid {
            layers, x_grid: xg, y_grid: yg,
            horizontal: c["dirs"].as_array().expect("dirs").iter().map(|v| int(v) == 1).collect(),
            h3_usage: h3u, h3_cap: h3c, v3_usage: v3u, v3_cap: v3c,
            h2_usage: grid2(&c["in"], "H2", yg), v2_usage: grid2(&c["in"], "V2", yg - 1),
            corr: HashMap::new(),
            log_2d: None,
            current_net: 0,
        };
        let net_ids: Vec<i64> = c["nets"].as_array().expect("nets").iter().map(|n| int(&n["id"])).collect();
        let mut nets: Vec<PassNet> = c["nets"].as_array().expect("nets").iter().map(|n| {
            let pinl: Vec<i16> = n["pinl"].as_array().expect("pinl").iter().map(|v| int(v) as i16).collect();
            let cost = n["cost"].as_array().expect("cost");
            let price = |p: &Value| { let v = p.as_f64().expect("f"); (v >= 0.0).then_some(v as f32) };
            PassNet {
                tree: tree(&n["tree"], int(&n["terms"]) as usize, layers as i16, &pinl),
                min_layer: int(&n["minl"]) as i32,
                max_layer: int(&n["maxl"]) as i32,
                edge_cost: int(&n["ec"]) as i8,
                layer_cost: cost.iter().map(|p| int(&p[0]) as i8).collect(),
                slack: n["slack"].as_f64().expect("slack") as f32,
                res_aware: int(&n["resaware"]) == 1,
                effective_resistance_aware: int(&n["effra"]) == 1,
                wire: cost.iter().map(|p| p[1].as_f64().expect("w") as f32).collect(),
                down: cost.iter().map(|p| price(&p[2])).collect(),
                up: cost.iter().map(|p| price(&p[3])).collect(),
            }
        }).collect();
        let params = PassParams {
            call: Maze3DCall {
                ripup_lb: int(&c["lb"]) as i32,
                ripup_ub: int(&c["ub"]) as i32,
                resistance_aware: int(&c["ra"]) == 1,
                incremental: int(&c["incr"]) == 1,
            },
            expand: int(&c["expand"]) as i32,
            detour_penalty: int(&c["detour"]) as i32,
        };
        let recovered = maze_route_3d_pass(params, &mut grid, &mut nets);
        assert_eq!(recovered as i64, int(&c["recovered"]), "{who}: recovered nets");

        let (h3u, _) = grid3(&c["out"], "H3", layers, yg);
        let (v3u, _) = grid3(&c["out"], "V3", layers, yg - 1);
        assert_eq!(grid.h3_usage, h3u, "{who}: 3D horizontal usage");
        assert_eq!(grid.v3_usage, v3u, "{who}: 3D vertical usage");
        assert_eq!(grid.h2_usage, grid2(&c["out"], "H2", yg), "{who}: 2D horizontal usage");
        assert_eq!(grid.v2_usage, grid2(&c["out"], "V2", yg - 1), "{who}: 2D vertical usage");
        for (net, id) in nets.iter().zip(&net_ids) {
            let want = tree(&c["after"][id.to_string()]["tree"], net.tree.num_terminals, layers as i16, &net.tree.pin_layers);
            assert_eq!(net.tree.edges, want.edges, "{who}: net {id} edges");
            assert_eq!(net.tree.nodes.len(), want.nodes.len(), "{who}: net {id} node count");
            for (i, (a, b)) in net.tree.nodes.iter().zip(&want.nodes).enumerate() {
                let k = b.conn.con_cnt as usize;
                assert_eq!(
                    (a.x, a.y, a.stack_alias, a.assigned, a.status, &a.nbr, a.conn.con_cnt, a.conn.bot_layer, a.conn.top_layer, a.conn.l_id, a.conn.h_id),
                    (b.x, b.y, b.stack_alias, b.assigned, b.status, &b.nbr, b.conn.con_cnt, b.conn.bot_layer, b.conn.top_layer, b.conn.l_id, b.conn.h_id),
                    "{who}: net {id} node {i}"
                );
                assert_eq!(&a.conn.e_id[..k], &b.conn.e_id[..k], "{who}: net {id} node {i} edge list");
                assert_eq!(&a.conn.heights[..k], &b.conn.heights[..k], "{who}: net {id} node {i} heights");
            }
        }
    }
    calls.len()
}

/// Every captured whole-pass call: every net's tree and both usage grids after.
#[test]
fn the_whole_3d_pass_matches_the_reference() {
    let g = read(&format!("{}/examples/grt_gate/pass3d.json", env!("CARGO_MANIFEST_DIR")));
    let n = replay(&g);
    assert!(n >= 4, "too few calls: {n}");
    // ⛔ Not vacuous: the kept calls must CHANGE nets — in both cost modes — or the replay above
    // would pass with a pass that does nothing. (The first selection changed nothing at all.)
    let calls = g["calls"].as_array().expect("calls");
    let changed: i64 = calls.iter().map(|c| int(&c["changed"])).sum();
    assert!(changed >= 20, "the golden's calls change too few nets: {changed}");
    assert!(calls.iter().any(|c| int(&c["ra"]) == 1 && int(&c["changed"]) > 0), "no resistance-aware call changes anything");
}

/// GRT_PASS3D_FULL=/path/to/i-all.json cargo test --release --test pass3d -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_whole_3d_pass_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_PASS3D_FULL").expect("set GRT_PASS3D_FULL");
    eprintln!("exhaustive: {} whole-pass calls", replay(&read(&path)));
}

/// ⛔ Only a search that ran dry counts for GRT-183; the zero-distance recovery does not. The
/// corpus's two zero-distance recoveries live in `overlapping_edges`, too big for the committed
/// golden — the exhaustive replay witnesses them; this pins the rule in CI.
#[test]
fn only_an_underflow_counts_for_grt183() {
    assert!(counts_for_grt183(Recovery::Underflow));
    assert!(!counts_for_grt183(Recovery::ZeroDistance));
}
