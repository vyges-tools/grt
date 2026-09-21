// SPDX-License-Identifier: Apache-2.0
//! R18b — `newRipup3DType3`, the 3D rip-up.
//!
//! Golden `ripup3d.json`, both cost modes: per rip-up the route, both alias nodes before and
//! after, and every planar step's 2D and 3D usage before and after.

use std::collections::HashMap;

use serde_json::Value;
use vyges_grt::softndr::UsageGrid;
use vyges_grt::{new_ripup_3d_type3, remove_edge_from_node, NodeConnections, Point3D};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn golden() -> Value {
    read(&format!("{}/examples/grt_gate/ripup3d.json", env!("CARGO_MANIFEST_DIR")))
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

const BIG_INT: i32 = 1_000_000_000;

fn node(v: &Value) -> NodeConnections {
    let mut n = NodeConnections {
        e_id: [0; 10],
        heights: [0; 10],
        con_cnt: int(&v["con"]) as i16,
        bot_layer: int(&v["bot"]) as i16,
        top_layer: int(&v["top"]) as i16,
        l_id: int(&v["lid"]) as i32,
        h_id: int(&v["hid"]) as i32,
    };
    for (i, p) in v["list"].as_array().expect("list").iter().enumerate() {
        n.e_id[i] = int(&p[0]) as i32;
        n.heights[i] = int(&p[1]) as i16;
    }
    n
}

/// Compare everything the reference exposes: the count, the range, the ids, and the live prefix.
fn assert_node(got: &NodeConnections, want: &Value, who: &str) {
    let w = node(want);
    let n = w.con_cnt as usize;
    assert_eq!(
        (got.con_cnt, got.bot_layer, got.top_layer, got.l_id, got.h_id),
        (w.con_cnt, w.bot_layer, w.top_layer, w.l_id, w.h_id),
        "{who}: count / range / ids"
    );
    assert_eq!(&got.e_id[..n], &w.e_id[..n], "{who}: edge list");
    assert_eq!(&got.heights[..n], &w.heights[..n], "{who}: heights");
}

/// Records every requested usage change; the replay compares them to what the reference applied.
#[derive(Default)]
struct Recorder(Vec<(char, i16, i16, i16, i32, bool)>);

impl UsageGrid for Recorder {
    fn add_usage_v_2d(&mut self, x: i16, y: i16, d: i32) {
        self.0.push(('V', -1, x, y, d, false));
    }
    fn add_usage_h_2d(&mut self, x: i16, y: i16, d: i32) {
        self.0.push(('H', -1, x, y, d, false));
    }
    fn add_usage_v_3d(&mut self, l: i16, x: i16, y: i16, d: i32) {
        self.0.push(('V', l, x, y, d, true));
    }
    fn add_usage_h_3d(&mut self, l: i16, x: i16, y: i16, d: i32) {
        self.0.push(('H', l, x, y, d, true));
    }
}

fn replay(g: &Value) -> usize {
    let records = g["records"].as_array().expect("records");
    for r in records {
        let who = format!("{} edge {}", r["design"], r["edge"]);
        let (n1a, n2a) = (int(&r["n1a"]) as usize, int(&r["n2a"]) as usize);
        // A sparse node table holding just the two aliases.
        let size = n1a.max(n2a) + 1;
        let mut nodes = vec![node(&r["B1"]); size];
        nodes[n1a] = node(&r["B1"]);
        nodes[n2a] = node(&r["B2"]);
        let pins: HashMap<usize, i32> = [(&r["B1"], n1a), (&r["B2"], n2a)]
            .iter()
            .filter(|(v, _)| int(&v["pin"]) >= 0)
            .map(|(v, id)| (*id, int(&v["pin"]) as i32))
            .collect();
        let grids: Vec<Point3D> = r["grids"].as_array().expect("grids").iter().map(|p| Point3D {
            x: int(&p[0]) as i16,
            y: int(&p[1]) as i16,
            layer: int(&p[2]) as i16,
        }).collect();
        let steps = r["steps"].as_array().expect("steps");
        // The layer cost per layer, from the steps that used it.
        let lc: HashMap<i16, i8> = steps.iter().map(|s| (int(&s[1]) as i16, int(&s[4]) as i8)).collect();
        let mut rec = Recorder::default();
        let ok = new_ripup_3d_type3(
            int(&r["edge"]) as usize,
            1, // every captured call is a real rip-up (0 refusals)
            (n1a, n2a),
            &grids,
            int(&r["routelen"]) as i32,
            &mut nodes,
            &|id| pins.get(&id).copied(),
            int(&r["ec"]) as i8,
            &|l| lc.get(&l).copied().expect("a layer the reference charged"),
            &mut rec,
        );
        assert_eq!(ok, Ok(true), "{who}");
        assert_node(&nodes[n1a], &r["A1"], &format!("{who} n1a"));
        assert_node(&nodes[n2a], &r["A2"], &format!("{who} n2a"));

        // Two requests per planar step (2D then 3D), matching the reference's APPLIED deltas.
        assert_eq!(rec.0.len(), 2 * steps.len(), "{who}: planar steps");
        for (k, s) in steps.iter().enumerate() {
            let dir = s[0].as_str().expect("dir").chars().next().expect("dir");
            let (l, x, y) = (int(&s[1]) as i16, int(&s[2]) as i16, int(&s[3]) as i16);
            let applied_2d = (int(&s[6]) - int(&s[5])) as i32;
            let applied_3d = (int(&s[8]) - int(&s[7])) as i32;
            assert_eq!(rec.0[2 * k], (dir, -1, x, y, applied_2d, false), "{who}: step {k} 2D");
            assert_eq!(rec.0[2 * k + 1], (dir, l, x, y, applied_3d, true), "{who}: step {k} 3D");
        }
    }
    records.len()
}

/// Every captured rip-up: both nodes after, and every usage change as the reference applied it.
///
/// ⚠️ This also checks the NDR-aware 2D update is INERT on the corpus: the request is the edge
/// cost, and it must equal what the reference applied — 304 NDR rip-ups included.
#[test]
fn ripup_3d_matches_the_reference() {
    let n = replay(&golden());
    assert!(n >= 300, "too few rip-ups: {n}");
}

/// GRT_RIPUP3D_FULL=/path/to/m3b-all.json cargo test --test ripup3d -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn ripup_3d_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_RIPUP3D_FULL").expect("set GRT_RIPUP3D_FULL");
    eprintln!("exhaustive: {} rip-ups", replay(&read(&path)));
}

/// ⚠️ The refusal is unreachable from R18's driver — its window already excludes `len <= 0`.
#[test]
fn no_captured_call_is_refused() {
    let g = golden();
    assert_eq!(int(&g["refusals"]), 0);
    assert!(int(&g["calls"]) >= 70_000);
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn conn(list: &[(i32, i16)]) -> NodeConnections {
    let mut n = NodeConnections {
        e_id: [0; 10],
        heights: [0; 10],
        con_cnt: list.len() as i16,
        bot_layer: 0,
        top_layer: 0,
        l_id: 0,
        h_id: 0,
    };
    for (i, &(e, h)) in list.iter().enumerate() {
        n.e_id[i] = e;
        n.heights[i] = h;
    }
    n
}

/// ⛔ The edge is not in the list: the count STILL drops, and every entry — the last included —
/// was considered first. Never captured.
#[test]
fn removing_an_absent_edge_still_drops_the_last_entry() {
    let mut n = conn(&[(1, 2), (2, 5)]);
    remove_edge_from_node(&mut n, 9, None);
    assert_eq!(n.con_cnt, 1);
    assert_eq!((n.bot_layer, n.l_id, n.top_layer, n.h_id), (2, 1, 5, 2));
}

/// ⚠️ A Steiner node left with nothing becomes (-1, 0) with both ids unset — not the (layers, -1)
/// "no layers" pair. Never captured: Steiner nodes are never emptied.
#[test]
fn an_emptied_steiner_node_is_minus_one_zero() {
    let mut n = conn(&[(4, 3)]);
    remove_edge_from_node(&mut n, 4, None);
    assert_eq!((n.con_cnt, n.bot_layer, n.top_layer, n.l_id, n.h_id), (0, -1, 0, BIG_INT, BIG_INT));
}

/// ⛔ Strict comparisons: a pin's edge AT the pin layer claims no id; a Steiner node whose edges
/// are all on layer 0 keeps no top id; on a tie the FIRST remaining edge keeps the id.
#[test]
fn strict_comparisons_decide_the_ids() {
    let mut pin = conn(&[(1, 3), (2, 3), (7, 9)]);
    remove_edge_from_node(&mut pin, 7, Some(3));
    assert_eq!((pin.bot_layer, pin.l_id, pin.top_layer, pin.h_id), (3, BIG_INT, 3, BIG_INT));

    let mut steiner = conn(&[(1, 0), (2, 0), (3, 4)]);
    remove_edge_from_node(&mut steiner, 3, None);
    assert_eq!((steiner.bot_layer, steiner.l_id, steiner.top_layer, steiner.h_id), (0, 1, 0, BIG_INT));

    let mut tie = conn(&[(5, 2), (6, 2), (8, 6), (9, 6)]);
    remove_edge_from_node(&mut tie, 1, None); // absent — every entry considered
    assert_eq!((tie.l_id, tie.h_id), (5, 8));
}

/// Vias give back nothing; a planar step gives back at its LOWER endpoint; a diagonal is fatal;
/// a zero-length edge is refused untouched.
#[test]
fn the_give_back_skips_vias_and_rejects_diagonals() {
    let p = |x, y, layer| Point3D { x, y, layer };
    let mut nodes = vec![conn(&[(0, 1)]), conn(&[(0, 1)])];
    let mut rec = Recorder::default();
    let grids = [p(2, 5, 1), p(2, 4, 1), p(2, 4, 2), p(1, 4, 2)];
    let ok = new_ripup_3d_type3(0, 3, (0, 1), &grids, 3, &mut nodes, &|_| None, 2, &|l| (l * 3) as i8, &mut rec);
    assert_eq!(ok, Ok(true));
    assert_eq!(
        rec.0,
        vec![('V', -1, 2, 4, -2, false), ('V', 1, 2, 4, -3, true), ('H', -1, 1, 4, -2, false), ('H', 2, 1, 4, -6, true)]
    );

    let mut nodes = vec![conn(&[(0, 1)]), conn(&[(0, 1)])];
    let bad = [p(0, 0, 1), p(1, 1, 1)];
    let err = new_ripup_3d_type3(0, 2, (0, 1), &bad, 1, &mut nodes, &|_| None, 1, &|_| 1, &mut Recorder::default());
    assert_eq!(err.map_err(|e| e.step), Err(0));

    let mut nodes = vec![conn(&[(0, 1)]), conn(&[(0, 1)])];
    let before = nodes.clone();
    let r = new_ripup_3d_type3(0, 0, (0, 1), &grids, 3, &mut nodes, &|_| None, 1, &|_| 1, &mut Recorder::default());
    assert_eq!((r, nodes), (Ok(false), before));
}
