// SPDX-License-Identifier: Apache-2.0
//! R18g — the 3D tree surgery's node shifts.
//!
//! Goldens, both cost modes: `shift3d_type1.json` (R18g1, `updateRouteType13D` + `copyGrids3D` —
//! three nodes, the new position, both edges before and after, the moved node) and
//! `shift3d_type2.json` (R18g2, `updateRouteType23D` — five nodes, three edges before and after,
//! each slot's whole grid vector INCLUDING its size).

use std::collections::HashMap;

use serde_json::Value;
use vyges_grt::{update_route_type1_3d, update_route_type2_3d, Point3D, RouteType, ShiftError, SurgeryEdge3D, SurgeryNode3D};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn route_type(t: i64) -> RouteType {
    [RouteType::NoRoute, RouteType::LRoute, RouteType::ZRoute, RouteType::MazeRoute][t as usize]
}

fn edge(v: &Value) -> SurgeryEdge3D {
    SurgeryEdge3D {
        n1: int(&v["n1"]) as usize,
        n2: int(&v["n2"]) as usize,
        route_type: route_type(int(&v["type"])),
        routelen: int(&v["rl"]) as i32,
        len: int(&v["len"]) as i32,
        grids: v["grids"].as_array().expect("grids").iter().map(|p| Point3D {
            x: int(&p[0]) as i16,
            y: int(&p[1]) as i16,
            layer: int(&p[2]) as i16,
        }).collect(),
    }
}

fn replay_type1(g: &Value) -> usize {
    let records = g["records"].as_array().expect("records");
    for r in records {
        let who = format!("{} {:?}", r["design"], r["e1"]);
        let ids: Vec<usize> = r["nodes"].as_array().expect("nodes").iter().map(|n| int(&n[0]) as usize).collect();
        let size = ids.iter().max().expect("nodes") + 1;
        let mut nodes = vec![SurgeryNode3D { x: -1, y: -1, bot_layer: -1 }; size];
        for n in r["nodes"].as_array().expect("nodes") {
            nodes[int(&n[0]) as usize] = SurgeryNode3D { x: int(&n[1]) as i16, y: int(&n[2]) as i16, bot_layer: int(&n[3]) as i16 };
        }
        let (e1, e2) = (int(&r["B1"]["id"]) as usize, int(&r["B2"]["id"]) as usize);
        let mut edges: HashMap<usize, SurgeryEdge3D> = HashMap::new();
        edges.insert(e1, edge(&r["B1"]));
        edges.insert(e2, edge(&r["B2"]));
        let esize = e1.max(e2) + 1;
        let mut dense: Vec<SurgeryEdge3D> = (0..esize).map(|i| edges.get(&i).cloned().unwrap_or(SurgeryEdge3D {
            n1: 0, n2: 0, route_type: RouteType::NoRoute, routelen: 0, len: 0, grids: vec![],
        })).collect();
        let e1pos = (int(&r["e1"][0]) as i16, int(&r["e1"][1]) as i16);
        update_route_type1_3d(&mut nodes, ids[0], ids[1], ids[2], e1pos, &mut dense, e1, e2)
            .unwrap_or_else(|e| panic!("{who}: {e:?}"));
        for (id, key) in [(e1, "A1"), (e2, "A2")] {
            let want = edge(&r[key]);
            let got = &dense[id];
            assert_eq!(
                (got.n1, got.n2, got.route_type, got.routelen, got.len),
                (want.n1, want.n2, want.route_type, want.routelen, want.len),
                "{who}: edge {id} header"
            );
            assert_eq!(got.grids, want.grids, "{who}: edge {id} grids");
        }
        assert_eq!(
            (i64::from(nodes[ids[0]].x), i64::from(nodes[ids[0]].y)),
            (int(&r["n1pos"][0]), int(&r["n1pos"][1])),
            "{who}: moved node"
        );
    }
    records.len()
}

/// Every captured type-1 shift: both edges after (endpoints, type, length, points) and the node.
#[test]
fn type1_shift_matches_the_reference() {
    let g = read(&format!("{}/examples/grt_gate/shift3d_type1.json", env!("CARGO_MANIFEST_DIR")));
    let n = replay_type1(&g);
    assert!(n >= 300, "too few calls: {n}");
}

/// GRT_SHIFT3D_TYPE1_FULL=/path/to/g1-all.json cargo test --test shift3d -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn type1_shift_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_SHIFT3D_TYPE1_FULL").expect("set GRT_SHIFT3D_TYPE1_FULL");
    eprintln!("exhaustive: {} calls", replay_type1(&read(&path)));
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn p(x: i16, y: i16, layer: i16) -> Point3D {
    Point3D { x, y, layer }
}

fn maze(n1: usize, n2: usize, grids: Vec<Point3D>) -> SurgeryEdge3D {
    SurgeryEdge3D { n1, n2, route_type: RouteType::MazeRoute, routelen: grids.len() as i32 - 1, len: 0, grids }
}

/// ⛔ With a via stack at the new position, the near half ends at the FIRST point there and the
/// far half starts at the LAST; ⛔ a via fill joins the far half to the other edge's layer.
#[test]
fn a_via_stack_at_e1_splits_first_and_last_and_the_join_is_filled() {
    // nodes: n1 = 0 at (3,0), A1 = 1 at (0,0), A2 = 2 at (5,0).
    let mut nodes = vec![
        SurgeryNode3D { x: 3, y: 0, bot_layer: 1 },
        SurgeryNode3D { x: 0, y: 0, bot_layer: 1 },
        SurgeryNode3D { x: 5, y: 0, bot_layer: 1 },
    ];
    // (A1 -> n1): along layer 1 to x=2, via up to 2 at x=2, then to x=3 on layer 2.
    let a1n1 = maze(1, 0, vec![p(0, 0, 1), p(1, 0, 1), p(2, 0, 1), p(2, 0, 2), p(3, 0, 2)]);
    // (n1 -> A2): leaves n1 on layer 4.
    let n1a2 = maze(0, 2, vec![p(3, 0, 4), p(4, 0, 4), p(5, 0, 4)]);
    let mut edges = vec![a1n1, n1a2];
    update_route_type1_3d(&mut nodes, 0, 1, 2, (2, 0), &mut edges, 0, 1).expect("ok");
    // Near half stops at the FIRST (2,0): layer 1.
    assert_eq!(edges[0].grids, vec![p(0, 0, 1), p(1, 0, 1), p(2, 0, 1)]);
    // Far half from the LAST (2,0), then vias 3..=4 at n1's old cell, then the rest of (n1, A2).
    assert_eq!(
        edges[1].grids,
        vec![p(2, 0, 2), p(3, 0, 2), p(3, 0, 3), p(3, 0, 4), p(4, 0, 4), p(5, 0, 4)]
    );
    assert_eq!((edges[1].n1, edges[1].n2, edges[1].routelen, edges[1].len), (0, 2, 5, 3));
    assert_eq!((nodes[0].x, nodes[0].y), (2, 0));
}

/// ⛔ A stepless second edge contributes only n1's point at its BOTTOM layer — and no fill.
#[test]
fn a_stepless_edge_is_one_point_at_the_bottom_layer() {
    let mut nodes = vec![
        SurgeryNode3D { x: 3, y: 0, bot_layer: 6 },
        SurgeryNode3D { x: 0, y: 0, bot_layer: 1 },
        SurgeryNode3D { x: 3, y: 0, bot_layer: 1 },
    ];
    let a1n1 = maze(1, 0, vec![p(0, 0, 1), p(1, 0, 1), p(2, 0, 1), p(3, 0, 1)]);
    let stepless = SurgeryEdge3D { n1: 0, n2: 2, route_type: RouteType::MazeRoute, routelen: 0, len: 0, grids: vec![] };
    let mut edges = vec![a1n1, stepless];
    update_route_type1_3d(&mut nodes, 0, 1, 2, (1, 0), &mut edges, 0, 1).expect("ok");
    // Far half = from (1,0) to the end of the first list; the stepless edge adds nothing past its
    // single point, and no via fill (the fill needs a second list longer than one point).
    assert_eq!(edges[1].grids, vec![p(1, 0, 1), p(2, 0, 1), p(3, 0, 1)]);
}

/// Fatal cases: a one-point first list; a new position not on the edge.
#[test]
fn a_type1_shift_rejects_a_single_point_edge_and_an_off_edge_position() {
    let mut nodes = vec![SurgeryNode3D { x: 3, y: 0, bot_layer: 1 }; 3];
    let stepless = SurgeryEdge3D { n1: 1, n2: 0, route_type: RouteType::NoRoute, routelen: 0, len: 0, grids: vec![] };
    let mut edges = vec![stepless.clone(), stepless];
    assert_eq!(update_route_type1_3d(&mut nodes, 0, 1, 2, (3, 0), &mut edges, 0, 1), Err(ShiftError::SinglePointEdge));
    let mut edges = vec![maze(1, 0, vec![p(0, 0, 1), p(1, 0, 1)]), maze(0, 2, vec![p(1, 0, 1), p(2, 0, 1)])];
    assert_eq!(update_route_type1_3d(&mut nodes, 0, 1, 2, (9, 9), &mut edges, 0, 1), Err(ShiftError::NotOnEdge));
}

// ─── R18g2 — type 2 ─────────────────────────────────────────────────────────────────────────

fn replay_type2(g: &Value) -> usize {
    let records = g["records"].as_array().expect("records");
    for r in records {
        let who = format!("{} {:?}", r["design"], r["e1"]);
        let ids: Vec<usize> = r["nodes"].as_array().expect("nodes").iter().map(|n| int(&n[0]) as usize).collect();
        let mut nodes = vec![SurgeryNode3D { x: -1, y: -1, bot_layer: -1 }; ids.iter().max().expect("n") + 1];
        for n in r["nodes"].as_array().expect("nodes") {
            nodes[int(&n[0]) as usize] = SurgeryNode3D { x: int(&n[1]) as i16, y: int(&n[2]) as i16, bot_layer: int(&n[3]) as i16 };
        }
        let slots: Vec<usize> = ["B1", "B2", "B3"].iter().map(|k| int(&r[k]["id"]) as usize).collect();
        let mut edges: Vec<SurgeryEdge3D> = (0..=*slots.iter().max().expect("s")).map(|_| SurgeryEdge3D {
            n1: 0, n2: 0, route_type: RouteType::NoRoute, routelen: 0, len: 0, grids: vec![],
        }).collect();
        for (k, &id) in ["B1", "B2", "B3"].iter().zip(&slots) {
            edges[id] = edge(&r[k]);
        }
        let e1 = (int(&r["e1"][0]) as i16, int(&r["e1"][1]) as i16);
        update_route_type2_3d(&nodes, ids[0], (ids[1], ids[2]), (ids[3], ids[4]), e1, &mut edges, (slots[0], slots[1], slots[2]))
            .unwrap_or_else(|e| panic!("{who}: {e:?}"));
        for (k, &id) in ["A1", "A2", "A3"].iter().zip(&slots) {
            let want = edge(&r[k]);
            let got = &edges[id];
            assert_eq!(
                (got.n1, got.n2, got.route_type, got.routelen, got.len),
                (want.n1, want.n2, want.route_type, want.routelen, want.len),
                "{who}: slot {id} header"
            );
            assert_eq!(got.grids, want.grids, "{who}: slot {id} vector (size and points)");
        }
    }
    records.len()
}

/// Every captured type-2 shift: all three slots after, each slot's whole vector.
#[test]
fn type2_shift_matches_the_reference() {
    let g = read(&format!("{}/examples/grt_gate/shift3d_type2.json", env!("CARGO_MANIFEST_DIR")));
    let n = replay_type2(&g);
    assert!(n >= 150, "too few calls: {n}");
    // ⛔ TRIPWIRE: the via-fill bound disagreement (finding 12) never occurred — 0 of 1,879.
    assert_eq!(int(&g["fill_mismatch"]), 0);
}

/// GRT_SHIFT3D_TYPE2_FULL=/path/to/g2-all.json cargo test --test shift3d -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn type2_shift_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_SHIFT3D_TYPE2_FULL").expect("set GRT_SHIFT3D_TYPE2_FULL");
    eprintln!("exhaustive: {} calls", replay_type2(&read(&path)));
}

/// Nodes and slots for a type-2 case: n1 = 0 at (2,0); A1 = 1 at (0,0); A2 = 2 at (4,0);
/// C1 = 3 at (2,3); C2 = 4 at (2,6). Slots: 0 = (A1, n1), 1 = (n1, A2), 2 = (C1, C2).
fn type2_case(n1a2_first_layers: (i16, i16), a1n1_last_layer: i16) -> (Vec<SurgeryNode3D>, Vec<SurgeryEdge3D>) {
    let nodes = vec![
        SurgeryNode3D { x: 2, y: 0, bot_layer: 1 },
        SurgeryNode3D { x: 0, y: 0, bot_layer: 1 },
        SurgeryNode3D { x: 4, y: 0, bot_layer: 1 },
        SurgeryNode3D { x: 2, y: 3, bot_layer: 2 },
        SurgeryNode3D { x: 2, y: 6, bot_layer: 2 },
    ];
    let (f0, f1) = n1a2_first_layers;
    let edges = vec![
        maze(1, 0, vec![p(0, 0, a1n1_last_layer), p(1, 0, a1n1_last_layer), p(2, 0, a1n1_last_layer)]),
        maze(0, 2, vec![p(2, 0, f0), p(2, 0, f1), p(3, 0, f1), p(4, 0, f1)]),
        maze(3, 4, vec![p(2, 3, 2), p(2, 4, 2), p(2, 5, 2), p(2, 6, 2)]),
    ];
    (nodes, edges)
}

/// The merge joins the two lists with a via fill, and the landed-on edge splits at E1.
#[test]
fn a_type2_shift_merges_and_splits() {
    let (nodes, mut edges) = type2_case((3, 3), 1);
    update_route_type2_3d(&nodes, 0, (1, 2), (3, 4), (2, 4), &mut edges, (0, 1, 2)).expect("ok");
    // (A1,A2) in slot 2: A1's list, vias 2..=3 at n1, then (n1,A2) past its first point.
    assert_eq!(
        edges[2].grids,
        vec![p(0, 0, 1), p(1, 0, 1), p(2, 0, 1), p(2, 0, 2), p(2, 0, 3), p(2, 0, 3), p(3, 0, 3), p(4, 0, 3)]
    );
    assert_eq!((edges[2].routelen, edges[2].len), (7, 4));
    assert_eq!(edges[0].grids, vec![p(2, 3, 2), p(2, 4, 2)]);
    assert_eq!(edges[1].grids, vec![p(2, 4, 2), p(2, 5, 2), p(2, 6, 2)]);
    // No orientation, no endpoint writes, no type change on any slot.
    assert_eq!((edges[0].n1, edges[0].n2), (1, 0));
}

/// ⛔ Finding 12, the "too few" side: descending, the fill is SIZED from (n1,A2)'s first layer (3)
/// but LOOPS down only to its second point's layer (4): one fill point short, so the vector's last
/// slot keeps `resize`'s default (0,0,0). Never captured.
#[test]
fn the_descending_fill_bound_can_leave_a_default_point() {
    let (nodes, mut edges) = type2_case((3, 4), 6);
    update_route_type2_3d(&nodes, 0, (1, 2), (3, 4), (2, 4), &mut edges, (0, 1, 2)).expect("ok");
    let g = &edges[2].grids;
    assert_eq!(g.len(), 3 + 3 + 3, "sized for fill 5, 4, 3");
    assert_eq!(g[3..5], [p(2, 0, 5), p(2, 0, 4)], "fill stopped at layer 4");
    assert_eq!(g[g.len() - 1], p(0, 0, 0), "the unwritten tail keeps the default");
}

/// ⛔ Finding 12, the "too many" side: the loop runs past the size — undefined behaviour in the
/// reference, an error here. Never captured.
#[test]
fn the_descending_fill_bound_can_overrun() {
    let (nodes, mut edges) = type2_case((3, 1), 6);
    let r = update_route_type2_3d(&nodes, 0, (1, 2), (3, 4), (2, 4), &mut edges, (0, 1, 2));
    assert_eq!(r, Err(ShiftError::WriteBeyondEnd));
}

/// ⚠️ A single-point merge sets `routelen = 0` and resizes nothing: a maze-route slot ends EMPTY;
/// any other slot keeps its old points. (The merged slot was always a maze route when captured.)
#[test]
fn a_single_point_merge_resizes_nothing() {
    let nodes = vec![SurgeryNode3D { x: 2, y: 0, bot_layer: 1 }; 5];
    let stepless = |n1, n2| SurgeryEdge3D { n1, n2, route_type: RouteType::MazeRoute, routelen: 0, len: 0, grids: vec![] };
    let landed = maze(3, 4, vec![p(2, 0, 2), p(2, 1, 2)]);
    for (ty, want_len) in [(RouteType::MazeRoute, 0), (RouteType::LRoute, 2)] {
        let mut slot = landed.clone();
        slot.route_type = ty;
        let mut edges = vec![stepless(1, 0), stepless(0, 2), slot];
        update_route_type2_3d(&nodes, 0, (1, 2), (3, 4), (2, 0), &mut edges, (0, 1, 2)).expect("ok");
        assert_eq!((edges[2].routelen, edges[2].grids.len()), (0, want_len), "{ty:?}");
    }
}

/// ⛔ `copyGrids3D`'s stepless point — the node's cell at its BOTTOM layer, gated on `routelen`,
/// not the route type — is observable in exactly one place: type 2's split of a stepless landed-on
/// edge, which writes that point into both halves. Here (C1, C2) is maze-typed with no steps and a
/// stale point in its array; the halves must carry C1's cell at C1's bottom layer (5), not the
/// stale point. Never captured.
#[test]
fn a_stepless_landed_on_edge_splits_to_the_bottom_layer_point() {
    let nodes = vec![
        SurgeryNode3D { x: 2, y: 0, bot_layer: 1 }, // n1
        SurgeryNode3D { x: 0, y: 0, bot_layer: 1 }, // A1
        SurgeryNode3D { x: 4, y: 0, bot_layer: 1 }, // A2
        SurgeryNode3D { x: 7, y: 7, bot_layer: 5 }, // C1
        SurgeryNode3D { x: 7, y: 7, bot_layer: 5 }, // C2
    ];
    let stale = SurgeryEdge3D {
        n1: 3, n2: 4, route_type: RouteType::MazeRoute, routelen: 0, len: 0, grids: vec![p(9, 9, 9)],
    };
    let mut edges = vec![
        maze(1, 0, vec![p(0, 0, 1), p(1, 0, 1), p(2, 0, 1)]),
        maze(0, 2, vec![p(2, 0, 1), p(3, 0, 1), p(4, 0, 1)]),
        stale,
    ];
    update_route_type2_3d(&nodes, 0, (1, 2), (3, 4), (7, 7), &mut edges, (0, 1, 2)).expect("ok");
    assert_eq!(edges[0].grids, vec![p(7, 7, 5)]);
    assert_eq!(edges[1].grids, vec![p(7, 7, 5)]);
}
