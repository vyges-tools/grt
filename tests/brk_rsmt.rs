// SPDX-License-Identifier: Apache-2.0
//! R5 / R7 — `gen_brk_RSMT`, replayed call by call against the reference.
//!
//! Golden `brk_rsmt.json`: whole runs, every call in order. Per net the replay checks the builder
//! and its coefficient, the tree after edge shifting, the copied tree, the whole segment list and
//! each tree edge's route; per call, on small grids, the estimated usage at exit.
//!
//! The Steiner-tree engine's trees (alpha > 0) are INPUTS here — that engine is scored on its own.
//! The router's flute path calls the engine's pre-sorted FLUTE for real.
//!
//! ⛔ **Known divergence: the NDR OVERFLOW charge.** The reference's `updateEstUsage` goes through
//! `getCostNDRAware`, which grt does not implement yet. For an NDR net (edge cost > 1) with room on
//! its layers that cost is plain `±edgeCost` on R7's rip-up and re-route — so ordinary NDR runs
//! replay exactly. On an edge in NDR overflow it is `100 × edgeCost` (measured: usage 900 under a
//! cost-9 `clk`), and the plain rip-up leaves 891 behind. [`NDR_OVERFLOW`] names the runs that
//! reach it; they must diverge and every other run must match, so the list fails the day the NDR
//! cost lands.

use serde_json::Value;
use vyges_grt::*;

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i32 {
    v.as_i64().expect("integer") as i32
}

fn arr(v: &Value) -> &Vec<Value> {
    v.as_array().expect("array")
}

/// RLE rows of `[value..., count]` → one flat row of value tuples.
fn unrle(row: &Value) -> Vec<Vec<f64>> {
    arr(row)
        .iter()
        .flat_map(|p| {
            let p = arr(p);
            let vals: Vec<f64> = p[..p.len() - 1].iter().map(|v| v.as_f64().expect("number")).collect();
            std::iter::repeat(vals).take(int(&p[p.len() - 1]) as usize)
        })
        .collect()
}

/// A usage dump as `(est, red)` per edge, `[y][x]`.
fn usage(u: &Value, dir: &str) -> Vec<Vec<(f64, u16)>> {
    arr(&u[dir]).iter().map(|r| unrle(r).into_iter().map(|v| (v[0], v[1] as u16)).collect()).collect()
}

thread_local! {
    // ⚠️ Per thread: the tables hold `Rc`, so they cannot be shared across test threads.
    static LUT: vyges_stt::flute::lut::Lut =
        vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
}

fn flutes(xs: &[i32], ys: &[i32], s: &[usize], acc: i32) -> RsmtTree {
    let t = LUT.with(|lut| vyges_stt::flute::medium_degree::flutes_all_degree_acc(lut, xs.len(), xs, ys, s, acc)).expect("flute builds a tree");
    RsmtTree { deg: t.deg, length: t.length, branch: t.branch.iter().map(|b| Branch { x: b.x, y: b.y, n: b.n }).collect() }
}

fn tree_of(v: &Value) -> RsmtTree {
    RsmtTree {
        deg: int(&v["deg"]) as usize,
        length: v["length"].as_i64().expect("length"),
        branch: arr(&v["branch"]).iter().map(|b| Branch { x: int(&b[0]), y: int(&b[1]), n: int(&b[2]) as usize }).collect(),
    }
}

fn seg(v: &Value, cost: i8) -> Segment {
    Segment { x1: int(&v[0]), y1: int(&v[1]), x2: int(&v[2]), y2: int(&v[3]), edge_cost: cost }
}

#[derive(Default, Debug)]
struct Seen {
    runs: usize,
    calls: usize,
    nets: usize,
    flute_nets: usize,
    shifted: usize,
    shifts: i32,
    copied: usize,
    routed_edges: usize,
    usage_checked: usize,
    ndr_checked: usize,
    r6_checked: usize,
    r6_segs: usize,
    boundaries: usize,
    boundary_nets: usize,
    incremental: usize,
    gate_hvh: usize,
    gate_vhv: usize,
    maze_routes: usize,
    maze_passes: usize,
    usage_errors: usize,
    loop_iterations: usize,
    snapshot_batched: usize,
    loops_removed: usize,
    r15_checked: usize,
    r16_checked: usize,
    r16_widened: usize,
    r16_res_aware: usize,
    thinned: usize,
    loop_partial_slack: usize,
    loop_extra_stops: usize,
    stacked_pins: usize,
    htree: usize,
}

/// ⛔ Known limitation — INCREMENTAL passes. A pass that keeps other nets' routes
/// (`is_incremental_grt_`) rebuilds the used-grid sets from the COMMITTED usage at run() entry
/// (`rebuildUsedGrids`), which this replay does not carry: those runs match on the overflow outputs
/// and estimated usage at B7 and diverge on the COMMITTED usage — the other nets' routes — which is
/// what the used grids are rebuilt from. They must diverge exactly there, and every
/// other run must match — so this list fails the day the incremental setup lands.
const INCREMENTAL: [&str; 4] =
    ["repair_antennas_adjacent_jumpers-", "repair_antennas_allow_congestion-", "repair_antennas_from_odb-", "repair_antennas_only_diodes-"];

/// ⛔ Known limitation — the SNAPSHOT-BATCHED maze router. With a batch width configured and enough
/// nets, `mazeRouteMSMD` routes early iterations in parallel snapshot batches ("intentionally not
/// exact-preserving", per its own comment) instead of the sequential kernel this engine transcribes.
/// Those runs must diverge at their first loop iteration's result, and every other run must match.
const SNAPSHOT_BATCHED: [&str; 2] = ["snapshot_batched_smoke-", "snapshot_batched_single_thread_smoke-"];

/// Replay every run; list every run that diverges rather than stopping at the first. `bounds` is
/// the boundary golden: each run's router state after R7, R8, R9, R10, per `global_route`.
fn replay(g: &Value, bounds: &Value) -> Seen {
    let mut seen = Seen::default();
    let mut failed = Vec::new();
    let by_design: std::collections::HashMap<&str, &Value> =
        arr(&bounds["runs"]).iter().map(|r| (r["design"].as_str().expect("design"), r)).collect();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        let mut one = Seen::default();
        let b = by_design.get(who).copied();
        let (known, expect) = if INCREMENTAL.iter().any(|p| who.starts_with(p)) {
            (true, "B7: committed usage")
        } else if SNAPSHOT_BATCHED.iter().any(|p| who.starts_with(p)) {
            (true, "B14_1: getOverflow2D")
        } else {
            (false, "")
        };
        match (std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| replay_run(r, b, &mut one))), known) {
            (Ok(()), false) => seen = add(seen, one),
            (Err(e), true) => {
                let msg = e.downcast_ref::<String>().cloned().unwrap_or_default();
                if msg.contains(expect) {
                    if expect.starts_with("B7") { seen.incremental += 1 } else { seen.snapshot_batched += 1 }
                } else {
                    failed.push(format!("{who}: diverges, but not where expected ({expect}): {msg}"));
                }
            }
            (Ok(()), true) => failed.push(format!("{who}: now MATCHES where it was expected to diverge at {expect} — update the known list")),
            (Err(e), false) => failed.push(format!("{who}: {}", e.downcast_ref::<String>().cloned().unwrap_or_default())),
        }
    }
    assert!(failed.is_empty(), "{} runs diverge:\n{}", failed.len(), failed.join("\n"));
    seen
}

/// Compare our router state with one boundary: the overflow outputs, estimated usage, the
/// used-grid sets, and every routed net's node statuses and edge routes.
fn check_boundary(at: &str, b: &Value, scan: &Overflow2DScan, g2d: &Graph2d, state: &[NetState], seen: &mut Seen) {
    let tag = b["tag"].as_str().expect("tag");
    let at = format!("{at} {tag}");
    assert_eq!((scan.total_overflow, scan.max_overflow, scan.ahth), (int(&b["total_overflow"]), int(&b["max_overflow"]), int(&b["ahth"])),
               "{at}: getOverflow2D (total, max, ahth)");
    if tag.starts_with("B12_") || tag.starts_with("B14_") || tag == "B15" {
        assert_eq!(scan.total_usage, int(&b["t_usage"]), "{at}: getOverflow2Dmaze tUsage");
    }
    // ⚠️ A THINNED loop boundary carries only its parameters and the scalars above.
    if b["thinned"].as_bool() == Some(true) {
        seen.thinned += 1;
        return;
    }
    for (dir, rows) in [("H", &b["est"]["H"]), ("V", &b["est"]["V"])] {
        for (y, row) in arr(rows).iter().enumerate() {
            let flat: Vec<f64> = arr(row).iter().flat_map(|p| std::iter::repeat(p[0].as_f64().expect("f")).take(int(&p[1]) as usize)).collect();
            for (x, &want) in flat.iter().enumerate() {
                let ours = if dir == "H" { g2d.est.h(x, y) } else { g2d.est.v(x, y) };
                assert_eq!(ours, want, "{at}: estimated usage {dir} ({x}, {y})");
            }
        }
    }
    for (dir, rows) in [("H", &b["usage"]["H"]), ("V", &b["usage"]["V"])] {
        for (y, row) in arr(rows).iter().enumerate() {
            let flat: Vec<i32> = arr(row).iter().flat_map(|p| std::iter::repeat(int(&p[0])).take(int(&p[1]) as usize)).collect();
            for (x, &want) in flat.iter().enumerate() {
                let ours = if dir == "H" { g2d.est.usage_h(x, y) } else { g2d.est.usage_v(x, y) };
                assert_eq!(ours as i32, want, "{at}: committed usage {dir} ({x}, {y})");
            }
        }
    }
    // The congestion history, present from B13 on.
    for (key, what) in [("last", "last_usage"), ("cong", "congCNT")] {
        let Some(rows) = b.get(key) else { continue };
        for (dir, rows) in [("H", &rows["H"]), ("V", &rows["V"])] {
            for (y, row) in arr(rows).iter().enumerate() {
                let flat: Vec<i32> = arr(row).iter().flat_map(|p| std::iter::repeat(int(&p[0])).take(int(&p[1]) as usize)).collect();
                for (x, &want) in flat.iter().enumerate() {
                    let ours = match (key, dir) {
                        ("last", "H") => g2d.est.last_usage_h(x, y) as i32,
                        ("last", _) => g2d.est.last_usage_v(x, y) as i32,
                        (_, "H") => g2d.est.cong_cnt_h(x, y) as i32,
                        _ => g2d.est.cong_cnt_v(x, y) as i32,
                    };
                    assert_eq!(ours, want, "{at}: {what} {dir} ({x}, {y})");
                }
            }
        }
    }
    for (dir, ours) in [("H", &g2d.used_h), ("V", &g2d.used_v)] {
        let want: Vec<(i32, i32)> = arr(&b["used"][dir]).iter().map(|p| (int(&p[0]), int(&p[1]))).collect();
        assert_eq!(ours.iter().copied().collect::<Vec<_>>(), want, "{at}: used grids {dir}");
    }
    for (id, n) in b["nets"].as_object().expect("nets") {
        let id: usize = id.parse().expect("net id");
        let t = state[id].tree.as_ref().unwrap_or_else(|| panic!("{at}: net {id} has no tree"));
        let nodes: Vec<(i32, i32, i32)> = arr(&n["nodes"]).iter().map(|v| (int(&v[0]), int(&v[1]), int(&v[2]))).collect();
        let ours: Vec<(i32, i32, i32)> = t.nodes.iter().map(|v| (v.x as i32, v.y as i32, v.status as i32)).collect();
        assert_eq!(ours, nodes, "{at}: net {id} nodes (x, y, status)");
        let edges: Vec<[i32; 7]> = arr(&n["edges"]).iter().map(|e| std::array::from_fn(|k| int(&e[k]))).collect();
        let ours: Vec<[i32; 7]> = t.edges.iter().zip(&t.routes).map(|(e, r)| {
            [e.n1 as i32, e.n2 as i32, e.len, r.kind as i32, r.x_first as i32, r.hvh as i32, r.z_point as i32]
        }).collect();
        assert_eq!(ours, edges, "{at}: net {id} edges (n1, n2, len, type, xFirst, HVH, Zpoint)");
        seen.boundary_nets += 1;
        if let Some(topo) = b.get("topo").and_then(|t| t.get(id.to_string())) {
            for (i, v) in arr(topo).iter().enumerate() {
                let cnt = int(&v[0]) as usize;
                let want: Vec<(usize, usize)> = (0..cnt).map(|k| (int(&v[1 + k]) as usize, int(&v[4 + k]) as usize)).collect();
                let ours: Vec<(usize, usize)> = (0..t.nbr_count[i]).map(|k| (t.nbr[i][k], t.edge[i][k])).collect();
                assert_eq!(ours, want, "{at}: net {id} node {i} neighbours (count {cnt})");
            }
        }
        if let Some(ns) = b.get("netstate").and_then(|t| t.get(id.to_string())) {
            assert_eq!((state[id].slack.to_bits(), state[id].critical), (int(&ns["slack_bits"]) as u32, ns["critical"].as_bool().expect("critical")),
                       "{at}: net {id} (slack bits, critical)");
        }
        for (eid, r) in t.routes.iter().enumerate().filter(|(_, r)| r.kind == RouteKind::MazeRoute) {
            let g = &b["grids"][format!("{id},{eid}")];
            assert!(!g.is_null(), "{at}: net {id} edge {eid}: a maze route the reference does not have");
            let pts: Vec<(i32, i32)> = arr(&g["points"]).iter().map(|p| (int(&p[0]), int(&p[1]))).collect();
            assert_eq!((r.routelen, r.last_routelen, &r.grids[..=r.routelen as usize]), (int(&g["routelen"]), int(&g["last"]), &pts[..]),
                       "{at}: net {id} edge {eid} maze route (routelen, last_routelen, grids)");
            seen.maze_routes += 1;
        }
        if tag == "B10" && t.num_terminals > 2 {
            // A Z route on a net of more than two terminals came through the congestion gate.
            for r in t.routes.iter().filter(|r| r.kind == RouteKind::ZRoute) {
                if r.hvh { seen.gate_hvh += 1 } else { seen.gate_vhv += 1 }
            }
        }
    }
    seen.boundaries += 1;
}

/// The first edge where a grid differs from a usage dump, if any.
fn usage_diff(est: &EstimateGrid, u: &Value) -> Option<String> {
    for (dir, rows) in [("H", usage(u, "H")), ("V", usage(u, "V"))] {
        for (y, row) in rows.iter().enumerate() {
            for (x, &(e, _)) in row.iter().enumerate() {
                let ours = if dir == "H" { est.h(x, y) } else { est.v(x, y) };
                if ours != e {
                    return Some(format!("{dir} ({x}, {y}): engine {ours}, reference {e}"));
                }
            }
        }
    }
    None
}

fn replay_run(r: &Value, bounds: Option<&Value>, seen: &mut Seen) {
    {
        let who = r["design"].as_str().expect("design");
        let calls = arr(&r["calls"]);
        let ndr = calls.iter().flat_map(|c| arr(&c["nets"])).any(|n| int(&n["cost"]) != 1);
        let max_id = calls.iter().flat_map(|c| arr(&c["nets"])).map(|n| int(&n["id"]) as usize).max().unwrap_or(0);
        let mut state: Vec<NetState> = vec![NetState::default(); max_id + 1];
        let max_layer = calls.iter().flat_map(|c| arr(&c["nets"])).map(|n| int(&n["max"]) as usize).max().unwrap_or(0);
        // R6's state carried into R7: our estimated usage and NDR ledger, not the dump's.
        let mut chain: Option<Graph2d> = None;
        // Each `global_route` contributes one group of boundaries; R7 calls pair with them in order.
        let mut pass = 0usize;
        for (ci, c) in calls.iter().enumerate() {
            let f = arr(&c["flags"]);
            let flags = BrkFlags { congestion_driven: int(&f[0]) == 1, re_route: int(&f[1]) == 1, gen_tree: int(&f[2]) == 1, no_adj: int(&f[4]) == 1 };
            assert_eq!(int(&f[3]), 0, "{who}: newType is false at both call sites");
            let (xg, yg) = (int(&c["xg"]) as usize, int(&c["yg"]) as usize);
            let small = c["small"].as_bool().expect("small");
            let at = format!("{who} call {ci}");

            // The grid at entry: usage and reductions on small grids, zero otherwise.
            let mut est = EstimateGrid::new(xg, yg);
            let (mut red_h, mut red_v) = (vec![0u16; xg * yg], vec![0u16; xg * yg]);
            let mut caps = Caps3D { x_grid: xg, layers: Vec::new() };
            if small {
                for (y, row) in usage(&c["entry"], "H").iter().enumerate() {
                    for (x, &(e, red)) in row.iter().enumerate() {
                        est.update_h(x as i32, x as i32 + 1, y as i32, e);
                        red_h[y * xg + x] = red;
                    }
                }
                for (y, row) in usage(&c["entry"], "V").iter().enumerate() {
                    for (x, &(e, red)) in row.iter().enumerate() {
                        est.update_v(x as i32, y as i32, y as i32 + 1, e);
                        red_v[y * xg + x] = red;
                    }
                }
                let layer_rows = |d: &str| -> Vec<Vec<i32>> {
                    arr(&c["caps"][d]).iter().map(|l| arr(l).iter().flat_map(|row| unrle(row).into_iter().map(|v| v[0] as i32)).collect()).collect()
                };
                caps.layers = layer_rows("H").into_iter().zip(layer_rows("V")).map(|(h, v)| CapLayer { h, v }).collect();
            }
            // The NDR ledger as `initEdgesCapacityPerLayer` leaves it: every layer's capacity from the
            // 3D edges (horizontal edges to x < xg-1, vertical to y < yg-1), no NDR net anywhere.
            let mut ledger = NdrLedger::new(xg, yg, caps.layers.len().max(max_layer + 1));
            for (l, cl) in caps.layers.iter().enumerate() {
                for y in 0..yg {
                    for x in 0..xg {
                        if x + 1 < xg {
                            ledger.update_cap_3d(x, y, l, true, cl.h[y * xg + x] as f64);
                        }
                        if y + 1 < yg {
                            ledger.update_cap_3d(x, y, l, false, cl.v[y * xg + x] as f64);
                        }
                    }
                }
            }
            // R7 continues from OUR R6: its usage must be the reference's R6 exit (R7's entry dump).
            let mut g2d = Graph2d::new(xg, yg, 1);
            g2d.est = est;
            g2d.ndr = ledger;
            let chained = flags.re_route && small && chain.is_some();
            if chained {
                let g = chain.take().expect("chained");
                if let Some(d) = usage_diff(&g.est, &c["entry"]) {
                    panic!("{at}: R6 exit usage {d}");
                }
                g2d = g;
                seen.r6_checked += 1;
            }

            // Nets, indexed by id; absent ids stay empty.
            let tnets = arr(&c["nets"]);
            let pins: Vec<(Vec<i32>, Vec<i32>)> = tnets.iter().map(|n| arr(&n["pins"]).iter().map(|p| (int(&p[0]), int(&p[1]))).unzip()).collect();
            let lecs: Vec<Vec<i8>> = tnets.iter().map(|n| arr(&n["lec"]).iter().map(|v| int(v) as i8).collect()).collect();
            let empty: (Vec<i32>, Vec<i32>) = (Vec::new(), Vec::new());
            let mut by_id: Vec<Option<usize>> = vec![None; max_id + 1];
            for (i, n) in tnets.iter().enumerate() {
                by_id[int(&n["id"]) as usize] = Some(i);
            }
            let nets: Vec<RsmtNet<'_>> = by_id
                .iter()
                .map(|slot| match slot {
                    Some(i) => {
                        let n = &tnets[*i];
                        RsmtNet {
                            pins_x: &pins[*i].0,
                            pins_y: &pins[*i].1,
                            alpha: n["alpha"].as_f64().expect("alpha") as f32,
                            edge_cost: int(&n["cost"]) as i8,
                            min_layer: int(&n["min"]) as usize,
                            max_layer: int(&n["max"]) as usize,
                            layer_edge_cost: &lecs[*i],
                        }
                    }
                    None => RsmtNet { pins_x: &empty.0, pins_y: &empty.1, alpha: 0.0, edge_cost: 1, min_layer: 0, max_layer: 0, layer_edge_cost: &[] },
                })
                .collect();
            let net_ids: Vec<usize> = tnets.iter().map(|n| int(&n["id"]) as usize).collect();

            // The segments going in: R5 starts from an empty list; R7 from the list R6 left, whose
            // coordinates must be R5's and whose bends R6 chose.
            for n in tnets {
                let id = int(&n["id"]) as usize;
                let cost = int(&n["cost"]) as i8;
                if flags.re_route {
                    let rl: Vec<RoutedSegment> = arr(&n["rl"]).iter().map(|s| RoutedSegment { seg: seg(s, cost), x_first: int(&s[4]) == 1 }).collect();
                    let ours: Vec<Segment> = state[id].seglist.iter().map(|s| s.seg).collect();
                    assert_eq!(ours, rl.iter().map(|s| s.seg).collect::<Vec<_>>(), "{at}: net {id}: R7's incoming segments are not R5's");
                    if chained {
                        // ⛔ Our R6 chose these bends; they must be the reference's.
                        assert_eq!(state[id].seglist, rl, "{at}: net {id}: R6's bends (xFirst)");
                        seen.r6_segs += rl.len();
                    } else {
                        state[id].seglist = rl;
                    }
                } else {
                    state[id].seglist.clear();
                }
            }

            let stt: std::collections::HashMap<usize, RsmtTree> = tnets.iter().map(|n| (int(&n["id"]) as usize, tree_of(&n["rt"]))).collect();
            let (rh, rv) = (red_h.clone(), red_v.clone());
            let red_h_f = move |x: usize, y: usize| rh[y * xg + x];
            let red_v_f = move |x: usize, y: usize| rv[y * xg + x];
            let mut grid = BrkGrid {
                g: &mut g2d,
                red_h: &red_h_f,
                red_v: &red_v_f,
                caps: &caps,
                h_capacity: int(&c["hcap"]),
                v_capacity: int(&c["vcap"]),
                via_cost: 0.0,
            };
            let sum = gen_brk_rsmt(flags, &net_ids, &nets, &mut state, &mut grid, &|id| stt[&id].clone(), &flutes)
                .unwrap_or_else(|e| panic!("{at}: {e:?}"));

            for (rec, n) in sum.nets.iter().zip(tnets) {
                let id = int(&n["id"]) as usize;
                let at = format!("{at}: net {id} {}", n["name"].as_str().unwrap_or(""));
                let rp = &n["rp"];
                let kind = match rp["kind"].as_str() {
                    Some("stt") => TreeKind::Stt,
                    Some("normal") => TreeKind::Normal,
                    Some("congest") => TreeKind::Congest,
                    k => panic!("{at}: kind {k:?}"),
                };
                assert_eq!(rec.kind, kind, "{at}: builder");
                assert_eq!(rec.coeff_v.map(f32::to_bits), rp["coeff_bits"].as_u64().map(|b| b as u32), "{at}: coeffV");
                let tri = |v: &Value| match int(v) { -1 => None, b => Some(b == 1) };
                assert_eq!(rec.htree, if flags.no_adj { None } else { tri(&rp["htree"]) }, "{at}: HTreeSuite");
                assert_eq!(rec.congested, tri(&rp["cong"]), "{at}: netCongestion");
                assert_eq!(rec.shifts, n.get("rx").map(int), "{at}: edgeShiftNew's count");
                assert_eq!(rec.tree, tree_of(&n["rt"]), "{at}: the tree");
                seen.htree += (rec.htree == Some(true)) as usize;
                if kind != TreeKind::Stt {
                    seen.flute_nets += 1;
                }
                if let Some(s) = rec.shifts {
                    seen.shifted += 1;
                    seen.shifts += s;
                }

                if let Some(rc) = n.get("rc") {
                    let t = rec.copied.as_ref().expect("copied");
                    for (i, node) in arr(&rc["nodes"]).iter().enumerate() {
                        let nb: Vec<(usize, usize)> = arr(&node[4]).iter().map(|p| (int(&p[0]) as usize, int(&p[1]) as usize)).collect();
                        let ours: Vec<(usize, usize)> = (0..t.nbr_count[i]).map(|k| (t.nbr[i][k], t.edge[i][k])).collect();
                        assert_eq!(
                            (t.nodes[i].x as i32, t.nodes[i].y as i32, t.nodes[i].status as i32, t.nbr_count[i] as i32, ours),
                            (int(&node[0]), int(&node[1]), int(&node[2]), int(&node[3]), nb),
                            "{at}: copied node {i}"
                        );
                    }
                    let edges: Vec<(usize, usize, i32)> = arr(&rc["edges"]).iter().map(|e| (int(&e[0]) as usize, int(&e[1]) as usize, int(&e[2]))).collect();
                    assert_eq!(t.edges.iter().map(|e| (e.n1, e.n2, e.len)).collect::<Vec<_>>(), edges, "{at}: copied edges");
                    let want: Vec<i32> = arr(&rc["pins"]).iter().map(int).collect();
                    assert_eq!(t.node_to_pin_idx, want, "{at}: node_to_pin_idx");
                    let mut pos: Vec<(i32, i32)> = pins[by_id[id].unwrap()].0.iter().copied().zip(pins[by_id[id].unwrap()].1.iter().copied()).collect();
                    pos.sort();
                    seen.stacked_pins += pos.windows(2).any(|w| w[0] == w[1]) as usize;
                    seen.copied += 1;
                }

                let cost = int(&n["cost"]) as i8;
                let rs: Vec<Segment> = arr(&n["rs"]).iter().map(|s| seg(s, cost)).collect();
                assert_eq!(state[id].seglist.iter().map(|s| s.seg).collect::<Vec<_>>(), rs, "{at}: the segment list");

                if let Some(rr) = n.get("rr") {
                    let want: Vec<TreeRoute> = arr(rr).iter().map(|e| TreeRoute {
                        kind: match int(&e[0]) { 0 => RouteKind::NoRoute, 1 => RouteKind::LRoute, t => panic!("{at}: route type {t}") },
                        x_first: int(&e[1]) == 1,
                        ..TreeRoute::default()
                    }).collect();
                    let t = state[id].tree.as_ref().expect("routed");
                    // ⚠️ Large grids carry no usage in the dump, so their L decisions cannot be
                    // replayed; their trees, copies and segment lists still are.
                    if small {
                        assert_eq!(t.routes, want, "{at}: newrouteL");
                        seen.routed_edges += want.len();
                    }
                }
                seen.nets += 1;
            }
            assert_eq!(sum.nets.len(), tnets.len(), "{at}: nets visited");

            if small {
                let exit = (usage(&c["exit"], "H"), usage(&c["exit"], "V"));
                let mut first_diff = None;
                'scan: for (dir, rows) in [("H", &exit.0), ("V", &exit.1)] {
                    for (y, row) in rows.iter().enumerate() {
                        for (x, &(e, red)) in row.iter().enumerate() {
                            let (ours, our_red) = if dir == "H" { (g2d.est.h(x, y), red_h[y * xg + x]) } else { (g2d.est.v(x, y), red_v[y * xg + x]) };
                            if ours != e || our_red != red {
                                first_diff = Some(format!("{dir} ({x}, {y}): engine {ours}/{our_red}, reference {e}/{red}"));
                                break 'scan;
                            }
                        }
                    }
                }
                if let Some(d) = first_diff {
                    panic!("{at}: exit usage {d}");
                }
                seen.usage_checked += 1;
                seen.ndr_checked += (ndr && flags.re_route) as usize;
            }
            // R6: `routeLAll(true)` from R5's state, carried into the next call.
            if !flags.re_route && small {
                let mut grid6 = BrkGrid {
                    g: &mut g2d,
                    red_h: &red_h_f,
                    red_v: &red_v_f,
                    caps: &caps,
                    h_capacity: int(&c["hcap"]),
                    v_capacity: int(&c["vcap"]),
                    via_cost: 0.0,
                    };
                route_l_all(&net_ids, &nets, &mut state, &mut grid6);
                chain = Some(g2d);
            } else if chained {
                // Past R7, from our own state: B7, then R8 (`newrouteLAll(false, true)`) and B8.
                let group: Vec<&Value> = bounds.map(|b| arr(&b["boundaries"]).iter().collect()).unwrap_or_default();
                // The budget can strip a committed run's maze-phase boundaries; otherwise a missing
                // B11 means the reference stopped in convertToMazeroute.
                let maze_stripped = bounds.and_then(|b| b["maze_stripped"].as_bool()).unwrap_or(false);
                let at_b = |tag: &str| group.iter().filter(|x| x["tag"] == tag).nth(pass).copied();
                if let Some(b7) = at_b("B7") {
                    for (d, cap) in [("H", &mut g2d.cap_h), ("V", &mut g2d.cap_v)] {
                        for (y, row) in arr(&b7["cap"][d]).iter().enumerate() {
                            let flat: Vec<u16> = arr(row).iter().flat_map(|p| std::iter::repeat(int(&p[0]) as u16).take(int(&p[1]) as usize)).collect();
                            for (x, c) in flat.into_iter().enumerate() {
                                cap[y * xg + x] = c;
                            }
                        }
                    }
                    let scan = g2d.get_overflow_2d();
                    check_boundary(&at, b7, &scan, &g2d, &state, seen);
                    let mut grid8 = BrkGrid {
                        g: &mut g2d,
                        red_h: &red_h_f,
                        red_v: &red_v_f,
                        caps: &caps,
                        h_capacity: int(&c["hcap"]),
                        v_capacity: int(&c["vcap"]),
                        via_cost: 0.0,
                    };
                    newroute_l_all(false, true, &net_ids, &nets, &mut state, &mut grid8);
                    let scan = g2d.get_overflow_2d();
                    check_boundary(&at, at_b("B8").expect("B8 follows B7"), &scan, &g2d, &state, seen);
                    // R9 `spiralRouteAll`; no overflow scan follows, so B9 still carries R8's.
                    let mut grid9 = BrkGrid {
                        g: &mut g2d,
                        red_h: &red_h_f,
                        red_v: &red_v_f,
                        caps: &caps,
                        h_capacity: int(&c["hcap"]),
                        v_capacity: int(&c["vcap"]),
                        via_cost: 0.0,
                    };
                    let nl = caps.layers.len().max(max_layer + 1) as i16;
                    spiral_route_all(&net_ids, &nets, &mut state, &mut grid9, nl, &|_, _| 0);
                    check_boundary(&at, at_b("B9").expect("B9 follows B8"), &scan, &g2d, &state, seen);
                    // R10 `newrouteZAll(10)`, then `getOverflow2D`.
                    let mut grid10 = BrkGrid {
                        g: &mut g2d,
                        red_h: &red_h_f,
                        red_v: &red_v_f,
                        caps: &caps,
                        h_capacity: int(&c["hcap"]),
                        v_capacity: int(&c["vcap"]),
                        via_cost: 0.0,
                    };
                    newroute_z_all(10, &net_ids, &nets, &mut state, &mut grid10);
                    let scan = g2d.get_overflow_2d();
                    check_boundary(&at, at_b("B10").expect("B10 follows B9"), &scan, &g2d, &state, seen);
                    // The maze phase, where the committed sample keeps it: R11, three R12 rounds, R13.
                    let (hcap, vcap) = (int(&c["hcap"]), int(&c["vcap"]));
                    let b11 = at_b("B11");
                    let viol = if maze_stripped { Vec::new() } else { convert_to_mazeroute_all(&net_ids, &mut state, &mut g2d, hcap, vcap) };
                    if b11.is_none() && !maze_stripped {
                        // ⛔ The reference raised GRT-0228/0229 in check2DEdgesUsage and stopped here.
                        assert!(!viol.is_empty(), "{at}: the reference stopped in convertToMazeroute, but check2DEdgesUsage found nothing");
                        seen.usage_errors += 1;
                    }
                    if let Some(b11) = b11 {
                        assert!(viol.is_empty(), "{at}: check2DEdgesUsage {viol:?}");
                        check_boundary(&at, b11, &scan, &g2d, &state, seen);
                        let mut grid12 = BrkGrid {
                            g: &mut g2d,
                            red_h: &red_h_f,
                            red_v: &red_v_f,
                            caps: &caps,
                            h_capacity: hcap,
                            v_capacity: vcap,
                            via_cost: 0.0,
                        };
                        let mut last = scan;
                        let pattern_scan = scan;
                        let mut last_lc = 0.0f32;
                        lv_rounds(scan.max_overflow, &net_ids, &nets, &mut state, &mut grid12, &mut |k, round, g, st| {
                            let b = at_b(&format!("B12_{k}")).unwrap_or_else(|| panic!("B12_{k} follows B11"));
                            assert_eq!(round.logistic_coef as f64, b["logistic_coef"].as_f64().expect("f"), "{at} B12_{k}: logistic_coef");
                            check_boundary(&at, b, &round.scan, g, st, seen);
                            last = round.scan;
                            last_lc = round.logistic_coef;
                        });
                        init_for_congestion_loop(&net_ids, &mut state, &mut g2d);
                        let b13 = at_b("B13").expect("B13 follows B12");
                        // The nets' slacks come from timing setup: an INPUT, seeded from B13.
                        for (id, ns) in b13["netstate"].as_object().expect("netstate") {
                            let id: usize = id.parse().expect("id");
                            state[id].slack = f32::from_bits(int(&ns["slack_bits"]) as u32);
                            state[id].critical = ns["critical"].as_bool().expect("critical");
                        }
                        check_boundary(&at, b13, &last, &g2d, &state, seen);
                        seen.maze_passes += 1;

                        // R14 — the congestion loop, run by OUR schedule. Every maze pass's computed
                        // parameters must equal the reference's (BQ, BH), and the state before and
                        // after it the reference's boundaries.
                        let pass_pres: Vec<&Value> = {
                            // This pass's iterations: those between this B13 and the next B7 (if any).
                            let tags: Vec<&str> = group.iter().map(|b| b["tag"].as_str().unwrap_or("")).collect();
                            let starts: Vec<usize> = tags.iter().enumerate().filter(|(_, t)| **t == "B13").map(|(k, _)| k).collect();
                            let from = starts[pass];
                            let to = tags.iter().enumerate().skip(from + 1).find(|(_, t)| **t == "B7").map_or(tags.len(), |(k, _)| k);
                            group[from..to].iter().filter(|b| b["tag"].as_str().is_some_and(|t| t.starts_with("B14pre_"))).copied().collect()
                        };
                        let cnp = pass_pres.first().map_or(0, |b| int(&b["params"]["cnp"]));
                        let start = LoopStart {
                            pattern_max_overflow: pattern_scan.max_overflow,
                            logistic_coef: last_lc,
                            scan: last,
                            overflow_iterations: 50,
                            critical_nets_percentage: cnp,
                        };
                        let mut grid14 = BrkGrid {
                            g: &mut g2d,
                            red_h: &red_h_f,
                            red_v: &red_v_f,
                            caps: &caps,
                            h_capacity: hcap,
                            v_capacity: vcap,
                            via_cost: 0.0,
                        };
                        let mut befores = 0usize;
                        let mut final_ready = false;
                        let mut has_2d_overflow = false;
                        let mut prev_scan = last;
                        let outcome = congestion_loop(&start, &net_ids, &nets, &mut state, &mut grid14, &mut |ev, g, st| match ev {
                            LoopEvent::Before { params: p, schedule: sc } => {
                                let i = p.iter;
                                let pre = at_b(&format!("B14pre_{i}")).unwrap_or_else(|| panic!("{at}: iteration {i} the reference did not run"));
                                let (q, h) = (&pre["params"], &pre["history"]);
                                let ours = [
                                    ("enlarge", p.expand), ("ripup_threshold", p.ripup_threshold), ("maze_edge_threshold", p.maze_edge_threshold),
                                    ("ordering", p.ordering as i32), ("via", p.via), ("L", p.l), ("costheight", p.cost.cost_height as i32),
                                    ("slope", p.cost.slope), ("upType", sc.up_type), ("stopDEC", sc.stop_dec as i32), ("THRESH_M", sc.thresh_m),
                                    ("cost_step", sc.cost_step), ("max_adj", sc.max_adj),
                                ];
                                for (k, v) in ours {
                                    assert_eq!(v, int(&q[k]), "{at} iteration {i}: schedule {k}");
                                }
                                assert_eq!(p.cost.logistic_coef.to_bits(), q["logistic"].as_f64().expect("f").to_bits(), "{at} iteration {i}: logistic_coef");
                                assert_eq!(p.slack_th.to_bits(), int(&q["slack_th_bits"]) as u32, "{at} iteration {i}: slack_th");
                                let (ut, ah, sd) = sc.history_args;
                                assert_eq!((ut, ah, sd as i32, sc.max_adj), (int(&h["up_type"]), int(&h["ahth"]), int(&h["stop_dec"]), int(&h["max_adj"])),
                                           "{at} iteration {i}: updateCongestionHistory (upType, ahth, stopDEC, max_adj)");
                                check_boundary(&at, pre, &prev_scan, g, st, seen);
                                befores += 1;
                            }
                            LoopEvent::After { kind, iter, scan } => {
                                let tag = match kind { PassKind::Main => "B14", PassKind::Extra20 => "B14b", PassKind::ExtraCopyRs => "B14c" };
                                let b = at_b(&format!("{tag}_{iter}")).unwrap_or_else(|| panic!("{at}: {tag}_{iter} missing"));
                                check_boundary(&at, b, scan, g, st, seen);
                                prev_scan = *scan;
                                if kind == &PassKind::Main { seen.loop_iterations += 1 } else { seen.loop_extra_stops += 1 }
                            }
                        });
                        match outcome {
                            Ok(end) => {
                                assert_eq!(end.iterations as usize, pass_pres.len(), "{at}: the loop ran {} iterations; the reference {}", end.iterations, pass_pres.len());
                                // ⛔ Asserted, not noted: no corpus loop reaches these branches. A recapture
                                // that does fails here and points at the constructed-only coverage.
                                assert_eq!(end.rare, RareBranches::default(), "{at}: a loop branch the corpus never reached fired: {:?}", end.rare);
                                final_ready = true;
                                has_2d_overflow = end.has_2d_overflow;
                            }
                            Err(e) if e.contains("CalculatePartialSlack") => {
                                assert!(cnp != 0, "{at}: partial slack refused with cnp 0");
                                seen.loop_partial_slack += 1;
                            }
                            Err(e) => panic!("{at}: {e}"),
                        }
                        let _ = befores;
                        // R15 — freeRR (the loop's own backup, dropped with it) and removeLoops, then
                        // getOverflow2Dmaze.
                        if final_ready {
                            if let Some(b15) = at_b("B15") {
                                let removed = remove_loops_all(&net_ids, &nets, &mut state, &mut g2d);
                                // ⛔ Asserted, not noted: no corpus run leaves a loop for R15 to remove (0
                                // across all 112 exhaustive runs that reach B15), so B15 witnesses only the
                                // walk and the overflow scan. The removal itself is witnessed by the
                                // constructed cases in `tests/removeloops.rs` alone (whose own capture
                                // found no loop either); a recapture that removes one fails here.
                                assert_eq!(removed, 0, "{at}: R15 removed {removed} loops; the corpus never did");
                                seen.loops_removed += removed;
                                let scan = g2d.get_overflow_2d_maze();
                                check_boundary(&at, b15, &scan, &g2d, &state, seen);
                                seen.r15_checked += 1;
                                // R16 — layerAssignment, then getOverflow3D. The 3D capacity is set up
                                // before routing and nothing in 2D touches it: an INPUT, from B15.
                                if let Some(b16) = at_b("B16") {
                                    let mut g3 = graph3d_from(b15, xg, yg);
                                    assert!(g3.h_usage.iter().chain(&g3.v_usage).all(|l| l.iter().all(|&u| u == 0)), "{at}: 3D usage before layer assignment");
                                    let dims = &b15["dims"];
                                    let dirs: Vec<LayerDir> = arr(&dims["dirs"]).iter().map(|d| match d.as_str() { Some("H") => LayerDir::Horizontal, Some("V") => LayerDir::Vertical, _ => LayerDir::Other }).collect();
                                    let attrs = layer_attrs(b15, b16, max_id + 1);
                                    let p = LayerParams { layer_dir: &dirs, resistance_aware: int(&dims["resaware"]) == 1, liberty: int(&dims["liberty"]) == 1, has_2d_overflow };
                                    let assigned = match layer_assignment(&net_ids, &nets, &attrs, &mut state, &mut g3, &p) {
                                        Ok(()) => true,
                                        // ⛔ Resistance-aware with a liberty library: updateSlacks keeps a net
                                        // (an unconstrained CLOCK net is not skipped) and reads its resistance
                                        // from the database — not wired. A declared stop, counted.
                                        Err(e) if e.contains("needs its resistance") => {
                                            assert!(p.liberty && p.resistance_aware, "{at}: {e} without liberty + resistance-aware");
                                            seen.r16_res_aware += 1;
                                            false
                                        }
                                        Err(e) => panic!("{at}: {e}"),
                                    };
                                    if assigned {
                                    let ov3 = get_overflow_3d_all(&g2d, &g3);
                                    assert_eq!((b16["logistic_coef"].as_f64().expect("past_cong") as i32, int(&b16["total_overflow"]), int(&b16["t_usage"])), (scan.total_overflow, ov3.total, ov3.total_usage),
                                               "{at} B16: (past_cong, getOverflow3D overflow, 3D usage)");
                                    check_boundary(&at, b16, &Overflow2DScan { total_overflow: ov3.total, ..scan }, &g2d, &state, seen);
                                    check_3d(&at, b16, &g3, &state, &nets, seen);
                                    seen.r16_checked += 1;
                                    }
                                }
                            }
                        }
                    }
                }
                pass += 1;
            }
            seen.calls += 1;
        }
        seen.runs += 1;
    }
}

#[test]
fn gen_brk_rsmt_matches_the_reference() {
    let dir = format!("{}/examples/grt_gate", env!("CARGO_MANIFEST_DIR"));
    let s = replay(&read(&format!("{dir}/brk_rsmt.json")), &read(&format!("{dir}/boundaries.json")));
    eprintln!("{s:?}");
    assert!(s.runs >= 40 && s.flute_nets >= 990 && s.shifted >= 70 && s.shifts > 0 && s.copied >= 900
            && s.routed_edges >= 1000 && s.usage_checked >= 60 && s.ndr_checked >= 2 && s.r6_checked >= 40 && s.boundaries >= 160 && s.gate_hvh > 0 && s.gate_vhv > 0 && s.maze_passes >= 18 && s.loop_iterations >= 50 && s.htree > 0,
            "{s:?}");
}

/// GRT_BRK_RSMT_FULL=/path/to/r7-all.json GRT_BOUNDARIES_FULL=/path/to/rb-all.json cargo test --release --test brk_rsmt -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn gen_brk_rsmt_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_BRK_RSMT_FULL").expect("set GRT_BRK_RSMT_FULL");
    let bpath = std::env::var("GRT_BOUNDARIES_FULL").expect("set GRT_BOUNDARIES_FULL");
    eprintln!("exhaustive: {:?}", replay(&read(&path), &read(&bpath)));
}

fn add(a: Seen, b: Seen) -> Seen {
    Seen {
        runs: a.runs + b.runs, calls: a.calls + b.calls, nets: a.nets + b.nets, flute_nets: a.flute_nets + b.flute_nets,
        shifted: a.shifted + b.shifted, shifts: a.shifts + b.shifts, copied: a.copied + b.copied,
        routed_edges: a.routed_edges + b.routed_edges, usage_checked: a.usage_checked + b.usage_checked,
        ndr_checked: a.ndr_checked + b.ndr_checked, r6_checked: a.r6_checked + b.r6_checked, r6_segs: a.r6_segs + b.r6_segs, boundaries: a.boundaries + b.boundaries, boundary_nets: a.boundary_nets + b.boundary_nets, incremental: a.incremental + b.incremental, gate_hvh: a.gate_hvh + b.gate_hvh, gate_vhv: a.gate_vhv + b.gate_vhv, maze_routes: a.maze_routes + b.maze_routes, maze_passes: a.maze_passes + b.maze_passes, usage_errors: a.usage_errors + b.usage_errors, loop_iterations: a.loop_iterations + b.loop_iterations, snapshot_batched: a.snapshot_batched + b.snapshot_batched, loops_removed: a.loops_removed + b.loops_removed, r15_checked: a.r15_checked + b.r15_checked, r16_checked: a.r16_checked + b.r16_checked, r16_widened: a.r16_widened + b.r16_widened, r16_res_aware: a.r16_res_aware + b.r16_res_aware, thinned: a.thinned + b.thinned, loop_partial_slack: a.loop_partial_slack + b.loop_partial_slack, loop_extra_stops: a.loop_extra_stops + b.loop_extra_stops, stacked_pins: a.stacked_pins + b.stacked_pins, htree: a.htree + b.htree,
    }
}

// ─── Constructed cases: branches the corpus never reaches ───────────────────────────────────

use std::cell::RefCell;

/// The 3D capacity (B15's `c3`) and usage (`u3`) as the router holds them: H rows over `x < xg-1`,
/// V rows over `y < yg-1`, both indexed `y * xg + x`.
fn graph3d_from(b: &Value, xg: usize, yg: usize) -> Graph3d {
    // ⚠️ A single-row grid has no vertical edges, so its V rows are absent, not empty.
    let nl = arr(&b["c3"]["H"]).len();
    let layers = |key: &str, d: &str| -> Vec<Vec<u16>> {
        if b[key][d].is_null() {
            return vec![vec![0u16; xg * yg]; nl];
        }
        arr(&b[key][d]).iter().map(|l| {
            let mut v = vec![0u16; xg * yg];
            for (y, row) in arr(l).iter().enumerate() {
                for (x, val) in unrle(row).into_iter().enumerate() {
                    v[y * xg + x] = val[0] as u16;
                }
            }
            v
        }).collect()
    };
    let h_cap = layers("c3", "H");
    Graph3d { x_grid: xg, num_layers: h_cap.len(), v_cap: layers("c3", "V"), h_usage: layers("u3", "H"), v_usage: layers("u3", "V"), h_cap }
}

/// Per net: pin layers (B15's `pin_layers`) and B15's `BA` — the NDR rule, the clock flag,
/// `isResAware` on entry, `getLayerEdgeCost` over every layer — and the timer's slack, which
/// `updateSlacks` writes when it runs: an INPUT, read from B16 after it ran.
fn layer_attrs(b15: &Value, b16: &Value, n: usize) -> Vec<NetLayerAttrs> {
    let mut out = vec![NetLayerAttrs { pin_layers: Vec::new(), has_ndr: false, is_clock: false, is_res_aware: false, layer_edge_cost: Vec::new(), sta_slack: 0.0 }; n];
    // A call with no routed nets writes no pin layers at all.
    let none = serde_json::Map::new();
    for (id, pl) in b15["pin_layers"].as_object().unwrap_or(&none) {
        let id: usize = id.parse().expect("id");
        let a = arr(&b15["attrs"][id.to_string()]);
        let sta_slack = f32::from_bits(int(&b16["netstate"][id.to_string()]["slack_bits"]) as u32);
        out[id] = NetLayerAttrs {
            pin_layers: arr(pl).iter().map(|v| int(v) as i16).collect(),
            has_ndr: int(&a[0]) == 1,
            is_clock: int(&a[1]) == 1,
            is_res_aware: int(&a[2]) == 1,
            layer_edge_cost: arr(&a[6]).iter().map(|v| int(v) as i8).collect(),
            sta_slack,
        };
    }
    out
}

/// The 3D half of a boundary from B16 on: per-layer usage, each node's layer extremes, each net's
/// (possibly widened) layer range, and each maze route's layers.
fn check_3d(at: &str, b: &Value, g3: &Graph3d, state: &[NetState], nets: &[RsmtNet<'_>], seen: &mut Seen) {
    let at = format!("{at} {}", b["tag"].as_str().expect("tag"));
    let (xg, yg) = (int(&b["xg"]) as usize, int(&b["yg"]) as usize);
    let want = graph3d_from(&serde_json::json!({ "c3": b["u3"], "u3": b["u3"] }), xg, yg);
    for l in 0..g3.num_layers {
        assert_eq!(g3.h_usage[l], want.h_usage[l], "{at}: 3D usage H layer {l}");
        assert_eq!(g3.v_usage[l], want.v_usage[l], "{at}: 3D usage V layer {l}");
    }
    for (id, nodes) in b["layers"].as_object().unwrap_or(&serde_json::Map::new()) {
        let id: usize = id.parse().expect("id");
        let t = state[id].tree.as_ref().expect("tree");
        let ours: Vec<[i32; 6]> = t.walk.iter().map(|w| [i32::from(w.top_layer), i32::from(w.bot_layer), w.h_id, w.l_id, w.edges.len() as i32, i32::from(w.assigned)]).collect();
        let want: Vec<[i32; 6]> = arr(nodes).iter().map(|v| std::array::from_fn(|k| int(&v[k]))).collect();
        assert_eq!(ours, want, "{at}: net {id} nodes (topL, botL, hID, lID, conCNT, assigned)");
        let a = arr(&b["attrs"][id.to_string()]);
        let range = state[id].layer_range.unwrap_or((nets[id].min_layer, nets[id].max_layer));
        assert_eq!((range.0 as i32, range.1 as i32), (int(&a[3]), int(&a[4])), "{at}: net {id} layer range");
        // ⛔ Asserted, not noted: no corpus net widens its layer range (every such run either has
        // room in range or 2D overflow). The widening rules are witnessed by `tests/layertable.rs`;
        // its persistence across edges by a constructed case alone.
        assert!(state[id].layer_range.is_none(), "{at}: net {id} widened its layer range; the corpus never did");
        seen.r16_widened += usize::from(state[id].layer_range.is_some());
        for (eid, r) in t.routes.iter().enumerate().filter(|(_, r)| r.kind == RouteKind::MazeRoute) {
            let pts: Vec<i16> = arr(&b["grids"][format!("{id},{eid}")]["points"]).iter().map(|p| int(&p[2]) as i16).collect();
            // ⚠️ A zero-length edge is never assigned: its points keep `GPoint3D`'s default layer, 0.
            let ours = if r.layers.is_empty() { vec![0; r.routelen as usize + 1] } else { r.layers[..=r.routelen as usize].to_vec() };
            assert_eq!(ours, pts, "{at}: net {id} edge {eid} layers");
        }
    }
}

fn no_red(_: usize, _: usize) -> u16 {
    0
}

fn grid_caps(xg: usize, yg: usize, cap: i32) -> Caps3D {
    Caps3D { x_grid: xg, layers: vec![CapLayer { h: vec![cap; xg * yg], v: vec![cap; xg * yg] }] }
}

fn net<'a>(x: &'a [i32], y: &'a [i32]) -> RsmtNet<'a> {
    RsmtNet { pins_x: x, pins_y: y, alpha: 0.0, edge_cost: 1, min_layer: 0, max_layer: 0, layer_edge_cost: &[1] }
}

/// ⛔ `fluteCongest` stretches each gap between consecutive SORTED pins by the usage across the
/// net's span, and maps flute's answer back through `mapxy`. The usage sum is `int += double`
/// (0.6 added twice is 0, not 1.2), usage includes the reduction, a stretched gap is at least 1,
/// and y's factor carries coeffV.
#[test]
fn flute_congest_stretches_each_gap_by_its_usage() {
    let (xg, yg) = (8, 5);
    let mut g2d = Graph2d::new(xg, yg, 1);
    for y in 0..=3 {
        g2d.est.update_h(0, 2, y, 500.0); // gap 0: 8 edges × 500 = 4000 → factor 4000/(2·4·1000) = 0.5
        g2d.est.update_h(4, 6, y, 0.6); // gap 2: every add truncates to 0 (rounding would not) → unchanged
    }
    let red_h = |x: usize, y: usize| (x == 2 && y == 0) as u16; // gap 1: reduction 1 → 1/8000 of 200 is 0 → floored to 1
    g2d.est.update_v(0, 0, 1, 1.0); // y gap 0: one edge of 1…
    let red_v = |x: usize, y: usize| if y == 0 && x < 7 && x > 0 { 1u16 } else { 0 }; // …plus red 1 on six more = 7
    let caps = grid_caps(xg, yg, 0);
    let grid = BrkGrid { g: &mut g2d, red_h: &red_h, red_v: &red_v, caps: &caps, h_capacity: 1000, v_capacity: 10, via_cost: 0.0 };
    let sorted = SortedPins { xs: vec![0, 2, 4, 6], ys: vec![0, 1, 2, 3], s: vec![0, 1, 2, 3] };
    let seen = RefCell::new((Vec::new(), Vec::new()));
    let fake = |xs: &[i32], ys: &[i32], _: &[usize], _: i32| {
        *seen.borrow_mut() = (xs.to_vec(), ys.to_vec());
        RsmtTree { deg: 4, length: 0, branch: vec![Branch { x: 101, y: 20, n: 0 }, Branch { x: 150, y: 220, n: 0 }] }
    };
    // The raw pins are deliberately NOT the sorted ones: fluteCongest reads the stored order.
    let t = flute_congest(&[6, 4, 2, 0], &[3, 2, 1, 0], &sorted, 2, 2.0, &grid, &fake);
    // y gap 0: 7 × coeffV 2 / (1 · 7 · 10) = 0.2 → 20.
    assert_eq!(*seen.borrow(), (vec![0, 100, 101, 301], vec![0, 20, 120, 220]));
    assert_eq!((t.branch[0].x, t.branch[0].y), (4, 1), "mapped back through the stretched coordinates");
    assert_eq!((t.branch[1].x, t.branch[1].y), (-1, 3), "a coordinate flute invented maps to -1");
}

/// `mapxy` is a binary search: a hit returns the original coordinate, a miss `-1`.
#[test]
fn mapxy_maps_back_or_gives_minus_one() {
    let (xs, nxs) = ([1, 2, 3, 4, 5], [0, 10, 20, 30, 40]);
    for (i, &n) in nxs.iter().enumerate() {
        assert_eq!(mapxy(n, &xs, &nxs, 5), xs[i]);
    }
    assert_eq!(mapxy(15, &xs, &nxs, 5), -1);
}

/// ⛔ At or OVER the net's summed capacity counts (`>=`), and the walk follows the bend the segment
/// took: x-first checks row y1 then column x2.
#[test]
fn net_congestion_is_at_or_over_capacity() {
    let mut g2d = Graph2d::new(4, 4, 1);
    g2d.est.update_h(0, 1, 0, 3.0);
    let caps = grid_caps(4, 4, 3);
    let grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 3, v_capacity: 3, via_cost: 0.0 };
    let (x, y) = ([0, 2], [0, 2]);
    let seg = RoutedSegment { seg: Segment { x1: 0, y1: 0, x2: 2, y2: 2, edge_cost: 1 }, x_first: true };
    assert!(net_congestion(&net(&x, &y), &[seg], &grid), "usage 3 on capacity 3 is congested");
}

#[test]
fn net_congestion_walks_the_bend_taken() {
    let mut g2d = Graph2d::new(4, 4, 1);
    g2d.est.update_v(0, 0, 1, 5.0); // column x1 — on the y-first path only
    let caps = grid_caps(4, 4, 3);
    let grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 3, v_capacity: 3, via_cost: 0.0 };
    let (x, y) = ([0, 2], [0, 2]);
    let s = Segment { x1: 0, y1: 0, x2: 2, y2: 2, edge_cost: 1 };
    assert!(!net_congestion(&net(&x, &y), &[RoutedSegment { seg: s, x_first: true }], &grid));
    assert!(net_congestion(&net(&x, &y), &[RoutedSegment { seg: s, x_first: false }], &grid));
}

/// A net whose old route is congested is rebuilt by `fluteCongest`, from the pins R5 sorted.
#[test]
fn a_congested_net_takes_the_congestion_flute() {
    let mut g2d = Graph2d::new(8, 8, 1);
    let caps = grid_caps(8, 8, 1);
    let (x, y) = ([0, 3, 5, 7], [0, 4, 2, 6]);
    let nets = [net(&x, &y)];
    let mut state = vec![NetState::default()];
    // Pins 0, 1 on Steiner node 4; pins 2, 3 and node 4 on the root, node 5.
    let fake = |xs: &[i32], ys: &[i32], _: &[usize], _: i32| RsmtTree {
        deg: 4,
        length: 0,
        branch: [4, 4, 5, 5].iter().enumerate().map(|(i, &n)| Branch { x: xs[i], y: ys[i], n })
            .chain([Branch { x: xs[1], y: ys[1], n: 5 }, Branch { x: xs[2], y: ys[2], n: 5 }]).collect(),
    };
    let r5 = BrkFlags { congestion_driven: false, re_route: false, gen_tree: false, no_adj: false };
    let r7 = BrkFlags { congestion_driven: true, re_route: true, gen_tree: true, no_adj: false };
    let no_stt = |_: usize| -> RsmtTree { unreachable!("alpha 0") };
    {
        let mut grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 1, v_capacity: 1, via_cost: 0.0 };
        gen_brk_rsmt(r5, &[0], &nets, &mut state, &mut grid, &no_stt, &fake).expect("R5");
    }
    // Another net's usage fills the first segment's path — y-first, the bend an unrouted segment
    // carries — so it is congested once this net is ripped up.
    let first = state[0].seglist[0].seg;
    g2d.est.update_v(first.x1, first.y1.min(first.y2), first.y1.max(first.y2), 5.0);
    g2d.est.update_h(first.x1, first.x2, first.y2, 5.0);
    let mut grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 1, v_capacity: 1, via_cost: 0.0 };
    let sum = gen_brk_rsmt(r7, &[0], &nets, &mut state, &mut grid, &no_stt, &fake).expect("R7");
    assert_eq!((sum.nets[0].kind, sum.nets[0].congested), (TreeKind::Congest, Some(true)));
}

/// ⛔ From degree 1000 both sorts are `stable_sort`: ties keep their input order. The selection
/// sorts below that do not — so a tie-heavy net tells them apart.
#[test]
fn degree_1000_switches_both_sorts_to_stable() {
    let d = 1000;
    let x: Vec<i32> = (0..d).map(|i| (i % 7) as i32).collect();
    let y: Vec<i32> = (0..d).map(|i| ((i * 13) % 5) as i32).collect();
    let got = RefCell::new(Vec::new());
    let fake = |_: &[i32], _: &[i32], s: &[usize], _: i32| {
        *got.borrow_mut() = s.to_vec();
        RsmtTree { deg: d, length: 0, branch: Vec::new() }
    };
    let (_, sorted) = flute_normal(&x, &y, 2, 1.36, &fake);
    let mut by_x: Vec<usize> = (0..d).collect();
    by_x.sort_by_key(|&p| x[p]);
    let mut rank = vec![0; d];
    for (i, &p) in by_x.iter().enumerate() {
        rank[p] = i;
    }
    let mut by_y = by_x.clone();
    by_y.sort_by_key(|&p| y[p]);
    let want: Vec<usize> = by_y.iter().map(|&p| rank[p]).collect();
    assert_eq!(*got.borrow(), want);
    assert_eq!(sorted.expect("degree > 3 keeps its sort").s, want);
}

/// GRT-188: a node with a fourth neighbour.
#[test]
fn copy_st_tree_refuses_a_fourth_neighbour() {
    let (x, y) = ([0, 2, 0, 2], [0, 0, 2, 2]);
    let t = RsmtTree { deg: 4, length: 0, branch: (0..4).map(|i| Branch { x: x[i], y: y[i], n: 4 }).chain([Branch { x: 1, y: 1, n: 4 }]).collect() };
    assert_eq!(copy_st_tree(&t, &net(&x, &y)), Err(CopyTreeError::InvalidNeighbors));
}

/// GRT-189: two roots leave one edge short.
#[test]
fn copy_st_tree_refuses_a_second_root() {
    let (x, y) = ([0, 2], [0, 0]);
    let t = RsmtTree { deg: 2, length: 0, branch: vec![Branch { x: 0, y: 0, n: 0 }, Branch { x: 2, y: 0, n: 1 }] };
    assert_eq!(copy_st_tree(&t, &net(&x, &y)), Err(CopyTreeError::EdgeCount { edges: 0, nodes: 2 }));
}

fn tree(deg: usize, b: &[(i32, i32, usize)]) -> RsmtTree {
    RsmtTree { deg, length: 0, branch: b.iter().map(|&(x, y, n)| Branch { x, y, n }).collect() }
}

fn at(t: &RsmtTree) -> Vec<(i32, i32, usize)> {
    t.branch.iter().map(|b| (b.x, b.y, b.n)).collect()
}

/// ⛔ `edgeShift`'s two ties. Two horizontal Steiner edges on row 2, which costs 5 per edge; rows
/// 3 and 4 are free. Each edge's best rows are 3 and 4 (cost 0, a TIE → the LOWER row, 3), and
/// both edges gain 40 (a TIE → the FIRST pair, S5–S6). Worked by hand from the reference's rules.
#[test]
fn edge_shift_ties_keep_the_lowest_row_and_the_first_pair() {
    let mut est = EstimateGrid::new(12, 6);
    est.update_h(2, 10, 2, 5.0);
    let mut t = tree(5, &[(2, 0, 5), (2, 4, 5), (6, 4, 6), (10, 0, 7), (10, 4, 7), (2, 2, 6), (6, 2, 7), (10, 2, 7)]);
    assert_eq!(edge_shift(&mut t, 5, &est), 1);
    assert_eq!((t.branch[5].y, t.branch[6].y, t.branch[7].y), (3, 3, 2), "S5–S6 moved to row 3; S7 stayed");
}

/// ⛔ The costs are `int += double`, truncated at every add: 0.6 on each of four edges sums to 0,
/// so the loaded row costs nothing and nothing moves. Rounding would sum to 4 and shift.
#[test]
fn edge_shift_costs_truncate_at_every_add() {
    let mut est = EstimateGrid::new(8, 6);
    est.update_h(2, 6, 2, 0.6);
    let mut t = tree(4, &[(2, 0, 4), (2, 4, 4), (6, 0, 5), (6, 4, 5), (2, 2, 5), (6, 2, 5)]);
    assert_eq!(edge_shift(&mut t, 4, &est), 0);
}

/// ⛔ The shift range reads each end's neighbour table THREE wide whatever its count. S4 has two
/// neighbours, so its third slot is the zero-initialised 0 — pin 0, on row 4 — which widens the
/// range to 1..=4 and lets the edge reach the free row 4. (S5's fourth neighbour spills into the
/// next row of the flat table, as in the reference's `multi_array`.)
#[test]
fn edge_shift_reads_an_unfilled_neighbour_slot_as_node_0() {
    let mut est = EstimateGrid::new(8, 6);
    for y in 1..=3 {
        est.update_h(2, 6, y, 10.0);
    }
    let mut t = tree(4, &[(6, 4, 5), (2, 1, 4), (6, 0, 5), (6, 3, 5), (2, 2, 5), (6, 2, 5)]);
    assert_eq!(edge_shift(&mut t, 4, &est), 1);
    assert_eq!((t.branch[4].y, t.branch[5].y), (4, 4));
}

/// ⛔ `edgeShiftNew`'s rounds. Two coincident Steiner pairs, A (S7 on S8) and B (S9 on S10), each
/// with a horizontal child on both ends. Round 1 swaps A's children; round 2 finds A first again —
/// the pair just tried — so takes the SECOND pair and swaps B; round 3 finds A first, no longer the
/// last tried, and swaps it back. Net: B swapped, A as it was. Worked by hand.
#[test]
fn edge_shift_new_takes_the_second_pair_and_runs_three_rounds() {
    let est = EstimateGrid::new(20, 20);
    let mut t = tree(7, &[
        (1, 4, 7), (4, 1, 7), (7, 4, 8), (1, 16, 9), (4, 19, 9), (7, 16, 10), (10, 12, 11),
        (4, 4, 8), (4, 4, 11), (4, 16, 10), (4, 16, 11), (10, 10, 11),
    ]);
    assert_eq!(edge_shift_new(&mut t, 7, &est), 0);
    let parents: Vec<usize> = at(&t).iter().map(|b| b.2).collect();
    assert_eq!(parents, vec![7, 7, 8, 10, 9, 9, 11, 8, 11, 10, 11, 11]);
}

/// A tree for the driver-level cases: `(x, y, parent)` per branch, pins first.
fn st_tree(deg: usize, b: &[(i32, i32, usize)], pins: &[(i32, i32)]) -> NetState {
    let (px, py): (Vec<i32>, Vec<i32>) = pins.iter().copied().unzip();
    let t = copy_st_tree(&tree(deg, b), &RsmtNet { layer_edge_cost: &[1], ..net(&px, &py) }).expect("valid tree");
    NetState { tree: Some(t), ..NetState::default() }
}

/// ⛔ `spiralRouteAll` routes a net's edges in the order the walk reaches them from the pins, not by
/// edge index. Here the walk reaches S6–S7 (edge 6) before S5–S6 (edge 5), and edge 6's route loads
/// column 5, which edge 5's x-first L needs: in the walk's order edge 5 turns y-first; by index it
/// would tie and go x-first. Worked by hand from the reference's rules.
#[test]
fn spiral_route_all_walks_outward_from_the_pins() {
    let pins = [(12, 0), (0, 2), (2, 0), (5, 7), (10, 2)];
    let mut state = vec![st_tree(5, &[(12, 0, 7), (0, 2, 5), (2, 0, 5), (5, 7, 6), (10, 2, 7), (0, 0, 6), (5, 5, 7), (10, 0, 7)], &pins)];
    let (px, py): (Vec<i32>, Vec<i32>) = pins.iter().copied().unzip();
    let nets = [RsmtNet { layer_edge_cost: &[1], ..net(&px, &py) }];
    let mut g2d = Graph2d::new(14, 10, 1);
    // ⚠️ Capacity 10 puts the f32 bound at exactly 9.0: AT the bound is free, above it is not.
    g2d.est.update_v(5, 0, 5, 9.0); // at the bound: free until one more route lands on it
    g2d.est.update_h(5, 10, 5, 10.0); // over the bound: edge 6 avoids its x-first L
    let caps = grid_caps(14, 10, 10);
    let mut grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 10, v_capacity: 10, via_cost: 0.0 };
    spiral_route_all(&[0], &nets, &mut state, &mut grid, 1, &|_, _| 0);
    let r = &state[0].tree.as_ref().expect("tree").routes;
    assert_eq!((r[6].x_first, r[5].x_first), (false, false), "edge 6 y-first, then edge 5 y-first around the loaded column");
}

/// ⛔ The Z route reads and marks the ALIAS nodes. S3 sits on pin 0, so it is aliased to node 0: the
/// Z of edge S3–p1 must count on node 0 and leave node 3's counters alone.
#[test]
fn the_z_route_marks_the_alias_nodes() {
    let pins = [(0, 0), (20, 15), (0, 5)];
    let mut state = vec![st_tree(3, &[(0, 0, 3), (20, 15, 3), (0, 5, 3), (0, 0, 3)], &pins)];
    let (px, py): (Vec<i32>, Vec<i32>) = pins.iter().copied().unzip();
    let nets = [RsmtNet { layer_edge_cost: &[1], ..net(&px, &py) }];
    let mut g2d = Graph2d::new(22, 17, 1);
    let caps = grid_caps(22, 17, 0); // no capacity anywhere: the gate rips up every long L
    let mut grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 10, v_capacity: 10, via_cost: 0.0 };
    spiral_route_all(&[0], &nets, &mut state, &mut grid, 1, &|_, _| 0);
    let before: Vec<(i32, i32)> = state[0].tree.as_ref().expect("tree").walk.iter().map(|w| (w.h_id, w.l_id)).collect();
    newroute_z_all(10, &[0], &nets, &mut state, &mut grid);
    let t = state[0].tree.as_ref().expect("tree");
    assert_eq!(t.walk[3].stack_alias, 0, "S3 is aliased to pin 0");
    assert_eq!(t.routes[1].kind, RouteKind::ZRoute, "edge S3-p1 was Z-routed");
    let bump = |i: usize| (t.walk[i].h_id - before[i].0) + (t.walk[i].l_id - before[i].1);
    assert_eq!((bump(0), bump(3)), (1, 0), "the Z counts on the alias, not on S3");
}

/// ⛔ The monotonic cost table's HEIGHT (`costheight_`) weighs congestion against length. Edge
/// (0,0)–(4,0) on row 0 (usage 8 after its rip-up) against a detour through row 1 (usage 4), columns
/// free, logistic coefficient 0.2: at height 4 the straight route stays; at height 8 the congestion
/// term outweighs two extra edges and it detours. The expected routes come from an independent model
/// of the reference's search, not from this engine. The corpus never raises the height (it takes
/// `maxOverflow > 700`).
#[test]
fn the_monotonic_cost_height_trades_congestion_for_length() {
    let route_with = |height: i32| -> Vec<(i32, i32)> {
        let pins = [(0, 0), (4, 0)];
        let mut state = vec![st_tree(2, &[(0, 0, 1), (4, 0, 1)], &pins)];
        {
            let r = &mut state[0].tree.as_mut().expect("tree").routes[0];
            (r.kind, r.grids, r.routelen) = (RouteKind::MazeRoute, (0..=4).map(|x| (x, 0)).collect(), 4);
        }
        let (px, py): (Vec<i32>, Vec<i32>) = pins.iter().copied().unzip();
        let nets = [RsmtNet { layer_edge_cost: &[1], ..net(&px, &py) }];
        let mut g2d = Graph2d::new(7, 3, 1);
        for x in 0..6 {
            // Row 0 carries the net's own route (+1) on x < 4; rows 1 and 2 are fixed load.
            g2d.est.update_usage_h(x, 0, if x < 4 { 9.0 } else { 8.0 });
            g2d.est.update_usage_h(x, 1, 4.0);
            g2d.est.update_usage_h(x, 2, 30.0);
        }
        let caps = grid_caps(7, 3, 10);
        let mut grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 10, v_capacity: 10, via_cost: 0.0 };
        route_monotonic_all(1, 2, 0.2, height, &[0], &nets, &mut state, &mut grid);
        state[0].tree.as_ref().expect("tree").routes[0].grids.clone()
    };
    assert_eq!(route_with(4), (0..=4).map(|x| (x, 0)).collect::<Vec<_>>(), "height 4: straight along row 0");
    assert_eq!(route_with(8), vec![(0, 0), (0, 1), (1, 1), (2, 1), (3, 1), (4, 1), (4, 0)], "height 8: through row 1");
}

/// ⛔ The LV rounds' schedule: `newTH` 10, 5, then floored at 1; `enlarge_` 10, 15, 20; and the
/// logistic coefficient from the PREVIOUS scan's max overflow (`2 / (1 + ln 0)` is -0 on an empty grid).
#[test]
fn the_lv_rounds_follow_the_reference_schedule() {
    let mut g2d = Graph2d::new(4, 4, 1);
    let caps = grid_caps(4, 4, 10);
    let mut grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 10, v_capacity: 10, via_cost: 0.0 };
    let (height, rounds) = lv_rounds(701, &[], &[], &mut [], &mut grid, &mut |_, _, _, _| {});
    assert_eq!(height, 8, "maxOverflow > 700 raises the cost height");
    let schedule: Vec<(i32, i32)> = rounds.iter().map(|r| (r.threshold, r.enlarge)).collect();
    assert_eq!(schedule, vec![(10, 10), (5, 15), (1, 20)]);
    assert_eq!(rounds[0].logistic_coef, (2.0 / (1.0 + 701f64.ln())) as f32, "round 0 reads the pattern phase's max overflow");
    assert_eq!(rounds[1].logistic_coef.to_bits(), (-0.0f32).to_bits(), "round 1 reads round 0's scan: nothing routed, ln 0");
}

/// ⛔ The seeding WRITES each point's edge into an array, so a point two edges share keeps the LAST
/// write. No corpus search ends on such a point.
#[test]
fn corr_edge_keeps_the_last_write() {
    let writes = [((3, 4), 7), ((5, 5), 2), ((3, 4), 9)];
    assert_eq!(corr_edge_at(&writes, (3, 4)), Some(9));
    assert_eq!(corr_edge_at(&writes, (5, 5)), Some(2));
    assert_eq!(corr_edge_at(&writes, (0, 0)), None);
}

/// ⛔ `StNetOrder` stamps an uncongested net still carrying the sentinel slack, past the first 30% of
/// the congestion order, with `f32::MAX` — and WRITES it back onto the net, so it persists into later
/// rounds. Only a run with the partial-slack pass has sentinel slacks, and no such run is replayable.
#[test]
fn st_net_order_stamps_and_keeps_the_deprioritised_slack() {
    let pins = [(0, 0), (4, 0)];
    let (px, py): (Vec<i32>, Vec<i32>) = pins.iter().copied().unzip();
    let nets: Vec<RsmtNet<'_>> = (0..4).map(|_| RsmtNet { layer_edge_cost: &[1], ..net(&px, &py) }).collect();
    let mut state: Vec<NetState> = (0..4).map(|_| st_tree(2, &[(0, 0, 1), (4, 0, 1)], &pins)).collect();
    let mut g2d = Graph2d::new(6, 2, 1);
    let caps = grid_caps(6, 2, 10);
    let grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 10, v_capacity: 10, via_cost: 0.0 };
    // Four uncongested nets, all at the sentinel: 30% of 4 is 1, so positions 1..3 are stamped.
    let order = st_net_order(&[0, 1, 2, 3], &nets, &mut state, &grid);
    assert_eq!(order, vec![0, 1, 2, 3], "stable: the stamped nets keep their order, after the unstamped one");
    let slacks: Vec<f32> = state.iter().map(|s| s.slack).collect();
    assert_eq!(slacks, vec![vyges_grt::ripup::SLACK_SENTINEL, f32::MAX, f32::MAX, f32::MAX]);
}

/// ⛔ The loop's `enlarge_` steps by 5 (`ESTEP3`) while the overflow is under 500, clamped at half the
/// grid width. A wide grid with one permanently overflowing edge keeps the loop running unclamped
/// for a few rounds: 20, 25, 30, 35. (Every corpus run that replays the loop is clamped at 17.)
#[test]
fn the_congestion_loop_steps_enlarge_until_the_clamp() {
    let pins = [(0, 0), (4, 0)];
    let mut state = vec![st_tree(2, &[(0, 0, 1), (4, 0, 1)], &pins)];
    {
        let r = &mut state[0].tree.as_mut().expect("tree").routes[0];
        (r.kind, r.grids, r.routelen) = (RouteKind::MazeRoute, (0..=4).map(|x| (x, 0)).collect(), 4);
    }
    let (px, py): (Vec<i32>, Vec<i32>) = pins.iter().copied().unzip();
    let nets = [RsmtNet { layer_edge_cost: &[1], ..net(&px, &py) }];
    let mut g2d = Graph2d::new(80, 3, 1);
    for x in 0..4 {
        g2d.est.update_usage_h(x, 0, 1.0); // the net's own route
        g2d.used_h.insert((x, 0));
    }
    // Capacity 0 on row 0 everywhere, 10 elsewhere: the route overflows wherever it runs on row 0.
    for y in 0..3usize {
        for x in 0..80usize {
            g2d.cap_h[y * 80 + x] = if y == 0 { 0 } else { 10 };
            g2d.cap_v[y * 80 + x] = 10;
        }
    }
    let caps = grid_caps(80, 3, 10);
    let mut grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 10, v_capacity: 10, via_cost: 0.0 };
    let scan = grid.g.get_overflow_2d_maze();
    assert!(scan.total_overflow > 0 && scan.total_overflow < 500);
    let start = LoopStart { pattern_max_overflow: 0, logistic_coef: 0.0, scan, overflow_iterations: 4, critical_nets_percentage: 0 };
    let mut expands = Vec::new();
    let _ = congestion_loop(&start, &[0], &nets, &mut state, &mut grid, &mut |ev, _, _| {
        if let LoopEvent::Before { params, .. } = ev {
            expands.push(params.expand);
        }
    });
    assert_eq!(expands, vec![20, 25, 30, 35], "ESTEP3 per round, under the clamp of 40");
}

/// ⛔ `copyBR` restores the trees `copyRS` saved AND moves the committed usage with them: the current
/// routes are given back, the saved ones charged. No corpus loop reaches it (`i > 80`).
#[test]
fn copy_br_restores_the_saved_routes_and_their_usage() {
    let pins = [(0, 0), (4, 0)];
    let mut state = vec![st_tree(2, &[(0, 0, 1), (4, 0, 1)], &pins)];
    let row = |y: i32| -> Vec<(i32, i32)> { let mut v = vec![(0, 0)]; if y > 0 { v.push((0, y)); } v.extend((1..=4).map(|x| (x, y))); if y > 0 { v.push((4, 0)); } v };
    {
        let r = &mut state[0].tree.as_mut().expect("tree").routes[0];
        (r.kind, r.grids, r.routelen) = (RouteKind::MazeRoute, row(0), 4);
    }
    let (px, py): (Vec<i32>, Vec<i32>) = pins.iter().copied().unzip();
    let nets = [RsmtNet { layer_edge_cost: &[1], ..net(&px, &py) }];
    let mut g2d = Graph2d::new(6, 3, 1);
    for x in 0..4 {
        g2d.est.update_usage_h(x, 0, 1.0);
    }
    let caps = grid_caps(6, 3, 10);
    let mut grid = BrkGrid { g: &mut g2d, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 10, v_capacity: 10, via_cost: 0.0 };
    let saved = copy_rs(&[0], &state);
    // Move the net onto row 1 by hand, usage and all.
    for x in 0..4 {
        grid.g.est.update_usage_h(x, 0, -1.0);
        grid.g.est.update_usage_h(x, 1, 1.0);
    }
    grid.g.est.update_usage_v(0, 0, 1.0);
    grid.g.est.update_usage_v(4, 0, 1.0);
    {
        let r = &mut state[0].tree.as_mut().expect("tree").routes[0];
        (r.grids, r.routelen) = (row(1), 6);
    }
    copy_br(&[0], &nets, &mut state, &mut grid, Some(&saved));
    assert_eq!(state[0].tree.as_ref().expect("tree").routes[0].grids, row(0), "the saved route is back");
    let usage: Vec<(u16, u16)> = (0..4).map(|x| (g2d.est.usage_h(x, 0), g2d.est.usage_h(x, 1))).collect();
    assert_eq!(usage, vec![(1, 0); 4], "row 0 charged again, row 1 given back");
    assert_eq!((g2d.est.usage_v(0, 0), g2d.est.usage_v(4, 0)), (0, 0), "the detour's columns given back");
}

/// R15, constructed — no corpus run leaves a loop (asserted in the replay), so this is the driver's
/// only witness. `removeLoops` walks every positive-length edge of every net, cuts the stretch
/// between two visits of one point, gives back what that stretch was charged through the net's
/// `updateUsageH/V`, and shortens `routelen` by the points cut.
///
/// ⛔ For an NDR net (edge cost > 1) the give-back goes through `getCostNDRAware`, which charges an
/// edge ONCE per net and gives it back WHOLE: an edge the loop crosses and the kept path also
/// crosses is left uncharged. Transcribed, not levelled — pinned by the second half.
#[test]
fn remove_loops_all_cuts_the_detour_and_gives_back_its_usage() {
    use vyges_grt::estimate::Usage2d as _;
    let pins = [(0, 0), (4, 0)];
    // Out along row 0, up and back over row 1, down onto (1,0) again, then on to (4,0).
    let looped = vec![(0, 0), (1, 0), (1, 1), (2, 1), (2, 0), (1, 0), (2, 0), (3, 0), (4, 0)];
    let (px, py): (Vec<i32>, Vec<i32>) = pins.iter().copied().unzip();
    let run = |edge_cost: i8| {
        // Two nets on the same looped route; net 1 only so the count is summed over nets.
        let mut state = vec![st_tree(2, &[(0, 0, 1), (4, 0, 1)], &pins), st_tree(2, &[(0, 0, 1), (4, 0, 1)], &pins)];
        for st in &mut state {
            let r = &mut st.tree.as_mut().expect("tree").routes[0];
            (r.kind, r.grids, r.routelen) = (RouteKind::MazeRoute, looped.clone(), 8);
        }
        let lec = [edge_cost];
        let nets = [RsmtNet { edge_cost, layer_edge_cost: &lec, ..net(&px, &py) }, RsmtNet { edge_cost, layer_edge_cost: &lec, ..net(&px, &py) }];
        let mut g2d = Graph2d::new(6, 3, 1);
        // Charged as the route stands, through each net's own ledger as a route is.
        for id in 0..2 {
            let nn = nets[id].ndr_net(id);
            let mut u = g2d.for_net(&nn);
            for w in looped.windows(2) {
                let ((x1, y1), (x2, y2)) = (w[0], w[1]);
                if y1 == y2 { u.update_usage_h(x1.min(x2), y1, f64::from(edge_cost)) } else { u.update_usage_v(x1, y1.min(y2), f64::from(edge_cost)) }
            }
        }
        let removed = remove_loops_all(&[0, 1], &nets, &mut state, &mut g2d);
        let r = &state[0].tree.as_ref().expect("tree").routes[0];
        let row0: Vec<u16> = (0..4).map(|x| g2d.est.usage_h(x, 0)).collect();
        let detour = (g2d.est.usage_h(1, 1), g2d.est.usage_v(1, 0), g2d.est.usage_v(2, 0));
        (removed, r.routelen, r.grids[..=4].to_vec(), row0, detour)
    };
    let straight = vec![(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)];
    let (removed, routelen, grids, row0, detour) = run(1);
    assert_eq!((removed, routelen), (2, 4), "one loop per net, four points cut");
    assert_eq!(grids, straight, "the straight path is left");
    assert_eq!(row0, vec![2; 4], "row 0 keeps one step each per net; (1,0) was crossed twice and loses one");
    assert_eq!(detour, (0, 0, 0), "the detour given back");
    // NDR: no capacity here, so every first charge is an overflow charge (100 x edge cost).
    let (removed, routelen, grids, row0, detour) = run(2);
    assert_eq!((removed, routelen, grids), (2, 4, straight), "the same cut");
    assert_eq!(row0, vec![400, 0, 400, 400], "(1,0) charged once per net, given back whole though the path still crosses it");
    assert_eq!(detour, (0, 0, 0), "the detour given back");
}

/// One net of a constructed layer-assignment case: its pins, flute branches, layer range and
/// `getLayerEdgeCost` over every layer.
struct LaNet {
    pins: Vec<(i32, i32)>,
    branches: Vec<(i32, i32, usize)>,
    range: (usize, usize),
    lec: Vec<i8>,
    ndr: bool,
    clock: bool,
}

impl LaNet {
    fn two_pin(a: (i32, i32), b: (i32, i32), nl: usize) -> LaNet {
        LaNet { pins: vec![a, b], branches: vec![(a.0, a.1, 1), (b.0, b.1, 1)], range: (0, nl - 1), lec: vec![1; nl], ndr: false, clock: false }
    }
}

/// Run R16 on constructed nets: each edge routed straight from its `n1` to its `n2`, every pin on
/// layer 0, the 3D capacity from `cap(horizontal, layer, x, y)` and no usage.
fn la_case(dirs: &[LayerDir], xg: usize, yg: usize, spec: &[LaNet], cap: impl Fn(bool, usize, usize, usize) -> u16, ids: &[usize], resistance_aware: bool, has_2d_overflow: bool) -> (Vec<NetState>, Graph3d) {
    let nl = dirs.len();
    let mut state: Vec<NetState> = spec.iter().map(|n| st_tree(n.pins.len(), &n.branches, &n.pins)).collect();
    for ns in &mut state {
        let t = ns.tree.as_mut().expect("tree");
        for e in 0..t.edges.len() {
            let (a, b) = (t.nodes[t.edges[e].n1], t.nodes[t.edges[e].n2]);
            let (a, b) = ((i32::from(a.x), i32::from(a.y)), (i32::from(b.x), i32::from(b.y)));
            assert!(a.0 == b.0 || a.1 == b.1, "constructed edges are straight");
            let n = (b.0 - a.0).abs() + (b.1 - a.1).abs();
            let r = &mut t.routes[e];
            r.kind = RouteKind::MazeRoute;
            r.routelen = n;
            r.grids = (0..=n).map(|k| (a.0 + (b.0 - a.0).signum() * k, a.1 + (b.1 - a.1).signum() * k)).collect();
        }
    }
    let pins: Vec<(Vec<i32>, Vec<i32>)> = spec.iter().map(|n| n.pins.iter().copied().unzip()).collect();
    let nets: Vec<RsmtNet<'_>> = spec.iter().zip(&pins).map(|(n, p)| RsmtNet {
        edge_cost: *n.lec.iter().max().expect("lec"),
        min_layer: n.range.0,
        max_layer: n.range.1,
        layer_edge_cost: &n.lec[n.range.0..=n.range.1],
        ..net(&p.0, &p.1)
    }).collect();
    let attrs: Vec<NetLayerAttrs> = spec.iter().map(|n| NetLayerAttrs { pin_layers: vec![0; n.pins.len()], has_ndr: n.ndr, is_clock: n.clock, is_res_aware: false, layer_edge_cost: n.lec.clone(), sta_slack: 0.0 }).collect();
    let layers = |h: bool| -> Vec<Vec<u16>> { (0..nl).map(|l| (0..xg * yg).map(|i| cap(h, l, i % xg, i / xg)).collect()).collect() };
    let mut g3 = Graph3d { x_grid: xg, num_layers: nl, h_cap: layers(true), v_cap: layers(false), h_usage: vec![vec![0; xg * yg]; nl], v_usage: vec![vec![0; xg * yg]; nl] };
    let p = LayerParams { layer_dir: dirs, resistance_aware, liberty: false, has_2d_overflow };
    layer_assignment(ids, &nets, &attrs, &mut state, &mut g3, &p).expect("layer assignment");
    (state, g3)
}

/// The layers of net `id`'s edge running between the nodes at `a` and `b`, after R16.
fn la_layers(state: &[NetState], id: usize, a: (i16, i16), b: (i16, i16)) -> Vec<i16> {
    let t = state[id].tree.as_ref().expect("tree");
    let e = (0..t.edges.len()).find(|&e| {
        let (p, q) = (t.nodes[t.edges[e].n1], t.nodes[t.edges[e].n2]);
        ((p.x, p.y) == a && (q.x, q.y) == b) || ((p.x, p.y) == b && (q.x, q.y) == a)
    }).expect("edge");
    t.routes[e].layers.clone()
}

const HVH: [LayerDir; 3] = [LayerDir::Horizontal, LayerDir::Vertical, LayerDir::Horizontal];

/// ⛔ `netpinOrderInc` decides who is assigned first, and the first net takes the scarce layer. Two
/// identical nets on row 0, room for ONE on layer 0: the winner stays on its pins' layer, the
/// loser climbs to layer 2. Keys in order: NDR first, then (resistance-aware only) clocks first,
/// then … the net id — NOT the order the nets are listed in.
#[test]
fn layer_assignment_gives_the_scarce_layer_to_the_first_net_in_order() {
    let cap = |h: bool, l: usize, _x: usize, _y: usize| if h && l == 0 { 1 } else { 5 };
    let winner = |ndr1: bool, clock1: bool, ra: bool| {
        let mut b = LaNet::two_pin((0, 0), (3, 0), 3);
        (b.ndr, b.clock) = (ndr1, clock1);
        let spec = [LaNet::two_pin((0, 0), (3, 0), 3), b];
        let (st, _) = la_case(&HVH, 4, 3, &spec, cap, &[1, 0], ra, false);
        let on0 = |id: usize| la_layers(&st, id, (0, 0), (3, 0)).iter().all(|&l| l == 0);
        assert_ne!(on0(0), on0(1), "exactly one net stays on layer 0");
        if on0(0) { 0 } else { 1 }
    };
    assert_eq!(winner(false, false, false), 0, "a tie falls to the lower net id, whatever the listing order");
    assert_eq!(winner(true, false, false), 1, "an NDR net goes first");
    assert_eq!(winner(false, true, true), 1, "a clock net goes first when resistance-aware");
    assert_eq!(winner(false, true, false), 0, "the clock key does not exist otherwise");
}

/// ⛔ `assignEdge` charges each step `getLayerEdgeCost(layer)`, not 1: a cost-2 net fills a
/// capacity-2 layer on its own, and the next net must climb.
#[test]
fn layer_assignment_charges_each_step_its_layer_edge_cost() {
    let cap = |h: bool, l: usize, _x: usize, _y: usize| if h && l == 0 { 2 } else { 5 };
    let mut wide = LaNet::two_pin((0, 0), (3, 0), 3);
    wide.lec = vec![2; 3];
    let spec = [wide, LaNet::two_pin((0, 0), (3, 0), 3)];
    let (st, g3) = la_case(&HVH, 4, 3, &spec, cap, &[0, 1], false, false);
    assert!(la_layers(&st, 0, (0, 0), (3, 0)).iter().all(|&l| l == 0), "the wide net takes layer 0");
    assert!(la_layers(&st, 1, (0, 0), (3, 0)).contains(&2), "and leaves the narrow one no room there");
    assert_eq!((0..3).map(|x| (g3.h_usage[0][x], g3.h_usage[2][x])).collect::<Vec<_>>(), vec![(2, 1); 3], "usage charged at each net's cost");
}

/// ⛔ A vertical step reads the edge at the LOWER of its two rows, whichever way the route runs.
/// Downward from row 2: layer 1 has room on rows 0–1 only, layer 3 on every row; read correctly
/// the net stays on layer 1, the nearer.
#[test]
fn layer_assignment_reads_a_downward_step_at_its_lower_row() {
    let dirs = [LayerDir::Horizontal, LayerDir::Vertical, LayerDir::Horizontal, LayerDir::Vertical];
    let cap = |h: bool, l: usize, _x: usize, y: usize| if h { 5 } else if l == 3 || (l == 1 && y < 2) { 1 } else { 0 };
    // The edge runs from the tree's root: root the tree at the top pin.
    let net = LaNet { branches: vec![(0, 2, 0), (0, 0, 0)], ..LaNet::two_pin((0, 2), (0, 0), 4) };
    let (st, _) = la_case(&dirs, 3, 4, &[net], cap, &[0], false, false);
    let t = st[0].tree.as_ref().expect("tree");
    assert_eq!(t.routes[0].grids.first(), Some(&(0, 2)), "the route runs downward");
    assert_eq!(la_layers(&st, 0, (0, 2), (0, 0)), vec![1, 1, 1]);
}

/// ⛔ The walk continues THROUGH Steiner nodes: an edge between two of them is reached only once
/// one of its ends has been expanded. An H-shaped net whose bar (row 1) has no room on layer 0.
#[test]
fn layer_assignment_reaches_an_edge_between_two_steiner_nodes() {
    let pins = vec![(1, 0), (1, 2), (3, 0), (3, 2)];
    let spec = [LaNet { pins, branches: vec![(1, 0, 4), (1, 2, 4), (3, 0, 5), (3, 2, 5), (1, 1, 5), (3, 1, 5)], range: (0, 2), lec: vec![1; 3], ndr: false, clock: false }];
    let cap = |h: bool, l: usize, _x: usize, y: usize| if h && l == 0 && y == 1 { 0 } else { 5 };
    let (st, g3) = la_case(&HVH, 5, 3, &spec, cap, &[0], false, false);
    assert!(la_layers(&st, 0, (1, 1), (3, 1)).contains(&2), "the bar is assigned, on layer 2");
    assert_eq!((g3.h_usage[2][5 + 1], g3.h_usage[2][5 + 2]), (1, 1), "and charged there");
}

/// ⛔ `assignEdge` WIDENS a net's layer range (`setMinLayer`) when no layer in it has room — unless
/// the design has 2D overflow — and the widening STAYS for the net's later edges. A net confined
/// to layers 1–2 whose first edge finds layer 2 full: it reaches down to layer 0, and its second
/// edge, which layer 2 could carry, then stays on layer 0 beside its pins as well.
#[test]
fn layer_assignment_keeps_a_widened_range_for_later_edges() {
    let spec = |_: ()| [LaNet { pins: vec![(0, 0), (4, 0), (2, 2)], branches: vec![(0, 0, 3), (4, 0, 3), (2, 2, 3), (2, 0, 3)], range: (1, 2), lec: vec![1; 3], ndr: false, clock: false }];
    let cap = |h: bool, l: usize, x: usize, _y: usize| if h && l == 2 && x < 2 { 0 } else { 5 };
    let (st, _) = la_case(&HVH, 5, 3, &spec(()), cap, &[0], false, false);
    assert_eq!(st[0].layer_range, Some((0, 2)), "widened down to layer 0");
    assert_eq!(la_layers(&st, 0, (0, 0), (2, 0)), vec![0, 0, 0], "the full edge on layer 0");
    assert_eq!(la_layers(&st, 0, (4, 0), (2, 0)), vec![0, 0, 0], "the next edge still may use it");
    // With 2D overflow the range is never widened, and layer 0 stays barred.
    let (st, _) = la_case(&HVH, 5, 3, &spec(()), cap, &[0], false, true);
    assert_eq!(st[0].layer_range, None, "no widening under 2D overflow");
    assert!(la_layers(&st, 0, (4, 0), (2, 0)).contains(&2), "so the next edge climbs to layer 2");
}
