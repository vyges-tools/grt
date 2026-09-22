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
    stacked_pins: usize,
    htree: usize,
}

/// Replay every run; list every run that diverges rather than stopping at the first.
fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    let mut failed = Vec::new();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        let mut one = Seen::default();
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| replay_run(r, &mut one))) {
            Ok(()) => seen = add(seen, one),
            Err(e) => failed.push(format!("{who}: {}", e.downcast_ref::<String>().cloned().unwrap_or_default())),
        }
    }
    assert!(failed.is_empty(), "{} runs diverge:\n{}", failed.len(), failed.join("\n"));
    seen
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

fn replay_run(r: &Value, seen: &mut Seen) {
    {
        let who = r["design"].as_str().expect("design");
        let calls = arr(&r["calls"]);
        let ndr = calls.iter().flat_map(|c| arr(&c["nets"])).any(|n| int(&n["cost"]) != 1);
        let max_id = calls.iter().flat_map(|c| arr(&c["nets"])).map(|n| int(&n["id"]) as usize).max().unwrap_or(0);
        let mut state: Vec<NetState> = vec![NetState::default(); max_id + 1];
        let max_layer = calls.iter().flat_map(|c| arr(&c["nets"])).map(|n| int(&n["max"]) as usize).max().unwrap_or(0);
        // R6's state carried into R7: our estimated usage and NDR ledger, not the dump's.
        let mut chain: Option<(EstimateGrid, NdrLedger)> = None;
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
            let chained = flags.re_route && small && chain.is_some();
            if chained {
                let (e, l) = chain.take().expect("chained");
                if let Some(d) = usage_diff(&e, &c["entry"]) {
                    panic!("{at}: R6 exit usage {d}");
                }
                est = e;
                ledger = l;
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
                est: &mut est,
                red_h: &red_h_f,
                red_v: &red_v_f,
                caps: &caps,
                h_capacity: int(&c["hcap"]),
                v_capacity: int(&c["vcap"]),
                via_cost: 0.0,
                ndr: &mut ledger,
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
                    let want: Vec<Option<bool>> = arr(rr).iter().map(|e| match int(&e[0]) {
                        0 => { assert_eq!(int(&e[1]), 0, "{at}: a NoRoute edge keeps xFirst false"); None }
                        1 => Some(int(&e[1]) == 1),
                        t => panic!("{at}: route type {t}"),
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
                            let (ours, our_red) = if dir == "H" { (est.h(x, y), red_h[y * xg + x]) } else { (est.v(x, y), red_v[y * xg + x]) };
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
                    est: &mut est,
                    red_h: &red_h_f,
                    red_v: &red_v_f,
                    caps: &caps,
                    h_capacity: int(&c["hcap"]),
                    v_capacity: int(&c["vcap"]),
                    via_cost: 0.0,
                    ndr: &mut ledger,
                };
                route_l_all(&net_ids, &nets, &mut state, &mut grid6);
                chain = Some((est, ledger));
            }
            seen.calls += 1;
        }
        seen.runs += 1;
    }
}

#[test]
fn gen_brk_rsmt_matches_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/brk_rsmt.json", env!("CARGO_MANIFEST_DIR"))));
    eprintln!("{s:?}");
    assert!(s.runs >= 40 && s.flute_nets >= 990 && s.shifted >= 70 && s.shifts > 0 && s.copied >= 900
            && s.routed_edges >= 1000 && s.usage_checked >= 60 && s.ndr_checked >= 2 && s.r6_checked >= 40 && s.htree > 0,
            "{s:?}");
}

/// GRT_BRK_RSMT_FULL=/path/to/r7-all.json cargo test --release --test brk_rsmt -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn gen_brk_rsmt_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_BRK_RSMT_FULL").expect("set GRT_BRK_RSMT_FULL");
    eprintln!("exhaustive: {:?}", replay(&read(&path)));
}

fn add(a: Seen, b: Seen) -> Seen {
    Seen {
        runs: a.runs + b.runs, calls: a.calls + b.calls, nets: a.nets + b.nets, flute_nets: a.flute_nets + b.flute_nets,
        shifted: a.shifted + b.shifted, shifts: a.shifts + b.shifts, copied: a.copied + b.copied,
        routed_edges: a.routed_edges + b.routed_edges, usage_checked: a.usage_checked + b.usage_checked,
        ndr_checked: a.ndr_checked + b.ndr_checked, r6_checked: a.r6_checked + b.r6_checked, r6_segs: a.r6_segs + b.r6_segs, stacked_pins: a.stacked_pins + b.stacked_pins, htree: a.htree + b.htree,
    }
}

// ─── Constructed cases: branches the corpus never reaches ───────────────────────────────────

use std::cell::RefCell;

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
    let mut est = EstimateGrid::new(xg, yg);
    for y in 0..=3 {
        est.update_h(0, 2, y, 500.0); // gap 0: 8 edges × 500 = 4000 → factor 4000/(2·4·1000) = 0.5
        est.update_h(4, 6, y, 0.6); // gap 2: every add truncates to 0 (rounding would not) → unchanged
    }
    let red_h = |x: usize, y: usize| (x == 2 && y == 0) as u16; // gap 1: reduction 1 → 1/8000 of 200 is 0 → floored to 1
    est.update_v(0, 0, 1, 1.0); // y gap 0: one edge of 1…
    let red_v = |x: usize, y: usize| if y == 0 && x < 7 && x > 0 { 1u16 } else { 0 }; // …plus red 1 on six more = 7
    let caps = grid_caps(xg, yg, 0);
    let grid = BrkGrid { est: &mut est, red_h: &red_h, red_v: &red_v, caps: &caps, h_capacity: 1000, v_capacity: 10, via_cost: 0.0 , ndr: &mut NdrLedger::new(1, 1, 1) };
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
    let mut est = EstimateGrid::new(4, 4);
    est.update_h(0, 1, 0, 3.0);
    let caps = grid_caps(4, 4, 3);
    let grid = BrkGrid { est: &mut est, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 3, v_capacity: 3, via_cost: 0.0 , ndr: &mut NdrLedger::new(1, 1, 1) };
    let (x, y) = ([0, 2], [0, 2]);
    let seg = RoutedSegment { seg: Segment { x1: 0, y1: 0, x2: 2, y2: 2, edge_cost: 1 }, x_first: true };
    assert!(net_congestion(&net(&x, &y), &[seg], &grid), "usage 3 on capacity 3 is congested");
}

#[test]
fn net_congestion_walks_the_bend_taken() {
    let mut est = EstimateGrid::new(4, 4);
    est.update_v(0, 0, 1, 5.0); // column x1 — on the y-first path only
    let caps = grid_caps(4, 4, 3);
    let grid = BrkGrid { est: &mut est, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 3, v_capacity: 3, via_cost: 0.0 , ndr: &mut NdrLedger::new(1, 1, 1) };
    let (x, y) = ([0, 2], [0, 2]);
    let s = Segment { x1: 0, y1: 0, x2: 2, y2: 2, edge_cost: 1 };
    assert!(!net_congestion(&net(&x, &y), &[RoutedSegment { seg: s, x_first: true }], &grid));
    assert!(net_congestion(&net(&x, &y), &[RoutedSegment { seg: s, x_first: false }], &grid));
}

/// A net whose old route is congested is rebuilt by `fluteCongest`, from the pins R5 sorted.
#[test]
fn a_congested_net_takes_the_congestion_flute() {
    let mut est = EstimateGrid::new(8, 8);
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
        let mut grid = BrkGrid { est: &mut est, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 1, v_capacity: 1, via_cost: 0.0 , ndr: &mut NdrLedger::new(1, 1, 1) };
        gen_brk_rsmt(r5, &[0], &nets, &mut state, &mut grid, &no_stt, &fake).expect("R5");
    }
    // Another net's usage fills the first segment's path — y-first, the bend an unrouted segment
    // carries — so it is congested once this net is ripped up.
    let first = state[0].seglist[0].seg;
    est.update_v(first.x1, first.y1.min(first.y2), first.y1.max(first.y2), 5.0);
    est.update_h(first.x1, first.x2, first.y2, 5.0);
    let mut grid = BrkGrid { est: &mut est, red_h: &no_red, red_v: &no_red, caps: &caps, h_capacity: 1, v_capacity: 1, via_cost: 0.0 , ndr: &mut NdrLedger::new(1, 1, 1) };
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
