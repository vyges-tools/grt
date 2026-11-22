// SPDX-License-Identifier: Apache-2.0
//! The NDR-aware usage cost (`Graph2D::getCostNDRAware`) and its deficit ledger, at the f11b4276 pin.
//!
//! Golden `examples/grt_gate/ndr_cost.json` (captured at the pin): every call an NDR net makes, in
//! order, with the lifecycle events of the state (init / copy / clear) per graph instance. The
//! replay THREADS the ledger through the calls: before each call the engine's layer capacities and
//! NDR capacities, the net's presence and record on the edge, the edge's deficit and the total must
//! equal the reference's; then the returned cost, the same state after, and the edge's whole ledger
//! (every NDR net's record, in database-id order — what a rip-up's refit leaves) must too.
//!
//! ⚠️ Capacities are set by `updateCap3D` / `addCap3D`, which the capture does not log: each layer's
//! is taken from the first call that touches it (with no NDR debit yet), and a later change of the
//! raw capacity is applied as `addCap3D` (the debit kept) — the NDR capacity must still match.
//! The constructed cases below take branches the corpus does not reach.

use serde_json::Value;
use std::collections::{HashMap, HashSet};
use vyges_grt::estimate::Usage2d as _;
use vyges_grt::*;

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}
fn int(v: &Value) -> i64 {
    v.as_i64().unwrap_or_else(|| panic!("int: {v}"))
}
fn num(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("num: {v}"))
}
fn arr(v: &Value) -> &Vec<Value> {
    v.as_array().expect("array")
}

struct Inst {
    ledger: NdrLedger,
    known: HashSet<(bool, usize, usize, usize)>,
}

#[derive(Default, Debug)]
struct Seen {
    runs: usize,
    calls: usize,
    est_calls: usize,
    usage_calls: usize,
    adds: usize,
    removes: usize,
    no_charge: usize,
    deficit_adds: usize,
    deficit_removes: usize,
    refits: usize,
    shared_edges: usize,
    cap_adjusts: usize,
    inits: usize,
    reinits: usize,
    copies: usize,
    clears: usize,
    instances: usize,
}

/// The net's record on the edge as the capture prints it: `[layer, amount, deficit]`.
fn record(u: NdrUsage) -> Vec<i64> {
    vec![i64::from(u.layer), i64::from(u.amount), i64::from(u.deficit)]
}

fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        let events = arr(&r["events"]);
        let calls = || events.iter().filter(|e| e["ev"] == "call");
        let xg = calls().map(|e| int(&e["x"])).max().unwrap_or(0) as usize + 2;
        let yg = calls().map(|e| int(&e["y"])).max().unwrap_or(0) as usize + 2;
        let nl = calls().map(|e| int(&e["max"])).max().unwrap_or(0) as usize + 1;
        // Net ids in order of appearance; each one's database id (the refit's order).
        let mut names: HashMap<String, usize> = HashMap::new();
        let mut db_ids: Vec<u32> = Vec::new();
        let mut inst: HashMap<i64, Inst> = HashMap::new();
        let fresh = || Inst { ledger: NdrLedger::new(xg, yg, nl), known: HashSet::new() };

        for (i, e) in events.iter().enumerate() {
            let at = format!("{who} event {i}");
            let gi = int(&e["g"]);
            match e["ev"].as_str() {
                Some("init") => {
                    // A repeated init on the same instance is the resize (records, deficits and
                    // debits dropped, capacities kept), not a new graph.
                    match inst.get_mut(&gi) {
                        Some(s) => {
                            s.ledger.init_cap_3d(xg, yg, nl);
                            s.known.clear();
                            seen.reinits += 1;
                        }
                        None => {
                            inst.insert(gi, fresh());
                        }
                    }
                    seen.inits += 1;
                }
                Some("copy") => {
                    let from = int(&e["from"]);
                    let (l, k) = inst.get(&from).map(|s| (s.ledger.clone(), s.known.clone())).unwrap_or_else(|| {
                        let f = fresh();
                        (f.ledger, f.known)
                    });
                    let s = inst.entry(gi).or_insert_with(fresh);
                    s.ledger.copy_routing_state_from(&l, e["include"].as_bool().expect("include"));
                    s.known = k;
                    seen.copies += 1;
                }
                Some("clear") => {
                    inst.entry(gi).or_insert_with(fresh).ledger.clear_ndr_nets();
                    seen.clears += 1;
                }
                Some("call") => {
                    let (h, x, y) = (e["h"].as_bool().expect("h"), int(&e["x"]) as usize, int(&e["y"]) as usize);
                    let (lo, hi) = (int(&e["min"]) as usize, int(&e["max"]) as usize);
                    let n = names.len();
                    let id = *names.entry(e["net"].as_str().expect("net").to_string()).or_insert(n);
                    let db = int(&e["db_id"]) as u32;
                    if id == db_ids.len() {
                        db_ids.push(db);
                    }
                    assert_eq!(db_ids[id], db, "{at}: one net, one database id");
                    let s = inst.entry(gi).or_insert_with(fresh);
                    s.ledger.set_db_ids(db_ids.clone());
                    let net = NdrCostNet {
                        id,
                        edge_cost: int(&e["ec"]) as i8,
                        min_layer: lo,
                        max_layer: hi,
                        layer_edge_cost: Some(arr(&e["lec"]).iter().map(|v| int(v) as i8).collect()),
                        soft_ndr: e["soft"].as_bool().expect("soft"),
                    };
                    let caps = arr(&e["caps"]);
                    for (k, l) in (lo..=hi).enumerate() {
                        let (cap, cap_ndr) = (int(&caps[k][0]), int(&caps[k][1]));
                        if s.known.insert((h, l, x, y)) {
                            assert_eq!(cap_ndr, cap, "{at}: layer {l} first touched with an NDR debit already taken");
                            s.ledger.update_cap_3d(x, y, l, h, cap as f64);
                        } else if i64::from(s.ledger.cap(h, l, x, y).cap) != cap {
                            let delta = cap as i32 - i32::from(s.ledger.cap(h, l, x, y).cap);
                            s.ledger.add_cap_3d(x, y, l, h, delta);
                            seen.cap_adjusts += 1;
                        }
                        let c = s.ledger.cap(h, l, x, y);
                        assert_eq!((i64::from(c.cap), i64::from(c.cap_ndr())), (cap, cap_ndr), "{at}: layer {l} capacity before the call");
                    }
                    let state = |l: &NdrLedger| (l.has_net(h, x, y, id), i64::from(l.edge_ndr_deficit(h, x, y)), i64::from(l.total_ndr_deficit()));
                    let want = |v: &Value| (v["present"].as_bool().expect("present"), int(&v["edge_deficit"]), int(&v["total_deficit"]));
                    assert_eq!(state(&s.ledger), want(e), "{at}: (present, edge deficit, total) before");
                    let rec_want = e.get("usage").map(|u| arr(u).iter().map(int).collect::<Vec<_>>());
                    assert_eq!(s.ledger.usage(h, x, y, id).map(record), rec_want, "{at}: the net's record before");

                    let amount = num(&e["amount"]);
                    let others_before: Vec<(usize, NdrUsage)> =
                        (0..names.len()).filter(|&o| o != id).filter_map(|o| s.ledger.usage(h, x, y, o).map(|u| (o, u))).collect();
                    let cost = s.ledger.get_cost_ndr_aware(&net, x, y, amount, h);
                    let out = &e["out"];
                    assert_eq!(cost, num(&out["cost"]), "{at}: cost of {amount} for {} on {}{x},{y}", net.id, if h { "H" } else { "V" });
                    assert_eq!(state(&s.ledger), want(out), "{at}: (present, edge deficit, total) after");
                    let after: Vec<i64> = arr(&out["cap_ndr"]).iter().map(int).collect();
                    let ours: Vec<i64> = (lo..=hi).map(|l| i64::from(s.ledger.cap(h, l, x, y).cap_ndr())).collect();
                    assert_eq!(ours, after, "{at}: NDR capacity after");
                    let mut ledger: Vec<Vec<i64>> = (0..names.len())
                        .filter_map(|o| s.ledger.usage(h, x, y, o).map(|u| [vec![i64::from(db_ids[o])], record(u)].concat()))
                        .collect();
                    ledger.sort();
                    let want_ledger: Vec<Vec<i64>> = arr(&out["ledger"]).iter().map(|row| arr(row).iter().map(int).collect()).collect();
                    assert_eq!(ledger, want_ledger, "{at}: the edge's ledger after (database id, layer, amount, deficit)");

                    seen.calls += 1;
                    match e["kind"].as_str() {
                        Some("E") => seen.est_calls += 1,
                        Some("U") => seen.usage_calls += 1,
                        k => panic!("{at}: caller kind {k:?}"),
                    }
                    if amount < 0.0 { seen.removes += 1 } else { seen.adds += 1 }
                    seen.no_charge += (cost == 0.0) as usize;
                    let (d0, d1) = (int(&e["edge_deficit"]), int(&out["edge_deficit"]));
                    seen.deficit_adds += (amount >= 0.0 && d1 > d0) as usize;
                    seen.deficit_removes += (amount < 0.0 && d1 < d0) as usize;
                    seen.refits += others_before.iter().any(|&(o, u)| s.ledger.usage(h, x, y, o) != Some(u)) as usize;
                    seen.shared_edges += (want_ledger.len() >= 2) as usize;
                }
                k => panic!("{at}: event {k:?}"),
            }
        }
        seen.instances += inst.len();
        seen.runs += 1;
    }
    seen
}

#[test]
fn the_ndr_aware_cost_and_its_ledger_match_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/ndr_cost.json", env!("CARGO_MANIFEST_DIR"))));
    eprintln!("{s:?}");
    // Every ledger rule witnessed: a deficit recorded and returned, a refit, two NDR nets on an edge.
    assert!(s.calls > 0 && s.est_calls > 0 && s.usage_calls > 0 && s.adds > 0 && s.removes > 0 && s.no_charge > 0
            && s.deficit_adds > 0 && s.deficit_removes > 0 && s.refits > 0 && s.shared_edges > 0, "{s:?}");
}

/// GRT_NDR_COST_FULL=/path/to/nc-all.json cargo test --release --test ndr_cost -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_ndr_aware_cost_and_its_ledger_match_the_reference_exhaustively() {
    let path = std::env::var("GRT_NDR_COST_FULL").expect("set GRT_NDR_COST_FULL");
    eprintln!("exhaustive: {:?}", replay(&read(&path)));
}

// ─── Constructed cases: branches the corpus never reaches ───────────────────────────────────

fn ndr(id: usize, ec: i8) -> NdrCostNet {
    NdrCostNet { id, edge_cost: ec, min_layer: 0, max_layer: 1, layer_edge_cost: Some(vec![ec, ec]), soft_ndr: false }
}

fn ledger_with(cap: f64) -> NdrLedger {
    let mut l = NdrLedger::new(4, 4, 2);
    for x in 0..3 {
        for y in 0..4 {
            l.update_cap_3d(x, y, 0, true, cap);
            l.update_cap_3d(x, y, 1, true, cap);
        }
    }
    l
}

fn ledger() -> NdrLedger {
    ledger_with(10.0)
}

/// An edge-cost-1 net's amount passes unchanged and leaves no trace.
#[test]
fn an_edge_cost_1_net_passes_through() {
    let mut l = ledger();
    let before = l.clone();
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 1), 1, 1, 0.5, true), 0.5);
    assert_eq!(l, before);
}

/// ⛔ Only a NEGATIVE amount removes: zero is an add, charges the full edge cost, and debits the
/// first layer with room.
#[test]
fn a_zero_amount_is_an_add() {
    let mut l = ledger();
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 0.0, true), 3.0);
    assert!(l.has_net(true, 1, 1, 0));
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr(), 7);
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true), 0.0, "a second add counts nothing");
}

/// ⛔ Upstream #11963: where no layer has room the net is charged its TRUE edge cost (no 100×), its
/// lowest layer is debited anyway, and the tracks it misses on its best layer —
/// `min over layers of max(0, cost − max(0, NDR capacity))` — are its deficit on the edge and in
/// the total. The rip-up gives the deficit back with the record.
#[test]
fn a_net_that_fits_no_layer_records_its_deficit_not_an_overflow_charge() {
    let mut l = ledger();
    let big = NdrCostNet { layer_edge_cost: Some(vec![12, 14]), ..ndr(1, 14) };
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true), 3.0);
    // Layer 0 has 7 left, layer 1 10: the net misses 12 − 7 = 5 there, 14 − 10 = 4 here.
    assert_eq!(l.compute_ndr_deficit(&big, 1, 1, true), 4);
    assert_eq!(l.ndr_deficit_if_routed(&big, 1, 1, true), 4, "the edge's 0, plus the net's own");
    assert_eq!(l.get_cost_ndr_aware(&big, 1, 1, 1.0, true), 14.0);
    assert_eq!(l.usage(true, 1, 1, 1), Some(NdrUsage { layer: 0, amount: 12, deficit: 4 }));
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit()), (4, 4));
    assert!(l.has_ndr_deficit(true, 1, 1));
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr(), -5, "the lowest layer is debited anyway");
    assert_eq!(l.ndr_deficit(&big, 1, 1, true), 4);
    assert_eq!(l.ndr_deficit_if_routed(&big, 1, 1, true), 4, "a net on the edge sees the edge's deficit only");
    assert_eq!(l.ndr_deficit_if_routed(&ndr(2, 3), 1, 1, true), 4, "another net that fits still sees it");
    assert_eq!(l.ndr_deficit_if_routed(&ndr(2, 1), 1, 1, true), 0, "a regular net never does");
    assert_eq!(l.get_cost_ndr_aware(&big, 1, 1, -1.0, true), -14.0);
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit(), l.cap(true, 0, 1, 1).cap_ndr()), (0, 0, 7));
}

/// `computeNDRDeficit` with no layer in the net's range: nothing can host it, the deficit is its
/// whole edge cost.
#[test]
fn no_layer_in_range_misses_the_whole_edge_cost() {
    let l = ledger();
    let empty = NdrCostNet { min_layer: 1, max_layer: 0, ..ndr(0, 6) };
    assert_eq!(l.compute_ndr_deficit(&empty, 1, 1, true), 6);
}

/// ⛔ Upstream #11963 (74d34bd075 `refitNDRDeficitNets`): when a net leaves an edge that still has a
/// deficit, every deficit net there is released and reserved again in DATABASE-ID order, so the
/// freed tracks go to the lowest database id — not the lowest router id.
#[test]
fn a_leaving_net_frees_its_layer_for_the_deficit_nets_in_database_id_order() {
    let mut l = ledger_with(5.0);
    // Router ids 0..4; database ids: net 2 is 9, net 3 is 1.
    l.set_db_ids(vec![0, 1, 9, 1]);
    let n = |id| ndr(id, 5);
    for id in 0..4 {
        l.get_cost_ndr_aware(&n(id), 1, 1, 1.0, true);
    }
    // 0 on layer 0, 1 on layer 1; 2 and 3 fit nowhere.
    assert_eq!(l.usage(true, 1, 1, 2).map(|u| u.deficit), Some(5));
    assert_eq!(l.usage(true, 1, 1, 3).map(|u| u.deficit), Some(5));
    assert_eq!(l.total_ndr_deficit(), 10);
    l.get_cost_ndr_aware(&n(1), 1, 1, -1.0, true);
    assert_eq!(l.usage(true, 1, 1, 3), Some(NdrUsage { layer: 1, amount: 5, deficit: 0 }), "database id 1 refits first");
    assert_eq!(l.usage(true, 1, 1, 2).map(|u| u.deficit), Some(5));
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit()), (5, 5));
}

/// Upstream #11963: `initCap3D` resizes the capacities (kept where the extents overlap) and then
/// `initNDRnets` drops every record, deficit and debit — a new run's net must not be taken for
/// one of the last run's.
#[test]
fn a_reinit_drops_the_records_deficits_and_debits_but_keeps_the_capacities() {
    let mut l = ledger();
    let big = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(1, 20) };
    l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true);
    l.get_cost_ndr_aware(&big, 1, 1, 1.0, true);
    assert!(l.total_ndr_deficit() > 0);
    l.init_cap_3d(4, 4, 2);
    assert!(!l.has_net(true, 1, 1, 0) && !l.has_net(true, 1, 1, 1));
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit()), (0, 0));
    assert_eq!((l.cap(true, 0, 1, 1).cap, l.cap(true, 0, 1, 1).cap_ndr()), (10, 10));
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true), 3.0, "net 0 is new on the edge");
}

/// `copyRoutingStateFrom` takes the capacities with the edges, and the NDR state (records, debits,
/// deficits) only when asked; without it the three are dropped together.
#[test]
fn a_copy_takes_the_ndr_state_only_when_asked() {
    let mut src = ledger();
    src.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true);
    src.get_cost_ndr_aware(&NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(1, 20) }, 1, 1, 1.0, true);
    let mut with = ledger();
    with.get_cost_ndr_aware(&ndr(5, 3), 2, 2, 1.0, true);
    let mut without = with.clone();
    with.copy_routing_state_from(&src, true);
    without.copy_routing_state_from(&src, false);
    assert!(with.has_net(true, 1, 1, 0) && !with.has_net(true, 2, 2, 5));
    assert!(!without.has_net(true, 1, 1, 0) && !without.has_net(true, 2, 2, 5));
    assert_eq!(with.cap(true, 0, 1, 1).cap_ndr(), -13, "with the records, the debits come across");
    assert_eq!(without.cap(true, 0, 1, 1).cap_ndr(), 10, "without them, the NDR capacity is the capacity");
    assert_eq!((with.total_ndr_deficit() > 0, without.total_ndr_deficit(), without.edge_ndr_deficit(true, 1, 1)), (true, 0, 0));
}

/// `clearNDRnets` drops the records, the per-edge deficits and their total (#11963 1fe08158b2);
/// the debits stay.
#[test]
fn clearing_drops_records_and_deficits_but_keeps_the_debits() {
    let mut l = ledger();
    let big = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(1, 20) };
    l.get_cost_ndr_aware(&big, 1, 1, 1.0, true);
    l.clear_ndr_nets();
    assert!(!l.has_net(true, 1, 1, 1));
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit(), l.cap(true, 0, 1, 1).cap_ndr()), (0, 0, -10));
}

/// `addCap3D` moves the capacity; what NDR nets reserved stays reserved.
#[test]
fn an_adjustment_keeps_the_debit() {
    let mut l = ledger();
    l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true);
    l.add_cap_3d(1, 1, 0, true, -4);
    assert_eq!((l.cap(true, 0, 1, 1).cap, l.cap(true, 0, 1, 1).cap_ndr()), (6, 3));
    l.update_cap_3d(1, 1, 0, true, 8.0);
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr(), 8, "updateCap3D drops the debit");
}

/// ⛔ Removing a net that is not on the edge charges nothing and changes nothing — the reference's
/// comment: the second half-cost removal of the first routing pass.
#[test]
fn removing_an_absent_net_charges_nothing() {
    let mut l = ledger();
    let before = l.clone();
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, -1.5, true), 0.0);
    assert_eq!(l, before);
}

/// `computeNDRDeficit`: a layer other NDR nets already oversubscribed has NO free track — the net
/// misses its whole width there, not more.
#[test]
fn an_oversubscribed_layer_counts_as_no_free_track() {
    let mut l = ledger_with(5.0);
    let one = |id| NdrCostNet { max_layer: 0, layer_edge_cost: Some(vec![5]), ..ndr(id, 5) };
    l.get_cost_ndr_aware(&one(0), 1, 1, 1.0, true);
    l.get_cost_ndr_aware(&one(1), 1, 1, 1.0, true);
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr(), -5);
    assert_eq!(l.compute_ndr_deficit(&one(2), 1, 1, true), 5);
}

/// A 4 × 4 graph: every 2D capacity 10, every layer 5 NDR tracks over two layers.
fn graph() -> Graph2d {
    let mut g = Graph2d::new(4, 4, 2);
    g.cap_h = vec![10; 16];
    g.cap_v = vec![10; 16];
    for x in 0..4 {
        for y in 0..4 {
            for l in 0..2 {
                g.ndr.update_cap_3d(x, y, l, true, 5.0);
                g.ndr.update_cap_3d(x, y, l, false, 5.0);
            }
        }
    }
    g
}

/// ⛔ Upstream #11963 `getUsageNDRAware`: a net that would leave the edge short of NDR tracks sees
/// the edge as at least `capacity − 1 + 100 × deficit`, with the EDGE's own 2D capacity (#11963
/// 01b0fe5704, `getCapH(x, y)`, not `h_capacity_`); a net that fits, and every regular net, sees
/// the usage unchanged, and a usage above the floor is kept.
#[test]
fn a_net_that_would_miss_tracks_sees_the_edge_as_overflowed() {
    let mut g = graph();
    let wide = NdrCostNet { layer_edge_cost: Some(vec![6, 6]), ..ndr(1, 6) };
    assert_eq!(g.ndr.ndr_deficit_if_routed(&wide, 1, 1, true), 1);
    assert_eq!(g.usage_ndr_aware(&wide, 1, 1, true, 3.0), 10.0 - 1.0 + 100.0);
    assert_eq!(g.usage_ndr_aware(&wide, 1, 1, true, 500.0), 500.0, "max, not replace");
    assert_eq!(g.usage_ndr_aware(&ndr(2, 3), 1, 1, true, 3.0), 3.0, "a net that fits");
    assert_eq!(g.usage_ndr_aware(&ndr(2, 1), 1, 1, true, 3.0), 3.0, "a regular net");
    g.cap_h[4 + 1] = 4; // (1, 1)
    assert_eq!(g.usage_ndr_aware(&wide, 1, 1, true, 3.0), 4.0 - 1.0 + 100.0, "the edge's own capacity");
    g.cap_v[4 + 2] = 7; // (2, 1)
    assert_eq!(g.usage_ndr_aware(&wide, 2, 1, false, 0.0), 7.0 - 1.0 + 100.0, "the vertical edge's");
}

/// ⛔ Upstream #11963: the pattern routes price each edge through `getEstUsageRedH/V(net, …)`, so an
/// NDR net turns its L away from edges where it would miss tracks; a regular net, seeing the same
/// empty grid, keeps the tie-break bend (x first).
#[test]
fn an_l_route_turns_away_from_where_the_net_would_miss_tracks() {
    let mut g = graph();
    // Row 0 (the x-first L's horizontal run for a (0,0)→(2,2) segment) has 2 NDR tracks a layer.
    for x in 0..2 {
        for l in 0..2 {
            g.ndr.update_cap_3d(x, 0, l, true, 2.0);
        }
    }
    let red = |_: usize, _: usize| 0u16;
    let route = |g: &mut Graph2d, net: &NdrCostNet| {
        let before = g.clone();
        let mut seg = RoutedSegment { seg: vyges_grt::rsmt::Segment { x1: 0, y1: 0, x2: 2, y2: 2, edge_cost: net.edge_cost }, x_first: false };
        route_seg_l_first_time(&mut g.for_net(net), &mut seg, 0.0, 0.0, &red, &red);
        *g = before;
        seg.x_first
    };
    assert!(route(&mut g, &ndr(1, 1)), "a regular net keeps the tie bend");
    assert!(!route(&mut g, &NdrCostNet { layer_edge_cost: Some(vec![3, 3]), ..ndr(1, 3) }), "the NDR net goes y first");
}

/// A graph with an NDR deficit on horizontal edge (1, 1) and vertical edge (2, 0): a net of 20
/// NDR tracks fits neither layer of 5 there.
fn graph_with_deficits() -> Graph2d {
    let mut g = graph();
    let fat = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(9, 20) };
    g.for_net(&fat).update_usage_h(1, 1, 1.0);
    g.for_net(&fat).update_usage_v(2, 0, 1.0);
    assert!(g.ndr.has_ndr_deficit(true, 1, 1) && g.ndr.has_ndr_deficit(false, 2, 0));
    g
}

/// ⛔ Upstream #11963 `newRipupCheck`: an NDR net (edge cost above 1) is torn up off an edge where
/// the NDR nets fit no single layer, even with the usage far under the threshold; a regular net
/// on the same route is not. Both directions are read at the walk's lower coordinate.
#[test]
fn the_maze_gate_tears_an_ndr_net_off_an_edge_with_a_deficit() {
    let g = graph_with_deficits();
    let zero = |_: i32, _: i32| 0.0;
    let mut regular = g.clone();
    let mut wide = g.clone();
    // (2,1) → (1,1) → (1,2): horizontal (1,1) walked right to left.
    let h_route = [(2, 1), (1, 1), (1, 2)];
    // (2,1) → (2,0): vertical (2,0) walked downwards.
    let v_route = [(3, 1), (2, 1), (2, 0)];
    for route in [&h_route[..], &v_route[..]] {
        assert_eq!(new_ripup_check(&mut regular.for_net(&ndr(3, 1)), route, 2, 0, (10, 10), 1, None, &zero, &zero), None);
        assert_eq!(new_ripup_check(&mut wide.for_net(&ndr(3, 3)), route, 2, 0, (10, 10), 3, None, &zero, &zero), Some(RipupReason::Congested));
    }
}

/// ⛔ Upstream #11963 `newRipupCongestedL`: the same for the L rip-up — an NDR net leaves an L whose
/// checked column or row has an NDR deficit, though no edge is over capacity.
#[test]
fn the_l_rip_up_takes_an_ndr_net_off_a_deficit() {
    let cap = |_: i32, _: i32, _: bool| 10;
    // (0,0) → (2,1) y first checks column 0, then row 1: the deficit at (1,1).
    // (1,0) → (2,1) x first checks column 2: the deficit at (2,0).
    for (p1, p2, x_first) in [((0, 0), (2, 1), false), ((1, 0), (2, 1), true)] {
        for (ec, rips) in [(1, false), (3, true)] {
            let mut g = graph_with_deficits();
            let mut statuses = vec![2i16, 2];
            let got = new_ripup_congested_l(&mut g.for_net(&ndr(3, ec)), &mut statuses, (0, 1), p1, p2, x_first, 2, ec, &cap);
            assert_eq!(got, rips, "{p1:?}→{p2:?} x_first {x_first} edge cost {ec}");
        }
    }
}

/// ⛔ Upstream #11963 `getOverflow2D` / `getOverflow2Dmaze`: the tracks the NDR nets miss count as
/// overflow too — added to the total, not to the maximum, and not to the usage.
#[test]
fn the_overflow_totals_count_the_ndr_deficit() {
    // Committed: the 20-track net on two edges of capacity 10 → 10 + 10 over, 15 + 15 missed.
    let g = graph_with_deficits();
    let s = g.get_overflow_2d_maze();
    assert_eq!((s.total_overflow, s.max_overflow, s.total_usage), (10 + 10 + 15 + 15, 10, 40));
    // Estimated: the same net estimated on one edge.
    let mut g = graph();
    let fat = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(9, 20) };
    g.for_net(&fat).update_h(1, 2, 1, 1.0);
    let s = g.get_overflow_2d();
    assert_eq!((s.total_overflow, s.max_overflow), (10 + 15, 10));
}

/// ⛔ Upstream #11963 run(): R17 (demote the congested NDR nets) also fires when an NDR deficit is
/// left though the 2D and 3D totals match; the 3D maze passes run when the 2D overflow is ALL NDR
/// deficit (`canRunMaze3D`: `overflow_2d == getTotalNDRDeficit()`), not only at zero.
#[test]
fn run_demotes_on_a_left_deficit_and_runs_3d_maze_past_one() {
    use vyges_grt::run::{can_run_maze_3d, ndr_mismatch};
    assert!(!ndr_mismatch(7, 7, 0));
    assert!(ndr_mismatch(7, 7, 1), "the totals match, a deficit is left");
    assert!(ndr_mismatch(7, 6, 0));
    assert!(can_run_maze_3d(0, 0));
    assert!(can_run_maze_3d(5, 5), "all of it is deficit");
    assert!(!can_run_maze_3d(6, 5), "one track of usage overflow");
}
