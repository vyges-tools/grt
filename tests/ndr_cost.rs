// SPDX-License-Identifier: Apache-2.0
//! The NDR-aware usage cost (`getCostNDRAware`), replayed call by call against the reference.
//!
//! Golden `ndr_cost.json`: every call an NDR net makes, in order, with the lifecycle events of the
//! state (init / copy / clear) per graph instance. The replay THREADS the state through the calls:
//! before each call the engine's capacities, the net's presence on the edge and the edge's overflow
//! count must equal the reference's; then the returned cost and the state after must too.
//!
//! ⚠️ Capacities are set by `updateCap3D` at init, which the capture does not log; each layer's is
//! taken from the first call that touches it, and that call must see it UNTOUCHED (`cap_ndr == cap`).

use serde_json::Value;
use std::collections::{HashMap, HashSet};
use vyges_grt::*;

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn num(v: &Value) -> f64 {
    v.as_f64().expect("number")
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
    overflow_adds: usize,
    overflow_removes: usize,
    no_charge: usize,
    cap_below_zero: usize,
    inits: usize,
    reinits: usize,
    copies: usize,
    clears: usize,
    instances: usize,
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
        let mut names: HashMap<String, usize> = HashMap::new();
        let mut inst: HashMap<i64, Inst> = HashMap::new();
        let fresh = || Inst { ledger: NdrLedger::new(xg, yg, nl), known: HashSet::new() };

        for (i, e) in events.iter().enumerate() {
            let at = format!("{who} event {i}");
            let gi = int(&e["g"]);
            match e["ev"].as_str() {
                Some("init") => {
                    // A repeated init on the same instance is the resize, not a reset.
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
                    let s = inst.entry(gi).or_insert_with(fresh);
                    let (h, x, y) = (e["h"].as_bool().expect("h"), int(&e["x"]) as usize, int(&e["y"]) as usize);
                    let (lo, hi) = (int(&e["min"]) as usize, int(&e["max"]) as usize);
                    let n = names.len();
                    let id = *names.entry(e["net"].as_str().expect("net").to_string()).or_insert(n);
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
                        let (cap, cap_ndr) = (int(&caps[k][0]), num(&caps[k][1]));
                        if s.known.insert((h, l, x, y)) {
                            assert_eq!(cap_ndr, cap as f64, "{at}: layer {l} first touched with cap_ndr already drawn");
                            s.ledger.update_cap_3d(x, y, l, h, cap as f64);
                        }
                        let c = s.ledger.cap(h, l, x, y);
                        assert_eq!((c.cap as i64, c.cap_ndr), (cap, cap_ndr), "{at}: layer {l} capacity before the call");
                    }
                    assert_eq!(s.ledger.has_net(h, x, y, id), e["present"].as_bool().expect("present"), "{at}: presence before");
                    assert_eq!(s.ledger.overflow(h, x, y) as i64, int(&e["overflow"]), "{at}: overflow before");

                    let amount = num(&e["amount"]);
                    let cost = s.ledger.get_cost_ndr_aware(&net, x, y, amount, h);
                    let out = &e["out"];
                    assert_eq!(cost, num(&out["cost"]), "{at}: cost of {amount} for {} on {}{x},{y}", net.id, if h { "H" } else { "V" });
                    assert_eq!(s.ledger.has_net(h, x, y, id), out["present"].as_bool().expect("present"), "{at}: presence after");
                    assert_eq!(s.ledger.overflow(h, x, y) as i64, int(&out["overflow"]), "{at}: overflow after");
                    let after: Vec<f64> = arr(&out["cap_ndr"]).iter().map(num).collect();
                    let ours: Vec<f64> = (lo..=hi).map(|l| s.ledger.cap(h, l, x, y).cap_ndr).collect();
                    assert_eq!(ours, after, "{at}: cap_ndr after");

                    seen.calls += 1;
                    match e["kind"].as_str() {
                        Some("E") => seen.est_calls += 1,
                        Some("U") => seen.usage_calls += 1,
                        k => panic!("{at}: caller kind {k:?}"),
                    }
                    if amount < 0.0 { seen.removes += 1 } else { seen.adds += 1 }
                    seen.overflow_adds += (cost > OVERFLOW_COST_MULTIPLIER) as usize;
                    seen.overflow_removes += (cost < -OVERFLOW_COST_MULTIPLIER) as usize;
                    seen.no_charge += (cost == 0.0) as usize;
                    seen.cap_below_zero += ours.iter().any(|&c| c < 0.0) as usize;
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
fn the_ndr_aware_cost_matches_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/ndr_cost.json", env!("CARGO_MANIFEST_DIR"))));
    eprintln!("{s:?}");
    assert!(s.calls > 0 && s.overflow_adds > 0 && s.overflow_removes > 0 && s.no_charge > 0 && s.est_calls > 0 && s.usage_calls > 0, "{s:?}");
}

/// GRT_NDR_COST_FULL=/path/to/nc-all.json cargo test --release --test ndr_cost -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_ndr_aware_cost_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_NDR_COST_FULL").expect("set GRT_NDR_COST_FULL");
    eprintln!("exhaustive: {:?}", replay(&read(&path)));
}

// ─── Constructed cases: branches the corpus never reaches ───────────────────────────────────

fn ndr(id: usize, ec: i8) -> NdrCostNet {
    NdrCostNet { id, edge_cost: ec, min_layer: 0, max_layer: 1, layer_edge_cost: Some(vec![ec, ec]), soft_ndr: false }
}

fn ledger() -> NdrLedger {
    let mut l = NdrLedger::new(4, 4, 2);
    for x in 0..3 {
        for y in 0..4 {
            l.update_cap_3d(x, y, 0, true, 10.0);
            l.update_cap_3d(x, y, 1, true, 10.0);
        }
    }
    l
}

/// An edge-cost-1 net's amount passes unchanged and leaves no trace — the corpus capture logs only
/// NDR nets, so this is the one branch it cannot see.
#[test]
fn an_edge_cost_1_net_passes_through() {
    let mut l = ledger();
    let before = l.clone();
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 1), 1, 1, 0.5, true), 0.5);
    assert_eq!(l, before);
}

/// ⛔ Only a NEGATIVE amount removes: zero is an add, and charges the full edge cost.
#[test]
fn a_zero_amount_is_an_add() {
    let mut l = ledger();
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 0.0, true), 3.0);
    assert!(l.has_net(true, 1, 1, 0));
    // …and draws no layer down: `updateNDRCapLayer` draws only for `amount >= 0` inside the loop,
    // so the first layer with room still pays. (The below-zero fallback needs `amount > 0`.)
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr, 7.0);
}

/// ⛔ A re-init is a RESIZE: on a grid of the same size the NDR sets and the capacities survive
/// (until `updateCap3D` rewrites the capacities); the overflow counts do not.
#[test]
fn a_reinit_keeps_the_sets_and_capacities_but_not_the_overflow() {
    let mut l = ledger();
    let big = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(1, 20) };
    l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true);
    assert_eq!(l.get_cost_ndr_aware(&big, 1, 1, 1.0, true), 2000.0, "no layer has room for 20: overflow");
    assert_eq!(l.overflow(true, 1, 1), 1);
    l.init_cap_3d(4, 4, 2);
    assert!(l.has_net(true, 1, 1, 0) && l.has_net(true, 1, 1, 1), "sets survive");
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr, 10.0 - 3.0 - 20.0, "capacities survive until updateCap3D");
    assert_eq!(l.overflow(true, 1, 1), 0, "Graph2D::init reset the overflow");
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true), 0.0, "net 0 is still counted as present");
}

/// `copyRoutingStateFrom` takes capacities and overflow counts with the edges; the NDR sets only
/// when asked, and otherwise clears its own.
#[test]
fn a_copy_takes_the_sets_only_when_asked() {
    let mut src = ledger();
    src.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true);
    let mut with = ledger();
    with.get_cost_ndr_aware(&ndr(5, 3), 2, 2, 1.0, true);
    let mut without = with.clone();
    with.copy_routing_state_from(&src, true);
    without.copy_routing_state_from(&src, false);
    assert!(with.has_net(true, 1, 1, 0) && !with.has_net(true, 2, 2, 5));
    assert!(!without.has_net(true, 1, 1, 0) && !without.has_net(true, 2, 2, 5));
    assert_eq!(without.cap(true, 0, 1, 1).cap_ndr, 7.0, "capacities come across either way");
}

/// `clearNDRnets` empties the sets and nothing else.
#[test]
fn clearing_keeps_capacities_and_overflow() {
    let mut l = ledger();
    let big = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(1, 20) };
    l.get_cost_ndr_aware(&big, 1, 1, 1.0, true);
    l.clear_ndr_nets();
    assert!(!l.has_net(true, 1, 1, 1));
    assert_eq!((l.overflow(true, 1, 1), l.cap(true, 0, 1, 1).cap_ndr), (1, -10.0));
}

/// ⛔ Removing a net that is not on the edge charges nothing and changes nothing — the reference's
/// comment: the second half-cost removal of the first routing pass. No corpus call does it.
#[test]
fn removing_an_absent_net_charges_nothing() {
    let mut l = ledger();
    let before = l.clone();
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, -1.5, true), 0.0);
    assert_eq!(l, before);
}
