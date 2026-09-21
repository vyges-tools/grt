// SPDX-License-Identifier: Apache-2.0
//! The move pricing — `getWireResistance`, `getWireCost`, `getViaCost`, `getMazeRouteCost3D`.
//!
//! Golden `pricing.json`: EVERY distinct record of the corpus (both cost modes), grouped by
//! technology table, from all three callers — the 3D maze search, layer assignment and the net
//! resistance estimate. Floats are stored as their IEEE bits, so every comparison is exact.

use serde_json::Value;
use vyges_grt::{
    get_maze_route_cost_3d, get_via_cost, get_wire_cost, get_wire_resistance, MoveCost, TechLayers, WireNet,
};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn f64_of(v: &Value) -> f64 {
    f64::from_bits(v.as_u64().expect("bits"))
}

fn f32_of(v: &Value) -> f32 {
    f32::from_bits(v.as_u64().expect("bits") as u32)
}

fn tech(t: &Value) -> TechLayers {
    TechLayers {
        dbu_per_micron: int(&t["dbu"]) as i32,
        width: t["width"].as_array().expect("width").iter().map(|v| int(v) as i32).collect(),
        resistance: t["res"].as_array().expect("res").iter().map(f64_of).collect(),
        via_resistance: t["via"].as_array().expect("via").iter().map(|v| (!v.is_null()).then(|| f64_of(v))).collect(),
    }
}

/// `[ndr, width, minl, maxl]` at `r[i..]` -> the net's side (the NDR width only when it has one).
fn wire_net(r: &[Value], i: usize) -> WireNet {
    WireNet {
        ndr_width: (int(&r[i]) == 1).then(|| int(&r[i + 1]) as i32),
        min_layer: int(&r[i + 2]) as i32,
        max_layer: int(&r[i + 3]) as i32,
    }
}

fn rows<'a>(t: &'a Value, k: &str) -> &'a Vec<Value> {
    t[k].as_array().expect("records")
}

#[derive(Default)]
struct Seen {
    wr: usize,
    wr_ndr: usize,
    wr_out: usize,
    wc_ra: usize,
    vc_ra: usize,
    mc_via: usize,
    mc_wire_ra: usize,
}

fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    for t in g["tables"].as_array().expect("tables") {
        let tl = tech(t);
        let who = format!("{:?}", t["designs"][0]);
        for r in rows(t, "wr") {
            let r = r.as_array().expect("row");
            let (layer, length) = (int(&r[1]) as i32, int(&r[2]) as i32);
            let net = wire_net(r, 3);
            let got = get_wire_resistance(&tl, layer, length, net);
            assert_eq!(got.to_bits(), f32_of(&r[7]).to_bits(), "{who}: getWireResistance {r:?}: {got}");
            seen.wr += 1;
            seen.wr_ndr += net.ndr_width.is_some() as usize;
            seen.wr_out += (layer < net.min_layer || layer > net.max_layer) as usize;
        }
        for r in rows(t, "wc") {
            let r = r.as_array().expect("row");
            let ra = int(&r[1]) == 1;
            let got = get_wire_cost(&tl, ra, int(&r[2]) as i32, int(&r[3]) as i32, wire_net(r, 4));
            assert_eq!(got as i64, int(&r[8]), "{who}: getWireCost {r:?}");
            seen.wc_ra += ra as usize;
        }
        for r in rows(t, "vc") {
            let r = r.as_array().expect("row");
            let ra = int(&r[1]) == 1;
            let got = get_via_cost(&tl, ra, int(&r[2]) as i32, int(&r[3]) as i32);
            assert_eq!(got as i64, int(&r[4]), "{who}: getViaCost {r:?}");
            seen.vc_ra += ra as usize;
        }
        for r in rows(t, "mc") {
            let r = r.as_array().expect("row");
            let mv = MoveCost {
                from_layer: int(&r[1]) as i32,
                to_layer: int(&r[2]) as i32,
                dx: int(&r[3]) as i32,
                dy: int(&r[4]) as i32,
                is_via: int(&r[5]) == 1,
            };
            let ra = int(&r[8]) == 1;
            let got = get_maze_route_cost_3d(&tl, ra, int(&r[6]) as i32, int(&r[7]) as i32, mv, wire_net(r, 9));
            assert_eq!(got.to_bits(), f32_of(&r[13]).to_bits(), "{who}: getMazeRouteCost3D {r:?}: {got}");
            seen.mc_via += mv.is_via as usize;
            seen.mc_wire_ra += (!mv.is_via && ra) as usize;
        }
    }
    seen
}

/// Every distinct pricing record of the corpus.
#[test]
fn the_move_pricing_matches_the_reference() {
    let g = read(&format!("{}/examples/grt_gate/pricing.json", env!("CARGO_MANIFEST_DIR")));
    let s = replay(&g);
    // ⛔ Not vacuous: the branches that make a price differ from the plain one must be in the
    // golden — an NDR width, an out-of-range layer, and resistance-aware wire, via and move costs.
    assert!(s.wr >= 60, "too few wire resistances: {}", s.wr);
    assert!(s.wr_ndr > 0 && s.wr_out > 0, "NDR {} / out-of-range {}", s.wr_ndr, s.wr_out);
    assert!(s.wc_ra >= 40 && s.vc_ra >= 20, "resistance-aware wire {} / via {}", s.wc_ra, s.vc_ra);
    assert!(s.mc_via > 0 && s.mc_wire_ra > 0, "move: via {} / resistance-aware wire {}", s.mc_via, s.mc_wire_ra);
}

// ─── Constructed cases: branches the corpus never reaches ───────────────────────────────────

/// Two routing layers, 1 µm wide at 1000 dbu per micron.
fn two_layers(res: [f64; 2], via: [f64; 1]) -> TechLayers {
    TechLayers { dbu_per_micron: 1000, width: vec![1000, 1000], resistance: res.to_vec(), via_resistance: vec![Some(via[0]), None] }
}

const IN_RANGE: WireNet = WireNet { ndr_width: None, min_layer: 0, max_layer: 1 };

/// ⛔ `getWireCost` returns 0 when layer 0 has NO resistance (`default_resistance <= 0.0`) —
/// without the guard the division is by zero. The corpus's zero-resistance technology is only
/// ever priced in plain mode.
#[test]
fn a_wire_cost_is_zero_when_layer_zero_has_no_resistance() {
    let t = two_layers([0.0, 2.0], [1.0]);
    assert_eq!(get_wire_cost(&t, true, 1, 5000, IN_RANGE), 0);
}

/// ⛔ The wire cost is CAPPED at `BIG_INT`: an out-of-range layer's `1e9` over a layer-0 sheet
/// resistance below one ohm exceeds it. Every captured technology's layer 0 is above one ohm where
/// an out-of-range layer was priced.
#[test]
fn a_wire_cost_is_capped_at_big_int() {
    let t = two_layers([0.38, 0.25], [5.0]);
    let net = WireNet { ndr_width: None, min_layer: 1, max_layer: 1 };
    assert_eq!(get_wire_cost(&t, true, 0, 5000, net), 1_000_000_000);
}

/// ⛔ `getViaResistance` sums into a `float` from `double` terms (`float += double`), rounding once
/// per addition, not per term: cuts of 0.1 and 17.2 total 17.3f (173 units of 0.1f), where summing
/// the narrowed terms gives 17.300001f (174).
#[test]
fn a_via_stack_sums_double_terms_into_a_float() {
    let t = TechLayers {
        dbu_per_micron: 1000,
        width: vec![1000; 3],
        resistance: vec![1.0; 3],
        via_resistance: vec![Some(0.1), Some(17.2), None],
    };
    assert_eq!(get_via_cost(&t, true, 0, 2), 173);
}

/// ⛔ A via move costs `via_cost_` — not 1 — plus its resistance cost. The 3D pass always runs with
/// `via_cost_ = 1`, so only this case separates them.
#[test]
fn a_via_move_costs_via_cost_plus_its_resistance() {
    let t = two_layers([1.0, 1.0], [5.0]);
    let mv = MoveCost { from_layer: 0, to_layer: 1, dx: 0, dy: 0, is_via: true };
    assert_eq!(get_maze_route_cost_3d(&t, true, 3, 1000, mv, IN_RANGE), 4.0);
}

/// ⛔ `getMazeRouteCost3D` returns `1.0f + (float) cost`: the cost is narrowed BEFORE the add, and
/// the sum narrowed again. A cost of 2^24 + 1 narrows to 2^24, and 2^24 + 1 narrows back to 2^24 —
/// adding in `double` would give 2^24 + 2.
#[test]
fn a_wire_move_adds_in_float() {
    let t = two_layers([1.0000001, 1000.0], [1.0]);
    let mv = MoveCost { from_layer: 1, to_layer: 1, dx: 1, dy: 0, is_via: false };
    assert_eq!(get_wire_cost(&t, true, 1, 16_777_218, IN_RANGE), 16_777_217);
    assert_eq!(get_maze_route_cost_3d(&t, true, 1, 16_777_218, mv, IN_RANGE), 16_777_216.0);
}
