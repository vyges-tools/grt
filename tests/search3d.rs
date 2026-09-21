// SPDX-License-Identifier: Apache-2.0
//! R18e — the 3D maze search, and R18f — its backtrace.
//!
//! Golden `search3d.json`, both cost modes: sampled searches with their inputs (region, per-layer
//! direction, move prices, admission bits, layer range, original length, mode, detour penalty,
//! seeds, destinations) and outputs (pop order, crossing, every reached cell's final state).

use std::collections::HashMap;

use serde_json::Value;
use vyges_grt::{backtrace_3d, maze_search_3d, Cell3, CellState, Dir3, Point3D, Recovery, Search3DInputs};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn golden() -> Value {
    read(&format!("{}/examples/grt_gate/search3d.json", env!("CARGO_MANIFEST_DIR")))
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn dir(d: i64) -> Dir3 {
    [Dir3::North, Dir3::East, Dir3::South, Dir3::West, Dir3::Origin, Dir3::Up, Dir3::Down][d as usize]
}

fn replay(g: &Value) -> (usize, usize) {
    let records = g["records"].as_array().expect("records");
    let mut pops_total = 0;
    for r in records {
        let who = format!("{} net {} edge {}", r["design"], r["net"], r["edge"]);
        let (xr, hv) = (int(&r["xr"]), int(&r["hv"]));
        let decode = |off: i64| -> Cell3 { ((off / hv) as i16, ((off % hv) / xr) as i16, ((off % hv) % xr) as i16) };
        let reg: Vec<i32> = r["region"].as_array().expect("region").iter().map(|v| int(v) as i32).collect();
        let (x1, y1) = (reg[0], reg[2]);
        let layers = r["L"].as_array().expect("L");
        let horizontal: Vec<bool> = layers.iter().map(|e| int(&e["hz"]) == 1).collect();
        let bits: Vec<Vec<Vec<u8>>> = layers.iter().map(|e| {
            e["bits"].as_array().expect("bits").iter().map(|b| b.as_str().expect("s").bytes().collect()).collect()
        }).collect();
        let price = |e: &Value, k: &str| e.get(k).and_then(Value::as_f64).map(|v| v as f32);
        let wire: Vec<f32> = layers.iter().map(|e| price(e, "wire").expect("wire")).collect();
        let down: Vec<Option<f32>> = layers.iter().map(|e| price(e, "down")).collect();
        let up: Vec<Option<f32>> = layers.iter().map(|e| price(e, "up")).collect();

        let admits = |l: i16, x: i32, y: i32| -> bool {
            let b = &bits[l as usize];
            if horizontal[l as usize] {
                b[(y - y1) as usize][(x - x1) as usize] == b'1'
            } else {
                b[(x - x1) as usize][(y - y1) as usize] == b'1'
            }
        };
        let wire_cost = |l: i16| wire[l as usize];
        let via_cost = |from: i16, to: i16| {
            if to < from { down[from as usize] } else { up[from as usize] }.expect("a via the reference priced")
        };
        let inp = Search3DInputs {
            num_layers: layers.len() as i16,
            region: (reg[0], reg[1], reg[2], reg[3]),
            horizontal: &horizontal,
            min_layer: int(&r["minl"]) as i32,
            max_layer: int(&r["maxl"]) as i32,
            admits: &admits,
            wire_cost: &wire_cost,
            via_cost: &via_cost,
            original_len: int(&r["olen"]) as i32,
            resistance_aware: int(&r["ra"]) == 1,
            detour_penalty: int(&r["detour"]) as i32,
        };
        let cells = |k: &str| -> Vec<Cell3> { r[k].as_array().expect(k).iter().map(|v| decode(int(v))).collect() };
        let got = maze_search_3d(&inp, &cells("src"), &cells("dst")).unwrap_or_else(|e| panic!("{who}: {e:?}"));

        assert_eq!(got.pops, cells("pops"), "{who}: pop order");
        let cross = int(&r["cross"]);
        assert_eq!(got.crossing, (cross >= 0).then(|| decode(cross)), "{who}: crossing");

        let want: HashMap<Cell3, &Value> = r["cells"].as_array().expect("cells").iter()
            .map(|c| ((int(&c[0]) as i16, int(&c[1]) as i16, int(&c[2]) as i16), c)).collect();
        assert_eq!(got.reached.len(), want.len(), "{who}: reached cell count");
        for (c, s) in &got.reached {
            let w = want.get(c).unwrap_or_else(|| panic!("{who}: {c:?} reached here, not in the reference"));
            assert_eq!(s.dist as i64, int(&w[3]), "{who}: dist at {c:?}");
            assert_eq!(s.path_len as i64, int(&w[4]), "{who}: path_len at {c:?}");
            assert_eq!(s.dir, dir(int(&w[8])), "{who}: direction at {c:?}");
            // A seed's parent is never written by the reference (whatever an earlier search left).
            if s.dir != Dir3::Origin {
                let p = s.parent.expect("a relaxed cell has a parent");
                assert_eq!(
                    (p.0 as i64, p.2 as i64, p.1 as i64),
                    (int(&w[5]), int(&w[6]), int(&w[7])),
                    "{who}: parent (layer, x, y) at {c:?}"
                );
            }
        }
        // R18f: the backtrace, run on THIS engine's search state, against the reference's outcome.
        let states: HashMap<Cell3, CellState> = got.reached.iter().copied().collect();
        let bt = &r["bt"];
        match (backtrace_3d(got.crossing, &|c| states[&c]), bt["kind"].as_str().expect("kind")) {
            (Ok(b), "path") => {
                let want: Vec<Point3D> = bt["grids"].as_array().expect("grids").iter().map(|p| Point3D {
                    x: int(&p[0]) as i16,
                    y: int(&p[1]) as i16,
                    layer: int(&p[2]) as i16,
                }).collect();
                assert_eq!(b.grids, want, "{who}: backtrace path");
                assert_eq!(
                    (b.head_room as i64, i64::from(b.orig_layer), i64::from(b.last_layer)),
                    (int(&bt["headroom"]), int(&bt["origL"]), int(&bt["lastL"])),
                    "{who}: head room / layers"
                );
            }
            (Err(Recovery::ZeroDistance), "zero") | (Err(Recovery::Underflow), "underflow") => {}
            (got_bt, want) => panic!("{who}: backtrace {got_bt:?}, reference {want}"),
        }
        pops_total += got.pops.len();
    }
    (records.len(), pops_total)
}

/// Every captured search: pop order, crossing, and every reached cell's final state.
#[test]
fn search_3d_matches_the_reference() {
    let (n, pops) = replay(&golden());
    assert!(n >= 100 && pops >= 10_000, "too little: {n} searches, {pops} pops");
}

/// GRT_SEARCH3D_FULL=/path/to/m3e-all.json cargo test --test search3d -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn search_3d_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_SEARCH3D_FULL").expect("set GRT_SEARCH3D_FULL");
    eprintln!("exhaustive: {:?} (searches, pops)", replay(&read(&path)));
}

/// ⚠️ No captured search ran dry (0 of 73,179) — but TWO took the zero-distance recovery (both on
/// `overlapping_edges`), which the reference does not count for GRT-183. Neither is in the sample;
/// R18h captures them. A tripwire: a change in either count means the golden needs a look.
#[test]
fn recoveries_are_as_measured() {
    let g = golden();
    assert_eq!(int(&g["underflow"]), 0);
    assert_eq!(int(&g["zero_recover"]), 2);
    assert!(int(&g["searches"]) >= 70_000);
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn inputs<'a>(
    horizontal: &'a [bool],
    admits: &'a dyn Fn(i16, i32, i32) -> bool,
    wire: &'a dyn Fn(i16) -> f32,
    via: &'a dyn Fn(i16, i16) -> f32,
    ra: bool,
) -> Search3DInputs<'a> {
    Search3DInputs {
        num_layers: horizontal.len() as i16,
        region: (0, 3, 0, 3),
        horizontal,
        min_layer: 0,
        max_layer: horizontal.len() as i32 - 1,
        admits,
        wire_cost: wire,
        via_cost: via,
        original_len: 1,
        resistance_aware: ra,
        detour_penalty: 7,
    }
}

/// ⛔ Planar moves follow the layer's PREFERRED direction only: on a horizontal layer the search
/// cannot move in y without a via.
#[test]
fn planar_moves_follow_the_preferred_direction() {
    let hz = [true];
    let open = |_: i16, _: i32, _: i32| true;
    let one = |_: i16| 1.0f32;
    let via = |_: i16, _: i16| 1.0f32;
    let r = maze_search_3d(&inputs(&hz, &open, &one, &via, false), &[(0, 0, 0)], &[(0, 1, 0)]).expect("ok");
    // (0,1,0) is directly above in y: unreachable on a single horizontal layer.
    assert_eq!(r.crossing, None);
    assert!(r.reached.iter().all(|(c, _)| c.1 == 0), "never left row 0");
}

/// ⛔ The candidate is computed in f32 and stored TRUNCATED, per step: at 0.75 a step, every stored
/// distance stays 0 — the fraction is lost at each step, not once at the end.
#[test]
fn distances_are_truncated_per_step() {
    let hz = [true];
    let open = |_: i16, _: i32, _: i32| true;
    let frac = |_: i16| 0.75f32;
    let via = |_: i16, _: i16| 1.0f32;
    let r = maze_search_3d(&inputs(&hz, &open, &frac, &via, false), &[(0, 0, 0)], &[(0, 0, 3)]).expect("ok");
    let d: HashMap<Cell3, i32> = r.reached.iter().map(|(c, s)| (*c, s.dist)).collect();
    assert_eq!((d[&(0, 0, 1)], d[&(0, 0, 2)], d[&(0, 0, 3)]), (0, 0, 0));
}

/// ⛔ Above 2^24 the int-to-f32 conversion ROUNDS. One step at 2^24 then a via at 1 is
/// 16,777,217 — not representable in f32 — and the reference stores 16,777,216. Layer 1's planar
/// edges are closed, so the only route to the destination is along layer 0 and up.
#[test]
fn large_distances_round_in_f32() {
    let hz = [true, true];
    let layer0_only = |l: i16, _: i32, _: i32| l == 0;
    let wire = |l: i16| if l == 0 { 16_777_216.0f32 } else { 1.0 };
    let via = |_: i16, _: i16| 1.0f32;
    let r = maze_search_3d(&inputs(&hz, &layer0_only, &wire, &via, false), &[(0, 0, 0)], &[(1, 0, 1)]).expect("ok");
    let d: HashMap<Cell3, i32> = r.reached.iter().map(|(c, s)| (*c, s.dist)).collect();
    assert_eq!(d[&(0, 0, 1)], 16_777_216);
    assert_eq!(d[&(1, 0, 1)], 16_777_216, "16,777,217 rounded in f32");
}

/// ⛔ Three terms, rounded in f32 AFTER EACH ADDITION. The only route to the destination reaches
/// 2^24 on layer 0, vias up, then takes one step on layer 1 priced 1 with a detour penalty of 1:
/// f32 rounds `2^24 + 1` back to 2^24 before the penalty is added, so the result stays 2^24;
/// summing in f64 and rounding once would give 2^24 + 2. Expected values computed independently
/// with numpy's float32.
#[test]
fn the_candidate_rounds_after_each_addition() {
    let hz = [true, true];
    // Layer 1's edge leaving x = 0 is closed, so the cheap via at the seed leads nowhere.
    let admits = |l: i16, x: i32, _: i32| !(l == 1 && x == 0);
    let wire = |l: i16| if l == 0 { 16_777_216.0f32 } else { 1.0 };
    let via = |_: i16, _: i16| 1.0f32;
    let mut inp = inputs(&hz, &admits, &wire, &via, true);
    inp.detour_penalty = 1;
    inp.original_len = 0; // every planar step is past the original length
    let r = maze_search_3d(&inp, &[(0, 0, 0)], &[(1, 0, 2)]).expect("ok");
    let d: HashMap<Cell3, i32> = r.reached.iter().map(|(c, s)| (*c, s.dist)).collect();
    assert_eq!(d[&(0, 0, 1)], 16_777_216, "(0 + 2^24) + 1 rounds to 2^24");
    assert_eq!(d[&(1, 0, 1)], 16_777_216, "2^24 + via 1 rounds to 2^24");
    assert_eq!(d[&(1, 0, 2)], 16_777_216, "(2^24 + 1) + 1, rounded after EACH addition");
}

/// ⛔ Vias ignore admission and the layer range, carry no detour penalty and do not lengthen the
/// path; planar moves are refused on a closed edge.
#[test]
fn vias_ignore_admission_and_planar_moves_respect_it() {
    let hz = [true, false];
    let closed = |_: i16, _: i32, _: i32| false;
    let one = |_: i16| 1.0f32;
    let via = |_: i16, _: i16| 1.0f32;
    let mut inp = inputs(&hz, &closed, &one, &via, true);
    inp.max_layer = 0; // layer 1 outside the net's range — a via still reaches it
    let r = maze_search_3d(&inp, &[(0, 0, 0)], &[(1, 0, 0)]).expect("ok");
    assert_eq!(r.crossing, Some((1, 0, 0)));
    let s: HashMap<Cell3, _> = r.reached.iter().map(|(c, s)| (*c, *s)).collect();
    assert_eq!((s[&(1, 0, 0)].dist, s[&(1, 0, 0)].path_len, s[&(1, 0, 0)].dir), (1, 0, Dir3::Up));
    assert!(!s.contains_key(&(0, 0, 1)), "the closed planar edge was refused");
}

/// ⛔ The detour penalty applies only in resistance-aware mode and only once the new length
/// exceeds the edge's original length.
#[test]
fn the_detour_penalty_needs_resistance_aware_and_a_longer_path() {
    let hz = [true];
    let open = |_: i16, _: i32, _: i32| true;
    let one = |_: i16| 1.0f32;
    let via = |_: i16, _: i16| 1.0f32;
    for (ra, want) in [(false, (1, 2)), (true, (1, 9))] {
        let r = maze_search_3d(&inputs(&hz, &open, &one, &via, ra), &[(0, 0, 0)], &[(0, 0, 3)]).expect("ok");
        let d: HashMap<Cell3, i32> = r.reached.iter().map(|(c, s)| (*c, s.dist)).collect();
        // Step 1: length 1, not past the original 1 → no penalty. Step 2: length 2 → +7 when ra.
        assert_eq!((d[&(0, 0, 1)], d[&(0, 0, 2)]), want, "ra={ra}");
    }
}

// ─── R18f constructed cases ─────────────────────────────────────────────────────────────────

fn st(dist: i32, parent: Option<Cell3>, dir: Dir3) -> CellState {
    CellState { dist, path_len: 0, parent, dir }
}

/// The walk follows parents until a ZERO distance, reverses, and appends the crossing; `head_room`
/// is the index of the last point at the start position (a via stack there raises it).
#[test]
fn the_backtrace_walks_to_zero_and_counts_the_start_stack() {
    // seed (0,0,0) -> via up (1,0,0) -> via up (2,0,0) -> planar (2,0,1) = crossing.
    let cells: HashMap<Cell3, CellState> = [
        ((0, 0, 0), st(0, None, Dir3::Origin)),
        ((1, 0, 0), st(1, Some((0, 0, 0)), Dir3::Up)),
        ((2, 0, 0), st(2, Some((1, 0, 0)), Dir3::Up)),
        ((2, 0, 1), st(3, Some((2, 0, 0)), Dir3::East)),
    ].into_iter().collect();
    let b = backtrace_3d(Some((2, 0, 1)), &|c| cells[&c]).expect("a path");
    let p = |x, y, layer| Point3D { x, y, layer };
    assert_eq!(b.grids, vec![p(0, 0, 0), p(0, 0, 1), p(0, 0, 2), p(1, 0, 2)]);
    assert_eq!((b.head_room, b.orig_layer, b.last_layer), (2, 0, 2));
}

/// ⛔ A crossing at distance 0 is a recovery, not a one-point path; no crossing is the other one.
#[test]
fn a_zero_distance_crossing_and_underflow_both_recover() {
    let cells: HashMap<Cell3, CellState> = [((0, 0, 0), st(0, None, Dir3::Origin))].into_iter().collect();
    assert_eq!(backtrace_3d(Some((0, 0, 0)), &|c| cells[&c]), Err(Recovery::ZeroDistance));
    assert_eq!(backtrace_3d(None, &|c| cells[&c]), Err(Recovery::Underflow));
}

/// ⛔ The walk stops at DISTANCE 0, not at a seed: a relaxed cell holding 0 (a price below 1,
/// truncated) ends the path early. Never captured — every price is at least 1.
#[test]
fn the_walk_stops_at_the_first_zero_distance() {
    let cells: HashMap<Cell3, CellState> = [
        ((0, 0, 0), st(0, None, Dir3::Origin)),
        ((0, 0, 1), st(0, Some((0, 0, 0)), Dir3::East)), // relaxed, but truncated to 0
        ((0, 0, 2), st(1, Some((0, 0, 1)), Dir3::East)),
    ].into_iter().collect();
    let b = backtrace_3d(Some((0, 0, 2)), &|c| cells[&c]).expect("a path");
    assert_eq!(b.grids.len(), 2, "stopped at (0,0,1), not at the seed");
}
