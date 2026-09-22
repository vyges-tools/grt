// SPDX-License-Identifier: Apache-2.0
//! Id — I10 `applyAdjustments`, replayed step by step against the reference's router state.
//!
//! Golden `adjustments.json`: whole calls chosen for their features. From the state at entry, the
//! replay applies the captured obstruction stream (+ transition tiles) and compares the state after
//! the obstructions, then after each later step — blocked intervals, the resource save, the global
//! adjustment (the TECH values it leaves), the per-layer adjustments, and each region. The state is
//! every 3D and 2D edge's cap / red / real cap and every blocked-interval set.
//!
//! The macro designs (the only macro obstructions and transition tiles) are too large to commit;
//! the exhaustive replay runs them from the uncapped dump.

use std::collections::{BTreeSet, HashMap};

use serde_json::Value;
use vyges_grt::{
    adjust_tile_set, apply_obstruction_adjustment, compute_region_adjustments, compute_user_global_adjustments,
    compute_user_layer_adjustments, init_blocked_intervals, save_resources_before_adjustments, Direction, EdgeState,
    IntervalSet, Rect, RouterEdges,
};

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

fn direction(d: &str) -> Option<Direction> {
    match d {
        "H" => Some(Direction::Horizontal),
        "V" => Some(Direction::Vertical),
        _ => None,
    }
}

fn unrle(v: &Value) -> Vec<EdgeState> {
    arr(v)
        .iter()
        .flat_map(|p| {
            let e = EdgeState { cap: int(&p[0]) as u16, red: int(&p[1]) as u16, real_cap: int(&p[2]) as u16 };
            std::iter::repeat(e).take(int(&p[3]) as usize)
        })
        .collect()
}

/// The state a STEP dump describes, in the engine's layout.
struct State {
    h3: Vec<EdgeState>,
    v3: Vec<EdgeState>,
    h2: Vec<EdgeState>,
    v2: Vec<EdgeState>,
    hb: HashMap<(i32, i32, i32), IntervalSet>,
    vb: HashMap<(i32, i32, i32), IntervalSet>,
}

fn state(st: &Value, xg: i32, yg: i32, nl: i32) -> State {
    let (mut h3, mut v3, mut h2, mut v2) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for l in 0..nl {
        for y in 0..yg {
            h3.extend(unrle(&st["e3"][format!("H,{l},{y}")]));
        }
    }
    for l in 0..nl {
        for y in 0..yg {
            v3.extend(unrle(&st["e3"][format!("V,{l},{y}")]));
        }
    }
    for y in 0..yg {
        h2.extend(unrle(&st["e2"][format!("H,{y}")]));
    }
    for y in 0..yg - 1 {
        v2.extend(unrle(&st["e2"][format!("V,{y}")]));
    }
    let (mut hb, mut vb) = (HashMap::new(), HashMap::new());
    for b in arr(&st["bi"]) {
        let set = IntervalSet(arr(&b[4]).iter().map(|p| (int(&p[0]), int(&p[1]))).collect());
        let key = (int(&b[1]), int(&b[2]), int(&b[3]));
        if b[0].as_str() == Some("H") { hb.insert(key, set) } else { vb.insert(key, set) };
    }
    State { h3, v3, h2, v2, hb, vb }
}

fn first_diff(a: &[EdgeState], b: &[EdgeState]) -> Option<(usize, EdgeState, EdgeState)> {
    a.iter().zip(b).enumerate().find(|(_, (x, y))| x != y).map(|(i, (x, y))| (i, *x, *y))
}

fn check(e: &RouterEdges, st: &Value, who: &str, step: &str) {
    let want = state(st, e.x_grid, e.y_grid, e.num_layers);
    for (name, got, w) in [("H3", &e.h3, &want.h3), ("V3", &e.v3, &want.v3), ("H2", &e.h2, &want.h2), ("V2", &e.v2, &want.v2)] {
        assert_eq!(got.len(), w.len(), "{who} {step}: {name} size");
        if let Some((i, g, x)) = first_diff(got, w) {
            panic!("{who} after {step}: {name}[{i}] engine {g:?} reference {x:?}");
        }
    }
    assert_eq!(e.horizontal_blocked, want.hb, "{who} after {step}: horizontal blocked intervals");
    assert_eq!(e.vertical_blocked, want.vb, "{who} after {step}: vertical blocked intervals");
}

#[derive(Default)]
struct Seen {
    calls: usize,
    steps: usize,
    stream: usize,
    regions: usize,
    outside_die: usize,
    macro_obs: usize,
    tiles: usize,
}

fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        let mut log = Vec::new();
        for c in arr(&r["calls"]) {
            if int(&c["small"]) == 0 {
                continue;
            }
            let steps = &c["steps"];
            let (xg, yg, nl) = (int(&c["xg"]), int(&c["yg"]), int(&c["nl"]));
            let d = arr(&c["die"]);
            let entry = state(&steps["entry"], xg, yg, nl);
            let mut e = RouterEdges {
                die: Rect::new(int(&d[0]), int(&d[1]), int(&d[2]), int(&d[3])),
                tile_size: int(&c["tile"]),
                x_grid: xg,
                y_grid: yg,
                num_layers: nl,
                track_pitches: arr(&c["pitches"]).iter().map(int).collect(),
                h3: entry.h3,
                v3: entry.v3,
                h2: entry.h2,
                v2: entry.v2,
                horizontal_blocked: entry.hb,
                vertical_blocked: entry.vb,
                verbose: int(&c["verbose"]) == 1,
                log: Vec::new(),
            };
            // Per routing level (index 0 unused).
            let layers = arr(&c["layers"]);
            let mut dirs = vec![None];
            dirs.extend(layers.iter().map(|l| direction(l["dir"].as_str().expect("dir"))));
            let mut tech_adj = vec![0.0f32];
            tech_adj.extend(layers.iter().map(|l| l["adj"].as_f64().expect("f") as f32));

            // computeObstructionsAdjustments: the stream, then the transition tiles.
            for s in arr(&c["stream"]) {
                let rect = Rect::new(int(&s[1]), int(&s[2]), int(&s[3]), int(&s[4]));
                let layer = int(&s[5]);
                let die = e.die;
                if !(rect.x_max > die.x_min && rect.x_min < die.x_max && rect.y_max > die.y_min && rect.y_min < die.y_max) {
                    seen.outside_die += 1;
                }
                seen.macro_obs += (int(&s[6]) == 1) as usize;
                apply_obstruction_adjustment(&mut e, rect, layer, dirs[layer as usize], int(&s[6]) == 1);
                seen.stream += 1;
            }
            for t in arr(&c["tiles"]) {
                let layer = int(&t[0]);
                let tiles: BTreeSet<(i32, i32)> = arr(&t[1]).iter().map(|p| (int(&p[0]), int(&p[1]))).collect();
                seen.tiles += tiles.len();
                adjust_tile_set(&mut e, &tiles, layer, dirs[layer as usize]);
            }
            let Some(st) = steps.get("obstructions") else { continue };
            check(&e, st, who, "obstructions");
            seen.steps += 1;

            init_blocked_intervals(&mut e);
            let Some(st) = steps.get("blocked") else { continue };
            check(&e, st, who, "blocked");
            save_resources_before_adjustments(&mut e);
            let Some(st) = steps.get("saved") else { continue };
            check(&e, st, who, "saved");

            compute_user_global_adjustments(&mut tech_adj, c["adjustment"].as_f64().expect("f") as f32, int(&c["min"]), int(&c["max"]));
            let after: Vec<f32> = arr(&c["global_after"]).iter().map(|v| v.as_f64().expect("f") as f32).collect();
            assert_eq!(tech_adj[1..], after[..], "{who}: tech layer adjustments after the global step");
            let Some(st) = steps.get("global") else { continue };
            check(&e, st, who, "global");

            compute_user_layer_adjustments(&mut e, &tech_adj, &dirs, int(&c["min"]), int(&c["max"]));
            let Some(st) = steps.get("layer") else { continue };
            check(&e, st, who, "layer");
            seen.steps += 4;

            for (i, reg) in arr(&c["regions"]).iter().enumerate() {
                let rc = arr(&reg["rect"]);
                let layer = int(&reg["layer"]);
                let res = compute_region_adjustments(
                    &mut e,
                    Rect::new(int(&rc[0]), int(&rc[1]), int(&rc[2]), int(&rc[3])),
                    layer,
                    reg["adjustment"].as_f64().expect("f") as f32,
                    dirs[layer as usize],
                    int(&layers[(layer - 1) as usize]["use_pitch"]),
                );
                match steps.get(format!("region{i}")) {
                    Some(st) => {
                        assert!(res.is_ok(), "{who}: region {i} rejected by the engine only");
                        check(&e, st, who, &format!("region {i}"));
                        seen.regions += 1;
                    }
                    None => assert!(res.is_err(), "{who}: region {i} rejected by the reference only"),
                }
            }
            seen.calls += 1;
            log.extend(e.log.drain(..));
        }
        if !r["quiet"].as_bool().expect("flag") {
            let want: Vec<&str> = arr(&r["log"]).iter().map(|l| l.as_str().expect("line")).collect();
            assert_eq!(log, want, "{who}: GRT-0113/0114 lines");
        }
    }
    seen
}

#[test]
fn the_adjustments_match_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/adjustments.json", env!("CARGO_MANIFEST_DIR"))));
    assert!(s.calls >= 6 && s.stream >= 1000 && s.regions > 0, "calls {}, stream {}, regions {}", s.calls, s.stream, s.regions);
    // Tripwire: the corpus's only out-of-die obstructions (`inst_pin_out_of_die`) sit in a run that
    // ABORTS (GRT-28, verbose) before any later state — the rule is pinned by the constructed case
    // below, not witnessed. If a run ever carries them through, this count moves.
    assert_eq!(s.outside_die, 3, "out-of-die obstructions streamed");
}

/// GRT_ADJUSTMENTS_FULL=/path/to/id-all.json cargo test --release --test adjustments -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_adjustments_match_the_reference_exhaustively() {
    let path = std::env::var("GRT_ADJUSTMENTS_FULL").expect("set GRT_ADJUSTMENTS_FULL");
    let s = replay(&read(&path));
    assert!(s.macro_obs > 0 && s.tiles > 0, "macro obstructions {}, transition tiles {}", s.macro_obs, s.tiles);
    eprintln!("exhaustive: {} calls, {} steps, {} obstructions, {} regions", s.calls, s.steps, s.stream, s.regions);
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn small_grid() -> RouterEdges {
    let (xg, yg, nl) = (4, 4, 1);
    let cap = EdgeState { cap: 10, red: 0, real_cap: 0 };
    RouterEdges {
        die: Rect::new(0, 0, 400, 400),
        tile_size: 100,
        x_grid: xg,
        y_grid: yg,
        num_layers: nl,
        track_pitches: vec![10],
        h3: vec![cap; (xg * yg * nl) as usize],
        v3: vec![cap; (xg * yg * nl) as usize],
        h2: vec![cap; ((xg - 1) * yg) as usize],
        v2: vec![cap; (xg * (yg - 1)) as usize],
        horizontal_blocked: HashMap::new(),
        vertical_blocked: HashMap::new(),
        verbose: false,
        log: Vec::new(),
    }
}

/// ⛔ An obstruction that does not strictly overlap the die is NOT dropped: its rectangle stays
/// odb's default `Rect` — (0, 0, 0, 0) — and blocks at the ORIGIN. Here one far outside the die
/// (x 900) blocks the edge (0,0)→(1,0), exactly as a zero rectangle at the origin does.
#[test]
fn an_obstruction_outside_the_die_blocks_the_origin() {
    let mut outside = small_grid();
    apply_obstruction_adjustment(&mut outside, Rect::new(900, 900, 950, 950), 1, Some(Direction::Horizontal), false);
    let mut origin = small_grid();
    apply_obstruction_adjustment(&mut origin, Rect::new(0, 0, 0, 0), 1, Some(Direction::Horizontal), false);
    assert_eq!(outside.horizontal_blocked, origin.horizontal_blocked);
    assert!(outside.horizontal_blocked.contains_key(&(0, 0, 1)), "blocked at the origin");
}

use vyges_grt::compute_tile_reduce_interval;

/// ⛔ The interval sets are boost ICL's joining sets of RIGHT-OPEN intervals: an empty or inverted
/// interval adds nothing, and touching intervals merge. Only the macro designs (too large to commit)
/// carry an empty one.
#[test]
fn interval_sets_ignore_empty_intervals_and_join_touching_ones() {
    let mut s = IntervalSet::default();
    s.add(10, 10);
    s.add(30, 20);
    assert!(s.0.is_empty());
    s.add(0, 5);
    s.add(5, 8);
    s.add(20, 25);
    assert_eq!(s.0, vec![(0, 8), (20, 25)]);
    assert_eq!(s.blocked_track_count(4), 4, "13 covered / 4 per track, rounded up");
}

/// ⛔ A MACRO obstruction that would leave exactly ONE track widens its interval by a pitch
/// (`layer_cap - ceil((float) len / pitch) == 1`). Only the macro designs reach it.
#[test]
fn a_macro_leaving_one_track_widens_its_interval() {
    let obs = Rect::new(0, 110, 100, 150);
    let tile = Rect::new(0, 100, 100, 200);
    assert_eq!(compute_tile_reduce_interval(obs, tile, 10, true, Some(Direction::Horizontal), 5, true), (110, 160));
    assert_eq!(compute_tile_reduce_interval(obs, tile, 10, true, Some(Direction::Horizontal), 5, false), (110, 150));
    assert_eq!(compute_tile_reduce_interval(obs, tile, 10, true, Some(Direction::Horizontal), 6, true), (110, 150));
}

/// ⛔ A layer adjustment keeps at least ONE track where there was capacity (`floor(1 * 0.5) = 0` →
/// 1), unless the adjustment is exactly 1. The committed calls never floor an edge to 0.
#[test]
fn a_layer_adjustment_keeps_one_track() {
    let mut e = small_grid();
    for c in e.h3.iter_mut() {
        c.cap = 1;
    }
    compute_user_layer_adjustments(&mut e, &[0.0, 0.5], &[None, Some(Direction::Horizontal)], 1, 1);
    assert_eq!(e.h3[0].cap, 1);
    let mut e = small_grid();
    compute_user_layer_adjustments(&mut e, &[0.0, 1.0], &[None, Some(Direction::Horizontal)], 1, 1);
    assert_eq!(e.h3[0].cap, 0, "an adjustment of exactly 1 empties the edge");
}

/// ⛔ A transition tile's edge is halved but keeps at least one track. Every corpus transition
/// edge is wide enough that halving never reaches 0.
#[test]
fn a_transition_tile_keeps_one_track() {
    let mut e = small_grid();
    e.h3[0].cap = 1;
    e.h3[1].cap = 5;
    adjust_tile_set(&mut e, &BTreeSet::from([(0, 0), (1, 0)]), 1, Some(Direction::Horizontal));
    assert_eq!((e.h3[0].cap, e.h3[1].cap), (1, 2), "floor(0.5) → 1; floor(2.5) = 2");
}

/// ⛔ An INCREASE takes the 2D edge's reduction down by the increase, CLAMPED at 0 (`addRedH`:
/// `max(red + delta, 0)`). No corpus increase exceeds the 2D reduction.
#[test]
fn an_increase_clamps_the_2d_reduction_at_zero() {
    let mut e = small_grid();
    e.add_adjustment(0, 0, 1, 0, 1, 15, false);
    assert_eq!((e.h3[0].cap, e.h2[0].cap, e.h2[0].red), (15, 15, 0));
}

/// ⛔ GRT-72 only when the region sticks out on BOTH axes at one corner: out on x alone is accepted.
/// Every corpus region lies inside the die.
#[test]
fn a_region_outside_on_one_axis_is_accepted() {
    let mut e = small_grid();
    let r = compute_region_adjustments(&mut e, Rect::new(250, 100, 500, 200), 1, 0.5, Some(Direction::Horizontal), 10);
    assert!(r.is_ok());
    let r = compute_region_adjustments(&mut e, Rect::new(250, 250, 500, 500), 1, 0.5, Some(Direction::Horizontal), 10);
    assert!(r.is_err());
}
