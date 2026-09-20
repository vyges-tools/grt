// SPDX-License-Identifier: Apache-2.0
//! The first real routing decision — which way each diagonal segment bends.
//!
//! ⛔ **Sequential, so it is replayed and not spot-checked.** Committing one bend changes the
//! demand the next segment sees, so every decision depends on all its predecessors. A test that
//! checked decisions independently would pass an implementation that updated the grid wrongly.
//!
//! 🔑 **Two designs, because one of them cannot decide anything.** On an uncongested design every
//! edge is below its allowance, both candidate paths cost exactly zero, and all 284 decisions are
//! TIES — so that corpus validates the tie-break and nothing else. The congested one has
//! non-zero costs on all 284 and splits 126/158, so there the comparison genuinely decides.
//!
//! ⛔ **The costs are carried as raw IEEE bits, not decimal.** A first capture printed them with
//! nine fixed decimals and truncated one; the fix to shortest-round-trip still left a genuine
//! 1-ULP gap. Bits end the question, and a bit-exact match is the only claim worth making about
//! an accumulation this long.

#![allow(non_snake_case)]

use vyges_grt::*;

const CONGESTED: &str = include_str!("../examples/grt_gate/lroute_congested.json");
const CLEAR: &str = include_str!("../examples/grt_gate/lroute_clear.json");

struct Case {
    x_grids: usize,
    y_grids: usize,
    v_lb: f32,
    h_lb: f32,
    segments: Vec<Segment>,
    /// blockage per edge, keyed (x, y) -> (h_red, v_red)
    red: std::collections::BTreeMap<(usize, usize), (u16, u16)>,
    decisions: Vec<Decision>,
}

struct Decision {
    seg: Segment,
    cost_y_first: f64,
    cost_x_first: f64,
    x_first: bool,
}

fn parse(text: &str) -> Case {
    let v: serde_json::Value = serde_json::from_str(text).expect("golden parses");
    let g = v["grid"].as_array().unwrap();
    let lb = v["lb_bits"].as_array().unwrap();
    Case {
        x_grids: g[0].as_u64().unwrap() as usize,
        y_grids: g[1].as_u64().unwrap() as usize,
        // ⚠️ Captured as the exact f32 the reference held, via its raw bits — a decimal
        // rendering cost a whole cycle chasing a 1-ULP "disagreement" that was the capture.
        v_lb: lb[0].as_f64().unwrap() as f32,
        h_lb: lb[1].as_f64().unwrap() as f32,
        segments: v["segments"].as_array().unwrap().iter().map(|s| {
            let f: Vec<i64> = s.as_array().unwrap().iter().map(|x| x.as_i64().unwrap()).collect();
            Segment { x1: f[1] as i32, y1: f[2] as i32, x2: f[3] as i32, y2: f[4] as i32,
                      edge_cost: f[5] as i8 }
        }).collect(),
        red: v["red"].as_array().unwrap().iter().map(|r| {
            let f: Vec<i64> = r.as_array().unwrap().iter().map(|x| x.as_i64().unwrap()).collect();
            ((f[0] as usize, f[1] as usize), (f[2] as u16, f[3] as u16))
        }).collect(),
        decisions: v["decisions"].as_array().unwrap().iter().map(|d| Decision {
            seg: Segment {
                x1: d["x1"].as_i64().unwrap() as i32, y1: d["y1"].as_i64().unwrap() as i32,
                x2: d["x2"].as_i64().unwrap() as i32, y2: d["y2"].as_i64().unwrap() as i32,
                edge_cost: d["cost"].as_i64().unwrap() as i8,
            },
            // ⛔ Raw IEEE bits, not a decimal string. See the module note.
            cost_y_first: f64::from_bits(d["cost_y_first_bits"].as_str().unwrap().parse().unwrap()),
            cost_x_first: f64::from_bits(d["cost_x_first_bits"].as_str().unwrap().parse().unwrap()),
            x_first: d["x_first"].as_bool().unwrap(),
        }).collect(),
    }
}

/// Replay a whole design: estimate every segment, then decide every diagonal one in order.
fn replay(c: &Case) -> (usize, usize) {
    let mut grid = EstimateGrid::new(c.x_grids, c.y_grids);
    for s in &c.segments {
        estimate_one_seg(&mut grid, s);
    }

    let red_h = |x: usize, y: usize| c.red.get(&(x, y)).map(|r| r.0).unwrap_or(0);
    let red_v = |x: usize, y: usize| c.red.get(&(x, y)).map(|r| r.1).unwrap_or(0);

    let (mut checked, mut y_first) = (0, 0);
    for d in &c.decisions {
        let (ymin, ymax) = (d.seg.y1.min(d.seg.y2), d.seg.y1.max(d.seg.y2));
        let mut cost_y_first = 0.0;
        let mut cost_x_first = 0.0;
        for i in ymin..ymax {
            cost_y_first += congestion_cost(
                grid.v(d.seg.x1 as usize, i as usize), red_v(d.seg.x1 as usize, i as usize), c.v_lb);
            cost_x_first += congestion_cost(
                grid.v(d.seg.x2 as usize, i as usize), red_v(d.seg.x2 as usize, i as usize), c.v_lb);
        }
        for i in d.seg.x1..d.seg.x2 {
            cost_y_first += congestion_cost(
                grid.h(i as usize, d.seg.y2 as usize), red_h(i as usize, d.seg.y2 as usize), c.h_lb);
            cost_x_first += congestion_cost(
                grid.h(i as usize, d.seg.y1 as usize), red_h(i as usize, d.seg.y1 as usize), c.h_lb);
        }

        assert_eq!(cost_y_first, d.cost_y_first,
            "net {}: y-first cost at segment {checked}", d.seg.x1);
        assert_eq!(cost_x_first, d.cost_x_first,
            "net {}: x-first cost at segment {checked}", d.seg.x1);

        let shape = choose_l_shape(cost_y_first, cost_x_first);
        let want = if d.x_first { LShape::XFirst } else { LShape::YFirst };
        assert_eq!(shape, want, "decision {checked} bent the wrong way");

        commit_l_shape(&mut grid, &d.seg, shape);
        checked += 1;
        if shape == LShape::YFirst { y_first += 1; }
    }
    (checked, y_first)
}

#[test]
fn every_decision_of_a_CONGESTED_design_matches_the_reference() {
    // ⭐ The corpus that can actually fail: all 284 decisions carry non-zero cost and the split
    // is 126/158, so the comparison decides rather than the tie-break.
    let c = parse(CONGESTED);
    let (checked, y_first) = replay(&c);
    assert_eq!(checked, 284);
    assert_eq!(y_first, 126, "the split must stay uneven, or the comparison is not deciding");
    assert!(!c.red.is_empty(), "and the blockage term must be exercised");
}

#[test]
fn every_decision_of_an_UNCONGESTED_design_is_a_TIE_and_goes_the_same_way() {
    // ⬜ This corpus validates the tie-break ALONE. Every edge is below its allowance, so both
    // candidates cost exactly zero and the strict `<` sends all 284 to x-first. Kept because the
    // tie-break is a rule, and recorded as narrow so it is not mistaken for broad coverage.
    let c = parse(CLEAR);
    let (checked, y_first) = replay(&c);
    assert_eq!(checked, 284);
    assert_eq!(y_first, 0, "every decision is a tie, and a tie goes x-first");
    assert!(c.red.is_empty(), "and this design has no blockage at all");
    assert!(c.decisions.iter().all(|d| d.cost_y_first == 0.0 && d.cost_x_first == 0.0));
}

#[test]
fn a_TIE_goes_X_FIRST_because_the_comparison_is_STRICT() {
    // ⛔ `costL1 < costL2`, so equal costs fall to the else. `<=` would flip every tied segment,
    // and on an uncongested design that is ALL of them.
    assert_eq!(choose_l_shape(0.0, 0.0), LShape::XFirst);
    assert_eq!(choose_l_shape(1.0, 1.0), LShape::XFirst);
    assert_eq!(choose_l_shape(0.5, 1.0), LShape::YFirst);
    assert_eq!(choose_l_shape(1.0, 0.5), LShape::XFirst);
}

#[test]
fn cost_is_OVERFLOW_ONLY_and_includes_the_BLOCKAGE() {
    // Demand below the allowance is free; demand above it costs the excess. Blockage counts as
    // demand.
    assert_eq!(congestion_cost(5.0, 0, 10.0), 0.0, "below the allowance is free");
    assert_eq!(congestion_cost(12.0, 0, 10.0), 2.0, "only the excess counts");
    assert_eq!(congestion_cost(5.0, 8, 10.0), 3.0, "blockage counts as demand");
}

#[test]
fn the_capacity_bound_is_computed_in_THIRTY_TWO_BIT_float() {
    // ⛔ The reference computes `float LB = 0.9; lb = LB * capacity` in single precision. The
    // captured value proves it: 0.9 * 39 is 35.1 exactly in decimal, but the reference reports
    // 35.099998474 — an f32 artefact. Computing the bound in f64 gives a different threshold and
    // therefore a different answer for any demand that lands on it.
    assert_eq!(capacity_lower_bound(39), 35.099998474121094_f32);
    assert_ne!(capacity_lower_bound(39) as f64, 0.9_f64 * 39.0,
        "the two bounds are different numbers");

    // ⬜ But the difference is UNWITNESSED as a behaviour change: recomputing the bound in f64
    // passes every reference check on both designs here, because no demand lands in the gap
    // between the two thresholds. The width is kept because the reference uses it, not because a
    // mutation caught it — a distinction worth preserving rather than blurring.
}

#[test]
fn the_bound_WIDTH_is_unwitnessed_on_these_designs_and_this_records_it() {
    // The gap between the f32 and f64 bounds, on the capacities these designs use.
    let c = parse(CONGESTED);
    let gap = (c.h_lb as f64 - 0.9_f64 * 34.0).abs();
    assert!(gap > 0.0, "the thresholds differ");
    assert!(gap < 1e-5, "but only in the seventh decimal, which no demand here falls into");
}
