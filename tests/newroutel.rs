// SPDX-License-Identifier: Apache-2.0
//! Stage R8 — the via-guided L decision, checked against the reference.
//!
//! The congestion accumulation is already covered by the first L-routing stage. What is checked
//! here is what that stage does not have: the **via bias**, whose status-to-cost mapping is
//! crossed between the two endpoints, and the **connection marks** the chosen shape leaves, which
//! are crossed the same way.

#![allow(non_snake_case)]

use vyges_grt::*;

const GOLDEN: &str = include_str!("../examples/grt_gate/newroutel.json");

struct Row {
    n1: i16,
    n2: i16,
    bias_l1: f64,
    bias_l2: f64,
    via_cost: f64,
    l1: f64,
    l2: f64,
    x_first: bool,
    n1_after: i16,
    n2_after: i16,
}

fn rows() -> Vec<Row> {
    let v: serde_json::Value = serde_json::from_str(GOLDEN).expect("golden parses");
    let bits = |x: &serde_json::Value| f64::from_bits(x.as_str().unwrap().parse().unwrap());
    v.as_array().unwrap().iter().map(|r| Row {
        n1: r["n1"].as_i64().unwrap() as i16,
        n2: r["n2"].as_i64().unwrap() as i16,
        bias_l1: bits(&r["bias_l1_bits"]),
        bias_l2: bits(&r["bias_l2_bits"]),
        via_cost: bits(&r["via_cost_bits"]),
        l1: bits(&r["l1_bits"]),
        l2: bits(&r["l2_bits"]),
        x_first: r["x_first"].as_bool().unwrap(),
        n1_after: r["n1_after"].as_i64().unwrap() as i16,
        n2_after: r["n2_after"].as_i64().unwrap() as i16,
    }).collect()
}

#[test]
fn the_via_BIAS_matches_the_reference_on_every_decision() {
    // ⛔ The crossed mapping. Getting it the same way round for both endpoints is the obvious
    // mistake and would pass any test that only checked one of them.
    let rows = rows();
    for (i, r) in rows.iter().enumerate() {
        assert_eq!(via_bias(r.n1, r.n2, r.via_cost), (r.bias_l1, r.bias_l2),
            "decision {i}: bias for statuses ({}, {})", r.n1, r.n2);
    }
    // ⚠️ 568, not 284: this stage runs TWICE per edge — once via-guided and once from the
    // congestion-driven reroute. A first extractor keyed on (net, edge) and silently kept only
    // the last, halving the corpus without saying so.
    assert_eq!(rows.len(), 568);
}

#[test]
fn the_CHOICE_matches_the_reference_on_every_decision() {
    for (i, r) in rows().iter().enumerate() {
        let want = if r.x_first { LShape::XFirst } else { LShape::YFirst };
        assert_eq!(choose_l_shape(r.l1, r.l2), want, "decision {i}");
    }
}

#[test]
fn route_edge_leaves_the_CROSSED_marks_the_reference_does() {
    // ⛔ Driven through `route_edge`, not by applying the marks here. A first version of this
    // test did the latter — it asserted my understanding of the crossing and would have passed
    // an implementation that marked both nodes the same way, which a mutation proved.
    //
    // A diagonal edge on an empty grid: every congestion term is zero, so the bias alone decides
    // and the outcome is predictable from the statuses.
    let no_red = |_: usize, _: usize| 0u16;

    // n1 H-connected (2) biases L1, so L2 wins -> x-first -> marks n2 V and n1 H.
    let mut grid = EstimateGrid::new(8, 8);
    let mut nodes = vec![
        TreeNode { x: 1, y: 1, status: 2 },
        TreeNode { x: 4, y: 5, status: 0 },
    ];
    let edge = TreeEdge { n1: 0, n2: 1, len: 7 };
    let r = route_edge(&mut grid, &mut nodes, &edge, 1, 10.0, true, 100.0, 100.0, &no_red, &no_red);
    assert_eq!(r, EdgeRoute::L(LShape::XFirst));
    assert_eq!(nodes[0].status, 2, "n1 was already H-connected; marking H again changes nothing");
    assert_eq!(nodes[1].status, 1, "n2 is marked VERTICAL — the crossed half");

    // n2 V-connected (1) biases L1 too, so again L2 wins; now n1 starts clean and gains H.
    let mut grid = EstimateGrid::new(8, 8);
    let mut nodes = vec![
        TreeNode { x: 1, y: 1, status: 0 },
        TreeNode { x: 4, y: 5, status: 1 },
    ];
    let r = route_edge(&mut grid, &mut nodes, &edge, 1, 10.0, true, 100.0, 100.0, &no_red, &no_red);
    assert_eq!(r, EdgeRoute::L(LShape::XFirst));
    assert_eq!(nodes[0].status, 2, "n1 is marked HORIZONTAL — the other crossed half");
    assert_eq!(nodes[1].status, 1);
}

#[test]
fn the_marks_the_reference_recorded_follow_from_the_shape_it_chose() {
    // The corpus half: given the statuses before and the shape chosen, the statuses after must
    // follow. This checks the golden is self-consistent with the crossing rule.
    for (i, r) in rows().iter().enumerate() {
        let mut n1 = TreeNode { x: 0, y: 0, status: r.n1 };
        let mut n2 = TreeNode { x: 0, y: 0, status: r.n2 };
        if r.x_first {
            mark_v(&mut n2.status);
            mark_h(&mut n1.status);
        } else {
            mark_v(&mut n1.status);
            mark_h(&mut n2.status);
        }
        assert_eq!((n1.status, n2.status), (r.n1_after, r.n2_after), "decision {i}");
    }
}

#[test]
fn the_statuses_observed_stay_within_the_TWO_BIT_range() {
    // ⚠️ An empirical fact, asserted so it announces itself if it stops being true — the
    // reference does not enforce it, it only warns.
    let rows = rows();
    assert!(rows.iter().all(|r| (0..=3).contains(&r.n1) && (0..=3).contains(&r.n2)));
    assert!(rows.iter().all(|r| is_known_status(r.n1_after) && is_known_status(r.n2_after)));
}

#[test]
fn ONE_arm_of_the_bias_is_unwitnessed_and_this_records_which() {
    // ⬜ The bias has four arms and this design drives three. Measured over all 568 decisions:
    // `n1` takes only 0, 2 and 3, so the arm that penalises L2 because `n1` is already connected
    // VERTICALLY never fires. `n2` does take status 1, twice, so its mirror arm is covered.
    //
    // ⚠️ Asserted both ways, so the gap announces itself when a design finally exercises it.
    let rows = rows();
    assert_eq!(rows.iter().filter(|r| r.n1 == 1).count(), 0,
        "n1 now takes status 1 — that bias arm is witnessed at last, update this test");
    assert_eq!(rows.iter().filter(|r| r.n2 == 1).count(), 2,
        "the n2 status-1 arm must stay exercised");
    // and the synthetic case below is what covers the missing arm
}

#[test]
fn the_bias_mapping_is_CROSSED_between_the_two_endpoints() {
    // The synthetic case that covers all four arms, including the one the corpus does not reach.
    assert_eq!(via_bias(2, 0, 10.0), (10.0, 0.0), "n1 H-connected penalises L1");
    assert_eq!(via_bias(1, 0, 10.0), (0.0, 10.0), "n1 V-connected penalises L2");
    assert_eq!(via_bias(0, 2, 10.0), (0.0, 10.0), "n2 H-connected penalises L2 — the other way");
    assert_eq!(via_bias(0, 1, 10.0), (10.0, 0.0), "n2 V-connected penalises L1 — the other way");
    assert_eq!(via_bias(2, 1, 10.0), (20.0, 0.0), "and they accumulate");
}

#[test]
fn statuses_ZERO_and_THREE_earn_no_bias_at_all() {
    // Nothing connected yet, or both directions already are — either way the bias cannot
    // discriminate, so it contributes nothing.
    assert_eq!(via_bias(0, 0, 10.0), (0.0, 0.0));
    assert_eq!(via_bias(3, 3, 10.0), (0.0, 0.0));
}
