// SPDX-License-Identifier: Apache-2.0
//! Stage R14 — the rip-up order, checked against the reference on a real design.
//!
//! ⛔ **The reason this has its own gate: the congestion comparison barely discriminates.** On
//! this design **309 of 320 adjacent pairs tie**, and one tie group of **200 nets — 62% of the
//! design** — is ordered entirely by the sorts being *stable*. `sort_unstable_by` would scramble
//! them, and with them every rip-up decision that follows. There is no arithmetic error to find
//! here; there is a choice of sort function.

#![allow(non_snake_case)]

use vyges_grt::*;

const GOLDEN: &str = include_str!("../examples/grt_gate/ripup_order.json");

struct Golden {
    input: Vec<OrderTree>,
    after_congestion: Vec<String>,
    after_slack: Vec<String>,
    deprioritised: Vec<String>,
}

fn golden() -> Golden {
    let v: serde_json::Value = serde_json::from_str(GOLDEN).expect("golden parses");
    Golden {
        input: v["input"].as_array().unwrap().iter().map(|t| OrderTree {
            net: t["net"].as_str().unwrap().to_string(),
            xmin: t["xmin"].as_i64().unwrap() as i32,
            slack: t["slack"].as_f64().unwrap() as f32,
        }).collect(),
        after_congestion: v["after_congestion"].as_array().unwrap().iter()
            .map(|s| s.as_str().unwrap().to_string()).collect(),
        after_slack: v["after_slack"].as_array().unwrap().iter()
            .map(|s| s.as_str().unwrap().to_string()).collect(),
        deprioritised: v["deprioritised"].as_array().unwrap().iter()
            .map(|s| s.as_str().unwrap().to_string()).collect(),
    }
}

#[test]
fn the_CONGESTION_sort_matches_the_reference() {
    // ⚠️ Checked on its own, so a mismatch in the final order can be attributed to one of the
    // three steps rather than to "somewhere in three steps".
    let g = golden();
    assert_eq!(order_by_congestion(&g.input), g.after_congestion);
}

#[test]
fn the_FINAL_ripup_order_matches_the_reference() {
    let g = golden();
    assert_eq!(order_for_ripup(&g.input), g.after_slack);
    assert_eq!(g.input.len(), 321);
}

#[test]
fn the_corpus_is_almost_ENTIRELY_TIES_which_is_why_stability_decides_it() {
    // 🔑 The measurement that justifies the whole test. If this ever stops being true the gate
    // has become much weaker without anyone noticing.
    let g = golden();
    let by_name: std::collections::BTreeMap<&str, i32> =
        g.input.iter().map(|t| (t.net.as_str(), t.xmin)).collect();
    let xs: Vec<i32> = g.after_congestion.iter().map(|n| by_name[n.as_str()]).collect();
    let ties = (0..xs.len() - 1).filter(|&i| xs[i] == xs[i + 1]).count();
    assert_eq!(ties, 309, "309 of 320 adjacent pairs tie");

    let mut groups: std::collections::BTreeMap<i32, usize> = std::collections::BTreeMap::new();
    for x in &xs { *groups.entry(*x).or_default() += 1; }
    let largest = *groups.values().max().unwrap();
    assert_eq!(largest, 200, "and the largest tie group is 200 of 321 nets");
}

#[test]
fn an_UNSTABLE_sort_would_be_a_different_function_and_the_golden_proves_it() {
    // ⛔ Demonstrated rather than asserted: reversing the input changes nothing about any net's
    // congestion, so a STABLE sort's output must change (ties keep the new incoming order) while
    // the reference's answer does not. That is exactly the freedom an unstable sort would take.
    let g = golden();
    let mut reversed = g.input.clone();
    reversed.reverse();
    assert_ne!(
        order_by_congestion(&reversed),
        g.after_congestion,
        "if these matched, the tie order would not depend on input order and stability would be \
         irrelevant — the premise of this whole gate"
    );
}

#[test]
fn the_deprioritised_nets_are_exactly_the_ones_the_reference_touched() {
    // The middle step: position in the congestion-sorted list, sentinel slack, and no congestion.
    let g = golden();
    let threshold = g.input.len() * DEPRIORITISE_PERCENT / 100;
    assert_eq!(threshold, 96, "321 * 30 / 100 floors to 96");

    let by_name: std::collections::BTreeMap<&str, &OrderTree> =
        g.input.iter().map(|t| (t.net.as_str(), t)).collect();
    let mut ours: Vec<String> = g.after_congestion.iter().enumerate()
        .filter(|(position, n)| {
            let t = by_name[n.as_str()];
            t.slack == SLACK_SENTINEL && t.xmin == 0 && *position >= threshold
        })
        .map(|(_, n)| n.clone())
        .collect();
    ours.sort();
    assert_eq!(ours, g.deprioritised);
    assert_eq!(ours.len(), 139);
}

#[test]
fn the_threshold_FLOORS_because_it_is_integer_arithmetic() {
    // 10 * 30 / 100 == 3, not 3.0 rounded anywhere.
    assert_eq!(10 * DEPRIORITISE_PERCENT / 100, 3);
    assert_eq!(9 * DEPRIORITISE_PERCENT / 100, 2, "and 2.7 floors to 2");
}

#[test]
fn the_sentinel_is_the_THIRTY_TWO_BIT_float_minimum_and_is_compared_for_EQUALITY() {
    // ⚠️ An f64 intermediate anywhere in the chain stops this matching.
    assert_eq!(SLACK_SENTINEL, f32::MIN);
    assert_eq!(SLACK_SENTINEL as f64 as f32, SLACK_SENTINEL);
}

// ---- the iteration schedule --------------------------------------------------------------

#[test]
fn the_maze_threshold_steps_in_THREE_BANDS_and_clamps_at_zero() {
    assert_eq!(step_threshold_m(20), 10, "above 15: minus 10");
    assert_eq!(step_threshold_m(16), 6);
    assert_eq!(step_threshold_m(15), 11, "15 is in the middle band: minus 4");
    assert_eq!(step_threshold_m(2), 0, "2 is still the middle band, and clamps");
    assert_eq!(step_threshold_m(1), 0, "below 2: straight to zero");
    assert_eq!(step_threshold_m(0), 0);
}

#[test]
fn only_the_LOW_overflow_band_also_disables_the_ripup_threshold() {
    // ⚠️ Three bands, and one of them has a side effect the others do not.
    assert_eq!(cost_and_enlarge_step(3_000), (2, 10, None));
    assert_eq!(cost_and_enlarge_step(1_000), (2, 5, None));
    assert_eq!(cost_and_enlarge_step(499), (5, 5, Some(-1)), "and it disables rip-up");
}

#[test]
fn the_logistic_coefficient_NEVER_FALLS() {
    // ⛔ It takes the maximum with its previous value, so recomputing it fresh each iteration is
    // a different schedule.
    let high = logistic_coefficient(0.0, 10, 0);
    let lower_if_recomputed = logistic_coefficient(0.0, 10_000, 0);
    assert!(lower_if_recomputed < high, "a larger overflow gives a smaller fresh value");
    assert_eq!(logistic_coefficient(high, 10_000, 0), high, "so the old value is kept");
}

#[test]
fn the_threshold_comparison_is_INCLUSIVE_and_the_golden_CANNOT_show_it() {
    // ⬜ A characterised gap. Changing `position >= threshold` to `>` passes every reference
    // check, because on this design no ELIGIBLE net sits exactly at the threshold: the threshold
    // is position 96 and the eligible nets begin at 121.
    //
    // ⛔ **And the obvious synthetic case cannot show it either.** If every net is eligible, the
    // two comparisons produce the SAME ORDER — the net at the threshold is either last of the
    // untouched group or first of the de-prioritised one, which is the same index. Only its slack
    // VALUE differs, and the order is all this function returns. A first attempt at this test
    // asserted that order and passed under the mutation, which proved nothing.
    //
    // ⟹ Distinguishing it needs an INELIGIBLE net with a middling slack sitting after the
    // threshold, so that de-prioritising the boundary net moves it ACROSS that net.
    //
    // Five nets, all uncongested so the congestion sort is a pure tie and position is the input
    // index. threshold = 5 * 30 / 100 = 1.
    let nets = vec![
        OrderTree { net: "n0".into(), xmin: 0, slack: SLACK_SENTINEL },  // below threshold
        OrderTree { net: "n1".into(), xmin: 0, slack: SLACK_SENTINEL },  // AT the threshold
        OrderTree { net: "n2".into(), xmin: 0, slack: -1.0 },            // ineligible, middling
        OrderTree { net: "n3".into(), xmin: 0, slack: SLACK_SENTINEL },  // above threshold
        OrderTree { net: "n4".into(), xmin: 0, slack: -2.0 },            // ineligible, lower
    ];
    assert_eq!(nets.len() * DEPRIORITISE_PERCENT / 100, 1, "the threshold is position 1");

    // Inclusive (`>=`): n1 is de-prioritised to f32::MAX and lands AFTER n2 and n4.
    // Exclusive (`>`) would keep n1 at the sentinel, putting it SECOND, before both.
    assert_eq!(
        order_for_ripup(&nets),
        vec!["n0", "n4", "n2", "n1", "n3"],
        "n1 sits at the threshold and IS de-prioritised, so it crosses the ineligible nets"
    );
}

#[test]
fn the_reference_corpus_has_NO_eligible_net_at_the_threshold_and_says_so() {
    // ⚠️ Asserted so the gap announces itself if a future golden does exercise the boundary.
    let g = golden();
    let threshold = g.input.len() * DEPRIORITISE_PERCENT / 100;
    let by_name: std::collections::BTreeMap<&str, &OrderTree> =
        g.input.iter().map(|t| (t.net.as_str(), t)).collect();
    let at_threshold = g.after_congestion.iter().enumerate()
        .filter(|(position, n)| {
            let t = by_name[n.as_str()];
            t.slack == SLACK_SENTINEL && t.xmin == 0 && *position == threshold
        })
        .count();
    assert_eq!(at_threshold, 0,
        "an eligible net now sits exactly at the threshold, so the golden CAN decide `>=` vs `>` \
         — the synthetic case above is no longer the only witness");
}
