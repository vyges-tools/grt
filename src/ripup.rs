// SPDX-License-Identifier: Apache-2.0
//! Stage R14 — the order nets are ripped up and rerouted, and the schedule that drives it.
//!
//! ⛔ **This is where sort STABILITY stops being a style question.** The congestion comparison
//! discriminates almost nothing: on a real design **309 of 320 adjacent pairs tie**, and a single
//! tie group of **200 nets — 62% of the design** — is ordered entirely by the sort being stable.
//! An unstable sort scrambles them, and with them every rip-up decision that follows.

/// A net as the rip-up ordering sees it.
#[derive(Debug, Clone, PartialEq)]
pub struct OrderTree {
    pub net: String,
    /// Total congestion over the net's tree: the sum of `max(0, usage - capacity)` per edge.
    pub xmin: i32,
    pub slack: f32,
}

/// The sentinel a net above the critical-slack threshold is stamped with.
///
/// ⚠️ The reference writes it as `ceil(numeric_limits<float>::lowest())`, which is that value
/// unchanged — it is already integral. Compared for **equality** later, so an `f64` intermediate
/// anywhere in the chain would stop matching.
pub const SLACK_SENTINEL: f32 = f32::MIN;

/// The share of the ordered list below which a net is never de-prioritised.
///
/// ⚠️ Applied as `len * 30 / 100` in **integer** arithmetic, so it floors.
pub const DEPRIORITISE_PERCENT: usize = 30;

/// Order the nets for rip-up, the way stage R14 does.
///
/// Three steps, and the middle one is why they cannot be collapsed:
///
/// 1. **stable** sort by congestion, **descending** — most congested first;
/// 2. a de-prioritisation pass that reads each net's **position in the list step 1 just
///    produced**;
/// 3. **stable** sort by slack, **ascending**.
///
/// ⛔ **Step 2 depends on step 1's output positions**, not on any property of the net, so the two
/// sorts cannot be fused into one comparison. ⛔ **Both sorts must be stable**: ties keep the
/// incoming order, which is ultimately the name-sorted net order, and that is what decides 200 of
/// 321 nets on a real design.
///
/// Returns the final order.
pub fn order_for_ripup(nets: &[OrderTree]) -> Vec<String> {
    order_trees_for_ripup(nets).into_iter().map(|t| t.net).collect()
}

/// [`order_for_ripup`] returning the whole records — ⛔ with the slacks step 2 stamped, which the
/// reference writes back onto its nets (`setSlack`) and which persist into later rounds.
pub fn order_trees_for_ripup(nets: &[OrderTree]) -> Vec<OrderTree> {
    let mut v: Vec<OrderTree> = nets.to_vec();

    // 1 · descending congestion, STABLE
    v.sort_by(|a, b| b.xmin.cmp(&a.xmin));

    // 2 · de-prioritise uncongested non-critical nets in the tail
    //
    // ⚠️ The threshold is a position in the list above, and the comparison is `>=`. A net is
    // touched only when it is BOTH already carrying the sentinel AND has no congestion at all.
    let threshold = v.len() * DEPRIORITISE_PERCENT / 100;
    for (position, t) in v.iter_mut().enumerate() {
        if t.slack == SLACK_SENTINEL && t.xmin == 0 && position >= threshold {
            t.slack = f32::MAX;
        }
    }

    // 3 · ascending slack, STABLE
    v.sort_by(|a, b| a.slack.partial_cmp(&b.slack).expect("slack is never NaN"));
    v
}

/// The order after step 1 alone — exposed so a divergence can be attributed to one sort.
///
/// ⚠️ Without this, a mismatch in the final order says only "somewhere in three steps".
pub fn order_by_congestion(nets: &[OrderTree]) -> Vec<String> {
    let mut v: Vec<OrderTree> = nets.to_vec();
    v.sort_by(|a, b| b.xmin.cmp(&a.xmin));
    v.into_iter().map(|t| t.net).collect()
}

/// The constants that drive one congestion iteration.
///
/// ⛔ **These are tuned values, not a formula.** There is nothing to derive: an off-by-one in the
/// schedule changes the route. Named as the reference names them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Schedule {
    pub enlarge: i32,
    pub costheight: i32,
    pub threshold_m: i32,
    pub via_cost: i32,
    pub ripup_threshold: i32,
    pub layer_orientation: i32,
}

/// Starting values for the congestion loop.
pub const ENLARGE: i32 = 15;
pub const ESTEP1: i32 = 10;
pub const ESTEP2: i32 = 5;
pub const ESTEP3: i32 = 5;
pub const CSTEP1: i32 = 2;
pub const CSTEP2: i32 = 2;
pub const CSTEP3: i32 = 5;
pub const COSHEIGHT: i32 = 4;
pub const THRESH_M: i32 = 20;
pub const TH_STEP1: i32 = 10;
pub const TH_STEP2: i32 = 4;
pub const LV_ITER: i32 = 3;
pub const MAZE_ROUND: i32 = 500;
pub const VIA: i32 = 2;
pub const RIP_VALUE: i32 = -1;
pub const MAX_OVERFLOW_INCREASES: i32 = 25;
pub const SOFT_NDR_STAGNANT_TH: i32 = 10;
pub const SOFT_NDR_MAX_ITER: i32 = 15;

/// Step the maze-edge threshold down for the next iteration.
///
/// ⚠️ **Three bands, not a single decrement**, and the result is clamped at zero: above 15 it
/// drops by 10, from 2 to 15 by 4, and below 2 it goes straight to 0.
pub fn step_threshold_m(thresh_m: i32) -> i32 {
    let stepped = if thresh_m > 15 {
        thresh_m - TH_STEP1
    } else if thresh_m >= 2 {
        thresh_m - TH_STEP2
    } else {
        0
    };
    stepped.max(0)
}

/// Pick the cost step and the search-window growth from the current overflow.
///
/// ⚠️ **The low-overflow band also disables the rip-up threshold**, which the other two do not —
/// three bands, and only one of them has that side effect.
pub fn cost_and_enlarge_step(total_overflow: i32) -> (i32, i32, Option<i32>) {
    if total_overflow > 2_000 {
        (CSTEP1, ESTEP1, None)
    } else if total_overflow < 500 {
        (CSTEP3, ESTEP3, Some(-1)) // also sets ripup_threshold = -1
    } else {
        (CSTEP2, ESTEP2, None)
    }
}

/// The logistic coefficient for an iteration.
///
/// ⛔ **Monotone non-decreasing: it takes the MAXIMUM of the new value and the old one, so it
/// never falls.** Recomputing it fresh each iteration is a different schedule.
pub fn logistic_coefficient(previous: f32, max_overflow: i32, extra: i32) -> f32 {
    let denom = 1.0 + ((max_overflow + extra) as f32).ln();
    (2.0 / denom).max(previous)
}
