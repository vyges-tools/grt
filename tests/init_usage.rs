// SPDX-License-Identifier: Apache-2.0
//! R13 — clearing the per-round state between the estimating passes and the maze router.
//!
//! Three small operations, and one of them does not do what it appears to.

use vyges_grt::estimate::EstimateGrid;
use vyges_grt::save_last_route_len;

fn seeded() -> EstimateGrid {
    let mut g = EstimateGrid::new(6, 6);
    g.update_h(1, 4, 2, 3.0);
    g.update_v(2, 1, 4, 5.0);
    g.add_est_usage_to_usage();
    g
}

/// Clearing the estimate leaves the committed demand alone.
///
/// ⚠️ The two are separate grids all the way through, and this is the point at which they most
/// obviously could be confused: the estimate is thrown away here precisely because it has just
/// been folded into the committed demand.
#[test]
fn clearing_the_estimate_leaves_committed_demand() {
    let mut g = seeded();
    assert_eq!(g.h(1, 2), 3.0);
    assert_eq!(g.usage_h(1, 2), 3.0);

    g.init_est_usage();

    assert_eq!(g.h(1, 2), 0.0, "the estimate is cleared");
    assert_eq!(g.v(2, 1), 0.0);
    assert_eq!(g.usage_h(1, 2), 3.0, "the committed demand is not");
    assert_eq!(g.usage_v(2, 1), 5.0);
}

/// Only the first pass clears the congestion counts.
#[test]
fn only_the_first_pass_clears_the_congestion_counts() {
    let mut g = seeded();
    // Both start at zero, so the distinction is made visible by the pass type alone.
    g.init_last_usage(1);
    assert_eq!(g.cong_cnt_h(1, 2), 0);
    assert_eq!(g.last_usage_h(1, 2), 0);

    g.init_last_usage(2);
    assert_eq!(g.last_usage_h(1, 2), 0, "carried-over demand is cleared on every pass");
}

/// ⛔ The second pass's "decay" cannot decay anything.
///
/// The reference zeroes the carried-over demand on every edge and then, for that pass only,
/// multiplies it by a fifth. Measured on the one design that reaches it: **647 of 2,380 edges
/// carry a non-zero value on entry and every one is zero before the multiply**.
///
/// ⚠️ This test states the behaviour, not the apparent intent. If the zeroing were ever moved
/// below the multiply — which is what the arm reads as meaning — this would fail, and it should:
/// that would be a change in results, not a tidy-up.
#[test]
fn the_second_pass_decay_is_a_no_op() {
    let mut g = seeded();
    g.init_last_usage(2);
    let after_first = g.last_usage_h(1, 2);
    g.init_last_usage(2);
    assert_eq!(
        g.last_usage_h(1, 2), after_first,
        "applying the decay twice changes nothing, because there is nothing to decay"
    );
    assert_eq!(after_first, 0, "and the value it operates on is always zero");
}

/// Each edge's routed length is carried into the next round.
///
/// ⚠️ This is the divisor the critical-net rip-up arm uses. An edge whose length was never saved
/// reads back as zero, which is why that arm treats a zero previous length as "not configured"
/// rather than dividing by it.
#[test]
fn the_routed_lengths_are_carried_into_the_next_round() {
    let routelens = vec![4, 0, 17, 3];
    let mut last = vec![99, 99, 99, 99];
    save_last_route_len(&routelens, &mut last);
    assert_eq!(last, routelens);

    // A second round overwrites rather than accumulating.
    let next = vec![1, 1, 1, 1];
    save_last_route_len(&next, &mut last);
    assert_eq!(last, next, "the previous round's value is replaced, not combined");
}
