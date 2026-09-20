// SPDX-License-Identifier: Apache-2.0
//! The reference's shared constants, pinned.
//!
//! ⛔ **A sentinel is a constant like any other.** These were originally transcribed as
//! plausible-looking values rather than read from the reference, and nothing in any corpus caught
//! it — the real quantities never approach them. A test is the only thing that can.

use vyges_grt::BIG_INT;

/// `BIG_INT` is **1e9**, declared once and used only inside the router.
///
/// Audited against the reference: **one** definition (`static const int BIG_INT = 1e9` in
/// `FastRoute.h`) and **63** uses, all within `grt` — so there is no cross-module inconsistency
/// to reconcile. Of those uses, the ones in code transcribed here are:
///
/// | site | what it is |
/// | --- | --- |
/// | the maze search | the unreached-distance sentinel, and the test for "not yet in the heap" |
/// | the two-bend router | the starting cost and its tie-break partner |
/// | the monotonic router | the starting cost |
/// | layer assignment's reset | the initial value of both per-node counters |
///
/// The rest sit in functions not yet transcribed, and this test is what will catch the value
/// being guessed again when they are.
#[test]
fn the_infinity_sentinel_is_the_reference_value() {
    assert_eq!(BIG_INT, 1e9, "the reference declares BIG_INT as 1e9");

    // ⚠️ It is half of the plausible-looking alternative, which is why no corpus distinguishes
    // them: every real cost is orders of magnitude below either.
    assert!(BIG_INT < f64::from(i32::MAX), "1e9 is smaller than i32::MAX, not equal to it");
    assert_eq!(BIG_INT as i64, 1_000_000_000);
}

/// The sentinel must exceed anything the router can actually produce, or it stops being infinity.
///
/// ⚠️ The largest cost a single edge can carry is the table's last entry — the logistic ceiling
/// plus the ramp across ten times the capacity — and a path is a few hundred of those.
#[test]
fn the_sentinel_is_far_above_any_real_cost() {
    use vyges_grt::{cost_table, CostParams};
    let params = CostParams { slope: 3, logistic_coef: 2.0, cost_height: 8.0 };
    let table = cost_table(40, &params);
    let worst = table.last().copied().expect("non-empty");

    // A generous upper bound on a path: every step at the worst price.
    let longest_conceivable_path = 10_000.0;
    assert!(
        worst * longest_conceivable_path < BIG_INT,
        "a pathological path ({}) could reach the sentinel ({BIG_INT})",
        worst * longest_conceivable_path
    );
}
