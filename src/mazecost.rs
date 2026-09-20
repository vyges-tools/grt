// SPDX-License-Identifier: Apache-2.0
//! The maze router's edge-cost tables.
//!
//! Every edge the maze router considers is priced by a table lookup on its usage. The curve is a
//! logistic in the usage, plus a **linear ramp** once the usage passes the capacity — so an
//! overloaded edge keeps getting more expensive instead of levelling off at the logistic's
//! ceiling.
//!
//! ⛔ **Two tables here, one per direction, each against its own capacity.** The monotonic stage
//! earlier in the pipeline prices vertical edges from the *horizontal* table; this stage does not.
//! The two stages look similar and are not.
//!
//! ⚠️ **The span is forty times the capacity**, where the monotonic table spans ten and the
//! runaway-usage check allows a hundred. Three different multiples of capacity in three places,
//! none of them derived from the others.

/// How far past capacity the tables run.
pub const MAX_USAGE_MULTIPLIER: i32 = 40;

/// The knobs the congestion loop varies between iterations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CostParams {
    /// Divides the linear ramp past capacity — a **larger** slope makes overflow cheaper.
    pub slope: i32,
    pub logistic_coef: f64,
    pub cost_height: f64,
}

/// Price one edge at a given usage.
///
/// ⚠️ **The ramp is added on top of the logistic, not instead of it**, and only at or past
/// capacity. At exactly capacity the ramp contributes zero, so the two pieces meet continuously.
///
/// 🔑 **That continuity makes the boundary test unfalsifiable, and deliberately so.** Both
/// `index > capacity` and `index > capacity - 1` reproduce every captured entry — the first
/// because the term it skips is zero, the second because it is the same condition written
/// differently. Neither is a gap in the corpus; there is nothing there to catch.
///
/// ⚠️ `cost_height / slope` is a real division by an integer knob; the reference does not
/// pre-compute it, so neither does this.
pub fn get_cost(index: i32, capacity: i32, params: &CostParams) -> f64 {
    let x = f64::from(capacity - index) * params.logistic_coef;
    let mut cost = params.cost_height / (x.exp() + 1.0) + 1.0;
    if index >= capacity {
        cost += params.cost_height / f64::from(params.slope) * f64::from(index - capacity);
    }
    cost
}

/// Build one direction's table.
///
/// ⚠️ The reference builds this with a memo that **never hits while building** — it checks
/// whether the index is already in the table it is appending to, which it cannot be. The memo is
/// for later lookups; during construction every entry is computed.
pub fn cost_table(capacity: i32, params: &CostParams) -> Vec<f64> {
    let n = (MAX_USAGE_MULTIPLIER * capacity).max(0);
    (0..n).map(|i| get_cost(i, capacity, params)).collect()
}
