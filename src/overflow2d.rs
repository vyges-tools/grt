// SPDX-License-Identifier: Apache-2.0
//! The two planar overflow scans — `getOverflow2Dmaze` (R15, after the maze passes) and
//! `getOverflow2D` (R7/R8/R10, after the pattern passes).
//!
//! ⭐ **Diffed, not read separately.** The two reference functions are one skeleton with two
//! differences, and the differences are the whole point:
//!
//! | | maze scan | pattern scan |
//! | --- | --- | --- |
//! | reads | COMMITTED usage (`u16`) | ESTIMATED usage (`f64`) |
//! | total | an exact integer sum | `int += double`: TRUNCATED at every addition |
//! | overflow | `usage - cap`, exact | `(int)(est - cap)`: truncated toward zero |
//!
//! Both scan only the **used-grid sets**, never the whole grid: a cell enters a set when usage is
//! added to it and leaves only when the sets are cleared. So a cell whose usage fell back to zero
//! is still visited (contributing nothing), and a cell that gained usage by a path that does not
//! insert would be missed. The caller passes the sets, in their order.
//!
//! Both also set the maze router's history threshold `ahth` — 30 above 800,000 total usage, else
//! 20 — a side effect a later stage reads.
//!
//! ⚠️ The reference runs `check2DEdgesUsage` first ([`crate::check_2d_edges_usage`]); it can only
//! abort the run, never change what these return, so it is the caller's step, not folded in here.

/// One cell of a used-grid set.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UsedCell {
    pub x: i32,
    pub y: i32,
    pub usage: u16,
    pub est_usage: f64,
    pub cap: u16,
}

/// What a scan returns and sets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Overflow2DScan {
    /// Returned, and stored as the router's `total_overflow_`.
    pub total_overflow: i32,
    /// Written through the `maxOverflow` out-parameter.
    pub max_overflow: i32,
    /// The maze scan writes it through `tUsage`; the pattern scan only uses it for `ahth`.
    pub total_usage: i32,
    pub ahth: i32,
}

/// ⚠️ Strictly greater than 800,000.
pub fn history_threshold(total_usage: i32) -> i32 {
    if total_usage > 800_000 {
        30
    } else {
        20
    }
}

/// The skeleton both scans share: horizontal set, then vertical, summing per-cell overflow where
/// it is positive and keeping each direction's maximum.
fn scan(
    h: &[UsedCell],
    v: &[UsedCell],
    add_usage: impl Fn(i32, &UsedCell) -> i32,
    overflow_of: impl Fn(&UsedCell) -> i32,
) -> Overflow2DScan {
    let mut total_usage = 0;
    let mut dir = |cells: &[UsedCell]| {
        let (mut sum, mut max) = (0, 0);
        for c in cells {
            total_usage = add_usage(total_usage, c);
            let overflow = overflow_of(c);
            if overflow > 0 {
                sum += overflow;
                max = max.max(overflow);
            }
        }
        (sum, max)
    };
    let (h_overflow, max_h) = dir(h);
    let (v_overflow, max_v) = dir(v);
    Overflow2DScan {
        total_overflow: h_overflow + v_overflow,
        max_overflow: max_h.max(max_v),
        total_usage,
        ahth: history_threshold(total_usage),
    }
}

/// `getOverflow2Dmaze` — committed usage, exact integer arithmetic.
pub fn get_overflow_2d_maze(h: &[UsedCell], v: &[UsedCell]) -> Overflow2DScan {
    scan(h, v, |t, c| t + i32::from(c.usage), |c| i32::from(c.usage) - i32::from(c.cap))
}

/// `getOverflow2D` — ESTIMATED usage.
///
/// ⛔ `total_usage` is an `int` accumulating `double`s: each `+=` converts the sum back to `int`,
/// truncating toward zero, so fractions are lost per cell, not once at the end. ⛔ The per-cell
/// overflow is `(int)(est - cap)`, so an overflow below one whole unit counts as none.
pub fn get_overflow_2d(h: &[UsedCell], v: &[UsedCell]) -> Overflow2DScan {
    scan(
        h,
        v,
        |t, c| (f64::from(t) + c.est_usage) as i32,
        |c| (c.est_usage - f64::from(c.cap)) as i32,
    )
}
