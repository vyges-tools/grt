// SPDX-License-Identifier: Apache-2.0
//! Stage R10 — Z-routing: re-routing a long, congested diagonal edge with **two** bends instead
//! of one.
//!
//! An L route turns once; a Z route turns twice, so it has a free parameter — where the middle
//! segment sits. Two families are tried and the cheapest position across both wins:
//!
//! - **HVH**: along row `y1`, up column `bestZ`, along row `y2`. The parameter is a **column**.
//! - **VHV**: up column `x1`, along row `bestZ`, up column `x2`. The parameter is a **row**.
//!
//! ⚠️ Only edges longer than a threshold, never straight ones, and only when the rip-up gate says
//! the existing route is congested enough to be worth replacing.
//!
//! ⛔ **Every via term here is dead.** `via_cost_` is an `int` and is 0 whenever this stage runs,
//! so both family base costs and both endpoint penalties contribute nothing. The corpus asserts
//! the zero rather than relying on it.
//!
//! ⛔ **Two arrays the reference fills here are never read.** `cost_v_test_` and `cost_tb_test_`
//! are written throughout and no line combines them into `cost_hvh_test_`. The sibling routine
//! for two-terminal nets *does* combine them, and indexes `cost_tb_test_[i]` where this one
//! writes `cost_tb_test_[0]`. They are dead stores here, so they are not carried — omitting a
//! store nothing reads cannot change an answer, and carrying arrays nothing reads would suggest
//! they matter.
//!
//! ℹ️ `cost_hvh_test_` is sized from the y range while being indexed over a width, which looks
//! wrong and is not: both ranges are set to `max(x_grid, y_grid)`.

use crate::estimate::{congestion_cost, EstimateGrid};
use crate::lroute::{mark_h, mark_v};
use crate::spiral::SpiralNode;

/// The fixed penalty a congested edge adds to the tie-break cost.
///
/// ⚠️ Applied to the *test* cost only, and only when the edge is over capacity. Under capacity,
/// the negative slack itself is accumulated instead, so the tie-break prefers emptier edges.
pub const HCOST: f64 = 5000.0;

/// Where the middle segment of the Z sits, and which family it belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ZChoice {
    /// `true` for the horizontal-vertical-horizontal family.
    pub hvh: bool,
    /// A column when `hvh`, a row otherwise.
    pub z_point: i32,
}

/// The reference's `BIG_INT`, used as the starting "no candidate yet" cost.
const BIG_INT: f64 = i32::MAX as f64;

/// Accumulate one edge's congestion into a running cost and its tie-break partner.
///
/// ⚠️ Asymmetric on purpose: over capacity adds the real overflow to the cost and a flat penalty
/// to the tie-break; under capacity adds **nothing** to the cost and the negative slack to the
/// tie-break.
fn add_congestion(over: f64, cost: &mut f64, cost_test: &mut f64) {
    if over > 0.0 {
        *cost += over;
        *cost_test += HCOST;
    } else {
        *cost_test += over;
    }
}

/// Re-route one edge as a Z, choosing the cheapest middle segment.
///
/// ⛔ **The two endpoint statuses are read from the ALIAS nodes**, not from the edge's own
/// endpoints, and every mark and counter this function writes lands on the aliases too.
///
/// ⛔ **The via bias is crossed between the endpoints**: `status1` biases the family that would
/// need a via at `n1`, `status2` the opposite family at `n2`.
///
/// ⚠️ `x2 - x1` is used as a width with no ordering, exactly as the reference does — the caller
/// only reaches here for edges that are neither straight nor reversed.
#[allow(clippy::too_many_arguments)]
pub fn newroute_z(
    grid: &mut EstimateGrid,
    nodes: &mut [SpiralNode],
    n1a: usize,
    n2a: usize,
    (x1, y1): (i32, i32),
    (x2, y2): (i32, i32),
    edge_cost: i8,
    via_cost: f64,
    v_lb: f32,
    h_lb: f32,
    red_v: &dyn Fn(usize, usize) -> u16,
    red_h: &dyn Fn(usize, usize) -> u16,
) -> ZChoice {
    let status1 = nodes[n1a].status;
    let status2 = nodes[n2a].status;

    let seg_width = (x2 - x1) as usize;
    let y1_smaller = y1 < y2;
    let (ymin, ymax) = (y1.min(y2), y1.max(y2));
    let seg_height = (ymax - ymin) as usize;

    // A node connected only vertically already needs a via to start horizontally, and vice versa.
    let hvh_base = if status1 == 1 { via_cost } else { 0.0 };
    let vhv_base = if status1 == 2 { via_cost } else { 0.0 };
    let mut cost_hvh = vec![hvh_base; seg_width];
    let mut cost_hvh_test = vec![hvh_base; seg_width];
    let mut cost_vhv = vec![vhv_base; seg_height];

    // ⚠️ The second endpoint penalises the OPPOSITE family to the first.
    //
    // ⛔ Dead in the shipped flow: `via_cost` is 0 every time this stage runs, so neither arm
    // changes a cost. Transcribed and asserted as zero by the corpus rather than claimed as
    // validated — swapping the two status values is a mutation nothing can kill.
    if status2 == 2 {
        for c in cost_vhv.iter_mut() {
            *c += via_cost;
        }
    } else if status2 == 1 {
        for i in 0..seg_width {
            cost_hvh[i] += via_cost;
            cost_hvh_test[i] += via_cost;
        }
    }

    let over_v = |grid: &EstimateGrid, x: usize, y: usize| -> f64 {
        grid.v(x, y) + f64::from(red_v(x, y)) - f64::from(v_lb)
    };
    let over_h = |grid: &EstimateGrid, x: usize, y: usize| -> f64 {
        grid.h(x, y) + f64::from(red_h(x, y)) - f64::from(h_lb)
    };

    // The vertical middle segment, per candidate column.
    let mut cost_v = vec![0.0; seg_width];
    let mut sink = 0.0;
    for i in x1..x2 {
        for j in ymin..ymax {
            let over = over_v(grid, i as usize, j as usize);
            add_congestion(over, &mut cost_v[(i - x1) as usize], &mut sink);
        }
    }

    // The two horizontal boundary runs, as a running total across candidate columns: moving the
    // Z one column right adds a span of row y1 and gives back a span of row y2.
    let mut cost_tb = vec![0.0; seg_width];
    for j in x1..x2 {
        let over = over_h(grid, j as usize, y2 as usize);
        add_congestion(over, &mut cost_tb[0], &mut sink);
    }
    for i in 1..seg_width {
        cost_tb[i] = cost_tb[i - 1];
        let gained = over_h(grid, (x1 + i as i32 - 1) as usize, y1 as usize);
        add_congestion(gained, &mut cost_tb[i], &mut sink);
        let given_back = over_h(grid, (x1 + i as i32 - 1) as usize, y2 as usize);
        if given_back > 0.0 {
            cost_tb[i] -= given_back;
        }
    }

    // The horizontal middle segment, per candidate row.
    //
    // ⚠️ No tie-break partner here: the reference accumulates `max(0, over)` directly.
    //
    // 🔑 For the cost itself this is the same function as the accumulator used above — both add
    // the overflow when positive and nothing otherwise — and the two differ only in the
    // tie-break total, which nothing here reads. Written the reference's way rather than reusing
    // the accumulator, so the absence of a tie-break partner stays visible.
    let mut cost_h = vec![0.0; seg_height];
    for i in ymin..ymax {
        for j in x1..x2 {
            cost_h[(i - ymin) as usize] +=
                congestion_cost(grid.h(j as usize, i as usize), red_h(j as usize, i as usize), h_lb);
        }
    }

    // The two vertical boundary runs, as a running total across candidate rows.
    let mut cost_lr = vec![0.0; seg_height];
    for j in ymin..ymax {
        cost_lr[0] += if y1_smaller {
            congestion_cost(grid.v(x2 as usize, j as usize), red_v(x2 as usize, j as usize), v_lb)
        } else {
            // ⛔ The reference asks for the UNREDUCED usage on this one branch alone — no
            // blockage term — and flags it in its own comment. Transcribed, not levelled out.
            congestion_cost(grid.v(x1 as usize, j as usize), 0, v_lb)
        };
    }
    let (lr_near, lr_far) = if y1_smaller { (x1, x2) } else { (x2, x1) };
    for i in 1..seg_height {
        cost_lr[i] = cost_lr[i - 1];
        let row = (ymin + i as i32 - 1) as usize;
        cost_lr[i] +=
            congestion_cost(grid.v(lr_near as usize, row), red_v(lr_near as usize, row), v_lb);
        cost_lr[i] -=
            congestion_cost(grid.v(lr_far as usize, row), red_v(lr_far as usize, row), v_lb);
    }

    // ⛔ The two families are NOT compared the same way. A horizontal candidate can win a tie on
    // its tie-break cost; a vertical one needs a strictly lower cost and never updates the
    // tie-break total. So an exact tie between the families always goes to HVH — and that part
    // decides real cases.
    //
    // 🔑 **But the horizontal tie-break itself is inert, provably.** `cost_hvh_test` only ever
    // receives two uniform fills — the family base cost, and the second endpoint's penalty
    // applied to every entry — and nothing in this function ever varies it per candidate. It is
    // therefore constant in `i`, so the `== best_cost && test < bt_test` clause can never prefer
    // one column over another: the first candidate sets `bt_test` to that constant and no later
    // comparison against it can succeed. Deleting the clause is a mutation nothing can kill, and
    // that is a property of the code rather than of the corpus.
    //
    // ⚠️ The sibling routine for two-terminal nets **does** vary it, folding the per-candidate
    // test totals in before selecting. This one has no such line, which is also why the two
    // arrays feeding it are dead here.
    let mut hvh = true;
    let mut best_cost = BIG_INT;
    let mut bt_test = BIG_INT;
    let mut best_z = 0;
    for i in 0..seg_width {
        cost_hvh[i] += cost_v[i] + cost_tb[i];
        if cost_hvh[i] < best_cost || (cost_hvh[i] == best_cost && cost_hvh_test[i] < bt_test) {
            best_cost = cost_hvh[i];
            bt_test = cost_hvh_test[i];
            best_z = i as i32 + x1;
        }
    }
    for i in 0..seg_height {
        cost_vhv[i] += cost_h[i] + cost_lr[i];
        if cost_vhv[i] < best_cost {
            best_cost = cost_vhv[i];
            best_z = i as i32 + ymin;
            hvh = false;
        }
    }

    let cost = f64::from(edge_cost);
    if hvh {
        // ⚠️ Both endpoints take the SAME mark here, unlike the single-bend route where they are
        // crossed — a Z leaves and arrives horizontally.
        mark_h(&mut nodes[n1a].status);
        mark_h(&mut nodes[n2a].status);
        nodes[n1a].h_id += 1;
        nodes[n2a].h_id += 1;
        grid.update_h(x1, best_z, y1, cost);
        grid.update_h(best_z, x2, y2, cost);
        grid.update_v(best_z, ymin, ymax, cost);
    } else {
        mark_v(&mut nodes[n1a].status);
        mark_v(&mut nodes[n2a].status);
        nodes[n1a].l_id += 1;
        nodes[n2a].l_id += 1;
        if y1_smaller {
            grid.update_v(x1, y1, best_z, cost);
            grid.update_v(x2, best_z, y2, cost);
        } else {
            grid.update_v(x2, y2, best_z, cost);
            grid.update_v(x1, best_z, y1, cost);
        }
        grid.update_h(x1, x2, best_z, cost);
    }

    ZChoice { hvh, z_point: best_z }
}
