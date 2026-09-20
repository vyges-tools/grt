// SPDX-License-Identifier: Apache-2.0
//! Stage R6 — laying down the first usage estimate, before anything is routed.
//!
//! Every segment contributes to a 2D usage grid, and only then does L-routing look at it. The
//! two passes are separate on purpose: a fused loop would route the first net against an
//! incomplete picture of everyone else's demand.

use crate::rsmt::Segment;

/// The estimated-demand grid: one accumulator per horizontal and per vertical edge.
///
/// ⚠️ **The accumulator is floating point, not integral**, because a diagonal segment contributes
/// *half* its cost to each of four runs.
///
/// ⛔ **The two arrays have DIFFERENT shapes, and neither is the cell grid.** Edges live between
/// cells, so on an `x × y` grid there are `(x-1) × y` horizontal edges and `x × (y-1)` vertical
/// ones. The reference's accessors do **no bounds checking**, so reading the missing last column
/// or row returns whatever is in memory — which is how this was found: a first capture dumped
/// `h` at `x = x_grids - 1` and got `4.7e170`.
#[derive(Debug, Clone, PartialEq)]
pub struct EstimateGrid {
    pub x_grids: usize,
    pub y_grids: usize,
    h: Vec<f64>,
    v: Vec<f64>,
    /// Committed demand, as distinct from the estimate above.
    ///
    /// ⚠️ The two are separate all the way through the estimating stages; the estimate is folded
    /// into this one only when the router switches to maze routing, and it is **added**, not
    /// moved — the estimate is left in place.
    usage_h: Vec<f64>,
    usage_v: Vec<f64>,
}

impl EstimateGrid {
    pub fn new(x_grids: usize, y_grids: usize) -> Self {
        EstimateGrid {
            x_grids,
            y_grids,
            h: vec![0.0; x_grids.saturating_sub(1) * y_grids],
            v: vec![0.0; x_grids * y_grids.saturating_sub(1)],
            usage_h: vec![0.0; x_grids.saturating_sub(1) * y_grids],
            usage_v: vec![0.0; x_grids * y_grids.saturating_sub(1)],
        }
    }

    /// Committed horizontal demand on the edge leaving cell `(x, y)`.
    pub fn usage_h(&self, x: usize, y: usize) -> f64 {
        self.usage_h[y * self.h_columns() + x]
    }
    /// Committed vertical demand on the edge leaving cell `(x, y)`.
    pub fn usage_v(&self, x: usize, y: usize) -> f64 {
        self.usage_v[y * self.x_grids + x]
    }

    /// Charge one horizontal edge's committed demand — the reference's `updateUsageH`.
    ///
    /// ⚠️ A single edge, not a run: the walk that uses this charges each step as it takes it.
    pub fn update_usage_h(&mut self, x: i32, y: i32, amount: f64) {
        let i = y as usize * self.h_columns() + x as usize;
        self.usage_h[i] += amount;
    }
    /// Charge one vertical edge's committed demand — the reference's `updateUsageV`.
    pub fn update_usage_v(&mut self, x: i32, y: i32, amount: f64) {
        let i = y as usize * self.x_grids + x as usize;
        self.usage_v[i] += amount;
    }

    /// Fold the estimate into the committed demand — the reference's `addEstUsageToUsage`.
    ///
    /// ⛔ **Adds rather than replaces, and leaves the estimate untouched.** Every edge is visited,
    /// including ones no net ever reached.
    pub fn add_est_usage_to_usage(&mut self) {
        for (u, e) in self.usage_h.iter_mut().zip(self.h.iter()) {
            *u += *e;
        }
        for (u, e) in self.usage_v.iter_mut().zip(self.v.iter()) {
            *u += *e;
        }
    }
    /// Number of horizontal edges across: one fewer than the cell columns.
    pub fn h_columns(&self) -> usize {
        self.x_grids.saturating_sub(1)
    }
    /// Number of vertical edges down: one fewer than the cell rows.
    pub fn v_rows(&self) -> usize {
        self.y_grids.saturating_sub(1)
    }
    /// Demand on the horizontal edge leaving cell `(x, y)`. Panics past the last edge, where the
    /// reference would read out of bounds silently.
    pub fn h(&self, x: usize, y: usize) -> f64 {
        assert!(x < self.h_columns(), "no horizontal edge at x={x}; there are {}", self.h_columns());
        self.h[y * self.h_columns() + x]
    }
    /// Demand on the vertical edge leaving cell `(x, y)`.
    pub fn v(&self, x: usize, y: usize) -> f64 {
        assert!(y < self.v_rows(), "no vertical edge at y={y}; there are {}", self.v_rows());
        self.v[y * self.x_grids + x]
    }
    fn add_h(&mut self, x: usize, y: usize, amount: f64) {
        let i = y * self.h_columns() + x;
        self.h[i] += amount;
    }
    fn add_v(&mut self, x: usize, y: usize, amount: f64) {
        let i = y * self.x_grids + x;
        self.v[i] += amount;
    }

    /// Add demand along a horizontal run, from `x1` to `x2` on row `y`.
    ///
    /// ⚠️ **Half-open**: the edge leaving the last cell is not charged, because the run ends
    /// there. `x1 <= x2` is guaranteed by the segment ordering upstream.
    pub fn update_h(&mut self, x1: i32, x2: i32, y: i32, amount: f64) {
        for x in x1..x2 {
            self.add_h(x as usize, y as usize, amount);
        }
    }
    /// Add demand along a vertical run, from `y1` to `y2` in column `x`.
    pub fn update_v(&mut self, x: i32, y1: i32, y2: i32, amount: f64) {
        for y in y1..y2 {
            self.add_v(x as usize, y as usize, amount);
        }
    }
}

/// Charge one segment's demand onto the grid.
///
/// Three cases, and the third is the one with a rule in it:
///
/// | segment | charged |
/// | --- | --- |
/// | vertical (`x1 == x2`) | the full cost, down one column |
/// | horizontal (`y1 == y2`) | the full cost, along one row |
/// | diagonal | ⛔ **HALF the cost, four times** — down *both* columns and along *both* rows |
///
/// ⛔ **The diagonal case is the whole point of estimating.** The segment will eventually be
/// routed as an L, one way or the other, and which way is not known yet — so both possible paths
/// are charged at half weight. Charging the full cost to one of them would bias every later
/// decision toward the other.
///
/// ⚠️ **Only `y` is normalised, never `x`.** That is not an oversight: segments arrive ordered by
/// x alone, so `x1 <= x2` already holds while `y` may run either way. An upstream change to
/// lexicographic ordering would silently break this.
///
/// ⚠️ The halving is a **float** division of an `i8` cost. Both `f32` and `f64` represent
/// `i8 / 2` exactly, so the width does not change the answer here — stated rather than assumed.
pub fn estimate_one_seg(grid: &mut EstimateGrid, seg: &Segment) {
    let cost = seg.edge_cost as f64;
    let (ymin, ymax) = (seg.y1.min(seg.y2), seg.y1.max(seg.y2));

    if seg.x1 == seg.x2 {
        grid.update_v(seg.x1, ymin, ymax, cost);
    } else if seg.y1 == seg.y2 {
        grid.update_h(seg.x1, seg.x2, seg.y1, cost);
    } else {
        let half = cost / 2.0;
        grid.update_v(seg.x1, ymin, ymax, half);
        grid.update_v(seg.x2, ymin, ymax, half);
        grid.update_h(seg.x1, seg.x2, seg.y1, half);
        grid.update_h(seg.x1, seg.x2, seg.y2, half);
    }
}

/// Whether a segment needs L-routing at all.
///
/// ⚠️ **Only diagonal segments are L-routed.** A straight one is already its own route, and the
/// estimate pass has charged it in full.
pub fn needs_l_route(seg: &Segment) -> bool {
    seg.x1 != seg.x2 && seg.y1 != seg.y2
}

/// The first-time estimate pass over every net's segments.
///
/// ⛔ **This is pass ONE of TWO, and they are separate for a reason.** Every segment of every net
/// is estimated before any segment is L-routed, so the first net routes against the complete
/// picture of demand rather than against a grid that only its predecessors have filled in. Fusing
/// the loops changes every routing decision after the first net.
pub fn estimate_all(grid: &mut EstimateGrid, nets: &[Vec<Segment>]) {
    for segments in nets {
        for seg in segments {
            estimate_one_seg(grid, seg);
        }
    }
}

/// The fraction of an edge's capacity below which congestion costs nothing.
///
/// ⛔ **`f32`, and that is load-bearing.** The reference computes the bound as
/// `float LB = 0.9; lb = LB * capacity`, in single precision, and only then compares it against a
/// `double` demand. `0.9f32` is `0.899999976…` while `0.9f64` is `0.900000000…`, so on a capacity
/// of 10 the two bounds are `8.99999976` and `9.00000000`. A demand of exactly `9.0` overflows
/// a demand landing between them overflows under one and not the other.
///
/// ⬜ **Kept in `f32` because that is what the reference does, NOT because a difference has been
/// observed.** Recomputing the bound in `f64` passes every reference check on the designs scored
/// so far — no demand lands in the gap. The threshold genuinely differs; the *behaviour*
/// difference is unwitnessed, and saying so beats implying it was caught.
pub const CAPACITY_LOWER_BOUND_FRACTION: f32 = 0.9;

/// The congestion-free allowance for an edge of the given capacity.
pub fn capacity_lower_bound(capacity: i32) -> f32 {
    CAPACITY_LOWER_BOUND_FRACTION * capacity as f32
}

/// Which way a diagonal segment bends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LShape {
    /// Up the first column, then across the far row — the reference's `xFirst == false`.
    YFirst,
    /// Across the near row, then up the far column — the reference's `xFirst == true`.
    XFirst,
}

/// Congestion cost of one edge: demand above the allowance, and nothing below it.
///
/// ⚠️ **Demand includes the edge's blockage**, not just routing demand — the reference adds the
/// reduction term before comparing. ⚠️ The bound is `f32` and widens here; see
/// [`CAPACITY_LOWER_BOUND_FRACTION`].
pub fn congestion_cost(demand: f64, blockage: u16, lower_bound: f32) -> f64 {
    (demand + blockage as f64 - lower_bound as f64).max(0.0)
}

/// Choose which way a diagonal segment bends, by comparing the congestion on the two L paths.
///
/// The two candidates share their endpoints and differ in where they turn:
///
/// | shape | path | cost is |
/// | --- | --- | --- |
/// | [`LShape::YFirst`] | up column `x1`, then along row `y2` | column `x1` + row `y2` |
/// | [`LShape::XFirst`] | along row `y1`, then up column `x2` | column `x2` + row `y1` |
///
/// ⛔ **A TIE picks `XFirst`.** The comparison is a strict `costL1 < costL2`, so equal costs fall
/// to the else. Writing it as `<=` flips every tied segment, and ties are common because most
/// edges carry no congestion at all and contribute zero to both sides.
pub fn choose_l_shape(cost_y_first: f64, cost_x_first: f64) -> LShape {
    if cost_y_first < cost_x_first {
        LShape::YFirst
    } else {
        LShape::XFirst
    }
}

/// Commit a bend: give the chosen path the other half of the cost and take it back from the one
/// not taken.
///
/// ⚠️ The estimate pass charged **half** to each candidate. Committing adds another half to the
/// winner and subtracts a half from the loser, which leaves the winner at full cost and the loser
/// at zero — without either ever being recomputed from scratch.
pub fn commit_l_shape(grid: &mut EstimateGrid, seg: &Segment, shape: LShape) {
    let half = seg.edge_cost as f64 / 2.0;
    let (ymin, ymax) = (seg.y1.min(seg.y2), seg.y1.max(seg.y2));
    match shape {
        LShape::YFirst => {
            grid.update_v(seg.x1, ymin, ymax, half);
            grid.update_v(seg.x2, ymin, ymax, -half);
            grid.update_h(seg.x1, seg.x2, seg.y2, half);
            grid.update_h(seg.x1, seg.x2, seg.y1, -half);
        }
        LShape::XFirst => {
            grid.update_h(seg.x1, seg.x2, seg.y1, half);
            grid.update_h(seg.x1, seg.x2, seg.y2, -half);
            grid.update_v(seg.x2, ymin, ymax, half);
            grid.update_v(seg.x1, ymin, ymax, -half);
        }
    }
}

/// One edge whose committed demand has run past what the check allows.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UsageViolation {
    pub x: usize,
    pub y: usize,
    pub horizontal: bool,
    pub usage: f64,
    pub limit: i32,
}

/// How many times a layer's capacity an edge may carry before the check fires.
const MAX_USAGE_MULTIPLIER: i32 = 100;

/// Check no edge's committed demand has run away — the reference's `check2DEdgesUsage`.
///
/// ⚠️ **Strictly greater than the limit**, and the limit is a whole multiple of the capacity, so
/// an edge sitting exactly on it passes.
///
/// The reference raises an error per offending edge (GRT-228 horizontal, GRT-229 vertical) and
/// stops. Violations are returned here instead, so a caller can report them all and so the rule
/// is testable without a design that triggers it — no shipped design does.
pub fn check_2d_edges_usage(
    grid: &EstimateGrid,
    h_capacity: i32,
    v_capacity: i32,
) -> Vec<UsageViolation> {
    let mut out = Vec::new();
    let (h_limit, v_limit) = (MAX_USAGE_MULTIPLIER * h_capacity, MAX_USAGE_MULTIPLIER * v_capacity);
    for y in 0..grid.y_grids {
        for x in 0..grid.h_columns() {
            let usage = grid.usage_h(x, y);
            if usage > f64::from(h_limit) {
                out.push(UsageViolation { x, y, horizontal: true, usage, limit: h_limit });
            }
        }
    }
    for y in 0..grid.v_rows() {
        for x in 0..grid.x_grids {
            let usage = grid.usage_v(x, y);
            if usage > f64::from(v_limit) {
                out.push(UsageViolation { x, y, horizontal: false, usage, limit: v_limit });
            }
        }
    }
    out
}
