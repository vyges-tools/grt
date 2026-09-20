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
}

impl EstimateGrid {
    pub fn new(x_grids: usize, y_grids: usize) -> Self {
        EstimateGrid {
            x_grids,
            y_grids,
            h: vec![0.0; x_grids.saturating_sub(1) * y_grids],
            v: vec![0.0; x_grids * y_grids.saturating_sub(1)],
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
    fn update_h(&mut self, x1: i32, x2: i32, y: i32, amount: f64) {
        for x in x1..x2 {
            self.add_h(x as usize, y as usize, amount);
        }
    }
    /// Add demand along a vertical run, from `y1` to `y2` in column `x`.
    fn update_v(&mut self, x: i32, y1: i32, y2: i32, amount: f64) {
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
