// SPDX-License-Identifier: Apache-2.0
//! Undoing a route, and the two gates that decide whether to.
//!
//! ⚠️ Not to be confused with [`crate::ripup`], which decides the **order** nets are reconsidered
//! in. This module is the reference's `RipUp.cpp`: giving back the demand a route charged, and
//! the two predicates that ask whether a route is worth replacing.
//!
//! ⛔ **The two gates read different grids.** The L gate asks about the **estimate**; the maze
//! gate asks about **committed** demand. They are not interchangeable, and each undoes the layer
//! it asked about.
//!
//! 🔑 **Every undo indexes an edge at its LOWER endpoint** — `min(a, b)` here, which is the same
//! rule the monotonic walk states as `step > 0 ? i : i - 1`. Two independent sites, one rule.

use crate::estimate::EstimateGrid;

/// A route as the rip-up code finds it.
///
/// ⚠️ Deliberately separate from the symbolic shapes fed to the expansion stage: by this point a
/// route may already have been walked out into grid points, which that enum cannot express.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RoutedShape {
    /// Nothing was routed.
    None,
    /// One bend; `x_first` is the reference's `route.xFirst`.
    L { x_first: bool },
    /// Two bends at `z_point`.
    Z { hvh: bool, z_point: i32 },
    /// Already walked out into points.
    Maze { grids: Vec<(i32, i32)>, routelen: usize },
}

/// Which demand layer an undo acts on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Layer {
    Estimate,
    Committed,
}

fn give_back_h(grid: &mut EstimateGrid, x1: i32, x2: i32, y: i32, cost: f64, layer: Layer) {
    match layer {
        Layer::Estimate => grid.update_h(x1, x2, y, -cost),
        Layer::Committed => grid.update_usage_h(x1, y, -cost),
    }
}

fn give_back_v(grid: &mut EstimateGrid, x: i32, y1: i32, y2: i32, cost: f64, layer: Layer) {
    match layer {
        Layer::Estimate => grid.update_v(x, y1, y2, -cost),
        Layer::Committed => grid.update_usage_v(x, y1, -cost),
    }
}

/// Give back the estimated demand one edge's route charged — the reference's `newRipup`.
///
/// ⛔ **A degenerate edge is skipped entirely**, not merely given back nothing: it never charged
/// anything, and its stored shape is not meaningful.
///
/// ⚠️ Each arm mirrors the stage that charged it, run in reverse with a negated cost. A shape
/// whose undo does not match its route leaves demand behind that nothing will ever remove.
pub fn new_ripup(
    grid: &mut EstimateGrid,
    (x1, y1): (i32, i32),
    (x2, y2): (i32, i32),
    shape: &RoutedShape,
    edge_cost: i8,
) {
    let cost = f64::from(edge_cost);
    let (ymin, ymax) = (y1.min(y2), y1.max(y2));
    match shape {
        RoutedShape::None => {}
        RoutedShape::L { x_first: true } => {
            give_back_h(grid, x1, x2, y1, cost, Layer::Estimate);
            give_back_v(grid, x2, ymin, ymax, cost, Layer::Estimate);
        }
        RoutedShape::L { x_first: false } => {
            give_back_v(grid, x1, ymin, ymax, cost, Layer::Estimate);
            give_back_h(grid, x1, x2, y2, cost, Layer::Estimate);
        }
        RoutedShape::Z { hvh: true, z_point } => {
            give_back_h(grid, x1, *z_point, y1, cost, Layer::Estimate);
            give_back_v(grid, *z_point, ymin, ymax, cost, Layer::Estimate);
            give_back_h(grid, *z_point, x2, y2, cost, Layer::Estimate);
        }
        RoutedShape::Z { hvh: false, z_point } => {
            // ⚠️ Tested with `<`, not `<=`: an edge with `y1 == y2` would be straight and never
            // reach the Z router at all.
            if y1 < y2 {
                give_back_v(grid, x1, y1, *z_point, cost, Layer::Estimate);
                give_back_h(grid, x1, x2, *z_point, cost, Layer::Estimate);
                give_back_v(grid, x2, *z_point, y2, cost, Layer::Estimate);
            } else {
                give_back_v(grid, x1, *z_point, y1, cost, Layer::Estimate);
                give_back_h(grid, x1, x2, *z_point, cost, Layer::Estimate);
                give_back_v(grid, x2, y2, *z_point, cost, Layer::Estimate);
            }
        }
        RoutedShape::Maze { grids, routelen } => {
            give_back_walked(grid, grids, *routelen, cost, Layer::Estimate);
        }
    }
}

/// Give back a walked route's demand, one grid step at a time.
///
/// ⛔ **The edge is indexed at the LOWER of the two endpoints.** A step is between two cells, and
/// the edge belongs to the smaller index whichever way the walk went.
///
/// ⚠️ **Only `routelen` steps are taken, not `grids.len() - 1`.** The reference sizes the point
/// buffer from the edge's length and walks the stored route length; the two agree, but the loop
/// bound is the stored one.
fn give_back_walked(
    grid: &mut EstimateGrid,
    grids: &[(i32, i32)],
    routelen: usize,
    cost: f64,
    layer: Layer,
) {
    for i in 0..routelen {
        let (ax, ay) = grids[i];
        let (bx, by) = grids[i + 1];
        if ax == bx {
            give_back_v(grid, ax, ay.min(by), ay.min(by) + 1, cost, layer);
        } else if ay == by {
            give_back_h(grid, ax.min(bx), ax.min(bx) + 1, ay, cost, layer);
        } else {
            // The reference raises an error here and stops; a diagonal step is not a route.
            panic!("diagonal step between ({ax},{ay}) and ({bx},{by}) in a walked route");
        }
    }
}

/// Give back every edge of one net — the reference's `newRipupNet`.
///
/// It is [`new_ripup`] over each edge, and is kept separate only because the reference does.
pub fn new_ripup_net(
    grid: &mut EstimateGrid,
    edges: &[((i32, i32), (i32, i32), RoutedShape)],
    edge_cost: i8,
) {
    for (p1, p2, shape) in edges {
        new_ripup(grid, *p1, *p2, shape, edge_cost);
    }
}

/// Whether a one-bend route runs over a congested edge, undoing it if so.
///
/// ⛔ **Only the route's OWN two runs are examined**, not the whole bounding box: the vertical run
/// at the column the bend leaves from, and the horizontal run at the row it arrives on. Which
/// column and which row depends on the bend's direction.
///
/// ⛔ **This reads the ESTIMATE and compares against the raw capacity** — no lower-bound fraction,
/// no blockage term. Strictly greater, so an edge sitting exactly at capacity is not congested.
///
/// ⚠️ **The marks are given back asymmetrically.** The horizontal mark is removed only from a
/// **Steiner** node — a node whose index is at or past the terminal count — while the vertical
/// mark is removed unconditionally. A terminal therefore keeps its horizontal mark even after the
/// route that set it has gone.
#[allow(clippy::too_many_arguments)]
pub fn new_ripup_congested_l(
    grid: &mut EstimateGrid,
    statuses: &mut [i16],
    (n1, n2): (usize, usize),
    (x1, y1): (i32, i32),
    (x2, y2): (i32, i32),
    x_first: bool,
    num_terminals: usize,
    edge_cost: i8,
    capacity: &dyn Fn(i32, i32, bool) -> i32,
) -> bool {
    let (ymin, ymax) = (y1.min(y2), y1.max(y2));
    let x_check = if x_first { x2 } else { x1 };
    let y_check = if x_first { y1 } else { y2 };

    let mut need = false;
    for i in ymin..ymax {
        if grid.v(x_check as usize, i as usize) > f64::from(capacity(x_check, i, false)) {
            need = true;
            break;
        }
    }
    if !need {
        for i in x1..x2 {
            if grid.h(i as usize, y_check as usize) > f64::from(capacity(i, y_check, true)) {
                need = true;
                break;
            }
        }
    }

    if need {
        let cost = f64::from(edge_cost);
        if x_first {
            if n1 >= num_terminals {
                statuses[n1] -= 2;
            }
            statuses[n2] -= 1;
            give_back_h(grid, x1, x2, y1, cost, Layer::Estimate);
            give_back_v(grid, x2, ymin, ymax, cost, Layer::Estimate);
        } else {
            if n2 >= num_terminals {
                statuses[n2] -= 2;
            }
            statuses[n1] -= 1;
            give_back_v(grid, x1, ymin, ymax, cost, Layer::Estimate);
            give_back_h(grid, x1, x2, y2, cost, Layer::Estimate);
        }
    }
    need
}

/// Why a walked route was torn up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RipupReason {
    /// It runs over an edge at or past its threshold.
    Congested,
    /// It is a critical net whose route has grown too long.
    CriticalDetour,
}

/// Whether a walked route should be replaced, undoing it if so.
///
/// ⛔ **This reads COMMITTED demand, and the threshold is subtracted from the CAPACITY, not added
/// to the usage.** The comparison is `>=`, so an edge exactly at `capacity - threshold` already
/// counts as congested — one step earlier than a `>` would.
///
/// ⚠️ **The second reason has nothing to do with congestion.** A net whose slack is inside the
/// critical band, and whose route has grown to **twice** its previous length or more, is torn up
/// so it can be given a shorter path. It is only consulted when the congestion test has already
/// said no.
///
/// ⛔ **A sentinel slack disqualifies a net**: the reference guards with
/// `slack > ceil(lowest float)`, which excludes the value it uses to mean "no slack known". A net
/// carrying the sentinel would otherwise look like the most critical net in the design.
#[allow(clippy::too_many_arguments)]
pub fn new_ripup_check(
    grid: &mut EstimateGrid,
    grids: &[(i32, i32)],
    routelen: usize,
    ripup_threshold: i32,
    (h_capacity, v_capacity): (i32, i32),
    edge_cost: i8,
    critical: Option<CriticalCheck>,
    used_h: &dyn Fn(i32, i32) -> f64,
    used_v: &dyn Fn(i32, i32) -> f64,
) -> Option<RipupReason> {
    let mut reason = None;
    for i in 0..routelen {
        let (ax, ay) = grids[i];
        let (bx, by) = grids[i + 1];
        if ax == bx {
            if used_v(ax, ay.min(by)) >= f64::from(v_capacity - ripup_threshold) {
                reason = Some(RipupReason::Congested);
                break;
            }
        } else if ay == by && used_h(ax.min(bx), ay) >= f64::from(h_capacity - ripup_threshold) {
            reason = Some(RipupReason::Congested);
            break;
        }
    }

    if reason.is_none() {
        if let Some(c) = critical {
            if c.applies(routelen) {
                reason = Some(RipupReason::CriticalDetour);
            }
        }
    }

    if reason.is_some() {
        give_back_walked(grid, grids, routelen, f64::from(edge_cost), Layer::Committed);
    }
    reason
}

/// The critical-net arm of the maze gate.
#[derive(Debug, Clone, Copy)]
pub struct CriticalCheck {
    /// Whether the run enables critical-net handling at all.
    pub enabled: bool,
    /// The route length recorded before this round; zero disables the check.
    pub last_routelen: usize,
    /// The slack threshold for this round; zero disables the check.
    pub critical_slack: f32,
    pub slack: f32,
}

impl CriticalCheck {
    /// ⚠️ Four conditions, and three of them are "is this even configured". Only the last two are
    /// about the net: its slack inside the band, and its route at least **twice** its previous
    /// length.
    fn applies(&self, routelen: usize) -> bool {
        if !self.enabled || self.last_routelen == 0 || self.critical_slack == 0.0 {
            return false;
        }
        let delta = routelen as f32 / self.last_routelen as f32;
        self.slack <= self.critical_slack && self.slack > f32::MIN.ceil() && delta >= 2.0
    }
}
