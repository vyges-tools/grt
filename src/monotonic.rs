// SPDX-License-Identifier: Apache-2.0
//! Stage R12 — monotonic routing: replacing a congested edge with the cheapest pair of bends.
//!
//! An L route turns once and a Z route twice at a position chosen along one axis. This stage is
//! more general: it searches **every cell of an enlarged box** for a midpoint, and independently
//! picks an orientation for each of the two L-shaped halves. Four orientation pairs times every
//! cell in the box.
//!
//! The search is made affordable by two prefix-sum tables: the running cost along each row and
//! down each column, so any run's cost is one subtraction.
//!
//! ⛔ **The cost of an edge is a table lookup, not the usage itself.** The table is a logistic
//! curve in the usage, rebuilt whenever the routing loop changes its coefficient, and the lookup
//! **saturates** at the last entry rather than growing without bound.

use crate::estimate::EstimateGrid;

/// The reference's `BIG_INT`, used as the starting "no candidate yet" cost.
const BIG_INT: f64 = i32::MAX as f64;

/// Build the per-usage cost table — the reference's `h_cost_table_`.
///
/// ⚠️ **Used for vertical edges as well as horizontal ones**, despite the name: there is no
/// separate vertical table, so a column's capacity never enters the cost.
///
/// The curve is `costheight / (exp((h_capacity - i) * logis_cof) + 1) + 1`, which is near `1` for
/// usage well under capacity and near `costheight + 1` well over it.
pub fn monotonic_cost_table(costheight: f64, h_capacity: i32, logis_cof: f64) -> Vec<f64> {
    let n = (10 * h_capacity).max(0) as usize;
    (0..n)
        .map(|i| {
            let x = (f64::from(h_capacity) - i as f64) * logis_cof;
            costheight / (x.exp() + 1.0) + 1.0
        })
        .collect()
}

/// Look one edge's usage up in the table.
///
/// ⛔ **Saturating**: a usage past the end of the table takes the last entry. The table covers ten
/// times the capacity, so this only bites on a badly overloaded edge — but it is a clamp, not an
/// error.
fn cost_of(table: &[f64], usage: f64) -> f64 {
    let idx = (usage as usize).min(table.len() - 1);
    table[idx]
}

/// The search box for one edge.
///
/// ⚠️ **Asymmetric between the axes.** The x bounds are taken from the endpoints in the order
/// they are stored, so this assumes `x1 <= x2`; the y bounds are ordered explicitly first.
///
/// ⛔ **On a net with more than two terminals the box is pulled back to the edge's own span**
/// wherever another node of the same tree lies outside it — on that side only. A detour past a
/// sibling node would cross its territory.
pub fn monotonic_box(
    (x1, y1): (i32, i32),
    (x2, y2): (i32, i32),
    enlarge: i32,
    (x_grid, y_grid): (i32, i32),
    other_nodes: &[(i32, i32)],
    num_terminals: usize,
) -> (i32, i32, i32, i32) {
    let mut xmin = (x1 - enlarge).max(0);
    let mut xmax = (x_grid - 1).min(x2 + enlarge);
    let (yminorig, ymaxorig) = (y1.min(y2), y1.max(y2));
    let mut ymin = (yminorig - enlarge).max(0);
    let mut ymax = (y_grid - 1).min(ymaxorig + enlarge);

    if num_terminals > 2 {
        for &(nx, ny) in other_nodes {
            if nx < x1 {
                xmin = x1;
            }
            if nx > x2 {
                xmax = x2;
            }
            if ny < yminorig {
                ymin = yminorig;
            }
            if ny > ymaxorig {
                ymax = ymaxorig;
            }
        }
    }
    (xmin, xmax, ymin, ymax)
}

/// What the search and the walk leave behind.
#[derive(Debug, Clone, PartialEq)]
pub struct MonotonicRoute {
    pub px: i32,
    pub py: i32,
    /// Orientation of the half from the first endpoint to the midpoint: `true` horizontal first.
    pub bl1: bool,
    /// Orientation of the half from the midpoint to the second endpoint.
    pub bl2: bool,
    pub best: f64,
    pub points: Vec<(i32, i32)>,
    pub routelen: i32,
}

/// Search the box for the cheapest midpoint and orientation pair.
///
/// ⛔ **Ties go to the first candidate in iteration order — rows outward, columns within a row.**
/// The comparison is strict, and the four orientation pairs are tried in a fixed order, so an
/// exact tie between them keeps whichever was tried first.
///
/// ⚠️ The two pairs that turn the same way at both ends pay `via_cost`; the two that alternate do
/// not. **That penalty is dead in the shipped flow** — the router leaves the via cost at zero
/// until a later phase — so all four compete unpenalised. Transcribed, and asserted as zero.
#[allow(clippy::too_many_arguments)]
fn choose_midpoint(
    (x1, y1): (i32, i32),
    (x2, y2): (i32, i32),
    (xmin, xmax, ymin, ymax): (i32, i32, i32, i32),
    d1: &Grid2D,
    d2: &Grid2D,
    via_cost: f64,
) -> (i32, i32, bool, bool, f64) {
    let (mut best, mut px, mut py, mut bl1, mut bl2) = (BIG_INT, 0, 0, false, false);

    for j in ymin..=ymax {
        for i in xmin..=xmax {
            // Reaching the midpoint's row first, then its column; or the reverse.
            let tmp1 = (d2.at(x1, j) - d2.at(x1, y1)).abs() + (d1.at(i, j) - d1.at(x1, j)).abs();
            let tmp2 = (d2.at(i, j) - d2.at(i, y1)).abs() + (d1.at(i, y1) - d1.at(x1, y1)).abs();
            // Leaving the midpoint for the far endpoint, the same two ways.
            let tmp3 = (d2.at(i, y2) - d2.at(i, j)).abs() + (d1.at(i, y2) - d1.at(x2, y2)).abs();
            let tmp4 = (d2.at(x2, y2) - d2.at(x2, j)).abs() + (d1.at(x2, j) - d1.at(i, j)).abs();

            let (mut tmp, mut lh1, mut lh2) = (tmp1 + tmp4, false, true);
            if tmp2 + tmp3 < tmp {
                tmp = tmp2 + tmp3;
                lh1 = true;
                lh2 = false;
            }
            if tmp1 + tmp3 + via_cost < tmp {
                tmp = tmp1 + tmp3 + via_cost;
                lh1 = false;
                lh2 = false;
            }
            if tmp2 + tmp4 + via_cost < tmp {
                tmp = tmp2 + tmp4 + via_cost;
                lh1 = true;
                lh2 = true;
            }
            if tmp < best {
                best = tmp;
                px = i;
                py = j;
                bl1 = lh1;
                bl2 = lh2;
            }
        }
    }
    (px, py, bl1, bl2, best)
}

/// A prefix-sum table over the search box.
///
/// ⚠️ The reference allocates these once for the whole grid and reuses them across every edge,
/// writing and reading only the box. Holding just the box is equivalent — every read below is
/// inside it — and makes the bounds explicit.
struct Grid2D {
    xmin: i32,
    ymin: i32,
    width: usize,
    cells: Vec<f64>,
}

impl Grid2D {
    fn new(xmin: i32, xmax: i32, ymin: i32, ymax: i32) -> Self {
        let width = (xmax - xmin + 1) as usize;
        let height = (ymax - ymin + 1) as usize;
        Grid2D { xmin, ymin, width, cells: vec![0.0; width * height] }
    }
    fn at(&self, x: i32, y: i32) -> f64 {
        self.cells[(y - self.ymin) as usize * self.width + (x - self.xmin) as usize]
    }
    fn set(&mut self, x: i32, y: i32, v: f64) {
        let i = (y - self.ymin) as usize * self.width + (x - self.xmin) as usize;
        self.cells[i] = v;
    }
}

/// Walk the chosen route, emitting a point per step and charging the demand.
///
/// ⛔ **An edge is charged at its lower endpoint, so walking backwards charges `i - 1`, not `i`.**
/// The cell a step leaves and the edge that step crosses are not the same index when the step is
/// negative, and conflating them charges the wrong edge on every leftward or downward run.
///
/// ⚠️ Each run stops **before** its destination; the destination is emitted by the run that
/// follows, and the final endpoint is appended once at the end. So no corner is emitted twice.
fn walk(
    grid: &mut EstimateGrid,
    points: &mut Vec<(i32, i32)>,
    (x1, y1): (i32, i32),
    (px, py): (i32, i32),
    (x2, y2): (i32, i32),
    bl1: bool,
    bl2: bool,
    edge_cost: f64,
) {
    fn walk_h(
        grid: &mut EstimateGrid,
        points: &mut Vec<(i32, i32)>,
        from_x: i32,
        to_x: i32,
        y: i32,
        cost: f64,
    ) {
        let step = if to_x >= from_x { 1 } else { -1 };
        let mut i = from_x;
        while i != to_x {
            points.push((i, y));
            let edge = if step > 0 { i } else { i - 1 };
            grid.update_usage_h(edge, y, cost);
            i += step;
        }
    }
    fn walk_v(
        grid: &mut EstimateGrid,
        points: &mut Vec<(i32, i32)>,
        x: i32,
        from_y: i32,
        to_y: i32,
        cost: f64,
    ) {
        let step = if to_y >= from_y { 1 } else { -1 };
        let mut i = from_y;
        while i != to_y {
            points.push((x, i));
            let edge = if step > 0 { i } else { i - 1 };
            grid.update_usage_v(x, edge, cost);
            i += step;
        }
    }
    let segment = |grid: &mut EstimateGrid,
                       points: &mut Vec<(i32, i32)>,
                       h_first: bool,
                       (fx, fy): (i32, i32),
                       (tx, ty): (i32, i32)| {
        if h_first {
            walk_h(grid, points, fx, tx, fy, edge_cost);
            walk_v(grid, points, tx, fy, ty, edge_cost);
        } else {
            walk_v(grid, points, fx, fy, ty, edge_cost);
            walk_h(grid, points, fx, tx, ty, edge_cost);
        }
    };

    segment(grid, points, bl1, (x1, y1), (px, py));
    segment(grid, points, bl2, (px, py), (x2, y2));
    points.push((x2, y2));
}

/// Re-route one edge monotonically: search the box, then walk the winner.
#[allow(clippy::too_many_arguments)]
pub fn route_monotonic(
    grid: &mut EstimateGrid,
    (x1, y1): (i32, i32),
    (x2, y2): (i32, i32),
    (xmin, xmax, ymin, ymax): (i32, i32, i32, i32),
    cost_table: &[f64],
    via_cost: f64,
    edge_cost: i8,
    used_h: &dyn Fn(i32, i32) -> f64,
    used_v: &dyn Fn(i32, i32) -> f64,
) -> MonotonicRoute {
    let mut d1 = Grid2D::new(xmin, xmax, ymin, ymax);
    let mut d2 = Grid2D::new(xmin, xmax, ymin, ymax);

    // Running cost rightwards along each row, and upwards along each column. Both start at zero
    // on the box's near edge, which the allocation already gives.
    for j in ymin..=ymax {
        for i in xmin..xmax {
            d1.set(i + 1, j, d1.at(i, j) + cost_of(cost_table, used_h(i, j)));
        }
    }
    for j in ymin..ymax {
        for i in xmin..=xmax {
            d2.set(i, j + 1, d2.at(i, j) + cost_of(cost_table, used_v(i, j)));
        }
    }

    let (px, py, bl1, bl2, best) =
        choose_midpoint((x1, y1), (x2, y2), (xmin, xmax, ymin, ymax), &d1, &d2, via_cost);

    let mut points = Vec::new();
    walk(grid, &mut points, (x1, y1), (px, py), (x2, y2), bl1, bl2, f64::from(edge_cost));
    let routelen = points.len() as i32 - 1;

    MonotonicRoute { px, py, bl1, bl2, best, points, routelen }
}

/// Walk a route whose midpoint and orientations are already known.
///
/// Exposed so the walk can be replayed on every routed edge, where carrying the whole search box
/// would not fit. It is the same code the full routine runs.
#[allow(clippy::too_many_arguments)]
pub fn walk_monotonic_route(
    grid: &mut EstimateGrid,
    (x1, y1): (i32, i32),
    (px, py): (i32, i32),
    (x2, y2): (i32, i32),
    bl1: bool,
    bl2: bool,
    edge_cost: i8,
) -> (Vec<(i32, i32)>, i32) {
    let mut points = Vec::new();
    walk(grid, &mut points, (x1, y1), (px, py), (x2, y2), bl1, bl2, f64::from(edge_cost));
    let routelen = points.len() as i32 - 1;
    (points, routelen)
}
