// SPDX-License-Identifier: Apache-2.0
//! Stage R8 — re-routing each tree edge as an L, biased away from vias.
//!
//! ⚠️ **This works on TREE EDGES, not on the segment list.** The earlier estimate pass consumed
//! segments; from here on the router walks each net's Steiner tree directly.
//!
//! The decision is the same congestion comparison as the first L-routing pass, plus a **via
//! bias**: a node already connected horizontally makes a vertical departure cost a via, and vice
//! versa. That bias is what stops the router zig-zagging through turns it has already committed.

use crate::estimate::{congestion_cost, EstimateGrid, LShape};

/// A node of a net's Steiner tree, with the connection state the via bias reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TreeNode {
    pub x: i32,
    pub y: i32,
    /// A two-bit set held as a small integer: bit 0 = connected vertically, bit 1 = connected
    /// horizontally. 0 is unconnected, 3 is both.
    ///
    /// ⚠️ **The range is an empirical fact, not an enforced invariant.** Measured over a whole
    /// design: `n1` takes 0, 2, 3 and `n2` takes 0, 1, 2, 3 — never more. The reference does not
    /// trust it either, warning (GRT-179) on anything outside `0..=3` before carrying on.
    pub status: i32,
}

/// An edge of a net's Steiner tree.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TreeEdge {
    pub n1: usize,
    pub n2: usize,
    /// Manhattan length. ⚠️ A zero-length edge is not routed at all.
    pub len: i32,
}

/// Mark a node as connected VERTICALLY.
///
/// ⚠️ Arithmetic, not a bit operation: `if status % 2 == 0 { status += 1 }`. For bit 0 the parity
/// test *is* the bit test, so this coincides with `status |= 1` at every value.
pub fn mark_v(node: &mut TreeNode) {
    if node.status % 2 == 0 {
        node.status += 1;
    }
}

/// Mark a node as connected HORIZONTALLY.
///
/// ⚠️ **The guard is `status < 2`, which is only `status |= 2` while status stays under 4.** At 4,
/// 5, 8 or 12 the guard is false and nothing is set, where a bit-set would give 6, 7, 10 or 14.
/// Measured over a whole design, no status ever exceeds 3, so the two agree everywhere observed —
/// but the arithmetic is transcribed rather than "simplified", because it is what the reference
/// does and the range is not enforced.
pub fn mark_h(node: &mut TreeNode) {
    if node.status < 2 {
        node.status += 2;
    }
}

/// The via penalty each candidate shape earns from the two endpoints' existing connections.
///
/// ⛔ **The mapping is CROSSED between the two nodes, and getting it the same way round for both
/// is the obvious mistake.** L1 departs `n1` vertically and arrives at `n2` horizontally; L2 does
/// the opposite. So a node already connected *horizontally* penalises whichever shape leaves it
/// *vertically* — which is L1 at `n1` but L2 at `n2`.
///
/// ⚠️ **Status 0 and 3 cost nothing**: nothing is connected yet, or both directions already are,
/// so a via is needed either way and the bias cannot discriminate.
///
/// ⚠️ The reference assigns for `n1` (`costL1 = via_cost`) and accumulates for `n2` (`costL2 +=`).
/// That is equivalent only because both costs are zero when the block runs, and it is transcribed
/// as accumulation here rather than silently "corrected".
pub fn via_bias(n1_status: i32, n2_status: i32, via_cost: f64) -> (f64, f64) {
    let (mut l1, mut l2) = (0.0, 0.0);
    match n1_status {
        2 => l1 += via_cost,
        1 => l2 += via_cost,
        _ => {}
    }
    match n2_status {
        2 => l2 += via_cost,
        1 => l1 += via_cost,
        _ => {}
    }
    (l1, l2)
}

/// Whether a node's status is one the via bias understands.
///
/// ⚠️ The reference warns (GRT-179) on anything outside `0..=3` and then carries on. Reported
/// here rather than warned, because a caller that produced such a status has a bug worth seeing.
pub fn is_known_status(status: i32) -> bool {
    (0..=3).contains(&status)
}

/// How one tree edge was routed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeRoute {
    /// The edge has zero length and is not routed.
    None,
    /// Straight down a column.
    Vertical,
    /// Straight along a row.
    Horizontal,
    /// Bent, one way or the other.
    L(LShape),
}

/// Route one tree edge, updating the demand grid and both endpoints' connection state.
///
/// ⚠️ **The full edge cost is charged here, not half.** The first pass charged half to each
/// candidate because it had pre-loaded both; this pass rips up the previous route first, so the
/// chosen path takes the whole cost and the other takes none.
pub fn route_edge(
    grid: &mut EstimateGrid,
    nodes: &mut [TreeNode],
    edge: &TreeEdge,
    edge_cost: i8,
    via_cost: f64,
    via_guided: bool,
    v_lb: f32,
    h_lb: f32,
    red_v: &dyn Fn(usize, usize) -> u16,
    red_h: &dyn Fn(usize, usize) -> u16,
) -> EdgeRoute {
    if edge.len <= 0 {
        return EdgeRoute::None;
    }
    let (x1, y1) = (nodes[edge.n1].x, nodes[edge.n1].y);
    let (x2, y2) = (nodes[edge.n2].x, nodes[edge.n2].y);
    let (ymin, ymax) = (y1.min(y2), y1.max(y2));
    let cost = edge_cost as f64;

    if x1 == x2 {
        grid.update_v(x1, ymin, ymax, cost);
        mark_v(&mut nodes[edge.n1]);
        mark_v(&mut nodes[edge.n2]);
        return EdgeRoute::Vertical;
    }
    if y1 == y2 {
        grid.update_h(x1, x2, y1, cost);
        mark_h(&mut nodes[edge.n1]);
        mark_h(&mut nodes[edge.n2]);
        return EdgeRoute::Horizontal;
    }

    let (mut cost_l1, mut cost_l2) = if via_guided {
        via_bias(nodes[edge.n1].status, nodes[edge.n2].status, via_cost)
    } else {
        (0.0, 0.0)
    };

    for j in ymin..ymax {
        cost_l1 += congestion_cost(grid.v(x1 as usize, j as usize), red_v(x1 as usize, j as usize), v_lb);
        cost_l2 += congestion_cost(grid.v(x2 as usize, j as usize), red_v(x2 as usize, j as usize), v_lb);
    }
    for j in x1..x2 {
        cost_l1 += congestion_cost(grid.h(j as usize, y2 as usize), red_h(j as usize, y2 as usize), h_lb);
        cost_l2 += congestion_cost(grid.h(j as usize, y1 as usize), red_h(j as usize, y1 as usize), h_lb);
    }

    // ⛔ A tie goes to L2, as everywhere else in this router.
    if cost_l1 < cost_l2 {
        // ⚠️ The marks are crossed to match the shape: L1 leaves n1 vertically, reaches n2
        // horizontally.
        mark_v(&mut nodes[edge.n1]);
        mark_h(&mut nodes[edge.n2]);
        grid.update_v(x1, ymin, ymax, cost);
        grid.update_h(x1, x2, y2, cost);
        EdgeRoute::L(LShape::YFirst)
    } else {
        mark_v(&mut nodes[edge.n2]);
        mark_h(&mut nodes[edge.n1]);
        grid.update_h(x1, x2, y1, cost);
        grid.update_v(x2, ymin, ymax, cost);
        EdgeRoute::L(LShape::XFirst)
    }
}
