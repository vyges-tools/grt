// SPDX-License-Identifier: Apache-2.0
//! R19 — the three-dimensional checks and the via bookkeeping.
//!
//! Four self-contained passes that run once the routing is final: count the vias, total the
//! three-dimensional overflow, verify each route is a connected path with valid layers, and make
//! sure every pin is reachable from its net's routing.
//!
//! ⛔ **Three of the four gate edges differently, and none is written in terms of the others.**
//! The via count and the checker both look at an edge's length, but one asks `len > 0` and the
//! other `len == 0`; the pin-coverage pass asks `len > 0 || routelen > 0`. On an edge with a
//! positive length and no steps, or a zero length and real steps, they disagree.

use crate::full3d::Point3D;

/// One edge of a routed net, as these passes read it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RoutedEdge {
    pub len: i32,
    pub routelen: i32,
    /// Index of the edge's first and second node, for the checker.
    pub n1: usize,
    pub n2: usize,
    pub grids: Vec<Point3D>,
}

/// A node of the net's tree.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoutedNode {
    pub x: i16,
    pub y: i16,
    pub bot_layer: i16,
    pub top_layer: i16,
    /// The layer of the pin this node carries, or `None` for a Steiner node.
    pub pin_layer: Option<i16>,
}

// ─── The via count ──────────────────────────────────────────────────────────────────────────

/// Count every step that changes layer.
///
/// ⛔ Only edges with a **positive length** are counted — an edge with real steps but no length
/// contributes nothing, which is the same gate `ConvertToFull3DType2` uses and the opposite of
/// the checker's.
pub fn three_d_via(edges: &[RoutedEdge]) -> i32 {
    let mut vias = 0;
    for edge in edges {
        if edge.len <= 0 {
            continue;
        }
        for j in 0..edge.routelen.max(0) as usize {
            if edge.grids[j].layer != edge.grids[j + 1].layer {
                vias += 1;
            }
        }
    }
    vias
}

// ─── The three-dimensional overflow ─────────────────────────────────────────────────────────

/// One grid edge's usage against its capacity, on one layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cell3D {
    /// `true` for a horizontal edge.
    pub horizontal: bool,
    pub usage: i32,
    pub capacity: i32,
}

/// What the overflow pass produces.
///
/// ⚠️ **The reference returns the total USAGE and sets the total overflow as a side effect.** Two
/// different numbers, and reading the returned one as congestion is the obvious mistake.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Overflow3D {
    pub horizontal: i32,
    pub vertical: i32,
    pub max_horizontal: i32,
    pub max_vertical: i32,
    /// Horizontal plus vertical. This is what the reference stores.
    pub total: i32,
    /// Every cell's usage, overflowing or not. This is what the reference **returns**.
    pub total_usage: i32,
}

/// Total the three-dimensional overflow.
///
/// ⛔ **Usage is accumulated for every cell, overflow only for the cells that exceed capacity.**
/// The running total therefore counts cells the overflow figures ignore.
pub fn get_overflow_3d(cells: &[Cell3D]) -> Overflow3D {
    let mut out = Overflow3D::default();
    for cell in cells {
        out.total_usage += cell.usage;
        let overflow = cell.usage - cell.capacity;
        // ⚠️ Strictly positive: a cell exactly at capacity is not overflowing.
        if overflow > 0 {
            if cell.horizontal {
                out.horizontal += overflow;
                out.max_horizontal = out.max_horizontal.max(overflow);
            } else {
                out.vertical += overflow;
                out.max_vertical = out.max_vertical.max(overflow);
            }
        }
    }
    out.total = out.horizontal + out.vertical;
    out
}

// ─── The route checker ──────────────────────────────────────────────────────────────────────

/// Something the checker found wrong with a net's routing.
///
/// ⚠️ Returned rather than logged. The reference raises two of these as errors and prints the
/// other three only under a debug flag, which means three real defects are invisible by default —
/// so a checker that reports them all is strictly more useful and no less faithful.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteDefect {
    /// A terminal whose node layer range excludes its pin's layer. ⛔ An error upstream.
    FloatingPin { node: usize },
    /// A grid point on a negative layer. ⛔ An error upstream.
    NegativeLayer { edge: usize, point: usize, layer: i16 },
    /// The route does not start at its first node's position.
    StartsElsewhere { edge: usize },
    /// The route does not end at its second node's position.
    EndsElsewhere { edge: usize },
    /// Two consecutive points are not adjacent — the route is not a connected path.
    NotAPath { edge: usize, step: usize, distance: i32 },
}

/// Check one net's routing.
///
/// ⛔ **The edge gate here is `len == 0`, not `len > 0`.** An edge with a *negative* length is
/// therefore checked, where the via count would skip it. Transcribed as written.
pub fn check_route_3d(nodes: &[RoutedNode], edges: &[RoutedEdge]) -> Vec<RouteDefect> {
    let mut out = Vec::new();

    // ⚠️ Only terminals carry a pin, and only terminals are checked for floating.
    for (node_id, node) in nodes.iter().enumerate() {
        if let Some(pin_layer) = node.pin_layer {
            if node.bot_layer > pin_layer || node.top_layer < pin_layer {
                out.push(RouteDefect::FloatingPin { node: node_id });
            }
        }
    }

    for (edge_id, edge) in edges.iter().enumerate() {
        if edge.len == 0 {
            continue;
        }
        let route_len = edge.routelen.max(0) as usize;
        let (n1, n2) = (nodes[edge.n1], nodes[edge.n2]);

        if edge.grids[0].x != n1.x || edge.grids[0].y != n1.y {
            out.push(RouteDefect::StartsElsewhere { edge: edge_id });
        }
        if edge.grids[route_len].x != n2.x || edge.grids[route_len].y != n2.y {
            out.push(RouteDefect::EndsElsewhere { edge: edge_id });
        }

        for i in 0..route_len {
            let (a, b) = (edge.grids[i], edge.grids[i + 1]);
            // ⛔ Position and LAYER together: one step may move in exactly one of the three.
            let distance = i32::from(b.x - a.x).abs()
                + i32::from(b.y - a.y).abs()
                + i32::from(b.layer - a.layer).abs();
            // ⚠️ The reference also tests `distance < 0`, which a sum of absolute values cannot
            // be. Transcribed as the single reachable half.
            if distance > 1 {
                out.push(RouteDefect::NotAPath { edge: edge_id, step: i, distance });
            }
        }

        // ⚠️ Inclusive of the last point, unlike the step loop above.
        for i in 0..=route_len {
            if edge.grids[i].layer < 0 {
                out.push(RouteDefect::NegativeLayer {
                    edge: edge_id,
                    point: i,
                    layer: edge.grids[i].layer,
                });
            }
        }
    }
    out
}

// ─── Pin coverage ───────────────────────────────────────────────────────────────────────────

/// A via stack the pass appends so a pin can be reached.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViaStackEdge {
    pub pin: usize,
    pub routelen: i32,
    pub grids: Vec<Point3D>,
}

/// Append a via stack for every pin the routing does not already reach.
///
/// ⛔ **The layer range starts INVERTED** — the bottom at the layer count and the top at `-1` — so
/// a pin no edge ever visits keeps that range and is judged un-covered. The stack it then gets
/// runs all the way to the layer count.
///
/// ⚠️ The range is keyed by **position**, so two pins sharing a position share a range.
pub fn ensure_pin_coverage(
    terminals: &[RoutedNode],
    edges: &[RoutedEdge],
    num_layers: i16,
) -> Vec<ViaStackEdge> {
    use std::collections::BTreeMap;

    let mut range: BTreeMap<(i16, i16), (i16, i16)> = BTreeMap::new();
    for t in terminals {
        range.insert((t.x, t.y), (num_layers, -1));
    }

    for edge in edges {
        // ⛔ A third gate: length OR steps, where the via count asks only about length.
        if !(edge.len > 0 || edge.routelen > 0) {
            continue;
        }
        for i in 0..=edge.routelen.max(0) as usize {
            let g = edge.grids[i];
            if let Some(r) = range.get_mut(&(g.x, g.y)) {
                r.0 = r.0.min(g.layer);
                r.1 = r.1.max(g.layer);
            }
        }
    }

    let mut out = Vec::new();
    for (pin, node) in terminals.iter().enumerate() {
        let (min_layer, max_layer) = range[&(node.x, node.y)];
        // ⚠️ Only the node's BOTTOM layer is tested. A top layer outside the range is not a
        // reason to add anything.
        if node.bot_layer < min_layer || node.bot_layer > max_layer {
            let (routelen, grids) = if node.bot_layer < min_layer {
                (
                    i32::from(min_layer - node.bot_layer),
                    (node.bot_layer..=min_layer)
                        .map(|l| Point3D { x: node.x, y: node.y, layer: l })
                        .collect(),
                )
            } else {
                (
                    i32::from(node.bot_layer - max_layer),
                    (max_layer..=node.bot_layer)
                        .map(|l| Point3D { x: node.x, y: node.y, layer: l })
                        .collect(),
                )
            };
            out.push(ViaStackEdge { pin, routelen, grids });
        }
    }
    out
}
