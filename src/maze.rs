// SPDX-License-Identifier: Apache-2.0
//! The maze router — the stage that re-routes congested edges by shortest path.
//!
//! This module grows piece by piece alongside the reference's `maze.cpp`. The edge-cost tables
//! it prices with live in [`crate::mazecost`], built once per congestion iteration.

/// One of a net's edges, paired with the length that decides when it is routed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OrderNetEdge {
    /// ⚠️ The **routed** length — how many steps the current path takes — not the Manhattan
    /// distance between the endpoints. A detour therefore raises an edge's priority here.
    pub length: i32,
    pub edge_id: usize,
}

/// Order one net's edges **longest first** — the reference's `netedgeOrderDec`.
///
/// ⛔ **Stable, and the stability is the specification, not an implementation detail.** Edges of
/// equal routed length keep their index order, and equal lengths are common: a net whose edges
/// are all freshly routed straight lines has many. Sorting unstably would reorder them by
/// whatever the algorithm happened to do.
///
/// ⚠️ **Descending.** The longest edge is routed first, when the grid is least crowded by this
/// net's own new demand.
pub fn netedge_order_dec(routelens: &[i32]) -> Vec<OrderNetEdge> {
    let mut out: Vec<OrderNetEdge> = routelens
        .iter()
        .enumerate()
        .map(|(edge_id, &length)| OrderNetEdge { length, edge_id })
        .collect();
    out.sort_by(|a, b| b.length.cmp(&a.length));
    out
}
