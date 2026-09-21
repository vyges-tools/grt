// SPDX-License-Identifier: Apache-2.0
//! R17 — demoting congested non-default-rule nets.
//!
//! A net with a non-default rule costs more per edge than a plain one. When two-dimensional and
//! three-dimensional congestion disagree, the router demotes the NDR nets that sit on a congested
//! edge to "soft NDR": their edge cost drops to one, and the usage they had charged is
//! re-accounted at the new cost.
//!
//! ⛔ **The gate above this never fires on any shipped design** — see the tests. A design with NDR
//! nets has no congestion, and a congested design has no NDR nets. The rules below are therefore
//! transcribed from the reference and pinned by constructed cases, not by a corpus.
//!
//! ⛔ **The re-accounting is a BRACKET, and the order inside it is the behaviour.** Usage is
//! removed at the old cost, the net is demoted, and usage is re-added at the new one. Both the
//! two- and three-dimensional updates do this, and both read the cost **again** after the
//! demotion — so the second read returns the new value, not the old.
//!
//! ⚠️ **"Not a wire" is a different test in each of the two updates.** The planar update skips a
//! step whose x and y both repeat — a via by position. The layered update skips a step whose
//! layer changes. Neither is written in terms of the other.

use crate::full3d::Point3D;

/// One edge of a net, reduced to what these passes read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NdrEdge {
    pub routelen: i32,
    pub grids: Vec<Point3D>,
}

/// One net, as the demotion sees it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NdrNet {
    pub net_id: usize,
    pub has_ndr: bool,
    pub is_soft_ndr: bool,
    /// The net's edge cost. ⚠️ An `int8_t` in the reference.
    pub edge_cost: i8,
    /// Per layer, or `None` when the net has no per-layer costs.
    pub layer_edge_cost: Option<Vec<i8>>,
    pub edges: Vec<NdrEdge>,
}

impl NdrNet {
    /// ⛔ Returns **1** once the net is soft — the per-layer table is bypassed rather than
    /// rewritten, so demoting a net changes every layer's cost at once.
    pub fn layer_edge_cost(&self, layer: i16) -> i8 {
        match (&self.layer_edge_cost, self.is_soft_ndr) {
            (Some(costs), false) => costs[layer as usize],
            _ => 1,
        }
    }
}

/// What the congestion tests read.
pub trait CongestionView {
    /// Two-dimensional overflow on the vertical edge at this position.
    fn overflow_v(&self, x: i16, y: i16) -> i32;
    fn overflow_h(&self, x: i16, y: i16) -> i32;
    /// Capacity less usage on the three-dimensional edge — negative means oversubscribed.
    fn available_v(&self, layer: i16, x: i16, y: i16) -> i32;
    fn available_h(&self, layer: i16, x: i16, y: i16) -> i32;
}

/// What the usage updates write.
pub trait UsageGrid {
    fn add_usage_v_2d(&mut self, x: i16, y: i16, delta: i32);
    fn add_usage_h_2d(&mut self, x: i16, y: i16, delta: i32);
    fn add_usage_v_3d(&mut self, layer: i16, x: i16, y: i16, delta: i32);
    fn add_usage_h_3d(&mut self, layer: i16, x: i16, y: i16, delta: i32);
}

/// Which NDR nets sit on a congested edge.
///
/// ⛔ **Both loops stop at the first congested step.** The edge loop and the step loop each carry
/// the flag in their condition, so a net is examined only until one congested step is found.
///
/// ⚠️ A step is congested when the two-dimensional edge overflows **or** the three-dimensional
/// one is oversubscribed — either alone is enough.
pub fn congested_ndr_nets(nets: &[NdrNet], view: &dyn CongestionView) -> Vec<usize> {
    let mut out = Vec::new();
    for net in nets {
        // ⛔ A net already demoted is skipped, so the pass cannot demote twice.
        if !net.has_ndr || net.is_soft_ndr {
            continue;
        }
        let mut is_congested = false;
        for edge in &net.edges {
            if is_congested {
                break;
            }
            if edge.routelen <= 0 || edge.grids.is_empty() {
                continue;
            }
            for i in 0..edge.routelen as usize {
                if is_congested {
                    break;
                }
                let (a, b) = (edge.grids[i], edge.grids[i + 1]);
                // ⛔ A layer change is a via, and vias are not tested.
                if a.layer != b.layer {
                    continue;
                }
                if a.x == b.x {
                    let min_y = a.y.min(b.y);
                    is_congested = view.overflow_v(a.x, min_y) > 0
                        || view.available_v(a.layer, a.x, min_y) < 0;
                } else {
                    // ⚠️ Anything not vertical is treated as horizontal here, with no test that
                    // the y values actually agree. The layered usage update below DOES test it.
                    let min_x = a.x.min(b.x);
                    is_congested = view.overflow_h(min_x, a.y) > 0
                        || view.available_h(a.layer, min_x, a.y) < 0;
                }
            }
        }
        if is_congested {
            out.push(net.net_id);
        }
    }
    out
}

/// Charge or refund a net's planar usage at the given cost.
///
/// ⛔ Skips a step that moves in neither x nor y — a via **by position**. A step that changes
/// layer while moving is still charged.
pub fn update_planar_net_usage(net: &NdrNet, edge_cost: i32, grid: &mut dyn UsageGrid) {
    for edge in &net.edges {
        if edge.routelen <= 0 || edge.grids.is_empty() {
            continue;
        }
        for i in 0..edge.routelen as usize {
            let (a, b) = (edge.grids[i], edge.grids[i + 1]);
            if a.x == b.x && a.y == b.y {
                continue;
            }
            if a.x == b.x {
                grid.add_usage_v_2d(a.x, a.y.min(b.y), edge_cost);
            } else if a.y == b.y {
                grid.add_usage_h_2d(a.x.min(b.x), a.y, edge_cost);
            }
        }
    }
}

/// Charge or refund a net's layered usage, scaled by that layer's cost.
///
/// ⛔ Skips a step whose **layer** changes — a different test from the planar update's. And the
/// per-step charge is `cost * layer_cost`, so demoting the net changes the multiplier as well as
/// letting the caller flip the sign.
pub fn update_net_3d_usage(net: &NdrNet, cost: i32, grid: &mut dyn UsageGrid) {
    for edge in &net.edges {
        if edge.routelen <= 0 || edge.grids.is_empty() {
            continue;
        }
        for i in 0..edge.routelen as usize {
            let (a, b) = (edge.grids[i], edge.grids[i + 1]);
            if a.layer != b.layer {
                continue;
            }
            let layer_cost = i32::from(net.layer_edge_cost(a.layer));
            if a.x == b.x {
                grid.add_usage_v_3d(a.layer, a.x, a.y.min(b.y), cost * layer_cost);
            } else if a.y == b.y {
                // ⚠️ Explicitly tested here, unlike the congestion scan above.
                grid.add_usage_h_3d(a.layer, a.x.min(b.x), a.y, cost * layer_cost);
            }
        }
    }
}

/// Demote one net: refund at the old cost, set soft, charge at the new one.
///
/// ⛔ **The cost is read twice, once on each side of the demotion.** The refund uses the net's
/// original edge cost and the charge uses the demoted one, which is why this cannot be written as
/// a single signed pass.
pub fn apply_soft_ndr(net: &mut NdrNet, grid: &mut dyn UsageGrid) {
    update_planar_net_usage(net, -i32::from(net.edge_cost), grid);
    set_soft_ndr(net);
    update_planar_net_usage(net, i32::from(net.edge_cost), grid);
}

/// Mark the net soft and drop its edge cost to one.
///
/// ⚠️ The per-layer costs are not rewritten; [`NdrNet::layer_edge_cost`] bypasses them once the
/// net is soft.
pub fn set_soft_ndr(net: &mut NdrNet) {
    net.is_soft_ndr = true;
    net.edge_cost = 1;
}

/// Demote every NDR net sitting on a congested edge.
///
/// `update_3d` is the reference's `is_incremental_grt_ || is_3d_step_`: when set, the layered
/// usage is re-accounted around the demotion as well as the planar usage.
///
/// ⛔ Returns the demoted ids. An empty result means the pass returned **before warning**.
pub fn disable_ndr_for_congested_nets(
    nets: &mut [NdrNet],
    view: &dyn CongestionView,
    grid: &mut dyn UsageGrid,
    update_3d: bool,
) -> Vec<usize> {
    let congested = congested_ndr_nets(nets, view);
    if congested.is_empty() {
        return congested;
    }
    for id in &congested {
        let Some(net) = nets.iter_mut().find(|n| n.net_id == *id) else {
            continue;
        };
        // ⛔ The layered bracket is OUTSIDE the planar one: remove at the old per-layer cost,
        // demote (which changes both costs), then re-add at the new one.
        if update_3d {
            update_net_3d_usage(net, -1, grid);
        }
        apply_soft_ndr(net, grid);
        if update_3d {
            update_net_3d_usage(net, 1, grid);
        }
    }
    congested
}
