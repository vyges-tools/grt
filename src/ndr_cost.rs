// SPDX-License-Identifier: Apache-2.0
//! The NDR-aware usage cost — `Graph2D::getCostNDRAware` and the state it keeps.
//!
//! Every usage update in the router, estimated and committed alike, passes its amount through
//! this cost. For a net with edge cost 1 the amount passes unchanged. For an NDR net (edge cost
//! > 1) the amount is IGNORED except for its sign, and the edge remembers the net:
//!
//! | call | net already on the edge | net not on the edge |
//! | --- | --- | --- |
//! | add (`amount >= 0`) | 0 | `+edgeCost`, or `+100 × edgeCost` in NDR overflow |
//! | remove (`amount < 0`) | `-edgeCost`, or `-100 × edgeCost` while the edge is in overflow | 0 |
//!
//! ⚠️ **So a net is counted once per edge however many times it is added.** The first L-routing
//! pass adds half the cost twice; only the first half counts, as the full edge cost.
//!
//! ⛔ **Overflow is decided per call, by the edge's overflow count or the layers' NDR capacity.**
//! Each layer in the net's range keeps a `cap_ndr` that NDR nets draw down by their layer edge cost;
//! when no layer has room the edge enters overflow and every NDR net added to it costs 100×.
//!
//! Stages in the reference's order: [`NdrLedger::init_cap_3d`], [`NdrLedger::update_cap_3d`],
//! [`NdrLedger::has_ndr_capacity`], [`NdrLedger::get_cost_ndr_aware`],
//! [`NdrLedger::update_ndr_cap_layer`]; the lifecycle: [`NdrLedger::copy_routing_state_from`],
//! [`NdrLedger::clear_ndr_nets`].

use std::collections::BTreeSet;

/// The reference's `OVERFLOW_COST_MULTIPLIER`.
pub const OVERFLOW_COST_MULTIPLIER: f64 = 100.0;

/// One layer's capacity on one edge (`Graph2D::Cap3D`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct NdrCap {
    /// ⛔ `uint16_t` — the reference narrows the capacity it is given.
    pub cap: u16,
    /// The capacity left for NDR nets. `double`, and it can go negative (see
    /// [`NdrLedger::update_ndr_cap_layer`]).
    pub cap_ndr: f64,
}

/// What the cost reads from a net.
#[derive(Debug, Clone, PartialEq)]
pub struct NdrCostNet {
    /// The net's identity on an edge — the reference keys its sets by `FrNet*`.
    pub id: usize,
    pub edge_cost: i8,
    pub min_layer: usize,
    pub max_layer: usize,
    /// The per-layer edge cost for `min_layer..=max_layer`, or `None` when the net has no per-layer
    /// costs.
    pub layer_edge_cost: Option<Vec<i8>>,
    pub soft_ndr: bool,
}

impl NdrCostNet {
    /// `FrNet::getLayerEdgeCost` — the per-layer cost, or 1 for a net without one or a SOFT NDR.
    pub fn layer_edge_cost(&self, layer: usize) -> i8 {
        match &self.layer_edge_cost {
            Some(v) if !self.soft_ndr => v[layer - self.min_layer],
            _ => 1,
        }
    }
}

/// The NDR half of `Graph2D`: the per-layer capacities (`h_cap_3D_` / `v_cap_3D_`), the NDR nets
/// on each edge (`h_ndr_nets_` / `v_ndr_nets_`), and each edge's overflow count (`Edge::ndr_overflow`).
///
/// ⚠️ The reference indexes all of these `[x][y]` (caps `[layer][x][y]`); held flat here the same way.
#[derive(Debug, Clone, PartialEq)]
pub struct NdrLedger {
    pub x_grid: usize,
    pub y_grid: usize,
    pub num_layers: usize,
    h_cap: Vec<NdrCap>,
    v_cap: Vec<NdrCap>,
    h_nets: Vec<BTreeSet<usize>>,
    v_nets: Vec<BTreeSet<usize>>,
    /// ⛔ `uint16_t`, and sized like the EDGES — `(x-1) × y` and `x × (y-1)` — not the cells.
    h_overflow: Vec<u16>,
    v_overflow: Vec<u16>,
}

impl NdrLedger {
    /// `Graph2D::init` then `initCap3D`: every overflow count 0, every set empty, every capacity 0.
    pub fn new(x_grid: usize, y_grid: usize, num_layers: usize) -> Self {
        let cells = x_grid * y_grid;
        NdrLedger {
            x_grid,
            y_grid,
            num_layers,
            h_cap: vec![NdrCap::default(); num_layers * cells],
            v_cap: vec![NdrCap::default(); num_layers * cells],
            h_nets: vec![BTreeSet::new(); cells],
            v_nets: vec![BTreeSet::new(); cells],
            h_overflow: vec![0; x_grid.saturating_sub(1) * y_grid],
            v_overflow: vec![0; x_grid * y_grid.saturating_sub(1)],
        }
    }

    fn cell(&self, x: usize, y: usize) -> usize {
        x * self.y_grid + y
    }
    fn cap_at(&self, layer: usize, x: usize, y: usize) -> usize {
        layer * self.x_grid * self.y_grid + self.cell(x, y)
    }
    fn edge_at(&self, horizontal: bool, x: usize, y: usize) -> usize {
        if horizontal { x * self.y_grid + y } else { x * (self.y_grid - 1) + y }
    }

    /// One layer's capacity on an edge.
    pub fn cap(&self, horizontal: bool, layer: usize, x: usize, y: usize) -> NdrCap {
        let i = self.cap_at(layer, x, y);
        if horizontal { self.h_cap[i] } else { self.v_cap[i] }
    }
    /// Whether the net is recorded on the edge.
    pub fn has_net(&self, horizontal: bool, x: usize, y: usize, net: usize) -> bool {
        let i = self.cell(x, y);
        if horizontal { self.h_nets[i].contains(&net) } else { self.v_nets[i].contains(&net) }
    }
    /// The edge's NDR overflow count.
    pub fn overflow(&self, horizontal: bool, x: usize, y: usize) -> u16 {
        let i = self.edge_at(horizontal, x, y);
        if horizontal { self.h_overflow[i] } else { self.v_overflow[i] }
    }

    /// `initCap3D` — ⛔ a RESIZE, not a reset: boost's `multi_array::resize` keeps every element where
    /// the old and new extents overlap. On a grid of the same size the capacities and the NDR sets of
    /// the previous run survive; only [`update_cap_3d`](Self::update_cap_3d) then overwrites the
    /// capacities. (The overflow counts are reset by `Graph2D::init`, which runs first.)
    pub fn init_cap_3d(&mut self, x_grid: usize, y_grid: usize, num_layers: usize) {
        let mut next = NdrLedger::new(x_grid, y_grid, num_layers);
        for l in 0..num_layers.min(self.num_layers) {
            for x in 0..x_grid.min(self.x_grid) {
                for y in 0..y_grid.min(self.y_grid) {
                    let (a, b) = (next.cap_at(l, x, y), self.cap_at(l, x, y));
                    next.h_cap[a] = self.h_cap[b];
                    next.v_cap[a] = self.v_cap[b];
                }
            }
        }
        for x in 0..x_grid.min(self.x_grid) {
            for y in 0..y_grid.min(self.y_grid) {
                let (a, b) = (next.cell(x, y), self.cell(x, y));
                next.h_nets[a] = std::mem::take(&mut self.h_nets[b]);
                next.v_nets[a] = std::mem::take(&mut self.v_nets[b]);
            }
        }
        *self = next;
    }

    /// `updateCap3D` — set a layer's capacity and its NDR capacity to the same value.
    /// ⛔ The capacity arrives as `double` and is narrowed to `uint16_t`; `cap_ndr` keeps the double.
    pub fn update_cap_3d(&mut self, x: usize, y: usize, layer: usize, horizontal: bool, cap: f64) {
        let i = self.cap_at(layer, x, y);
        let c = if horizontal { &mut self.h_cap[i] } else { &mut self.v_cap[i] };
        c.cap = cap as u16;
        c.cap_ndr = cap;
    }

    /// `hasNDRCapacity` — whether ANY single layer in the net's range still has room for its layer
    /// edge cost. A net with edge cost 1 always has room.
    pub fn has_ndr_capacity(&self, net: &NdrCostNet, x: usize, y: usize, horizontal: bool) -> bool {
        if net.edge_cost == 1 {
            return true;
        }
        (net.min_layer..=net.max_layer).any(|l| self.cap(horizontal, l, x, y).cap_ndr >= net.layer_edge_cost(l) as f64)
    }

    /// `getCostNDRAware` — the usage an update actually charges, and the bookkeeping behind it.
    ///
    /// ⛔ `amount` is read for its SIGN only once the net is an NDR net, and `0` counts as an add.
    pub fn get_cost_ndr_aware(&mut self, net: &NdrCostNet, x: usize, y: usize, amount: f64, horizontal: bool) -> f64 {
        let ec = net.edge_cost;
        if ec == 1 {
            return amount;
        }
        let (cell, e) = (self.cell(x, y), self.edge_at(horizontal, x, y));
        let present = if horizontal { self.h_nets[cell].contains(&net.id) } else { self.v_nets[cell].contains(&net.id) };
        let mut cost = 0.0;
        if amount < 0.0 {
            if present {
                if horizontal { self.h_nets[cell].remove(&net.id) } else { self.v_nets[cell].remove(&net.id) };
                let ov = if horizontal { &mut self.h_overflow[e] } else { &mut self.v_overflow[e] };
                if *ov > 0 {
                    *ov -= 1;
                    cost = -OVERFLOW_COST_MULTIPLIER * ec as f64;
                } else {
                    cost = -(ec as f64);
                }
                self.update_ndr_cap_layer(x, y, net, horizontal, amount);
            }
        } else if !present {
            let ov = self.overflow(horizontal, x, y);
            if ov > 0 || !self.has_ndr_capacity(net, x, y, horizontal) {
                let ov = if horizontal { &mut self.h_overflow[e] } else { &mut self.v_overflow[e] };
                *ov = ov.wrapping_add(1);
                cost = OVERFLOW_COST_MULTIPLIER * ec as f64;
            } else {
                cost = ec as f64;
            }
            if horizontal { self.h_nets[cell].insert(net.id) } else { self.v_nets[cell].insert(net.id) };
            self.update_ndr_cap_layer(x, y, net, horizontal, amount);
        }
        cost
    }

    /// `updateNDRCapLayer` — draw the net's layer edge cost from the FIRST layer with room, or give it
    /// back to the first layer that can take it.
    ///
    /// ⛔ Giving back needs `cap - cap_ndr >= cost` (the layer must have lent at least that much).
    /// ⛔ **When no layer has room on an ADD, the lowest layer is drawn down anyway** — `cap_ndr`
    /// goes negative — so a later remove releases it first. A remove that finds no layer changes
    /// nothing, and neither does an amount of exactly 0.
    pub fn update_ndr_cap_layer(&mut self, x: usize, y: usize, net: &NdrCostNet, horizontal: bool, amount: f64) {
        if net.edge_cost == 1 {
            return;
        }
        for l in net.min_layer..=net.max_layer {
            let i = self.cap_at(l, x, y);
            let c = if horizontal { &mut self.h_cap[i] } else { &mut self.v_cap[i] };
            let lec = net.layer_edge_cost(l) as f64;
            if amount < 0.0 {
                if c.cap as f64 - c.cap_ndr >= lec {
                    c.cap_ndr += lec;
                    return;
                }
            } else if c.cap_ndr >= lec {
                c.cap_ndr -= lec;
                return;
            }
        }
        if amount > 0.0 {
            let i = self.cap_at(net.min_layer, x, y);
            let c = if horizontal { &mut self.h_cap[i] } else { &mut self.v_cap[i] };
            c.cap_ndr -= net.layer_edge_cost(net.min_layer) as f64;
        }
    }

    /// `copyRoutingStateFrom` — take another graph's capacities and overflow counts (they travel
    /// with the edges), and its NDR sets only when asked; otherwise the sets are CLEARED.
    pub fn copy_routing_state_from(&mut self, other: &NdrLedger, include_ndr_state: bool) {
        *self = other.clone();
        if !include_ndr_state {
            self.clear_ndr_nets();
        }
    }

    /// `clearNDRnets` — empty every edge's NDR set. Capacities and overflow counts are left alone.
    pub fn clear_ndr_nets(&mut self) {
        self.h_nets.iter_mut().chain(self.v_nets.iter_mut()).for_each(BTreeSet::clear);
    }
}
