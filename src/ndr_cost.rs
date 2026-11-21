// SPDX-License-Identifier: Apache-2.0
//! The NDR-aware usage cost — `Graph2D::getCostNDRAware` and the state it keeps.
//!
//! Every usage update in the router, estimated and committed alike, passes its amount through
//! this cost. For a net with edge cost 1 the amount passes unchanged. For an NDR net (edge cost
//! > 1) the amount is IGNORED except for its sign, and the edge remembers the net:
//!
//! | call | net already on the edge | net not on the edge |
//! | --- | --- | --- |
//! | add (`amount >= 0`) | 0 | `+edgeCost` |
//! | remove (`amount < 0`) | `-edgeCost` | 0 |
//!
//! ⚠️ **So a net is counted once per edge however many times it is added.** The first L-routing
//! pass adds half the cost twice; only the first half counts, as the full edge cost.
//!
//! 🔑 **NDR deficits (OpenROAD #11963 / #12148, f11b4276 pin) replace the 100× overflow charge.**
//! Each layer in a net's range has an NDR capacity — its capacity less what NDR nets reserved
//! there (`ndr_debit_`). An NDR net reserves its layer edge cost on the FIRST layer with room; where
//! none has room it reserves on its lowest layer anyway and records the tracks it misses on its best
//! layer as its DEFICIT on the edge. The shared usage always gets the true edge cost; the deficits
//! are kept per edge and in a running total, which only NDR nets price
//! ([`NdrLedger::ndr_deficit_if_routed`]) and which the 2D overflow counts. When a net leaves an edge
//! that still has a deficit, the remaining deficit nets are re-reserved in database-id order
//! ([`NdrLedger::refit_ndr_deficit_nets`]).
//!
//! Stages in the reference's order: [`NdrLedger::init_cap_3d`], [`NdrLedger::update_cap_3d`],
//! [`NdrLedger::compute_ndr_deficit`], [`NdrLedger::get_cost_ndr_aware`],
//! [`NdrLedger::reserve_ndr_cap_layer`], [`NdrLedger::release_ndr_cap_layer`],
//! [`NdrLedger::refit_ndr_deficit_nets`]; the lifecycle: [`NdrLedger::copy_routing_state_from`],
//! [`NdrLedger::clear_ndr_nets`].

use std::collections::BTreeMap;

/// One layer's capacity on one edge: `h_cap_3D_`/`v_cap_3D_` (`uint16_t`) and what NDR nets
/// reserved there (`ndr_debit_`, an `int`; absent = 0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NdrCap {
    pub cap: u16,
    pub debit: i32,
}

impl NdrCap {
    /// `getCapNDR`: the capacity less the debit — negative when NDR nets oversubscribe the layer.
    pub fn cap_ndr(&self) -> i32 {
        i32::from(self.cap) - self.debit
    }
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

/// `Graph2D::NDRUsage` — what one NDR net took on one edge: the layer whose NDR capacity was
/// debited (-1 none), by how much, and the tracks it misses on its best layer (0 = it fits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NdrUsage {
    pub layer: i16,
    pub amount: i8,
    pub deficit: i8,
}

/// The NDR half of `Graph2D`: the per-layer capacities and their NDR debits, the NDR nets on each
/// edge (`ndr_nets_`), each edge's NDR deficit (`Edge::ndr_deficit`) and their total.
///
/// ⚠️ The reference indexes the capacities `[layer][x][y]` and keys the sparse maps by edge; held
/// dense here, which no reader can tell apart (the maps are only ever looked up, never walked).
/// Each record keeps the net it belongs to, so a refit can reserve for it again.
#[derive(Debug, Clone, PartialEq)]
pub struct NdrLedger {
    pub x_grid: usize,
    pub y_grid: usize,
    pub num_layers: usize,
    h_cap: Vec<NdrCap>,
    v_cap: Vec<NdrCap>,
    h_nets: Vec<BTreeMap<usize, (NdrUsage, NdrCostNet)>>,
    v_nets: Vec<BTreeMap<usize, (NdrUsage, NdrCostNet)>>,
    /// ⛔ `uint16_t`, and sized like the EDGES — `(x-1) × y` and `x × (y-1)` — not the cells.
    h_deficit: Vec<u16>,
    v_deficit: Vec<u16>,
    /// `ndr_deficit_total_`.
    deficit_total: i32,
    /// Each net's database id by net id (`getDbNet()->getId()`), the refit's order; a net id past
    /// the end orders by itself.
    db_ids: Vec<u32>,
}

impl NdrLedger {
    /// `Graph2D::init` then `initCap3D`: no record, no deficit, every capacity 0.
    pub fn new(x_grid: usize, y_grid: usize, num_layers: usize) -> Self {
        let cells = x_grid * y_grid;
        NdrLedger {
            x_grid,
            y_grid,
            num_layers,
            h_cap: vec![NdrCap::default(); num_layers * cells],
            v_cap: vec![NdrCap::default(); num_layers * cells],
            h_nets: vec![BTreeMap::new(); cells],
            v_nets: vec![BTreeMap::new(); cells],
            h_deficit: vec![0; x_grid.saturating_sub(1) * y_grid],
            v_deficit: vec![0; x_grid * y_grid.saturating_sub(1)],
            deficit_total: 0,
            db_ids: Vec::new(),
        }
    }

    /// The database id of each net, by net id: the order [`refit_ndr_deficit_nets`] re-reserves in.
    ///
    /// [`refit_ndr_deficit_nets`]: Self::refit_ndr_deficit_nets
    pub fn set_db_ids(&mut self, db_ids: Vec<u32>) {
        self.db_ids = db_ids;
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
    fn nets(&self, horizontal: bool, x: usize, y: usize) -> &BTreeMap<usize, (NdrUsage, NdrCostNet)> {
        let i = self.cell(x, y);
        if horizontal { &self.h_nets[i] } else { &self.v_nets[i] }
    }
    fn nets_mut(&mut self, horizontal: bool, x: usize, y: usize) -> &mut BTreeMap<usize, (NdrUsage, NdrCostNet)> {
        let i = self.cell(x, y);
        if horizontal { &mut self.h_nets[i] } else { &mut self.v_nets[i] }
    }
    fn cap_mut(&mut self, horizontal: bool, layer: usize, x: usize, y: usize) -> &mut NdrCap {
        let i = self.cap_at(layer, x, y);
        if horizontal { &mut self.h_cap[i] } else { &mut self.v_cap[i] }
    }
    fn deficit_mut(&mut self, horizontal: bool, x: usize, y: usize) -> &mut u16 {
        let e = self.edge_at(horizontal, x, y);
        if horizontal { &mut self.h_deficit[e] } else { &mut self.v_deficit[e] }
    }

    /// One layer's capacity on an edge.
    pub fn cap(&self, horizontal: bool, layer: usize, x: usize, y: usize) -> NdrCap {
        let i = self.cap_at(layer, x, y);
        if horizontal { self.h_cap[i] } else { self.v_cap[i] }
    }
    /// Whether the net is recorded on the edge.
    pub fn has_net(&self, horizontal: bool, x: usize, y: usize, net: usize) -> bool {
        self.nets(horizontal, x, y).contains_key(&net)
    }
    /// The record a net left on an edge.
    pub fn usage(&self, horizontal: bool, x: usize, y: usize, net: usize) -> Option<NdrUsage> {
        self.nets(horizontal, x, y).get(&net).map(|(u, _)| *u)
    }
    /// `getEdgeNDRDeficit`: the deficit of all the NDR nets using the edge.
    pub fn edge_ndr_deficit(&self, horizontal: bool, x: usize, y: usize) -> i32 {
        let e = self.edge_at(horizontal, x, y);
        i32::from(if horizontal { self.h_deficit[e] } else { self.v_deficit[e] })
    }
    /// `hasNDRDeficit`: an NDR net using the edge fits no single layer there.
    pub fn has_ndr_deficit(&self, horizontal: bool, x: usize, y: usize) -> bool {
        self.edge_ndr_deficit(horizontal, x, y) > 0
    }
    /// `getTotalNDRDeficit` — the running total.
    pub fn total_ndr_deficit(&self) -> i32 {
        self.deficit_total
    }

    /// `initCap3D`: the capacities RESIZED (boost's `resize` keeps the elements where the old and
    /// new extents overlap), then `initNDRnets` — every record, deficit and debit dropped.
    ///
    /// Upstream rule (#11963): the records of a previous run must go — its nets are deleted by
    /// then, and a new net at the same address would be taken for one already on the edge.
    pub fn init_cap_3d(&mut self, x_grid: usize, y_grid: usize, num_layers: usize) {
        let mut next = NdrLedger::new(x_grid, y_grid, num_layers);
        for l in 0..num_layers.min(self.num_layers) {
            for x in 0..x_grid.min(self.x_grid) {
                for y in 0..y_grid.min(self.y_grid) {
                    let (a, b) = (next.cap_at(l, x, y), self.cap_at(l, x, y));
                    next.h_cap[a].cap = self.h_cap[b].cap;
                    next.v_cap[a].cap = self.v_cap[b].cap;
                }
            }
        }
        next.db_ids = std::mem::take(&mut self.db_ids);
        *self = next;
    }

    /// `updateCap3D` — set a layer's capacity (the double narrowed to `uint16_t`) and drop its
    /// NDR debit.
    pub fn update_cap_3d(&mut self, x: usize, y: usize, layer: usize, horizontal: bool, cap: f64) {
        let c = self.cap_mut(horizontal, layer, x, y);
        c.cap = cap as u16;
        c.debit = 0;
    }

    /// `addCap3D` — add to a layer's capacity; what NDR nets reserved stays reserved (the debit is
    /// unchanged). ⛔ `cap` is `uint16_t`: `cap += delta` wraps as the reference's does.
    pub fn add_cap_3d(&mut self, x: usize, y: usize, layer: usize, horizontal: bool, delta: i32) {
        let c = self.cap_mut(horizontal, layer, x, y);
        c.cap = (i32::from(c.cap) + delta) as u16;
    }

    /// `computeNDRDeficit` — the tracks the net would miss on its best layer if it used the edge
    /// now: per layer in range `max(0, layer edge cost − max(0, NDR capacity))`, the minimum,
    /// stopping at the first 0; with no layer in range, the net's whole edge cost.
    pub fn compute_ndr_deficit(&self, net: &NdrCostNet, x: usize, y: usize, horizontal: bool) -> i32 {
        let mut deficit = i32::MAX;
        let mut l = net.min_layer;
        while l <= net.max_layer && deficit > 0 {
            let free = self.cap(horizontal, l, x, y).cap_ndr().max(0);
            deficit = deficit.min((i32::from(net.layer_edge_cost(l)) - free).max(0));
            l += 1;
        }
        if deficit == i32::MAX { i32::from(net.edge_cost) } else { deficit }
    }

    /// `getNDRDeficit` — the net's own deficit on the edge; 0 for an edge-cost-1 net or one not on
    /// the edge.
    pub fn ndr_deficit(&self, net: &NdrCostNet, x: usize, y: usize, horizontal: bool) -> i32 {
        if net.edge_cost == 1 {
            return 0;
        }
        self.nets(horizontal, x, y).get(&net.id).map_or(0, |(u, _)| i32::from(u.deficit))
    }

    /// `getNDRDeficitIfRouted` — the edge's deficit once the net uses it: the edge's own, plus
    /// what the net would miss when it is not on the edge yet. 0 for an edge-cost-1 net.
    pub fn ndr_deficit_if_routed(&self, net: &NdrCostNet, x: usize, y: usize, horizontal: bool) -> i32 {
        if net.edge_cost == 1 {
            return 0;
        }
        let edge = self.edge_ndr_deficit(horizontal, x, y);
        if self.has_net(horizontal, x, y, net.id) {
            return edge;
        }
        edge + self.compute_ndr_deficit(net, x, y, horizontal)
    }

    /// `getCostNDRAware` — the usage an update actually charges, and the bookkeeping behind it.
    ///
    /// ⛔ `amount` is read for its SIGN only once the net is an NDR net, and `0` counts as an add.
    /// A rip-up gives back exactly what the net's record says it took (the nets are not ripped up
    /// in the order they were routed), then refits the edge's deficit nets while a deficit is left.
    pub fn get_cost_ndr_aware(&mut self, net: &NdrCostNet, x: usize, y: usize, amount: f64, horizontal: bool) -> f64 {
        let ec = net.edge_cost;
        if ec == 1 {
            return amount;
        }
        let present = self.has_net(horizontal, x, y, net.id);
        if amount < 0.0 {
            if !present {
                return 0.0;
            }
            let (usage, _) = self.nets_mut(horizontal, x, y).remove(&net.id).expect("present");
            let d = self.deficit_mut(horizontal, x, y);
            *d = d.wrapping_sub(usage.deficit as u16);
            self.deficit_total -= i32::from(usage.deficit);
            self.release_ndr_cap_layer(x, y, horizontal, usage);
            if self.has_ndr_deficit(horizontal, x, y) {
                self.refit_ndr_deficit_nets(x, y, horizontal);
            }
            -f64::from(ec)
        } else if !present {
            let usage = self.reserve_ndr_cap_layer(x, y, net, horizontal);
            let d = self.deficit_mut(horizontal, x, y);
            *d = d.wrapping_add(usage.deficit as u16);
            self.deficit_total += i32::from(usage.deficit);
            self.nets_mut(horizontal, x, y).insert(net.id, (usage, net.clone()));
            f64::from(ec)
        } else {
            0.0
        }
    }

    /// `reserveNDRCapLayer` — debit the net's layer edge cost from the FIRST layer in its range
    /// with room (deficit 0); with none, compute the deficit and debit its lowest layer anyway (the
    /// NDR capacity goes negative: the oversubscription, restored by the rip-up).
    pub fn reserve_ndr_cap_layer(&mut self, x: usize, y: usize, net: &NdrCostNet, horizontal: bool) -> NdrUsage {
        for l in net.min_layer..=net.max_layer {
            let lec = net.layer_edge_cost(l);
            if self.cap(horizontal, l, x, y).cap_ndr() >= i32::from(lec) {
                self.cap_mut(horizontal, l, x, y).debit += i32::from(lec);
                return NdrUsage { layer: l as i16, amount: lec, deficit: 0 };
            }
        }
        let lec = net.layer_edge_cost(net.min_layer);
        let deficit = self.compute_ndr_deficit(net, x, y, horizontal) as i8;
        self.cap_mut(horizontal, net.min_layer, x, y).debit += i32::from(lec);
        NdrUsage { layer: net.min_layer as i16, amount: lec, deficit }
    }

    /// `releaseNDRCapLayer` — give back exactly what [`reserve_ndr_cap_layer`](Self::reserve_ndr_cap_layer)
    /// took; a record with no layer gives back nothing.
    pub fn release_ndr_cap_layer(&mut self, x: usize, y: usize, horizontal: bool, usage: NdrUsage) {
        if usage.layer < 0 {
            return;
        }
        self.cap_mut(horizontal, usage.layer as usize, x, y).debit -= i32::from(usage.amount);
    }

    /// `refitNDRDeficitNets` (#11963 74d34bd075) — the tracks a leaving net freed may host the NDR
    /// nets that miss tracks on the edge: each net with a deficit is released and reserved again,
    /// in DATABASE-ID order (the reference's map is ordered by address), its deficit and the
    /// totals moved by the difference.
    pub fn refit_ndr_deficit_nets(&mut self, x: usize, y: usize, horizontal: bool) {
        let mut deficit_nets: Vec<usize> = self.nets(horizontal, x, y).iter().filter(|(_, (u, _))| u.deficit > 0).map(|(&id, _)| id).collect();
        let key = |id: usize| self.db_ids.get(id).copied().unwrap_or(id as u32);
        deficit_nets.sort_by_key(|&id| key(id));
        for id in deficit_nets {
            let (old, net) = self.nets(horizontal, x, y)[&id].clone();
            self.release_ndr_cap_layer(x, y, horizontal, old);
            let refit = self.reserve_ndr_cap_layer(x, y, &net, horizontal);
            let delta = i32::from(refit.deficit) - i32::from(old.deficit);
            let d = self.deficit_mut(horizontal, x, y);
            *d = (i32::from(*d) + delta) as u16;
            self.deficit_total += delta;
            self.nets_mut(horizontal, x, y).insert(id, (refit, net));
        }
    }

    /// `resetNDRCap` — every NDR debit dropped: the NDR capacity is the full capacity again.
    pub fn reset_ndr_cap(&mut self) {
        for c in self.h_cap.iter_mut().chain(self.v_cap.iter_mut()) {
            c.debit = 0;
        }
    }

    /// `copyRoutingStateFrom` — take another graph's capacities with the edges, and its NDR state
    /// (records, debits, deficits) only when asked; otherwise the records, the debits and the
    /// deficits are ONE piece of bookkeeping and all three are dropped.
    pub fn copy_routing_state_from(&mut self, other: &NdrLedger, include_ndr_state: bool) {
        let db_ids = std::mem::take(&mut self.db_ids);
        *self = other.clone();
        if self.db_ids.is_empty() {
            self.db_ids = db_ids;
        }
        if !include_ndr_state {
            self.clear_ndr_nets();
            self.reset_ndr_cap();
        }
    }

    /// `clearNDRnets` — every record dropped, with the per-edge deficits (the sums of the records)
    /// and their total (#11963 1fe08158b2). The debits are left alone.
    pub fn clear_ndr_nets(&mut self) {
        self.h_nets.iter_mut().chain(self.v_nets.iter_mut()).for_each(BTreeMap::clear);
        self.h_deficit.iter_mut().chain(self.v_deficit.iter_mut()).for_each(|d| *d = 0);
        self.deficit_total = 0;
    }
}
