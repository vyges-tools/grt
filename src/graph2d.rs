// SPDX-License-Identifier: Apache-2.0
//! The router's 2D graph (`Graph2D`) as the pattern-routing stages share it: the estimated usage,
//! the NDR ledger, the used-grid sets and the 2D capacities.
//!
//! ⚠️ The edge reductions (`red`) are held by the caller, not here: they are fixed for the run, and
//! the per-edge routing rules read them while this struct is borrowed mutably for the update.

use std::collections::BTreeSet;

use crate::estimate::{Usage2d, EstimateGrid};
use crate::ndr_cost::{NdrCostNet, NdrLedger};
use crate::overflow2d::{get_overflow_2d, Overflow2DScan, UsedCell};

#[derive(Debug, Clone, PartialEq)]
pub struct Graph2d {
    pub est: EstimateGrid,
    pub ndr: NdrLedger,
    /// `h_used_ggrid_` / `v_used_ggrid_` — ⛔ `std::set<pair<int,int>>`: ordered by `(x, y)`, and an
    /// edge enters when an update's RAW amount is positive (whatever the NDR charge made of it) and
    /// never leaves during the run.
    pub used_h: BTreeSet<(i32, i32)>,
    pub used_v: BTreeSet<(i32, i32)>,
    /// The 2D capacity per edge, `[y * x_grid + x]`.
    pub cap_h: Vec<u16>,
    pub cap_v: Vec<u16>,
}

impl Graph2d {
    pub fn new(x_grid: usize, y_grid: usize, num_layers: usize) -> Self {
        Graph2d {
            est: EstimateGrid::new(x_grid, y_grid),
            ndr: NdrLedger::new(x_grid, y_grid, num_layers),
            used_h: BTreeSet::new(),
            used_v: BTreeSet::new(),
            cap_h: vec![0; x_grid * y_grid],
            cap_v: vec![0; x_grid * y_grid],
        }
    }

    /// The usage updates of one net — `updateEstUsageH/V(…, net, …)`.
    pub fn for_net<'a>(&'a mut self, net: &'a NdrCostNet) -> NetUsage<'a> {
        NetUsage { g: self, net }
    }

    /// `clearUsed` — empty both used-grid sets (the run's R2).
    pub fn clear_used(&mut self) {
        self.used_h.clear();
        self.used_v.clear();
    }

    /// `getOverflow2D`'s scan over the used grids, in set order.
    pub fn get_overflow_2d(&self) -> Overflow2DScan {
        let xg = self.est.x_grids;
        let cell = |x: i32, y: i32, est: f64, cap: u16| UsedCell { x, y, usage: 0, est_usage: est, cap };
        let h: Vec<UsedCell> = self.used_h.iter().map(|&(x, y)| cell(x, y, self.est.h(x as usize, y as usize), self.cap_h[y as usize * xg + x as usize])).collect();
        let v: Vec<UsedCell> = self.used_v.iter().map(|&(x, y)| cell(x, y, self.est.v(x as usize, y as usize), self.cap_v[y as usize * xg + x as usize])).collect();
        get_overflow_2d(&h, &v)
    }
}

/// One net's view of the graph for usage updates: each edge charged through the NDR-aware cost,
/// and marked used when the amount is positive.
///
/// ⚠️ The interval is walked edge by edge in increasing coordinate, as the reference's
/// `for (x = lo; x < hi; x++) updateEstUsageH(x, …)` does — each edge's charge reads and writes that
/// edge's NDR state.
pub struct NetUsage<'a> {
    g: &'a mut Graph2d,
    net: &'a NdrCostNet,
}

impl Usage2d for NetUsage<'_> {
    fn h(&self, x: usize, y: usize) -> f64 {
        self.g.est.h(x, y)
    }
    fn v(&self, x: usize, y: usize) -> f64 {
        self.g.est.v(x, y)
    }
    fn update_h(&mut self, x1: i32, x2: i32, y: i32, amount: f64) {
        for x in x1..x2 {
            let c = self.g.ndr.get_cost_ndr_aware(self.net, x as usize, y as usize, amount, true);
            self.g.est.update_h(x, x + 1, y, c);
            if amount > 0.0 {
                self.g.used_h.insert((x, y));
            }
        }
    }
    fn update_v(&mut self, x: i32, y1: i32, y2: i32, amount: f64) {
        for y in y1..y2 {
            let c = self.g.ndr.get_cost_ndr_aware(self.net, x as usize, y as usize, amount, false);
            self.g.est.update_v(x, y, y + 1, c);
            if amount > 0.0 {
                self.g.used_v.insert((x, y));
            }
        }
    }
    fn update_usage_h(&mut self, x: i32, y: i32, amount: f64) {
        let c = self.g.ndr.get_cost_ndr_aware(self.net, x as usize, y as usize, amount, true);
        self.g.est.update_usage_h(x, y, c);
        if amount > 0.0 {
            self.g.used_h.insert((x, y));
        }
    }
    fn update_usage_v(&mut self, x: i32, y: i32, amount: f64) {
        let c = self.g.ndr.get_cost_ndr_aware(self.net, x as usize, y as usize, amount, false);
        self.g.est.update_usage_v(x, y, c);
        if amount > 0.0 {
            self.g.used_v.insert((x, y));
        }
    }
}
