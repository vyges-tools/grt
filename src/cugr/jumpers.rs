// SPDX-License-Identifier: Apache-2.0
//! CUGR's side of antenna repair's jumper pass: the resource questions GlobalRouter forwards to
//! CUGR when it routed (`hasAvailableResources`, `hasJumperResources`), and the re-adoption of a
//! jumpered route (`updateJumperedRoute` / `restoreNetDemand` → `restoreNetRoute`).

use std::collections::HashMap;

use super::geo::Point;
use super::grid_graph::Commit;
use super::restore::Restore;
use super::Cugr;
use crate::repair_antennas::{JumperGrid, JumperRouter};
use crate::GSegment;

pub struct CugrJumpers<'a> {
    pub cugr: &'a mut Cugr,
    /// GlobalRouter's grid (`dbuToTile`, clamped).
    pub grid: JumperGrid,
    /// Net name → index.
    pub index: HashMap<String, usize>,
    /// Each restore, for the trace: `(net, answer)`.
    pub restores: Vec<(String, Restore)>,
    pub commits: Vec<Commit>,
    /// A path the reference takes that is not modelled here (a net re-queued for rerouting, a
    /// restore of a net the router does not hold, an invalid layer): the command is refused.
    pub unmodelled: Option<String>,
}

impl CugrJumpers<'_> {
    fn net(&self, name: &str) -> Option<usize> {
        self.index.get(name).copied()
    }

    /// `isEdgeInGrid`: a layer below the top and a cell on the grid.
    fn in_grid(&self, layer: i32, x: i32, y: i32) -> bool {
        let g = &self.cugr.grid;
        layer < g.num_layers as i32 && x >= 0 && (x as usize) < g.x_size && y >= 0 && (y as usize) < g.y_size
    }

    fn restore(&mut self, route: &[GSegment], net: &str) -> bool {
        let Some(k) = self.net(net) else {
            self.unmodelled = Some(format!("cugr: restoring net {net}, which the router does not hold, is not modelled"));
            return false;
        };
        match self.cugr.restore_net_route(k, route, &mut self.commits) {
            Ok(r) => {
                let ok = matches!(r, Restore::Ok(_));
                self.restores.push((net.to_string(), r));
                ok
            }
            Err(e) => {
                self.unmodelled = Some(format!("cugr: restoring net {net}: {e:?}"));
                false
            }
        }
    }
}

impl JumperRouter for CugrJumpers<'_> {
    /// `CUGR::hasAvailableResources`: the edge at the point's cell on the (1-based) layer has free
    /// capacity for the net's per-layer factor (1 for a net it does not hold); off the grid, none.
    fn has_available_resources(&mut self, _is_horizontal: bool, x: i32, y: i32, layer_level: i32, net: &str) -> bool {
        let layer = layer_level - 1;
        if layer < 0 {
            self.unmodelled = Some(format!("GRT-0705: invalid layer index {layer_level}"));
            return false;
        }
        let (gx, gy) = (self.grid.dbu_to_tile(x, true), self.grid.dbu_to_tile(y, false));
        if !self.in_grid(layer, gx, gy) {
            return false;
        }
        let demand = self.net(net).map_or(1.0, |k| self.cugr.nets[k].ndr_cost(layer as usize));
        let e = self.cugr.grid.edge(layer as usize, gx as usize, gy as usize);
        e.capacity - e.demand >= demand
    }

    /// `CUGR::hasJumperResources`: the jumper's whole demand fits, per edge.
    ///
    /// Upstream rule: per edge, the jumper wire (+ factor) on its layer, the original wire two
    /// layers down CREDITED back (− factor, only when the net holds a tree), and at each end the
    /// flank deposits of the two vias climbing to the jumper — summed, then each edge with positive
    /// demand must have that much free capacity. Both ends must be on the grid; the jumper layer
    /// must have two layers under it.
    fn has_jumper_resources(&mut self, init: (i32, i32), fin: (i32, i32), layer_level: i32, net: &str) -> bool {
        let layer = layer_level - 1;
        let g = &self.cugr.grid;
        if layer - 2 < 0 || layer >= g.num_layers as i32 {
            return false;
        }
        let ends = [
            Point::new(self.grid.dbu_to_tile(init.0, true), self.grid.dbu_to_tile(init.1, false)),
            Point::new(self.grid.dbu_to_tile(fin.0, true), self.grid.dbu_to_tile(fin.1, false)),
        ];
        if ends.iter().any(|e| !self.in_grid(layer, e.x, e.y)) {
            return false;
        }
        let k = self.net(net);
        let costs: Vec<f64> = k.map(|k| self.cugr.nets[k].ndr_costs.clone()).unwrap_or_default();
        let factor = |l: i32| k.map_or(1.0, |k| self.cugr.nets[k].ndr_cost(l as usize));
        let mut demands: HashMap<(usize, i32, i32), f64> = HashMap::new();
        let wire = |demands: &mut HashMap<(usize, i32, i32), f64>, l: i32, sign: f64| {
            let d = g.layer_directions[l as usize];
            let f = sign * factor(l);
            for c in ends[0].get(d).min(ends[1].get(d))..ends[0].get(d).max(ends[1].get(d)) {
                let mut lower = Point::new(0, 0);
                lower.set(d, c);
                lower.set(1 - d, ends[0].get(1 - d));
                *demands.entry((l as usize, lower.x, lower.y)).or_insert(0.0) += f;
            }
        };
        wire(&mut demands, layer, 1.0);
        if k.is_some_and(|k| self.cugr.nets[k].routing_tree.is_some()) {
            wire(&mut demands, layer - 2, -1.0);
        }
        for e in &ends {
            for l in (layer - 2)..layer {
                let l = l as usize;
                for (fl, edge, demand, f) in g.via_flank_edges(l, *e, self.cugr.design.via_demand_length_lower[l], self.cugr.design.via_demand_length_upper[l], &costs) {
                    *demands.entry((fl, edge.x, edge.y)).or_insert(0.0) += demand * f;
                }
            }
        }
        demands.iter().all(|(&(l, x, y), &d)| {
            let e = g.edge(l, x as usize, y as usize);
            !(d > 0.0 && e.capacity - e.demand < d)
        })
    }

    /// `updateJumperedRoute` under CUGR: re-adopt the jumpered route (`restoreNetRoute`).
    fn update_jumpered_route(&mut self, route: &[GSegment], _: (i32, i32), _: (i32, i32), _: i32, _: i32, net: &str) -> bool {
        self.restore(route, net)
    }

    /// `restoreNetDemand`: re-adopt the rolled-back route; a failure re-queues the net for a
    /// reroute (GRT-0311), which is not modelled.
    fn restore_net_demand(&mut self, route: &[GSegment], net: &str) {
        if !self.restore(route, net) && self.unmodelled.is_none() {
            self.unmodelled = Some(format!("cugr: net {net} could not be re-adopted after a rejected jumper (GRT-0311) — the reroute is not modelled"));
        }
    }

    /// CUGR's edges are not FastRoute's: its answer is `has_available_resources` alone.
    fn headroom(&self, _: bool, _: i32, _: i32, _: i32, _: &str) -> (i32, i32, i32) {
        (-1, -1, -1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cugr::design::{DesignFacts, TechLayerFacts, TechViaFacts};
    use crate::cugr::grid_graph::GrTree;
    use crate::cugr::layers::MetalLayerFacts;
    use crate::cugr::{init, Cugr};

    fn layer(name: &str, level: i32, horizontal: bool) -> TechLayerFacts {
        TechLayerFacts {
            name: name.into(),
            is_routing: true,
            routing_level: level,
            upper_layer: format!("{name}_cut"),
            metal: Some(MetalLayerFacts {
                name: name.into(),
                routing_level: level,
                horizontal,
                width: 100,
                min_width: 100,
                spacing: 100,
                resistance: 0.0,
                via_resistance: 0.0,
                tracks: (200, 100, 30),
                area: 0,
                v55_widths_and_lengths: None,
                v55_table: None,
                adjustment: 0.0,
            }),
        }
    }

    fn via(name: &str, bottom: &str, top: &str) -> TechViaFacts {
        TechViaFacts {
            name: name.into(),
            bottom: bottom.into(),
            top: top.into(),
            boxes: vec![(bottom.to_string(), (0, 0, 100, 100)), (top.to_string(), (0, 0, 100, 100))],
            or_default: false,
            is_default: true,
        }
    }

    /// 6×6 gcells of 1000; m1 H, m2 V, m3 H, ALL routable (min layer 1); vias m1–m2, m2–m3 whose
    /// 100×100 pads block 2 tracks: 200 of demand length, 0.1 per 2000-wide flank.
    fn router() -> Cugr {
        let f = DesignFacts {
            dbu_per_micron: 1000,
            die: (0, 0, 6001, 6001),
            gcell_tile_size: 1000,
            layers: vec![layer("m1", 1, true), layer("m2", 2, false), layer("m3", 3, true)],
            vias: vec![via("v12", "m1", "m2"), via("v23", "m2", "m3")],
            nets: Vec::new(),
            instance_shapes: Vec::new(),
            design_obstructions: Vec::new(),
            min_layer_for_clock: 0,
            max_layer_for_clock: 0,
        };
        let mut c = init(&f, &[], 1, 3).unwrap();
        let n = crate::cugr::pattern_route::tests_support::net(&c.grid, &[&[(0, 1, 1)], &[(0, 3, 1)]]);
        c.nets.push(n);
        c
    }

    fn jumpers(c: &mut Cugr) -> CugrJumpers<'_> {
        let grid = JumperGrid { grid: crate::Grid { tile_size: 1000, area: crate::Rect::new(0, 0, 6001, 6001) }, x_grids: 6, y_grids: 6 };
        CugrJumpers { cugr: c, grid, index: [("n".to_string(), 0)].into_iter().collect(), restores: Vec::new(), commits: Vec::new(), unmodelled: None }
    }

    // Upstream rule (CUGR `hasAvailableResources`): the free capacity must cover the net's
    // PER-LAYER NDR factor, not one track. No CUGR jumper script has an NDR net.
    #[test]
    fn available_resources_use_the_ndr_factor() {
        let mut c = router();
        c.nets[0].ndr_costs[2] = 2.0;
        let cap = c.grid.graph_edges[2][1][1].capacity;
        c.grid.graph_edges[2][1][1].demand = cap - 1.5;
        let mut j = jumpers(&mut c);
        assert!(!j.has_available_resources(true, 1500, 1500, 3, "n"), "1.5 free < a factor of 2");
        assert!(j.has_available_resources(true, 1500, 1500, 3, "other"), "a net it does not hold asks 1");
    }

    // Upstream rule (CUGR `hasJumperResources`): the original wire two layers down is CREDITED
    // (−1 per edge) when the net holds a tree, so an overfull lower edge that a via end also
    // loads (+0.1) still passes. No CUGR jumper script puts a via end on such an edge.
    #[test]
    fn jumper_resources_credit_the_original_wire() {
        let mut c = router();
        let mut t = GrTree::default();
        t.add(0, Point::new(1, 1));
        c.nets[0].routing_tree = Some(t);
        let cap = c.grid.graph_edges[0][1][1].capacity;
        c.grid.graph_edges[0][1][1].demand = cap - 0.05;
        let mut j = jumpers(&mut c);
        assert!(j.has_jumper_resources((1500, 1500), (3500, 1500), 3, "n"), "+0.1 via, −1 wire on m1 (1,1)");
        j.cugr.nets[0].routing_tree = None;
        assert!(!j.has_jumper_resources((1500, 1500), (3500, 1500), 3, "n"), "no tree, no credit: 0.1 > 0.05");
    }
}
