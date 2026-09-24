// SPDX-License-Identifier: Apache-2.0
//! The second global router, `global_route -use_cugr`.
//!
//! Reimplemented from the behaviour of CUGR 2.0 (the Chinese University of Hong Kong; the EDGE
//! global router) as the OpenROAD project's global router embeds it — no source is carried over.
//!
//! Built one stage at a time, earliest first. What exists:
//!
//! | stage | reference | here |
//! | --- | --- | --- |
//! | 0 · the model | `Design`, `GridGraph`, `GRNet` construction | [`init`] |
//! | 1 · pattern routing | `CUGR::patternRoute` | [`Cugr::pattern_route`] |
//!
//! [`init`] is `CUGR::init`'s call sequence and does no work of its own; so is each stage.

pub mod design;
pub mod geo;
pub mod grid_graph;
pub mod grnet;
pub mod layers;
pub mod pattern_route;
pub mod trace;
#[cfg(feature = "odb")]
pub mod read;
#[cfg(feature = "odb")]
pub mod route;

use design::{Design, DesignError, DesignFacts};
use grid_graph::{GridError, GridGraph};
use grnet::GrNet;

/// CUGR's tuning constants (`Constants`), at the reference's defaults.
#[derive(Debug, Clone, PartialEq)]
pub struct Constants {
    pub weight_wire_length: f64,
    pub weight_via_number: f64,
    pub weight_short_area: f64,
    /// 0-based; `init` sets it to the min routing layer − 1.
    pub min_routing_layer: i32,
    pub cost_logistic_slope: f64,
    pub max_detour_ratio: f64,
    pub target_detour_count: i32,
    pub via_multiplier: f64,
    pub maze_logistic_slope: f64,
    pub resistance_min_net_length: i32,
    pub resistance_weight: f64,
    pub congestion_gate_penalty: f64,
}

impl Default for Constants {
    fn default() -> Self {
        Constants {
            weight_wire_length: 0.5,
            weight_via_number: 4.0,
            weight_short_area: 500.0,
            min_routing_layer: 1,
            cost_logistic_slope: 1.0,
            max_detour_ratio: 0.25,
            target_detour_count: 20,
            via_multiplier: 2.0,
            maze_logistic_slope: 0.5,
            resistance_min_net_length: 3,
            resistance_weight: 50.0,
            congestion_gate_penalty: 4.0,
        }
    }
}

/// The router after `init`: the design, the graph, and the nets it will route.
#[derive(Debug, Clone, PartialEq)]
pub struct Cugr {
    pub constants: Constants,
    pub design: Design,
    pub grid: GridGraph,
    /// Indexed by net index (every design net is valid at `init`).
    pub nets: Vec<GrNet>,
}

/// A stage that stopped where the reference would have gone on to something not modelled, or
/// would have raised an error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StageError {
    Pattern(pattern_route::PatternError),
    Commit(grid_graph::CommitError),
}

impl Cugr {
    /// `sortNetIndices(nets, res_aware_order=false)`: a STABLE sort by `(slack, bbox half
    /// perimeter)`, from index order.
    pub fn sort_net_indices(&self, indices: &mut [usize]) {
        indices.sort_by(|&a, &b| {
            let (na, nb) = (&self.nets[a], &self.nets[b]);
            (na.slack, na.bounding_box.hp()).partial_cmp(&(nb.slack, nb.bounding_box.hp())).unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    /// `patternRoute` (stage 1): every net in the neutral order; a net of two or more pins is
    /// pattern-routed and its tree's usage committed before the next is routed.
    ///
    /// `alphas[k]` is net `k`'s Steiner alpha. With `trace`, the stage's records are appended.
    pub fn pattern_route(&mut self, alphas: &[f32], stt: pattern_route::SteinerBuilder<'_>, log: &mut Vec<String>, mut trace: Option<&mut Vec<String>>) -> Result<(), StageError> {
        let mut order: Vec<usize> = (0..self.nets.len()).collect();
        self.sort_net_indices(&mut order);
        if let Some(t) = trace.as_deref_mut() {
            trace::order(t, self, &order);
        }
        for &k in &order {
            if self.nets[k].num_pins() < 2 {
                continue;
            }
            let cx = pattern_route::CostContext { grid: &self.grid, design: &self.design, constants: &self.constants, cost_multiplier: 1.0 };
            let route = pattern_route::pattern_route_net(&mut self.nets[k], alphas[k], stt, &cx, log).map_err(StageError::Pattern)?;
            let mut commits = Vec::new();
            let tree = self.nets[k].routing_tree.clone().expect("set by the route");
            self.grid.commit_tree(&self.design, &tree, false, &self.nets[k].ndr_costs, &mut commits).map_err(StageError::Commit)?;
            if let Some(t) = trace.as_deref_mut() {
                trace::net_route(t, &self.nets[k], &route, &commits, 1);
            }
        }
        if let Some(t) = trace {
            trace::demand(t, &self.grid, 1);
        }
        Ok(())
    }

    /// `updateCongestedNets(threshold 1)` over every routed net: those whose tree crosses an edge
    /// with more demand than capacity. Empty means stages 3 to 5 have nothing to do.
    pub fn congested_nets(&self) -> Vec<usize> {
        (0..self.nets.len())
            .filter(|&k| self.nets[k].routing_tree.as_ref().is_some_and(|t| self.grid.check_congestion(t, 1.0) > 0))
            .collect()
    }
}

impl Cugr {
    /// `gridlineCenter(dimension, index)`: a gcell's LOW gridline plus half the NOMINAL gcell —
    /// not the gcell's own centre (the last gcell is wider).
    pub fn gridline_center(&self, dimension: usize, index: i32) -> i32 {
        self.grid.gridlines[dimension][index as usize] + self.design.gridline_spacing / 2
    }

    /// `GRNet::isLocal`: no preferred access point, or every one at the same cell.
    pub fn is_local(net: &grnet::GrNet) -> bool {
        let mut cells = net.preferred_aps.values().map(|&(p, _)| p);
        match cells.next() {
            None => true,
            Some(first) => cells.all(|p| p == first),
        }
    }

    /// `getNetRoute` / `buildNetRoute`: a net of two or more pins that is not local, as DBU
    /// segments at gridline centres, in tree preorder.
    ///
    /// Upstream rule: a same-layer child is one segment from the lower to the higher corner, a
    /// degenerate one (same cell) skipped; a layer change is one via segment per layer crossed, at
    /// the parent's cell, bottom up. Layers are 1-based routing levels.
    pub fn net_route(&self, net: &grnet::GrNet) -> Vec<crate::GSegment> {
        let mut route = Vec::new();
        let Some(tree) = net.routing_tree.as_ref() else { return route };
        if net.num_pins() < 2 || Cugr::is_local(net) {
            return route;
        }
        for n in tree.preorder() {
            let node = &tree.nodes[n];
            for &ch in &node.children {
                let child = &tree.nodes[ch];
                if node.layer == child.layer {
                    if node.p == child.p {
                        continue;
                    }
                    route.push(crate::GSegment {
                        init_x: self.gridline_center(0, node.p.x.min(child.p.x)),
                        init_y: self.gridline_center(1, node.p.y.min(child.p.y)),
                        init_layer: node.layer + 1,
                        final_x: self.gridline_center(0, node.p.x.max(child.p.x)),
                        final_y: self.gridline_center(1, node.p.y.max(child.p.y)),
                        final_layer: child.layer + 1,
                        is_jumper: false,
                    });
                } else {
                    let (x, y) = (self.gridline_center(0, node.p.x), self.gridline_center(1, node.p.y));
                    for l in node.layer.min(child.layer)..node.layer.max(child.layer) {
                        route.push(crate::GSegment { init_x: x, init_y: y, init_layer: l + 1, final_x: x, final_y: y, final_layer: l + 2, is_jumper: false });
                    }
                }
            }
        }
        route
    }

    /// `getITermsAccessPoints` / `getBTermsAccessPoints` for one pin, by terminal name: the chosen
    /// cell's LOW gridlines and the top of the pin's layer interval, as a 1-based layer.
    pub fn pin_access_point(&self, net: &grnet::GrNet, pin_name: &str, is_port: bool) -> Option<(i32, i32, i32)> {
        let design_net = self.design.nets.get(net.index)?;
        let pin = design_net.pins.iter().find(|p| p.name == pin_name && p.is_port == is_port)?;
        let &(p, layers) = net.preferred_aps.get(&pin.index)?;
        Some((self.grid.gridlines[0][p.x as usize], self.grid.gridlines[1][p.y as usize], layers.high + 1))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InitError {
    Design(DesignError),
    Grid(GridError),
}

/// `CUGR::init(min, max, clock_nets)`: `Design`, then `GridGraph`, then one `GRNet` per net.
///
/// `driver_terms[k]` is the first driver terminal of `facts.nets[k]` (see
/// [`grnet::GrNet::new`]). NDR costs are not modelled yet.
pub fn init(facts: &DesignFacts, driver_terms: &[String], min_routing_layer: i32, max_routing_layer: i32) -> Result<Cugr, InitError> {
    let constants = Constants { min_routing_layer: min_routing_layer - 1, ..Constants::default() };
    let design = Design::new(facts, &constants, min_routing_layer, max_routing_layer).map_err(InitError::Design)?;
    let grid = GridGraph::new(&design, constants.min_routing_layer as usize).map_err(InitError::Grid)?;
    let nets = design
        .nets
        .iter()
        .map(|n| GrNet::new(n, &driver_terms[n.facts_index], &grid))
        .collect::<Result<Vec<_>, _>>()
        .map_err(InitError::Grid)?;
    Ok(Cugr { constants, design, grid, nets })
}

#[cfg(test)]
mod tests {
    use super::design::{DesignFacts, TechLayerFacts};
    use super::geo::{Interval, Point};
    use super::grid_graph::GrTree;
    use super::grnet::{GrNet, GrPoint};
    use super::layers::MetalLayerFacts;
    use super::*;

    fn layer(name: &str, level: i32, horizontal: bool) -> TechLayerFacts {
        TechLayerFacts {
            name: name.into(),
            is_routing: true,
            routing_level: level,
            upper_layer: String::new(),
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

    /// Gridlines 0, 1000, …, 4000, 5900: the last gcell is 1900 wide.
    fn router() -> Cugr {
        let f = DesignFacts {
            dbu_per_micron: 1000,
            die: (0, 0, 5900, 5900),
            gcell_tile_size: 1000,
            layers: vec![layer("m1", 1, true), layer("m2", 2, false), layer("m3", 3, true)],
            vias: Vec::new(),
            nets: Vec::new(),
            instance_shapes: Vec::new(),
            design_obstructions: Vec::new(),
            min_layer_for_clock: 0,
            max_layer_for_clock: 0,
        };
        init(&f, &[], 2, 3).unwrap()
    }

    fn net_with(tree: GrTree, aps: &[(usize, Point)]) -> GrNet {
        GrNet {
            index: 0,
            name: "n".into(),
            pin_access_points: vec![vec![GrPoint { layer: 0, p: Point::new(0, 0) }]; 2],
            bounding_box: Default::default(),
            driver_pin_index: -1,
            layer_range: design::LayerRange { min_layer: 1, max_layer: 2 },
            slack: 0.0,
            preferred_aps: aps.iter().map(|&(pin, p)| (pin, (p, Interval::point(0)))).collect(),
            ndr_costs: vec![1.0; 3],
            routing_tree: Some(tree),
            shape_ap_choices: Vec::new(),
        }
    }

    // Upstream rule (CUGR `buildNetRoute` / `gridlineCenter`): a cell's DBU point is its LOW
    // gridline plus HALF THE NOMINAL gcell — in the 1900-wide last gcell (4000..5900) that is 4500,
    // not its own centre 4950. A same-layer child at the SAME cell is skipped; a layer change is one
    // via segment per layer crossed, at the parent's cell, bottom up; layers are 1-based.
    #[test]
    fn net_route_segments() {
        let c = router();
        let mut t = GrTree::default();
        let root = t.add(0, Point::new(1, 4));
        let up = t.add(2, Point::new(1, 4));
        let same = t.add(2, Point::new(1, 4));
        let far = t.add(2, Point::new(4, 4));
        t.nodes[root].children.push(up);
        t.nodes[up].children.push(same);
        t.nodes[up].children.push(far);
        let n = net_with(t, &[(0, Point::new(1, 4)), (1, Point::new(4, 4))]);
        let r = c.net_route(&n);
        let seg = |s: &crate::GSegment| (s.init_x, s.init_y, s.init_layer, s.final_x, s.final_y, s.final_layer);
        let got: Vec<_> = r.iter().map(seg).collect();
        assert_eq!(got, vec![(1500, 4500, 1, 1500, 4500, 2), (1500, 4500, 2, 1500, 4500, 3), (1500, 4500, 3, 4500, 4500, 3)]);
    }

    // Upstream rule (CUGR `getNetRoute`, `GRNet::isLocal`): a net whose pins all chose ONE cell
    // exports nothing — `addRemainingGuides` covers it.
    #[test]
    fn a_local_net_exports_nothing() {
        let c = router();
        let mut t = GrTree::default();
        let a = t.add(1, Point::new(2, 2));
        let b = t.add(2, Point::new(2, 2));
        t.nodes[a].children.push(b);
        assert!(c.net_route(&net_with(t.clone(), &[(0, Point::new(2, 2)), (1, Point::new(2, 2))])).is_empty());
        assert_eq!(c.net_route(&net_with(t, &[(0, Point::new(2, 2)), (1, Point::new(2, 3))])).len(), 1);
    }
}
