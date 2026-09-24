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
