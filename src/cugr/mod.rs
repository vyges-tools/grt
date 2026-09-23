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
//!
//! [`init`] is `CUGR::init`'s call sequence and does no work of its own.

pub mod design;
pub mod geo;
pub mod grid_graph;
pub mod grnet;
pub mod layers;
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
