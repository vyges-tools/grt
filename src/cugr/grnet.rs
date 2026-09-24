// SPDX-License-Identifier: Apache-2.0
//! One net as the router holds it (`GRNet`): the gcells each pin's shapes touch, the net's
//! bounding box over them, and which pin drives it.

use std::collections::{BTreeMap, HashSet};

use super::design::{CugrNet, LayerRange};
use super::geo::{BoxT, Interval, Point};
use super::grid_graph::{GrTree, GridError, GridGraph};

/// A gcell on a layer (`GRPoint`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GrPoint {
    pub layer: i32,
    pub p: Point,
}

#[derive(Debug, Clone, PartialEq)]
pub struct GrNet {
    pub index: usize,
    pub name: String,
    /// Per pin, the gcells its shapes overlap, in first-seen order.
    pub pin_access_points: Vec<Vec<GrPoint>>,
    pub bounding_box: BoxT,
    /// The pin that is the net's first driver terminal, -1 when none is.
    pub driver_pin_index: i32,
    pub layer_range: LayerRange,
    pub slack: f32,
    /// `preferred_aps_`: per pin, the chosen cell and the pin's own layers (a `std::map`: pin order).
    pub preferred_aps: BTreeMap<usize, (Point, Interval)>,
    /// Per-layer NDR demand factor (`computeNdrCosts`); all 1 for a net without a rule.
    pub ndr_costs: Vec<f64>,
    pub routing_tree: Option<GrTree>,
    /// The last `selectShapeAccessPoint` choices, for the trace: `(pin, index, accessibility,
    /// distance, bbox centre)`.
    pub shape_ap_choices: Vec<(usize, i32, i32, i32, Point)>,
    /// Demoted to the default rule by RRR (`setSoftNdr`).
    pub soft_ndr: bool,
}

impl GrNet {
    /// The `GRNet` constructor.
    ///
    /// Upstream rule: each pin's shapes are mapped to the gcells they overlap (`rangeSearchCells`)
    /// on the shape's own layer, x outer and y inner, each (layer, x, y) kept once per pin; a shape
    /// mapping to no cell is skipped. The bounding box runs over every pin's cells. The driver is
    /// the pin whose terminal is the net's first driver terminal.
    pub fn new(net: &CugrNet, driver_term: &str, grid: &GridGraph) -> Result<GrNet, GridError> {
        let mut pin_access_points = vec![Vec::new(); net.pins.len()];
        let mut driver_pin_index = -1;
        for pin in &net.pins {
            let mut included = HashSet::new();
            for shape in &pin.shapes {
                let cells = grid.range_search_cells(&shape.b)?;
                if !cells.is_valid() {
                    continue;
                }
                for x in cells.lx()..=cells.hx() {
                    for y in cells.ly()..=cells.hy() {
                        if included.insert((shape.layer, x, y)) {
                            pin_access_points[pin.index].push(GrPoint { layer: shape.layer, p: Point::new(x, y) });
                        }
                    }
                }
            }
            let term = if pin.is_port { format!("B:{}", pin.name) } else { format!("I:{}", pin.name) };
            if term == driver_term {
                driver_pin_index = pin.index as i32;
            }
        }
        let mut bounding_box = BoxT::default();
        for points in &pin_access_points {
            for g in points {
                bounding_box.update(g.p);
            }
        }
        Ok(GrNet {
            index: net.index,
            name: net.name.clone(),
            pin_access_points,
            bounding_box,
            driver_pin_index,
            layer_range: net.layer_range,
            slack: 0.0,
            preferred_aps: BTreeMap::new(),
            ndr_costs: vec![1.0; grid.num_layers],
            routing_tree: None,
            shape_ap_choices: Vec::new(),
            soft_ndr: false,
        })
    }

    /// `isInsideLayerRange`.
    pub fn is_inside_layer_range(&self, layer: i32) -> bool {
        layer >= self.layer_range.min_layer && layer <= self.layer_range.max_layer
    }

    /// `getNdrCost(layer)`: 1 outside the vector.
    pub fn ndr_cost(&self, layer: usize) -> f64 {
        self.ndr_costs.get(layer).copied().unwrap_or(1.0)
    }

    /// `hasNdr`: some layer's factor strictly above 1.
    pub fn has_ndr(&self) -> bool {
        self.ndr_costs.iter().any(|&c| c > 1.0)
    }

    /// `setSoftNdr`: every factor back to 1.
    pub fn set_soft_ndr(&mut self) {
        self.soft_ndr = true;
        self.ndr_costs.iter_mut().for_each(|c| *c = 1.0);
    }

    /// `getDriverAccessPoint`: the driver pin's chosen cell, if it has one.
    pub fn driver_access_point(&self) -> Option<Point> {
        usize::try_from(self.driver_pin_index).ok().and_then(|d| self.preferred_aps.get(&d)).map(|&(p, _)| p)
    }

    pub fn num_pins(&self) -> usize {
        self.pin_access_points.len()
    }
}
