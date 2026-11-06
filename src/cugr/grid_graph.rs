// SPDX-License-Identifier: Apache-2.0
//! CUGR's gcell graph (`GridGraph`): per layer, one edge per pair of neighbouring gcells along the
//! layer's direction, each with a fractional track CAPACITY and a DEMAND.
//!
//! [`GridGraph::new`] is the constructor's sequence: gcell centres → tracks per gcell row →
//! capacities from tracks → obstacles deducted → the per-layer resource totals → the user's
//! adjustment. Capacities are doubles: an obstacle that blocks part of an edge's span removes that
//! FRACTION of the track, so the sums below must run in the reference's order.
#![allow(clippy::needless_range_loop)] // index loops walk the grids by the coordinates the traces report

use std::collections::BTreeMap;
use std::rc::Rc;

use super::design::Design;
use super::grnet::GrNet;
use super::geo::{BoxT, Interval, Point};
use super::layers::{H, V};

/// `GraphEdge`: `{(l, x, y), (l, x+1, y)}` on a horizontal layer, `{(l, x, y), (l, x, y+1)}` on a
/// vertical one, stored at its lower cell.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GraphEdge {
    pub capacity: f64,
    pub demand: f64,
}

/// A read the reference would make out of range; refused rather than guessed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GridError {
    /// `rangeSearchRows` would index a gridline past either end: a shape wholly outside the die.
    ShapeOutsideDie { dimension: usize, low: i32, high: i32 },
}

#[derive(Debug, Clone, PartialEq)]
pub struct GridGraph {
    pub gridlines: [Vec<i32>; 2],
    pub grid_centers: [Vec<i32>; 2],
    pub layer_names: Vec<String>,
    pub layer_directions: Vec<usize>,
    pub dbu_per_micron: i32,
    pub m2_pitch: i32,
    pub num_layers: usize,
    pub x_size: usize,
    pub y_size: usize,
    /// `Constants::min_routing_layer`, 0-based.
    pub min_routing_layer: usize,
    pub unit_length_wire_cost: f64,
    pub unit_via_cost: f64,
    pub unit_length_short_costs: Vec<f64>,
    /// `[layer][x][y]`.
    pub graph_edges: Vec<Vec<Vec<GraphEdge>>>,
    /// Per layer, the track count of each gcell row (horizontal) or column (vertical).
    pub grid_tracks: Vec<Vec<i32>>,
    /// Per layer, the capacity total after obstacles and BEFORE the user's adjustment, rounded.
    pub original_resources_per_layer: Vec<i32>,
}

impl GridGraph {
    /// The constructor.
    pub fn new(design: &Design, min_routing_layer: usize) -> Result<GridGraph, GridError> {
        let gridlines = design.gridlines.clone();
        let (x_size, y_size) = (gridlines[0].len() - 1, gridlines[1].len() - 1);
        let num_layers = design.num_layers();
        let mut g = GridGraph {
            grid_centers: [Vec::new(), Vec::new()],
            gridlines,
            layer_names: design.layers.iter().map(|l| l.name.clone()).collect(),
            layer_directions: design.layers.iter().map(|l| l.direction).collect(),
            dbu_per_micron: design.dbu_per_micron,
            m2_pitch: design.layers[1].pitch,
            num_layers,
            x_size,
            y_size,
            min_routing_layer,
            unit_length_wire_cost: design.unit_length_wire_cost,
            unit_via_cost: design.unit_via_cost,
            unit_length_short_costs: design.unit_length_short_costs.clone(),
            graph_edges: vec![vec![vec![GraphEdge::default(); y_size]; x_size]; num_layers],
            grid_tracks: vec![Vec::new(); num_layers],
            original_resources_per_layer: Vec::new(),
        };
        g.compute_grid_centers();
        g.init_capacities_from_tracks(design);
        g.deduct_obstacles(design)?;
        g.sum_original_resources();
        g.apply_user_adjustments(design);
        Ok(g)
    }

    /// Each gcell's centre, `(line[i] + line[i + 1]) / 2` in int.
    fn compute_grid_centers(&mut self) {
        for d in 0..2 {
            self.grid_centers[d] = self.gridlines[d].windows(2).map(|w| (w[0] + w[1]) / 2).collect();
        }
    }

    pub fn size(&self, dimension: usize) -> usize {
        if dimension == 0 {
            self.x_size
        } else {
            self.y_size
        }
    }

    /// `getEdgeLength(direction, edge_index)`: centre to centre.
    pub fn edge_length(&self, direction: usize, edge_index: usize) -> i32 {
        self.grid_centers[direction][edge_index + 1] - self.grid_centers[direction][edge_index]
    }

    /// `ref_resistance_`, `ref_via_resistance_`: the first non-zero sheet and via resistance over
    /// the layers in order — the res-aware costs' references (0 when a technology has none).
    pub fn ref_resistances(design: &Design) -> (f64, f64) {
        let first = |f: &dyn Fn(&super::layers::MetalLayer) -> f64| design.layers.iter().map(f).find(|&r| r > 0.0).unwrap_or(0.0);
        (first(&|l| l.resistance), first(&|l| l.via_resistance))
    }

    /// `getWireResistanceCost`: a res-aware net's wire from `u` to `v` on `layer`.
    ///
    /// Upstream rules: the width is the net's NDR width on the layer, else the layer's; with no
    /// width, no reference, or a layer without resistance the cost is 0. The length is the edge
    /// lengths summed in DBU; `R = sheet * length / width`, then `weight * R / ref`, left to right.
    pub fn wire_resistance_cost(&self, design: &Design, c: &super::Constants, layer: usize, u: Point, v: Point, wire_width: i32) -> f64 {
        let l = &design.layers[layer];
        let width = if wire_width > 0 { f64::from(wire_width) } else { f64::from(l.width) };
        let (reference, _) = Self::ref_resistances(design);
        if width <= 0.0 || reference <= 0.0 || l.resistance <= 0.0 {
            return 0.0;
        }
        let direction = self.layer_directions[layer];
        let (lo, hi) = (u.get(direction).min(v.get(direction)), u.get(direction).max(v.get(direction)));
        let length: i32 = (lo..hi).map(|i| self.edge_length(direction, i as usize)).sum();
        let resistance = l.resistance * f64::from(length) / width;
        c.resistance_weight * resistance / reference
    }

    /// `getViaResistanceCost`: one via above `lower_layer`, `weight * R / ref`; 0 without either.
    pub fn via_resistance_cost(&self, design: &Design, c: &super::Constants, lower_layer: usize) -> f64 {
        let (_, reference) = Self::ref_resistances(design);
        let via_r = design.layers[lower_layer].via_resistance;
        if reference <= 0.0 || via_r <= 0.0 {
            return 0.0;
        }
        c.resistance_weight * via_r / reference
    }

    /// `getNetResistance`: a tree's wire and via resistance, summed in preorder, each node's
    /// children in order.
    ///
    /// Upstream rules: a same-layer child is a wire, `sheet * length / width` with the NDR width if
    /// set on that layer (else the layer's; none, skipped); a layer change is the via resistance of
    /// every layer crossed, bottom up. No tree, 0.
    pub fn net_resistance(&self, design: &Design, tree: Option<&GrTree>, ndr_widths: &[i32]) -> f64 {
        let Some(tree) = tree else { return 0.0 };
        let mut total = 0.0;
        for i in tree.preorder() {
            let node = &tree.nodes[i];
            for &c in &node.children {
                let child = &tree.nodes[c];
                if node.layer == child.layer {
                    if node.layer < 0 {
                        continue;
                    }
                    let layer = node.layer as usize;
                    let l = &design.layers[layer];
                    let ndr_width = ndr_widths.get(layer).copied().unwrap_or(0);
                    let width = if ndr_width > 0 { f64::from(ndr_width) } else { f64::from(l.width) };
                    if width <= 0.0 {
                        continue;
                    }
                    let direction = self.layer_directions[layer];
                    let (lo, hi) = (node.p.get(direction).min(child.p.get(direction)), node.p.get(direction).max(child.p.get(direction)));
                    let length: i32 = (lo..hi).map(|e| self.edge_length(direction, e as usize)).sum();
                    total += l.resistance * f64::from(length) / width;
                } else {
                    for l in node.layer.min(child.layer)..node.layer.max(child.layer) {
                        total += design.layers[l as usize].via_resistance;
                    }
                }
            }
        }
        total
    }

    /// `getTreeLength`: the planar length in gcells (vias add nothing). No tree, 0.
    pub fn tree_length(tree: Option<&GrTree>) -> i32 {
        let Some(tree) = tree else { return 0 };
        tree.preorder().into_iter().flat_map(|i| tree.nodes[i].children.iter().map(move |&c| (i, c))).map(|(i, c)| (tree.nodes[i].p.x - tree.nodes[c].p.x).abs() + (tree.nodes[i].p.y - tree.nodes[c].p.y).abs()).sum()
    }

    pub fn edge(&self, layer: usize, x: usize, y: usize) -> &GraphEdge {
        &self.graph_edges[layer][x][y]
    }

    /// Tracks per gcell row/column, then every edge's capacity set to its row's track count.
    ///
    /// Upstream rule: a row's tracks are those `rangeSearchTracks` finds between its two
    /// gridlines, INCLUDING both bounds — then a track exactly on the upper gridline is given to
    /// the next row, except in the last row. Layers below the min routing layer keep their track
    /// counts but capacity 0; the last edge of each run (the one past the last gcell) stays 0.
    fn init_capacities_from_tracks(&mut self, design: &Design) {
        for (l, layer) in design.layers.iter().enumerate() {
            let perp = 1 - layer.direction;
            let n_grids = self.gridlines[perp].len() - 1;
            self.grid_tracks[l] = (0..n_grids)
                .map(|gi| {
                    let loc = Interval::new(self.gridlines[perp][gi], self.gridlines[perp][gi + 1]);
                    let r = layer.range_search_tracks(loc, true);
                    if !r.is_valid() {
                        return 0;
                    }
                    let mut n = r.range() + 1;
                    if gi != n_grids - 1 && layer.track_location(r.high) == loc.high {
                        n -= 1;
                    }
                    n
                })
                .collect();
            if l < self.min_routing_layer {
                continue;
            }
            if layer.direction == V {
                for x in 0..self.x_size {
                    let n = f64::from(self.grid_tracks[l][x]);
                    for y in 0..self.y_size.saturating_sub(1) {
                        self.graph_edges[l][x][y].capacity = n;
                    }
                }
            } else {
                for y in 0..self.y_size {
                    let n = f64::from(self.grid_tracks[l][y]);
                    for x in 0..self.x_size.saturating_sub(1) {
                        self.graph_edges[l][x][y].capacity = n;
                    }
                }
            }
        }
    }

    /// `rangeSearchGridlines`: the gridlines within `[low, high]`, by two `lower_bound`s.
    ///
    /// Upstream rule: `high` is clamped to the last gridline when past the end, else stepped back
    /// one when the gridline found lies above the interval. `low` is not clamped.
    pub fn range_search_gridlines(&self, dimension: usize, loc: Interval) -> Interval {
        let lines = &self.gridlines[dimension];
        let low = lines.partition_point(|&g| g < loc.low) as i32;
        let mut high = lines.partition_point(|&g| g < loc.high) as i32;
        if high as usize >= lines.len() {
            high = lines.len() as i32 - 1;
        } else if lines[high as usize] > loc.high {
            high -= 1;
        }
        Interval::new(low, high)
    }

    /// `rangeSearchRows`: the gcells a DBU interval overlaps.
    ///
    /// Upstream rule: an interval starting exactly on a gridline starts in that gcell, otherwise
    /// in the one before; one ending exactly on a gridline ends in the gcell before it, otherwise
    /// in the gcell of its last gridline, clamped to the last.
    pub fn range_search_rows(&self, dimension: usize, loc: Interval) -> Result<Interval, GridError> {
        let lines = &self.gridlines[dimension];
        let lr = self.range_search_gridlines(dimension, loc);
        let outside = || GridError::ShapeOutsideDie { dimension, low: loc.low, high: loc.high };
        let at_low = *lines.get(lr.low as usize).ok_or_else(outside)?;
        let at_high = *usize::try_from(lr.high).ok().and_then(|h| lines.get(h)).ok_or_else(outside)?;
        Ok(Interval::new(
            if at_low == loc.low { lr.low } else { (lr.low - 1).max(0) },
            if at_high == loc.high { lr.high - 1 } else { lr.high.min(self.size(dimension) as i32 - 1) },
        ))
    }

    /// `rangeSearchCells`.
    pub fn range_search_cells(&self, b: &BoxT) -> Result<BoxT, GridError> {
        Ok(BoxT::from_intervals(self.range_search_rows(0, b.x)?, self.range_search_rows(1, b.y)?))
    }

    /// Obstacles deducted from capacity, per layer from `max(1, min_routing_layer)` (layer 0 is
    /// never deducted: `getAllObstacles(skip_m1)`).
    ///
    /// Upstream rule, per obstacle: enlarge it ACROSS the tracks by the parallel-run spacing for
    /// `(its narrower side, min(shortest edge, its extent along the tracks))` plus half the wire
    /// width minus one; file it under every gcell row that enlarged box overlaps, and under every
    /// edge from two before its first gridline to its last. Per edge, each track the obstacle
    /// covers has its usable span — the edge's centre-to-centre interval — cut: to nothing at the
    /// gridline when the obstacle straddles it, else from the side the obstacle is on. The edge's
    /// capacity is the SUM over its tracks of usable span / edge span, in track order.
    fn deduct_obstacles(&mut self, design: &Design) -> Result<(), GridError> {
        let obstacles = design.all_obstacles(true);
        for l in self.min_routing_layer.max(1)..self.num_layers {
            let layer = &design.layers[l];
            let d = layer.direction;
            let perp = 1 - d;
            let n_grids = self.gridlines[perp].len() - 1;
            let n_edges = self.gridlines[d].len() - 2;
            let min_edge_length = (0..n_edges).map(|e| self.grid_centers[d][e + 1] - self.grid_centers[d][e]).min().unwrap_or(i32::MAX);
            // Per gcell row, the obstacles filed under it (shared, as the reference's shared_ptr).
            let mut obstacles_in_grid: Vec<Vec<Rc<(BoxT, Interval)>>> = vec![Vec::new(); n_grids];
            for obs in &obstacles[l] {
                let width = obs.x.range().min(obs.y.range());
                let spacing = layer.parallel_spacing_for(width, min_edge_length.min(obs.get(d).range())) + layer.width / 2 - 1;
                let mut margin = Point::new(0, 0);
                margin.set(perp, spacing);
                let obs_box = BoxT::new(obs.lx() - margin.x, obs.ly() - margin.y, obs.hx() + margin.x, obs.hy() + margin.y);
                let track_range = layer.range_search_tracks(obs_box.get(perp), true);
                let obstacle = Rc::new((obs_box, track_range));
                let grid_range = self.range_search_rows(perp, obs_box.get(perp))?;
                for gi in grid_range.low..=grid_range.high {
                    obstacles_in_grid[gi as usize].push(Rc::clone(&obstacle));
                }
            }
            let mut grid_track_range = Interval::default();
            for gi in 0..n_grids {
                if gi == 0 {
                    grid_track_range = Interval::new(0, self.grid_tracks[l][0] - 1);
                } else {
                    grid_track_range.low = grid_track_range.high + 1;
                    grid_track_range.high += self.grid_tracks[l][gi];
                }
                if !grid_track_range.is_valid() || obstacles_in_grid[gi].is_empty() {
                    continue;
                }
                let mut obstacles_at_edge: Vec<Vec<Rc<(BoxT, Interval)>>> = vec![Vec::new(); n_edges];
                for obstacle in &obstacles_in_grid[gi] {
                    let gridline_range = self.range_search_gridlines(d, obstacle.0.get(d));
                    let (lo, hi) = ((gridline_range.low - 2).max(0), gridline_range.high.min(n_edges as i32 - 1));
                    for e in lo..=hi {
                        obstacles_at_edge[e as usize].push(Rc::clone(obstacle));
                    }
                }
                for e in 0..n_edges {
                    if obstacles_at_edge[e].is_empty() {
                        continue;
                    }
                    let gridline = self.gridlines[d][e + 1];
                    let edge_interval = Interval::new(self.grid_centers[d][e], self.grid_centers[d][e + 1]);
                    let mut usable = vec![edge_interval; (grid_track_range.range() + 1) as usize];
                    for obstacle in &obstacles_at_edge[e] {
                        let affected = grid_track_range.intersect_with(&obstacle.1);
                        if !affected.is_valid() {
                            continue;
                        }
                        let span = obstacle.0.get(d);
                        for t in affected.low..=affected.high {
                            let u = &mut usable[(t - grid_track_range.low) as usize];
                            if span.low <= gridline && span.high >= gridline {
                                *u = Interval::new(gridline, gridline);
                            } else if span.high < gridline {
                                u.low = u.low.max(span.high);
                            } else if span.low > gridline {
                                u.high = u.high.min(span.low);
                            }
                        }
                    }
                    let mut capacity = 0.0f64;
                    for u in &usable {
                        capacity += f64::from(u.range()) / f64::from(edge_interval.range());
                    }
                    if d == V {
                        self.graph_edges[l][gi][e].capacity = capacity;
                    } else {
                        self.graph_edges[l][e][gi].capacity = capacity;
                    }
                }
            }
        }
        Ok(())
    }

    /// `original_resources_per_layer_`: each layer's capacity total, rounded, BEFORE adjustment.
    /// Summed row by row along the layer's direction, as the reference loops.
    fn sum_original_resources(&mut self) {
        self.original_resources_per_layer = (0..self.num_layers)
            .map(|l| {
                let mut sum = 0.0f64;
                if self.layer_directions[l] == H {
                    for y in 0..self.y_size {
                        for x in 0..self.x_size.saturating_sub(1) {
                            sum += self.graph_edges[l][x][y].capacity;
                        }
                    }
                } else {
                    for x in 0..self.x_size {
                        for y in 0..self.y_size.saturating_sub(1) {
                            sum += self.graph_edges[l][x][y].capacity;
                        }
                    }
                }
                sum.round() as i32
            })
            .collect();
    }

    /// The user's per-layer adjustment, over every edge of the layer.
    ///
    /// Upstream rule: capacity × (1 − adjustment), the float adjustment widened to double; an edge
    /// that had capacity keeps at least one track unless the adjustment is exactly 1.
    fn apply_user_adjustments(&mut self, design: &Design) {
        for (l, layer) in design.layers.iter().enumerate() {
            let adjustment = layer.adjustment;
            if adjustment == 0.0 {
                continue;
            }
            for column in &mut self.graph_edges[l] {
                for edge in column.iter_mut() {
                    let orig_cap = edge.capacity;
                    edge.capacity *= 1.0 - f64::from(adjustment);
                    if orig_cap > 0.0 && adjustment != 1.0 {
                        edge.capacity = edge.capacity.max(1.0);
                    }
                }
            }
        }
    }
}

/// A routing tree (`GRTreeNode`), as an arena: node 0 is the root.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct GrTree {
    pub nodes: Vec<GrTreeNode>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GrTreeNode {
    pub layer: i32,
    pub p: Point,
    pub children: Vec<usize>,
}

impl GrTree {
    pub fn add(&mut self, layer: i32, p: Point) -> usize {
        self.nodes.push(GrTreeNode { layer, p, children: Vec::new() });
        self.nodes.len() - 1
    }
    /// `GRTreeNode::preorder`: a node, then each child's subtree in order.
    pub fn preorder(&self) -> Vec<usize> {
        let mut out = Vec::new();
        if self.nodes.is_empty() {
            return out;
        }
        let mut stack = vec![0];
        while let Some(n) = stack.pop() {
            out.push(n);
            stack.extend(self.nodes[n].children.iter().rev());
        }
        out
    }
}

/// `GridGraph::DemandKind`: what a tree charges an edge for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemandKind {
    /// A wire along the layer's preferred direction.
    Wire,
    /// The pad of an adopted wrong-way wire on its flanking edges.
    WrongWay,
    /// The pad of a via stack on its flanking edges.
    ViaPad,
}

/// `GridGraph::TreeDemand`: one edge charge of a tree — `source` is the wire's lower cell, the
/// wrong-way cell crossed, or the via column's cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TreeDemand {
    pub layer: usize,
    pub edge: Point,
    pub demand: f64,
    pub factor: f64,
    pub kind: DemandKind,
    pub source: Point,
}

/// A native tree the reference would reject (`logger_->error`), refused rather than committed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CommitError {
    /// GRT-0307: a wire below the min routing layer.
    WireBelowMinLayer { layer: i32 },
    /// GRT-1252/1253: a wire whose ends are not aligned across its layer's direction.
    WrongWayWire { layer: i32 },
    /// GRT-1250/1251: a via above the top layer.
    ViaAboveTop { layer: i32 },
}

impl GridGraph {
    /// `logistic(input, slope)`: `1 / (1 + exp(input × slope))`.
    pub fn logistic(input: f64, slope: f64) -> f64 {
        1.0 / (1.0 + (input * slope).exp())
    }

    /// `getWireCost(layer, lower, demand, net_factor)`: one edge at `demand` tracks.
    ///
    /// Upstream rule: the demand's length (`demand × factor × edge length`) at the unit wire
    /// cost, plus the same length at the layer's short cost scaled by the logistic of the edge's
    /// free capacity — or by 1 on an edge with less than one track. A wire (demand ≥ 1) that does
    /// not fit a usable edge pays the congestion gate on top. Each `+=` is evaluated left to right
    /// as the reference writes it; the double sums are order-sensitive.
    pub fn wire_cost_at(&self, layer: usize, lower: Point, demand: f64, net_factor: f64, c: &super::Constants, cost_multiplier: f64) -> f64 {
        let direction = self.layer_directions[layer];
        let edge_length = self.edge_length(direction, lower.get(direction) as usize);
        let demand_length = demand * net_factor * f64::from(edge_length);
        let edge = &self.graph_edges[layer][lower.x as usize][lower.y as usize];
        let mut cost = demand_length * self.unit_length_wire_cost;
        cost += demand_length
            * self.unit_length_short_costs[layer]
            * if edge.capacity < 1.0 { 1.0 } else { GridGraph::logistic(edge.capacity - edge.demand, c.cost_logistic_slope * cost_multiplier) };
        if c.congestion_gate_penalty > 0.0 && demand >= 1.0 && edge.capacity >= 1.0 && edge.capacity - edge.demand < demand * net_factor {
            cost += demand_length * self.unit_length_wire_cost * c.congestion_gate_penalty;
        }
        cost
    }

    /// `getWireCost(layer, u, v, net_factor)`: every edge of a straight wire, one track each, in
    /// ascending order. `None` where the ends are not aligned (GRT-1249).
    pub fn wire_cost(&self, layer: usize, u: Point, v: Point, net_factor: f64, c: &super::Constants, cost_multiplier: f64) -> Option<f64> {
        let direction = self.layer_directions[layer];
        if u.get(1 - direction) != v.get(1 - direction) {
            return None;
        }
        let mut cost = 0.0;
        let (l, h) = (u.get(direction).min(v.get(direction)), u.get(direction).max(v.get(direction)));
        for i in l..h {
            let mut lower = u;
            lower.set(direction, i);
            cost += self.wire_cost_at(layer, lower, 1.0, net_factor, c, cost_multiplier);
        }
        Some(cost)
    }

    /// `forEachFlankEdge(layer, loc)`: the edge below `loc` along the layer's direction and the
    /// edge at it, each where it exists, with their summed length.
    fn flank_edges(&self, layer: usize, loc: Point) -> Vec<(Point, i32)> {
        let direction = self.layer_directions[layer];
        let at = loc.get(direction);
        let mut lower_loc = loc;
        lower_loc.set(direction, at - 1);
        let lower = if at > 0 { self.edge_length(direction, (at - 1) as usize) } else { 0 };
        let higher = if (at as usize) < self.size(direction) - 1 { self.edge_length(direction, at as usize) } else { 0 };
        let sum = lower + higher;
        let mut out = Vec::new();
        if sum == 0 {
            return out;
        }
        if lower > 0 {
            out.push((lower_loc, sum));
        }
        if higher > 0 {
            out.push((loc, sum));
        }
        out
    }

    /// `commit`: demand += demand × factor.
    fn commit(&mut self, layer: usize, lower: Point, demand: f64, net_factor: f64, log: &mut Vec<Commit>) {
        self.graph_edges[layer][lower.x as usize][lower.y as usize].demand += demand * net_factor;
        log.push(Commit { layer, p: lower, delta: demand * net_factor });
    }

    /// `stackPadLength(layer, low, high)`: a via stack's pad on `layer` is the longer of the
    /// via below's upper demand length and the via above's lower one (0 at the stack's ends).
    pub fn stack_pad_length(design: &Design, layer: usize, low: usize, high: usize) -> f64 {
        let upper = if layer > low { design.via_demand_length_upper[layer - 1] } else { 0.0 };
        let lower = if layer < high { design.via_demand_length_lower[layer] } else { 0.0 };
        upper.max(lower)
    }

    /// `forEachPadFlankEdge(layer, low, high, loc, net_costs)`: the stack's pad on `layer`
    /// spread over the layer's flanking edges, `(edge, pad / flank span, the layer's NDR factor)`.
    /// (The reference's developer switch `via_demand_enabled`, on by default, is not modelled.)
    pub fn pad_flank_edges(&self, design: &Design, layer: usize, low: usize, high: usize, loc: Point, net_costs: &[f64]) -> Vec<(Point, f64, f64)> {
        let pad = GridGraph::stack_pad_length(design, layer, low, high);
        let factor = net_costs.get(layer).copied().unwrap_or(1.0);
        self.flank_edges(layer, loc).into_iter().map(|(edge, sum)| (edge, pad / f64::from(sum), factor)).collect()
    }

    /// `forEachViaColumnImpl(tree)`: every layer change as an `(x, y, low, high)` span, sorted,
    /// and spans at one cell merged into one stack wherever they overlap or touch — pattern
    /// routing splits a stack where a child attaches, and an adopted route keeps one edge per via.
    pub fn via_columns(tree: &GrTree) -> Vec<(Point, usize, usize)> {
        let mut spans: Vec<(i32, i32, i32, i32)> = Vec::new();
        for n in tree.preorder() {
            let node = &tree.nodes[n];
            for &ch in &node.children {
                let child = &tree.nodes[ch];
                if node.layer != child.layer {
                    spans.push((node.p.x, node.p.y, node.layer.min(child.layer), node.layer.max(child.layer)));
                }
            }
        }
        spans.sort();
        let mut out = Vec::new();
        let mut i = 0;
        while i < spans.len() {
            let (x, y, low, mut high) = spans[i];
            i += 1;
            while i < spans.len() && spans[i].0 == x && spans[i].1 == y && spans[i].2 <= high {
                high = high.max(spans[i].3);
                i += 1;
            }
            out.push((Point { x, y }, low as usize, high as usize));
        }
        out
    }

    /// `forEachTreeWireEdgeImpl(tree, adopted)`: the wire edges and the wrong-way cells a tree
    /// occupies, each ONCE (sorted by layer, x, y and deduplicated: the union of two merged nets
    /// can cross an edge twice) — wires first, then wrong-way cells. An adopted span below the min
    /// routing layer occupies nothing; a wrong-way span on a native tree, or a diagonal one on any,
    /// is the reference's GRT-1252/1253 error.
    fn tree_wire_edges(&self, tree: &GrTree, adopted: bool) -> Result<(Vec<(i32, i32, i32)>, Vec<(i32, i32, i32)>), CommitError> {
        let (mut wires, mut wrong_way) = (Vec::new(), Vec::new());
        for n in tree.preorder() {
            let node = &tree.nodes[n];
            for &ch in &node.children {
                let child = &tree.nodes[ch];
                if node.layer != child.layer {
                    continue;
                }
                let layer = node.layer as usize;
                if adopted && layer < self.min_routing_layer {
                    continue;
                }
                let direction = self.layer_directions[layer];
                let perp = 1 - direction;
                if node.p.get(perp) != child.p.get(perp) {
                    let diagonal = node.p.get(direction) != child.p.get(direction);
                    if !adopted || diagonal {
                        return Err(CommitError::WrongWayWire { layer: node.layer });
                    }
                    for c in node.p.get(perp).min(child.p.get(perp))..node.p.get(perp).max(child.p.get(perp)) {
                        let mut cell = node.p;
                        cell.set(perp, c);
                        wrong_way.push((node.layer, cell.x, cell.y));
                    }
                } else {
                    for c in node.p.get(direction).min(child.p.get(direction))..node.p.get(direction).max(child.p.get(direction)) {
                        let mut lower = node.p;
                        lower.set(direction, c);
                        wires.push((node.layer, lower.x, lower.y));
                    }
                }
            }
        }
        for cells in [&mut wires, &mut wrong_way] {
            cells.sort();
            cells.dedup();
        }
        Ok((wires, wrong_way))
    }

    /// `forEachTreeDemandImpl(tree, adopted, net_costs)`: everything a tree charges, in the
    /// reference's order — each wire edge (one track), then each wrong-way cell's flank edges
    /// (the layer's wrong-way demand length over the flank span), then each via COLUMN's stack:
    /// on every layer from its low to its high, ONE pad on that layer's flank edges. One walk
    /// serves commit, rip-up and every measure of a tree's demand, as upstream's does.
    pub fn tree_demands(&self, design: &Design, tree: &GrTree, adopted: bool, net_costs: &[f64]) -> Result<Vec<TreeDemand>, CommitError> {
        let (wires, wrong_way) = self.tree_wire_edges(tree, adopted)?;
        let factor_of = |layer: usize| net_costs.get(layer).copied().unwrap_or(1.0);
        let mut out = Vec::new();
        for (layer, x, y) in wires {
            let p = Point { x, y };
            out.push(TreeDemand { layer: layer as usize, edge: p, demand: 1.0, factor: factor_of(layer as usize), kind: DemandKind::Wire, source: p });
        }
        for (layer, x, y) in wrong_way {
            let (layer, cell) = (layer as usize, Point { x, y });
            for (edge, sum) in self.flank_edges(layer, cell) {
                out.push(TreeDemand { layer, edge, demand: design.wrong_way_demand_length[layer] / f64::from(sum), factor: factor_of(layer), kind: DemandKind::WrongWay, source: cell });
            }
        }
        for (loc, low, high) in GridGraph::via_columns(tree) {
            if high >= self.num_layers {
                return Err(CommitError::ViaAboveTop { layer: high as i32 - 1 });
            }
            for l in low..=high {
                for (edge, demand, factor) in self.pad_flank_edges(design, l, low, high, loc, net_costs) {
                    out.push(TreeDemand { layer: l, edge, demand, factor, kind: DemandKind::ViaPad, source: loc });
                }
            }
        }
        Ok(out)
    }

    /// `commitTree(tree, rip_up, net_costs, adopted)`: every charge of `tree_demands`, added (or
    /// removed on a rip-up) at `demand × factor`. A WIRE below the min routing layer is the
    /// reference's GRT-0307 error.
    pub fn commit_tree(&mut self, design: &Design, tree: &GrTree, rip_up: bool, net_costs: &[f64], adopted: bool, log: &mut Vec<Commit>) -> Result<(), CommitError> {
        for charge in self.tree_demands(design, tree, adopted, net_costs)? {
            if charge.kind == DemandKind::Wire && charge.layer < self.min_routing_layer {
                return Err(CommitError::WireBelowMinLayer { layer: charge.layer as i32 });
            }
            self.commit(charge.layer, charge.edge, if rip_up { -charge.demand } else { charge.demand }, charge.factor, log);
        }
        Ok(())
    }

    /// `collectTreeEdges(net)`: every edge the tree charges (`forEachTreeDemandImpl`) on a
    /// routable layer, as `(layer, x, y)`, sorted and unique (`std::array<int, 3>` order). No tree:
    /// none.
    pub fn collect_tree_edges(&self, design: &Design, tree: Option<&GrTree>, adopted: bool, net_costs: &[f64]) -> Result<Vec<(usize, i32, i32)>, CommitError> {
        let Some(tree) = tree else { return Ok(Vec::new()) };
        let mut edges: Vec<(usize, i32, i32)> = self
            .tree_demands(design, tree, adopted, net_costs)?
            .into_iter()
            .filter(|c| c.layer >= self.min_routing_layer)
            .map(|c| (c.layer, c.edge.x, c.edge.y))
            .collect();
        edges.sort_unstable();
        edges.dedup();
        Ok(edges)
    }

    /// `edgeOverflow(edge)`: `std::max(0.0, demand - capacity)` — 0 unless strictly positive.
    pub fn edge_overflow(&self, (layer, x, y): (usize, i32, i32)) -> f64 {
        let e = &self.graph_edges[layer][x as usize][y as usize];
        let over = e.demand - e.capacity;
        if 0.0 < over { over } else { 0.0 }
    }

    /// `sumOverflow(edges)`: the edges' overflows added in the list's order.
    pub fn sum_overflow(&self, edges: &[(usize, i32, i32)]) -> f64 {
        let mut sum = 0.0;
        for &e in edges {
            sum += self.edge_overflow(e);
        }
        sum
    }

    /// `pinAccessTops(net)`: per gcell of the net's preferred access points, the HIGHEST top layer
    /// of the points there, each at least the net's min layer (itself at least the min routing
    /// layer), keyed and ordered by gcell.
    pub fn pin_access_tops(&self, net: &GrNet) -> BTreeMap<(i32, i32), i32> {
        let net_min_layer = (self.min_routing_layer as i32).max(net.layer_range.min_layer);
        let mut tops = BTreeMap::new();
        for (p, layers) in net.preferred_aps.values() {
            let top = layers.high.max(net_min_layer);
            let at = tops.entry((p.x, p.y)).or_insert(top);
            *at = (*at).max(top);
        }
        tops
    }

    /// `reliefScore(net, threshold)`: the overflow the net's removal alone would relieve. Over every
    /// charge of `tree_demands` (wires, wrong-way cells, via-stack pads), the edge's demand above
    /// capacity × threshold, capped at the charge itself. Skipped: charges below the min routing
    /// layer (no capacity) and via pads at a pin gcell at or below that pin's access top (no
    /// reroute moves them). A net with no tree scores 0.
    pub fn relief_score(&self, design: &Design, net: &GrNet, threshold: f64) -> f64 {
        let Some(tree) = &net.routing_tree else { return 0.0 };
        let pin_tops = self.pin_access_tops(net);
        let charges = self.tree_demands(design, tree, net.adopted, &net.ndr_costs).expect("invariant: a routed net's tree was committed, so its demand walk succeeds");
        let mut score = 0.0;
        for charge in charges {
            if charge.layer < self.min_routing_layer {
                continue;
            }
            if charge.kind == DemandKind::ViaPad && pin_tops.get(&(charge.source.x, charge.source.y)).is_some_and(|&top| charge.layer as i32 <= top) {
                continue;
            }
            let edge = &self.graph_edges[charge.layer][charge.edge.x as usize][charge.edge.y as usize];
            let overflow = edge.demand - edge.capacity * threshold;
            if overflow > 0.0 {
                score += (charge.demand * charge.factor).min(overflow);
            }
        }
        score
    }
}

/// A 2D view over both directions: `[direction][x][y]` (`GridGraphView`).
pub type View<T> = Vec<Vec<Vec<T>>>;

/// `GridGraphView<bool>::check(u, v)`: any flagged edge on the straight run from `u` to `v`.
pub fn view_check(view: &View<bool>, u: Point, v: Point) -> bool {
    if u.y == v.y {
        (u.x.min(v.x)..u.x.max(v.x)).any(|x| view[H][x as usize][u.y as usize])
    } else {
        (u.y.min(v.y)..u.y.max(v.y)).any(|y| view[V][u.x as usize][y as usize])
    }
}

/// `GridGraphView<CostT>::sum(u, v)`: the run's edge costs, summed in ascending order from 0.
pub fn view_sum(view: &View<f64>, u: Point, v: Point) -> f64 {
    let mut res = 0.0;
    if u.y == v.y {
        for x in u.x.min(v.x)..u.x.max(v.x) {
            res += view[H][x as usize][u.y as usize];
        }
    } else {
        for y in u.y.min(v.y)..u.y.max(v.y) {
            res += view[V][u.x as usize][y as usize];
        }
    }
    res
}

impl GridGraph {
    /// `extractCongestionView`: per direction, an edge overflowing (`capacity - demand < 0`) on
    /// any routable layer of that direction.
    pub fn extract_congestion_view(&self) -> View<bool> {
        let mut view = vec![vec![vec![false; self.y_size]; self.x_size]; 2];
        for l in self.min_routing_layer..self.num_layers {
            let d = self.layer_directions[l];
            for x in 0..self.x_size {
                for y in 0..self.y_size {
                    let e = &self.graph_edges[l][x][y];
                    if e.capacity - e.demand < 0.0 {
                        view[d][x][y] = true;
                    }
                }
            }
        }
        view
    }

    /// The per-direction layers from the min routing layer, and the smallest short cost among them.
    fn direction_layers(&self, direction: usize) -> (Vec<usize>, f64) {
        let mut layers = Vec::new();
        let mut short = f64::MAX;
        for l in self.min_routing_layer..self.num_layers {
            if self.layer_directions[l] == direction {
                layers.push(l);
                short = short.min(self.unit_length_short_costs[l]);
            }
        }
        (layers, short)
    }

    /// `extractWireCostView(view, net_costs)`: per direction and edge, length × (unit wire cost +
    /// the direction's smallest short cost × the logistic of the free capacity summed over its
    /// layers — or 1 where under one track). An NDR net (a factor > 1 on the direction's layers)
    /// uses the best SINGLE layer's headroom less `(factor - 1)` instead. The last edge of each
    /// run keeps its initial value (`f64::MAX` in a fresh view).
    pub fn extract_wire_cost_view(&self, view: &mut View<f64>, net_costs: &[f64], c: &super::Constants, cost_multiplier: f64) {
        let sized = view.len() == 2 && view[0].len() == self.x_size && (self.x_size == 0 || view[0][0].len() == self.y_size);
        if !sized {
            *view = vec![vec![vec![f64::MAX; self.y_size]; self.x_size]; 2];
        }
        for direction in 0..2 {
            let (layers, short) = self.direction_layers(direction);
            let mut net_factor: f64 = 1.0;
            for &l in &layers {
                if let Some(&f) = net_costs.get(l) {
                    net_factor = net_factor.max(f);
                }
            }
            let ndr_active = net_factor > 1.0;
            for x in 0..self.x_size {
                for y in 0..self.y_size {
                    let edge_index = if direction == H { x } else { y };
                    if edge_index >= self.size(direction) - 1 {
                        continue;
                    }
                    let mut capacity = 0.0;
                    let effective = if ndr_active {
                        let mut best = f64::MIN;
                        for &l in &layers {
                            let e = &self.graph_edges[l][x][y];
                            capacity += e.capacity;
                            best = best.max(e.capacity - e.demand);
                        }
                        best - (net_factor - 1.0)
                    } else {
                        let mut demand = 0.0;
                        for &l in &layers {
                            let e = &self.graph_edges[l][x][y];
                            capacity += e.capacity;
                            demand += e.demand;
                        }
                        capacity - demand
                    };
                    let length = f64::from(self.edge_length(direction, edge_index));
                    view[direction][x][y] = length
                        * (self.unit_length_wire_cost
                            + short * if capacity < 1.0 { 1.0 } else { GridGraph::logistic(effective, c.maze_logistic_slope * cost_multiplier) });
                }
            }
        }
    }

    /// `updateWireCostView(view, net)`: re-price every edge the net's tree (if any) CHARGES
    /// (`forEachTreeDemandImpl`: wire edges, wrong-way flank edges, via-stack pad edges) on a
    /// routable layer, each cell computed as the default (NDR-blind) extraction computes it
    /// (`wireCostViewCell(default_view_context_)`): length × (unit wire cost + the direction's
    /// smallest short cost × the logistic of the direction's summed free capacity, or 1 under one
    /// track).
    pub fn update_wire_cost_view(&self, view: &mut View<f64>, design: &Design, net: &GrNet, c: &super::Constants, cost_multiplier: f64) -> Result<(), CommitError> {
        let Some(tree) = net.routing_tree.as_ref() else { return Ok(()) };
        let dirs = [self.direction_layers(0), self.direction_layers(1)];
        for charge in self.tree_demands(design, tree, net.adopted, &net.ndr_costs)? {
            if charge.layer < self.min_routing_layer {
                continue;
            }
            let direction = self.layer_directions[charge.layer];
            let (x, y) = (charge.edge.x as usize, charge.edge.y as usize);
            let edge_index = if direction == H { x } else { y };
            let (layers, short) = &dirs[direction];
            let (mut capacity, mut demand) = (0.0, 0.0);
            for &l in layers {
                let e = &self.graph_edges[l][x][y];
                capacity += e.capacity;
                demand += e.demand;
            }
            let length = f64::from(self.edge_length(direction, edge_index));
            view[direction][x][y] = length
                * (self.unit_length_wire_cost
                    + short * if capacity < 1.0 { 1.0 } else { GridGraph::logistic(capacity - demand, c.maze_logistic_slope * cost_multiplier) });
        }
        Ok(())
    }

    /// `computeCongestionInformation`'s `cap_per_layer_` (`getTotalCapacityPerLayer`): per layer the
    /// edge capacities summed in the layer's row order and ROUNDED; 0 for a layer below the min
    /// routing layer (skipped there: it holds only pin-access via demand).
    pub fn total_capacity_per_layer(&self) -> Vec<i32> {
        (0..self.num_layers)
            .map(|l| {
                if l < self.min_routing_layer {
                    return 0;
                }
                let mut sum = 0.0f64;
                if self.layer_directions[l] == H {
                    for y in 0..self.y_size {
                        for x in 0..self.x_size.saturating_sub(1) {
                            sum += self.graph_edges[l][x][y].capacity;
                        }
                    }
                } else {
                    for x in 0..self.x_size {
                        for y in 0..self.y_size.saturating_sub(1) {
                            sum += self.graph_edges[l][x][y].capacity;
                        }
                    }
                }
                sum.round() as i32
            })
            .collect()
    }

    /// `reportResources`' rows on the CUGR path: one per grid layer, in layer order —
    /// `getOriginalResources()[l]` (before the user's adjustment) and
    /// [`Self::total_capacity_per_layer`], the direction the tech layer's.
    pub fn resource_rows(&self) -> Vec<crate::driver::ResourceRow<'_>> {
        let derated = self.total_capacity_per_layer();
        (0..self.num_layers)
            .map(|l| crate::driver::ResourceRow { name: &self.layer_names[l], horizontal: self.layer_directions[l] == H, original: self.original_resources_per_layer[l], derated: derated[l] })
            .collect()
    }

    /// `computeCongestionInformation` → `totalOverflow`: per routable layer the overflow
    /// (`demand - capacity` where positive) summed in the layer's row order and ROUNDED, then the
    /// layers added.
    pub fn total_overflow(&self) -> i32 {
        let mut total = 0;
        for l in self.min_routing_layer..self.num_layers {
            let mut sum = 0.0;
            let mut acc = |x: usize, y: usize| {
                let e = &self.graph_edges[l][x][y];
                let overflow = e.demand - e.capacity;
                if overflow > 0.0 {
                    sum += overflow;
                }
            };
            if self.layer_directions[l] == H {
                for y in 0..self.y_size {
                    for x in 0..self.x_size.saturating_sub(1) {
                        acc(x, y);
                    }
                }
            } else {
                for x in 0..self.x_size {
                    for y in 0..self.y_size.saturating_sub(1) {
                        acc(x, y);
                    }
                }
            }
            total += sum.round() as i32;
        }
        total
    }
}

/// One `commit`, as the reference's trace records it: `(layer, lower cell, demand × factor)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Commit {
    pub layer: usize,
    pub p: Point,
    pub delta: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cugr::design::{DesignFacts, Shape, ShapeLayer, TechLayerFacts};
    use crate::cugr::layers::MetalLayerFacts;
    use crate::cugr::Constants;

    fn layer(name: &str, level: i32, horizontal: bool, pitch: i32, first: i32, num: i32, adjustment: f32) -> TechLayerFacts {
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
                tracks: (pitch, first, num),
                area: 0,
                v55_widths_and_lengths: None,
                v55_table: None,
                adjustment,
            }),
        }
    }

    /// A 3×3-gcell die: gridlines 0, 1000, 2000, 3001 (3001, not 3000 — `computeGrid` stops one
    /// gcell early when the die is an exact multiple). m1 H, m2 V, m3 H, pitch 200 from 100.
    fn facts() -> DesignFacts {
        DesignFacts {
            dbu_per_micron: 1000,
            die: (0, 0, 3001, 3001),
            gcell_tile_size: 1000,
            layers: vec![layer("m1", 1, true, 200, 100, 15, 0.0), layer("m2", 2, false, 200, 100, 15, 0.0), layer("m3", 3, true, 200, 100, 15, 0.0)],
            vias: Vec::new(),
            nets: Vec::new(),
            instance_shapes: Vec::new(),
            design_obstructions: Vec::new(),
            min_layer_for_clock: 0,
            max_layer_for_clock: 0,
        }
    }

    fn graph(f: &DesignFacts) -> GridGraph {
        let d = Design::new(f, &Constants::default(), 2, 3).unwrap();
        GridGraph::new(&d, 1).unwrap()
    }

    // Upstream rule (GridGraph ctor): a row's tracks include both gridlines, then a track exactly
    // on the upper gridline goes to the next row — except in the last row. Tracks at 100, 300, …,
    // 2900: 5 per 1000-wide row, none on a gridline here; below the min routing layer capacity is
    // 0 and the edge past the last gcell stays 0.
    #[test]
    fn tracks_per_row_and_capacity() {
        let g = graph(&facts());
        assert_eq!(g.grid_tracks[1], vec![5, 5, 5]);
        assert_eq!(g.edge(1, 0, 0).capacity, 5.0);
        assert_eq!(g.edge(1, 0, 2).capacity, 0.0, "no edge past the last gcell");
        assert_eq!(g.edge(0, 0, 0).capacity, 0.0, "below the min routing layer");
        let mut f = facts();
        f.layers[1].metal.as_mut().unwrap().tracks = (250, 0, 13); // 0, 250, …, 3000
        let g = graph(&f);
        // [0,1000]: 0,250,500,750,1000 -> 5, minus the one ON 1000; [1000,2000]: 1000..2000 -> 4;
        // last [2000,3001]: 2000..3000 -> 5, the last row keeps its upper-bound track.
        assert_eq!(g.grid_tracks[1], vec![4, 4, 5]);
        // The last row's exemption needs a track ON the die's top edge: 1, 251, …, 3001. Rows
        // [0,1000] and [1000,2000] hold 4 each; the last, [2000,3001], holds 2001…3001 = 5 and
        // KEEPS the one on 3001 — every other row would give it away.
        f.layers[1].metal.as_mut().unwrap().tracks = (250, 1, 13);
        assert_eq!(graph(&f).grid_tracks[1], vec![4, 4, 5]);
    }

    // Upstream rule (GridGraph ctor, obstacles): enlarged across the tracks, then each covered
    // track's usable span on an edge is cut — to nothing when the obstacle straddles the edge's
    // gridline. A full-height m2 obstacle x 250..550 — enlarged by parallel spacing 0 (no V55
    // table) + 100/2 - 1 = 49 to 201..599, covering the tracks at 300 and 500 — blocks those two
    // tracks on every edge of column 0: capacity 5 -> 3.
    #[test]
    fn obstacle_straddling_the_gridline_blocks_the_track() {
        let mut f = facts();
        let m2 = Some(ShapeLayer { is_routing: true, routing_level: 2 });
        f.design_obstructions = vec![Shape { layer: m2, rect: (250, 0, 550, 3000) }];
        let g = graph(&f);
        assert_eq!(g.edge(1, 0, 0).capacity, 3.0);
        assert_eq!(g.edge(1, 1, 0).capacity, 5.0, "the next column is untouched");
        assert_eq!(g.original_resources_per_layer[1], 3 * 2 + 5 * 2 + 5 * 2);
    }

    // Upstream rule (GridGraph ctor, obstacles): an obstacle wholly on one side of the gridline cuts
    // the usable span from that side, leaving a FRACTION of the track. Centres are 500, 1500: an m2
    // obstacle y 0..800 (below the 1000 gridline) on the tracks at 300/500 leaves [800, 1500] of
    // [500, 1500] — 0.7 of each — on edge 0, summed in TRACK order (the double sum is order-
    // sensitive): tracks 100, 300, 500, 700, 900.
    #[test]
    fn obstacle_on_one_side_leaves_a_fraction() {
        let mut f = facts();
        let m2 = Some(ShapeLayer { is_routing: true, routing_level: 2 });
        f.design_obstructions = vec![Shape { layer: m2, rect: (250, 0, 550, 800) }];
        let g = graph(&f);
        assert_eq!(g.edge(1, 0, 0).capacity, 0.0 + 1.0 + 0.7 + 0.7 + 1.0 + 1.0);
    }

    // Upstream rule (GridGraph ctor, adjustment): capacity × (1 - adjustment) in double from the
    // float, at least one track where there was capacity unless the adjustment is exactly 1.
    #[test]
    fn user_adjustment_keeps_one_track() {
        let mut f = facts();
        f.layers[1].metal.as_mut().unwrap().adjustment = 0.9;
        let g = graph(&f);
        assert_eq!(g.edge(1, 0, 0).capacity, (5.0 * (1.0 - f64::from(0.9f32))).max(1.0));
        assert_eq!(g.original_resources_per_layer[1], 30, "totals are taken before the adjustment");
        f.layers[1].metal.as_mut().unwrap().adjustment = 1.0;
        assert_eq!(graph(&f).edge(1, 0, 0).capacity, 0.0, "exactly 1 removes everything");
    }

    // Upstream rule (`reportResources` on CUGR: `getOriginalResources` / `getTotalCapacityPerLayer`):
    // original = the capacities BEFORE the user's adjustment; derated = `computeCongestionInformation`'s
    // `cap_per_layer_`, the capacities as init leaves them, summed and ROUNDED per layer, and 0 below
    // the min routing layer whatever its edges hold. m2 (vertical, 6 edges of 5): adjustment 0.9
    // leaves each at least one track: 30 -> 6.
    #[test]
    fn resource_rows_are_original_before_adjustment_and_rounded_capacity_after() {
        let mut f = facts();
        f.layers[1].metal.as_mut().unwrap().adjustment = 0.9;
        let mut g = graph(&f);
        g.graph_edges[0][0][0].capacity = 7.0;
        g.graph_edges[2][0][0].capacity += 0.5;
        let rows = g.resource_rows();
        let got: Vec<(&str, bool, i32, i32)> = rows.iter().map(|r| (r.name, r.horizontal, r.original, r.derated)).collect();
        assert_eq!(got, [("m1", true, 0, 0), ("m2", false, 30, 6), ("m3", true, 30, 31)], "30.5 rounds half away from zero");
    }

    // Upstream rule (CUGR `totalOverflow` ← `computeCongestionInformation`): each layer's overflow
    // is summed and ROUNDED on its own, then the layers are added — 0.4 on m2 and 0.4 on m3 is 0,
    // where rounding the total would give 1.
    #[test]
    fn total_overflow_rounds_per_layer() {
        let mut g = graph(&facts());
        g.graph_edges[1][0][0].demand = 5.4;
        g.graph_edges[2][0][0].demand = 5.4;
        assert_eq!(g.total_overflow(), 0);
        g.graph_edges[1][0][1].demand = 5.2;
        assert_eq!(g.total_overflow(), 1, "0.4 + 0.2 on m2 rounds to 1");
    }

    // Upstream rule (GridGraph `rangeSearchRows`): starting ON a gridline starts in that gcell,
    // ending ON one ends in the gcell before it.
    #[test]
    fn range_search_rows_on_and_off_gridlines() {
        let g = graph(&facts());
        assert_eq!(g.range_search_rows(0, Interval::new(1000, 2000)).unwrap(), Interval::new(1, 1));
        assert_eq!(g.range_search_rows(0, Interval::new(999, 2001)).unwrap(), Interval::new(0, 2));
        assert_eq!(g.range_search_rows(0, Interval::new(3001, 3001)).unwrap(), Interval::new(3, 2), "a point on the die edge is empty");
        assert!(g.range_search_rows(0, Interval::new(3100, 3200)).is_err(), "past the die: the reference reads out of range");
    }

    // Upstream rules (`getWireResistanceCost`, `getNetResistance`): a net's NDR width on a layer
    // replaces the layer's (0 = the layer's), and the references are the FIRST NON-ZERO sheet and
    // via resistance over the layers in order — here m2's, as m1 has none. A via stack adds each
    // crossed layer's via resistance. No net of the stage-2 corpus has an NDR.
    #[test]
    fn via_stack_pads_once_per_layer_and_wires_once_per_net() {
        // Upstream rule (forEachTreeDemandImpl / forEachViaColumnImpl / stackPadLength, at the
        // pin): a tree's charges are its wire edges (sorted, each once), then its wrong-way cells,
        // then each via COLUMN — layer changes at one cell merged where they overlap or touch —
        // charging ONE pad per layer from low to high: max(the via below's upper length, the via
        // above's lower length), spread over the layer's flanking edges.
        let f = facts();
        let mut d = Design::new(&f, &Constants::default(), 2, 3).unwrap();
        d.via_demand_length_lower = vec![10.0, 20.0, 0.0];
        d.via_demand_length_upper = vec![30.0, 40.0, 0.0];
        let g = GridGraph::new(&d, 1).unwrap();
        let c = Point::new(1, 1);
        // A stack 0 -> 1 -> 2 split at layer 1 (two tree edges), then two branches on layer 2
        // that both cross the edge (1,1)-(2,1).
        let mut t = GrTree::default();
        let l0 = t.add(0, c);
        let l1 = t.add(1, c);
        let l2 = t.add(2, c);
        let a = t.add(2, Point::new(2, 1));
        let b = t.add(2, Point::new(2, 1));
        t.nodes[l0].children.push(l1);
        t.nodes[l1].children.push(l2);
        t.nodes[l2].children.extend([a, b]);
        let charges = g.tree_demands(&d, &t, false, &[]).unwrap();
        let wires: Vec<_> = charges.iter().filter(|x| x.kind == DemandKind::Wire).collect();
        assert_eq!(wires.len(), 1, "an edge crossed twice is charged once");
        assert_eq!((wires[0].layer, wires[0].edge), (2, c));
        assert!(charges.iter().position(|x| x.kind == DemandKind::ViaPad) > charges.iter().position(|x| x.kind == DemandKind::Wire), "wires first");
        let pad = |layer: usize| -> Vec<f64> { charges.iter().filter(|x| x.kind == DemandKind::ViaPad && x.layer == layer).map(|x| x.demand).collect() };
        let span = |layer: usize| -> f64 {
            let dir = g.layer_directions[layer];
            f64::from(g.edge_length(dir, 0) + g.edge_length(dir, 1))
        };
        // One column 0..2: layer 0 = its lower 10, layer 1 = max(30, 20) = 30 (not 30 + 20),
        // layer 2 = the via below's upper 40.
        assert_eq!(pad(0), vec![10.0 / span(0); 2]);
        assert_eq!(pad(1), vec![30.0 / span(1); 2]);
        assert_eq!(pad(2), vec![40.0 / span(2); 2]);
        assert_eq!(GridGraph::via_columns(&t), vec![(c, 0, 2)], "the split stack is one column");
        // A rip-up removes exactly what the commit added.
        let span2 = span(2);
        let mut g = g;
        let mut log = Vec::new();
        g.commit_tree(&d, &t, false, &[], false, &mut log).unwrap();
        // The wire's edge is also a flank edge of the column's layer-2 pad.
        assert_eq!(g.edge(2, 1, 1).demand, 1.0 + 40.0 / span2);
        g.commit_tree(&d, &t, true, &[], false, &mut log).unwrap();
        // (Up to float residue: the same adds in the same order, negated, as upstream.)
        assert!(g.graph_edges.iter().flatten().flatten().all(|e| e.demand.abs() < 1e-12));
    }

    #[test]
    fn resistance_takes_the_ndr_width_and_the_first_nonzero_reference() {
        let mut f = facts();
        f.layers[1].metal.as_mut().unwrap().resistance = 2.0;
        f.layers[2].metal.as_mut().unwrap().resistance = 4.0;
        f.layers[1].metal.as_mut().unwrap().via_resistance = 3.0;
        let d = Design::new(&f, &Constants::default(), 2, 3).unwrap();
        let g = GridGraph::new(&d, 1).unwrap();
        let c = Constants::default();
        assert_eq!(GridGraph::ref_resistances(&d), (2.0, 3.0));
        let dir = g.layer_directions[2];
        let len = f64::from(g.edge_length(dir, 0) + g.edge_length(dir, 1));
        let (a, b) = (Point::new(0, 0), Point::new(2, 0));
        assert_eq!(g.wire_resistance_cost(&d, &c, 2, a, b, 0), 50.0 * (4.0 * len / 100.0) / 2.0);
        assert_eq!(g.wire_resistance_cost(&d, &c, 2, a, b, 200), 50.0 * (4.0 * len / 200.0) / 2.0);
        assert_eq!(g.via_resistance_cost(&d, &c, 1), 50.0 * 3.0 / 3.0);
        assert_eq!(g.via_resistance_cost(&d, &c, 0), 0.0, "a layer with no via resistance costs nothing");
        let mut t = GrTree::default();
        let root = t.add(1, a);
        let up = t.add(2, a);
        let far = t.add(2, b);
        t.nodes[root].children.push(up);
        t.nodes[up].children.push(far);
        assert_eq!(g.net_resistance(&d, Some(&t), &[0, 0, 200]), 3.0 + 4.0 * len / 200.0);
        assert_eq!(g.net_resistance(&d, Some(&t), &[]), 3.0 + 4.0 * len / 100.0);
        assert_eq!(GridGraph::tree_length(Some(&t)), 2);
    }
}
