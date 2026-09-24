// SPDX-License-Identifier: Apache-2.0
//! CUGR's gcell graph (`GridGraph`): per layer, one edge per pair of neighbouring gcells along the
//! layer's direction, each with a fractional track CAPACITY and a DEMAND.
//!
//! [`GridGraph::new`] is the constructor's sequence: gcell centres → tracks per gcell row →
//! capacities from tracks → obstacles deducted → the per-layer resource totals → the user's
//! adjustment. Capacities are doubles: an obstacle that blocks part of an edge's span removes that
//! FRACTION of the track, so the sums below must run in the reference's order.

use std::rc::Rc;

use super::design::Design;
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

    /// `forEachViaFlankEdgeImpl(layer_index, loc, net_costs)`: a via between `layer_index` and the
    /// layer above deposits, on each of the two layers' flank edges, that layer's via demand length
    /// spread over the flank span: `(layer, edge, demand, the layer's NDR factor)`.
    pub fn via_flank_edges(&self, layer_index: usize, loc: Point, via_demand_lower: f64, via_demand_upper: f64, net_costs: &[f64]) -> Vec<(usize, Point, f64, f64)> {
        let mut out = Vec::new();
        for l in layer_index..=(layer_index + 1).min(self.num_layers - 1) {
            let factor = net_costs.get(l).copied().unwrap_or(1.0);
            let length = if l == layer_index { via_demand_lower } else { via_demand_upper };
            for (edge, sum) in self.flank_edges(l, loc) {
                out.push((l, edge, if sum > 0 { length / f64::from(sum) } else { 0.0 }, factor));
            }
        }
        out
    }

    /// `getViaCost(layer_index, loc, net_costs)`: the unit via cost times the larger NDR factor of
    /// the two layers, plus the wire cost of every flank deposit at its demand.
    pub fn via_cost(&self, design: &Design, layer_index: usize, loc: Point, net_costs: &[f64], c: &super::Constants, cost_multiplier: f64) -> f64 {
        let lower = net_costs.get(layer_index).copied().unwrap_or(1.0);
        let upper = net_costs.get(layer_index + 1).copied().unwrap_or(1.0);
        let mut cost = self.unit_via_cost * lower.max(upper);
        for (l, edge, demand, factor) in self.via_flank_edges(layer_index, loc, design.via_demand_length_lower[layer_index], design.via_demand_length_upper[layer_index], net_costs) {
            cost += self.wire_cost_at(l, edge, demand, factor, c, cost_multiplier);
        }
        cost
    }

    /// `commit`: demand += demand × factor.
    fn commit(&mut self, layer: usize, lower: Point, demand: f64, net_factor: f64, log: &mut Vec<Commit>) {
        self.graph_edges[layer][lower.x as usize][lower.y as usize].demand += demand * net_factor;
        log.push(Commit { layer, p: lower, delta: demand * net_factor });
    }

    /// `commitTree(tree, rip_up, net_costs, adopted)`, in preorder — each same-layer child a wire
    /// (one track per edge crossed), each layer change a via per layer crossed at the parent's
    /// cell.
    ///
    /// Upstream rule for an ADOPTED tree (restored from a route): a span below the min routing
    /// layer commits nothing, and a WRONG-WAY span deposits, per cell it crosses, the layer's
    /// wrong-way demand length over that cell's flank edges (`commitWrongWayWire`). A native tree
    /// with either is an error, as is a diagonal span on any tree.
    pub fn commit_tree(&mut self, design: &Design, tree: &GrTree, rip_up: bool, net_costs: &[f64], adopted: bool, log: &mut Vec<Commit>) -> Result<(), CommitError> {
        let sign = if rip_up { -1.0 } else { 1.0 };
        for n in tree.preorder() {
            let node = &tree.nodes[n];
            for &ch in &node.children {
                let child = &tree.nodes[ch];
                if node.layer == child.layer {
                    // Wrong way first; GRT-0307 per edge committed (`commitWire`), so a
                    // zero-length wire never trips it.
                    let layer = node.layer as usize;
                    if adopted && layer < self.min_routing_layer {
                        continue;
                    }
                    let direction = self.layer_directions[layer];
                    let perp = 1 - direction;
                    let factor = net_costs.get(layer).copied().unwrap_or(1.0);
                    if node.p.get(perp) != child.p.get(perp) {
                        let diagonal = node.p.get(direction) != child.p.get(direction);
                        if !adopted || diagonal {
                            return Err(CommitError::WrongWayWire { layer: node.layer });
                        }
                        for c in node.p.get(perp).min(child.p.get(perp))..node.p.get(perp).max(child.p.get(perp)) {
                            let mut cell = node.p;
                            cell.set(perp, c);
                            let length = design.wrong_way_demand_length[layer];
                            for (edge, sum) in self.flank_edges(layer, cell) {
                                self.commit(layer, edge, sign * (length / f64::from(sum)), factor, log);
                            }
                        }
                        continue;
                    }
                    let (lo, hi) = (node.p.get(direction).min(child.p.get(direction)), node.p.get(direction).max(child.p.get(direction)));
                    for i in lo..hi {
                        if layer < self.min_routing_layer {
                            return Err(CommitError::WireBelowMinLayer { layer: node.layer });
                        }
                        let mut lower = node.p;
                        lower.set(direction, i);
                        self.commit(layer, lower, sign, factor, log);
                    }
                } else {
                    for l in node.layer.min(child.layer)..node.layer.max(child.layer) {
                        let l = l as usize;
                        if l + 1 >= self.num_layers {
                            return Err(CommitError::ViaAboveTop { layer: l as i32 });
                        }
                        for (fl, edge, demand, factor) in self.via_flank_edges(l, node.p, design.via_demand_length_lower[l], design.via_demand_length_upper[l], net_costs) {
                            self.commit(fl, edge, sign * demand, factor, log);
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// `checkCongestion(tree, threshold)`: the tree's wire edges with demand > capacity × threshold.
    pub fn check_congestion(&self, tree: &GrTree, threshold: f64) -> usize {
        let mut num = 0;
        for n in tree.preorder() {
            let node = &tree.nodes[n];
            for &ch in &node.children {
                let child = &tree.nodes[ch];
                if node.layer != child.layer {
                    continue;
                }
                let layer = node.layer as usize;
                let d = self.layer_directions[layer];
                let (lo, hi) = (node.p.get(d).min(child.p.get(d)), node.p.get(d).max(child.p.get(d)));
                let r = node.p.get(1 - d);
                for c in lo..hi {
                    let (x, y) = if d == H { (c, r) } else { (r, c) };
                    let e = &self.graph_edges[layer][x as usize][y as usize];
                    if e.demand > e.capacity * threshold {
                        num += 1;
                    }
                }
            }
        }
        num
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
                    let effective;
                    if ndr_active {
                        let mut best = f64::MIN;
                        for &l in &layers {
                            let e = &self.graph_edges[l][x][y];
                            capacity += e.capacity;
                            best = best.max(e.capacity - e.demand);
                        }
                        effective = best - (net_factor - 1.0);
                    } else {
                        let mut demand = 0.0;
                        for &l in &layers {
                            let e = &self.graph_edges[l][x][y];
                            capacity += e.capacity;
                            demand += e.demand;
                        }
                        effective = capacity - demand;
                    }
                    let length = f64::from(self.edge_length(direction, edge_index));
                    view[direction][x][y] = length
                        * (self.unit_length_wire_cost
                            + short * if capacity < 1.0 { 1.0 } else { GridGraph::logistic(effective, c.maze_logistic_slope * cost_multiplier) });
                }
            }
        }
    }

    /// `updateWireCostView(view, tree)`: re-price (the summed, NDR-blind way) every edge the tree's
    /// wires cross, and at each via, per layer crossed, the edge at the cell and the one before it
    /// along that layer's direction.
    pub fn update_wire_cost_view(&self, view: &mut View<f64>, tree: &GrTree, c: &super::Constants, cost_multiplier: f64) {
        let dirs = [self.direction_layers(0), self.direction_layers(1)];
        let update = |view: &mut View<f64>, direction: usize, x: i32, y: i32| {
            let edge_index = if direction == H { x } else { y } as usize;
            if edge_index >= self.size(direction) - 1 {
                return;
            }
            let (layers, short) = &dirs[direction];
            let (mut capacity, mut demand) = (0.0, 0.0);
            for &l in layers {
                let e = &self.graph_edges[l][x as usize][y as usize];
                capacity += e.capacity;
                demand += e.demand;
            }
            let length = f64::from(self.edge_length(direction, edge_index));
            view[direction][x as usize][y as usize] = length
                * (self.unit_length_wire_cost
                    + short * if capacity < 1.0 { 1.0 } else { GridGraph::logistic(capacity - demand, c.maze_logistic_slope * cost_multiplier) });
        };
        for n in tree.preorder() {
            let node = &tree.nodes[n];
            for &ch in &node.children {
                let child = &tree.nodes[ch];
                if node.layer == child.layer {
                    let direction = self.layer_directions[node.layer as usize];
                    if direction == H {
                        for x in node.p.x.min(child.p.x)..node.p.x.max(child.p.x) {
                            update(view, direction, x, node.p.y);
                        }
                    } else {
                        for y in node.p.y.min(child.p.y)..node.p.y.max(child.p.y) {
                            update(view, direction, node.p.x, y);
                        }
                    }
                } else {
                    for l in node.layer.min(child.layer)..node.layer.max(child.layer) {
                        let direction = self.layer_directions[l as usize];
                        update(view, direction, node.p.x, node.p.y);
                        if node.p.get(direction) > 0 {
                            update(view, direction, node.p.x - 1 + direction as i32, node.p.y - direction as i32);
                        }
                    }
                }
            }
        }
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
}
