// SPDX-License-Identifier: Apache-2.0
//! R18i — the whole 3D maze pass, assembled: the R18a driver with R18b–h behind its per-edge seam.
//!
//! One edge, in the reference's order: read its original length → rip it up (R18b) → build the
//! search region → seed both frontiers (R18d), recording `corr_edge` → search (R18e) → backtrace
//! (R18f) → either recover (R18h) or perform the tree surgery (R18g), charging usage → hand the
//! retry list back to the driver.
//!
//! ⚠️ State that outlives one edge, and why it is here:
//! - 3D usage (read by the search's admission test, written by rip-up and commit) and 2D usage
//!   (written only — nothing in this pass reads it);
//! - `corr_edge`, which PERSISTS across searches: a cell keeps the last edge any seeding wrote to
//!   it. The surgery only reads cells the same search seeded, so no initial state is needed.
//!
//! ⚠️ The 2D usage change is the net's edge cost. In the reference it goes through the NDR-aware
//! update, measured inert on every rip-up captured; the end-to-end check below is what would show
//! otherwise.

use std::collections::HashMap;

use crate::maze3d::{
    backtrace_3d, maze_route_msmd_order_3d, maze_search_3d, new_ripup_3d_type3,
    recover_edge, setup_heap_3d, tree_surgery_3d, Cell3, EdgeResult, Maze3DCall, Maze3DEdgeWork,
    NodeConnections, OrderedNet, Recovery, Search3DInputs, SeedEdge, SeedNode, Tree3D,
};
use crate::softndr::UsageGrid;

/// The router state the pass reads and writes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PassGrid {
    pub layers: usize,
    pub x_grid: usize,
    pub y_grid: usize,
    /// Per layer: preferred direction horizontal?
    pub horizontal: Vec<bool>,
    /// 3D usage / capacity: horizontal edges `[l][y][x]` with `x < x_grid - 1`, vertical edges
    /// `[l][y][x]` with `y < y_grid - 1`.
    pub h3_usage: Vec<i32>,
    pub h3_cap: Vec<i32>,
    pub v3_usage: Vec<i32>,
    pub v3_cap: Vec<i32>,
    /// 2D usage, same shapes without the layer.
    pub h2_usage: Vec<i32>,
    pub v2_usage: Vec<i32>,
    /// `corr_edge_3D` — only cells some seeding has written.
    pub corr: HashMap<Cell3, usize>,
}

impl PassGrid {
    fn h3(&self, l: usize, y: usize, x: usize) -> usize {
        (l * self.y_grid + y) * (self.x_grid - 1) + x
    }
    fn v3(&self, l: usize, y: usize, x: usize) -> usize {
        (l * (self.y_grid - 1) + y) * self.x_grid + x
    }
}

/// One net as the pass needs it.
#[derive(Debug, Clone, PartialEq)]
pub struct PassNet {
    pub tree: Tree3D,
    pub min_layer: i32,
    pub max_layer: i32,
    pub edge_cost: i8,
    pub layer_cost: Vec<i8>,
    pub slack: f32,
    pub res_aware: bool,
    /// The value `resistance_aware_` holds while this net is routed (the driver sets it from the
    /// net only in a resistance-aware call; otherwise it keeps whatever it held).
    pub effective_resistance_aware: bool,
    /// Move prices: per layer the wire price, and the via price down / up (absent at the ends).
    pub wire: Vec<f32>,
    pub down: Vec<Option<f32>>,
    pub up: Vec<Option<f32>>,
}

/// The call's parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PassParams {
    pub call: Maze3DCall,
    pub expand: i32,
    pub detour_penalty: i32,
}

/// A usage grid adapter charging `sign * cost` — the rip-up gives back, the commit charges.
struct Charge<'a> {
    grid: &'a mut PassGrid,
}

impl UsageGrid for Charge<'_> {
    fn add_usage_v_2d(&mut self, x: i16, y: i16, d: i32) {
        let i = y as usize * self.grid.x_grid + x as usize;
        self.grid.v2_usage[i] += d;
    }
    fn add_usage_h_2d(&mut self, x: i16, y: i16, d: i32) {
        let i = y as usize * (self.grid.x_grid - 1) + x as usize;
        self.grid.h2_usage[i] += d;
    }
    fn add_usage_v_3d(&mut self, l: i16, x: i16, y: i16, d: i32) {
        let i = self.grid.v3(l as usize, y as usize, x as usize);
        self.grid.v3_usage[i] += d;
    }
    fn add_usage_h_3d(&mut self, l: i16, x: i16, y: i16, d: i32) {
        let i = self.grid.h3(l as usize, y as usize, x as usize);
        self.grid.h3_usage[i] += d;
    }
}

/// Charge the usage requests the recovery or the surgery produced: +edge cost in 2D, +layer edge
/// cost in 3D, at each request's cell.
fn charge(grid: &mut PassGrid, net: &PassNet, requests: &[(bool, i16, i16, i16)]) {
    let mut c = Charge { grid };
    for &(horizontal, l, x, y) in requests {
        let lc = i32::from(net.layer_cost[l as usize]);
        let ec = i32::from(net.edge_cost);
        if horizontal {
            c.add_usage_h_2d(x, y, ec);
            c.add_usage_h_3d(l, x, y, lc);
        } else {
            c.add_usage_v_2d(x, y, ec);
            c.add_usage_v_3d(l, x, y, lc);
        }
    }
}

struct PassWork<'a> {
    grid: &'a mut PassGrid,
    nets: &'a mut [PassNet],
    params: PassParams,
}

impl Maze3DEdgeWork for PassWork<'_> {
    fn num_edges(&self, net: usize) -> usize {
        self.nets[net].tree.edges.len()
    }
    fn edge_len(&mut self, net: usize, edge: usize) -> i32 {
        self.nets[net].tree.edges[edge].len
    }
    fn route_edge(&mut self, net_id: usize, edge_id: usize) -> EdgeResult {
        route_one_edge_3d(self.grid, &mut self.nets[net_id], edge_id, self.params)
    }
}

/// One edge's full per-edge work — the body of the driver's edge loop.
pub fn route_one_edge_3d(grid: &mut PassGrid, net: &mut PassNet, edge_id: usize, params: PassParams) -> EdgeResult {
    let t = &mut net.tree;
    let (n1, n2) = (t.edges[edge_id].n1, t.edges[edge_id].n2);
    let (n1x, n1y) = (t.nodes[n1].x, t.nodes[n1].y);
    let (n2x, n2y) = (t.nodes[n2].x, t.nodes[n2].y);
    let original_len = t.edges[edge_id].len;

    // R18b — rip up.
    let mut conns: Vec<NodeConnections> = t.nodes.iter().map(|n| n.conn).collect();
    let (terms, pins) = (t.num_terminals, t.pin_layers.clone());
    let pin_layer = |n: usize| (n < terms).then(|| i32::from(pins[n]));
    let e = t.edges[edge_id].clone();
    let lc = net.layer_cost.clone();
    let ripped = new_ripup_3d_type3(
        edge_id,
        e.len,
        (e.n1a, e.n2a),
        &e.grids,
        e.routelen,
        &mut conns,
        &pin_layer,
        net.edge_cost,
        &|l| lc[l as usize],
        &mut Charge { grid },
    )
    .expect("a captured rip-up never meets a diagonal step");
    for (n, c) in t.nodes.iter_mut().zip(conns) {
        n.conn = c;
    }
    if !ripped {
        return EdgeResult::default();
    }

    // The region: the edge's bounding box grown by min(expand, routelen), clamped to the grid.
    let enlarge = params.expand.min(t.edges[edge_id].routelen);
    let (xmin, xmax) = (i32::from(n1x.min(n2x)), i32::from(n1x.max(n2x)));
    let (ymin, ymax) = (i32::from(n1y.min(n2y)), i32::from(n1y.max(n2y)));
    let region = (
        (xmin - enlarge).max(0),
        (xmax + enlarge).min(grid.x_grid as i32 - 1),
        (ymin - enlarge).max(0),
        (ymax + enlarge).min(grid.y_grid as i32 - 1),
    );
    let (n1a, n2a) = (t.edges[edge_id].n1a, t.edges[edge_id].n2a);

    // R18d — seed, recording corr_edge.
    let seed_nodes: Vec<SeedNode> = t.nodes.iter().map(|n| SeedNode {
        x: n.x,
        y: n.y,
        neighbours: n.nbr.clone(),
        stack_alias: n.stack_alias,
        bot_layer: n.conn.bot_layer,
        top_layer: n.conn.top_layer,
    }).collect();
    let seed_edges: Vec<SeedEdge> = t.edges.iter().map(|e| SeedEdge {
        n1: e.n1,
        n2: e.n2,
        routelen: e.routelen,
        maze_route: e.route_type == crate::full3d::RouteType::MazeRoute,
        grids: e.grids.clone(),
    }).collect();
    let access = if terms == 2 {
        let a = |n: usize| t.pin_layers[t.nodes[n].stack_alias];
        (a(n1), a(n2))
    } else {
        (-1, -1)
    };
    let heaps = setup_heap_3d(terms, &seed_nodes, &seed_edges, edge_id, access, region);
    for &(cell, edge) in &heaps.corr_edge {
        grid.corr.insert(cell, edge);
    }

    // R18e — search.
    let (h3u, h3c, v3u, v3c) = (&grid.h3_usage, &grid.h3_cap, &grid.v3_usage, &grid.v3_cap);
    let admits = |l: i16, x: i32, y: i32| {
        let ec = i32::from(net.layer_cost[l as usize]);
        let (l, x, y) = (l as usize, x as usize, y as usize);
        if grid.horizontal[l] {
            let i = grid.h3(l, y, x);
            h3u[i] + ec <= h3c[i]
        } else {
            let i = grid.v3(l, y, x);
            v3u[i] + ec <= v3c[i]
        }
    };
    let wire = |l: i16| net.wire[l as usize];
    let via = |from: i16, to: i16| {
        if to < from { net.down[from as usize] } else { net.up[from as usize] }.expect("a priced via")
    };
    let inputs = Search3DInputs {
        num_layers: grid.layers as i16,
        region,
        horizontal: &grid.horizontal,
        min_layer: net.min_layer,
        max_layer: net.max_layer,
        admits: &admits,
        wire_cost: &wire,
        via_cost: &via,
        original_len,
        resistance_aware: net.effective_resistance_aware,
        detour_penalty: params.detour_penalty,
    };
    let found = maze_search_3d(&inputs, &heaps.src, &heaps.dest).expect("no GRT-60x in the corpus");

    // R18f — backtrace; R18h — recovery, or R18g — surgery.
    let states: HashMap<Cell3, crate::maze3d::CellState> = found.reached.iter().copied().collect();
    let t = &mut net.tree;
    match backtrace_3d(found.crossing, &|c| states[&c]) {
        Err(why) => {
            let requests = recover_edge(t, edge_id).expect("a recovered edge has length");
            charge(grid, net, &requests);
            EdgeResult { retry: vec![], recovered: counts_for_grt183(why) }
        }
        Ok(bt) => {
            let corr = &grid.corr;
            let lookup = |l: i16, y: i16, x: i16| corr.get(&(l, y, x)).copied().unwrap_or(0);
            let out = tree_surgery_3d(t, edge_id, &bt, ((n1x, n1y), (n2x, n2y)), (n1a, n2a), &lookup)
                .expect("no fatal shift in the corpus");
            let usage = out.usage.clone();
            charge(grid, net, &usage);
            EdgeResult { retry: out.retry, recovered: false }
        }
    }
}

/// Does this recovery count the net for GRT-183?
///
/// ⛔ Only the search running DRY does. The zero-distance crossing recovers the edge too, but the
/// reference's driver reaches `recoverEdge` there without setting `recovered_edge` — 2 such
/// recoveries in the corpus (both on `overlapping_edges`, witnessed by the exhaustive replay).
pub fn counts_for_grt183(why: Recovery) -> bool {
    why == Recovery::Underflow
}

/// The whole pass — `mazeRouteMSMDOrder3D` from its walk onward (the resistance-aware prelude —
/// `updateSlacks`, `netpinOrderInc`, the percentage and detour settings — has already run; the
/// nets arrive in the order it left, and the detour penalty it set is in `params`).
///
/// Returns the number of nets with an edge recovered from a search that ran dry (GRT-183).
pub fn maze_route_3d_pass(params: PassParams, grid: &mut PassGrid, nets: &mut [PassNet]) -> usize {
    let order: Vec<OrderedNet> = nets
        .iter()
        .enumerate()
        .map(|(i, n)| OrderedNet { net_id: i, slack: n.slack, res_aware: n.res_aware })
        .collect();
    let mut work = PassWork { grid, nets, params };
    let (_events, recovered) = maze_route_msmd_order_3d(&params.call, &order, &mut work);
    recovered
}
