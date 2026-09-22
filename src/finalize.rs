// SPDX-License-Identifier: Apache-2.0
//! Stages R15–R20 — run()'s finalisation after the congestion loop, on the shared router state.
//!
//! R15: `freeRR` (drops the loop's saved routes — held by the loop here, so nothing to do) and
//! `removeLoops` over every net ([`remove_loops`]), then `getOverflow2Dmaze`.
//!
//! R16: `layerAssignment` — `updateSlacks`, the node reset and edge registration, then
//! `layerAssignmentV4` (the net order, the breadth-first walk calling `assignEdge`, the layer
//! extremes) and `ConvertToFull3DType2` — then `getOverflow3D`. Each is one function below, in the
//! reference's order, named after it.

use std::collections::VecDeque;

use crate::brk_rsmt::{NetState, RouteKind, RsmtNet, TreeRoute};
use crate::checks3d::{get_overflow_3d, Cell3D, Overflow3D};
use crate::full3d::{convert_edge_to_full_3d, Edge3D, Point3D, RouteType};
use crate::graph2d::Graph2d;
use crate::layerdp::{assign_edge_layers, LayerDpInputs, LayerEnd};
use crate::layertable::{build_layer_grid, LayerDir, LayerRange, TableInputs};
use crate::maze::remove_loops;
use crate::netpinorder::{netpin_order_inc, NetForOrder};
use crate::spiral::{record_layer_extremes, register_edges, reset_and_alias, reset_for_layer_extremes, EdgeLayers, SpiralNode, LAYER_RESET};

/// `removeLoops` — cut every loop out of every positive-length edge's maze route, giving back the
/// committed usage of the stretch removed (through the NDR-aware charge).
///
/// ⚠️ The route's buffer keeps its size; points past the new `routelen` are stale, as the
/// reference leaves them.
pub fn remove_loops_all(net_ids: &[usize], nets: &[RsmtNet<'_>], state: &mut [NetState], g: &mut Graph2d) -> usize {
    let mut removed = 0;
    for &id in net_ids {
        let nn = nets[id].ndr_net(id);
        let Some(t) = state[id].tree.as_mut() else { continue };
        for eid in 0..t.edges.len() {
            if t.edges[eid].len <= 0 {
                continue;
            }
            let r = &mut t.routes[eid];
            let mut rl = r.routelen.max(0) as usize;
            removed += remove_loops(&mut g.for_net(&nn), &mut r.grids, &mut rl, nn.edge_cost);
            r.routelen = rl as i32;
        }
    }
    removed
}

/// The 3D edges (`h_edges_3D_`, `v_edges_3D_`): capacity and usage per layer, `[layer][y * x_grid + x]`.
///
/// ⛔ Both `uint16_t` in the reference, and usage is charged `usage += int8_t` — a wrap, not a clamp.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Graph3d {
    pub x_grid: usize,
    pub num_layers: usize,
    pub h_cap: Vec<Vec<u16>>,
    pub v_cap: Vec<Vec<u16>>,
    pub h_usage: Vec<Vec<u16>>,
    pub v_usage: Vec<Vec<u16>>,
}

impl Graph3d {
    fn at(&self, x: i32, y: i32) -> usize {
        y as usize * self.x_grid + x as usize
    }
}

/// What the router knows about one net beyond its tree, as layer assignment reads it.
#[derive(Debug, Clone, PartialEq)]
pub struct NetLayerAttrs {
    /// `getPinL()`, indexed by pin.
    pub pin_layers: Vec<i16>,
    /// `getDbNet()->getNonDefaultRule() != nullptr` — the RULE, not the edge cost: a net demoted to
    /// soft NDR keeps it.
    pub has_ndr: bool,
    pub is_clock: bool,
    /// `isResAware()` on entry to layer assignment.
    pub is_res_aware: bool,
    /// `getLayerEdgeCost(l)` for EVERY layer — the per-layer vector, indexed by absolute layer, or 1
    /// throughout for a net without one (or a soft-NDR net).
    pub layer_edge_cost: Vec<i8>,
    /// `getNetSlack` — the timer's slack for the net, which `updateSlacks` writes when it runs.
    pub sta_slack: f32,
}

/// Run-level inputs to R16.
pub struct LayerParams<'a> {
    /// `layer_directions_`.
    pub layer_dir: &'a [LayerDir],
    /// `enable_resistance_aware_`.
    pub resistance_aware: bool,
    /// A liberty library is loaded (`defaultLibertyLibrary() != nullptr`).
    pub liberty: bool,
    /// `has_2D_overflow_`, as the congestion loop left it.
    pub has_2d_overflow: bool,
}

/// `layerAssignment` — the whole of R16 bar the `getOverflow3D` that follows it in run().
///
/// ⚠️ `updateSlacks` runs only on a resistance-aware router with a liberty library, and is wired
/// only as far as its skip rules: a net that survives them needs its resistance read from the
/// database, which this engine does not do, so such a run is refused (see [`update_slacks`]).
pub fn layer_assignment(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState], g3: &mut Graph3d, p: &LayerParams<'_>) -> Result<(), String> {
    update_slacks(net_ids, nets, attrs, state, p)?;
    let nl = g3.num_layers as i16;
    for &id in net_ids {
        let pin_layers = node_pin_layers(id, attrs, state)?;
        let t = state[id].tree.as_mut().ok_or_else(|| format!("net {id}: no tree"))?;
        let coords: Vec<(i16, i16)> = t.nodes.iter().map(|n| (n.x, n.y)).collect();
        t.walk = reset_and_alias(&coords, t.num_terminals, &pin_layers, nl, LAYER_RESET);
    }
    for &id in net_ids {
        let t = state[id].tree.as_mut().expect("reset above");
        let edges: Vec<(usize, usize, i32)> = t.edges.iter().map(|e| (e.n1, e.n2, e.len)).collect();
        t.edge_reg = register_edges(&mut t.walk, &edges);
    }
    layer_assignment_v4(net_ids, nets, attrs, state, g3, p)?;
    convert_to_full_3d_type2(net_ids, state, g3.num_layers);
    Ok(())
}

/// The timer's infinity (`sta::INF`), which an unconstrained net's slack equals exactly.
const STA_INF: f32 = 1e30;

/// `updateSlacks`: every net's slack from the timer, then the resistance-aware marking — through
/// [`crate::slacks::update_slacks`], with the one input it cannot have refused.
///
/// ⛔ A net that survives the skip rules (unconstrained, short, positive slack) has its resistance
/// read from the database (`getNetResistanceOnLayer`) — not wired, so the run is refused rather than
/// scored on a made-up resistance. A net already resistance-aware on entry is refused too: its wire
/// and via costs would be read.
fn update_slacks(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState], p: &LayerParams<'_>) -> Result<(), String> {
    if let Some(&id) = net_ids.iter().find(|&&id| attrs[id].is_res_aware) {
        return Err(format!("net {id} is resistance-aware on entry to layer assignment: not wired"));
    }
    let lens: Vec<Vec<i32>> = net_ids.iter().map(|&id| state[id].tree.as_ref().map(|t| t.edges.iter().map(|e| e.len).collect()).unwrap_or_default()).collect();
    let input: Vec<crate::slacks::NetSlackInput<'_>> = net_ids.iter().zip(&lens).map(|(&id, len)| crate::slacks::NetSlackInput {
        net_id: id,
        slack: attrs[id].sta_slack,
        edge_len: len,
        num_pins: nets[id].pins_x.len() as i32,
        is_clock: attrs[id].is_clock,
        has_ndr: attrs[id].has_ndr,
        is_res_aware: attrs[id].is_res_aware,
        resistance: f32::NAN,
    }).collect();
    let sp = crate::slacks::SlackParams { enabled: p.liberty && p.resistance_aware, is_incremental: false, percentage: 0.0, infinity: STA_INF };
    let up = crate::slacks::update_slacks(&input, &sp);
    if let Some(o) = up.nets.iter().find(|o| o.resistance.is_some()) {
        return Err(format!("updateSlacks: net {} survives the skip rules and needs its resistance from the database: not wired", o.net_id));
    }
    if sp.enabled {
        for o in &up.nets {
            state[o.net_id].slack = o.slack;
        }
    }
    Ok(())
}

/// Each terminal node's pin layer, `getPinL()[node_to_pin_idx[d]]`; Steiner nodes read none.
fn node_pin_layers(id: usize, attrs: &[NetLayerAttrs], state: &[NetState]) -> Result<Vec<i16>, String> {
    let t = state[id].tree.as_ref().ok_or_else(|| format!("net {id}: no tree"))?;
    (0..t.num_terminals)
        .map(|d| {
            let pin = t.node_to_pin_idx[d];
            usize::try_from(pin).ok().and_then(|p| attrs[id].pin_layers.get(p).copied()).ok_or_else(|| format!("net {id}: terminal {d} has no pin ({pin})"))
        })
        .collect()
}

/// `layerAssignmentV4`.
fn layer_assignment_v4(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState], g3: &mut Graph3d, p: &LayerParams<'_>) -> Result<(), String> {
    for &id in net_ids {
        let t = state[id].tree.as_mut().expect("reset");
        for eid in 0..t.edges.len() {
            if t.edges[eid].len > 0 {
                let r = &mut t.routes[eid];
                let n = r.routelen as usize + 1;
                r.grids.resize(n, (0, 0));
                r.layers.resize(n, 0);
                t.edge_reg[eid].assigned = false;
            }
        }
    }
    for o in netpin_order(net_ids, attrs, state, p) {
        let id = o;
        let pin_layers = node_pin_layers(id, attrs, state)?;
        let ns = &mut state[id];
        let t = ns.tree.as_mut().expect("reset");
        let crate::brk_rsmt::StTree { walk, routes, edge_reg, num_terminals, .. } = &mut *t;
        let mut queue: VecDeque<usize> = VecDeque::new();
        for node in 0..*num_terminals {
            for k in 0..walk[node].edges.len() {
                let eid = walk[node].edges[k];
                if !edge_reg[eid].assigned {
                    queue.push_back(eid);
                    edge_reg[eid].assigned = true;
                }
            }
        }
        while let Some(eid) = queue.pop_front() {
            let (n1a, n2a) = edge_reg[eid].alias.expect("a queued edge has aliases");
            let dir = walk[n1a].assigned;
            assign_edge(&nets[id], &attrs[id], &mut ns.layer_range, walk, (n1a, n2a), eid, &mut routes[eid], dir, g3, p)?;
            edge_reg[eid].assigned = true;
            let next = if dir { n2a } else { n1a };
            if !walk[next].assigned {
                for k in 0..walk[next].edges.len() {
                    let e2 = walk[next].edges[k];
                    if !edge_reg[e2].assigned {
                        queue.push_back(e2);
                        edge_reg[e2].assigned = true;
                    }
                }
                walk[next].assigned = true;
            }
        }
        reset_for_layer_extremes(walk, *num_terminals, &pin_layers, g3.num_layers as i16);
        let ends: Vec<EdgeLayers> = t.edges.iter().zip(&t.routes).map(|(e, r)| {
            let (first, last) = if e.len > 0 { (r.layers[0], r.layers[r.routelen as usize]) } else { (0, 0) };
            EdgeLayers { n1: e.n1, n2: e.n2, len: e.len, first, last }
        }).collect();
        record_layer_extremes(&mut t.walk, &ends);
        // The node's status is canonical on the tree; the reset wrote it on the walk.
        for (n, w) in t.nodes.iter_mut().zip(&t.walk) {
            n.status = w.status as _;
        }
    }
    Ok(())
}

/// `netpinOrderInc` over `net_ids`, as net ids in `tree_order_pv_` order.
fn netpin_order(net_ids: &[usize], attrs: &[NetLayerAttrs], state: &[NetState], p: &LayerParams<'_>) -> Vec<usize> {
    let per: Vec<(Vec<i32>, Vec<i16>, i32)> = net_ids.iter().map(|&id| {
        let t = state[id].tree.as_ref().expect("reset");
        (t.edges.iter().map(|e| e.len).collect(), t.edges.iter().map(|e| t.nodes[e.n1].x).collect(), t.num_terminals as i32)
    }).collect();
    let input: Vec<NetForOrder<'_>> = net_ids.iter().zip(&per).map(|(&id, (len, n1x, nt))| NetForOrder {
        net_id: id,
        edge_len: len,
        edge_n1_x: n1x,
        num_terminals: *nt,
        has_ndr: attrs[id].has_ndr,
        is_res_aware: attrs[id].is_res_aware,
        // ⚠️ Read only for a resistance-aware net, which `update_slacks` has refused.
        res_aware_score: 0.0,
        is_clock: attrs[id].is_clock,
    }).collect();
    netpin_order_inc(&input, p.resistance_aware).into_iter().map(|o| o.tree_index).collect()
}

/// `assignEdge` — choose a layer for every point of one edge, from the end `dir` names (the first
/// when true), update the two alias nodes' layer extremes, and charge the 3D usage.
///
/// ⛔ The net's layer range is WIDENED in place when no layer in it has room (and the design has no
/// 2D overflow), and stays widened for every later edge — `range` is that persistent state.
#[allow(clippy::too_many_arguments)]
fn assign_edge(net: &RsmtNet<'_>, attrs: &NetLayerAttrs, range: &mut Option<(usize, usize)>, walk: &mut [SpiralNode], (n1a, n2a): (usize, usize), eid: usize, route: &mut TreeRoute, dir: bool, g3: &mut Graph3d, p: &LayerParams<'_>) -> Result<(), String> {
    let (nl, rl) = (g3.num_layers, route.routelen as usize);
    let (lo, hi) = range.unwrap_or((net.min_layer, net.max_layer));
    let lec: Vec<i32> = attrs.layer_edge_cost[..nl].iter().map(|&c| i32::from(c)).collect();
    // The free resource each step reads, on every layer: capacity less usage at the step's edge.
    let mut vertical = vec![false; rl];
    let mut resources = vec![vec![0i32; rl]; nl];
    for k in 0..rl {
        let (a, b) = (route.grids[k], route.grids[k + 1]);
        vertical[k] = a.0 == b.0;
        for (l, res) in resources.iter_mut().enumerate() {
            res[k] = if vertical[k] {
                let i = g3.at(a.0, a.1.min(b.1));
                i32::from(g3.v_cap[l][i]) - i32::from(g3.v_usage[l][i])
            } else {
                let i = g3.at(a.0.min(b.0), a.1);
                i32::from(g3.h_cap[l][i]) - i32::from(g3.h_usage[l][i])
            };
        }
    }
    let mut widened = LayerRange { min_layer: lo, max_layer: hi };
    let grid = build_layer_grid(&TableInputs {
        num_layers: nl,
        routelen: rl,
        step_is_vertical: &vertical,
        layer_dir: p.layer_dir,
        resources: &resources,
        layer_edge_cost: &lec,
        net_cost: i32::from(net.edge_cost),
        has_2d_overflow: p.has_2d_overflow,
    }, &mut widened);
    if (widened.min_layer, widened.max_layer) != (lo, hi) {
        *range = Some((widened.min_layer, widened.max_layer));
    }
    // ⚠️ Wire and via costs are zero unless the net is resistance-aware, which `update_slacks` refused.
    let edge_cost: Vec<i64> = lec.iter().map(|&c| i64::from(c)).collect();
    let (wire, via) = (vec![0i64; nl], vec![vec![0i64; nl]; nl]);
    let end = |w: &SpiralNode| LayerEnd { assigned: w.assigned, bot_layer: i32::from(w.bot_layer), top_layer: i32::from(w.top_layer) };
    let layers = assign_edge_layers(&LayerDpInputs {
        num_layers: nl,
        routelen: rl,
        min_layer: widened.min_layer,
        max_layer: widened.max_layer,
        layer_grid: &grid,
        edge_cost: &edge_cost,
        wire_cost: &wire,
        via_cost: &via,
    }, dir, end(&walk[n1a]), end(&walk[n2a]));
    route.layers = layers.iter().map(|&l| l as i16).collect();
    let (l0, lr, e) = (route.layers[0], route.layers[rl], eid as i32);
    let widen = |w: &mut SpiralNode, l: i16| {
        if l < w.bot_layer {
            w.bot_layer = l;
            w.l_id = e;
        }
        if l > w.top_layer {
            w.top_layer = l;
            w.h_id = e;
        }
    };
    let pin = |w: &mut SpiralNode, l: i16| {
        w.top_layer = l;
        w.bot_layer = l;
        w.l_id = e;
        w.h_id = e;
    };
    // The fixed end widens; the far end widens if it was assigned, else is pinned to its layer.
    let (fixed, far, lf, lfar) = if dir { (n1a, n2a, l0, lr) } else { (n2a, n1a, lr, l0) };
    widen(&mut walk[fixed], lf);
    if walk[far].assigned {
        widen(&mut walk[far], lfar);
    } else {
        pin(&mut walk[far], lfar);
    }
    if dir && walk[n2a].assigned && (lr > walk[n2a].top_layer || lr < walk[n2a].bot_layer) {
        return Err(format!("GRT-0202: target ending layer ({lr}) out of range"));
    }
    for k in 0..rl {
        let ((x1, y1), (x2, y2), l) = (route.grids[k], route.grids[k + 1], route.layers[k] as usize);
        let (cell, lc) = if x1 == x2 { (&mut g3.v_usage[l], g3.x_grid * y1.min(y2) as usize + x1 as usize) } else { (&mut g3.h_usage[l], g3.x_grid * y1 as usize + x1.min(x2) as usize) };
        cell[lc] = (i32::from(cell[lc]) + lec[l]) as u16;
    }
    Ok(())
}

/// `ConvertToFull3DType2` — insert the via points between consecutive points on different layers.
fn convert_to_full_3d_type2(net_ids: &[usize], state: &mut [NetState], num_layers: usize) {
    for &id in net_ids {
        let t = state[id].tree.as_mut().expect("reset");
        for (e, r) in t.edges.iter().zip(t.routes.iter_mut()) {
            if e.len <= 0 {
                continue;
            }
            let mut e3 = Edge3D {
                len: e.len,
                route_type: RouteType::MazeRoute,
                routelen: r.routelen,
                grids: r.grids.iter().zip(&r.layers).map(|(&(x, y), &l)| Point3D { x: x as i16, y: y as i16, layer: l }).collect(),
            };
            convert_edge_to_full_3d(&mut e3, num_layers);
            r.grids = e3.grids.iter().map(|q| (i32::from(q.x), i32::from(q.y))).collect();
            r.layers = e3.grids.iter().map(|q| q.layer).collect();
            r.routelen = e3.routelen;
            r.kind = RouteKind::MazeRoute;
        }
    }
}

/// `getOverflow3D` — every layer, over the 2D used-grid sets in their set order.
pub fn get_overflow_3d_all(g2d: &Graph2d, g3: &Graph3d) -> Overflow3D {
    let mut cells = Vec::new();
    for l in 0..g3.num_layers {
        for &(x, y) in &g2d.used_h {
            let i = g3.at(x, y);
            cells.push(Cell3D { horizontal: true, usage: i32::from(g3.h_usage[l][i]), capacity: i32::from(g3.h_cap[l][i]) });
        }
        for &(x, y) in &g2d.used_v {
            let i = g3.at(x, y);
            cells.push(Cell3D { horizontal: false, usage: i32::from(g3.v_usage[l][i]), capacity: i32::from(g3.v_cap[l][i]) });
        }
    }
    get_overflow_3d(&cells)
}
