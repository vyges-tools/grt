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
use std::collections::{BTreeMap, HashMap};

use crate::checks3d::{check_route_3d, ensure_pin_coverage, get_overflow_3d, three_d_via, Cell3D, Overflow3D, RouteDefect, RoutedEdge, RoutedNode};
use crate::fillvia::{fill_via, ViaEdge, ViaNet, ViaNode, ViaPin};
use crate::routes::{get_routes, GridOrigin, NetForRoutes, RouteEdge};
use crate::estimate::Usage2d;
use crate::maze3d::{prelude, Edge3D as SurgEdge3D, Maze3DCall, Node3D, NodeConnections, Tree3D};
use crate::maze3d_pass::{maze_route_3d_pass, PassGrid, PassNet, PassParams};
use crate::pricing::{get_maze_route_cost_3d, MoveCost, TechLayers, WireNet};
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

/// The resistance-aware router's run-level state: what `preProcessTechLayers` read, and the
/// members `updateSlacks`, `assignEdge` and the 3D pass read and write across calls.
pub struct ResAware<'a> {
    /// `db_layers_` — each routing layer and the cut layer above it.
    pub tech: TechLayers,
    pub tile_size: i32,
    /// `is_fixed_nets_percentage_` — `-res_aware_nets_percentage` was given.
    pub fixed: bool,
    /// `res_aware_nets_percentage_` — rewritten around the calls unless fixed.
    pub percentage: f32,
    /// `worst_*` — what the LAST `updateSlacks` accumulated; `getResAwareScore` divides by it.
    pub worst: crate::slacks::WorstMetrics,
    /// `resistance_aware_` — the member each net sets in `assignEdge` and the 3D pass; never reset.
    pub member: bool,
    /// The timer's slacks at each `updateSlacks` call, and how many calls have read them.
    pub slacks: crate::congestion_loop::TimerSlack<'a>,
    pub calls: usize,
}

/// `kInitialResAwareNetsPercentage`, `kMidResAwareNetsPercentage` (the 3D prelude sets the final).
pub const INITIAL_RES_AWARE_NETS_PERCENTAGE: f32 = 15.0;
pub const MID_RES_AWARE_NETS_PERCENTAGE: f32 = 30.0;

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
    /// The resistance-aware state — `None` when the router is not resistance-aware.
    pub ra: Option<&'a std::cell::RefCell<ResAware<'a>>>,
}

/// `layerAssignment` — the whole of R16 bar the `getOverflow3D` that follows it in run().
///
/// ⚠️ `updateSlacks` runs only on a resistance-aware router with a liberty library, and is wired
/// only as far as its skip rules: a net that survives them needs its resistance read from the
/// database, which this engine does not do, so such a run is refused (see [`update_slacks`]).
/// Returns `tree_order_pv_` as `netpinOrderInc` left it (net ids) — the 3D passes walk it.
pub fn layer_assignment(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState], g3: &mut Graph3d, p: &LayerParams<'_>) -> Result<Vec<usize>, String> {
    // is_3d_step_ false; 15% unless fixed; updateSlacks; 30% unless fixed; is_3d_step_ true.
    set_percentage(p, INITIAL_RES_AWARE_NETS_PERCENTAGE);
    update_slacks(net_ids, nets, attrs, state, p, false)?;
    set_percentage(p, MID_RES_AWARE_NETS_PERCENTAGE);
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
    let order = layer_assignment_v4(net_ids, nets, attrs, state, g3, p)?;
    convert_to_full_3d_type2(net_ids, state, g3.num_layers);
    for &id in net_ids {
        let pin_layers = node_pin_layers(id, attrs, state)?;
        state[id].tree3d = Some(tree_3d(state[id].tree.as_ref().expect("reset"), pin_layers, g3.num_layers));
    }
    Ok(order)
}

/// The timer's infinity (`sta::INF`), which an unconstrained net's slack equals exactly.
const STA_INF: f32 = 1e30;

/// `res_aware_nets_percentage_ = pct` unless `is_fixed_nets_percentage_`.
fn set_percentage(p: &LayerParams<'_>, pct: f32) {
    if let Some(ra) = p.ra {
        let mut ra = ra.borrow_mut();
        if !ra.fixed {
            ra.percentage = pct;
        }
    }
}

/// `getNetResistanceOnLayer` over the net's CURRENT tree: every wire segment on `layer` when given
/// (`is_3d_step_` false: the net's minimum layer), else on its own layer; every layer change a via
/// stack. An edge with no length and no route is skipped.
///
/// ⚠️ Before layer assignment the 2D routes carry no layers: every point reads the same one, so
/// no via is counted.
fn net_resistance_on_layer(id: usize, nets: &[RsmtNet<'_>], state: &[NetState], ra: &ResAware<'_>, layer: Option<i32>) -> f32 {
    let (lo, hi) = state[id].layer_range.unwrap_or((nets[id].min_layer, nets[id].max_layer));
    let wn = WireNet { ndr_width: None, min_layer: lo as i32, max_layer: hi as i32 };
    let wire = |l: i32, seg: i32| crate::pricing::get_wire_resistance(&ra.tech, l, seg * ra.tile_size, wn);
    let mut total = 0.0f32;
    match (&state[id].tree3d, layer) {
        (Some(t), None) => {
            for e in &t.edges {
                if e.len == 0 && e.routelen == 0 {
                    continue;
                }
                for i in 0..e.routelen.max(0) as usize {
                    let (a, b) = (e.grids[i], e.grids[i + 1]);
                    if a.layer == b.layer {
                        let seg = i32::from((a.x - b.x).abs() + (a.y - b.y).abs());
                        total += wire(i32::from(a.layer), seg);
                    } else {
                        total += crate::pricing::get_via_resistance(&ra.tech, i32::from(a.layer), i32::from(b.layer));
                    }
                }
            }
        }
        (_, Some(l)) => {
            let t = state[id].tree.as_ref().expect("a tree");
            for (e, r) in t.edges.iter().zip(&t.routes) {
                if e.len == 0 && r.routelen == 0 {
                    continue;
                }
                for i in 0..r.routelen.max(0) as usize {
                    let (a, b) = (r.grids[i], r.grids[i + 1]);
                    total += wire(l, (a.0 - b.0).abs() + (a.1 - b.1).abs());
                }
            }
        }
        (None, None) => unreachable!("a 3D step reads the 3D tree"),
    }
    total
}

/// `updateSlacks`: every net's slack from the timer, its length, and — for the nets that survive
/// the skip rules — its resistance and the resistance-aware marking, through
/// [`crate::slacks::update_slacks`]. Nets in `net_ids_` order: the worst metrics accumulate in it.
///
/// ⛔ A no-op without a liberty library or with the router not resistance-aware. The slacks are the
/// timer's, captured per call; a call beyond the capture is refused.
fn update_slacks(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState], p: &LayerParams<'_>, is_3d_step: bool) -> Result<(), String> {
    if !(p.liberty && p.resistance_aware) {
        return Ok(());
    }
    if let Some(&id) = net_ids.iter().find(|&&id| attrs[id].is_res_aware) {
        return Err(format!("net {id} is resistance-aware on entry to the run: not wired"));
    }
    let Some(ra_cell) = p.ra else {
        return update_slacks_without_resistances(net_ids, nets, attrs, state);
    };
    let mut ra = ra_cell.borrow_mut();
    let k = ra.calls;
    let slacks = ra.slacks.for_call(k).ok_or_else(|| format!("updateSlacks call {k}: the timer's slacks are not bound"))?;
    ra.calls += 1;
    let lens: Vec<Vec<i32>> = net_ids.iter().map(|&id| match &state[id].tree3d {
        Some(t) => t.edges.iter().map(|e| e.len).collect(),
        None => state[id].tree.as_ref().map(|t| t.edges.iter().map(|e| e.len).collect()).unwrap_or_default(),
    }).collect();
    let resistance: Vec<f32> = net_ids.iter().map(|&id| {
        let layer = if is_3d_step { None } else { Some(state[id].layer_range.map_or(nets[id].min_layer, |r| r.0) as i32) };
        net_resistance_on_layer(id, nets, state, &ra, layer)
    }).collect();
    let input: Vec<crate::slacks::NetSlackInput<'_>> = net_ids.iter().zip(&lens).zip(&resistance).map(|((&id, len), &res)| crate::slacks::NetSlackInput {
        net_id: id,
        slack: slacks[id],
        edge_len: len,
        num_pins: nets[id].pins_x.len() as i32,
        is_clock: attrs[id].is_clock,
        has_ndr: attrs[id].has_ndr,
        is_res_aware: state[id].res_aware,
        resistance: res,
    }).collect();
    let sp = crate::slacks::SlackParams { enabled: true, is_incremental: false, percentage: ra.percentage, infinity: STA_INF };
    let up = crate::slacks::update_slacks(&input, &sp);
    for o in &up.nets {
        let st = &mut state[o.net_id];
        st.slack = o.slack;
        st.net_length = o.net_length;
        if let Some(r) = o.resistance {
            st.resistance = r;
        }
        st.res_aware = o.is_res_aware;
    }
    ra.worst = up.worst;
    Ok(())
}

/// `updateSlacks` for a caller that binds no resistances (the replay): each net's slack from its
/// attributes, and ⛔ refused as soon as a net survives the skip rules — its resistance would be
/// read.
fn update_slacks_without_resistances(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState]) -> Result<(), String> {
    let lens: Vec<Vec<i32>> = net_ids.iter().map(|&id| match &state[id].tree3d {
        Some(t) => t.edges.iter().map(|e| e.len).collect(),
        None => state[id].tree.as_ref().map(|t| t.edges.iter().map(|e| e.len).collect()).unwrap_or_default(),
    }).collect();
    let input: Vec<crate::slacks::NetSlackInput<'_>> = net_ids.iter().zip(&lens).map(|(&id, len)| crate::slacks::NetSlackInput {
        net_id: id,
        slack: attrs[id].sta_slack,
        edge_len: len,
        num_pins: nets[id].pins_x.len() as i32,
        is_clock: attrs[id].is_clock,
        has_ndr: attrs[id].has_ndr,
        is_res_aware: state[id].res_aware,
        resistance: f32::NAN,
    }).collect();
    let sp = crate::slacks::SlackParams { enabled: true, is_incremental: false, percentage: 0.0, infinity: STA_INF };
    let up = crate::slacks::update_slacks(&input, &sp);
    if let Some(o) = up.nets.iter().find(|o| o.resistance.is_some()) {
        return Err(format!("updateSlacks: net {} survives the skip rules and needs its resistance from the database: not wired", o.net_id));
    }
    for o in &up.nets {
        state[o.net_id].slack = o.slack;
        state[o.net_id].net_length = o.net_length;
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
fn layer_assignment_v4(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState], g3: &mut Graph3d, p: &LayerParams<'_>) -> Result<Vec<usize>, String> {
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
    let order = netpin_order(net_ids, nets, attrs, state, p);
    for &id in &order {
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
            assign_edge(&nets[id], &attrs[id], ns.res_aware, &mut ns.layer_range, walk, (n1a, n2a), eid, &mut routes[eid], dir, g3, p)?;
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
    Ok(order)
}

/// `netpinOrderInc` over `net_ids`, as net ids in `tree_order_pv_` order.
fn netpin_order(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &[NetState], p: &LayerParams<'_>) -> Vec<usize> {
    // ⛔ The CURRENT tree: from R16's end the 3D one, which the 3D passes rewrite (R18b's prelude
    // sorts on what R18a's surgery left).
    let per: Vec<(Vec<i32>, Vec<i16>, i32)> = net_ids.iter().map(|&id| match &state[id].tree3d {
        Some(t) => (t.edges.iter().map(|e| e.len).collect(), t.edges.iter().map(|e| t.nodes[e.n1].x).collect(), t.num_terminals as i32),
        None => {
            let t = state[id].tree.as_ref().expect("reset");
            (t.edges.iter().map(|e| e.len).collect(), t.edges.iter().map(|e| t.nodes[e.n1].x).collect(), t.num_terminals as i32)
        }
    }).collect();
    let input: Vec<NetForOrder<'_>> = net_ids.iter().zip(&per).map(|(&id, (len, n1x, nt))| NetForOrder {
        net_id: id,
        edge_len: len,
        edge_n1_x: n1x,
        num_terminals: *nt,
        has_ndr: attrs[id].has_ndr,
        is_res_aware: state[id].res_aware,
        // getResAwareScore: the net's resistance, slack, pins and length over the worst metrics the
        // last updateSlacks left. ⛔ RAW: `netpin_order_inc` applies the reference's negation.
        res_aware_score: match (state[id].res_aware, p.ra) {
            (true, Some(ra)) => {
                let ra = ra.borrow();
                crate::slacks::res_aware_score(&ra.worst, state[id].resistance, state[id].slack, nets[id].pins_x.len() as i32, state[id].net_length)
            }
            _ => 0.0,
        },
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
fn assign_edge(net: &RsmtNet<'_>, attrs: &NetLayerAttrs, res_aware: bool, range: &mut Option<(usize, usize)>, walk: &mut [SpiralNode], (n1a, n2a): (usize, usize), eid: usize, route: &mut TreeRoute, dir: bool, g3: &mut Graph3d, p: &LayerParams<'_>) -> Result<(), String> {
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
    // A resistance-aware router prices this edge for resistance only if the NET is resistance-aware
    // (`resistance_aware_ = net->isResAware()`, a member left as the last net set it). The wire
    // price reads the net's range as widened above.
    let edge_cost: Vec<i64> = lec.iter().map(|&c| i64::from(c)).collect();
    let (mut wire, mut via) = (vec![0i64; nl], vec![vec![0i64; nl]; nl]);
    if let Some(ra) = p.ra {
        let mut ra = ra.borrow_mut();
        if p.resistance_aware {
            ra.member = res_aware;
        }
        if ra.member {
            let wn = WireNet { ndr_width: None, min_layer: widened.min_layer as i32, max_layer: widened.max_layer as i32 };
            for (l, w) in wire.iter_mut().enumerate() {
                *w = i64::from(crate::pricing::get_wire_cost(&ra.tech, true, l as i32, ra.tile_size, wn));
            }
            for (l, row) in via.iter_mut().enumerate() {
                for (i, v) in row.iter_mut().enumerate() {
                    if i != l {
                        *v = i64::from(crate::pricing::get_via_cost(&ra.tech, true, l as i32, i as i32));
                    }
                }
            }
        }
    }
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

/// The net's tree as the 3D passes hold it, from R16's state.
///
/// ⚠️ `heights[k]` is the layer at which the node's `k`-th registered edge meets it — V4 writes
/// `grids[0].layer` at `n1`'s alias and `grids[routelen].layer` at `n2`'s, in registration order,
/// which the full-3D expansion leaves unchanged (it keeps both end points).
/// ⚠️ A zero-length edge's aliases were never written (see [`crate::spiral::EdgeReg`]); the
/// reference reads back its default, 0.
fn tree_3d(t: &crate::brk_rsmt::StTree, pin_layers: Vec<i16>, num_layers: usize) -> Tree3D {
    let layer_at = |e: usize, node: usize| -> i16 {
        let (edge, r) = (&t.edges[e], &t.routes[e]);
        let k = if t.walk[edge.n1].stack_alias == node { 0 } else { r.routelen as usize };
        r.layers.get(k).copied().unwrap_or(0)
    };
    let nodes = t.walk.iter().enumerate().map(|(i, w)| {
        let mut conn = NodeConnections { e_id: [0; crate::spiral::MAX_CONNECTIONS], heights: [0; crate::spiral::MAX_CONNECTIONS], con_cnt: w.edges.len() as i16, bot_layer: w.bot_layer, top_layer: w.top_layer, l_id: w.l_id, h_id: w.h_id };
        for (k, &e) in w.edges.iter().enumerate() {
            conn.e_id[k] = e as i32;
            conn.heights[k] = layer_at(e, i);
        }
        Node3D {
            x: t.nodes[i].x,
            y: t.nodes[i].y,
            stack_alias: w.stack_alias,
            assigned: w.assigned,
            status: w.status,
            conn,
            nbr: (0..t.nbr_count[i]).map(|k| (t.nbr[i][k], t.edge[i][k])).collect(),
        }
    }).collect();
    let edges = t.edges.iter().zip(&t.routes).zip(&t.edge_reg).map(|((e, r), reg)| {
        let (n1a, n2a) = reg.alias.unwrap_or((0, 0));
        SurgEdge3D {
            n1: e.n1,
            n2: e.n2,
            n1a,
            n2a,
            len: e.len,
            route_type: route_type(r.kind),
            routelen: r.routelen,
            grids: r.grids.iter().enumerate().map(|(k, &(x, y))| Point3D { x: x as i16, y: y as i16, layer: r.layers.get(k).copied().unwrap_or(0) }).collect(),
        }
    }).collect();
    Tree3D { num_terminals: t.num_terminals, num_layers: num_layers as i16, pin_layers, nodes, edges }
}

fn route_type(k: RouteKind) -> RouteType {
    match k {
        RouteKind::NoRoute => RouteType::NoRoute,
        RouteKind::LRoute => RouteType::LRoute,
        RouteKind::ZRoute => RouteType::ZRoute,
        RouteKind::MazeRoute => RouteType::MazeRoute,
    }
}

/// Run-level inputs to one `mazeRouteMSMDOrder3D` call (R18).
pub struct Maze3dParams<'a> {
    pub layer: &'a LayerParams<'a>,
    /// `expand` — run()'s local `enlarge_` as the congestion loop left it.
    pub expand: i32,
    /// The rip-up window `ripupTHlb < len < ripupTHub`.
    pub ripup_lb: i32,
    pub ripup_ub: i32,
    /// `via_cost_`, which run() sets to 1 before the 3D passes.
    pub via_cost: i32,
}

/// `mazeRouteMSMDOrder3D` over the shared state: the resistance-aware prelude, then the pass
/// ([`maze_route_3d_pass`]) over `order` (`tree_order_pv_`), its 2D charges replayed through the
/// net's NDR-aware update. Returns the new order (the prelude may re-sort it) and GRT-183's count.
///
/// ⛔ A net routed resistance-aware prices its moves from the technology's resistances — not wired,
/// refused (as is any net `updateSlacks` would keep, as in R16).
pub fn maze_route_msmd_order_3d_all(order: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState], g2d: &mut Graph2d, g3: &mut Graph3d, p: &Maze3dParams<'_>) -> Result<(Vec<usize>, usize), String> {
    let call = Maze3DCall { ripup_lb: p.ripup_lb, ripup_ub: p.ripup_ub, resistance_aware: p.layer.resistance_aware, incremental: false };
    let mut order = order.to_vec();
    // The prelude: updateSlacks (over net_ids_, in id order — the worst metrics accumulate in it),
    // then 100% unless fixed, then netpinOrderInc.
    let fixed = p.layer.ra.is_some_and(|ra| ra.borrow().fixed);
    if call.resistance_aware {
        let mut by_id = order.clone();
        by_id.sort_unstable();
        update_slacks(&by_id, nets, attrs, state, p.layer, true)?;
        if let Some(pct) = prelude(&call, fixed).nets_percentage {
            set_percentage(p.layer, pct);
        }
        order = netpin_order(&by_id, nets, attrs, state, p.layer);
    }
    let detour_penalty = prelude(&call, fixed).detour_penalty.unwrap_or(0);
    let (nl, xg) = (g3.num_layers, g3.x_grid);
    let yg = g3.h_cap[0].len() / xg;
    let flat = |v: &[Vec<u16>], rows: usize, cols: usize| -> Vec<i32> {
        (0..nl).flat_map(|l| (0..rows).flat_map(move |y| (0..cols).map(move |x| (l, y, x)))).map(|(l, y, x)| i32::from(v[l][y * xg + x])).collect()
    };
    let mut grid = PassGrid {
        layers: nl,
        x_grid: xg,
        y_grid: yg,
        horizontal: p.layer.layer_dir.iter().map(|d| *d == LayerDir::Horizontal).collect(),
        h3_usage: flat(&g3.h_usage, yg, xg - 1),
        h3_cap: flat(&g3.h_cap, yg, xg - 1),
        v3_usage: flat(&g3.v_usage, yg - 1, xg),
        v3_cap: flat(&g3.v_cap, yg - 1, xg),
        h2_usage: (0..yg).flat_map(|y| (0..xg - 1).map(move |x| (x, y))).map(|(x, y)| i32::from(g2d.est.usage_h(x, y))).collect(),
        v2_usage: (0..yg - 1).flat_map(|y| (0..xg).map(move |x| (x, y))).map(|(x, y)| i32::from(g2d.est.usage_v(x, y))).collect(),
        corr: HashMap::new(),
        log_2d: Some(Vec::new()),
        current_net: 0,
    };
    let no_tech = TechLayers { dbu_per_micron: 1, width: Vec::new(), resistance: Vec::new(), via_resistance: Vec::new() };
    let ra = p.layer.ra.map(|r| r.borrow());
    let (tech, tile_size) = match &ra {
        Some(ra) => (&ra.tech, ra.tile_size),
        None => (&no_tech, 0),
    };
    let member = ra.as_ref().is_some_and(|ra| ra.member);
    let mut pass_nets = Vec::with_capacity(order.len());
    for &id in &order {
        // resistance_aware_ = net->isResAware() in a resistance-aware call; otherwise it keeps the
        // value it last held. ⚠️ Only the nets the pass walks set it, in its order.
        let effective = if call.resistance_aware { state[id].res_aware } else { member };
        let (lo, hi) = state[id].layer_range.unwrap_or((nets[id].min_layer, nets[id].max_layer));
        let wn = WireNet { ndr_width: None, min_layer: lo as i32, max_layer: hi as i32 };
        let price = |from: usize, to: usize, via: bool| get_maze_route_cost_3d(tech, effective, p.via_cost, tile_size, MoveCost { from_layer: from as i32, to_layer: to as i32, dx: i32::from(!via), dy: 0, is_via: via }, wn);
        pass_nets.push(PassNet {
            tree: state[id].tree3d.take().ok_or_else(|| format!("net {id}: no 3D tree"))?,
            min_layer: lo as i32,
            max_layer: hi as i32,
            edge_cost: nets[id].edge_cost,
            layer_cost: attrs[id].layer_edge_cost[..nl].to_vec(),
            slack: state[id].slack,
            res_aware: state[id].res_aware,
            effective_resistance_aware: effective,
            wire: (0..nl).map(|l| price(l, l, false)).collect(),
            down: (0..nl).map(|l| (l > 0).then(|| price(l, l - 1, true))).collect(),
            up: (0..nl).map(|l| (l + 1 < nl).then(|| price(l, l + 1, true))).collect(),
        });
    }
    drop(ra);
    let recovered = maze_route_3d_pass(PassParams { call, expand: p.expand, detour_penalty }, &mut grid, &mut pass_nets);
    // The member as the pass leaves it: the last net it walked set it (before its slack skip).
    if call.resistance_aware {
        if let (Some(ra), Some(&last)) = (p.layer.ra, order[..crate::maze3d::end_index(order.len(), true)].last()) {
            ra.borrow_mut().member = state[last].res_aware;
        }
    }
    // The 2D charges, in order, through each net's NDR-aware update.
    for &(i, horizontal, x, y, d) in grid.log_2d.as_ref().expect("logging") {
        let nn = nets[order[i]].ndr_net(order[i]);
        let mut u = g2d.for_net(&nn);
        if horizontal { u.update_usage_h(i32::from(x), i32::from(y), f64::from(d)) } else { u.update_usage_v(i32::from(x), i32::from(y), f64::from(d)) }
    }
    for l in 0..nl {
        for y in 0..yg {
            for x in 0..xg {
                if x + 1 < xg {
                    g3.h_usage[l][y * xg + x] = grid.h3_usage[(l * yg + y) * (xg - 1) + x] as u16;
                }
                if y + 1 < yg {
                    g3.v_usage[l][y * xg + x] = grid.v3_usage[(l * (yg - 1) + y) * xg + x] as u16;
                }
            }
        }
    }
    for (&id, pn) in order.iter().zip(pass_nets) {
        state[id].tree3d = Some(pn.tree);
    }
    Ok((order, recovered))
}

/// What R19 leaves besides the rewritten trees.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Finish3d {
    /// `getOverflow3D` after `fillVIA`: the overflow it stores and the usage it returns
    /// (`finallength`).
    pub overflow: Overflow3D,
    /// `threeDVIA`.
    pub num_via: i32,
    /// `checkRoute3D`'s non-fatal findings (the reference prints them only under a debug flag).
    pub defects: Vec<(usize, RouteDefect)>,
    /// Via stacks `ensurePinCoverage` appended.
    pub pin_stacks: usize,
}

/// R19 — `fillVIA`, `getOverflow3D`, `threeDVIA`, `checkRoute3D`, `ensurePinCoverage`, in run()'s
/// order, over the 3D trees.
///
/// ⛔ `checkRoute3D` raises two defects as errors (a floating pin, a negative layer) — returned as
/// `Err` here, as is `fillVIA`'s "no previous routing".
pub fn finish_3d(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState], g2d: &Graph2d, g3: &Graph3d) -> Result<Finish3d, String> {
    fill_via_all(net_ids, nets, attrs, state, g3.num_layers)?;
    let overflow = get_overflow_3d_all(g2d, g3);
    let num_via = three_d_via_all(net_ids, state);
    let defects = check_route_3d_all(net_ids, state)?;
    let pin_stacks = ensure_pin_coverage_all(net_ids, state, g3.num_layers);
    Ok(Finish3d { overflow, num_via, defects, pin_stacks })
}

/// `fillVIA` — the via stacks at every node, through [`fill_via`].
fn fill_via_all(net_ids: &[usize], nets: &[RsmtNet<'_>], attrs: &[NetLayerAttrs], state: &mut [NetState], num_layers: usize) -> Result<(), String> {
    let mut via_nets: Vec<ViaNet> = net_ids.iter().map(|&id| {
        let t = state[id].tree3d.as_ref().expect("a 3D tree");
        ViaNet {
            num_terminals: t.num_terminals,
            nodes: t.nodes.iter().map(|n| ViaNode { x: n.x, y: n.y, bot_layer: n.conn.bot_layer, top_layer: n.conn.top_layer, h_id: n.conn.h_id, l_id: n.conn.l_id, stack_alias: n.stack_alias }).collect(),
            edges: t.edges.iter().map(|e| ViaEdge { len: e.len, n1: e.n1, n2: e.n2, n1a: e.n1a, n2a: e.n2a, route_type: e.route_type, routelen: e.routelen, grids: e.grids.clone() }).collect(),
            pins: nets[id].pins_x.iter().zip(nets[id].pins_y).zip(&attrs[id].pin_layers).map(|((&x, &y), &l)| ViaPin { x, y, layer: i32::from(l) }).collect(),
        }
    }).collect();
    fill_via(&mut via_nets, num_layers as i16).map_err(|e| format!("net {}: edge {} has no previous routing", net_ids[e.net], e.edge))?;
    for (&id, vn) in net_ids.iter().zip(via_nets) {
        let t = state[id].tree3d.as_mut().expect("a 3D tree");
        for (e, ve) in t.edges.iter_mut().zip(vn.edges) {
            (e.route_type, e.routelen, e.grids) = (ve.route_type, ve.routelen, ve.grids);
        }
    }
    Ok(())
}

fn routed_edges(t: &Tree3D) -> Vec<RoutedEdge> {
    t.edges.iter().map(|e| RoutedEdge { len: e.len, routelen: e.routelen, n1: e.n1, n2: e.n2, grids: e.grids.clone() }).collect()
}

fn routed_nodes(t: &Tree3D) -> Vec<RoutedNode> {
    t.nodes.iter().enumerate().map(|(i, n)| RoutedNode { x: n.x, y: n.y, bot_layer: n.conn.bot_layer, top_layer: n.conn.top_layer, pin_layer: (i < t.num_terminals).then(|| t.pin_layers[i]) }).collect()
}

/// `threeDVIA` — the via count over every net.
fn three_d_via_all(net_ids: &[usize], state: &[NetState]) -> i32 {
    net_ids.iter().map(|&id| three_d_via(&routed_edges(state[id].tree3d.as_ref().expect("a 3D tree")))).sum()
}

/// `checkRoute3D` — every net's routing checked; the two fatal defects refused.
fn check_route_3d_all(net_ids: &[usize], state: &[NetState]) -> Result<Vec<(usize, RouteDefect)>, String> {
    let mut out = Vec::new();
    for &id in net_ids {
        let t = state[id].tree3d.as_ref().expect("a 3D tree");
        for d in check_route_3d(&routed_nodes(t), &routed_edges(t)) {
            if matches!(d, RouteDefect::FloatingPin { .. } | RouteDefect::NegativeLayer { .. }) {
                return Err(format!("checkRoute3D: net {id}: {d:?}"));
            }
            out.push((id, d));
        }
    }
    Ok(out)
}

/// `ensurePinCoverage` — a via stack appended as a new edge (a default `TreeEdge`: both ends node 0,
/// length 0, a maze route) for every terminal its routing does not reach.
pub fn ensure_pin_coverage_all(net_ids: &[usize], state: &mut [NetState], num_layers: usize) -> usize {
    let mut added = 0;
    for &id in net_ids {
        let t = state[id].tree3d.as_mut().expect("a 3D tree");
        let terminals: Vec<RoutedNode> = routed_nodes(t).into_iter().take(t.num_terminals).collect();
        for s in ensure_pin_coverage(&terminals, &routed_edges(t), num_layers as i16) {
            t.edges.push(SurgEdge3D { n1: 0, n2: 0, n1a: 0, n2a: 0, len: 0, route_type: RouteType::MazeRoute, routelen: s.routelen, grids: s.grids });
            added += 1;
        }
    }
    added
}

/// R20 — `getRoutes`: every net's 3D tree as database-unit segments, through
/// [`crate::routes::get_routes`], keyed by `db_id[net]` (the net's database id).
pub fn get_routes_all(net_ids: &[usize], state: &[NetState], db_id: &[u32], origin: GridOrigin) -> BTreeMap<u32, Vec<crate::GSegment>> {
    let nets: Vec<NetForRoutes> = net_ids.iter().map(|&id| {
        let t = state[id].tree3d.as_ref().expect("a 3D tree");
        NetForRoutes { db_id: db_id[id], edges: t.edges.iter().map(|e| RouteEdge { len: e.len, routelen: e.routelen, grids: e.grids.clone() }).collect() }
    }).collect();
    get_routes(&nets, origin)
}
