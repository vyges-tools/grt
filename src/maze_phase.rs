// SPDX-License-Identifier: Apache-2.0
//! Stages R11–R13 — from pattern routing to the maze phase: `convertToMazeroute`, the three
//! monotonic rounds (`routeMonotonicAll`), and the resets before the congestion loop.
//!
//! The rules are elsewhere — [`convert_to_mazeroute`], [`needs_ripup_check`], [`monotonic_box`],
//! [`monotonic_search`], [`walk_monotonic_route`], [`monotonic_cost_table`]. This module is the call
//! sequence over them on the shared router state, in the reference's order.

use crate::brk_rsmt::{BrkGrid, NetState, RouteKind, RsmtNet};
use crate::estimate::{check_2d_edges_usage, LShape, UsageViolation};
use crate::graph2d::Graph2d;
use crate::mazeconv::{convert_to_mazeroute, SymbolicRoute};
use crate::monotonic::{monotonic_box, monotonic_cost_table, monotonic_search, walk_monotonic_route};
use crate::overflow2d::Overflow2DScan;
use crate::ripup_route::{give_back_committed, needs_ripup_check};

/// `convertToMazeroute` — walk every edge's L or Z out into grid points, fold the estimate into the
/// committed usage (`addEstUsageToUsage`), then `check2DEdgesUsage`.
///
/// ⚠️ An edge's length is recomputed as its Manhattan distance; a `NoRoute` edge becomes a one-point
/// maze route of length 0.
pub fn convert_to_mazeroute_all(net_ids: &[usize], state: &mut [NetState], g: &mut Graph2d, h_capacity: i32, v_capacity: i32) -> Vec<UsageViolation> {
    for &id in net_ids {
        let t = state[id].tree.as_mut().expect("every routed net has a tree");
        for eid in 0..t.edges.len() {
            let e = t.edges[eid];
            let (a, b) = (&t.nodes[e.n1], &t.nodes[e.n2]);
            let r = &mut t.routes[eid];
            let shape = match r.kind {
                RouteKind::NoRoute => SymbolicRoute::NoRoute,
                RouteKind::LRoute => SymbolicRoute::L(if r.x_first { LShape::XFirst } else { LShape::YFirst }),
                RouteKind::ZRoute => SymbolicRoute::Z { hvh: r.hvh, z_point: r.z_point as i32 },
                RouteKind::MazeRoute => unreachable!("convertToMazeroute runs once, on pattern routes"),
            };
            let m = convert_to_mazeroute((a.x as i32, a.y as i32), (b.x as i32, b.y as i32), e.len, shape);
            t.edges[eid].len = m.len;
            r.kind = RouteKind::MazeRoute;
            r.grids = m.grids.iter().map(|p| (p.x, p.y)).collect();
            r.routelen = m.routelen;
        }
    }
    g.est.add_est_usage_to_usage();
    check_2d_edges_usage(&g.est, h_capacity, v_capacity)
}

/// `routeMonotonic(net, edge, threshold, enlarge)` — rip up a congested maze route and re-route it
/// monotonically inside a box around the edge.
///
/// ⛔ The gate is `newRipupCheck(…, ripup_threshold = threshold, critical_slack = 0, …)`: the
/// monotonic threshold doubles as the rip-up threshold, and a zero critical slack turns the
/// critical-net arm off. ⛔ The search reads the demand AFTER the rip-up.
#[allow(clippy::too_many_arguments)]
pub fn route_monotonic_edge(
    id: usize,
    eid: usize,
    net: &RsmtNet<'_>,
    st: &mut NetState,
    grid: &mut BrkGrid<'_>,
    cost_table: &[f64],
    threshold: i32,
    enlarge: i32,
) {
    let t = st.tree.as_mut().expect("tree");
    let e = t.edges[eid];
    if e.len <= threshold || e.len == 0 {
        return;
    }
    let (red_h, red_v) = (grid.red_h, grid.red_v);
    let g = &*grid.g;
    let used_h = |x: i32, y: i32| f64::from(g.usage_red_h(x, y, red_h(x as usize, y as usize)));
    let used_v = |x: i32, y: i32| f64::from(g.usage_red_v(x, y, red_v(x as usize, y as usize)));
    let caps = (grid.h_capacity, grid.v_capacity);
    let r = &t.routes[eid];
    assert_eq!(r.kind, RouteKind::MazeRoute, "GRT-500: route type is not maze");
    if needs_ripup_check(&r.grids, r.routelen as usize, threshold, caps, None, &used_h, &used_v).is_none() {
        return;
    }
    let nn = net.ndr_net(id);
    give_back_committed(&mut grid.g.for_net(&nn), &r.grids, r.routelen as usize, nn.edge_cost);

    let (p1, p2) = ((t.nodes[e.n1].x as i32, t.nodes[e.n1].y as i32), (t.nodes[e.n2].x as i32, t.nodes[e.n2].y as i32));
    let others: Vec<(i32, i32)> = t.nodes.iter().map(|n| (n.x as i32, n.y as i32)).collect();
    let dims = (grid.g.est.x_grids as i32, grid.g.est.y_grids as i32);
    let bx = monotonic_box(p1, p2, enlarge, dims, &others, t.num_terminals);
    let g = &*grid.g;
    let used_h = |x: i32, y: i32| f64::from(g.usage_red_h(x, y, red_h(x as usize, y as usize)));
    let used_v = |x: i32, y: i32| f64::from(g.usage_red_v(x, y, red_v(x as usize, y as usize)));
    let (px, py, bl1, bl2, _best) = monotonic_search(p1, p2, bx, cost_table, grid.via_cost, &used_h, &used_v);
    let (points, routelen) = walk_monotonic_route(&mut grid.g.for_net(&nn), p1, (px, py), p2, bl1, bl2, nn.edge_cost);
    let r = &mut t.routes[eid];
    r.grids = points;
    r.routelen = routelen;
}

/// `routeMonotonicAll(threshold, expand, logis_cof)` — build the cost table, then
/// [`route_monotonic_edge`] over every edge of every net.
///
/// ⚠️ The table is `h_cost_table_`, sized `10 × h_capacity_` from `costheight_` and the (float)
/// logistic coefficient; it serves vertical edges too.
#[allow(clippy::too_many_arguments)]
pub fn route_monotonic_all(
    threshold: i32,
    expand: i32,
    logis_cof: f32,
    costheight: i32,
    net_ids: &[usize],
    nets: &[RsmtNet<'_>],
    state: &mut [NetState],
    grid: &mut BrkGrid<'_>,
) {
    let table = monotonic_cost_table(f64::from(costheight), grid.h_capacity, f64::from(logis_cof));
    for &id in net_ids {
        let n = state[id].tree.as_ref().map_or(0, |t| t.edges.len());
        for eid in 0..n {
            route_monotonic_edge(id, eid, &nets[id], &mut state[id], grid, &table, threshold, expand);
        }
    }
}

/// One monotonic round as the run records it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LvRound {
    /// The `newTH` and `enlarge_` this round was routed with.
    pub threshold: i32,
    pub enlarge: i32,
    pub logistic_coef: f32,
    pub scan: Overflow2DScan,
}

/// run()'s LV block (R12) — `costheight_` from the pattern phase's `maxOverflow`, then three rounds
/// of `routeMonotonicAll(newTH, enlarge_, logistic_coef)` + `getOverflow2Dmaze`.
///
/// ⛔ `logistic_coef = 2.0 / (1 + log(maxOverflow))` is computed in double and stored in a FLOAT;
/// `log(0)` is `-inf`, so a round entered with no overflow gets `-0.0` and a flat cost table.
/// ⚠️ `maxOverflow > 700` also sets `VIA = 0`, `THRESH_M = 0`, `CSTEP1 = 30` for the congestion loop,
/// and a `logistic_coef` of 1.33 that the first round overwrites; those belong to R14.
/// `on_round(k, round, graph, state)` sees the state after each round — the sequencer exposes what
/// it passes on.
pub fn lv_rounds(
    max_overflow: i32,
    net_ids: &[usize],
    nets: &[RsmtNet<'_>],
    state: &mut [NetState],
    grid: &mut BrkGrid<'_>,
    on_round: &mut dyn FnMut(usize, &LvRound, &Graph2d, &[NetState]),
) -> (i32, Vec<LvRound>) {
    const COSHEIGHT: i32 = 4;
    const LV_ITER: usize = 3;
    let costheight = if max_overflow > 700 { 8 } else { COSHEIGHT };
    let (mut enlarge, mut new_th, mut max_ov) = (10, 10, max_overflow);
    let mut rounds = Vec::with_capacity(LV_ITER);
    for k in 0..LV_ITER {
        let logistic_coef = (2.0 / (1.0 + f64::from(max_ov).ln())) as f32;
        route_monotonic_all(new_th, enlarge, logistic_coef, costheight, net_ids, nets, state, grid);
        let scan = grid.g.get_overflow_2d_maze();
        max_ov = scan.max_overflow;
        let round = LvRound { threshold: new_th, enlarge, logistic_coef, scan };
        on_round(k, &round, grid.g, state);
        rounds.push(round);
        enlarge += 5;
        new_th = (new_th - 5).max(1);
    }
    (costheight, rounds)
}

/// R13 — `InitEstUsage`, `InitLastUsage(1)`, `SaveLastRouteLen`, in the run's order.
pub fn init_for_congestion_loop(net_ids: &[usize], state: &mut [NetState], g: &mut Graph2d) {
    g.est.init_est_usage();
    g.est.init_last_usage(1);
    for &id in net_ids {
        if let Some(t) = state[id].tree.as_mut() {
            for r in t.routes.iter_mut() {
                r.last_routelen = r.routelen;
            }
        }
    }
}
