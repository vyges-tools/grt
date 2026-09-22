// SPDX-License-Identifier: Apache-2.0
//! Stage R6 — `routeLAll(true)`: estimate every segment, then L-route every diagonal one.
//!
//! The rules are [`estimate_one_seg`] (both candidate Ls at half weight), [`choose_l_shape`] (the
//! cheaper L, a tie to x-first) and [`commit_l_shape`] (winner to full, loser to zero). This module
//! is the call sequence over them, with every update charged through the NDR-aware cost.
//!
//! ⚠️ `routeLAll` is called ONCE, with `firstTime = true`; its `false` branch (rip-up + `routeSegL`)
//! has no caller in `run()` and is not transcribed.

use crate::brk_rsmt::{newroute_l, BrkGrid, NetState, RouteKind, RoutedSegment, RsmtNet};
use crate::lroute::EdgeRoute;
use crate::ripup_route::{new_ripup_congested_l, new_ripup_net};
use crate::zroute::{newroute_z, newroute_z_edge};
use crate::spiral::{propagate_alias_status, register_edges, reset_and_alias, spiral_route, traversal_order, WALK_RESET};
use crate::estimate::{capacity_lower_bound, choose_l_shape, commit_l_shape, congestion_cost, estimate_one_seg, needs_l_route, Usage2d, LShape};

/// `routeSegLFirstTime` — price both Ls against the half-and-half estimate, then commit the cheaper.
///
/// ⚠️ The price is the same congestion sum the later L passes use — `est + red` above the lower
/// bound — over column `x1` + row `y2` (y-first) against column `x2` + row `y1` (x-first). The
/// segment's bend is recorded on it (`xFirst`).
pub fn route_seg_l_first_time<G: Usage2d + ?Sized>(
    grid: &mut G,
    seg: &mut RoutedSegment,
    v_lb: f32,
    h_lb: f32,
    red_v: &dyn Fn(usize, usize) -> u16,
    red_h: &dyn Fn(usize, usize) -> u16,
) {
    let s = seg.seg;
    let (ymin, ymax) = (s.y1.min(s.y2), s.y1.max(s.y2));
    let (mut cost_l1, mut cost_l2) = (0.0, 0.0);
    for i in ymin..ymax {
        cost_l1 += congestion_cost(grid.v(s.x1 as usize, i as usize), red_v(s.x1 as usize, i as usize), v_lb);
        cost_l2 += congestion_cost(grid.v(s.x2 as usize, i as usize), red_v(s.x2 as usize, i as usize), v_lb);
    }
    for i in s.x1..s.x2 {
        cost_l1 += congestion_cost(grid.h(i as usize, s.y2 as usize), red_h(i as usize, s.y2 as usize), h_lb);
        cost_l2 += congestion_cost(grid.h(i as usize, s.y1 as usize), red_h(i as usize, s.y1 as usize), h_lb);
    }
    let shape = choose_l_shape(cost_l1, cost_l2);
    commit_l_shape(grid, &s, shape);
    seg.x_first = shape == LShape::XFirst;
}

/// `routeLAll(true)` — the call sequence, and nothing else.
///
/// ⛔ Two passes over every net, in `net_ids` order: ALL segments are estimated before ANY is
/// L-routed. Straight segments are estimated and not routed; their `xFirst` stays as it was.
pub fn route_l_all(net_ids: &[usize], nets: &[RsmtNet<'_>], state: &mut [NetState], grid: &mut BrkGrid<'_>) {
    for &id in net_ids {
        let nn = nets[id].ndr_net(id);
        let mut g = grid.g.for_net(&nn);
        for s in &state[id].seglist {
            estimate_one_seg(&mut g, &s.seg);
        }
    }
    let (v_lb, h_lb) = (capacity_lower_bound(grid.v_capacity), capacity_lower_bound(grid.h_capacity));
    for &id in net_ids {
        let nn = nets[id].ndr_net(id);
        let mut g = grid.g.for_net(&nn);
        for s in state[id].seglist.iter_mut() {
            if needs_l_route(&s.seg) {
                route_seg_l_first_time(&mut g, s, v_lb, h_lb, grid.red_v, grid.red_h);
            }
        }
    }
}

/// `newrouteLAll(firstTime, viaGuided)` — [`newroute_l`] over every net in `net_ids` order, ripping
/// up each edge's previous route unless `first_time`. The run calls it once, as R8:
/// `newrouteLAll(false, true)`.
pub fn newroute_l_all(first_time: bool, via_guided: bool, net_ids: &[usize], nets: &[RsmtNet<'_>], state: &mut [NetState], grid: &mut BrkGrid<'_>) {
    for &id in net_ids {
        let nn = nets[id].ndr_net(id);
        let tree = state[id].tree.as_mut().expect("R7 copied every net's tree");
        newroute_l(tree, &nn, grid, !first_time, via_guided);
    }
}

/// `spiralRouteAll` — re-route every net outward from its pins, the call sequence and nothing else.
///
/// Four passes, each over EVERY net before the next begins, as the reference's four loops:
/// 1. reset each net's nodes and alias coincident Steiner nodes to the first node at their point
///    ([`reset_and_alias`], terminals at status 2);
/// 2. register each positive-length edge on its endpoints' alias nodes ([`register_edges`]);
/// 3. per net: rip up the whole net (`newRipupNet`), then route its edges in the breadth-first
///    order the walk reaches them from the pins ([`traversal_order`], [`spiral_route`]);
/// 4. copy each alias node's status to the nodes it stands for ([`propagate_alias_status`]).
///
/// ⚠️ The order is structural — the walk's `assigned` flags do not depend on the routes — so it is
/// computed once per net and then routed, which is what the reference's interleaved loop amounts to.
/// ⚠️ `pin_layer(net, pin)` fills `topL`/`botL` on terminals; nothing reads them before layer
/// assignment, which resets them itself.
pub fn spiral_route_all(
    net_ids: &[usize],
    nets: &[RsmtNet<'_>],
    state: &mut [NetState],
    grid: &mut BrkGrid<'_>,
    num_layers: i16,
    pin_layer: &dyn Fn(usize, usize) -> i16,
) {
    for &id in net_ids {
        let t = state[id].tree.as_mut().expect("R7 copied every net's tree");
        let coords: Vec<(i16, i16)> = t.nodes.iter().map(|n| (n.x, n.y)).collect();
        let pins: Vec<i16> = t.node_to_pin_idx.iter().map(|&p| pin_layer(id, p as usize)).collect();
        t.walk = reset_and_alias(&coords, t.num_terminals, &pins, num_layers, WALK_RESET);
    }
    for &id in net_ids {
        let t = state[id].tree.as_mut().expect("tree");
        let edges: Vec<(usize, usize, i32)> = t.edges.iter().map(|e| (e.n1, e.n2, e.len)).collect();
        t.edge_reg = register_edges(&mut t.walk, &edges);
    }
    let (v_lb, h_lb) = (capacity_lower_bound(grid.v_capacity), capacity_lower_bound(grid.h_capacity));
    for &id in net_ids {
        let nn = nets[id].ndr_net(id);
        let t = state[id].tree.as_mut().expect("tree");
        let routed: Vec<((i32, i32), (i32, i32), crate::ripup_route::RoutedShape)> = t
            .edges
            .iter()
            .zip(&t.routes)
            .filter(|(e, _)| e.len > 0)
            .map(|(e, r)| {
                let (a, b) = (&t.nodes[e.n1], &t.nodes[e.n2]);
                ((a.x as i32, a.y as i32), (b.x as i32, b.y as i32), r.shape())
            })
            .collect();
        let mut g = grid.g.for_net(&nn);
        new_ripup_net(&mut g, &routed, nn.edge_cost);
        for eid in traversal_order(&mut t.walk, &t.edge_reg, t.num_terminals) {
            let e = t.edges[eid];
            let r = spiral_route(&mut g, &mut t.walk, (e.n1, e.n2, e.len), nn.edge_cost, grid.via_cost, v_lb, h_lb, grid.red_v, grid.red_h);
            let rt = &mut t.routes[eid];
            match r {
                EdgeRoute::None => rt.kind = RouteKind::NoRoute,
                EdgeRoute::Vertical | EdgeRoute::L(LShape::YFirst) => (rt.kind, rt.x_first) = (RouteKind::LRoute, false),
                EdgeRoute::Horizontal | EdgeRoute::L(LShape::XFirst) => (rt.kind, rt.x_first) = (RouteKind::LRoute, true),
            }
        }
    }
    for &id in net_ids {
        let t = state[id].tree.as_mut().expect("tree");
        propagate_alias_status(&mut t.walk);
        for (n, w) in t.nodes.iter_mut().zip(&t.walk) {
            n.status = w.status;
        }
    }
}

/// `newrouteZAll(threshold)` — [`newroute_z_net`] over every net in `net_ids` order. The run calls
/// it once, as R10: `newrouteZAll(10)`.
pub fn newroute_z_all(threshold: i32, net_ids: &[usize], nets: &[RsmtNet<'_>], state: &mut [NetState], grid: &mut BrkGrid<'_>) {
    for &id in net_ids {
        newroute_z_net(threshold, id, &nets[id], &mut state[id], grid);
    }
}

/// `newrouteZ(net, threshold)` — per tree edge longer than `threshold` and not straight: rip up its
/// L if that runs over a congested edge ([`new_ripup_congested_l`]) and Z-route it
/// ([`newroute_z`]); otherwise, on a two-terminal net, rip it up and Z-route it regardless
/// ([`newroute_z_edge`]).
///
/// ⛔ One status array for the whole stage: the gate lowers the edge's OWN endpoints' statuses and
/// the Z route reads and marks the ALIAS nodes'. Both live on the walk's nodes (which R9 left equal
/// to the tree's); they are copied back to the tree when the net is done.
/// ⚠️ The reference's `else` arm for `len <= threshold` requires `len > threshold` and is dead.
pub fn newroute_z_net(threshold: i32, id: usize, net: &RsmtNet<'_>, st: &mut NetState, grid: &mut BrkGrid<'_>) {
    let nn = net.ndr_net(id);
    let (v_lb, h_lb) = (capacity_lower_bound(grid.v_capacity), capacity_lower_bound(grid.h_capacity));
    let t = st.tree.as_mut().expect("R7 copied every net's tree");
    let caps = grid.caps;
    let cap = |x: i32, y: i32, h: bool| caps.edge_capacity(net.min_layer, net.max_layer, x, y, h);
    for ind in 0..t.edges.len() {
        let e = t.edges[ind];
        if e.len <= threshold {
            continue;
        }
        let (n1, n2) = (e.n1, e.n2);
        let (p1, p2) = ((t.walk[n1].x as i32, t.walk[n1].y as i32), (t.walk[n2].x as i32, t.walk[n2].y as i32));
        if p1.0 == p2.0 || p1.1 == p2.1 {
            continue;
        }
        let mut g = grid.g.for_net(&nn);
        let mut statuses: Vec<i16> = t.walk.iter().map(|w| w.status).collect();
        let ripped = new_ripup_congested_l(&mut g, &mut statuses, (n1, n2), p1, p2, t.routes[ind].x_first, t.num_terminals, nn.edge_cost, &cap);
        for (w, s) in t.walk.iter_mut().zip(statuses) {
            w.status = s;
        }
        if ripped {
            let (n1a, n2a) = (t.walk[n1].stack_alias, t.walk[n2].stack_alias);
            t.routes[ind].kind = RouteKind::ZRoute;
            let z = newroute_z(&mut g, &mut t.walk, n1a, n2a, p1, p2, nn.edge_cost, grid.via_cost, v_lb, h_lb, grid.red_v, grid.red_h);
            t.routes[ind].hvh = z.hvh;
            t.routes[ind].z_point = z.z_point as i16;
        } else if t.num_terminals == 2 {
            let prior = t.routes[ind].shape();
            if let Some(z) = newroute_z_edge(&mut g, e.len, p1, p2, &prior, nn.edge_cost, v_lb, h_lb, grid.red_v, grid.red_h) {
                let rt = &mut t.routes[ind];
                (rt.kind, rt.hvh, rt.z_point) = (RouteKind::ZRoute, true, z as i16);
            }
        }
    }
    for (n, w) in t.nodes.iter_mut().zip(&t.walk) {
        n.status = w.status;
    }
}
