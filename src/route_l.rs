// SPDX-License-Identifier: Apache-2.0
//! Stage R6 — `routeLAll(true)`: estimate every segment, then L-route every diagonal one.
//!
//! The rules are [`estimate_one_seg`] (both candidate Ls at half weight), [`choose_l_shape`] (the
//! cheaper L, a tie to x-first) and [`commit_l_shape`] (winner to full, loser to zero). This module
//! is the call sequence over them, with every update charged through the NDR-aware cost.
//!
//! ⚠️ `routeLAll` is called ONCE, with `firstTime = true`; its `false` branch (rip-up + `routeSegL`)
//! has no caller in `run()` and is not transcribed.

use crate::brk_rsmt::{newroute_l, BrkGrid, NetState, RoutedSegment, RsmtNet};
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
