// SPDX-License-Identifier: Apache-2.0
//! Stage R14's router — `mazeRouteMSMDSequential`: one pass of 2D maze rip-up-and-reroute over every
//! net, on the shared router state.
//!
//! The rules are elsewhere and gated on their own: the rip-up gate ([`needs_ripup_check`]), the
//! congestion ordering ([`order_trees_for_ripup`]), the per-edge search ([`route_one_edge`]: region,
//! seeding, search, backtrace), and the tree surgery ([`split_edge`], [`update_route_type1`],
//! [`update_route_type2`], [`rewire_after_type2`]). This module is the call sequence over them, and
//! the adapter between the router's tree and the surgery's node/edge lists.
//!
//! ⚠️ Not transcribed, and refused rather than guessed: the partial-slack pass
//! (`CalculatePartialSlack`, which needs timing) and the rebuild after a failed surgery
//! (`reInitTree`, which no corpus pass reaches). The snapshot-batched variant is a separate mode.

use crate::brk_rsmt::{BrkGrid, NetState, RouteKind, RsmtNet, StTree};
use crate::lroute::TreeNode;
use crate::maze::{
    netedge_order_dec, rewire_after_type2, route_one_edge, split_edge, update_route_type1, update_route_type2, EdgeContext, EdgeOutcome,
    MazeEdge, MazeNode, RelaxInputs, SurgeryEdge,
};
use crate::mazecost::CostParams;
use crate::ripup::{order_trees_for_ripup, OrderTree};
use crate::ripup_route::{give_back_committed, needs_ripup_check, CriticalCheck, RipupReason};
use crate::spiral::{EdgeReg, SpiralNode};

/// The arguments of one `mazeRouteMSMD(iter, expand, ripup_threshold, maze_edge_threshold,
/// ordering, via, L, cost_params, slack_th)` call.
#[derive(Debug, Clone, Copy)]
pub struct MsmdParams {
    pub iter: i32,
    pub expand: i32,
    pub ripup_threshold: i32,
    pub maze_edge_threshold: i32,
    pub ordering: bool,
    pub via: i32,
    pub l: i32,
    pub cost: CostParams,
    pub slack_th: f32,
    /// `critical_nets_percentage_` — non-zero turns on the partial-slack pass and the gate's
    /// critical arm.
    pub critical_nets_percentage: i32,
}

/// What a pass leaves for the run loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MsmdResult {
    /// ⛔ The run-level `enlarge_` as the pass left it: every re-routed edge overwrites it with
    /// `min(expand, (iter / 6 + 3) * routelen)`, and the loop's next iteration starts from it.
    /// `None` when no edge was re-routed (the loop's value stands).
    pub enlarge: Option<i32>,
    pub slack_th: f32,
}

/// `StNetOrder` — order the nets by their routes' congestion, then by slack, and write the stamped
/// slacks back onto the nets.
pub fn st_net_order(net_ids: &[usize], nets: &[RsmtNet<'_>], state: &mut [NetState], grid: &BrkGrid<'_>) -> Vec<usize> {
    let g = &*grid.g;
    let trees: Vec<OrderTree> = net_ids
        .iter()
        .map(|&id| {
            let net = &nets[id];
            let cap = |x: i32, y: i32, h: bool| grid.caps.edge_capacity(net.min_layer, net.max_layer, x, y, h);
            let mut xmin = 0i32;
            if let Some(t) = state[id].tree.as_ref() {
                for r in &t.routes {
                    for k in 0..r.routelen.max(0) as usize {
                        let ((ax, ay), (bx, by)) = (r.grids[k], r.grids[k + 1]);
                        if ax == bx {
                            let y = ay.min(by);
                            xmin += (i32::from(g.est.usage_v(ax as usize, y as usize)) - cap(ax, y, false)).max(0);
                        } else {
                            let x = ax.min(bx);
                            xmin += (i32::from(g.est.usage_h(x as usize, ay as usize)) - cap(x, ay, true)).max(0);
                        }
                    }
                }
            }
            OrderTree { net: id.to_string(), xmin, slack: state[id].slack }
        })
        .collect();
    let ordered = order_trees_for_ripup(&trees);
    ordered
        .iter()
        .map(|t| {
            let id: usize = t.net.parse().expect("id");
            state[id].slack = t.slack;
            id
        })
        .collect()
}

/// `mazeRouteMSMDSequential` — the call sequence, and nothing else.
///
/// Per net (congestion order when `ordering`, else `net_ids`), per edge by descending route length:
/// recompute the length, skip if not over `maze_edge_threshold`, ask the gate (which gives the old
/// route back), search, move the path's ends onto the tree ([`attach_path_end`] for each side),
/// write the route and charge it.
pub fn maze_route_msmd_sequential(p: &MsmdParams, net_ids: &[usize], nets: &[RsmtNet<'_>], state: &mut [NetState], grid: &mut BrkGrid<'_>) -> Result<MsmdResult, String> {
    let slack_th = p.slack_th;
    let order: Vec<usize> = if p.ordering {
        if p.critical_nets_percentage != 0 {
            return Err("CalculatePartialSlack needs timing slacks; not transcribed".into());
        }
        st_net_order(net_ids, nets, state, grid)
    } else {
        net_ids.to_vec()
    };
    let mut enlarge = None;
    let dims = (grid.g.est.x_grids as i32, grid.g.est.y_grids as i32);
    for id in order {
        let net = &nets[id];
        let nn = net.ndr_net(id);
        let routelens: Vec<i32> = state[id].tree.as_ref().expect("tree").routes.iter().map(|r| r.routelen).collect();
        for oe in netedge_order_dec(&routelens) {
            let eid = oe.edge_id;
            let st = &mut state[id];
            let t = st.tree.as_mut().expect("tree");
            let e = t.edges[eid];
            let (p1, p2) = ((t.nodes[e.n1].x as i32, t.nodes[e.n1].y as i32), (t.nodes[e.n2].x as i32, t.nodes[e.n2].y as i32));
            t.edges[eid].len = (p2.0 - p1.0).abs() + (p2.1 - p1.1).abs();
            if t.edges[eid].len <= p.maze_edge_threshold {
                continue;
            }
            // The gate (`newRipupCheck` with the loop's threshold and critical slack).
            let (red_h, red_v) = (grid.red_h, grid.red_v);
            let reason = {
                let g = &*grid.g;
                let used_h = |x: i32, y: i32| f64::from(g.usage_red_h(x, y, red_h(x as usize, y as usize)));
                let used_v = |x: i32, y: i32| f64::from(g.usage_red_v(x, y, red_v(x as usize, y as usize)));
                let r = &t.routes[eid];
                let critical = Some(CriticalCheck {
                    enabled: p.critical_nets_percentage != 0,
                    last_routelen: r.last_routelen.max(0) as usize,
                    critical_slack: slack_th,
                    slack: st.slack,
                });
                needs_ripup_check(&r.grids, r.routelen as usize, p.ripup_threshold, (grid.h_capacity, grid.v_capacity), critical, &used_h, &used_v)
            };
            let Some(reason) = reason else { continue };
            if reason == RipupReason::CriticalDetour {
                st.critical = true;
            }
            {
                let r = &t.routes[eid];
                give_back_committed(&mut grid.g.for_net(&nn), &r.grids, r.routelen as usize, nn.edge_cost);
            }
            enlarge = Some(p.expand.min((p.iter / 6 + 3) * t.routes[eid].routelen));

            // The search.
            let (maze_nodes, maze_edges) = (maze_nodes_of(t), maze_edges_of(t));
            let outcome = {
                let g = &*grid.g;
                let used_h = |x: i32, y: i32| i32::from(g.usage_red_h(x, y, red_h(x as usize, y as usize)));
                let used_v = |x: i32, y: i32| i32::from(g.usage_red_v(x, y, red_v(x as usize, y as usize)));
                let last_h = |x: i32, y: i32| i32::from(g.est.last_usage_h(x as usize, y as usize));
                let last_v = |x: i32, y: i32| i32::from(g.est.last_usage_v(x as usize, y as usize));
                let relax = RelaxInputs {
                    l: p.l,
                    via: f64::from(p.via),
                    h_capacity: grid.h_capacity,
                    v_capacity: grid.v_capacity,
                    params: &p.cost,
                    used_h: &used_h,
                    used_v: &used_v,
                    last_h: &last_h,
                    last_v: &last_v,
                };
                let ctx = EdgeContext {
                    maze_edge_threshold: p.maze_edge_threshold,
                    expand: p.expand,
                    iter: p.iter,
                    is_critical: st.critical,
                    grid_size: dims,
                    num_terminals: t.num_terminals,
                    edge_cost: nn.edge_cost,
                    relax: &relax,
                    rip_up_says_reroute: true,
                };
                route_one_edge(dims.0 as usize + 1, &maze_nodes, &maze_edges, eid, &ctx)?
            };
            let EdgeOutcome::Routed { path, corr_edge, .. } = outcome else {
                return Err(format!("net {id} edge {eid}: the search did not route"));
            };
            let corr = |pt: (i32, i32)| corr_edge_at(&corr_edge, pt).expect("a path end is a seed");

            // The surgery, on the surgery's lists, then written back.
            let mut nodes = maze_nodes;
            let mut sedges = surgery_edges_of(t);
            let (e1, e2) = (path[0], *path.last().expect("a path has points"));
            let nt = t.num_terminals;
            let n2 = e.n2;
            let n1 = attach_path_end(&mut nodes, &mut sedges, nt, e.n1, n2, p1, e1, eid, &corr)?;
            attach_path_end(&mut nodes, &mut sedges, nt, e.n2, n1, p2, e2, eid, &corr)?;
            write_back(t, &nodes, &sedges);

            let r = &mut t.routes[eid];
            (r.grids, r.routelen, r.kind) = (path.clone(), path.len() as i32 - 1, RouteKind::MazeRoute);
            t.edges[eid].len = (e1.0 - e2.0).abs() + (e1.1 - e2.1).abs();
            let mut g = grid.g.for_net(&nn);
            use crate::estimate::Usage2d;
            for w in path.windows(2) {
                let ((ax, ay), (bx, by)) = (w[0], w[1]);
                if ax == bx {
                    g.update_usage_v(ax, ay.min(by), f64::from(nn.edge_cost));
                } else {
                    g.update_usage_h(ax.min(bx), ay, f64::from(nn.edge_cost));
                }
            }
        }
    }
    Ok(MsmdResult { enlarge, slack_th })
}

/// The tree edge the seeding recorded at `pt` — the reference's `corr_edge_[y][x]`, an array each
/// seeded point is WRITTEN into, so ⛔ a point seeded from two edges (a node they share) names the
/// LAST one written.
pub fn corr_edge_at(writes: &[((i32, i32), usize)], pt: (i32, i32)) -> Option<usize> {
    writes.iter().rev().find(|(q, _)| *q == pt).map(|&(_, e)| e)
}

/// Move one end of the re-routed edge (`n`, whose far end is `other`) onto the point the path
/// reached (`end`), when it differs from where the node stood (`at`) — the reference's
/// "consider subtree 1 / subtree 2" blocks.
///
/// ⛔ A PIN cannot move: it is first split ([`split_edge`]) and its stand-in moves instead. ⛔ If the
/// path's end sits on one of the node's own other edges it is a type-1 move; on any other edge of
/// the tree (`corr_edge` names it) it is type 2, and five nodes are rewired.
/// Returns the node now at this end (the stand-in, after a split).
#[allow(clippy::too_many_arguments)]
pub fn attach_path_end(
    nodes: &mut Vec<MazeNode>,
    edges: &mut Vec<SurgeryEdge>,
    num_terminals: usize,
    mut n: usize,
    other: usize,
    at: (i32, i32),
    end: (i32, i32),
    edge_id: usize,
    corr: &dyn Fn((i32, i32)) -> usize,
) -> Result<usize, String> {
    if end == at {
        return Ok(n);
    }
    if n < num_terminals {
        n = split_edge(nodes, edges, other, n, edge_id);
    }
    if n < num_terminals {
        return Ok(n);
    }
    let ce = corr(end);
    let (endpt1, endpt2) = (edges[ce].n1, edges[ce].n2);
    let nb = &nodes[n].neighbours;
    let (mut a1, mut a2, mut ea1, mut ea2) = if nb[0].0 == other {
        (nb[1].0, nb[2].0, nb[1].1, nb[2].1)
    } else if nb[1].0 == other {
        (nb[0].0, nb[2].0, nb[0].1, nb[2].1)
    } else {
        (nb[0].0, nb[1].0, nb[0].1, nb[1].1)
    };
    if endpt1 == n || endpt2 == n {
        if endpt1 == a2 || endpt2 == a2 {
            std::mem::swap(&mut a1, &mut a2);
            std::mem::swap(&mut ea1, &mut ea2);
        }
        update_route_type1(nodes, n, a1, a2, end, edges, ea1, ea2).map_err(|e| format!("reInitTree (not transcribed): {e}"))?;
        (nodes[n].x, nodes[n].y) = end;
    } else {
        let (c1, c2) = (endpt1, endpt2);
        update_route_type2(nodes, n, a1, a2, c1, c2, end, edges, ea1, ea2, ce).map_err(|e| format!("reInitTree (not transcribed): {e}"))?;
        (nodes[n].x, nodes[n].y) = end;
        rewire_after_type2(nodes, edges, n, other, a1, a2, c1, c2, edge_id, ea1, ea2, ce);
    }
    Ok(n)
}

fn maze_nodes_of(t: &StTree) -> Vec<MazeNode> {
    (0..t.nodes.len())
        .map(|i| MazeNode {
            x: t.nodes[i].x as i32,
            y: t.nodes[i].y as i32,
            neighbours: (0..t.nbr_count[i]).map(|k| (t.nbr[i][k], t.edge[i][k])).collect(),
            stack_alias: t.walk.get(i).map_or(i, |w| w.stack_alias),
        })
        .collect()
}

fn maze_edges_of(t: &StTree) -> Vec<MazeEdge> {
    t.edges
        .iter()
        .zip(&t.routes)
        .map(|(e, r)| MazeEdge { n1: e.n1, n2: e.n2, routelen: r.routelen.max(0) as usize, grids: r.grids.clone() })
        .collect()
}

fn surgery_edges_of(t: &StTree) -> Vec<SurgeryEdge> {
    t.edges
        .iter()
        .enumerate()
        .map(|(j, e)| {
            let r = &t.routes[j];
            let (n1a, n2a) = t.edge_reg.get(j).and_then(|g| g.alias).unwrap_or((0, 0));
            SurgeryEdge { n1: e.n1, n2: e.n2, n1a, n2a, routelen: r.routelen.max(0) as usize, grids: r.grids.clone(), is_maze_route: r.kind == RouteKind::MazeRoute, len: e.len }
        })
        .collect()
}

/// Write the surgery's lists back onto the tree. ⚠️ A node the split created starts as the
/// reference's `TreeNode` default: status 0, `hID`/`lID` -1, `topL`/`botL` -1, unassigned.
fn write_back(t: &mut StTree, nodes: &[MazeNode], edges: &[SurgeryEdge]) {
    for (i, n) in nodes.iter().enumerate() {
        if i == t.nodes.len() {
            t.nodes.push(TreeNode { x: n.x as i16, y: n.y as i16, status: 0 });
            t.nbr.push([0; 3]);
            t.edge.push([0; 3]);
            t.nbr_count.push(0);
            t.walk.push(SpiralNode { x: n.x as i16, y: n.y as i16, top_layer: -1, bot_layer: -1, assigned: false, stack_alias: n.stack_alias, status: 0, h_id: -1, l_id: -1, edges: Vec::new() });
        }
        (t.nodes[i].x, t.nodes[i].y) = (n.x as i16, n.y as i16);
        if let Some(w) = t.walk.get_mut(i) {
            (w.x, w.y, w.stack_alias) = (n.x as i16, n.y as i16, n.stack_alias);
        }
        t.nbr_count[i] = n.neighbours.len();
        for (k, &(v, e)) in n.neighbours.iter().enumerate() {
            t.nbr[i][k] = v;
            t.edge[i][k] = e;
        }
    }
    for (j, e) in edges.iter().enumerate() {
        if j == t.edges.len() {
            t.edges.push(crate::lroute::TreeEdge { n1: e.n1, n2: e.n2, len: e.len });
            t.routes.push(crate::brk_rsmt::TreeRoute::default());
            t.edge_reg.push(EdgeReg { alias: None, assigned: false });
        }
        t.edges[j] = crate::lroute::TreeEdge { n1: e.n1, n2: e.n2, len: e.len };
        let r = &mut t.routes[j];
        r.grids = e.grids.clone();
        r.routelen = e.routelen as i32;
        if e.is_maze_route {
            r.kind = RouteKind::MazeRoute;
        }
        if let Some(g) = t.edge_reg.get_mut(j) {
            g.alias = Some((e.n1a, e.n2a));
        }
    }
}
