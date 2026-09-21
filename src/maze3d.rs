// SPDX-License-Identifier: Apache-2.0
//! R18 — the three-dimensional maze pass, `mazeRouteMSMDOrder3D`.
//!
//! Runs twice after layer assignment (long edge window, then short) and only when the planar
//! overflow is zero. Measured before building: that gate passes on 67 of 76 runs per cost mode,
//! and the pass changes the routes of thousands of nets — it decides the final routes on most
//! designs.
//!
//! This module is built in pieces, each against its own capture. **R18a — this file so far — is
//! the driver's control sequence**: which nets are walked, which are skipped, which edges fall in
//! the window, and how the retry passes repeat. The per-edge work is handed to a
//! [`Maze3DEdgeWork`], so the sequence is testable now and each later piece slots in behind it:
//!
//! | piece | the per-edge work, in the reference's order |
//! | --- | --- |
//! | R18b | `newRipup3DType3` — rip the edge up (declines only a zero-length edge) |
//! | R18c | the 3D heap primitives |
//! | R18d | `setupHeap3D` — seed both subtrees |
//! | R18e | the six-direction search |
//! | R18f | the backtrace to the crossing |
//! | R18g | the tree surgery (`updateRouteType13D/23D`), which yields the retry list |
//! | R18h | the commit, and `recoverEdge` when no legal path was found |

/// The final nets percentage the resistance-aware prelude sets, unless the user fixed one.
pub const FINAL_RES_AWARE_NETS_PERCENTAGE: f32 = 100.0;
/// The detour penalty the resistance-aware prelude sets for an incremental run…
pub const LOW_DETOUR_PENALTY: i32 = 5;
/// …and for the first run.
pub const HIGH_DETOUR_PENALTY: i32 = 15;

/// One entry of the net order the pass walks (`tree_order_pv_`), as it stands AFTER the
/// resistance-aware prelude has re-ordered it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrderedNet {
    pub net_id: usize,
    pub slack: f32,
    pub res_aware: bool,
}

/// One call's parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Maze3DCall {
    /// The window is `ripup_lb < len < ripup_ub`, exclusive at both ends.
    pub ripup_lb: i32,
    pub ripup_ub: i32,
    pub resistance_aware: bool,
    pub incremental: bool,
}

/// What the resistance-aware prelude sets before the walk (the reference also re-runs
/// `updateSlacks` and `netpinOrderInc` there — both transcribed in R16).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prelude {
    pub nets_percentage: Option<f32>,
    pub detour_penalty: Option<i32>,
}

/// ⚠️ Nothing is set outside resistance-aware mode. Inside it, the nets percentage is raised to
/// the final value only when the user did not fix one — 4 captured calls keep a fixed 30%.
pub fn prelude(call: &Maze3DCall, fixed_nets_percentage: bool) -> Prelude {
    if !call.resistance_aware {
        return Prelude { nets_percentage: None, detour_penalty: None };
    }
    Prelude {
        nets_percentage: (!fixed_nets_percentage).then_some(FINAL_RES_AWARE_NETS_PERCENTAGE),
        detour_penalty: Some(if call.incremental { LOW_DETOUR_PENALTY } else { HIGH_DETOUR_PENALTY }),
    }
}

/// How many nets of the order are walked.
///
/// ⛔ Outside resistance-aware mode only the first **90%**, computed as `size * 0.9` in double
/// and TRUNCATED into an `int` — 495 nets walk 445, not 446. The reference's comment: the rest is
/// left to the incremental optimisations.
pub fn end_index(order_len: usize, resistance_aware: bool) -> usize {
    if resistance_aware {
        order_len
    } else {
        (order_len as f64 * 0.9) as i32 as usize
    }
}

/// ⚠️ Positive-slack nets are skipped only in a resistance-aware, NON-incremental call — and
/// `>= 0`, so zero slack is skipped too.
pub fn skips_for_slack(call: &Maze3DCall, slack: f32) -> bool {
    call.resistance_aware && !call.incremental && slack >= 0.0
}

/// ⛔ Exclusive at BOTH ends: `len >= ub || len <= lb` is outside.
pub fn edge_in_window(len: i32, call: &Maze3DCall) -> bool {
    !(len >= call.ripup_ub || len <= call.ripup_lb)
}

/// Retry passes are allowed only in an incremental, resistance-aware call.
pub fn max_reroute_iter(call: &Maze3DCall) -> i32 {
    if call.incremental && call.resistance_aware {
        5
    } else {
        0
    }
}

/// What routing one edge produced, as the driver needs it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EdgeResult {
    /// Edges whose content a node shift overwrote — the retry candidates.
    pub retry: Vec<usize>,
    /// No legal path was found and the original route was kept.
    pub recovered: bool,
}

/// The per-edge work behind the driver — pieces R18b–h.
pub trait Maze3DEdgeWork {
    fn num_edges(&self, net: usize) -> usize;
    /// ⚠️ Read at the check, not up front: routing an earlier edge of the same net can change a
    /// later edge's length through the tree surgery.
    fn edge_len(&mut self, net: usize, edge: usize) -> i32;
    /// Called once per walked net; in a resistance-aware call it carries the net's flag, which
    /// the reference stores as the router-wide `resistance_aware_` before routing any edge.
    fn begin_net(&mut self, _net: usize, _res_aware: Option<bool>) {}
    fn route_edge(&mut self, net: usize, edge: usize) -> EdgeResult;
}

/// One step of the walk, in order — the driver's own trace.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Maze3DEvent {
    Net { net: usize, skipped: bool },
    Edge { edge: usize, len: i32, in_window: bool },
    Pass { iter: i32, stop: bool, retry: Vec<usize> },
}

/// The walk — the reference's driver, and nothing else.
///
/// Returns every step and the number of nets with a recovered edge (the reference warns GRT-183
/// with it).
pub fn maze_route_msmd_order_3d(
    call: &Maze3DCall,
    order: &[OrderedNet],
    work: &mut impl Maze3DEdgeWork,
) -> (Vec<Maze3DEvent>, usize) {
    let mut events = Vec::new();
    let max_iter = max_reroute_iter(call);
    let mut recovered_nets = 0;

    for entry in &order[..end_index(order.len(), call.resistance_aware)] {
        let net = entry.net_id;
        if skips_for_slack(call, entry.slack) {
            events.push(Maze3DEvent::Net { net, skipped: true });
            continue;
        }
        work.begin_net(net, call.resistance_aware.then_some(entry.res_aware));
        events.push(Maze3DEvent::Net { net, skipped: false });

        // The first pass visits every edge in index order; later passes only the retry list.
        let mut to_process: Vec<usize> = (0..work.num_edges(net)).collect();
        let mut iter = 0;
        let mut recovered = false;
        loop {
            let mut next: Vec<usize> = Vec::new();
            for &edge in &to_process {
                let len = work.edge_len(net, edge);
                let in_window = edge_in_window(len, call);
                events.push(Maze3DEvent::Edge { edge, len, in_window });
                if !in_window {
                    continue;
                }
                let r = work.route_edge(net, edge);
                recovered |= r.recovered;
                next.extend(r.retry);
            }
            // ⚠️ Sorted and de-duplicated before the stop test, as the reference does.
            next.sort_unstable();
            next.dedup();
            let stop = next.is_empty() || iter >= max_iter;
            events.push(Maze3DEvent::Pass { iter, stop, retry: next.clone() });
            if stop {
                break;
            }
            to_process = next;
            iter += 1;
        }
        if recovered {
            recovered_nets += 1;
        }
    }
    (events, recovered_nets)
}
