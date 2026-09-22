// SPDX-License-Identifier: Apache-2.0
//! Stages R15–R20 — run()'s finalisation after the congestion loop, on the shared router state.
//!
//! R15: `freeRR` (drops the loop's saved routes — held by the loop here, so nothing to do) and
//! `removeLoops` over every net ([`remove_loops`]), then `getOverflow2Dmaze`.

use crate::brk_rsmt::{NetState, RsmtNet};
use crate::graph2d::Graph2d;
use crate::maze::remove_loops;

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
