// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 4c — the incremental re-route (`updateDirtyRoutesFastRoute`).
//!
//! End to end, `grt-incr-score.py` replays FastRoute's whole state (every 2D and 3D edge, the
//! used-grid sets, `net_ids_` with each net's pins and layers) where the first run ends, where the
//! incremental run starts and where it ends, and `grt-diode-score.py` the dirty nets and their
//! merged routes — on the three suite scripts that insert diodes, all exact. The rules below are
//! the ones those designs reach without testing their edges, each on a constructed case.

use vyges_grt::global_route::pin_positions_changed;
use vyges_grt::graph2d::Graph2d;
use vyges_grt::run::resumed_graph_2d;

/// `pinPositionsChanged` counts: order does not matter, multiplicity does.
#[test]
fn pin_change_is_a_multiset_comparison() {
    let (a, b, c) = ((3, 4, 1), (5, 4, 1), (3, 4, 2));
    assert!(!pin_positions_changed(&[a, b], &[b, a]));
    assert!(pin_positions_changed(&[a, b], &[a, c])); // same place, other connection layer
    // ⛔ A diode whose pin lands in the SAME gcell, on the same layer, as a pin already there
    // still changes the net: the count at that position goes from one to two.
    assert!(pin_positions_changed(&[a, b], &[a, b, a]));
    assert!(pin_positions_changed(&[a, a], &[a]));
}

/// `run()` under `is_incremental_grt_`: `clearUsed`, then `rebuildUsedGrids` — the used sets come
/// back from COMMITTED usage above zero, not from the last run's sets and not from the estimate.
#[test]
fn a_resumed_run_rebuilds_the_used_sets_from_committed_usage() {
    let mut g = Graph2d::new(3, 3, 1);
    g.est.update_usage_h(1, 2, 1.0); // committed, horizontal
    g.est.update_usage_v(2, 1, 2.0); // committed, vertical
    g.est.update_h(0, 1, 0, 3.0); // estimate only: never re-enters
    g.used_h.insert((0, 1)); // the last run's, now released: dropped
    g.used_v.insert((0, 0));
    let r = resumed_graph_2d(&g);
    assert_eq!(r.used_h.iter().copied().collect::<Vec<_>>(), vec![(1, 2)]);
    assert_eq!(r.used_v.iter().copied().collect::<Vec<_>>(), vec![(2, 1)]);
    // The rest of the graph is carried over untouched.
    assert_eq!((r.est.h(0, 0), r.est.usage_h(1, 2), r.est.usage_v(2, 1)), (3.0, 1, 2));
}

/// A release that takes an edge back to zero drops it from the rebuilt set; one left above zero
/// (another net still on it) keeps it.
#[test]
fn a_released_edge_leaves_the_set_only_at_zero() {
    let mut g = Graph2d::new(3, 3, 1);
    g.est.update_usage_h(0, 0, 2.0);
    g.est.update_usage_h(1, 0, 1.0);
    g.est.update_usage_h(0, 0, -1.0);
    g.est.update_usage_h(1, 0, -1.0);
    let r = resumed_graph_2d(&g);
    assert_eq!(r.used_h.iter().copied().collect::<Vec<_>>(), vec![(0, 0)]);
}
