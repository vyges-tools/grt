// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 4c — the incremental re-route (`updateDirtyRoutesFastRoute`).
//!
//! End to end, `grt-incr-score.py` replays FastRoute's whole state (every 2D and 3D edge, the
//! used-grid sets, `net_ids_` with each net's pins and layers) where the first run ends, where the
//! incremental run starts and where it ends, and `grt-diode-score.py` the dirty nets and their
//! merged routes — on the three suite scripts that insert diodes, all exact. The rules below are
//! the ones those designs reach without testing their edges, each on a constructed case.

use vyges_grt::netlist::pin_positions_changed;
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

use vyges_grt::full3d::{Point3D, RouteType};
use vyges_grt::maze3d::{Edge3D, Tree3D};
use vyges_grt::repair_antennas::update_route_grids_layer;

fn tree(points: &[(i16, i16, i16)], len: i32) -> Tree3D {
    let grids: Vec<Point3D> = points.iter().map(|&(x, y, layer)| Point3D { x, y, layer }).collect();
    let e = Edge3D { n1: 0, n2: 1, n1a: 0, n2a: 1, len, route_type: RouteType::MazeRoute, routelen: grids.len() as i32 - 1, grids };
    Tree3D { num_terminals: 2, num_layers: 6, pin_layers: vec![0, 0], nodes: Vec::new(), edges: vec![e] }
}

fn points(t: &Tree3D) -> Vec<(i16, i16, i16)> {
    t.edges[0].grids.iter().map(|p| (p.x, p.y, p.layer)).collect()
}

/// The same-layer unit steps a rip-up (`releaseNetResources`) takes back, per layer.
fn released(t: &Tree3D) -> Vec<(i16, i16, i16)> {
    let g = &t.edges[0].grids;
    (0..t.edges[0].routelen as usize).filter(|&i| g[i].layer == g[i + 1].layer).map(|i| (g[i].x.min(g[i + 1].x), g[i].y.min(g[i + 1].y), g[i].layer)).collect()
}

/// `updateRouteGridsLayer`: an interior run moves to the new layer with an OLD-layer copy of its
/// boundary point on each side, so the rip-up sees vias there — and releases exactly what the
/// jumper charged: layer 1 outside the span, layer 3 inside it.
#[test]
fn a_jumpered_run_is_relayered_with_vias_at_both_ends() {
    let mut t = tree(&[(0, 0, 1), (1, 0, 1), (2, 0, 1), (3, 0, 1)], 3);
    update_route_grids_layer(&mut t, (1, 0), (2, 0), 1, 3);
    assert_eq!(points(&t), vec![(0, 0, 1), (1, 0, 1), (1, 0, 3), (2, 0, 3), (2, 0, 1), (3, 0, 1)]);
    assert_eq!(t.edges[0].routelen, 5);
    // updateEdge2DAnd3DUsage over tiles 1..2 walks x = 1 only: that edge moved to layer 3.
    assert_eq!(released(&t), vec![(0, 0, 1), (1, 0, 3), (2, 0, 1)]);
}

/// At the route's ends no copy is added: the first point has no predecessor, the last no successor.
#[test]
fn a_run_at_the_route_ends_adds_no_via_there() {
    let mut t = tree(&[(0, 0, 1), (1, 0, 1), (2, 0, 1)], 2);
    update_route_grids_layer(&mut t, (0, 0), (2, 0), 1, 3);
    assert_eq!(points(&t), vec![(0, 0, 3), (1, 0, 3), (2, 0, 3)]);
    let mut t = tree(&[(0, 0, 1), (1, 0, 1), (2, 0, 1)], 2);
    update_route_grids_layer(&mut t, (0, 0), (0, 0), 1, 3);
    assert_eq!(points(&t), vec![(0, 0, 3), (0, 0, 1), (1, 0, 1), (2, 0, 1)]);
}

/// ⛔ Two back-to-back jumpers, applied one after the other: the second's run starts next to the
/// first's OLD-layer copy, which is outside it — so each keeps its own vias, and the two promoted
/// points never merge into one same-layer chain.
#[test]
fn back_to_back_jumpers_keep_separate_vias() {
    let mut t = tree(&[(0, 0, 1), (1, 0, 1), (2, 0, 1), (3, 0, 1)], 3);
    update_route_grids_layer(&mut t, (1, 0), (1, 0), 1, 3);
    update_route_grids_layer(&mut t, (2, 0), (2, 0), 1, 3);
    assert_eq!(points(&t), vec![(0, 0, 1), (1, 0, 1), (1, 0, 3), (1, 0, 1), (2, 0, 1), (2, 0, 3), (2, 0, 1), (3, 0, 1)]);
}

/// Untouched: a point on another layer inside the box; a span given final-first (the box is the
/// ends AS GIVEN, so it is empty); an edge with `len` and `routelen` both zero.
#[test]
fn what_the_relayering_leaves_alone() {
    let before = [(0, 0, 1), (1, 0, 2), (2, 0, 1)];
    let mut t = tree(&before, 2);
    update_route_grids_layer(&mut t, (1, 0), (1, 0), 1, 3);
    assert_eq!(points(&t), before.to_vec());
    let mut t = tree(&before, 2);
    update_route_grids_layer(&mut t, (2, 0), (0, 0), 1, 3);
    assert_eq!(points(&t), before.to_vec());
    let mut t = tree(&[(4, 4, 1)], 0);
    update_route_grids_layer(&mut t, (4, 4), (4, 4), 1, 3);
    assert_eq!(points(&t), vec![(4, 4, 1)]);
}
