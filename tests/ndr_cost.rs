// SPDX-License-Identifier: Apache-2.0
//! The NDR-aware usage cost (`Graph2D::getCostNDRAware`) and its ledger, at the f11b4276 pin.
//!
//! ⚠️ The call-by-call replay of `examples/grt_gate/ndr_cost.json` was captured at 4155ac31, under
//! the 100× overflow charge that OpenROAD #11963 removed (NDR deficits replace it). It is retired
//! here; a capture of the deficit ledger at the pin is on the heavy-run plan. Until then the rules
//! below are constructed, each from the reference's code.

use vyges_grt::estimate::Usage2d as _;
use vyges_grt::*;

fn ndr(id: usize, ec: i8) -> NdrCostNet {
    NdrCostNet { id, edge_cost: ec, min_layer: 0, max_layer: 1, layer_edge_cost: Some(vec![ec, ec]), soft_ndr: false }
}

fn ledger_with(cap: f64) -> NdrLedger {
    let mut l = NdrLedger::new(4, 4, 2);
    for x in 0..3 {
        for y in 0..4 {
            l.update_cap_3d(x, y, 0, true, cap);
            l.update_cap_3d(x, y, 1, true, cap);
        }
    }
    l
}

fn ledger() -> NdrLedger {
    ledger_with(10.0)
}

/// An edge-cost-1 net's amount passes unchanged and leaves no trace.
#[test]
fn an_edge_cost_1_net_passes_through() {
    let mut l = ledger();
    let before = l.clone();
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 1), 1, 1, 0.5, true), 0.5);
    assert_eq!(l, before);
}

/// ⛔ Only a NEGATIVE amount removes: zero is an add, charges the full edge cost, and debits the
/// first layer with room.
#[test]
fn a_zero_amount_is_an_add() {
    let mut l = ledger();
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 0.0, true), 3.0);
    assert!(l.has_net(true, 1, 1, 0));
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr(), 7);
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true), 0.0, "a second add counts nothing");
}

/// ⛔ Upstream #11963: where no layer has room the net is charged its TRUE edge cost (no 100×), its
/// lowest layer is debited anyway, and the tracks it misses on its best layer —
/// `min over layers of max(0, cost − max(0, NDR capacity))` — are its deficit on the edge and in
/// the total. The rip-up gives the deficit back with the record.
#[test]
fn a_net_that_fits_no_layer_records_its_deficit_not_an_overflow_charge() {
    let mut l = ledger();
    let big = NdrCostNet { layer_edge_cost: Some(vec![12, 14]), ..ndr(1, 14) };
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true), 3.0);
    // Layer 0 has 7 left, layer 1 10: the net misses 12 − 7 = 5 there, 14 − 10 = 4 here.
    assert_eq!(l.compute_ndr_deficit(&big, 1, 1, true), 4);
    assert_eq!(l.ndr_deficit_if_routed(&big, 1, 1, true), 4, "the edge's 0, plus the net's own");
    assert_eq!(l.get_cost_ndr_aware(&big, 1, 1, 1.0, true), 14.0);
    assert_eq!(l.usage(true, 1, 1, 1), Some(NdrUsage { layer: 0, amount: 12, deficit: 4 }));
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit()), (4, 4));
    assert!(l.has_ndr_deficit(true, 1, 1));
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr(), -5, "the lowest layer is debited anyway");
    assert_eq!(l.ndr_deficit(&big, 1, 1, true), 4);
    assert_eq!(l.ndr_deficit_if_routed(&big, 1, 1, true), 4, "a net on the edge sees the edge's deficit only");
    assert_eq!(l.ndr_deficit_if_routed(&ndr(2, 3), 1, 1, true), 4, "another net that fits still sees it");
    assert_eq!(l.ndr_deficit_if_routed(&ndr(2, 1), 1, 1, true), 0, "a regular net never does");
    assert_eq!(l.get_cost_ndr_aware(&big, 1, 1, -1.0, true), -14.0);
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit(), l.cap(true, 0, 1, 1).cap_ndr()), (0, 0, 7));
}

/// `computeNDRDeficit` with no layer in the net's range: nothing can host it, the deficit is its
/// whole edge cost.
#[test]
fn no_layer_in_range_misses_the_whole_edge_cost() {
    let l = ledger();
    let empty = NdrCostNet { min_layer: 1, max_layer: 0, ..ndr(0, 6) };
    assert_eq!(l.compute_ndr_deficit(&empty, 1, 1, true), 6);
}

/// ⛔ Upstream #11963 (74d34bd075 `refitNDRDeficitNets`): when a net leaves an edge that still has a
/// deficit, every deficit net there is released and reserved again in DATABASE-ID order, so the
/// freed tracks go to the lowest database id — not the lowest router id.
#[test]
fn a_leaving_net_frees_its_layer_for_the_deficit_nets_in_database_id_order() {
    let mut l = ledger_with(5.0);
    // Router ids 0..4; database ids: net 2 is 9, net 3 is 1.
    l.set_db_ids(vec![0, 1, 9, 1]);
    let n = |id| ndr(id, 5);
    for id in 0..4 {
        l.get_cost_ndr_aware(&n(id), 1, 1, 1.0, true);
    }
    // 0 on layer 0, 1 on layer 1; 2 and 3 fit nowhere.
    assert_eq!(l.usage(true, 1, 1, 2).map(|u| u.deficit), Some(5));
    assert_eq!(l.usage(true, 1, 1, 3).map(|u| u.deficit), Some(5));
    assert_eq!(l.total_ndr_deficit(), 10);
    l.get_cost_ndr_aware(&n(1), 1, 1, -1.0, true);
    assert_eq!(l.usage(true, 1, 1, 3), Some(NdrUsage { layer: 1, amount: 5, deficit: 0 }), "database id 1 refits first");
    assert_eq!(l.usage(true, 1, 1, 2).map(|u| u.deficit), Some(5));
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit()), (5, 5));
}

/// Upstream #11963: `initCap3D` resizes the capacities (kept where the extents overlap) and then
/// `initNDRnets` drops every record, deficit and debit — a new run's net must not be taken for
/// one of the last run's.
#[test]
fn a_reinit_drops_the_records_deficits_and_debits_but_keeps_the_capacities() {
    let mut l = ledger();
    let big = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(1, 20) };
    l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true);
    l.get_cost_ndr_aware(&big, 1, 1, 1.0, true);
    assert!(l.total_ndr_deficit() > 0);
    l.init_cap_3d(4, 4, 2);
    assert!(!l.has_net(true, 1, 1, 0) && !l.has_net(true, 1, 1, 1));
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit()), (0, 0));
    assert_eq!((l.cap(true, 0, 1, 1).cap, l.cap(true, 0, 1, 1).cap_ndr()), (10, 10));
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true), 3.0, "net 0 is new on the edge");
}

/// `copyRoutingStateFrom` takes the capacities with the edges, and the NDR state (records, debits,
/// deficits) only when asked; without it the three are dropped together.
#[test]
fn a_copy_takes_the_ndr_state_only_when_asked() {
    let mut src = ledger();
    src.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true);
    src.get_cost_ndr_aware(&NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(1, 20) }, 1, 1, 1.0, true);
    let mut with = ledger();
    with.get_cost_ndr_aware(&ndr(5, 3), 2, 2, 1.0, true);
    let mut without = with.clone();
    with.copy_routing_state_from(&src, true);
    without.copy_routing_state_from(&src, false);
    assert!(with.has_net(true, 1, 1, 0) && !with.has_net(true, 2, 2, 5));
    assert!(!without.has_net(true, 1, 1, 0) && !without.has_net(true, 2, 2, 5));
    assert_eq!(with.cap(true, 0, 1, 1).cap_ndr(), -13, "with the records, the debits come across");
    assert_eq!(without.cap(true, 0, 1, 1).cap_ndr(), 10, "without them, the NDR capacity is the capacity");
    assert_eq!((with.total_ndr_deficit() > 0, without.total_ndr_deficit(), without.edge_ndr_deficit(true, 1, 1)), (true, 0, 0));
}

/// `clearNDRnets` drops the records, the per-edge deficits and their total (#11963 1fe08158b2);
/// the debits stay.
#[test]
fn clearing_drops_records_and_deficits_but_keeps_the_debits() {
    let mut l = ledger();
    let big = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(1, 20) };
    l.get_cost_ndr_aware(&big, 1, 1, 1.0, true);
    l.clear_ndr_nets();
    assert!(!l.has_net(true, 1, 1, 1));
    assert_eq!((l.edge_ndr_deficit(true, 1, 1), l.total_ndr_deficit(), l.cap(true, 0, 1, 1).cap_ndr()), (0, 0, -10));
}

/// `addCap3D` moves the capacity; what NDR nets reserved stays reserved.
#[test]
fn an_adjustment_keeps_the_debit() {
    let mut l = ledger();
    l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, 1.0, true);
    l.add_cap_3d(1, 1, 0, true, -4);
    assert_eq!((l.cap(true, 0, 1, 1).cap, l.cap(true, 0, 1, 1).cap_ndr()), (6, 3));
    l.update_cap_3d(1, 1, 0, true, 8.0);
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr(), 8, "updateCap3D drops the debit");
}

/// ⛔ Removing a net that is not on the edge charges nothing and changes nothing — the reference's
/// comment: the second half-cost removal of the first routing pass.
#[test]
fn removing_an_absent_net_charges_nothing() {
    let mut l = ledger();
    let before = l.clone();
    assert_eq!(l.get_cost_ndr_aware(&ndr(0, 3), 1, 1, -1.5, true), 0.0);
    assert_eq!(l, before);
}

/// `computeNDRDeficit`: a layer other NDR nets already oversubscribed has NO free track — the net
/// misses its whole width there, not more.
#[test]
fn an_oversubscribed_layer_counts_as_no_free_track() {
    let mut l = ledger_with(5.0);
    let one = |id| NdrCostNet { max_layer: 0, layer_edge_cost: Some(vec![5]), ..ndr(id, 5) };
    l.get_cost_ndr_aware(&one(0), 1, 1, 1.0, true);
    l.get_cost_ndr_aware(&one(1), 1, 1, 1.0, true);
    assert_eq!(l.cap(true, 0, 1, 1).cap_ndr(), -5);
    assert_eq!(l.compute_ndr_deficit(&one(2), 1, 1, true), 5);
}

/// A 4 × 4 graph: every 2D capacity 10, every layer 5 NDR tracks over two layers.
fn graph() -> Graph2d {
    let mut g = Graph2d::new(4, 4, 2);
    g.cap_h = vec![10; 16];
    g.cap_v = vec![10; 16];
    for x in 0..4 {
        for y in 0..4 {
            for l in 0..2 {
                g.ndr.update_cap_3d(x, y, l, true, 5.0);
                g.ndr.update_cap_3d(x, y, l, false, 5.0);
            }
        }
    }
    g
}

/// ⛔ Upstream #11963 `getUsageNDRAware`: a net that would leave the edge short of NDR tracks sees
/// the edge as at least `capacity − 1 + 100 × deficit`, with the EDGE's own 2D capacity (#11963
/// 01b0fe5704, `getCapH(x, y)`, not `h_capacity_`); a net that fits, and every regular net, sees
/// the usage unchanged, and a usage above the floor is kept.
#[test]
fn a_net_that_would_miss_tracks_sees_the_edge_as_overflowed() {
    let mut g = graph();
    let wide = NdrCostNet { layer_edge_cost: Some(vec![6, 6]), ..ndr(1, 6) };
    assert_eq!(g.ndr.ndr_deficit_if_routed(&wide, 1, 1, true), 1);
    assert_eq!(g.usage_ndr_aware(&wide, 1, 1, true, 3.0), 10.0 - 1.0 + 100.0);
    assert_eq!(g.usage_ndr_aware(&wide, 1, 1, true, 500.0), 500.0, "max, not replace");
    assert_eq!(g.usage_ndr_aware(&ndr(2, 3), 1, 1, true, 3.0), 3.0, "a net that fits");
    assert_eq!(g.usage_ndr_aware(&ndr(2, 1), 1, 1, true, 3.0), 3.0, "a regular net");
    g.cap_h[4 + 1] = 4; // (1, 1)
    assert_eq!(g.usage_ndr_aware(&wide, 1, 1, true, 3.0), 4.0 - 1.0 + 100.0, "the edge's own capacity");
    g.cap_v[4 + 2] = 7; // (2, 1)
    assert_eq!(g.usage_ndr_aware(&wide, 2, 1, false, 0.0), 7.0 - 1.0 + 100.0, "the vertical edge's");
}

/// ⛔ Upstream #11963: the pattern routes price each edge through `getEstUsageRedH/V(net, …)`, so an
/// NDR net turns its L away from edges where it would miss tracks; a regular net, seeing the same
/// empty grid, keeps the tie-break bend (x first).
#[test]
fn an_l_route_turns_away_from_where_the_net_would_miss_tracks() {
    let mut g = graph();
    // Row 0 (the x-first L's horizontal run for a (0,0)→(2,2) segment) has 2 NDR tracks a layer.
    for x in 0..2 {
        for l in 0..2 {
            g.ndr.update_cap_3d(x, 0, l, true, 2.0);
        }
    }
    let red = |_: usize, _: usize| 0u16;
    let route = |g: &mut Graph2d, net: &NdrCostNet| {
        let before = g.clone();
        let mut seg = RoutedSegment { seg: vyges_grt::rsmt::Segment { x1: 0, y1: 0, x2: 2, y2: 2, edge_cost: net.edge_cost }, x_first: false };
        route_seg_l_first_time(&mut g.for_net(net), &mut seg, 0.0, 0.0, &red, &red);
        *g = before;
        seg.x_first
    };
    assert!(route(&mut g, &ndr(1, 1)), "a regular net keeps the tie bend");
    assert!(!route(&mut g, &NdrCostNet { layer_edge_cost: Some(vec![3, 3]), ..ndr(1, 3) }), "the NDR net goes y first");
}

/// A graph with an NDR deficit on horizontal edge (1, 1) and vertical edge (2, 0): a net of 20
/// NDR tracks fits neither layer of 5 there.
fn graph_with_deficits() -> Graph2d {
    let mut g = graph();
    let fat = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(9, 20) };
    g.for_net(&fat).update_usage_h(1, 1, 1.0);
    g.for_net(&fat).update_usage_v(2, 0, 1.0);
    assert!(g.ndr.has_ndr_deficit(true, 1, 1) && g.ndr.has_ndr_deficit(false, 2, 0));
    g
}

/// ⛔ Upstream #11963 `newRipupCheck`: an NDR net (edge cost above 1) is torn up off an edge where
/// the NDR nets fit no single layer, even with the usage far under the threshold; a regular net
/// on the same route is not. Both directions are read at the walk's lower coordinate.
#[test]
fn the_maze_gate_tears_an_ndr_net_off_an_edge_with_a_deficit() {
    let g = graph_with_deficits();
    let zero = |_: i32, _: i32| 0.0;
    let mut regular = g.clone();
    let mut wide = g.clone();
    // (2,1) → (1,1) → (1,2): horizontal (1,1) walked right to left.
    let h_route = [(2, 1), (1, 1), (1, 2)];
    // (2,1) → (2,0): vertical (2,0) walked downwards.
    let v_route = [(3, 1), (2, 1), (2, 0)];
    for route in [&h_route[..], &v_route[..]] {
        assert_eq!(new_ripup_check(&mut regular.for_net(&ndr(3, 1)), route, 2, 0, (10, 10), 1, None, &zero, &zero), None);
        assert_eq!(new_ripup_check(&mut wide.for_net(&ndr(3, 3)), route, 2, 0, (10, 10), 3, None, &zero, &zero), Some(RipupReason::Congested));
    }
}

/// ⛔ Upstream #11963 `newRipupCongestedL`: the same for the L rip-up — an NDR net leaves an L whose
/// checked column or row has an NDR deficit, though no edge is over capacity.
#[test]
fn the_l_rip_up_takes_an_ndr_net_off_a_deficit() {
    let cap = |_: i32, _: i32, _: bool| 10;
    // (0,0) → (2,1) y first checks column 0, then row 1: the deficit at (1,1).
    // (1,0) → (2,1) x first checks column 2: the deficit at (2,0).
    for (p1, p2, x_first) in [((0, 0), (2, 1), false), ((1, 0), (2, 1), true)] {
        for (ec, rips) in [(1, false), (3, true)] {
            let mut g = graph_with_deficits();
            let mut statuses = vec![2i16, 2];
            let got = new_ripup_congested_l(&mut g.for_net(&ndr(3, ec)), &mut statuses, (0, 1), p1, p2, x_first, 2, ec, &cap);
            assert_eq!(got, rips, "{p1:?}→{p2:?} x_first {x_first} edge cost {ec}");
        }
    }
}

/// ⛔ Upstream #11963 `getOverflow2D` / `getOverflow2Dmaze`: the tracks the NDR nets miss count as
/// overflow too — added to the total, not to the maximum, and not to the usage.
#[test]
fn the_overflow_totals_count_the_ndr_deficit() {
    // Committed: the 20-track net on two edges of capacity 10 → 10 + 10 over, 15 + 15 missed.
    let g = graph_with_deficits();
    let s = g.get_overflow_2d_maze();
    assert_eq!((s.total_overflow, s.max_overflow, s.total_usage), (10 + 10 + 15 + 15, 10, 40));
    // Estimated: the same net estimated on one edge.
    let mut g = graph();
    let fat = NdrCostNet { layer_edge_cost: Some(vec![20, 20]), ..ndr(9, 20) };
    g.for_net(&fat).update_h(1, 2, 1, 1.0);
    let s = g.get_overflow_2d();
    assert_eq!((s.total_overflow, s.max_overflow), (10 + 15, 10));
}

/// ⛔ Upstream #11963 run(): R17 (demote the congested NDR nets) also fires when an NDR deficit is
/// left though the 2D and 3D totals match; the 3D maze passes run when the 2D overflow is ALL NDR
/// deficit (`canRunMaze3D`: `overflow_2d == getTotalNDRDeficit()`), not only at zero.
#[test]
fn run_demotes_on_a_left_deficit_and_runs_3d_maze_past_one() {
    use vyges_grt::run::{can_run_maze_3d, ndr_mismatch};
    assert!(!ndr_mismatch(7, 7, 0));
    assert!(ndr_mismatch(7, 7, 1), "the totals match, a deficit is left");
    assert!(ndr_mismatch(7, 6, 0));
    assert!(can_run_maze_3d(0, 0));
    assert!(can_run_maze_3d(5, 5), "all of it is deficit");
    assert!(!can_run_maze_3d(6, 5), "one track of usage overflow");
}
