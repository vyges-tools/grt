// SPDX-License-Identifier: Apache-2.0
//! R17 — demoting congested non-default-rule nets.
//!
//! ⛔ **There is no golden here, because the stage never runs.** Across **152 evaluations of its
//! gate in 142 runs covering 139 designs in both cost modes, it fired zero times.** The reason is
//! structural and is recorded in the programme's upstream notes: a design with NDR nets has no
//! congestion, and a congested design has no NDR nets. Probes that squeezed the NDR design's
//! layers to 70%, 85% and 95% still produced no congestion at all.
//!
//! So these are constructed cases stating the reference's rules, and they claim no more than
//! that. ⚠️ Each rule is written the way the source writes it; none is evidence about how the
//! reference behaves on a real design.

use std::collections::HashMap;
use vyges_grt::{
    apply_soft_ndr, congested_ndr_nets, disable_ndr_for_congested_nets, update_net_3d_usage,
    update_planar_net_usage, CongestionView, NdrEdge, NdrNet, Point3D, UsageGrid,
};

#[derive(Default)]
struct Grid {
    overflow_v: HashMap<(i16, i16), i32>,
    overflow_h: HashMap<(i16, i16), i32>,
    available_v: HashMap<(i16, i16, i16), i32>,
    available_h: HashMap<(i16, i16, i16), i32>,
    usage_2d: HashMap<(char, i16, i16), i32>,
    usage_3d: HashMap<(char, i16, i16, i16), i32>,
}

impl CongestionView for Grid {
    fn overflow_v(&self, x: i16, y: i16) -> i32 {
        *self.overflow_v.get(&(x, y)).unwrap_or(&0)
    }
    fn overflow_h(&self, x: i16, y: i16) -> i32 {
        *self.overflow_h.get(&(x, y)).unwrap_or(&0)
    }
    fn available_v(&self, l: i16, x: i16, y: i16) -> i32 {
        *self.available_v.get(&(l, x, y)).unwrap_or(&100)
    }
    fn available_h(&self, l: i16, x: i16, y: i16) -> i32 {
        *self.available_h.get(&(l, x, y)).unwrap_or(&100)
    }
}

impl UsageGrid for Grid {
    fn add_usage_v_2d(&mut self, x: i16, y: i16, d: i32) {
        *self.usage_2d.entry(('v', x, y)).or_insert(0) += d;
    }
    fn add_usage_h_2d(&mut self, x: i16, y: i16, d: i32) {
        *self.usage_2d.entry(('h', x, y)).or_insert(0) += d;
    }
    fn add_usage_v_3d(&mut self, l: i16, x: i16, y: i16, d: i32) {
        *self.usage_3d.entry(('v', l, x, y)).or_insert(0) += d;
    }
    fn add_usage_h_3d(&mut self, l: i16, x: i16, y: i16, d: i32) {
        *self.usage_3d.entry(('h', l, x, y)).or_insert(0) += d;
    }
}

fn p(x: i16, y: i16, layer: i16) -> Point3D {
    Point3D { x, y, layer }
}

/// A net of one horizontal edge from (0,0) to (3,0) on layer 2, with per-layer costs.
fn net(id: usize, has_ndr: bool, soft: bool, cost: i8) -> NdrNet {
    NdrNet {
        net_id: id,
        has_ndr,
        is_soft_ndr: soft,
        edge_cost: cost,
        layer_edge_cost: Some(vec![1, 1, 4, 4]),
        edges: vec![NdrEdge {
            routelen: 3,
            grids: vec![p(0, 0, 2), p(1, 0, 2), p(2, 0, 2), p(3, 0, 2)],
        }],
    }
}

/// ⛔ A plain net and an already-demoted net are both skipped, so the pass cannot demote twice.
#[test]
fn only_undemoted_ndr_nets_are_considered() {
    let mut g = Grid::default();
    g.overflow_h.insert((0, 0), 5);
    let nets = vec![net(0, false, false, 4), net(1, true, true, 4), net(2, true, false, 4)];
    assert_eq!(
        congested_ndr_nets(&nets, &g),
        vec![2],
        "only the undemoted NDR net may be selected"
    );
}

/// ⛔ Either kind of congestion is enough on its own.
#[test]
fn two_dimensional_overflow_alone_selects_a_net() {
    let mut g = Grid::default();
    g.overflow_h.insert((1, 0), 1);
    assert_eq!(congested_ndr_nets(&[net(0, true, false, 4)], &g), vec![0]);
}

#[test]
fn an_oversubscribed_layer_alone_selects_a_net() {
    let mut g = Grid::default();
    // Negative availability on one step of the route, no 2D overflow anywhere.
    g.available_h.insert((2, 1, 0), -1);
    assert_eq!(congested_ndr_nets(&[net(0, true, false, 4)], &g), vec![0]);
}

/// ⚠️ Zero overflow is not congestion: the test is "greater than zero", and availability must be
/// **below** zero, not merely zero.
#[test]
fn the_congestion_thresholds_are_strict() {
    let mut g = Grid::default();
    g.overflow_h.insert((1, 0), 0);
    g.available_h.insert((2, 2, 0), 0);
    assert!(
        congested_ndr_nets(&[net(0, true, false, 4)], &g).is_empty(),
        "zero overflow and zero availability are not congestion"
    );
}

/// ⛔ A step that changes layer is a via and is never tested for congestion, however congested
/// the edge it sits on.
#[test]
fn a_layer_change_is_not_tested_for_congestion() {
    let mut g = Grid::default();
    // The congested position is only ever reached by the step that changes layer.
    g.overflow_h.insert((0, 0), 99);
    g.available_h.insert((2, 0, 0), -99);
    let n = NdrNet {
        net_id: 0,
        has_ndr: true,
        is_soft_ndr: false,
        edge_cost: 4,
        layer_edge_cost: Some(vec![1, 1, 4, 4]),
        edges: vec![NdrEdge {
            routelen: 1,
            grids: vec![p(0, 0, 2), p(1, 0, 3)],
        }],
    };
    assert!(
        congested_ndr_nets(&[n], &g).is_empty(),
        "a step that changes layer must be skipped"
    );
}

/// ⛔ An edge with no steps, or no points, is skipped rather than indexed.
#[test]
fn an_empty_edge_is_skipped() {
    let mut g = Grid::default();
    g.overflow_h.insert((0, 0), 99);
    let n = NdrNet {
        net_id: 0,
        has_ndr: true,
        is_soft_ndr: false,
        edge_cost: 4,
        layer_edge_cost: None,
        edges: vec![
            NdrEdge { routelen: 0, grids: vec![p(0, 0, 2)] },
            NdrEdge { routelen: 2, grids: Vec::new() },
        ],
    };
    assert!(congested_ndr_nets(&[n], &g).is_empty(), "neither edge may be walked");
}

/// ⛔ **The demotion is a bracket, and the cost is read on both sides of it.** Usage is refunded
/// at the old cost and charged at the new one, so the net effect on a three-step route is
/// `3 * (1 - 4)`, not zero and not `3 * -4`.
#[test]
fn the_planar_usage_is_refunded_at_the_old_cost_and_charged_at_the_new() {
    let mut g = Grid::default();
    let mut n = net(0, true, false, 4);
    apply_soft_ndr(&mut n, &mut g);

    assert!(n.is_soft_ndr, "the net must be marked soft");
    assert_eq!(n.edge_cost, 1, "and its edge cost dropped to one");
    for x in 0..3i16 {
        assert_eq!(
            g.usage_2d.get(&('h', x, 0)).copied().unwrap_or(0),
            -3,
            "step at x={x}: refunded 4 and charged 1"
        );
    }
}

/// ⛔ The layered bracket sits **outside** the planar one, so its refund uses the original
/// per-layer cost and its charge uses the demoted one — which is 1 for every layer.
#[test]
fn the_layered_usage_brackets_the_demotion_from_outside() {
    let mut view = Grid::default();
    view.overflow_h.insert((1, 0), 1);
    let mut sink = Grid::default();
    let mut nets = vec![net(0, true, false, 4)];

    let demoted = disable_ndr_for_congested_nets(&mut nets, &view, &mut sink, true);
    assert_eq!(demoted, vec![0]);
    for x in 0..3i16 {
        // ⛔ Layer 2's cost was 4 before the demotion and 1 after: -1*4 then +1*1. A single
        // signed pass either side of the demotion would give 0 or -6.
        assert_eq!(
            sink.usage_3d.get(&('h', 2, x, 0)).copied().unwrap_or(0),
            -3,
            "step at x={x}: the layered refund must use the ORIGINAL per-layer cost"
        );
    }
    // ⚠️ And the planar bracket ran inside it, on the same net.
    for x in 0..3i16 {
        assert_eq!(sink.usage_2d.get(&('h', x, 0)).copied().unwrap_or(0), -3);
    }
}

/// ⛔ Without the flag, only the planar usage is re-accounted; the layered grid is untouched.
#[test]
fn the_layered_usage_is_left_alone_when_the_flag_is_clear() {
    let mut g = Grid::default();
    g.overflow_h.insert((1, 0), 1);
    let mut nets = vec![net(0, true, false, 4)];
    let mut sink = Grid::default();
    let demoted = disable_ndr_for_congested_nets(&mut nets, &g, &mut sink, false);
    assert_eq!(demoted, vec![0]);
    assert!(sink.usage_3d.is_empty(), "no layered usage may be written");
    assert!(!sink.usage_2d.is_empty(), "the planar usage still is");
}

/// ⛔ With nothing congested the pass returns before doing anything at all.
#[test]
fn nothing_congested_means_nothing_written() {
    let g = Grid::default();
    let mut sink = Grid::default();
    let mut nets = vec![net(0, true, false, 4)];
    let demoted = disable_ndr_for_congested_nets(&mut nets, &g, &mut sink, true);
    assert!(demoted.is_empty());
    assert!(sink.usage_2d.is_empty() && sink.usage_3d.is_empty());
    assert!(!nets[0].is_soft_ndr, "no net may be demoted");
    assert_eq!(nets[0].edge_cost, 4, "and no cost may change");
}

/// ⛔ **The two updates disagree about what "not a wire" means**, and this pins the difference.
/// The planar update skips a step that repeats both coordinates; the layered update skips a step
/// whose layer changes. A step that changes layer *while moving* is charged by the planar update
/// and skipped by the layered one.
#[test]
fn the_two_updates_skip_different_steps() {
    let n = NdrNet {
        net_id: 0,
        has_ndr: true,
        is_soft_ndr: false,
        edge_cost: 1,
        layer_edge_cost: None,
        edges: vec![NdrEdge {
            routelen: 2,
            // A pure via, then a step that both moves and changes layer.
            grids: vec![p(0, 0, 2), p(0, 0, 3), p(4, 0, 4)],
        }],
    };
    let mut planar = Grid::default();
    update_planar_net_usage(&n, 1, &mut planar);
    assert_eq!(
        planar.usage_2d.get(&('h', 0, 0)).copied().unwrap_or(0),
        1,
        "the planar update charges the moving step although its layer changed"
    );
    assert_eq!(planar.usage_2d.len(), 1, "and skips the pure via");

    let mut layered = Grid::default();
    update_net_3d_usage(&n, 1, &mut layered);
    assert!(
        layered.usage_3d.is_empty(),
        "the layered update skips BOTH steps, because both change layer"
    );
}

/// ⛔ The **vertical** branch has its own thresholds, and they are strict too.
///
/// ⚠️ Added because every other case here routes along x, so the vertical branch was never
/// entered at all — a mutation weakening its overflow test survived the whole file.
#[test]
fn the_vertical_branch_has_its_own_strict_thresholds() {
    let vertical = |id: usize| NdrNet {
        net_id: id,
        has_ndr: true,
        is_soft_ndr: false,
        edge_cost: 4,
        layer_edge_cost: Some(vec![1, 1, 4, 4]),
        edges: vec![NdrEdge {
            routelen: 2,
            grids: vec![p(5, 0, 2), p(5, 1, 2), p(5, 2, 2)],
        }],
    };

    // Zero is not congestion, on either test.
    let mut g = Grid::default();
    g.overflow_v.insert((5, 0), 0);
    g.available_v.insert((2, 5, 1), 0);
    assert!(
        congested_ndr_nets(&[vertical(0)], &g).is_empty(),
        "zero overflow and zero availability are not congestion on a vertical step"
    );

    // Overflow above zero is.
    let mut g = Grid::default();
    g.overflow_v.insert((5, 1), 1);
    assert_eq!(congested_ndr_nets(&[vertical(0)], &g), vec![0]);

    // So is negative availability, on its own.
    let mut g = Grid::default();
    g.available_v.insert((2, 5, 0), -1);
    assert_eq!(congested_ndr_nets(&[vertical(0)], &g), vec![0]);
}

/// ⛔ A **negative** step count is skipped, not walked.
///
/// ⚠️ Added because a count of exactly zero makes the guard redundant — the walk would run zero
/// times either way — so only a negative count separates the two readings. The reference's loop
/// is signed and would simply not execute; ours would convert the count to an unsigned width and
/// read far past the end, so the guard is doing more work here than there.
#[test]
fn a_negative_step_count_is_skipped() {
    let mut g = Grid::default();
    g.overflow_h.insert((0, 0), 99);
    let n = NdrNet {
        net_id: 0,
        has_ndr: true,
        is_soft_ndr: false,
        edge_cost: 4,
        layer_edge_cost: None,
        edges: vec![NdrEdge { routelen: -1, grids: vec![p(0, 0, 2), p(1, 0, 2)] }],
    };
    assert!(congested_ndr_nets(&[n.clone()], &g).is_empty(), "the scan must skip it");

    // And so do both usage updates.
    let mut sink = Grid::default();
    update_planar_net_usage(&n, 1, &mut sink);
    update_net_3d_usage(&n, 1, &mut sink);
    assert!(sink.usage_2d.is_empty() && sink.usage_3d.is_empty());
}

/// ⚠️ **One mutation survives this file and is expected to**: removing the early return when
/// nothing is congested. The loop that follows iterates the same empty list, so in *our*
/// transcription the return is pure short-circuit.
///
/// ⛔ It is not redundant in the reference, where a warning sits between the two — the early
/// return is what stops it being emitted with a count of zero. We do not model the logger, so no
/// test here can distinguish them. Recorded rather than papered over.
#[test]
fn the_early_return_is_a_short_circuit_in_this_transcription() {
    let g = Grid::default();
    let mut sink = Grid::default();
    let mut nets = vec![net(0, true, false, 4)];
    let demoted = disable_ndr_for_congested_nets(&mut nets, &g, &mut sink, true);
    assert!(demoted.is_empty());
    // The observable consequence either way: nothing written, nothing demoted.
    assert!(sink.usage_2d.is_empty() && sink.usage_3d.is_empty());
    assert_eq!(nets[0].edge_cost, 4);
}
