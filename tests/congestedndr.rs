// SPDX-License-Identifier: Apache-2.0
//! The congestion loop's NDR selection — the third congestion scan in this engine.
//!
//! 2 calls, 12 nets, **257 steps read**, from the two designs whose escalation path fires.
//!
//! ⛔ **Its rules differ from R17's scan in four ways**, and the golden pins all four: it tests
//! every step including a layer change, reads only the planar overflow, never stops early, and
//! gates an edge on its length OR its step count rather than the step count alone. **11 captured
//! edges are skipped by that gate.**

use serde_json::Value;
use std::collections::HashMap;
use vyges_grt::{
    compute_congested_ndr_nets, congested_ndr_nets_by_fraction, sort_congested_ndr_nets,
    CongestedNdr, NdrEdge, NdrNet, Overflow2D, Point3D,
};

/// The overflow the scan reads, keyed by the step it reads it at.
///
/// ⚠️ Built from the capture's own per-step values rather than from a grid, because the capture
/// records what the reference read, not the grid behind it. A step our scan does not visit is one
/// the reference did, and vice versa — so a missing key is a divergence, not a default.
struct Reads {
    by_pos: HashMap<(char, i16, i16), i32>,
}

impl Overflow2D for Reads {
    fn overflow_v(&self, x: i16, y: i16) -> i32 {
        *self.by_pos.get(&('v', x, y)).unwrap_or_else(|| {
            panic!("our scan read a vertical step at ({x},{y}) that the reference never read")
        })
    }
    fn overflow_h(&self, x: i16, y: i16) -> i32 {
        *self.by_pos.get(&('h', x, y)).unwrap_or_else(|| {
            panic!("our scan read a horizontal step at ({x},{y}) that the reference never read")
        })
    }
}

struct Call {
    design: String,
    nets: Vec<NdrNet>,
    want_counts: Vec<(usize, u16)>,
    want_sorted: Vec<usize>,
    want_fractions: Vec<(f64, Vec<usize>)>,
    reads: Reads,
    skipped_edges: usize,
    steps: usize,
}

fn calls() -> Vec<Call> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/congestedndr.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["calls"].as_array().expect("calls").iter().map(|c| {
        let mut by_pos = HashMap::new();
        let mut skipped_edges = 0usize;
        let mut steps = 0usize;
        let mut nets = Vec::new();
        let mut want_counts = Vec::new();

        for n in c["nets"].as_array().expect("nets") {
            let mut edges = Vec::new();
            for e in n["edges"].as_array().expect("edges") {
                let grids: Vec<Point3D> = e["grids"].as_array().expect("g").iter().map(|p| {
                    let a = p.as_array().expect("triple");
                    Point3D {
                        x: a[0].as_i64().expect("x") as i16,
                        y: a[1].as_i64().expect("y") as i16,
                        layer: a[2].as_i64().expect("l") as i16,
                    }
                }).collect();
                let len = e["len"].as_i64().expect("len") as i32;
                let routelen = e["routelen"].as_i64().expect("rl") as i32;
                if !(len > 0 || routelen > 0) {
                    skipped_edges += 1;
                }
                for (i, o) in e["overflow"].as_array().expect("o").iter().enumerate() {
                    let a = o.as_array().expect("pair");
                    let dir = a[0].as_str().expect("dir").chars().next().expect("ch");
                    let value = a[1].as_i64().expect("val") as i32;
                    let (p0, p1) = (grids[i], grids[i + 1]);
                    let key = if dir == 'v' {
                        ('v', p0.x, p0.y.min(p1.y))
                    } else {
                        ('h', p0.x.min(p1.x), p0.y)
                    };
                    by_pos.insert(key, value);
                    steps += 1;
                }
                edges.push(NdrEdge { len, routelen, grids });
            }
            let net_id = n["net_id"].as_u64().expect("id") as usize;
            want_counts.push((net_id, n["want_count"].as_u64().expect("c") as u16));
            nets.push(NdrNet {
                net_id,
                has_ndr: n["has_ndr"].as_bool().expect("ndr"),
                is_soft_ndr: n["is_soft_ndr"].as_bool().expect("soft"),
                edge_cost: 1,
                layer_edge_cost: None,
                edges,
            });
        }

        let mut want_fractions: Vec<(f64, Vec<usize>)> = c["fractions"].as_object()
            .expect("fractions").iter()
            .map(|(k, v)| (
                k.parse::<f64>().expect("fraction"),
                v.as_array().expect("ids").iter()
                    .map(|x| x.as_u64().expect("id") as usize).collect(),
            )).collect();
        want_fractions.sort_by(|a, b| a.0.partial_cmp(&b.0).expect("ordered"));

        Call {
            design: c["design"].as_str().expect("design").to_string(),
            nets,
            want_counts,
            want_sorted: c["sorted"].as_array().expect("sorted").iter()
                .map(|x| x.as_u64().expect("id") as usize).collect(),
            want_fractions,
            reads: Reads { by_pos },
            skipped_edges,
            steps,
        }
    }).collect()
}

#[test]
fn the_congested_edge_counts_match_the_reference() {
    let all = calls();
    assert!(!all.is_empty(), "no calls captured");
    let (mut nets, mut steps, mut skipped, mut counted) = (0usize, 0usize, 0usize, 0usize);

    for c in &all {
        let got = compute_congested_ndr_nets(&c.nets, &c.reads);
        // Every net the reference counted, with its count.
        let want: Vec<CongestedNdr> = c.want_counts.iter()
            .filter(|(_, n)| *n > 0)
            .map(|(id, n)| CongestedNdr { net_id: *id, num_edges: *n })
            .collect();
        assert_eq!(got, want, "congested counts on {}", c.design);
        nets += c.nets.len();
        steps += c.steps;
        skipped += c.skipped_edges;
        counted += got.len();
    }
    assert!(nets >= 10, "too few nets: {nets}");
    assert!(steps >= 200, "too few steps read: {steps}");
    // ⛔ The edge gate must actually reject something, or it is untested.
    assert!(skipped >= 5, "the edge gate skips nothing: {skipped}");
    assert!(counted >= 5, "too few nets counted: {counted}");
}

#[test]
fn the_order_and_the_fractions_match_the_reference() {
    for c in &calls() {
        let mut got = compute_congested_ndr_nets(&c.nets, &c.reads);
        sort_congested_ndr_nets(&mut got);
        let order: Vec<usize> = got.iter().map(|n| n.net_id).collect();
        assert_eq!(order, c.want_sorted, "sorted order on {}", c.design);

        // ⚠️ Probed at fractions the router never passes, because the selector is pure and
        // asking it directly is the only way to learn what the clamp and the rounding do.
        for (fraction, want) in &c.want_fractions {
            assert_eq!(
                congested_ndr_nets_by_fraction(&got, *fraction), *want,
                "fraction {fraction} on {}", c.design
            );
        }
        assert!(c.want_fractions.len() >= 4, "too few fractions probed");
    }
}

/// ⛔ **The sort's instability cannot bite here, because no call has a tie** — asserted, because
/// the reference uses an unstable sort over a single-key comparator with no tie-break, the only
/// ordering in this engine that is not stable. Recorded as finding 9.
#[test]
fn no_call_has_two_nets_with_the_same_count() {
    for c in &calls() {
        let mut counts: Vec<u16> = c.want_counts.iter()
            .map(|(_, n)| *n).filter(|n| *n > 0).collect();
        let before = counts.len();
        counts.sort_unstable();
        counts.dedup();
        assert_eq!(
            counts.len(), before,
            "{} now has two NDR nets with the same congested-edge count — the unstable sort's \
             order becomes unspecified and our answer may differ from the reference's",
            c.design
        );
    }
}

// ─── The two scans, side by side ────────────────────────────────────────────────────────────

/// ⛔ **This scan counts a step that changes layer; R17's skips it.** The clearest single
/// difference between the two, and neither is written in terms of the other.
#[test]
fn this_scan_counts_a_layer_change_that_the_other_skips() {
    use vyges_grt::{congested_ndr_nets, CongestionView};

    struct Both {
        overflow: i32,
    }
    impl Overflow2D for Both {
        fn overflow_v(&self, _x: i16, _y: i16) -> i32 { self.overflow }
        fn overflow_h(&self, _x: i16, _y: i16) -> i32 { self.overflow }
    }
    impl CongestionView for Both {
        fn overflow_v(&self, _x: i16, _y: i16) -> i32 { self.overflow }
        fn overflow_h(&self, _x: i16, _y: i16) -> i32 { self.overflow }
        fn available_v(&self, _l: i16, _x: i16, _y: i16) -> i32 { 100 }
        fn available_h(&self, _l: i16, _x: i16, _y: i16) -> i32 { 100 }
    }

    // A single step that moves in x AND changes layer, on a congested edge.
    let net = NdrNet {
        net_id: 0,
        has_ndr: true,
        is_soft_ndr: false,
        edge_cost: 4,
        layer_edge_cost: None,
        edges: vec![NdrEdge {
            len: 1,
            routelen: 1,
            grids: vec![Point3D { x: 0, y: 0, layer: 2 }, Point3D { x: 1, y: 0, layer: 3 }],
        }],
    };
    let view = Both { overflow: 5 };

    assert_eq!(
        compute_congested_ndr_nets(&[net.clone()], &view),
        vec![CongestedNdr { net_id: 0, num_edges: 1 }],
        "this scan must count the step although its layer changes"
    );
    assert!(
        congested_ndr_nets(&[net], &view).is_empty(),
        "R17's scan must skip it as a via"
    );
}

/// ⛔ **The edge gates differ too.** An edge with a positive length but no steps is admitted here
/// and skipped by R17's scan — which is moot for the count, since it has no steps to walk, but
/// the gates are not the same test and a reader must not assume they are.
#[test]
fn the_edge_gates_are_not_the_same_test() {
    struct Zero;
    impl Overflow2D for Zero {
        fn overflow_v(&self, _x: i16, _y: i16) -> i32 { 0 }
        fn overflow_h(&self, _x: i16, _y: i16) -> i32 { 0 }
    }
    // len > 0 admits it here; routelen == 0 would reject it in R17's scan.
    let net = NdrNet {
        net_id: 0,
        has_ndr: true,
        is_soft_ndr: false,
        edge_cost: 4,
        layer_edge_cost: None,
        edges: vec![NdrEdge { len: 7, routelen: 0, grids: vec![Point3D { x: 0, y: 0, layer: 0 }] }],
    };
    assert!(
        compute_congested_ndr_nets(&[net], &Zero).is_empty(),
        "no steps means no count, whichever gate admitted the edge"
    );
}

/// ⚠️ The fraction is clamped and rounded up, so anything above zero takes at least one net and
/// anything at or below zero takes none. Probed beyond the captured range.
#[test]
fn the_fraction_is_clamped_and_rounded_up() {
    let nets: Vec<CongestedNdr> = (0..4)
        .map(|i| CongestedNdr { net_id: i, num_edges: 10 - i as u16 })
        .collect();
    assert_eq!(congested_ndr_nets_by_fraction(&nets, -1.0), Vec::<usize>::new());
    assert_eq!(congested_ndr_nets_by_fraction(&nets, 0.0), Vec::<usize>::new());
    assert_eq!(congested_ndr_nets_by_fraction(&nets, 0.01), vec![0], "rounds up to one");
    assert_eq!(congested_ndr_nets_by_fraction(&nets, 0.5), vec![0, 1]);
    assert_eq!(congested_ndr_nets_by_fraction(&nets, 1.0), vec![0, 1, 2, 3]);
    assert_eq!(
        congested_ndr_nets_by_fraction(&nets, 99.0), vec![0, 1, 2, 3],
        "a fraction above one is clamped, not multiplied out"
    );
}

/// ⛔ An edge with **no length but real steps** is walked — so the gate's `routelen` term is
/// load-bearing.
///
/// ⚠️ Added because the captured edges all have both terms positive or both zero, so the `or`
/// never decides anything in the corpus: the 11 skipped edges have length 0 **and** no steps.
#[test]
fn an_edge_with_no_length_but_real_steps_is_still_walked() {
    struct Congested;
    impl Overflow2D for Congested {
        fn overflow_v(&self, _x: i16, _y: i16) -> i32 { 9 }
        fn overflow_h(&self, _x: i16, _y: i16) -> i32 { 9 }
    }
    let net = NdrNet {
        net_id: 0,
        has_ndr: true,
        is_soft_ndr: false,
        edge_cost: 4,
        layer_edge_cost: None,
        edges: vec![NdrEdge {
            len: 0,
            routelen: 2,
            grids: vec![
                Point3D { x: 0, y: 0, layer: 0 },
                Point3D { x: 1, y: 0, layer: 0 },
                Point3D { x: 2, y: 0, layer: 0 },
            ],
        }],
    };
    assert_eq!(
        compute_congested_ndr_nets(&[net], &Congested),
        vec![CongestedNdr { net_id: 0, num_edges: 2 }],
        "the edge must be walked on its step count although its length is zero"
    );
}

/// ⛔ A net already demoted to soft NDR is skipped, so the scan cannot re-select it.
///
/// ⚠️ Added because every captured net is un-demoted at scan time — the escalation path recomputes
/// the list before demoting anything, so the corpus never shows a soft net being skipped.
#[test]
fn an_already_demoted_net_is_not_counted() {
    struct Congested;
    impl Overflow2D for Congested {
        fn overflow_v(&self, _x: i16, _y: i16) -> i32 { 9 }
        fn overflow_h(&self, _x: i16, _y: i16) -> i32 { 9 }
    }
    let edge = NdrEdge {
        len: 1,
        routelen: 1,
        grids: vec![Point3D { x: 0, y: 0, layer: 0 }, Point3D { x: 1, y: 0, layer: 0 }],
    };
    let soft = NdrNet {
        net_id: 0, has_ndr: true, is_soft_ndr: true, edge_cost: 1,
        layer_edge_cost: None, edges: vec![edge.clone()],
    };
    let hard = NdrNet {
        net_id: 1, has_ndr: true, is_soft_ndr: false, edge_cost: 4,
        layer_edge_cost: None, edges: vec![edge],
    };
    assert_eq!(
        compute_congested_ndr_nets(&[soft, hard], &Congested),
        vec![CongestedNdr { net_id: 1, num_edges: 1 }],
        "only the un-demoted net may be counted"
    );
}

/// ⚠️ **Two mutations survive this file, and both are equivalent rather than untested.**
///
/// 1. Dropping the gate's `len > 0` term changes nothing **anywhere**, not just here: the term can
///    only admit an edge whose step count is zero or less, and such an edge has no steps to walk.
///    It is redundant in the reference too.
/// 2. Dropping the fraction's clamp changes nothing **in Rust**, because a float-to-integer cast
///    saturates here — a negative product becomes zero and an oversized one is capped by the
///    `min` that follows. ⛔ In the reference that cast is undefined behaviour for a negative
///    value, so the clamp is load-bearing there and is transcribed for that reason, not this one.
///
/// This test states both so neither is mistaken for a gap that more capture would close.
#[test]
fn the_two_equivalent_mutations_are_equivalent_for_stated_reasons() {
    struct Congested;
    impl Overflow2D for Congested {
        fn overflow_v(&self, _x: i16, _y: i16) -> i32 { 9 }
        fn overflow_h(&self, _x: i16, _y: i16) -> i32 { 9 }
    }
    // (1) An edge admitted only by its length has no steps, so it contributes nothing.
    let net = NdrNet {
        net_id: 0, has_ndr: true, is_soft_ndr: false, edge_cost: 4, layer_edge_cost: None,
        edges: vec![NdrEdge { len: 99, routelen: 0, grids: vec![Point3D { x: 0, y: 0, layer: 0 }] }],
    };
    assert!(compute_congested_ndr_nets(&[net], &Congested).is_empty());

    // (2) The saturating cast, demonstrated: a negative fraction yields nothing either way.
    let nets: Vec<CongestedNdr> = (0..3)
        .map(|i| CongestedNdr { net_id: i, num_edges: 3 - i as u16 })
        .collect();
    assert_eq!(congested_ndr_nets_by_fraction(&nets, -5.0), Vec::<usize>::new());
    assert_eq!((-15.0_f64) as usize, 0, "the cast saturates rather than wrapping");
}

/// ⛔ **The counter's width is unobservable, and by a wide margin.** The reference counts congested
/// steps into a `uint16_t`, so a net with more than 65,535 of them wraps. The largest count in the
/// corpus is **25** — four orders of magnitude short — and a net would need a route longer than
/// any grid here to reach it.
///
/// ⚠️ Transcribed as `u16` anyway, because the width is the reference's and a wider counter is a
/// divergence whether or not a design reaches it. This asserts the distance so the claim is a
/// measurement rather than a shrug.
#[test]
fn no_captured_count_comes_near_the_counter_width() {
    let all = calls();
    let worst = all.iter()
        .flat_map(|c| c.want_counts.iter().map(|(_, n)| *n))
        .max()
        .expect("counts");
    assert!(
        worst < 1_000,
        "a net now counts {worst} congested steps — close enough to the 16-bit wrap that the \
         counter's width may become observable"
    );
    // ⚠️ And the counter must be doing real work, or "it never wraps" is vacuous.
    assert!(worst >= 10, "the largest count is only {worst}");
}
