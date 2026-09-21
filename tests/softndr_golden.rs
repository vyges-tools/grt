// SPDX-License-Identifier: Apache-2.0
//! R17's helpers, gated against the reference through the caller that DOES fire.
//!
//! ⛔ R17's own gate never fires (finding 8), so the rules in `softndr.rs` were first written from
//! the source and pinned by constructed cases only. But `applySoftNDR` and the planar usage
//! update are **also** called from the congestion loop's escalation path, which does fire — so
//! the same probe run there gives a real golden for the shared helpers.
//!
//! 8 demotions and **328 planar writes** from 2 designs. Thin, but it is the reference's own
//! answer rather than a reading of its source, and it caught nothing — which is the result worth
//! having.

use serde_json::Value;
use vyges_grt::{apply_soft_ndr, NdrEdge, NdrNet, Point3D, UsageGrid};

/// Records the writes in order, rather than accumulating them — the ORDER is what shows the
/// refund using the old cost and the charge using the new one.
#[derive(Default)]
struct Recorder {
    writes: Vec<(char, i16, i16, i32)>,
}

impl UsageGrid for Recorder {
    fn add_usage_v_2d(&mut self, x: i16, y: i16, d: i32) {
        self.writes.push(('v', x, y, d));
    }
    fn add_usage_h_2d(&mut self, x: i16, y: i16, d: i32) {
        self.writes.push(('h', x, y, d));
    }
    // ⚠️ The escalation path does not touch the layered grid — only R17's caller does, and that
    // one never fires. A write here would mean our transcription reaches further than the
    // reference's does on this path.
    fn add_usage_v_3d(&mut self, _l: i16, _x: i16, _y: i16, _d: i32) {
        panic!("the escalation path must not write layered usage");
    }
    fn add_usage_h_3d(&mut self, _l: i16, _x: i16, _y: i16, _d: i32) {
        panic!("the escalation path must not write layered usage");
    }
}

struct Case {
    design: String,
    net_id: usize,
    cost_before: i8,
    soft_before: bool,
    has_ndr: bool,
    layer_cost: Vec<i8>,
    edges: Vec<NdrEdge>,
    want_writes: Vec<(char, i16, i16, i32)>,
    cost_after: i8,
    soft_after: bool,
}

fn cases() -> Vec<Case> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/softndr.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["demotions"].as_array().expect("demotions").iter().map(|c| Case {
        design: c["design"].as_str().expect("design").to_string(),
        net_id: c["net_id"].as_u64().expect("id") as usize,
        cost_before: c["cost_before"].as_i64().expect("cost") as i8,
        soft_before: c["soft_before"].as_bool().expect("soft"),
        has_ndr: c["has_ndr"].as_bool().expect("ndr"),
        layer_cost: c["layer_cost"].as_array().expect("lc").iter()
            .map(|x| x.as_i64().expect("int") as i8).collect(),
        edges: c["edges"].as_array().expect("edges").iter().map(|e| NdrEdge {
            len: e["len"].as_i64().expect("len") as i32,
            routelen: e["routelen"].as_i64().expect("rl") as i32,
            grids: e["grids"].as_array().expect("g").iter().map(|p| {
                let a = p.as_array().expect("triple");
                Point3D {
                    x: a[0].as_i64().expect("x") as i16,
                    y: a[1].as_i64().expect("y") as i16,
                    layer: a[2].as_i64().expect("l") as i16,
                }
            }).collect(),
        }).collect(),
        want_writes: c["writes"].as_array().expect("w").iter().map(|w| {
            let a = w.as_array().expect("quad");
            (
                a[0].as_str().expect("dir").chars().next().expect("char"),
                a[1].as_i64().expect("x") as i16,
                a[2].as_i64().expect("y") as i16,
                a[3].as_i64().expect("d") as i32,
            )
        }).collect(),
        cost_after: c["cost_after"].as_i64().expect("ca") as i8,
        soft_after: c["soft_after"].as_bool().expect("sa"),
    }).collect()
}

#[test]
fn the_demotion_writes_match_the_reference_write_for_write() {
    let all = cases();
    assert!(all.len() >= 5, "corpus too thin: {}", all.len());
    let mut writes = 0usize;

    for c in &all {
        let mut net = NdrNet {
            net_id: c.net_id,
            has_ndr: c.has_ndr,
            is_soft_ndr: c.soft_before,
            edge_cost: c.cost_before,
            layer_edge_cost: Some(c.layer_cost.clone()),
            edges: c.edges.clone(),
        };
        let mut rec = Recorder::default();
        apply_soft_ndr(&mut net, &mut rec);

        assert_eq!(
            rec.writes, c.want_writes,
            "planar writes for net {} on {} — {} of ours against {} of the reference's",
            c.net_id, c.design, rec.writes.len(), c.want_writes.len()
        );
        assert_eq!(net.edge_cost, c.cost_after, "edge cost after on {}", c.design);
        assert_eq!(net.is_soft_ndr, c.soft_after, "soft flag after on {}", c.design);
        writes += rec.writes.len();
    }
    assert!(writes >= 300, "too few writes to be worth much: {writes}");
}

/// ⛔ The reference refunds the whole route at the old cost **before** charging any of it at the
/// new one. This asserts that shape on the captured sequences rather than on a constructed one:
/// the first half of every write list is negative and the second half positive, and the two halves
/// cover the same positions in the same order.
#[test]
fn every_capture_shows_a_refund_pass_then_a_charge_pass() {
    for c in &cases() {
        let w = &c.want_writes;
        assert_eq!(w.len() % 2, 0, "on {} the write count is odd", c.design);
        let half = w.len() / 2;
        let (refund, charge) = w.split_at(half);

        for (r, ch) in refund.iter().zip(charge.iter()) {
            assert_eq!((r.0, r.1, r.2), (ch.0, ch.1, ch.2),
                       "on {} the two passes visit different positions", c.design);
            assert_eq!(r.3, -i32::from(c.cost_before),
                       "on {} the refund is not at the ORIGINAL cost", c.design);
            assert_eq!(ch.3, i32::from(c.cost_after),
                       "on {} the charge is not at the DEMOTED cost", c.design);
        }
        // ⚠️ And the two costs must actually differ, or the bracket proves nothing.
        assert_ne!(c.cost_before, c.cost_after,
                   "on {} the demotion did not change the cost", c.design);
    }
}

/// ⚠️ **What this corpus cannot decide.** Every captured demotion starts from the same edge cost
/// and none exercises the layered bracket, because the caller that fires does not use it.
/// Asserted so the limits of the gate above are explicit rather than implied.
#[test]
fn the_reachable_path_exercises_only_part_of_the_cluster() {
    let all = cases();
    let costs: std::collections::HashSet<i8> = all.iter().map(|c| c.cost_before).collect();
    assert_eq!(
        costs.len(), 1,
        "more than one starting cost is now captured ({costs:?}) — the gate is broader than \
         this note says"
    );
    assert!(
        all.iter().all(|c| c.soft_before == false),
        "a net was demoted twice — the reference should have skipped it"
    );
    // ⚠️ The layered bracket and the congestion scan of R17 remain pinned by constructed cases
    // only; see tests/softndr.rs and finding 8.
}
