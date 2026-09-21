// SPDX-License-Identifier: Apache-2.0
//! R16 — the net ordering layer assignment walks in.
//!
//! 115 calls, 33,359 nets, from 71 designs in **both cost modes**.
//!
//! ⛔ Two of the six sort keys are identically zero unless the router runs resistance-aware, so
//! the plain half of this corpus decides four keys and the resistance-aware half decides all six.

use serde_json::Value;
use vyges_grt::{netpin_order_inc, NetForOrder, MIN_X_INITIAL};

struct Net {
    net_id: usize,
    edge_len: Vec<i32>,
    edge_n1_x: Vec<i16>,
    num_terminals: i32,
    has_ndr: bool,
    is_res_aware: bool,
    /// The key value as captured — already negated by the reference.
    score_key: f32,
    is_clock: bool,
    want_min_x: i32,
    want_length_per_pin: f32,
}

struct Call {
    design: String,
    ra_enabled: bool,
    nets: Vec<Net>,
    want_order: Vec<usize>,
}

fn calls() -> Vec<Call> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/netpinorder.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["calls"].as_array().expect("calls").iter().map(|c| Call {
        design: c["design"].as_str().expect("design").to_string(),
        ra_enabled: c["ra_enabled"].as_bool().expect("ra"),
        nets: c["nets"].as_array().expect("nets").iter().map(|n| Net {
            net_id: n["net_id"].as_u64().expect("id") as usize,
            edge_len: n["edge_len"].as_array().expect("len").iter()
                .map(|x| x.as_i64().expect("int") as i32).collect(),
            edge_n1_x: n["edge_n1_x"].as_array().expect("x").iter()
                .map(|x| x.as_i64().expect("int") as i16).collect(),
            num_terminals: n["num_terminals"].as_i64().expect("terms") as i32,
            has_ndr: n["has_ndr"].as_bool().expect("ndr"),
            is_res_aware: n["is_res_aware"].as_bool().expect("ra"),
            score_key: n["res_aware_score_negated"].as_f64().expect("score") as f32,
            is_clock: n["is_clock"].as_bool().expect("clk"),
            want_min_x: n["want_min_x"].as_i64().expect("xmin") as i32,
            want_length_per_pin: n["want_length_per_pin"].as_f64().expect("lpp") as f32,
        }).collect(),
        want_order: c["order"].as_array().expect("order").iter()
            .map(|x| x.as_u64().expect("id") as usize).collect(),
    }).collect()
}

fn inputs(c: &Call) -> Vec<NetForOrder<'_>> {
    c.nets.iter().map(|n| NetForOrder {
        net_id: n.net_id,
        edge_len: &n.edge_len,
        edge_n1_x: &n.edge_n1_x,
        num_terminals: n.num_terminals,
        has_ndr: n.has_ndr,
        is_res_aware: n.is_res_aware,
        // ⚠️ The capture holds the key, which the reference already negated. Negating back is
        // exact, and for a net that is not resistance-aware the value is unread either way.
        res_aware_score: if n.is_res_aware { -n.score_key } else { 0.0 },
        is_clock: n.is_clock,
    }).collect()
}

#[test]
fn the_net_order_matches_the_reference() {
    let all = calls();
    assert!(all.len() >= 100, "corpus too thin: {}", all.len());
    let (mut plain, mut aware, mut nets) = (0usize, 0usize, 0usize);

    for c in &all {
        let got = netpin_order_inc(&inputs(c), c.ra_enabled);
        let order: Vec<usize> = got.iter().map(|o| o.tree_index).collect();
        assert_eq!(
            order, c.want_order,
            "net order on {} ({} nets, resistance-aware {})",
            c.design, c.nets.len(), c.ra_enabled
        );
        // ⛔ The two intermediates are gated separately from the sort. A wrong reduction that
        // happens to preserve the order would otherwise pass.
        for o in &got {
            let n = c.nets.iter().find(|n| n.net_id == o.tree_index).expect("net");
            assert_eq!(o.min_x, n.want_min_x, "min x of net {} on {}", n.net_id, c.design);
            assert_eq!(
                o.length_per_pin, n.want_length_per_pin,
                "length per pin of net {} on {}", n.net_id, c.design
            );
        }
        if c.ra_enabled { aware += 1 } else { plain += 1 }
        nets += c.nets.len();
    }
    assert!(nets >= 20_000, "too few nets ordered: {nets}");
    // ⛔ Both modes. Without the resistance-aware half, two of the six keys are constant.
    assert!(plain >= 30, "too few default-mode calls: {plain}");
    assert!(aware >= 30, "too few resistance-aware calls: {aware}");
}

/// ⛔ Every key must be exercised by some call, or the ordering is being decided by fewer rules
/// than it looks.
#[test]
fn the_corpus_exercises_every_sort_key() {
    let all = calls();
    let count = |f: &dyn Fn(&Call) -> bool| all.iter().filter(|c| f(c)).count();

    let ndr = count(&|c| c.nets.iter().any(|n| n.has_ndr));
    let aware = count(&|c| c.nets.iter().any(|n| n.is_res_aware));
    let clock = count(&|c| c.nets.iter().any(|n| n.is_clock));
    // Ties on the earlier keys, so the later ones decide.
    let many = count(&|c| c.nets.len() > 8);

    assert!(ndr >= 5, "no call carries an NDR net: {ndr}");
    assert!(aware >= 20, "no call carries a resistance-aware net: {aware}");
    assert!(clock >= 20, "no call carries a clock net: {clock}");
    assert!(many >= 50, "too few calls large enough to tie on the early keys: {many}");

    // ⚠️ And the score must genuinely vary — a constant would leave the key unexercised even
    // where nets are flagged.
    let mut scores: Vec<f32> = all.iter().flat_map(|c| c.nets.iter())
        .filter(|n| n.is_res_aware).map(|n| n.score_key).collect();
    scores.sort_by(|a, b| a.partial_cmp(b).expect("no NaN"));
    scores.dedup();
    assert!(scores.len() >= 50, "the score barely varies: {} distinct values", scores.len());
}

/// ⛔ **No captured net has zero edges**, so the value the reduction starts from is never the
/// value that survives. Pinned by the constructed case below; this asserts the limitation.
#[test]
fn no_captured_net_is_edgeless() {
    let all = calls();
    let mut edgeless = 0usize;
    for c in &all {
        for n in &c.nets {
            if n.edge_len.is_empty() {
                edgeless += 1;
            }
        }
    }
    assert_eq!(
        edgeless, 0,
        "{edgeless} captured nets now have no edges — the starting value is reachable from a \
         capture and should be pinned by one rather than by a constructed case"
    );
}

/// ⛔ A net with no edges keeps the **narrow** type's maximum, 32,767 — not the maximum of the
/// field the value is stored in.
///
/// ⚠️ This is the difference the reference's own types make: the reduction runs in `int16_t`
/// while the record holds an `int`. Widening the reduction would put 2,147,483,647 here and sort
/// such a net differently against any net whose minimum exceeds 32,767.
#[test]
fn an_edgeless_net_keeps_the_narrow_maximum() {
    let nets = vec![NetForOrder {
        net_id: 0, edge_len: &[], edge_n1_x: &[], num_terminals: 2,
        has_ndr: false, is_res_aware: false, res_aware_score: 0.0, is_clock: false,
    }];
    let got = netpin_order_inc(&nets, false);
    assert_eq!(got[0].min_x, i32::from(MIN_X_INITIAL));
    assert_eq!(got[0].min_x, 32_767, "the reduction is not running in the narrow type");
    assert_eq!(got[0].length_per_pin, 0.0, "no edges means no length");
}

/// ⛔ The minimum is over each edge's **first** node only. An edge whose second node is further
/// left does not lower it.
#[test]
fn the_minimum_ignores_each_edges_second_node() {
    // One edge, first node at 40. Were the second node read, a smaller value would win.
    let nets = vec![NetForOrder {
        net_id: 0, edge_len: &[10], edge_n1_x: &[40], num_terminals: 2,
        has_ndr: false, is_res_aware: false, res_aware_score: 0.0, is_clock: false,
    }];
    assert_eq!(netpin_order_inc(&nets, false)[0].min_x, 40);
}

/// ⛔ Zero is the higher priority on both integer keys: an NDR net sorts before a plain one, and
/// a clock net before a non-clock one.
#[test]
fn ndr_and_clock_nets_sort_first() {
    let plain = |id: usize, ndr: bool, clk: bool| NetForOrder {
        net_id: id, edge_len: &[10], edge_n1_x: &[5], num_terminals: 2,
        has_ndr: ndr, is_res_aware: false, res_aware_score: 0.0, is_clock: clk,
    };
    // NDR outranks everything, including a clock net.
    let nets = vec![plain(0, false, true), plain(1, true, false)];
    let got = netpin_order_inc(&nets, true);
    assert_eq!(got[0].tree_index, 1, "the NDR net must come first");

    // With resistance-aware on, the clock net comes first among non-NDR nets.
    let nets = vec![plain(0, false, false), plain(1, false, true)];
    let got = netpin_order_inc(&nets, true);
    assert_eq!(got[0].tree_index, 1, "the clock net must come first");

    // ⛔ With it off the key is zero for BOTH, so the clock flag stops mattering and the tie
    // falls through to the net index.
    let got = netpin_order_inc(&nets, false);
    assert_eq!(got[0].tree_index, 0, "without the flag the clock key must not order anything");
    assert_eq!(got[0].clock, 0);
    assert_eq!(got[1].clock, 0);
}

/// ⚠️ The net's own index is the last key, so the order is total and no two nets can tie. A
/// comparator that stopped one key earlier would leave ties for the sort's stability to settle.
#[test]
fn the_net_index_breaks_every_remaining_tie() {
    let same = |id: usize| NetForOrder {
        net_id: id, edge_len: &[10], edge_n1_x: &[5], num_terminals: 2,
        has_ndr: false, is_res_aware: false, res_aware_score: 0.0, is_clock: false,
    };
    // Identical on all five earlier keys, presented in reverse index order.
    let nets = vec![same(7), same(3), same(9), same(1)];
    let got = netpin_order_inc(&nets, false);
    let order: Vec<usize> = got.iter().map(|o| o.tree_index).collect();
    assert_eq!(order, vec![1, 3, 7, 9], "the index must decide once everything else ties");
}

/// ⛔ The score key is zero for a net that is not resistance-aware, **whatever score it carries**.
///
/// ⚠️ Added because the mutation that drops that guard survived the whole corpus: the capture
/// cannot supply a score for a net the reference never computed one for, so every such net
/// arrives here carrying zero and the guard has nothing to guard. Only a constructed case can
/// present a non-zero score on an unflagged net.
#[test]
fn an_unflagged_net_scores_zero_whatever_it_carries() {
    let nets = vec![
        NetForOrder {
            net_id: 0, edge_len: &[10], edge_n1_x: &[5], num_terminals: 2,
            has_ndr: false, is_res_aware: false,
            // A score that would sort this net FIRST if the guard were dropped.
            res_aware_score: 1_000.0, is_clock: false,
        },
        NetForOrder {
            net_id: 1, edge_len: &[10], edge_n1_x: &[5], num_terminals: 2,
            has_ndr: false, is_res_aware: false, res_aware_score: 0.0, is_clock: false,
        },
    ];
    let got = netpin_order_inc(&nets, false);
    assert_eq!(got[0].res_aware_score, 0.0, "an unflagged net must score zero");
    assert_eq!(got[1].res_aware_score, 0.0);
    assert_eq!(
        got[0].tree_index, 0,
        "the unflagged net's score leaked into the key and reordered the nets"
    );
}

// ─── The two surviving mutations ────────────────────────────────────────────────────────────

/// ⛔ **Survivor 1 — computing the length per pin in double precision is a GAP with a threshold.**
///
/// The reference divides a `float` by an `int`, so the whole division happens in single
/// precision. Doing it in double and narrowing afterwards double-rounds, and the two can differ —
/// but only once the total length stops being exactly representable as a `float`, which happens
/// past 2²⁴. **No captured net comes near it**, so every total converts exactly and the two
/// forms agree on all 33,359 nets.
///
/// ⚠️ The assertion is the threshold, not the agreement: a design with a total edge length past
/// 2²⁴ would make the mutation killable and this note stale.
#[test]
fn no_captured_net_has_a_total_length_past_exact_float_range() {
    const EXACT_FLOAT_LIMIT: i64 = 1 << 24;
    let all = calls();
    let (mut worst, mut nets) = (0i64, 0usize);
    for c in &all {
        for n in &c.nets {
            let total: i64 = n.edge_len.iter().map(|x| i64::from(*x)).sum();
            worst = worst.max(total);
            nets += 1;
        }
    }
    assert!(
        worst < EXACT_FLOAT_LIMIT,
        "a net now totals {worst}, past the {EXACT_FLOAT_LIMIT} where a float stops being exact \
         — the precision of the division is observable and must be pinned by a case"
    );
    assert!(nets >= 20_000, "too few nets to call the range unreached: {nets}");
}

/// ⛔ **Survivor 2 — the sort's stability cannot matter, because the order is TOTAL.**
///
/// The last key is the net's own index and the indices within a call are distinct, so no two
/// records ever compare equal and there is nothing for a stable sort to preserve. Replacing the
/// stable sort with an unstable one is therefore equivalent — on this corpus and on any input
/// with distinct net indices.
///
/// ⚠️ It is still transcribed as a stable sort: the reference asks for one, and a later key
/// change — or a caller that presented a net twice — would make it load-bearing again. This test
/// asserts the precondition that makes the survivor harmless.
#[test]
fn every_call_presents_each_net_once() {
    for c in &calls() {
        let mut ids: Vec<usize> = c.nets.iter().map(|n| n.net_id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(
            ids.len(), before,
            "{} presents a net more than once — two records can now compare equal and the \
             sort's stability decides the result",
            c.design
        );
    }
}
