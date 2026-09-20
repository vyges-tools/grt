// SPDX-License-Identifier: Apache-2.0
//! R14 — the per-net loop, and the retry it can take.
//!
//! 40 passes from four designs, **19,800 net visits**. Each pass carries the nets it entered **in
//! order**, so a retry would show as a repeat at the same index — a count of visits could not
//! tell "processed every net once" from "skipped one and retried another".
//!
//! ⛔ **No pass retries, on any design.** A surgery that cannot place a contact point rebuilds the
//! net's tree and reprocesses it. Swept across **all 148 traceable designs — 713,915 net visits,
//! zero retries.** The recovery is transcribed from the reference and pinned by constructed
//! cases, and the absence is asserted so a recapture that reaches it fails loudly.

use serde_json::Value;
use vyges_grt::{maze_route_pass, AfterEdge};

struct Pass {
    design: String,
    net_count: usize,
    visits: Vec<(usize, usize)>,
    retries: Vec<(usize, usize)>,
}

fn passes() -> Vec<Pass> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/netloop.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let pairs = |v: &Value| -> Vec<(usize, usize)> {
        v.as_array().expect("pairs").iter()
            .map(|p| (p[0].as_u64().expect("a") as usize, p[1].as_u64().expect("b") as usize))
            .collect()
    };
    v["passes"].as_array().expect("passes").iter().map(|p| Pass {
        design: p["design"].as_str().expect("design").to_string(),
        net_count: p["net_count"].as_u64().expect("count") as usize,
        visits: pairs(&p["visits"]),
        retries: pairs(&p["retries"]),
    }).collect()
}

#[test]
fn the_loop_visits_every_net_once_in_order() {
    let all = passes();
    assert!(all.len() >= 30, "corpus too thin: {}", all.len());
    let mut visits = 0usize;

    for p in &all {
        let order: Vec<usize> = p.visits.iter().map(|(_, id)| *id).collect();
        let got = maze_route_pass(&order, |_| AfterEdge::Continue);
        assert_eq!(got, order, "visit order on {}", p.design);

        // ⚠️ The index the reference reports must advance by one each time, or a retry went
        // unrecorded.
        for (k, (idx, _)) in p.visits.iter().enumerate() {
            assert_eq!(*idx, k, "net index jumped on {} — an unrecorded retry", p.design);
        }
        assert_eq!(p.visits.len(), p.net_count, "a net was skipped on {}", p.design);
        visits += p.visits.len();
    }
    assert!(visits >= 10_000, "too few visits to be a gate: {visits}");
}

/// ⛔ Asserted as the absence it is, so a recapture that reaches the retry fails loudly.
#[test]
fn no_captured_pass_rebuilds_a_net() {
    for p in &passes() {
        assert!(
            p.retries.is_empty(),
            "{} now retries {} net(s) — the retry is reachable after all, and the notes on it \
             need revisiting", p.design, p.retries.len()
        );
    }
}

/// The retry reprocesses the **same** net, and the loop's own increment is what returns to it.
///
/// ⛔ **Constructed: no design reaches this.** The reference steps its net index back before
/// breaking out of the edge loop, so the index does not move — which is easy to write as "skip to
/// the next net" by mistake, and nothing in the corpus would catch that.
#[test]
fn a_rebuild_reprocesses_the_same_net() {
    let order = vec![7, 9, 4];
    let mut seen: Vec<usize> = Vec::new();
    let visited = maze_route_pass(&order, |net| {
        seen.push(net);
        // Fail the middle net exactly once.
        if net == 9 && seen.iter().filter(|n| **n == 9).count() == 1 {
            AfterEdge::RebuildAndRetry
        } else {
            AfterEdge::Continue
        }
    });
    assert_eq!(visited, vec![7, 9, 9, 4], "the failing net is entered again, in place");
    assert_eq!(visited.last(), Some(&4), "and the loop then carries on past it");
}

/// ⚠️ Nothing bounds the retry: a net that fails the same way every time is reprocessed forever.
///
/// The reference relies on the rebuilt tree differing from the one that failed. Pinned here so
/// the absence of a bound is a recorded property rather than an oversight.
#[test]
fn the_retry_is_unbounded_by_construction() {
    let order = vec![3];
    let mut attempts = 0usize;
    let visited = maze_route_pass(&order, |_| {
        attempts += 1;
        // Give up after a few, standing in for a tree that eventually differs.
        if attempts < 5 { AfterEdge::RebuildAndRetry } else { AfterEdge::Continue }
    });
    assert_eq!(visited, vec![3; 5], "the same net five times, with nothing stopping it sooner");
}
