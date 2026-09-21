// SPDX-License-Identifier: Apache-2.0
//! R18a — `mazeRouteMSMDOrder3D`'s control sequence.
//!
//! Golden `maze3d_control.json`, both cost modes: per call the window, mode and prelude values,
//! the order walked, and every step (net reached or skipped, edge checked, pass ended) in order.
//!
//! The per-edge work (R18b–h) is not built yet, so the replay supplies it from the trace: each
//! edge's length as the reference read it, and each pass's retry list. What is tested is the
//! driver — which nets, which edges, in what order, what the window says, when the loop stops.

use std::collections::VecDeque;

use serde_json::Value;
use vyges_grt::{
    edge_in_window, end_index, maze_route_msmd_order_3d, prelude, skips_for_slack, EdgeResult,
    Maze3DCall, Maze3DEdgeWork, Maze3DEvent, OrderedNet, FINAL_RES_AWARE_NETS_PERCENTAGE,
    HIGH_DETOUR_PENALTY, LOW_DETOUR_PENALTY,
};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn golden() -> Value {
    read(&format!("{}/examples/grt_gate/maze3d_control.json", env!("CARGO_MANIFEST_DIR")))
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

/// The per-edge work, played back from the trace.
///
/// `edge_len` pops the next captured edge check — and asserts it is the edge the driver asked
/// for, so any divergence in walk order fails at once. Each reached net carries its passes as
/// (checks in the pass, the pass's retry list); the list is handed back by the pass's first
/// in-window edge (the driver de-duplicates the union, so where it arrives does not matter).
struct Playback {
    nets: VecDeque<Vec<(usize, Vec<usize>)>>,
    passes: Vec<(usize, Vec<usize>)>,
    pass: usize,
    checked: usize,
    owe_retry: bool,
    checks: VecDeque<(usize, i32)>,
}

impl Maze3DEdgeWork for Playback {
    fn num_edges(&self, _net: usize) -> usize {
        self.passes[0].0
    }
    fn begin_net(&mut self, _net: usize, _res_aware: Option<bool>) {
        self.passes = self.nets.pop_front().expect("a reached net");
        (self.pass, self.checked, self.owe_retry) = (0, 0, true);
    }
    fn edge_len(&mut self, _net: usize, edge: usize) -> i32 {
        if self.checked == self.passes[self.pass].0 {
            (self.pass, self.checked, self.owe_retry) = (self.pass + 1, 0, true);
        }
        self.checked += 1;
        let (want, len) = self.checks.pop_front().expect("the reference checked another edge");
        assert_eq!(edge, want, "the driver checked a different edge");
        len
    }
    fn route_edge(&mut self, _net: usize, _edge: usize) -> EdgeResult {
        if std::mem::take(&mut self.owe_retry) {
            return EdgeResult { retry: self.passes[self.pass].1.clone(), recovered: false };
        }
        EdgeResult::default()
    }
}

fn replay(g: &Value) -> usize {
    let calls = g["calls"].as_array().expect("calls");
    let mut steps = 0;
    for c in calls {
        let call = Maze3DCall {
            ripup_lb: int(&c["lb"]) as i32,
            ripup_ub: int(&c["ub"]) as i32,
            resistance_aware: int(&c["ra"]) == 1,
            incremental: int(&c["incr"]) == 1,
        };
        let order: Vec<OrderedNet> = c["order"].as_array().expect("order").iter().map(|o| OrderedNet {
            net_id: int(&o[0]) as usize,
            slack: f32::from_bits(o[1].as_u64().expect("bits") as u32),
            res_aware: int(&o[2]) == 1,
        }).collect();
        let events = c["events"].as_array().expect("events");

        // What the reference walked, rebuilt as the driver's own event type — and, per reached
        // net, its passes as (checks, retry list) for the playback.
        let mut want = Vec::new();
        let mut checks = VecDeque::new();
        let mut nets: VecDeque<Vec<(usize, Vec<usize>)>> = VecDeque::new();
        let mut in_pass = 0usize;
        for e in events {
            match e[0].as_str().expect("tag") {
                "N" => {
                    let skipped = int(&e[2]) == 1;
                    if !skipped {
                        nets.push_back(Vec::new());
                        in_pass = 0;
                    }
                    want.push(Maze3DEvent::Net { net: int(&e[1]) as usize, skipped });
                }
                "E" => {
                    let (edge, len) = (int(&e[1]) as usize, int(&e[2]) as i32);
                    checks.push_back((edge, len));
                    in_pass += 1;
                    want.push(Maze3DEvent::Edge { edge, len, in_window: int(&e[3]) == 1 });
                }
                "P" => {
                    let retry: Vec<usize> =
                        e[3].as_array().expect("retry").iter().map(|v| int(v) as usize).collect();
                    nets.back_mut().expect("a pass inside a net").push((in_pass, retry.clone()));
                    in_pass = 0;
                    want.push(Maze3DEvent::Pass { iter: int(&e[1]) as i32, stop: int(&e[2]) == 1, retry });
                }
                t => panic!("tag {t}"),
            }
        }
        let mut play =
            Playback { nets, passes: Vec::new(), pass: 0, checked: 0, owe_retry: false, checks };

        let (got, recovered) = maze_route_msmd_order_3d(&call, &order, &mut play);
        let who = format!("{} ub={}", c["design"], call.ripup_ub);
        assert_eq!(got.len(), want.len(), "{who}: step count");
        for (i, (g, w)) in got.iter().zip(&want).enumerate() {
            assert_eq!(g, w, "{who}: step {i}");
        }
        assert!(play.checks.is_empty(), "{who}: the reference checked more edges");
        assert_eq!(recovered as i64, int(&c["recovered"]), "{who}: recovered nets");
        assert_eq!(end_index(order.len(), call.resistance_aware) as i64, int(&c["end_ind"]), "{who}: endIND");
        steps += got.len();
    }
    steps
}

/// Every captured call's walk, step for step.
#[test]
fn the_maze3d_walk_matches_the_reference() {
    let steps = replay(&golden());
    assert!(steps >= 20_000, "too few steps: {steps}");
}

/// GRT_MAZE3D_CONTROL_FULL=/path/to/m3a-all.json cargo test --test maze3d -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_maze3d_walk_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_MAZE3D_CONTROL_FULL").expect("set GRT_MAZE3D_CONTROL_FULL");
    eprintln!("exhaustive: {} steps", replay(&read(&path)));
}

/// The golden carries the rare shapes, so the replay above is not blind to them.
#[test]
fn the_golden_witnesses_retries_slack_skips_and_a_fixed_percentage() {
    let g = golden();
    let calls = g["calls"].as_array().expect("calls");
    let retried = calls.iter().any(|c| {
        c["events"].as_array().expect("e").iter().any(|e| e[0] == "P" && int(&e[2]) == 0)
    });
    let skipped = calls.iter().any(|c| {
        c["events"].as_array().expect("e").iter().any(|e| e[0] == "N" && int(&e[2]) == 1)
    });
    let fixed_pct = calls.iter().any(|c| {
        int(&c["ra"]) == 1 && f32::from_bits(int(&c["pct"]) as u32) != FINAL_RES_AWARE_NETS_PERCENTAGE
    });
    assert!(retried && skipped && fixed_pct, "retried={retried} skipped={skipped} fixed={fixed_pct}");
    // The prelude's recorded detour penalty follows the mode, as `prelude` says.
    for c in calls {
        let call = Maze3DCall {
            ripup_lb: 0,
            ripup_ub: 0,
            resistance_aware: int(&c["ra"]) == 1,
            incremental: int(&c["incr"]) == 1,
        };
        if let Some(d) = prelude(&call, false).detour_penalty {
            assert_eq!(int(&c["detour"]) as i32, d, "{}", c["design"]);
        }
    }
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn call(ra: bool, incr: bool) -> Maze3DCall {
    Maze3DCall { ripup_lb: 0, ripup_ub: 40, resistance_aware: ra, incremental: incr }
}

/// ⛔ 90% through a double, TRUNCATED; all of it in resistance-aware mode.
#[test]
fn end_index_is_ninety_percent_truncated() {
    assert_eq!(end_index(495, false), 445);
    assert_eq!(end_index(10, false), 9);
    assert_eq!(end_index(9, false), 8);
    assert_eq!(end_index(1, false), 0);
    assert_eq!(end_index(495, true), 495);
}

/// ⛔ Exclusive at both ends.
#[test]
fn the_edge_window_is_exclusive_at_both_ends() {
    let c = call(false, false);
    assert!(!edge_in_window(0, &c));
    assert!(edge_in_window(1, &c));
    assert!(edge_in_window(39, &c));
    assert!(!edge_in_window(40, &c));
}

/// ⚠️ Zero slack is skipped; nothing is skipped outside resistance-aware, or when incremental.
#[test]
fn slack_skips_need_resistance_aware_and_not_incremental() {
    assert!(skips_for_slack(&call(true, false), 0.0));
    assert!(!skips_for_slack(&call(true, false), -1e-9));
    assert!(!skips_for_slack(&call(true, true), 5.0));
    assert!(!skips_for_slack(&call(false, false), 5.0));
}

/// The prelude sets nothing outside resistance-aware mode, and never overrides a fixed percentage.
#[test]
fn the_prelude_follows_the_mode() {
    let p = prelude(&call(false, false), false);
    assert_eq!((p.nets_percentage, p.detour_penalty), (None, None));
    let p = prelude(&call(true, false), false);
    assert_eq!((p.nets_percentage, p.detour_penalty), (Some(100.0), Some(HIGH_DETOUR_PENALTY)));
    let p = prelude(&call(true, true), true);
    assert_eq!((p.nets_percentage, p.detour_penalty), (None, Some(LOW_DETOUR_PENALTY)));
}

/// A retry pass revisits only the retry list, sorted and de-duplicated, and stops at the cap.
#[test]
fn retry_passes_revisit_the_sorted_list_until_the_cap() {
    struct Always;
    impl Maze3DEdgeWork for Always {
        fn num_edges(&self, _: usize) -> usize {
            3
        }
        fn edge_len(&mut self, _: usize, _: usize) -> i32 {
            5
        }
        fn route_edge(&mut self, _: usize, edge: usize) -> EdgeResult {
            EdgeResult { retry: vec![2, edge.min(1), 2], recovered: edge == 0 }
        }
    }
    let order = [OrderedNet { net_id: 0, slack: -1.0, res_aware: true }];
    let (ev, recovered) = maze_route_msmd_order_3d(&call(true, true), &order, &mut Always);
    let passes: Vec<_> = ev.iter().filter_map(|e| match e {
        Maze3DEvent::Pass { iter, stop, retry } => Some((*iter, *stop, retry.clone())),
        _ => None,
    }).collect();
    assert_eq!(passes.len(), 6, "iterations 0..=5, stopping at the cap");
    assert_eq!(passes[0], (0, false, vec![0, 1, 2]));
    assert_eq!(passes[5], (5, true, vec![0, 1, 2]));
    assert_eq!(recovered, 1);
    // Outside incremental resistance-aware mode there is no second pass at all.
    let (ev, _) = maze_route_msmd_order_3d(&call(true, false), &order, &mut Always);
    assert_eq!(ev.iter().filter(|e| matches!(e, Maze3DEvent::Pass { .. })).count(), 1);
    // ⛔ And a ONE-net order walks nothing in plain mode: 90% of 1, truncated, is 0.
    let (ev, _) = maze_route_msmd_order_3d(&call(false, false), &order, &mut Always);
    assert!(ev.is_empty());
}
