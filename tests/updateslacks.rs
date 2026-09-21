// SPDX-License-Identifier: Apache-2.0
//! R16 — choosing which nets the router treats as resistance-aware.
//!
//! 87 calls, **11,564 nets, 993 of which survive the filters**, from 41 designs.
//!
//! ⛔ There is no default-mode half of this corpus, because there is nothing to capture: the pass
//! returns before touching anything unless the router is resistance-aware and a liberty library
//! is loaded. Every trace comes from a `global_route -resistance_aware` run.

use serde_json::Value;
use vyges_grt::{res_aware_score, update_slacks, NetSlackInput, SlackParams, WorstMetrics};

struct Net {
    net_id: usize,
    slack: f32,
    edge_len: Vec<i32>,
    num_pins: i32,
    is_clock: bool,
    has_ndr: bool,
    is_res_aware: bool,
    resistance: f32,
    want_net_length: i32,
    want_skipped: bool,
    want_score: Option<f32>,
    want_worst: Option<WorstMetrics>,
}

struct Call {
    design: String,
    is_incremental: bool,
    percentage: f32,
    infinity: f32,
    short_threshold: i32,
    nets: Vec<Net>,
    want_candidates: Vec<(usize, f32)>,
    want_after: Vec<(usize, bool)>,
}

fn calls() -> Vec<Call> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/updateslacks.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["calls"].as_array().expect("calls").iter().map(|c| {
        let mut after: Vec<(usize, bool)> = c["after"].as_object().expect("after").iter()
            .map(|(k, v)| (k.parse::<usize>().expect("id"), v.as_bool().expect("bool")))
            .collect();
        after.sort_unstable_by_key(|p| p.0);
        Call {
            design: c["design"].as_str().expect("design").to_string(),
            is_incremental: c["is_incremental"].as_bool().expect("incr"),
            percentage: c["percentage"].as_f64().expect("pct") as f32,
            infinity: c["infinity"].as_f64().expect("inf") as f32,
            short_threshold: c["short_threshold"].as_i64().expect("short") as i32,
            nets: c["nets"].as_array().expect("nets").iter().map(|n| Net {
                net_id: n["net_id"].as_u64().expect("id") as usize,
                slack: n["slack"].as_f64().expect("slack") as f32,
                edge_len: n["edge_len"].as_array().expect("len").iter()
                    .map(|x| x.as_i64().expect("int") as i32).collect(),
                num_pins: n["num_pins"].as_i64().expect("pins") as i32,
                is_clock: n["is_clock"].as_bool().expect("clk"),
                has_ndr: n["has_ndr"].as_bool().expect("ndr"),
                is_res_aware: n["is_res_aware"].as_bool().expect("rab"),
                resistance: n["resistance"].as_f64().expect("res") as f32,
                want_net_length: n["want_net_length"].as_i64().expect("nl") as i32,
                want_skipped: n["want_skipped"].as_bool().expect("skip"),
                want_score: n["want_score"].as_f64().map(|x| x as f32),
                want_worst: n["want_worst"].as_object().map(|w| WorstMetrics {
                    resistance: w["resistance"].as_f64().expect("wr") as f32,
                    slack: w["slack"].as_f64().expect("ws") as f32,
                    net_length: w["net_length"].as_i64().expect("wl") as i32,
                    fanout: w["fanout"].as_i64().expect("wf") as i32,
                }),
            }).collect(),
            want_candidates: c["candidates"].as_array().expect("cands").iter().map(|p| {
                let a = p.as_array().expect("pair");
                (a[0].as_u64().expect("id") as usize, a[1].as_f64().expect("score") as f32)
            }).collect(),
            want_after: after,
        }
    }).collect()
}

fn inputs(c: &Call) -> Vec<NetSlackInput<'_>> {
    c.nets.iter().map(|n| NetSlackInput {
        net_id: n.net_id,
        slack: n.slack,
        edge_len: &n.edge_len,
        num_pins: n.num_pins,
        is_clock: n.is_clock,
        has_ndr: n.has_ndr,
        is_res_aware: n.is_res_aware,
        resistance: n.resistance,
    }).collect()
}

fn params(c: &Call) -> SlackParams {
    SlackParams {
        enabled: true,
        is_incremental: c.is_incremental,
        percentage: c.percentage,
        infinity: c.infinity,
    }
}

#[test]
fn the_resistance_aware_selection_matches_the_reference() {
    let all = calls();
    assert!(all.len() >= 70, "corpus too thin: {}", all.len());
    let (mut nets, mut survivors, mut with_candidates) = (0usize, 0usize, 0usize);

    for c in &all {
        let got = update_slacks(&inputs(c), &params(c));

        // The marked set, which is what the rest of the router reads.
        // ⚠️ Sorted on both sides: the capture records the marks in a map, so its order is the
        // serialiser's, not the router's. The net order itself is checked below, per net.
        let mut after: Vec<(usize, bool)> =
            got.nets.iter().map(|o| (o.net_id, o.is_res_aware)).collect();
        after.sort_unstable_by_key(|p| p.0);
        assert_eq!(after, c.want_after, "resistance-aware set on {}", c.design);

        // The candidate list, in order, with its scores.
        assert_eq!(
            got.candidates, c.want_candidates,
            "candidate list on {} ({} nets)", c.design, c.nets.len()
        );

        for (o, n) in got.nets.iter().zip(c.nets.iter()) {
            assert_eq!(o.net_length, n.want_net_length,
                       "net length of {} on {}", n.net_id, c.design);
            // ⛔ Slack and length are written for EVERY net; resistance only for survivors.
            assert_eq!(o.slack, n.slack, "slack of {} on {}", n.net_id, c.design);
            assert_eq!(
                o.resistance.is_none(), n.want_skipped,
                "net {} on {} was {} when the reference {} it",
                n.net_id, c.design,
                if o.resistance.is_none() { "skipped" } else { "kept" },
                if n.want_skipped { "skipped" } else { "kept" }
            );
            nets += 1;
            survivors += usize::from(!n.want_skipped);
        }
        with_candidates += usize::from(!c.want_candidates.is_empty());
    }
    assert!(nets >= 8_000, "too few nets: {nets}");
    assert!(survivors >= 500, "too few nets survive the filters: {survivors}");
    assert!(with_candidates >= 5, "too few calls produce candidates: {with_candidates}");
}

/// ⛔ **The score is computed against PARTIALLY accumulated worst metrics** — the ones standing
/// when the net is reached, not the final ones. This gates that directly, per net, so an
/// implementation that hoisted the accumulation into its own pass fails here rather than
/// surviving on a marked set that happened to come out the same.
#[test]
fn every_score_matches_the_worst_metrics_as_they_stood() {
    let all = calls();
    let (mut scored, mut differ_from_final) = (0usize, 0usize);

    for c in &all {
        let got = update_slacks(&inputs(c), &params(c));
        for (o, n) in got.nets.iter().zip(c.nets.iter()) {
            let (Some(want_score), Some(want_worst)) = (n.want_score, n.want_worst) else {
                assert!(o.score.is_none(), "net {} on {} was scored and should not be",
                        n.net_id, c.design);
                continue;
            };
            assert_eq!(
                o.worst_at_score, Some(want_worst),
                "worst metrics when net {} was scored on {}", n.net_id, c.design
            );
            assert_eq!(
                o.score, Some(want_score),
                "score of net {} on {}", n.net_id, c.design
            );
            scored += 1;
            // Would the FINAL metrics have given a different answer? If never, the
            // accumulation order is not actually being tested by this corpus.
            let with_final = -res_aware_score(
                &got.worst, n.resistance, n.slack, n.num_pins, n.want_net_length,
            );
            if with_final != want_score {
                differ_from_final += 1;
            }
        }
    }
    assert!(scored >= 500, "too few scored nets: {scored}");
    // ⛔ Without this the test above would pass for an implementation that used the final
    // metrics throughout.
    assert!(
        differ_from_final >= 100,
        "only {differ_from_final} scores differ from what the FINAL metrics would give — the \
         accumulation order is barely exercised"
    );
}

/// ⛔ Each filter must be reached, and nets must get through.
#[test]
fn the_corpus_exercises_every_filter_and_flag() {
    let all = calls();
    let (mut short, mut unconstrained, mut positive, mut kept) = (0usize, 0usize, 0usize, 0usize);
    for c in &all {
        for n in &c.nets {
            let len: i32 = n.edge_len.iter().sum();
            if len <= c.short_threshold { short += 1 }
            if n.slack == c.infinity && !n.is_clock { unconstrained += 1 }
            if !c.is_incremental && n.slack > 0.0 && !n.is_clock { positive += 1 }
            if !n.want_skipped { kept += 1 }
        }
    }
    assert!(short >= 50, "the short-net filter is barely reached: {short}");
    assert!(unconstrained >= 500, "the unconstrained filter is barely reached: {unconstrained}");
    assert!(positive >= 50, "the positive-slack filter is barely reached: {positive}");
    assert!(kept >= 500, "too few nets get through: {kept}");

    let flag = |f: &dyn Fn(&Call) -> bool| all.iter().filter(|c| f(c)).count();
    assert!(flag(&|c| c.is_incremental) >= 3, "no incremental call");
    assert!(flag(&|c| c.nets.iter().any(|n| n.is_clock)) >= 20, "no clock net");
    assert!(flag(&|c| c.nets.iter().any(|n| n.has_ndr)) >= 5, "no NDR net");
    assert!(flag(&|c| c.nets.iter().any(|n| n.is_res_aware)) >= 20, "no pre-marked net");

    // ⚠️ More than one configured percentage, or the rounding is only ever seen at one ratio.
    let mut pcts: Vec<i32> = all.iter().filter(|c| !c.want_candidates.is_empty())
        .map(|c| c.percentage as i32).collect();
    pcts.sort_unstable();
    pcts.dedup();
    assert!(pcts.len() >= 2, "only one percentage produces candidates: {pcts:?}");
}

// ─── What the corpus cannot witness ─────────────────────────────────────────────────────────

/// ⛔ **No incremental call in the corpus produces a candidate**, so the rule that an incremental
/// run ignores the configured percentage and takes all of them is unwitnessed here.
#[test]
fn no_incremental_call_produces_candidates() {
    let all = calls();
    let bad = all.iter().filter(|c| c.is_incremental && !c.want_candidates.is_empty()).count();
    assert_eq!(
        bad, 0,
        "{bad} incremental calls now produce candidates — the percentage override is witnessed \
         and the constructed case below is redundant"
    );
    assert!(all.iter().any(|c| c.is_incremental), "no incremental call at all");
}

fn one_net(id: usize, slack: f32, len: i32, pins: i32) -> (Vec<i32>, usize, f32, i32) {
    (vec![len], id, slack, pins)
}

/// ⛔ An incremental run marks **every** candidate, whatever the configured percentage says.
#[test]
fn an_incremental_run_ignores_the_configured_percentage() {
    let lens: Vec<Vec<i32>> = (0..4).map(|i| vec![10 + i]).collect();
    let nets: Vec<NetSlackInput> = (0..4).map(|i| NetSlackInput {
        net_id: i,
        slack: -1.0 - i as f32,
        edge_len: &lens[i],
        num_pins: 2 + i as i32,
        is_clock: false,
        has_ndr: false,
        is_res_aware: false,
        resistance: 1.0 + i as f32,
    }).collect();
    let p = SlackParams {
        enabled: true, is_incremental: true, percentage: 15.0, infinity: 1e30,
    };
    let got = update_slacks(&nets, &p);
    assert_eq!(got.candidates.len(), 4, "all four should be candidates");
    assert_eq!(got.marked, 4, "an incremental run must mark every candidate");
    assert!(got.nets.iter().all(|o| o.is_res_aware), "all four must come out marked");

    // ⚠️ And the same inputs at 15% mark only one, which is what makes the override observable.
    let p = SlackParams { is_incremental: false, ..p };
    let got = update_slacks(&nets, &p);
    assert_eq!(got.marked, 1, "15% of four candidates rounds up to one");
}

/// ⛔ The count is rounded **up**, so a non-empty candidate list always marks at least one net —
/// even at a percentage that would otherwise round to zero.
#[test]
fn the_marked_count_rounds_up() {
    let lens: Vec<Vec<i32>> = (0..3).map(|_| vec![10]).collect();
    let nets: Vec<NetSlackInput> = (0..3).map(|i| NetSlackInput {
        net_id: i, slack: -1.0, edge_len: &lens[i], num_pins: 2,
        is_clock: false, has_ndr: false, is_res_aware: false, resistance: 1.0,
    }).collect();
    let got = update_slacks(&nets, &SlackParams {
        enabled: true, is_incremental: false, percentage: 1.0, infinity: 1e30,
    });
    assert_eq!(got.candidates.len(), 3);
    assert_eq!(got.marked, 1, "1% of three must round up to one, not down to zero");
}

/// ⛔ Disabled, the pass writes **nothing** — not the slack, not the length, not the marks.
#[test]
fn a_disabled_pass_touches_nothing() {
    let lens = vec![vec![40], vec![40]];
    let nets: Vec<NetSlackInput> = (0..2).map(|i| NetSlackInput {
        net_id: i, slack: -5.0, edge_len: &lens[i], num_pins: 9,
        is_clock: true, has_ndr: true, is_res_aware: false, resistance: 7.0,
    }).collect();
    let got = update_slacks(&nets, &SlackParams {
        enabled: false, is_incremental: false, percentage: 100.0, infinity: 1e30,
    });
    assert!(got.candidates.is_empty(), "a disabled pass must produce no candidates");
    assert_eq!(got.marked, 0);
    for o in &got.nets {
        assert_eq!(o.net_length, 0, "the length must not be written");
        assert_eq!(o.resistance, None, "the resistance must not be read");
        assert!(!o.is_res_aware, "no net may be marked — these would both qualify if it ran");
    }
}

/// ⛔ A clock or NDR net is marked outright and is **not** a candidate, so it never competes for
/// the percentage. It still contributes to the worst metrics.
#[test]
fn a_clock_or_ndr_net_is_marked_outright_and_never_competes() {
    let lens = vec![vec![40], vec![40], vec![40]];
    let nets = vec![
        NetSlackInput { net_id: 0, slack: -5.0, edge_len: &lens[0], num_pins: 9,
                        is_clock: true, has_ndr: false, is_res_aware: false, resistance: 7.0 },
        NetSlackInput { net_id: 1, slack: -4.0, edge_len: &lens[1], num_pins: 8,
                        is_clock: false, has_ndr: true, is_res_aware: false, resistance: 6.0 },
        NetSlackInput { net_id: 2, slack: -3.0, edge_len: &lens[2], num_pins: 7,
                        is_clock: false, has_ndr: false, is_res_aware: false, resistance: 5.0 },
    ];
    let got = update_slacks(&nets, &SlackParams {
        enabled: true, is_incremental: false, percentage: 100.0, infinity: 1e30,
    });
    assert_eq!(
        got.candidates.iter().map(|c| c.0).collect::<Vec<_>>(), vec![2],
        "only the plain net may be a candidate"
    );
    assert!(got.nets.iter().all(|o| o.is_res_aware), "all three end up marked");
    // ⚠️ The clock and NDR nets still fed the worst metrics before being excluded.
    assert_eq!(got.worst.fanout, 9, "the clock net's fanout must still count");
    assert_eq!(got.worst.resistance, 7.0);
}

/// ⛔ **The incremental exemption from the positive-slack filter never decides anything here.**
///
/// 48 nets across the incremental calls do have positive slack and are not clocks — so the
/// condition is *reached* — but **every one of them is already short or unconstrained**, and so
/// is filtered out by an earlier term whatever the incremental flag says. A branch being taken is
/// not the same as its boundary being tested.
#[test]
fn no_incremental_net_is_kept_only_by_the_incremental_exemption() {
    let all = calls();
    let (mut reached, mut decisive) = (0usize, 0usize);
    for c in all.iter().filter(|c| c.is_incremental) {
        for n in &c.nets {
            if n.slack > 0.0 && !n.is_clock {
                reached += 1;
                let len: i32 = n.edge_len.iter().sum();
                let short = len <= c.short_threshold;
                let unconstrained = n.slack == c.infinity && !n.is_clock;
                if !short && !unconstrained {
                    decisive += 1;
                }
            }
        }
    }
    assert!(reached >= 10, "the condition is not even reached: {reached}");
    assert_eq!(
        decisive, 0,
        "{decisive} incremental nets are now kept only by the incremental exemption — it is \
         witnessed by the corpus and the constructed case below is redundant"
    );
}

/// ⛔ An incremental run keeps a positive-slack net that a normal run would filter out.
#[test]
fn an_incremental_run_keeps_a_positive_slack_net() {
    let lens = vec![vec![40]];
    let net = |incremental: bool| {
        let nets = vec![NetSlackInput {
            net_id: 0,
            // Positive, finite, long enough, not a clock: filtered only by the positive-slack
            // rule, and only when the run is not incremental.
            slack: 5.0,
            edge_len: &lens[0],
            num_pins: 4,
            is_clock: false,
            has_ndr: false,
            is_res_aware: false,
            resistance: 2.0,
        }];
        update_slacks(&nets, &SlackParams {
            enabled: true, is_incremental: incremental, percentage: 100.0, infinity: 1e30,
        })
    };
    assert!(
        net(false).nets[0].resistance.is_none(),
        "a normal run must filter out a positive-slack net"
    );
    assert!(
        net(true).nets[0].resistance.is_some(),
        "an incremental run must keep it — the positive-slack filter does not apply"
    );
    assert_eq!(net(true).candidates.len(), 1, "and it becomes a candidate");
}

/// ⛔ **No captured net has exactly zero slack**, so the corpus cannot say whether the filter is
/// "greater than zero" or "at least zero".
#[test]
fn no_captured_net_has_exactly_zero_slack() {
    let all = calls();
    let zero = all.iter().flat_map(|c| c.nets.iter()).filter(|n| n.slack == 0.0).count();
    assert_eq!(
        zero, 0,
        "{zero} captured nets now have exactly zero slack — the filter's boundary is witnessed \
         and the constructed case below is redundant"
    );
    let total: usize = all.iter().map(|c| c.nets.len()).sum();
    assert!(total >= 8_000, "corpus too thin to call zero slack unreached: {total}");
}

/// ⛔ A net with exactly zero slack is **kept**: the filter is "greater than zero", not "at least
/// zero".
#[test]
fn a_net_with_exactly_zero_slack_is_kept() {
    let lens = vec![vec![40]];
    let nets = vec![NetSlackInput {
        net_id: 0, slack: 0.0, edge_len: &lens[0], num_pins: 4,
        is_clock: false, has_ndr: false, is_res_aware: false, resistance: 2.0,
    }];
    let got = update_slacks(&nets, &SlackParams {
        enabled: true, is_incremental: false, percentage: 100.0, infinity: 1e30,
    });
    assert!(
        got.nets[0].resistance.is_some(),
        "zero slack is not positive slack — the net must be kept"
    );
    assert_eq!(got.candidates.len(), 1, "and it must become a candidate");
}
