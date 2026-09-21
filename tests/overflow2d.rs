// SPDX-License-Identifier: Apache-2.0
//! The two planar overflow scans, `getOverflow2Dmaze` and `getOverflow2D`.
//!
//! Golden `overflow2d.json`, both cost modes: per scan call that printed its cells, the used-grid
//! sets in order and what the reference returned and set; per call (all of them) the totals and
//! every cell outside the used sets.

use serde_json::Value;
use vyges_grt::{get_overflow_2d, get_overflow_2d_maze, history_threshold, UsedCell};

fn golden() -> Value {
    read(&format!("{}/examples/grt_gate/overflow2d.json", env!("CARGO_MANIFEST_DIR")))
}

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn cells(v: &Value) -> Vec<UsedCell> {
    v.as_array().expect("cells").iter().map(|c| UsedCell {
        x: int(&c[0]) as i32,
        y: int(&c[1]) as i32,
        usage: int(&c[2]) as u16,
        // Carried as IEEE bits — a decimal round trip could move a truncated sum.
        est_usage: f64::from_bits(c[3].as_u64().expect("bits")),
        cap: int(&c[4]) as u16,
    }).collect()
}

/// Replay each call; returns (maze, pattern) calls compared.
fn replay(g: &Value) -> (usize, usize) {
    let (mut maze, mut pattern) = (0, 0);
    for r in g["records"].as_array().expect("records") {
        let (h, v) = (cells(&r["h"]), cells(&r["v"]));
        let got = if r["kind"] == "maze" {
            maze += 1;
            get_overflow_2d_maze(&h, &v)
        } else {
            pattern += 1;
            get_overflow_2d(&h, &v)
        };
        let w = &r["want"];
        let who = format!("{} {}", r["design"], r["kind"]);
        assert_eq!(got.total_overflow, int(&w["ret"]) as i32, "{who}: overflow");
        assert_eq!(got.max_overflow, int(&w["max"]) as i32, "{who}: max overflow");
        assert_eq!(got.ahth, int(&w["ahth"]) as i32, "{who}: ahth");
        // The pattern scan's total usage is internal — only `ahth` reads it — but the
        // instrumented reference prints it, so it is compared for both.
        assert_eq!(got.total_usage, int(&w["usage"]) as i32, "{who}: total usage");
    }
    (maze, pattern)
}

/// Both scans, every captured call's totals and side effect.
#[test]
fn both_overflow_scans_match_the_reference() {
    let (maze, pattern) = replay(&golden());
    assert!(maze >= 20 && pattern >= 20, "too few calls: {maze} maze, {pattern} pattern");
}

/// Every distinct captured call.
///
/// GRT_OVERFLOW2D_FULL=/path/to/overflow2d-all.json cargo test --test overflow2d -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn both_overflow_scans_match_the_reference_exhaustively() {
    let path = std::env::var("GRT_OVERFLOW2D_FULL").expect("set GRT_OVERFLOW2D_FULL");
    eprintln!("exhaustive: {:?}", replay(&read(&path)));
}

/// ⛔ TRIPWIRE: across every captured call (2,177, both scans, both modes) no cell outside the
/// used-grid sets carries usage — so "scan the set" and "scan the grid" agree on the whole
/// corpus. The rule is pinned by the constructed case below; this fails the day a design
/// separates the two, which is when the golden starts testing it.
#[test]
fn no_captured_call_has_usage_outside_the_used_sets() {
    let g = golden();
    let calls = g["calls"].as_array().expect("calls");
    assert!(calls.len() >= 1_000, "too few calls: {}", calls.len());
    let outside: i64 = calls.iter().map(|c| int(&c["outside"])).sum();
    assert_eq!(outside, 0, "a design now has usage outside the used sets — golden it");
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn c(usage: u16, est: f64, cap: u16) -> UsedCell {
    UsedCell { x: 0, y: 0, usage, est_usage: est, cap }
}

/// ⛔ Only the SET is scanned: a cell the caller leaves out contributes nothing, however
/// overflowing. (The corpus cannot show this; see the tripwire above.)
#[test]
fn only_the_used_set_is_scanned() {
    let inside = [c(3, 3.0, 1)];
    assert_eq!(get_overflow_2d_maze(&inside, &[]).total_overflow, 2);
    assert_eq!(get_overflow_2d_maze(&[], &[]).total_overflow, 0);
}

/// ⛔ The pattern scan truncates the running total at EVERY addition, not once at the end:
/// 0.6 + 0.6 is 0 + 0 = 0, not 1.
#[test]
fn the_pattern_scan_truncates_its_running_total_per_cell() {
    let h = [c(0, 0.6, 5), c(0, 0.6, 5)];
    assert_eq!(get_overflow_2d(&h, &[]).total_usage, 0);
    let h = [c(0, 1.6, 5), c(0, 1.6, 5)];
    assert_eq!(get_overflow_2d(&h, &[]).total_usage, 2);
}

/// ⛔ The pattern scan's per-cell overflow truncates toward zero: 0.9 over capacity is none.
#[test]
fn a_sub_unit_estimated_overflow_counts_as_none() {
    let r = get_overflow_2d(&[c(0, 4.9, 4)], &[c(0, 6.5, 4)]);
    assert_eq!((r.total_overflow, r.max_overflow), (2, 2));
}

/// ⛔ The two scans read DIFFERENT usage: the maze scan committed, the pattern scan estimated.
#[test]
fn the_two_scans_read_different_usage() {
    let h = [c(7, 2.0, 4)];
    assert_eq!(get_overflow_2d_maze(&h, &[]).total_overflow, 3);
    assert_eq!(get_overflow_2d(&h, &[]).total_overflow, 0);
}

/// ⚠️ The maximum is over each direction's POSITIVE overflows, then the larger of the two; the
/// total sums positive overflows only.
#[test]
fn max_and_total_count_only_positive_overflow() {
    let r = get_overflow_2d_maze(&[c(1, 0.0, 5), c(9, 0.0, 5)], &[c(6, 0.0, 5)]);
    assert_eq!((r.total_overflow, r.max_overflow, r.total_usage), (5, 4, 16));
}

/// ⚠️ `ahth` is 30 only STRICTLY above 800,000 total usage. No captured design gets there.
#[test]
fn the_history_threshold_steps_strictly_above_800000() {
    assert_eq!(history_threshold(800_000), 20);
    assert_eq!(history_threshold(800_001), 30);
    let big: Vec<UsedCell> = (0..13).map(|_| c(65_000, 0.0, 0)).collect(); // 845,000
    assert_eq!(get_overflow_2d_maze(&big, &[]).ahth, 30);
}
