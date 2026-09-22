// SPDX-License-Identifier: Apache-2.0
//! `CalculatePartialSlack` against the reference's own call on `critical_nets_percentage` (the only
//! suite case whose loop reads finite slacks), captured by `grt-slack-trace.py`: each routed net's
//! timer slack as bits, the threshold index and value, and the nets it demoted.
//!
//! The rule: the slack at `ceil(n * percentage / 100)` of the sorted slacks is the threshold, and
//! every net strictly ABOVE it takes the sentinel `ceil(lowest float)`.

use vyges_grt::maze_msmd::calculate_partial_slack;
use vyges_grt::NetState;

fn field<'a>(line: &'a str, key: &str) -> &'a str {
    line.split('|').find_map(|f| f.strip_prefix(key)).unwrap_or_else(|| panic!("{key} in {line}"))
}

#[test]
fn the_threshold_and_the_demoted_nets_match_the_reference() {
    let text = include_str!("data/partial-slack-critical_nets_percentage.txt");
    let (mut ids, mut slack, mut demoted) = (Vec::new(), Vec::new(), Vec::new());
    let mut partial = None;
    for line in text.lines() {
        if line.starts_with("VYGS|slack|") {
            ids.push(field(line, "id=").parse::<usize>().unwrap());
            slack.push(f32::from_bits(u32::from_str_radix(field(line, "bits="), 16).unwrap()));
        } else if line.starts_with("VYGS|demote|") {
            demoted.push(field(line, "id=").parse::<usize>().unwrap());
        } else if line.starts_with("VYGS|partial|") {
            let n: usize = field(line, "n=").parse().unwrap();
            let cnp: f32 = field(line, "cnp=").parse().unwrap();
            partial = Some((n, cnp, u32::from_str_radix(field(line, "bits="), 16).unwrap()));
        }
    }
    let (n, cnp, th_bits) = partial.expect("the partial record");
    assert_eq!((ids.len(), demoted.len()), (n, 223), "the capture's own counts");
    let mut timer = vec![0.0f32; ids.iter().max().unwrap() + 1];
    for (&id, &s) in ids.iter().zip(&slack) {
        timer[id] = s;
    }
    let mut state = vec![NetState::default(); timer.len()];
    let th = calculate_partial_slack(&ids, &mut state, &timer, cnp);
    assert_eq!(th.to_bits(), th_bits, "threshold {th} vs the reference's");
    let ours: Vec<usize> = ids.iter().copied().filter(|&id| state[id].slack == f32::MIN).collect();
    assert_eq!(ours, demoted, "the demoted nets, in net order");
    for &id in &ids {
        if !demoted.contains(&id) {
            assert_eq!(state[id].slack.to_bits(), timer[id].to_bits(), "net {id} keeps its timer slack");
        }
    }
}

// With no timing constraint every slack is the timer's INF: the threshold is INF, nothing is
// demoted (as the reference did on bus_route, 5 calls: `th=1.00000002e+30`, no demotion).
#[test]
fn unconstrained_slacks_demote_nothing() {
    let ids: Vec<usize> = (0..15).collect();
    let timer = vec![1.0e30f32; 15];
    let mut state = vec![NetState::default(); 15];
    let th = calculate_partial_slack(&ids, &mut state, &timer, 10.0);
    assert_eq!(th.to_bits(), 0x7149_f2ca);
    assert!(state.iter().all(|s| s.slack.to_bits() == 0x7149_f2ca));
}

// The index is ceil(n * p / 100) in f32, clamped to the last slack; no nets gives 0.0.
#[test]
fn the_index_rounds_up_and_clamps() {
    let ids: Vec<usize> = (0..3).collect();
    let timer = [1.0f32, 2.0, 3.0];
    let th = |p: f32| calculate_partial_slack(&ids, &mut vec![NetState::default(); 3], &timer, p);
    // 3 * 33 / 100 = 0.99 → index 1; 3 * 34 / 100 = 1.02 → index 2; 100% → index 3, clamped to 2.
    assert_eq!((th(10.0), th(33.0), th(34.0), th(100.0)), (2.0, 2.0, 3.0, 3.0));
    assert_eq!(calculate_partial_slack(&[], &mut [], &[], 30.0), 0.0);
}
