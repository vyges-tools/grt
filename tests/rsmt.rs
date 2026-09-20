// SPDX-License-Identifier: Apache-2.0
//! Stages R5 and R7 — a net's Steiner tree becomes the router's segments.
//!
//! 🔑 **Two goldens, because there are two tree sources and they produce different trees.** A
//! net's routing alpha selects between them, and the default is 0.3 — greater than zero — so the
//! ordinary path is the Steiner-tree engine. Setting alpha to zero switches the whole design to
//! a coefficient-weighted flute the router owns. Measured on one design each:
//!
//! | golden | nets taking the Steiner-engine path |
//! | --- | --- |
//! | `rsmt_stt.json` (default alpha) | **990 of 990** |
//! | `rsmt_flute.json` (alpha set to 0) | **0 of 990** |
//!
//! ⚠️ 990 rather than 495 because the stage runs **twice** per design — once initially and once
//! congestion-driven — and both invocations are captured.

#![allow(non_snake_case)]

use vyges_grt::*;

const STT: &str = include_str!("../examples/grt_gate/rsmt_stt.json");
const FLUTE: &str = include_str!("../examples/grt_gate/rsmt_flute.json");

struct Call {
    net: String,
    edge_cost: i8,
    branches: Vec<Branch>,
    segments: Vec<(i32, i32, i32, i32)>,
}

fn parse(text: &str) -> Vec<Call> {
    let v: serde_json::Value = serde_json::from_str(text).expect("golden parses");
    v.as_array().unwrap().iter().map(|c| Call {
        net: c["net"].as_str().unwrap().to_string(),
        edge_cost: c["edge_cost"].as_i64().unwrap() as i8,
        branches: c["branches"].as_array().unwrap().iter().map(|b| {
            let b = b.as_array().unwrap();
            Branch {
                x: b[0].as_i64().unwrap() as i32,
                y: b[1].as_i64().unwrap() as i32,
                n: b[2].as_u64().unwrap() as usize,
            }
        }).collect(),
        segments: c["segments"].as_array().unwrap().iter().map(|s| {
            let s = s.as_array().unwrap();
            (s[0].as_i64().unwrap() as i32, s[1].as_i64().unwrap() as i32,
             s[2].as_i64().unwrap() as i32, s[3].as_i64().unwrap() as i32)
        }).collect(),
    }).collect()
}

fn check(name: &str, text: &str) -> (usize, usize, usize) {
    let calls = parse(text);
    let (mut n_calls, mut n_segs, mut n_vert) = (0, 0, 0);
    for c in &calls {
        let got = segments_from_tree(&c.branches, c.edge_cost);
        assert_eq!(got.segments.len(), c.segments.len(),
            "{name}, net {}: produced {} segments, the reference produced {}",
            c.net, got.segments.len(), c.segments.len());
        // ⚠️ compared IN ORDER: the router consumes this list as a sequence.
        for (i, (g, w)) in got.segments.iter().zip(c.segments.iter()).enumerate() {
            assert_eq!((g.x1, g.y1, g.x2, g.y2), *w, "{name}, net {}, segment {i}", c.net);
            assert_eq!(g.edge_cost, c.edge_cost, "{name}, net {}, segment {i}: edge cost", c.net);
        }
        n_calls += 1;
        n_segs += got.segments.len();
        n_vert += got.segments.iter().filter(|s| s.x1 == s.x2).count();
    }
    (n_calls, n_segs, n_vert)
}

#[test]
fn every_segment_matches_the_REFERENCE_on_the_STEINER_ENGINE_path() {
    let (calls, segs, vert) = check("stt", STT);
    assert_eq!((calls, segs), (990, 1_838));
    assert_eq!(vert, 652, "and this many are vertical, where the swap rule bites");
}

#[test]
fn every_segment_matches_the_REFERENCE_on_the_FLUTE_path_too() {
    // ⭐ A second tree source, producing different trees over the same design — so the
    // transformation is exercised on shapes the first golden never contains.
    let (calls, segs, vert) = check("flute", FLUTE);
    assert_eq!((calls, segs), (990, 1_895));
    assert_eq!(vert, 680);
}

#[test]
fn a_VERTICAL_segment_is_always_SWAPPED_because_the_order_is_by_X_ALONE() {
    // ⛔ The rule that a lexicographic normalisation gets wrong, and only on vertical segments.
    // The branch's own point is (x, 10); its parent is (x, 20). Ordering by x alone finds
    // `x1 < x2` false, takes the else, and emits the PARENT first.
    let branches = vec![
        Branch { x: 5, y: 20, n: 0 },   // root, parent of itself
        Branch { x: 5, y: 10, n: 0 },   // vertical child
    ];
    let out = segments_from_tree(&branches, 1);
    assert_eq!(out.segments.len(), 1);
    let s = out.segments[0];
    assert_eq!((s.x1, s.y1, s.x2, s.y2), (5, 20, 5, 10),
        "the parent comes first: ordering is by x alone, and for a vertical segment it always swaps");
}

#[test]
fn a_HORIZONTAL_segment_keeps_the_lower_x_first() {
    let branches = vec![
        Branch { x: 20, y: 7, n: 0 },
        Branch { x: 10, y: 7, n: 0 },
    ];
    let s = segments_from_tree(&branches, 1).segments[0];
    assert_eq!((s.x1, s.y1, s.x2, s.y2), (10, 7, 20, 7));
}

#[test]
fn a_DEGENERATE_branch_emits_NOTHING() {
    // ⚠️ The accumulation sits before the non-degenerate test in the reference, and this test
    // first claimed that moving it inside would change the wirelength. **That was wrong**, and a
    // mutation proved it: a degenerate branch has Manhattan length zero by definition, so both
    // placements give the same total. The position is kept where the reference keeps it and is
    // recorded as EQUIVALENT rather than as a rule.
    //
    // ⬜ The wirelength total itself is not checked against the reference — the captured golden
    // carries branches and segments, not the running total, which the reference only prints under
    // a debug group. Stated so it is not mistaken for validated.
    let branches = vec![
        Branch { x: 3, y: 4, n: 0 },   // root: paired with itself, degenerate, contributes 0
        Branch { x: 3, y: 4, n: 0 },   // coincident child: degenerate, contributes 0
        Branch { x: 9, y: 4, n: 0 },   // real: contributes 6
    ];
    let out = segments_from_tree(&branches, 1);
    assert_eq!(out.segments.len(), 1, "only the non-degenerate branch emits");
    assert_eq!(out.wirelength, 6);
}

#[test]
fn the_ROOT_is_paired_with_ITSELF_and_so_contributes_nothing() {
    let branches = vec![Branch { x: 11, y: 12, n: 0 }];
    let out = segments_from_tree(&branches, 1);
    assert!(out.segments.is_empty());
    assert_eq!(out.wirelength, 0);
}

#[test]
fn the_edge_cost_is_carried_onto_every_segment() {
    let branches = vec![Branch { x: 0, y: 0, n: 0 }, Branch { x: 10, y: 0, n: 0 }];
    assert_eq!(segments_from_tree(&branches, 7).segments[0].edge_cost, 7);
}

#[test]
fn the_router_asks_flute_for_a_DIFFERENT_accuracy_than_the_builders_own_facade() {
    // ⚠️ 2 here, 3 there. The router does not inherit the builder's accuracy.
    assert_eq!(ROUTER_FLUTE_ACCURACY, 2);
    assert_eq!(COEFF_V_DEFAULT, 1.36);
    assert_eq!(COEFF_V_NO_ADJUSTMENTS, 1.2);
}
