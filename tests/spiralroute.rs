// SPDX-License-Identifier: Apache-2.0
//! R9 `spiralRoute` — the per-edge routing the outward walk drives.
//!
//! 16,229 distinct calls captured from four designs, 3,487 of them taking the bent arm, **1,619
//! of those exact ties**. The body is the earlier L re-route's with three differences, and those
//! are what this file checks: marks landing on the **alias** as well as the node, the `hID`/`lID`
//! counters incremented on the alias nodes, and the crossing between the two.
//!
//! ⛔ **The via bias is dead in the shipped flow and this file proves it rather than assuming it.**
//! `spiralRouteAll` has a single call site, between the assignment that zeroes `via_cost_` and the
//! one that raises it in the 3D phase. Every captured call carries `via_cost_ = 0`, asserted
//! below, so the bias contributes nothing and the unguarded-versus-`viaGuided` difference from
//! the earlier pass cannot be observed on any design.
//!
//! ⚠️ **How the bent arm is replayed.** Reproducing the captured costs would mean reconstructing
//! the whole demand grid at that instant. Instead the two halves are checked separately: the
//! comparison is replayed against the costs the reference actually computed (raw IEEE bits), and
//! the state transition is driven through the real function with the grid rigged so the captured
//! arm is the one taken. Neither half is a re-implementation of the other.

use serde_json::Value;
use vyges_grt::estimate::EstimateGrid;
use vyges_grt::{chooses_y_first, spiral_route, EdgeRoute, LShape, SpiralNode};

struct Row {
    design: String,
    n1: usize,
    n2: usize,
    len: i32,
    x1: i16,
    y1: i16,
    x2: i16,
    y2: i16,
    n1a: usize,
    n2a: usize,
    s1: i16,
    s2: i16,
    s1a: i16,
    s2a: i16,
    h1a0: i32,
    l1a0: i32,
    h2a0: i32,
    l2a0: i32,
    edge_cost: i8,
    cost_l1_bits: i64,
    cost_l2_bits: i64,
    x_first: bool,
    s1_after: i16,
    s1a_after: i16,
    s2_after: i16,
    s2a_after: i16,
    h1a: i32,
    l1a: i32,
    h2a: i32,
    l2a: i32,
}

fn golden() -> (Vec<Row>, Vec<(String, u64)>) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/spiralroute.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");

    let cols: Vec<&str> = v["columns"].as_array().expect("columns").iter()
        .map(|c| c.as_str().expect("name")).collect();
    let idx = |name: &str| cols.iter().position(|c| *c == name)
        .unwrap_or_else(|| panic!("column {name}"));
    let (i_n1, i_n2, i_len) = (idx("n1"), idx("n2"), idx("len"));
    let (i_x1, i_y1, i_x2, i_y2) = (idx("x1"), idx("y1"), idx("x2"), idx("y2"));
    let (i_n1a, i_n2a) = (idx("n1a"), idx("n2a"));
    let (i_s1, i_s2, i_s1a, i_s2a) = (idx("s1"), idx("s2"), idx("s1a"), idx("s2a"));
    let (i_h10, i_l10) = (idx("h1a0"), idx("l1a0"));
    let (i_h20, i_l20) = (idx("h2a0"), idx("l2a0"));
    let i_ec = idx("edge_cost");
    let (i_c1, i_c2, i_xf) = (idx("cost_l1_bits"), idx("cost_l2_bits"), idx("x_first"));
    let (i_a1, i_a1a) = (idx("s1_after"), idx("s1a_after"));
    let (i_a2, i_a2a) = (idx("s2_after"), idx("s2a_after"));
    let (i_h1, i_l1, i_h2, i_l2) = (idx("h1a"), idx("l1a"), idx("h2a"), idx("l2a"));

    let rows = v["rows"].as_array().expect("rows").iter().map(|r| {
        let n = |i: usize| r[i].as_i64().expect("int");
        Row {
            design: r[0].as_str().expect("design").to_string(),
            n1: n(i_n1) as usize, n2: n(i_n2) as usize, len: n(i_len) as i32,
            x1: n(i_x1) as i16, y1: n(i_y1) as i16, x2: n(i_x2) as i16, y2: n(i_y2) as i16,
            n1a: n(i_n1a) as usize, n2a: n(i_n2a) as usize,
            s1: n(i_s1) as i16, s2: n(i_s2) as i16,
            s1a: n(i_s1a) as i16, s2a: n(i_s2a) as i16,
            h1a0: n(i_h10) as i32, l1a0: n(i_l10) as i32,
            h2a0: n(i_h20) as i32, l2a0: n(i_l20) as i32,
            edge_cost: n(i_ec) as i8,
            cost_l1_bits: n(i_c1), cost_l2_bits: n(i_c2),
            x_first: n(i_xf) != 0,
            s1_after: n(i_a1) as i16, s1a_after: n(i_a1a) as i16,
            s2_after: n(i_a2) as i16, s2a_after: n(i_a2a) as i16,
            h1a: n(i_h1) as i32, l1a: n(i_l1) as i32,
            h2a: n(i_h2) as i32, l2a: n(i_l2) as i32,
        }
    }).collect();

    let consts = v["constants"].as_object().expect("constants").iter()
        .map(|(k, c)| (k.clone(), c["via_cost_bits"].as_u64().expect("bits")))
        .collect();
    (rows, consts)
}

/// ⛔ Stated as an assertion, not a comment: if a design ever ran this stage with a non-zero via
/// cost, every claim about the bias in this file would be wrong and the test would say so.
#[test]
fn the_via_cost_is_zero_on_every_captured_call() {
    let (_, consts) = golden();
    assert!(!consts.is_empty());
    for (design, bits) in consts {
        assert_eq!(bits, 0, "{design} ran spiralRoute with a non-zero via cost");
    }
}

/// The comparison, replayed against the costs the reference actually computed.
#[test]
fn the_bend_matches_the_reference_on_its_own_costs() {
    let (rows, _) = golden();
    let (mut checked, mut ties) = (0usize, 0usize);
    for r in &rows {
        if r.cost_l1_bits < 0 {
            continue; // not a bent edge
        }
        let l1 = f64::from_bits(r.cost_l1_bits as u64);
        let l2 = f64::from_bits(r.cost_l2_bits as u64);
        assert_eq!(
            chooses_y_first(l1, l2), !r.x_first,
            "bend disagrees on {} ({l1} vs {l2})", r.design
        );
        checked += 1;
        ties += usize::from(r.cost_l1_bits == r.cost_l2_bits);
    }
    assert!(checked >= 3000, "too few bent edges: {checked}");
    // ⚠️ Without ties the tie-break is untested, and here they are nearly half the decisions.
    assert!(ties >= 1000, "corpus has too few ties to test the tie-break: {ties}");
}

/// Build the node array one call needs, with the alias relationships the reference had.
fn nodes_for(r: &Row) -> Vec<SpiralNode> {
    let n = *[r.n1, r.n2, r.n1a, r.n2a].iter().max().expect("nonempty") + 1;
    let mut nodes: Vec<SpiralNode> = (0..n)
        .map(|d| SpiralNode {
            x: 0, y: 0, top_layer: -1, bot_layer: 0, assigned: false,
            stack_alias: d, status: 0, h_id: 0, l_id: 0, edges: Vec::new(),
        })
        .collect();
    nodes[r.n1].x = r.x1;
    nodes[r.n1].y = r.y1;
    nodes[r.n2].x = r.x2;
    nodes[r.n2].y = r.y2;
    nodes[r.n1].stack_alias = r.n1a;
    nodes[r.n2].stack_alias = r.n2a;
    // ⚠️ Set the alias statuses BEFORE the nodes' own: when a node is its own alias the two are
    // the same slot, and the node's value is the one the reference recorded.
    nodes[r.n1a].status = r.s1a;
    nodes[r.n2a].status = r.s2a;
    nodes[r.n1].status = r.s1;
    nodes[r.n2].status = r.s2;
    // ⛔ The counters are CUMULATIVE over the net's edges — reset once, in the stage's first
    // pass, not per call. Starting them at zero checks only this call's delta and disagrees with
    // the reference from the second edge of every net onward.
    nodes[r.n1a].h_id = r.h1a0;
    nodes[r.n1a].l_id = r.l1a0;
    nodes[r.n2a].h_id = r.h2a0;
    nodes[r.n2a].l_id = r.l2a0;
    nodes
}

/// Every captured call, driven through the real function.
#[test]
fn state_transitions_match_the_reference() {
    let (rows, _) = golden();
    let (mut v_arm, mut h_arm, mut l_arm, mut aliased) = (0usize, 0usize, 0usize, 0usize);

    for r in &rows {
        let mut nodes = nodes_for(r);
        let span = usize::from(r.x1.max(r.x2).max(r.y1.max(r.y2)) as u16) + 2;
        let mut grid = EstimateGrid::new(span, span);

        // Rig the blockage so the captured arm is the one taken. A bend recorded as y-first had
        // the x2 column cost more, so that column is made expensive; everything else stays at
        // zero, where both candidates cost exactly the same and the tie falls to x-first.
        let want_y_first = !r.x_first;
        let x2 = r.x2 as usize;
        let red_v = move |x: usize, _y: usize| -> u16 {
            if want_y_first && x == x2 { 60000 } else { 0 }
        };
        let red_h = |_x: usize, _y: usize| -> u16 { 0 };

        let got = spiral_route(
            &mut grid, &mut nodes, (r.n1, r.n2, r.len),
            r.edge_cost, 0.0, 1000.0, 1000.0, &red_v, &red_h,
        );

        match got {
            EdgeRoute::Vertical => { assert!(!r.x_first); v_arm += 1; }
            EdgeRoute::Horizontal => { assert!(r.x_first); h_arm += 1; }
            EdgeRoute::L(LShape::YFirst) => { assert!(!r.x_first); l_arm += 1; }
            EdgeRoute::L(LShape::XFirst) => { assert!(r.x_first); l_arm += 1; }
            EdgeRoute::None => panic!("zero-length edge reached spiral_route on {}", r.design),
        }

        assert_eq!(nodes[r.n1].status, r.s1_after, "n1 status, {}", r.design);
        assert_eq!(nodes[r.n1a].status, r.s1a_after, "n1a status, {}", r.design);
        assert_eq!(nodes[r.n2].status, r.s2_after, "n2 status, {}", r.design);
        assert_eq!(nodes[r.n2a].status, r.s2a_after, "n2a status, {}", r.design);
        assert_eq!(nodes[r.n1a].h_id, r.h1a, "n1a hID, {}", r.design);
        assert_eq!(nodes[r.n1a].l_id, r.l1a, "n1a lID, {}", r.design);
        assert_eq!(nodes[r.n2a].h_id, r.h2a, "n2a hID, {}", r.design);
        assert_eq!(nodes[r.n2a].l_id, r.l2a, "n2a lID, {}", r.design);

        aliased += usize::from(r.n1a != r.n1 || r.n2a != r.n2);
    }

    // ⚠️ All three arms, and enough aliased calls that marking only the node would be caught.
    assert!(v_arm >= 100, "vertical arm barely exercised: {v_arm}");
    assert!(h_arm >= 100, "horizontal arm barely exercised: {h_arm}");
    assert!(l_arm >= 3000, "bent arm barely exercised: {l_arm}");
    assert!(aliased >= 100, "corpus never marks through an alias: {aliased}");
}

/// A zero-length edge is set to "no route" and touches nothing else.
///
/// ⛔ **Unreachable from the only caller, and constructed for that reason.** The outward walk
/// pre-marks zero-length edges assigned so they are never queued, so no design can drive this arm
/// — it survived a deliberate mutation across all 16,229 captured calls. The guard is in the
/// reference, so it is transcribed and pinned here rather than left as the one branch a mutation
/// can flip silently.
#[test]
fn a_degenerate_edge_is_not_routed() {
    let mut nodes = vec![
        SpiralNode {
            x: 3, y: 4, top_layer: -1, bot_layer: 0, assigned: false,
            stack_alias: 0, status: 1, h_id: 7, l_id: 8, edges: Vec::new(),
        },
        SpiralNode {
            x: 9, y: 4, top_layer: -1, bot_layer: 0, assigned: false,
            stack_alias: 1, status: 2, h_id: 5, l_id: 6, edges: Vec::new(),
        },
    ];
    let before = nodes.clone();
    let mut grid = EstimateGrid::new(16, 16);
    let zero = |_x: usize, _y: usize| -> u16 { 0 };

    let got = spiral_route(&mut grid, &mut nodes, (0, 1, 0), 1, 0.0, 1000.0, 1000.0, &zero, &zero);

    assert_eq!(got, EdgeRoute::None);
    assert_eq!(nodes, before, "a degenerate edge must not mark, count or route anything");
}
