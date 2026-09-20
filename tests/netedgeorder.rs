// SPDX-License-Identifier: Apache-2.0
//! R14 piece 2 — the order a net's own edges are routed in.
//!
//! 1,476 nets from four designs. A net's edges are sorted by **routed length, longest first**,
//! and the sort is **stable**.
//!
//! ⛔ **Stability is the specification here, and the corpus proves it can be tested.** 673 of the
//! captured cases have a tie group *and* required the sort to move something — those are the ones
//! where an unstable sort could give a different answer. A corpus with ties but nothing moving
//! would say nothing about stability at all, which is why that count is asserted rather than
//! assumed.

use serde_json::Value;
use vyges_grt::{netedge_order_dec, OrderNetEdge};

struct Net {
    design: String,
    input: Vec<(usize, i32)>,
    output: Vec<(usize, i32)>,
}

fn nets() -> Vec<Net> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/netedgeorder.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let pairs = |v: &Value, k: &str| -> Vec<(usize, i32)> {
        v[k].as_array().unwrap_or_else(|| panic!("{k}")).iter()
            .map(|e| (e[0].as_u64().expect("id") as usize, e[1].as_i64().expect("len") as i32))
            .collect()
    };
    v["nets"].as_array().expect("nets").iter().map(|n| Net {
        design: n["design"].as_str().expect("design").to_string(),
        input: pairs(n, "input"),
        output: pairs(n, "output"),
    }).collect()
}

/// Whether an unstable sort could have produced a different answer on this net.
fn distinguishes(n: &Net) -> bool {
    let mut counts: std::collections::HashMap<i32, usize> = Default::default();
    for (_, len) in &n.output {
        *counts.entry(*len).or_default() += 1;
    }
    let tied = counts.values().any(|c| *c > 1);
    let moved: Vec<usize> = n.input.iter().map(|(e, _)| *e).collect();
    let after: Vec<usize> = n.output.iter().map(|(e, _)| *e).collect();
    tied && moved != after
}

#[test]
fn edge_orders_match_the_reference() {
    let nets = nets();
    assert!(nets.len() >= 1000, "corpus too thin: {}", nets.len());

    for n in &nets {
        // The input is always the edges in index order with their routed lengths, which is what
        // the reference builds before sorting.
        let routelens: Vec<i32> = n.input.iter().map(|(_, l)| *l).collect();
        assert_eq!(
            n.input.iter().map(|(e, _)| *e).collect::<Vec<_>>(),
            (0..n.input.len()).collect::<Vec<_>>(),
            "the input is not in edge-index order on {}", n.design
        );

        let got = netedge_order_dec(&routelens);
        let want: Vec<OrderNetEdge> = n.output.iter()
            .map(|&(edge_id, length)| OrderNetEdge { edge_id, length })
            .collect();
        assert_eq!(got, want, "edge order on {}", n.design);
    }
}

/// ⛔ Without cases where stability actually decides, the test above would pass an unstable sort.
#[test]
fn the_corpus_can_tell_stable_from_unstable() {
    let nets = nets();
    let deciding = nets.iter().filter(|n| distinguishes(n)).count();
    assert!(
        deciding >= 300,
        "only {deciding} cases could distinguish a stable sort — the gate would be vacuous"
    );

    let biggest = nets.iter()
        .flat_map(|n| {
            let mut counts: std::collections::HashMap<i32, usize> = Default::default();
            for (_, len) in &n.output {
                *counts.entry(*len).or_default() += 1;
            }
            counts.into_values().max()
        })
        .max()
        .unwrap_or(0);
    assert!(biggest >= 10, "the largest tie group is only {biggest} edges");
}

/// Longest first, and ties keep index order.
#[test]
fn the_order_is_descending_and_ties_keep_index_order() {
    let got = netedge_order_dec(&[3, 7, 3, 9, 7, 3]);
    assert_eq!(
        got.iter().map(|e| e.edge_id).collect::<Vec<_>>(),
        vec![3, 1, 4, 0, 2, 5],
        "9 first, then the two 7s in index order, then the three 3s in index order"
    );
    assert_eq!(got.iter().map(|e| e.length).collect::<Vec<_>>(), vec![9, 7, 7, 3, 3, 3]);
}

/// ⚠️ The length sorted on is the **routed** length, not the distance between the endpoints — so a
/// detour raises an edge's priority. A zero-length route sorts last rather than being skipped.
#[test]
fn a_zero_length_route_sorts_last_and_is_not_dropped() {
    let got = netedge_order_dec(&[0, 5, 0, 2]);
    assert_eq!(got.iter().map(|e| e.edge_id).collect::<Vec<_>>(), vec![1, 3, 0, 2]);
    assert_eq!(got.len(), 4, "every edge is present, however short");
}
