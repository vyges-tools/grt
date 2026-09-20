// SPDX-License-Identifier: Apache-2.0
//! R16 — layer assignment's reset and edge registration.
//!
//! 1,324 nets from four designs, 424 of which alias a node (699 aliased nodes in all).
//!
//! ⛔ **These two passes are the outward walk's, bar two values.** A literal diff of the two
//! reference functions shows every other line identical: the counters start at the reference's
//! infinity rather than zero, and a terminal is marked **1** rather than **2**. They share one
//! implementation here rather than a copy that can drift — and this corpus is what holds that
//! claim, because if either ever diverges further the shared code stops matching.

use serde_json::Value;
use vyges_grt::{register_edges, reset_and_alias, LAYER_RESET, WALK_RESET};

struct Net {
    design: String,
    num_terminals: usize,
    num_layers: i16,
    coords: Vec<(i16, i16)>,
    pin_layers: Vec<i16>,
    edges: Vec<(usize, usize, i32)>,
    alias: Vec<usize>,
    status: Vec<i16>,
    h_id: Vec<i32>,
    l_id: Vec<i32>,
    bot: Vec<i16>,
    top: Vec<i16>,
    node_edges: Vec<Vec<usize>>,
    edge_alias: Vec<(usize, usize)>,
}

fn nets() -> Vec<Net> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/layerassign.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["nets"].as_array().expect("nets").iter().map(|n| {
        let nodes = n["nodes"].as_array().expect("nodes");
        let i = |o: &Value, k: &str| o[k].as_i64().unwrap_or_else(|| panic!("{k}"));
        let num_terminals = n["num_terminals"].as_u64().expect("terms") as usize;
        Net {
            design: n["design"].as_str().expect("design").to_string(),
            num_terminals,
            num_layers: n["num_layers"].as_i64().expect("layers") as i16,
            coords: nodes.iter().map(|d| (i(d, "x") as i16, i(d, "y") as i16)).collect(),
            pin_layers: nodes[..num_terminals].iter()
                .map(|d| i(d, "bot_layer") as i16).collect(),
            edges: n["edges"].as_array().expect("edges").iter()
                .map(|e| (i(e, "n1") as usize, i(e, "n2") as usize, i(e, "len") as i32))
                .collect(),
            alias: nodes.iter().map(|d| i(d, "alias") as usize).collect(),
            status: nodes.iter().map(|d| i(d, "status") as i16).collect(),
            h_id: nodes.iter().map(|d| i(d, "h_id") as i32).collect(),
            l_id: nodes.iter().map(|d| i(d, "l_id") as i32).collect(),
            bot: nodes.iter().map(|d| i(d, "bot_layer") as i16).collect(),
            top: nodes.iter().map(|d| i(d, "top_layer") as i16).collect(),
            node_edges: nodes.iter().map(|d| {
                d["edges"].as_array().expect("eids").iter()
                    .map(|e| e.as_u64().expect("eid") as usize).collect()
            }).collect(),
            edge_alias: n["edges"].as_array().expect("edges").iter()
                .map(|e| (i(e, "n1a") as usize, i(e, "n2a") as usize)).collect(),
        }
    }).collect()
}

#[test]
fn the_reset_matches_the_reference() {
    let all = nets();
    assert!(all.len() >= 1000, "corpus too thin: {}", all.len());
    let mut aliased = 0usize;

    for n in &all {
        let nodes = reset_and_alias(
            &n.coords, n.num_terminals, &n.pin_layers, n.num_layers, LAYER_RESET,
        );
        for (d, node) in nodes.iter().enumerate() {
            assert_eq!(node.stack_alias, n.alias[d], "alias of node {d} on {}", n.design);
            assert_eq!(node.status, n.status[d], "status of node {d} on {}", n.design);
            assert_eq!(node.h_id, n.h_id[d], "first counter of node {d} on {}", n.design);
            assert_eq!(node.l_id, n.l_id[d], "second counter of node {d} on {}", n.design);
            assert_eq!(node.bot_layer, n.bot[d], "bottom layer of node {d} on {}", n.design);
            assert_eq!(node.top_layer, n.top[d], "top layer of node {d} on {}", n.design);
        }
        aliased += nodes.iter().enumerate().filter(|(d, x)| x.stack_alias != *d).count();
    }
    // ⚠️ Without aliased nodes the whole coincidence resolution is decided by nothing.
    assert!(aliased >= 300, "corpus never aliases a node: {aliased}");
}

#[test]
fn the_edge_registration_matches_the_reference() {
    for n in &nets() {
        let mut nodes = reset_and_alias(
            &n.coords, n.num_terminals, &n.pin_layers, n.num_layers, LAYER_RESET,
        );
        let regs = register_edges(&mut nodes, &n.edges);

        let got: Vec<Vec<usize>> = nodes.iter().map(|x| x.edges.clone()).collect();
        assert_eq!(got, n.node_edges, "edge lists on {}", n.design);

        for (e, reg) in regs.iter().enumerate() {
            match reg.alias {
                Some(pair) => assert_eq!(pair, n.edge_alias[e], "edge {e} aliases on {}", n.design),
                // ⚠️ A degenerate edge's alias fields are left untouched by the reference, so the
                // captured values are whatever was there — asserted as unwritten, not as a value.
                None => assert!(n.edges[e].2 <= 0, "edge {e} should have been registered"),
            }
        }
    }
}

/// ⛔ The two stages differ in exactly two values, and both are observable.
#[test]
fn the_two_resets_differ_only_in_the_counters_and_the_terminal_status() {
    let coords = vec![(4, 4), (9, 4), (4, 4)];
    let walk = reset_and_alias(&coords, 2, &[1, 2], 10, WALK_RESET);
    let layer = reset_and_alias(&coords, 2, &[1, 2], 10, LAYER_RESET);

    // Everything structural is identical.
    for (a, b) in walk.iter().zip(&layer) {
        assert_eq!((a.x, a.y), (b.x, b.y));
        assert_eq!(a.stack_alias, b.stack_alias, "the aliasing is the same rule");
        assert_eq!((a.bot_layer, a.top_layer), (b.bot_layer, b.top_layer));
        assert_eq!(a.assigned, b.assigned);
    }
    // And exactly two things are not.
    assert_eq!((walk[0].status, layer[0].status), (2, 1), "terminals are marked differently");
    assert_eq!((walk[2].status, layer[2].status), (0, 0), "but Steiner nodes are not");
    assert_eq!(walk[0].h_id, 0);
    assert_eq!(layer[0].h_id, 1_000_000_000, "the counters start at the reference's infinity");
    assert_eq!(walk[0].l_id, 0);
    assert_eq!(layer[0].l_id, 1_000_000_000);
}
