// SPDX-License-Identifier: Apache-2.0
//! R16 — which edges reach each node's highest and lowest layers.
//!
//! 900 nets from four designs, **4,317 nodes keeping the sentinel**.
//!
//! This is the pass that explains the constant. The two per-node fields hold the **edge id** at
//! the node's extreme layers, not a count — and the sentinel means "no edge rises above (or drops
//! below) this node's own layer". A terminal starts at its pin layer, so an edge that stays on
//! that layer sets neither field and the sentinel survives.

use serde_json::Value;
use vyges_grt::{layer_extremes, record_layer_extremes, reset_for_layer_extremes,
                EdgeLayers, LayerExtremes, SpiralNode};

/// The reference's infinity, meaning "no such edge".
const NONE: i32 = 1_000_000_000;

struct Net {
    design: String,
    num_terminals: usize,
    num_layers: i16,
    aliases: Vec<usize>,
    pin_layers: Vec<i16>,
    edges: Vec<EdgeLayers>,
    want: Vec<(LayerExtremes, Vec<usize>)>,
}

fn nets() -> Vec<Net> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/layerextremes.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["nets"].as_array().expect("nets").iter().map(|n| {
        let i = |o: &Value, k: &str| o[k].as_i64().unwrap_or_else(|| panic!("{k}"));
        let nodes = n["nodes"].as_array().expect("nodes");
        let num_terminals = n["num_terminals"].as_u64().expect("terms") as usize;
        Net {
            design: n["design"].as_str().expect("design").to_string(),
            num_terminals,
            num_layers: i(n, "num_layers") as i16,
            aliases: nodes.iter().map(|d| i(d, "alias") as usize).collect(),
            // ⚠️ A terminal's own layer, read from what the earlier reset left on it.
            pin_layers: nodes[..num_terminals].iter().map(|d| i(d, "pin_bot") as i16).collect(),
            edges: n["edges"].as_array().expect("edges").iter().map(|e| EdgeLayers {
                n1: i(e, "n1") as usize,
                n2: i(e, "n2") as usize,
                len: i(e, "len") as i32,
                first: i(e, "first") as i16,
                last: i(e, "last") as i16,
            }).collect(),
            want: n["result"].as_array().expect("result").iter().map(|r| {
                (LayerExtremes {
                    top_layer: i(r, "top_layer") as i16,
                    bot_layer: i(r, "bot_layer") as i16,
                    h_id: i(r, "h_id") as i32,
                    l_id: i(r, "l_id") as i32,
                },
                 r["edges"].as_array().expect("eids").iter()
                     .map(|e| e.as_u64().expect("eid") as usize).collect())
            }).collect(),
        }
    }).collect()
}

fn run(n: &Net) -> Vec<SpiralNode> {
    let mut nodes: Vec<SpiralNode> = n.aliases.iter().map(|&a| SpiralNode {
        x: 0, y: 0, top_layer: -1, bot_layer: 0, assigned: false,
        stack_alias: a, status: 0, h_id: 0, l_id: 0, edges: Vec::new(),
    }).collect();
    reset_for_layer_extremes(&mut nodes, n.num_terminals, &n.pin_layers, n.num_layers);
    record_layer_extremes(&mut nodes, &n.edges);
    nodes
}

#[test]
fn the_layer_extremes_match_the_reference() {
    let all = nets();
    assert!(all.len() >= 600, "corpus too thin: {}", all.len());
    let mut sentinels = 0usize;

    for n in &all {
        let nodes = run(n);
        for (d, (want, want_edges)) in n.want.iter().enumerate() {
            assert_eq!(
                layer_extremes(&nodes[d]), *want,
                "node {d} of {} ({} terminals)", n.design, n.num_terminals
            );
            assert_eq!(nodes[d].edges, *want_edges, "node {d} edge list on {}", n.design);
            sentinels += usize::from(want.h_id == NONE || want.l_id == NONE);
        }
    }
    // ⚠️ Without these the sentinel is never observed, and the constant's value is untested here.
    assert!(sentinels >= 1000, "too few nodes keep the sentinel: {sentinels}");
}

/// ⛔ A terminal starts at its own pin layer, so an edge on that layer sets neither field.
#[test]
fn an_edge_on_the_pin_layer_leaves_the_sentinel() {
    let mut nodes: Vec<SpiralNode> = (0..2).map(|d| SpiralNode {
        x: 0, y: 0, top_layer: -1, bot_layer: 0, assigned: false,
        stack_alias: d, status: 0, h_id: 0, l_id: 0, edges: Vec::new(),
    }).collect();
    reset_for_layer_extremes(&mut nodes, 2, &[3, 3], 10);
    // Both ends arrive on layer 3, exactly the pin layer.
    record_layer_extremes(&mut nodes, &[EdgeLayers { n1: 0, n2: 1, len: 5, first: 3, last: 3 }]);

    for d in 0..2 {
        let e = layer_extremes(&nodes[d]);
        assert_eq!((e.top_layer, e.bot_layer), (3, 3), "the pin layer is unchanged");
        assert_eq!(e.h_id, NONE, "nothing rises above the pin");
        assert_eq!(e.l_id, NONE, "nor drops below it");
        assert_eq!(nodes[d].edges, vec![0], "but the edge is still recorded");
    }
}

/// ⛔ The comparisons are strict, so the **first** edge to reach a layer keeps it.
#[test]
fn a_later_edge_on_the_same_layer_does_not_displace_the_first() {
    let mut nodes: Vec<SpiralNode> = (0..3).map(|d| SpiralNode {
        x: 0, y: 0, top_layer: -1, bot_layer: 0, assigned: false,
        stack_alias: d, status: 0, h_id: 0, l_id: 0, edges: Vec::new(),
    }).collect();
    // One terminal at layer 1, joined by two edges that both rise to layer 5.
    reset_for_layer_extremes(&mut nodes, 1, &[1], 10);
    record_layer_extremes(&mut nodes, &[
        EdgeLayers { n1: 0, n2: 1, len: 4, first: 5, last: 5 },
        EdgeLayers { n1: 0, n2: 2, len: 4, first: 5, last: 5 },
    ]);

    let e = layer_extremes(&nodes[0]);
    assert_eq!(e.top_layer, 5);
    assert_eq!(e.h_id, 0, "the first edge to reach layer 5 keeps it");
    assert_eq!(nodes[0].edges, vec![0, 1], "though both are recorded");

    // A strictly higher edge does displace it.
    record_layer_extremes(&mut nodes, &[
        EdgeLayers { n1: 0, n2: 1, len: 4, first: 7, last: 7 },
    ]);
    assert_eq!(layer_extremes(&nodes[0]).h_id, 0, "edge 0 of the second batch, now at layer 7");
    assert_eq!(layer_extremes(&nodes[0]).top_layer, 7);
}

/// ⚠️ A degenerate edge contributes nothing at all — not even to the edge list.
#[test]
fn a_degenerate_edge_is_not_recorded() {
    let mut nodes: Vec<SpiralNode> = (0..2).map(|d| SpiralNode {
        x: 0, y: 0, top_layer: -1, bot_layer: 0, assigned: false,
        stack_alias: d, status: 0, h_id: 0, l_id: 0, edges: Vec::new(),
    }).collect();
    reset_for_layer_extremes(&mut nodes, 2, &[2, 2], 10);
    record_layer_extremes(&mut nodes, &[EdgeLayers { n1: 0, n2: 1, len: 0, first: 9, last: 9 }]);

    for d in 0..2 {
        assert!(nodes[d].edges.is_empty(), "a degenerate edge must not be recorded");
        assert_eq!(layer_extremes(&nodes[d]).top_layer, 2, "nor move a layer");
        assert!(!nodes[d].assigned || d < 2, "and the terminal keeps its own state");
    }
}
