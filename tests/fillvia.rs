// SPDX-License-Identifier: Apache-2.0
//! R19 — `fillVIA`, the via stacks at a net's nodes.
//!
//! Golden `fillvia.json`: per net, the state the pass read, the arm each edge took and which end
//! claimed a stack, the state it left, and the net's share of the pass's two counters. Captured in
//! both cost modes, because layer assignment — which decides every node's layer range and which
//! edge carries its stack — reads the resistance-aware weights.

use serde_json::Value;
use vyges_grt::{
    fill_via, get_via_stack_range, EdgeFill, EndClaim, Point3D, RouteType, ViaCounts, ViaEdge,
    ViaNet, ViaNode, ViaPin, NO_EDGE,
};

fn golden() -> Value {
    read(&format!("{}/examples/grt_gate/fillvia.json", env!("CARGO_MANIFEST_DIR")))
}

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn pts(v: &Value) -> Vec<Point3D> {
    v.as_array().expect("pts").iter().map(|p| Point3D {
        x: int(&p[0]) as i16,
        y: int(&p[1]) as i16,
        layer: int(&p[2]) as i16,
    }).collect()
}

fn route_type(v: &Value) -> RouteType {
    match int(v) {
        0 => RouteType::NoRoute,
        1 => RouteType::LRoute,
        2 => RouteType::ZRoute,
        3 => RouteType::MazeRoute,
        t => panic!("route type {t}"),
    }
}

fn edge(v: &Value) -> ViaEdge {
    ViaEdge {
        len: int(&v["len"]) as i32,
        n1: int(&v["n1"]) as usize,
        n2: int(&v["n2"]) as usize,
        n1a: int(&v["n1a"]) as usize,
        n2a: int(&v["n2a"]) as usize,
        route_type: route_type(&v["type"]),
        routelen: int(&v["routelen"]) as i32,
        grids: pts(&v["grids"]),
    }
}

fn net(r: &Value) -> ViaNet {
    ViaNet {
        num_terminals: int(&r["num_terminals"]) as usize,
        nodes: r["nodes"].as_array().expect("nodes").iter().map(|n| ViaNode {
            x: int(&n["x"]) as i16,
            y: int(&n["y"]) as i16,
            bot_layer: int(&n["botL"]) as i16,
            top_layer: int(&n["topL"]) as i16,
            h_id: int(&n["hID"]) as i32,
            l_id: int(&n["lID"]) as i32,
            stack_alias: int(&n["alias"]) as usize,
        }).collect(),
        edges: r["edges"].as_array().expect("edges").iter().map(edge).collect(),
        pins: r["pins"].as_array().expect("pins").iter().map(|p| ViaPin {
            x: int(&p[0]) as i32,
            y: int(&p[1]) as i32,
            layer: int(&p[2]) as i32,
        }).collect(),
    }
}

/// Render what the engine decided in the trace's own vocabulary, so the two sequences compare as
/// lists: `S1`/`S2` for a claimed end, `B` for a positive-length edge's point count, `ZE` for a
/// zero-length edge reaching the resolve, `Z` for one reaching the range test.
fn render(fills: &[EdgeFill]) -> Vec<Vec<i64>> {
    let mut out = Vec::new();
    // tags: S1 = 1, S2 = 2, B = 3, ZE = 4, Z = 5
    for (e, f) in fills.iter().enumerate() {
        let e = e as i64;
        match *f {
            EdgeFill::Positive { first, second, points } => {
                if let Some(c) = first {
                    out.push(vec![1, e, c.node as i64, i64::from(c.terminal)]);
                }
                if let Some(c) = second {
                    out.push(vec![2, e, c.node as i64, i64::from(c.terminal)]);
                }
                out.push(vec![3, e, i64::from(points)]);
            }
            EdgeFill::ZeroSkipped { effective: (a, b) } => {
                out.push(vec![4, e, a as i64, b as i64]);
            }
            EdgeFill::Zero { effective: (a, b), bottom, top } => {
                out.push(vec![4, e, a as i64, b as i64]);
                out.push(vec![5, e, a as i64, b as i64, i64::from(bottom), i64::from(top)]);
            }
        }
    }
    out
}

fn want_seq(r: &Value) -> Vec<Vec<i64>> {
    r["seq"].as_array().expect("seq").iter().map(|s| {
        let s = s.as_array().expect("step");
        let tag = match s[0].as_str().expect("tag") {
            "S1" => 1,
            "S2" => 2,
            "B" => 3,
            "ZE" => 4,
            "Z" => 5,
            t => panic!("tag {t}"),
        };
        std::iter::once(tag).chain(s[1..].iter().map(int)).collect()
    }).collect()
}

// ─── Against the reference ──────────────────────────────────────────────────────────────────

/// Every captured net: the call sequence, every edge after, and the two counters.
#[test]
fn fill_via_matches_the_reference() {
    let edges_checked = replay(&golden());
    assert!(edges_checked >= 1_000, "too few edges: {edges_checked}");
}

/// The whole uncapped corpus, every distinct net of both cost modes.
///
/// GRT_FILLVIA_FULL=/path/to/fillvia-all.json cargo test --test fillvia -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn fill_via_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_FILLVIA_FULL").expect("set GRT_FILLVIA_FULL");
    let edges_checked = replay(&read(&path));
    eprintln!("exhaustive: {edges_checked} edges");
}

/// Replay each record; returns the number of edges compared.
fn replay(g: &Value) -> usize {
    let records = g["records"].as_array().expect("records");
    assert!(records.len() >= 100, "too few nets: {}", records.len());
    let mut edges_checked = 0usize;

    for r in records {
        let who = format!("{} net {}", r["design"].as_str().expect("design"), r["net_id"]);
        let mut nets = vec![net(r)];
        let num_layers = int(&r["num_layers"]) as i16;
        let (counts, fills) = fill_via(&mut nets, num_layers)
            .unwrap_or_else(|e| panic!("{who}: fatal {e:?}"));

        assert_eq!(render(&fills[0]), want_seq(r), "{who}: call sequence");
        let after: Vec<ViaEdge> = r["after"].as_array().expect("after").iter().map(edge).collect();
        assert_eq!(nets[0].edges.len(), after.len(), "{who}: edge count");
        for (i, (got, want)) in nets[0].edges.iter().zip(&after).enumerate() {
            assert_eq!(got, want, "{who}: edge {i} after");
        }
        assert_eq!(
            counts,
            ViaCounts {
                pin_nodes: int(&r["want_t1"]) as i32,
                steiner_nodes: int(&r["want_t2"]) as i32,
            },
            "{who}: counters"
        );
        edges_checked += after.len();
    }
    edges_checked
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn p(x: i16, y: i16, layer: i16) -> Point3D {
    Point3D { x, y, layer }
}

fn node(x: i16, y: i16, bot: i16, top: i16, h_id: i32, l_id: i32, alias: usize) -> ViaNode {
    ViaNode { x, y, bot_layer: bot, top_layer: top, h_id, l_id, stack_alias: alias }
}

fn straight(len: i32, n1: usize, n2: usize, layer: i16) -> ViaEdge {
    ViaEdge {
        len,
        n1,
        n2,
        n1a: n1,
        n2a: n2,
        route_type: RouteType::LRoute,
        routelen: len,
        grids: (0..=len as i16).map(|x| p(x, 0, layer)).collect(),
    }
}

/// ⛔ The pin range is matched by POSITION over all the net's pins, and comes back INVERTED when
/// no pin sits there.
#[test]
fn the_via_stack_range_is_by_position_and_inverted_when_empty() {
    let pins = [
        ViaPin { x: 3, y: 4, layer: 2 },
        ViaPin { x: 3, y: 4, layer: 5 },
        ViaPin { x: 9, y: 4, layer: 0 },
    ];
    let n = node(3, 4, 0, 0, NO_EDGE, NO_EDGE, 0);
    assert_eq!(get_via_stack_range(&pins, &n), (2, 5));
    let empty = node(1, 1, 0, 0, NO_EDGE, NO_EDGE, 0);
    assert_eq!(get_via_stack_range(&pins, &empty), (i16::MAX, -1));
}

/// ⛔ The "only if vias were added" guard compares points to steps and so is always true: an edge
/// that claims nothing is still rewritten, restamped as a maze route, and trimmed to the points
/// it walked.
#[test]
fn every_positive_length_edge_is_restamped_and_trimmed() {
    let mut e = straight(2, 0, 1, 1);
    e.grids.push(p(7, 7, 7)); // past `routelen`, dropped by the rewrite
    let mut nets = vec![ViaNet {
        num_terminals: 2,
        // Neither node names edge 0 as its highest or lowest.
        nodes: vec![node(0, 0, 1, 1, 5, 5, 0), node(2, 0, 1, 1, 5, 5, 1)],
        edges: vec![e],
        pins: vec![],
    }];
    let (counts, fills) = fill_via(&mut nets, 4).expect("routes");
    assert_eq!(counts, ViaCounts::default());
    assert_eq!(fills[0], vec![EdgeFill::Positive { first: None, second: None, points: 3 }]);
    let got = &nets[0].edges[0];
    assert_eq!(got.route_type, RouteType::MazeRoute);
    assert_eq!(got.routelen, 2);
    assert_eq!(got.grids, vec![p(0, 0, 1), p(1, 0, 1), p(2, 0, 1)]);
}

/// ⛔ A terminal's first-end stack goes UP from the bottom to below the top, then DOWN from the top
/// to above the edge's layer. Only the upward run is counted.
#[test]
fn a_first_end_terminal_stack_climbs_then_descends() {
    let mut nets = vec![ViaNet {
        num_terminals: 2,
        nodes: vec![node(0, 0, 1, 3, 0, NO_EDGE, 0), node(2, 0, 2, 2, 7, 7, 1)],
        edges: vec![straight(2, 0, 1, 2)],
        // A second pin at the first node widens its range down to 0.
        pins: vec![ViaPin { x: 0, y: 0, layer: 0 }, ViaPin { x: 2, y: 0, layer: 2 }],
    }];
    let (counts, _) = fill_via(&mut nets, 5).expect("routes");
    assert_eq!(
        nets[0].edges[0].grids,
        vec![p(0, 0, 0), p(0, 0, 1), p(0, 0, 2), p(0, 0, 3), p(0, 0, 2), p(1, 0, 2), p(2, 0, 2)],
    );
    assert_eq!(counts.pin_nodes, 3);
}

/// ⛔ The lowest-layer edge carries the stack only when no edge rises above the node AND the node
/// is a terminal. The same bookkeeping on a Steiner node claims nothing.
#[test]
fn the_lowest_layer_edge_claims_only_at_a_terminal() {
    let mk = |terminals: usize| ViaNet {
        num_terminals: terminals,
        nodes: vec![node(0, 0, 1, 1, NO_EDGE, 0, 0), node(2, 0, 1, 1, 9, 9, 1)],
        edges: vec![straight(2, 0, 1, 1)],
        pins: vec![],
    };
    let mut at_pin = vec![mk(1)];
    let (_, f) = fill_via(&mut at_pin, 4).expect("routes");
    assert!(matches!(f[0][0], EdgeFill::Positive { first: Some(EndClaim { node: 0, terminal: true }), .. }));
    let mut at_steiner = vec![mk(0)];
    let (_, f) = fill_via(&mut at_steiner, 4).expect("routes");
    assert!(matches!(f[0][0], EdgeFill::Positive { first: None, .. }));
}

/// ⛔ At the second end, a Steiner node's vias are counted only when the FIRST end's node is also
/// a Steiner node — the reference gates the count on the other end.
#[test]
fn the_second_end_steiner_count_is_gated_on_the_first_end() {
    let mk = |terminals: usize| ViaNet {
        num_terminals: terminals,
        // node 1 is a Steiner node spanning layers 0..=2; edge 0 is its highest.
        nodes: vec![node(0, 0, 2, 2, 9, 9, 0), node(2, 0, 0, 2, 0, NO_EDGE, 1)],
        edges: vec![straight(2, 0, 1, 2)],
        pins: vec![],
    };
    // First end a terminal: two vias added, none counted.
    let mut a = vec![mk(1)];
    let (ca, _) = fill_via(&mut a, 4).expect("routes");
    // First end a Steiner node too: the same two vias, both counted.
    let mut b = vec![mk(0)];
    let (cb, _) = fill_via(&mut b, 4).expect("routes");
    assert_eq!(a[0].edges[0].grids, b[0].edges[0].grids);
    assert_eq!(a[0].edges[0].grids[3..], [p(2, 0, 1), p(2, 0, 0)]);
    assert_eq!(ca.steiner_nodes, 0);
    assert_eq!(cb.steiner_nodes, 2);
}

/// ⚠️ A second-end terminal whose pins sit ABOVE the arrival layer gets no climb: the descending
/// run is empty and the ascending one starts at the bottom, so the layers between are skipped.
#[test]
fn a_second_end_stack_does_not_climb_from_below_the_bottom() {
    let mut nets = vec![ViaNet {
        num_terminals: 2,
        nodes: vec![node(0, 0, 1, 1, 9, 9, 0), node(2, 0, 3, 4, 0, NO_EDGE, 1)],
        edges: vec![straight(2, 0, 1, 1)],
        pins: vec![],
    }];
    fill_via(&mut nets, 6).expect("routes");
    assert_eq!(
        nets[0].edges[0].grids,
        vec![p(0, 0, 1), p(1, 0, 1), p(2, 0, 1), p(2, 0, 3), p(2, 0, 4)],
    );
}

/// ⛔ A positive length with no steps is the reference's fatal error — raised after the first
/// end's stack was counted, and leaving the edge unwritten.
#[test]
fn a_positive_length_without_steps_is_fatal() {
    let mut e = straight(1, 0, 1, 1);
    e.routelen = 0;
    let before = e.clone();
    let mut nets = vec![ViaNet {
        num_terminals: 2,
        nodes: vec![node(0, 0, 1, 1, 0, NO_EDGE, 0), node(1, 0, 1, 1, 9, 9, 1)],
        edges: vec![e],
        pins: vec![],
    }];
    let err = fill_via(&mut nets, 4).expect_err("fatal");
    assert_eq!((err.net, err.edge), (0, 0));
    assert_eq!(nets[0].edges[0], before);
}

fn zero(n1: usize, n2: usize, len: i32) -> ViaEdge {
    ViaEdge {
        len,
        n1,
        n2,
        n1a: 0,
        n2a: 0,
        route_type: RouteType::NoRoute,
        routelen: 0,
        grids: vec![],
    }
}

/// ⛔ The zero-length range is the span of the two ends' BOTTOM layers — neither top is read.
#[test]
fn a_zero_length_stack_spans_the_two_bottoms_only() {
    let mut nets = vec![ViaNet {
        num_terminals: 0,
        nodes: vec![node(5, 5, 1, 4, 9, 9, 0), node(5, 5, 3, 6, 9, 9, 1)],
        edges: vec![zero(0, 1, 0)],
        pins: vec![],
    }];
    let (_, f) = fill_via(&mut nets, 8).expect("routes");
    assert_eq!(f[0][0], EdgeFill::Zero { effective: (0, 1), bottom: 1, top: 3 });
    let e = &nets[0].edges[0];
    assert_eq!(e.grids, vec![p(5, 5, 1), p(5, 5, 2), p(5, 5, 3)]);
    assert_eq!((e.routelen, e.route_type), (2, RouteType::MazeRoute));
}

/// ⚠️ With one end still carrying no layers, its bottom is the layer COUNT, and that becomes the
/// range's top — a stack reaching one layer past the last.
#[test]
fn a_zero_length_stack_against_an_unassigned_end_reaches_the_layer_count() {
    let mut nets = vec![ViaNet {
        num_terminals: 0,
        // node 1 has no layers and aliases to itself, so resolving does not help it.
        nodes: vec![node(5, 5, 2, 2, 9, 9, 0), node(5, 5, 4, -1, 9, 9, 1)],
        edges: vec![zero(0, 1, 0)],
        pins: vec![],
    }];
    let (_, f) = fill_via(&mut nets, 4).expect("routes");
    assert_eq!(f[0][0], EdgeFill::Zero { effective: (0, 1), bottom: 2, top: 4 });
    assert_eq!(nets[0].edges[0].grids.last(), Some(&p(5, 5, 4)));
}

/// ⛔ Resolved through the alias only when the node itself carries no layers; skipped when both
/// resolved ends still carry none. ⛔ A NEGATIVE length takes this arm too.
#[test]
fn a_zero_length_edge_resolves_through_the_alias_and_can_be_skipped() {
    let mut nets = vec![ViaNet {
        num_terminals: 1,
        nodes: vec![
            node(5, 5, 1, 1, 9, 9, 0),
            node(5, 5, 4, -1, 9, 9, 0), // no layers, aliases to the terminal
            node(6, 6, 4, -1, 9, 9, 2),
            node(6, 6, 4, -1, 9, 9, 3),
        ],
        edges: vec![zero(1, 0, 0), zero(2, 3, -1)],
        pins: vec![ViaPin { x: 5, y: 5, layer: 3 }],
    }];
    let (_, f) = fill_via(&mut nets, 4).expect("routes");
    // Node 1 resolves to the terminal; the pin at (5, 5) widens the range up to 3.
    assert_eq!(f[0][0], EdgeFill::Zero { effective: (0, 0), bottom: 1, top: 3 });
    assert_eq!(f[0][1], EdgeFill::ZeroSkipped { effective: (2, 3) });
    assert!(nets[0].edges[1].grids.is_empty());
}

/// ⛔ The lowest-layer edge claims only when NO edge rises above the node — the sentinel test is
/// load-bearing. ⚠️ No captured net has a terminal whose lowest edge differs from a real highest
/// one (0 of 12,987), so only this case separates the rule from "lowest edge at a terminal".
#[test]
fn the_lowest_layer_edge_does_not_claim_when_another_edge_is_highest() {
    let mut nets = vec![ViaNet {
        num_terminals: 2,
        // node 0's highest edge is some OTHER edge (5); edge 0 is merely its lowest.
        nodes: vec![node(0, 0, 1, 3, 5, 0, 0), node(2, 0, 1, 1, 9, 9, 1)],
        edges: vec![straight(2, 0, 1, 1)],
        pins: vec![],
    }];
    let (_, f) = fill_via(&mut nets, 4).expect("routes");
    assert!(matches!(f[0][0], EdgeFill::Positive { first: None, .. }));
}

/// ⛔ Both resolved ends of a zero-length edge extend the range by their pins. ⚠️ When both are
/// terminals the second extension is a no-op — they share a position, so the lookup returns the
/// same range — which is why only a Steiner FIRST end with a terminal SECOND end separates it.
/// 0 of 12,987 captured nets have that shape.
#[test]
fn a_zero_length_edge_extends_by_the_second_ends_pins_too() {
    let mut nets = vec![ViaNet {
        num_terminals: 1,
        // node 1 is a Steiner node WITH layers, so it does not resolve to the terminal.
        nodes: vec![node(5, 5, 2, 2, 9, 9, 0), node(5, 5, 1, 1, 9, 9, 0)],
        edges: vec![zero(1, 0, 0)],
        pins: vec![ViaPin { x: 5, y: 5, layer: 4 }],
    }];
    let (_, f) = fill_via(&mut nets, 6).expect("routes");
    assert_eq!(f[0][0], EdgeFill::Zero { effective: (1, 0), bottom: 1, top: 4 });
}
