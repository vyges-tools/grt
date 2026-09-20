// SPDX-License-Identifier: Apache-2.0
//! R9 `spiralRouteAll` — passes 1 to 3 against a reference-captured golden.
//!
//! The trace is taken after passes 1 and 2 have completed and while pass 3 drains its queue, so
//! every case carries both the inputs and each output in one record.
//!
//! ⛔ **Five designs, because most of them cannot reach the alias branch.** Swept across all 147
//! traceable `grt` cases: only `gcd_flute` (266 aliased nodes) and `overlapping_edges` (43) ever
//! collapse a coincident Steiner node at any scale. Every PD-built design gives 0 or 1, so a
//! corpus without a FLUTE case validates the alias resolution against nothing at all.
//!
//! ⚠️ `spiralRouteAll` runs once per rip-up iteration, so each net appears many times with a
//! different tree. Those are kept as separate cases — a first extractor attached every
//! iteration's visit order to the FIRST record for each net id, which silently produced a corpus
//! where most cases had no visits to check.
//!
//! Pass 4 is covered separately: its golden would have to be read after the per-edge routing has
//! mutated the statuses, which is the next stage, so what is checked here is its rule applied to
//! the captured pass-1 statuses, not a captured pass-4 value.

use serde_json::Value;
use vyges_grt::{propagate_alias_status, register_edges, reset_and_alias, traversal_order, WALK_RESET};

/// `num_layers` only ever lands on a node that is never given a pin layer, and the trace records
/// what the reference used.
const NUM_LAYERS: i16 = 10;

struct Case {
    name: String,
    num_terminals: usize,
    coords: Vec<(i16, i16)>,
    pin_layers: Vec<i16>,
    edges: Vec<(usize, usize, i32)>,
    expect_alias: Vec<usize>,
    expect_status: Vec<i16>,
    expect_node_edges: Vec<Vec<usize>>,
    expect_edge_alias: Vec<(usize, usize)>,
    zero_len_alias_in_trace: Vec<(usize, usize)>,
    expect_edge_assigned: Vec<bool>,
    expect_visit_order: Vec<usize>,
}

fn ints(v: &Value, key: &str) -> Vec<i64> {
    v[key]
        .as_array()
        .unwrap_or_else(|| panic!("{key} is an array"))
        .iter()
        .map(|x| x.as_i64().expect("integer"))
        .collect()
}

fn pairs(v: &Value, key: &str) -> Vec<(i64, i64)> {
    v[key]
        .as_array()
        .unwrap_or_else(|| panic!("{key} is an array"))
        .iter()
        .map(|p| (p[0].as_i64().expect("int"), p[1].as_i64().expect("int")))
        .collect()
}

fn cases() -> Vec<Case> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/spiral.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["cases"]
        .as_array()
        .expect("cases array")
        .iter()
        .map(|c| Case {
            name: c["name"].as_str().expect("name").to_string(),
            num_terminals: c["num_terminals"].as_u64().expect("terms") as usize,
            coords: pairs(c, "coords")
                .into_iter()
                .map(|(x, y)| (x as i16, y as i16))
                .collect(),
            pin_layers: ints(c, "pin_layers").into_iter().map(|l| l as i16).collect(),
            edges: c["edges"]
                .as_array()
                .expect("edges")
                .iter()
                .map(|e| {
                    (
                        e[0].as_u64().expect("n1") as usize,
                        e[1].as_u64().expect("n2") as usize,
                        e[2].as_i64().expect("len") as i32,
                    )
                })
                .collect(),
            expect_alias: ints(c, "expect_alias").into_iter().map(|a| a as usize).collect(),
            expect_status: ints(c, "expect_status").into_iter().map(|s| s as i16).collect(),
            expect_node_edges: c["expect_node_edges"]
                .as_array()
                .expect("node edges")
                .iter()
                .map(|l| {
                    l.as_array()
                        .expect("list")
                        .iter()
                        .map(|e| e.as_u64().expect("eid") as usize)
                        .collect()
                })
                .collect(),
            expect_edge_alias: pairs(c, "expect_edge_alias")
                .into_iter()
                .map(|(a, b)| (a as usize, b as usize))
                .collect(),
            zero_len_alias_in_trace: pairs(c, "expect_edge_alias")
                .into_iter()
                .map(|(a, b)| (a as usize, b as usize))
                .collect(),
            expect_edge_assigned: c["expect_edge_assigned"]
                .as_array()
                .expect("assigned")
                .iter()
                .map(|b| b.as_bool().expect("bool"))
                .collect(),
            expect_visit_order: ints(c, "expect_visit_order")
                .into_iter()
                .map(|e| e as usize)
                .collect(),
        })
        .collect()
}

type Run = (
    Vec<usize>,
    Vec<i16>,
    Vec<Vec<usize>>,
    Vec<Option<(usize, usize)>>,
    Vec<bool>,
    Vec<usize>,
);

fn run(c: &Case) -> Run {
    let mut nodes = reset_and_alias(&c.coords, c.num_terminals, &c.pin_layers, NUM_LAYERS, WALK_RESET);
    let alias: Vec<usize> = nodes.iter().map(|n| n.stack_alias).collect();
    let status: Vec<i16> = nodes.iter().map(|n| n.status).collect();
    let edges = register_edges(&mut nodes, &c.edges);
    let node_edges: Vec<Vec<usize>> = nodes.iter().map(|n| n.edges.clone()).collect();
    let edge_alias: Vec<Option<(usize, usize)>> = edges.iter().map(|e| e.alias).collect();
    let edge_assigned: Vec<bool> = edges.iter().map(|e| e.assigned).collect();
    let order = traversal_order(&mut nodes, &edges, c.num_terminals);
    (alias, status, node_edges, edge_alias, edge_assigned, order)
}

#[test]
fn aliases_and_statuses_match_the_reference() {
    let cases = cases();
    assert!(cases.len() >= 1000, "corpus too thin: {}", cases.len());
    let mut aliased = 0usize;
    for c in &cases {
        let (alias, status, ..) = run(c);
        assert_eq!(alias, c.expect_alias, "alias mismatch on {}", c.name);
        assert_eq!(status, c.expect_status, "status mismatch on {}", c.name);
        aliased += alias.iter().enumerate().filter(|(d, &a)| a != *d).count();
    }
    // ⚠️ Without this the whole alias resolution could be `stack_alias = d` and still pass.
    assert!(aliased >= 100, "corpus never aliases a node: {aliased}");
}

#[test]
fn edge_registration_matches_the_reference() {
    for c in &cases() {
        let (_, _, node_edges, edge_alias, edge_assigned, _) = run(c);
        assert_eq!(node_edges, c.expect_node_edges, "eID list mismatch on {}", c.name);
        // ⛔ A zero-length edge has no alias pair at all, and the reference's untouched field
        // reading back as 0 is asserted separately so it stays a recorded observation rather than
        // something the implementation depends on.
        for (e, got) in edge_alias.iter().enumerate() {
            match got {
                Some(pair) => {
                    assert_eq!(*pair, c.expect_edge_alias[e], "n1a/n2a on edge {e} of {}", c.name)
                }
                None => assert_eq!(
                    c.zero_len_alias_in_trace[e],
                    (0, 0),
                    "zero-length edge {e} of {} had a non-default alias in the trace",
                    c.name
                ),
            }
        }
        assert_eq!(
            edge_assigned, c.expect_edge_assigned,
            "initial assigned mismatch on {}",
            c.name
        );
    }
}

#[test]
fn traversal_visits_edges_in_the_reference_order() {
    let mut total = 0usize;
    for c in &cases() {
        let (.., order) = run(c);
        assert_eq!(order, c.expect_visit_order, "visit order mismatch on {}", c.name);
        total += order.len();
    }
    assert!(total >= 5000, "too few visits to be a gate: {total}");
}

/// Every edge with a positive length is visited exactly once, and a zero-length edge never.
///
/// ⚠️ Worth asserting apart from the order: a visit order can match position by position and
/// still be the wrong SET if the corpus has too little branching, so the branch count is checked
/// too.
#[test]
fn every_positive_length_edge_is_visited_exactly_once() {
    let mut branching = 0usize;
    let mut zero_len = 0usize;
    for c in &cases() {
        let (.., order) = run(c);
        let mut seen = vec![0usize; c.edges.len()];
        for &e in &order {
            seen[e] += 1;
        }
        for (e, &(_, _, len)) in c.edges.iter().enumerate() {
            let want = usize::from(len > 0);
            assert_eq!(seen[e], want, "edge {e} visited {} times on {}", seen[e], c.name);
            zero_len += usize::from(len == 0);
        }
        if c.edges.iter().filter(|&&(_, _, l)| l > 0).count() > 2 {
            branching += 1;
        }
    }
    assert!(branching >= 100, "corpus has too few multi-edge nets: {branching}");
    assert!(zero_len >= 50, "corpus has too few zero-length edges: {zero_len}");
}

/// Pass 4 gives every node the status of the node it aliases to. Terminals are always their own
/// alias, so a Steiner node sitting on a pin inherits the pin's status — the only way a node that
/// pass 1 set to 0 can be non-zero before any routing has run.
#[test]
fn pass_four_copies_status_from_the_alias() {
    let mut inherited = 0usize;
    for c in &cases() {
        let mut nodes = reset_and_alias(&c.coords, c.num_terminals, &c.pin_layers, NUM_LAYERS, WALK_RESET);
        propagate_alias_status(&mut nodes);
        for (d, node) in nodes.iter().enumerate() {
            let na = c.expect_alias[d];
            assert_eq!(node.status, c.expect_status[na], "node {d} of {}", c.name);
            if na != d && node.status != c.expect_status[d] {
                inherited += 1;
            }
        }
    }
    assert!(inherited > 0, "no node actually inherited a status: vacuous");
}

// ---------------------------------------------------------------------------
// Constructed cases for two rules that no shipped design reaches.
//
// ⚠️ These are synthetic, and that is the point: both rules were read out of the reference and
// then found to survive a deliberate mutation across all 7,088 captured net states. A rule with
// no witness is a rule the gate does not hold, so the topology that separates it is built here by
// hand rather than left uncovered.
// ---------------------------------------------------------------------------

/// A Steiner node aliases onto the **first** recorded node at its coordinate, not the last.
///
/// ⛔ This can only differ where one coordinate holds **two or more terminals**, because a
/// matched Steiner node is never itself recorded — so a coordinate otherwise has exactly one
/// candidate and first and last coincide. Measured across the corpus: zero coordinates have two
/// terminals and an aliased node, so every captured case is blind to the distinction.
///
/// Upstream states the rule in its own words at `DataType.h`: the duplicate takes "the index of
/// the first node", and that it applies to Steiner nodes only, never pins.
#[test]
fn alias_resolves_to_the_first_recorded_node() {
    // Two terminals stacked on one coordinate, then a Steiner node on the same spot.
    let coords = vec![(4, 7), (4, 7), (4, 7), (9, 9)];
    let nodes = reset_and_alias(&coords, 2, &[1, 2], NUM_LAYERS, WALK_RESET);

    assert_eq!(nodes[0].stack_alias, 0, "a terminal is always its own alias");
    assert_eq!(nodes[1].stack_alias, 1, "a terminal never aliases, even onto another terminal");
    assert_eq!(nodes[2].stack_alias, 0, "the Steiner node takes the FIRST of the two, not the last");
    assert_eq!(nodes[3].stack_alias, 3, "a Steiner node with no coincidence stays itself");

    // The terminals keep their own pin layers rather than sharing through the alias.
    assert_eq!((nodes[0].bot_layer, nodes[0].top_layer), (1, 1));
    assert_eq!((nodes[1].bot_layer, nodes[1].top_layer), (2, 2));
}

/// The traversal is seeded from the **terminals only** and discovers the rest breadth-first.
///
/// ⛔ Seeding from every node instead gives the same answer on all 7,088 captured net states,
/// because the designs happen to index Steiner nodes in discovery order. It is not an
/// equivalence: it needs a Steiner node indexed *below* another that lies on its only path to a
/// terminal, which is what this tree is.
///
/// ```text
///   0 ──e0── 3 ──e2── 4 ──e3── 2        0, 1 terminals;  2, 3, 4 Steiner
///            │                          node 2 is reachable only through 4, then 3
///   1 ──e1───┘
/// ```
///
/// Breadth-first from the terminals reaches 3 before 4 before 2, giving `e0 e1 e2 e3`. Walking
/// the nodes in index order would queue node 2's edge `e3` before node 3's `e2`, giving
/// `e0 e1 e3 e2`.
#[test]
fn traversal_is_seeded_from_terminals_not_every_node() {
    let coords = vec![(0, 0), (10, 0), (5, 5), (1, 1), (3, 3)];
    let edges = vec![(0usize, 3usize, 4i32), (1, 3, 4), (3, 4, 4), (4, 2, 4)];

    let mut nodes = reset_and_alias(&coords, 2, &[0, 0], NUM_LAYERS, WALK_RESET);
    let regs = register_edges(&mut nodes, &edges);
    let order = traversal_order(&mut nodes, &regs, 2);

    assert_eq!(order, vec![0, 1, 2, 3], "breadth-first from the terminals");
    assert_ne!(order, vec![0, 1, 3, 2], "this is the node-index order the rule must not produce");
}

/// A zero-length edge is never traversed, however it is attached.
#[test]
fn zero_length_edges_are_never_visited() {
    let coords = vec![(0, 0), (6, 0), (0, 0)];
    // e1 is degenerate: node 2 sits exactly on terminal 0.
    let edges = vec![(0usize, 1usize, 6i32), (0, 2, 0i32)];

    let mut nodes = reset_and_alias(&coords, 2, &[0, 0], NUM_LAYERS, WALK_RESET);
    let regs = register_edges(&mut nodes, &edges);

    assert_eq!(regs[1].alias, None, "the reference leaves a degenerate edge's aliases unwritten");
    assert!(regs[1].assigned, "and marks it assigned so it is never queued");
    assert_eq!(traversal_order(&mut nodes, &regs, 2), vec![0]);
}
