// SPDX-License-Identifier: Apache-2.0
//! R14 — the per-edge call sequence, end to end.
//!
//! 360 re-routed edges from four designs, 1,582 path points, 279 of them on multi-pin nets. Each
//! record carries the net's whole tree, every threshold and knob, and the usage patch over the
//! search region — everything the sequence reads — and is checked on the path it produced.
//!
//! ⛔ **This validates the THREADING between stages, not the stages.** Each is already gated
//! separately. What this catches is a region computed correctly and then handed to the wrong
//! seeding, or a search whose result is backtraced against the wrong state — the failures that a
//! per-stage corpus cannot see because every stage is individually right.

use serde_json::Value;
use vyges_grt::mazecost::CostParams;
use vyges_grt::{route_one_edge, EdgeContext, EdgeOutcome, MazeEdge, MazeNode, RelaxInputs};

struct Case {
    design: String,
    edge_id: usize,
    num_terminals: usize,
    maze_edge_threshold: i32,
    expand: i32,
    iter: i32,
    is_critical: bool,
    grid: (i32, i32),
    l: i32,
    via: f64,
    h_capacity: i32,
    v_capacity: i32,
    params: CostParams,
    x_range: usize,
    region: (i32, i32, i32, i32),
    nodes: Vec<MazeNode>,
    edges: Vec<MazeEdge>,
    usage: Vec<(i32, i32, Vec<[i32; 4]>)>,
    path: Vec<(i32, i32)>,
}

fn cases() -> Vec<Case> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/e2e.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["edges"].as_array().expect("edges").iter().map(|c| {
        let i = |k: &str| c[k].as_i64().unwrap_or_else(|| panic!("{k}")) as i32;
        let r = c["region"].as_array().expect("region");
        let n = |k: usize| r[k].as_i64().expect("bound") as i32;
        Case {
            design: c["design"].as_str().expect("design").to_string(),
            edge_id: c["edge_id"].as_u64().expect("edge") as usize,
            num_terminals: c["num_terminals"].as_u64().expect("terms") as usize,
            maze_edge_threshold: i("maze_edge_threshold"),
            expand: i("expand"),
            iter: i("iter"),
            is_critical: c["is_critical"].as_bool().expect("crit"),
            grid: (i("x_grid"), i("y_grid")),
            l: i("l"),
            via: f64::from(i("via")),
            h_capacity: i("h_capacity"),
            v_capacity: i("v_capacity"),
            params: CostParams {
                slope: i("slope"),
                logistic_coef: f64::from_bits(c["logis_bits"].as_u64().expect("logis")),
                cost_height: f64::from_bits(c["height_bits"].as_u64().expect("height")),
            },
            x_range: c["x_range"].as_u64().expect("xr") as usize,
            region: (n(0), n(1), n(2), n(3)),
            nodes: c["nodes"].as_array().expect("nodes").iter().map(|d| MazeNode {
                x: d["x"].as_i64().expect("x") as i32,
                y: d["y"].as_i64().expect("y") as i32,
                stack_alias: 0,
                neighbours: d["neighbours"].as_array().expect("nbrs").iter()
                    .map(|p| (p[0].as_u64().expect("n") as usize,
                              p[1].as_u64().expect("e") as usize)).collect(),
            }).collect(),
            edges: c["edges"].as_array().expect("edges").iter().map(|e| MazeEdge {
                n1: e["n1"].as_u64().expect("n1") as usize,
                n2: e["n2"].as_u64().expect("n2") as usize,
                routelen: e["routelen"].as_u64().expect("rl") as usize,
                grids: e["grids"].as_array().expect("grids").iter()
                    .map(|p| (p[0].as_i64().expect("x") as i32,
                              p[1].as_i64().expect("y") as i32)).collect(),
            }).collect(),
            usage: c["usage"].as_array().expect("usage").iter().map(|u| {
                let cells = u["cells"].as_array().expect("cells").iter().map(|q| {
                    let a = q.as_array().expect("quad");
                    [a[0].as_i64().expect("a") as i32, a[1].as_i64().expect("b") as i32,
                     a[2].as_i64().expect("c") as i32, a[3].as_i64().expect("d") as i32]
                }).collect();
                (u["y"].as_i64().expect("y") as i32, u["x0"].as_i64().expect("x0") as i32, cells)
            }).collect(),
            path: c["path"].as_array().expect("path").iter()
                .map(|p| (p[0].as_i64().expect("x") as i32, p[1].as_i64().expect("y") as i32))
                .collect(),
        }
    }).collect()
}

#[test]
fn the_per_edge_sequence_produces_the_reference_path() {
    let all = cases();
    assert!(all.len() >= 300, "corpus too thin: {}", all.len());
    let (mut points, mut multi) = (0usize, 0usize);

    for c in &all {
        let cell = |x: i32, y: i32, k: usize| -> i32 {
            let Some((_, x0, row)) = c.usage.iter().find(|(ry, _, _)| *ry == y) else {
                return 0;
            };
            row.get((x - x0) as usize).map_or(0, |q| q[k])
        };
        let used_h = |x: i32, y: i32| cell(x, y, 0);
        let last_h = |x: i32, y: i32| cell(x, y, 1);
        let used_v = |x: i32, y: i32| cell(x, y, 2);
        let last_v = |x: i32, y: i32| cell(x, y, 3);

        let relax = RelaxInputs {
            l: c.l,
            via: c.via,
            h_capacity: c.h_capacity,
            v_capacity: c.v_capacity,
            params: &c.params,
            used_h: &used_h,
            used_v: &used_v,
            last_h: &last_h,
            last_v: &last_v,
        };
        let ctx = EdgeContext {
            maze_edge_threshold: c.maze_edge_threshold,
            expand: c.expand,
            iter: c.iter,
            is_critical: c.is_critical,
            grid_size: c.grid,
            num_terminals: c.num_terminals,
            edge_cost: 1,
            relax: &relax,
            // Every captured edge reached the search, so the gate said yes for all of them.
            rip_up_says_reroute: true,
        };

        let got = route_one_edge(c.x_range, &c.nodes, &c.edges, c.edge_id, &ctx)
            .unwrap_or_else(|e| panic!("sequence failed on {}: {e}", c.design));

        match got {
            EdgeOutcome::Routed { path, region, src, dest } => {
                // ⛔ The region the sequence computed, not one recomputed beside it: a region
                // handed to the wrong stage usually still yields the same path.
                assert_eq!(region, c.region, "region on {} edge {}", c.design, c.edge_id);
                assert!(!src.is_empty() && !dest.is_empty(), "both frontiers must be seeded");
                for (x, y) in src.iter().chain(dest.iter()) {
                    assert!(
                        *x >= region.0 && *x <= region.1 && *y >= region.2 && *y <= region.3,
                        "a seed at ({x},{y}) is outside the region the search was given, on {}",
                        c.design
                    );
                }
                assert_eq!(
                    path, c.path,
                    "path on {} net edge {} ({} terminals, region {:?})",
                    c.design, c.edge_id, c.num_terminals, c.region
                );
                points += path.len();
            }
            other => panic!("expected a route on {}, got {other:?}", c.design),
        }
        multi += usize::from(c.num_terminals > 2);
    }
    assert!(points >= 1000, "too few path points to be a gate: {points}");
    // ⚠️ Multi-pin nets exercise the subtree traversal; two-pin ones skip it entirely.
    assert!(multi >= 100, "too few multi-pin nets: {multi}");
}

/// ⚠️ Both gates are invisible here, and inherently so: every captured edge **passed** them, so
/// skipping either changes nothing on this corpus. Their boundaries are pinned where they can be
/// — the length gate beside the region, the rip-up gate against its own captured verdicts.
#[test]
fn the_sequence_computes_the_same_region_the_reference_did() {
    use vyges_grt::maze_edge_region;
    for c in &cases() {
        let e = &c.edges[c.edge_id];
        let got = maze_edge_region(
            (c.nodes[e.n1].x, c.nodes[e.n1].y),
            (c.nodes[e.n2].x, c.nodes[e.n2].y),
            c.expand, c.iter, e.routelen as i32, c.is_critical, c.grid,
        );
        assert_eq!(got, c.region, "region on {} edge {}", c.design, c.edge_id);
    }
}

/// ⚠️ What this corpus **cannot** decide, recorded so it is not mistaken for coverage.
///
/// Three mutations survive here and are gated elsewhere:
///
/// | mutation | where it dies |
/// | --- | --- |
/// | the region derived from the edge's span rather than its route length | the region corpus, which has **185** edges where the two differ — this one has **none** |
/// | the length gate skipped | beside the region, where the boundary is constructed |
/// | the rip-up gate ignored | against its own captured verdicts, which include refusals |
///
/// ⛔ Every edge in this corpus **passed** both gates and was routed monotonically, which is what
/// makes all three invisible. That is a property of "edges that got re-routed", not an oversight
/// — but a suite that only had this file would be weaker than it looks.
#[test]
fn the_corpus_cannot_decide_the_route_length_cap() {
    let all = cases();
    let differing = all.iter().filter(|c| {
        let e = &c.edges[c.edge_id];
        let (a, b) = (&c.nodes[e.n1], &c.nodes[e.n2]);
        e.routelen as i32 != (a.x - b.x).abs() + (a.y - b.y).abs()
    }).count();
    assert_eq!(
        differing, 0,
        "{differing} edges now have a route longer than their span — this corpus has become \
         able to decide the route-length cap, and the note above is out of date"
    );
}
