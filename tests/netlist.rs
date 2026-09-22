// SPDX-License-Identifier: Apache-2.0
//! If — I14 `initNetlist`: the layer range, the gate that decides which nets reach the router,
//! their router pins and root, NDR edge costs, the net degree (GRT-0001/0002), and the pin-access
//! resources for pad/macro pins.
//!
//! Golden `netlist.json`: whole initNetlist calls (feature carriers first, then smallest-first).
//! The exhaustive replay runs every call of the corpus.

use serde_json::Value;
use vyges_grt::{
    add_resources_for_pin_access, compute_net_degree, compute_track_consumption, find_fastroute_pins, get_net_layer_range,
    has_stacked_vias, initial_net_order_is_kept, makes_fastroute_net, net_max_routing_layer, pin_access_edges,
    report_net_degree, AccessPinFacts, Direction, EdgeState, NdrLayerRule, NetlistGrid, PinEdge, Rect, RouterEdges,
    RouterPinFacts,
};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i32 {
    v.as_i64().expect("integer") as i32
}

fn arr(v: &Value) -> &Vec<Value> {
    v.as_array().expect("array")
}

fn edge_of(o: i32) -> PinEdge {
    [PinEdge::North, PinEdge::South, PinEdge::East, PinEdge::West, PinEdge::None][o as usize]
}

fn state(v: &Value) -> EdgeState {
    EdgeState { cap: int(&v[0]) as u16, red: int(&v[1]) as u16, real_cap: int(&v[2]) as u16 }
}

#[derive(Default)]
struct Seen {
    calls: usize,
    nets: usize,
    made: usize,
    ndr: usize,
    access: usize,
    log_runs: usize,
}

fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        let mut log = Vec::new();
        for c in arr(&r["calls"]) {
            let grid = NetlistGrid {
                x_min: int(&c["x_min"]),
                y_min: int(&c["y_min"]),
                tile_size: int(&c["tile"]),
                x_grids: int(&c["xg"]),
                y_grids: int(&c["yg"]),
                num_layers: int(&c["nl"]),
            };
            let (bmin, bmax, cmin, cmax) = (int(&c["block_min"]), int(&c["block_max"]), int(&c["clk_min"]), int(&c["clk_max"]));
            let nets = arr(&c["nets"]);
            assert!(initial_net_order_is_kept(int(&c["seed"]), nets.len()), "{who}: a seeded shuffle — not implemented");
            let mut degree_input = Vec::new();
            for n in nets {
                let name = n["name"].as_str().expect("name");
                let facts: Vec<&Value> = arr(&n["pin_facts"]).iter().collect();
                let conn: Vec<i32> = facts.iter().map(|p| int(&p[2])).collect();
                let (min, max) = get_net_layer_range(&conn, int(&n["nonleaf_clock"]) == 1, bmin, bmax, cmin, cmax);
                let bterms: Vec<i32> = arr(&n["bterms"]).iter().map(int).collect();
                let made = makes_fastroute_net(facts.len(), int(&n["has_wires"]) == 1, || {
                    has_stacked_vias(int(&n["wire_cnt"]) as u32, int(&n["via_cnt"]) as u32, int(&n["via_points"]) as usize, &bterms, max)
                });
                assert_eq!(made, int(&n["made"]) == 1, "{who}: net {name} reaches the router");
                degree_input.push((facts.len(), made));
                if made {
                    let fr = &n["fr"];
                    assert_eq!((min, max), (int(&fr["min"]), int(&fr["max"])), "{who}: net {name} layer range");
                    let pins: Vec<RouterPinFacts> = facts
                        .iter()
                        .map(|p| RouterPinFacts { on_grid: (int(&p[0]), int(&p[1])), connection_layer: int(&p[2]), is_driver: int(&p[3]) == 1 })
                        .collect();
                    let (got, root) = find_fastroute_pins(&pins, grid, net_max_routing_layer(int(&n["sig_clock"]) == 1, cmax, bmax));
                    let want: Vec<(i32, i32, i32)> = arr(&fr["pins"]).iter().map(|p| (int(&p[0]), int(&p[1]), int(&p[2]))).collect();
                    assert_eq!((got, root as i32), (want, int(&fr["root"])), "{who}: net {name} router pins and root");
                    let rules: Option<Vec<NdrLayerRule>> = (!n["ndr"].is_null()).then(|| {
                        arr(&n["rules"])
                            .iter()
                            .map(|r| {
                                if r[1].as_str() == Some("skip") {
                                    NdrLayerRule { level: int(&r[0]), default_width: 0, default_pitch: 1, ndr_spacing: 0, ndr_width: 0 }
                                } else {
                                    NdrLayerRule { level: int(&r[0]), default_width: int(&r[1]), default_pitch: int(&r[2]), ndr_spacing: int(&r[3]), ndr_width: int(&r[4]) }
                                }
                            })
                            .collect()
                    });
                    let (cost, per_layer) = compute_track_consumption(rules.as_deref(), bmin, bmax, grid.num_layers).expect("no GRT-272");
                    assert_eq!(cost as i32, int(&fr["cost"]), "{who}: net {name} edge cost");
                    let want_pl: Option<Vec<i8>> = (!n["ndr"].is_null()).then(|| arr(&n["ndr"]).iter().map(|v| int(v) as i8).collect());
                    assert_eq!(per_layer, want_pl, "{who}: net {name} per-layer edge costs");
                    seen.made += 1;
                    seen.ndr += rules.is_some() as usize;
                }
                seen.nets += 1;
            }
            let (dmin, dmax) = compute_net_degree(&degree_input);
            assert_eq!((dmin, dmax), (int(&c["degree"][0]), int(&c["degree"][1])), "{who}: net degree");
            report_net_degree(&degree_input, int(&c["verbose"]) == 1, &mut log);

            // Pin access: only with macros or pads, never incremental.
            let dirs: Vec<Option<Direction>> = arr(&c["dirs"])
                .iter()
                .map(|d| match d.as_str() {
                    Some("H") => Some(Direction::Horizontal),
                    Some("V") => Some(Direction::Vertical),
                    _ => None,
                })
                .collect();
            let want_access = arr(&c["access"]);
            if int(&c["macros"]) == 1 && int(&c["incremental"]) == 0 {
                let access_nets: Vec<(bool, Vec<AccessPinFacts>)> = nets
                    .iter()
                    .map(|n| {
                        (
                            int(&n["connected"]) == 1,
                            arr(&n["pin_facts"])
                                .iter()
                                .map(|p| AccessPinFacts {
                                    on_grid: (int(&p[0]), int(&p[1])),
                                    connection_layer: int(&p[2]),
                                    edge: edge_of(int(&p[4])),
                                    connected_to_pad_or_macro: int(&p[5]) == 1,
                                })
                                .collect(),
                        )
                    })
                    .collect();
                let edges = pin_access_edges(&access_nets, grid, &|l| dirs[(l - 1) as usize]);
                let want_edges: Vec<(i32, i32, i32, i32, i32)> = want_access
                    .iter()
                    .map(|a| { let e = &a["edge"]; (int(&e[0]), int(&e[1]), int(&e[2]), int(&e[3]), int(&e[4])) })
                    .collect();
                assert_eq!(edges, want_edges, "{who}: pin-access edges");
                // Each edge from its captured state, one track added.
                for (a, &e) in want_access.iter().zip(&edges) {
                    let (xg, yg, nl) = (grid.x_grids, grid.y_grids, grid.num_layers);
                    let n3 = (xg * yg * nl) as usize;
                    let mut re = RouterEdges {
                        die: Rect::new(grid.x_min, grid.y_min, grid.x_min + xg * grid.tile_size, grid.y_min + yg * grid.tile_size),
                        tile_size: grid.tile_size,
                        x_grid: xg,
                        y_grid: yg,
                        num_layers: nl,
                        track_pitches: vec![],
                        h3: vec![EdgeState::default(); n3],
                        v3: vec![EdgeState::default(); n3],
                        h2: vec![EdgeState::default(); ((xg - 1) * yg) as usize],
                        v2: vec![EdgeState::default(); (xg * (yg - 1)) as usize],
                        horizontal_blocked: Default::default(),
                        vertical_blocked: Default::default(),
                        verbose: false,
                        log: vec![],
                    };
                    let (x1, y1, _, y2, layer) = e;
                    let horizontal = y1 == y2;
                    let i3 = (((layer - 1) * yg + y1) * xg + x1) as usize;
                    let i2 = if horizontal { (y1 * (xg - 1) + x1) as usize } else { (y1 * xg + x1) as usize };
                    let (b3, b2) = (&a["before"][0], &a["before"][1]);
                    if horizontal { re.h3[i3] = state(b3) } else { re.v3[i3] = state(b3) };
                    if !b2.is_null() {
                        if horizontal { re.h2[i2] = state(b2) } else { re.v2[i2] = state(b2) };
                    }
                    add_resources_for_pin_access(&mut re, &[e]);
                    let got3 = if horizontal { re.h3[i3] } else { re.v3[i3] };
                    assert_eq!(got3, state(&a["after"][0]), "{who}: pin-access 3D edge {e:?}");
                    if !a["after"][1].is_null() {
                        let got2 = if horizontal { re.h2[i2] } else { re.v2[i2] };
                        assert_eq!(got2, state(&a["after"][1]), "{who}: pin-access 2D edge {e:?}");
                    }
                    seen.access += 1;
                }
            } else {
                assert!(want_access.is_empty(), "{who}: pin access outside its condition");
            }
            seen.calls += 1;
        }
        if !r["quiet"].as_bool().expect("flag") {
            let want: Vec<&str> = arr(&r["log"]).iter().map(|l| l.as_str().expect("line")).collect();
            assert_eq!(log, want, "{who}: GRT-0001/0002 lines");
            seen.log_runs += 1;
        }
    }
    seen
}

#[test]
fn the_netlist_matches_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/netlist.json", env!("CARGO_MANIFEST_DIR"))));
    assert!(s.calls >= 100 && s.made >= 3000, "calls {}, made {}", s.calls, s.made);
    assert!(s.ndr > 0 && s.access > 0 && s.log_runs >= 90, "ndr {}, access {}, log runs {}", s.ndr, s.access, s.log_runs);
}

/// GRT_NETLIST_FULL=/path/to/if-all.json cargo test --release --test netlist -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_netlist_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_NETLIST_FULL").expect("set GRT_NETLIST_FULL");
    let s = replay(&read(&path));
    eprintln!("exhaustive: {} calls, {} nets, {} made, {} NDR, {} pin-access edges, {} runs' logs", s.calls, s.nets, s.made, s.ndr, s.access, s.log_runs);
}

// ─── Constructed cases: branches the corpus never reaches ───────────────────────────────────

const G: NetlistGrid = NetlistGrid { x_min: 0, y_min: 0, tile_size: 100, x_grids: 10, y_grids: 10, num_layers: 5 };

/// ⛔ A block terminal counts as "above the max layer" only STRICTLY above: one whose lowest box is
/// ON the max layer does not, so one via point no longer matches.
#[test]
fn a_bterm_on_the_max_layer_is_not_above_it() {
    assert!(has_stacked_vias(0, 3, 1, &[6], 5));
    assert!(!has_stacked_vias(0, 3, 1, &[5], 5));
    assert!(!has_stacked_vias(1, 3, 1, &[6], 5), "any wire segment disqualifies");
}

/// ⛔ Router pins cap their layer at the net's max — the CLOCK max for a net whose signal type is
/// clock, when set. The corpus never sets a clock layer range.
#[test]
fn a_clock_net_caps_its_pins_at_the_clock_max() {
    assert_eq!(net_max_routing_layer(true, 3, 5), 3);
    assert_eq!(net_max_routing_layer(false, 3, 5), 5);
    assert_eq!(net_max_routing_layer(true, 0, 5), 5, "unset (0) → the block max");
    let p = [RouterPinFacts { on_grid: (50, 50), connection_layer: 5, is_driver: false }];
    assert_eq!(find_fastroute_pins(&p, G, net_max_routing_layer(true, 3, 5)).0, vec![(0, 0, 3)]);
}

/// ⛔ The root is the LAST driver kept. No corpus net keeps two drivers.
#[test]
fn the_root_is_the_last_driver() {
    let p = |x, d| RouterPinFacts { on_grid: (x, 50), connection_layer: 1, is_driver: d };
    let (pins, root) = find_fastroute_pins(&[p(50, false), p(150, true), p(250, true)], G, 5);
    assert_eq!((pins.len(), root), (3, 2));
}

/// ⛔ With no net through the gate, a verbose run reports degree 0 / 0 — not the `INT_MAX` / 1
/// sentinels. No corpus call is verbose with only single-pin nets.
#[test]
fn an_empty_netlist_reports_zero_degrees() {
    let mut log = Vec::new();
    report_net_degree(&[(1, false)], true, &mut log);
    assert_eq!(log, ["[INFO GRT-0001] Minimum degree: 0", "[INFO GRT-0002] Maximum degree: 0"]);
    assert_eq!(compute_net_degree(&[(1, false)]), (i32::MAX, 1));
}

/// ⛔ Pin access on a HORIZONTAL layer: an east pin gains the edge to its right, any other the edge
/// to its left — none at the grid's first column. The corpus's pin-access pins are all on vertical
/// layers.
#[test]
fn pin_access_on_a_horizontal_layer_faces_the_edge() {
    let pin = |x, edge| AccessPinFacts { on_grid: (x, 450), connection_layer: 1, edge, connected_to_pad_or_macro: true };
    let nets = vec![(true, vec![pin(350, PinEdge::East), pin(350, PinEdge::West), pin(50, PinEdge::West)])];
    let edges = pin_access_edges(&nets, G, &|_| Some(Direction::Horizontal));
    assert_eq!(edges, vec![(3, 4, 4, 4, 1), (2, 4, 3, 4, 1)]);
}
