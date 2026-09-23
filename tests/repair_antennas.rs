// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 3 — jumper insertion.
//!
//! End to end, `grt-jumper-score.py` replays the reference's own call sequence (`VYGJ|` trace:
//! every graph node, seed, DFS pop and push, scanned tile with its headroom, candidate and try)
//! with the checker's violations captured: 6 suite scripts, 6,402 lines, 41 jumpers — exact — and
//! the wires built from the re-saved guides after them exact too. What those designs never reach
//! is pinned here on constructed cases, each marked golden-blind:
//!
//! - every seed set had ONE node, so the libc++ `unordered_set` order is replayed against a probe
//!   compiled with the reference's own hermetic libc++ (`libcxx-unordered-set-probe.cpp`);
//! - FastRoute never rejects a jumper, so the fallback positions and the rollback are CUGR-only
//!   paths, reached here through a router that says no;
//! - no segment got two positions, so `addJumperOnSegments`' skip and early exit are constructed.

use std::collections::{BTreeMap, BTreeSet};

use vyges_grt::finalize::Graph3d;
use vyges_grt::repair_antennas::{
    add_jumper_and_vias, add_jumper_on_segments, build_segment_graph, get_jumper_position, get_segments_connected_to_pin, get_violations,
    jumper_insertion, overlaps, AntViolation, FastRouteJumpers, GatePin, JumperGrid, JumperInputs, JumperRouter, JumperSizes, LibcxxIntSet,
    NetViolations, TechLayer, TechLayers,
};
use vyges_grt::{GSegment, Grid, Rect};

const T: i32 = 100;

/// li1, mcon, met1, via, met2, via2, met3, via3, met4, via4, met5 — routing levels 1..6 with a
/// cut layer between each pair, as sky130's stack.
fn tech() -> TechLayers {
    let names = ["li1", "mcon", "met1", "via", "met2", "via2", "met3", "via3", "met4", "via4", "met5"];
    let n = names.len();
    TechLayers(
        names
            .iter()
            .enumerate()
            .map(|(i, name)| TechLayer {
                name: (*name).into(),
                routing_level: if i % 2 == 0 { (i / 2 + 1) as i32 } else { 0 },
                is_routing: i % 2 == 0,
                upper: (i + 1 < n).then_some(i + 1),
                lower: i.checked_sub(1),
            })
            .collect(),
    )
}

fn grid() -> JumperGrid {
    JumperGrid { grid: Grid { tile_size: T, area: Rect::new(0, 0, 20 * T, 5 * T) }, x_grids: 20, y_grids: 5 }
}

fn g3(cap: u16) -> Graph3d {
    let cells = 20 * 5;
    Graph3d {
        x_grid: 20,
        num_layers: 6,
        h_cap: vec![vec![cap; cells]; 6],
        v_cap: vec![vec![cap; cells]; 6],
        h_usage: vec![vec![0; cells]; 6],
        v_usage: vec![vec![0; cells]; 6],
    }
}

/// libc++'s `std::unordered_set<int>` iteration order and bucket count, against the reference's
/// own libc++: 423 insert sequences (duplicates, rehash points, ascending and descending runs).
#[test]
fn libcxx_unordered_set_order_matches_the_probe() {
    let text = include_str!("data/libcxx-unordered-set-int.txt");
    let mut n = 0;
    for line in text.lines() {
        let (inserts, rest) = line.split_once(" ->").expect("`<inserts> -> <order> |bc=<n>`");
        let (order, bc) = rest.rsplit_once("|bc=").expect("bucket count");
        let mut s = LibcxxIntSet::default();
        for v in inserts.split_whitespace() {
            s.insert(v.parse().unwrap());
        }
        let want: Vec<usize> = order.split_whitespace().map(|v| v.parse().unwrap()).collect();
        assert_eq!(s.iter(), want, "inserts {inserts}");
        assert_eq!(s.bucket_count(), bc.trim().parse::<usize>().unwrap(), "inserts {inserts}");
        n += 1;
    }
    assert_eq!(n, 423);
}

/// `getJumperPosition`'s four branches — and the TIE, which goes to the far side (`target + tile`)
/// because the "nearer the far end" test is strict.
#[test]
fn jumper_position_branches() {
    let s = JumperSizes::new(T);
    assert_eq!(get_jumper_position(1000, 2000, 900, &s), 1100); // target before the window
    assert_eq!(get_jumper_position(1000, 2000, 2000, &s), 2000 - T - 2 * T); // at/after its end
    assert_eq!(get_jumper_position(1000, 2000, 1800, &s), 1800 - T - 2 * T); // nearer the far end
    assert_eq!(get_jumper_position(1000, 2000, 1200, &s), 1200 + T); // nearer the near end
    assert_eq!(get_jumper_position(1000, 2000, 1500, &s), 1500 + T); // equidistant: far side
}

/// The jumper's five segments, in the order they are appended — which is the order its guides are
/// written: start via, end via on `layer`, the same on `layer + 1`, then the flagged jumper.
#[test]
fn jumper_and_vias_append_order() {
    let mut route = Vec::new();
    add_jumper_and_vias(&mut route, (150, 50), (350, 50), 3);
    let got: Vec<(i32, i32, i32, i32, i32, i32, bool)> =
        route.iter().map(|s| (s.init_x, s.init_y, s.init_layer, s.final_x, s.final_y, s.final_layer, s.is_jumper)).collect();
    assert_eq!(
        got,
        vec![
            (150, 50, 3, 150, 50, 4, false),
            (350, 50, 3, 350, 50, 4, false),
            (150, 50, 4, 150, 50, 5, false),
            (350, 50, 4, 350, 50, 5, false),
            (150, 50, 5, 350, 50, 5, true),
        ]
    );
}

/// `getViolations`: only a violation with a routing layer TWO above, at or under the max routing
/// layer, can take a jumper; the highest such violation layer bounds the graph.
#[test]
fn violations_a_jumper_can_reach() {
    let v = |l| AntViolation { routing_level: l, gates: vec![] };
    let (ids, max) = get_violations(&[v(2), v(4), v(5), v(3)], &tech(), 6);
    assert_eq!(ids, vec![0, 1, 3]); // level 5 needs 7: none
    assert_eq!(max, 4);
    let (ids, max) = get_violations(&[v(4)], &tech(), 5);
    assert!(ids.is_empty()); // 6 exists but is above the max routing layer
    assert_eq!(max, -1);
}

/// `Rect::overlaps` is STRICT: boxes that only touch do not connect.
#[test]
fn overlap_is_strict() {
    let a = Rect::new(0, 0, 100, 100);
    assert!(overlaps(&a, &Rect::new(99, 0, 200, 100)));
    assert!(!overlaps(&a, &Rect::new(100, 0, 200, 100)));
    assert!(!overlaps(&a, &Rect::new(0, 100, 100, 200)));
}

/// A via is THREE nodes — the cut layer above its lower layer carrying its route index, then one
/// on each routing layer with `seg_id = -1` — and every node lists itself as a neighbour.
#[test]
fn via_is_three_nodes_and_nodes_neighbour_themselves() {
    let tech = tech();
    let route = vec![GSegment::new(50, 50, 1, 50, 50, 2)];
    let (graph, n) = build_segment_graph(&route, 3, &tech, &grid().grid);
    assert_eq!(n, 3);
    let at = |name: &str| tech.0.iter().position(|l| l.name == name).unwrap();
    let ids = |l: usize| graph[&l].iter().map(|n| (n.node_id, n.seg_id)).collect::<Vec<_>>();
    assert_eq!(ids(at("mcon")), vec![(0, 0)]);
    assert_eq!(ids(at("li1")), vec![(1, -1)]);
    assert_eq!(ids(at("met1")), vec![(2, -1)]);
    // same layer first (itself), then lower, then upper
    assert_eq!(graph[&at("mcon")][0].adjs, vec![(at("mcon"), 0), (at("li1"), 0), (at("met1"), 0)]);
}

/// A segment above the bound is left out, even when its LOWER layer is what decides: a via 3→4
/// is in a graph bounded at 3, a wire on 4 is not.
#[test]
fn graph_takes_segments_by_their_lower_layer() {
    let tech = tech();
    let route = vec![GSegment::new(50, 50, 3, 50, 50, 4), GSegment::new(50, 50, 4, 250, 50, 4)];
    let (_, n) = build_segment_graph(&route, 3, &tech, &grid().grid);
    assert_eq!(n, 3);
}

/// `dbuToTile` clamps to the grid; `getPositionOnGrid` puts a point on the far edge in the LAST
/// cell rather than one past it.
#[test]
fn grid_addressing_at_the_edges() {
    let g = grid();
    assert_eq!(g.dbu_to_tile(-5, true), 0);
    assert_eq!(g.dbu_to_tile(20 * T + 7, true), 19);
    assert_eq!(g.position_on_grid((20 * T, 5 * T)), (19 * T + T / 2, 4 * T + T / 2));
    assert_eq!(g.position_on_grid((130, 260)), (150, 250));
}

/// ⛔ Golden-blind: more than one routing layer reaching the pin makes the seed order a POINTER
/// hash (the outer map is keyed by `dbTechLayer*`) — refused, not guessed.
#[test]
fn a_pin_on_two_layers_is_refused() {
    let tech = tech();
    let route = vec![GSegment::new(50, 50, 1, 50, 50, 2)];
    let (graph, _) = build_segment_graph(&route, 3, &tech, &grid().grid);
    let li1 = 0;
    let met1 = 2;
    let pin = Rect::new(40, 40, 60, 60);
    let gate = GatePin { name: "u/A".into(), inst_rect: Rect::new(0, 0, 100, 100), pin_boxes: vec![(li1, pin), (met1, pin)] };
    assert!(get_segments_connected_to_pin(&gate, &graph).is_err());
    let one = GatePin { pin_boxes: vec![(li1, pin)], ..gate };
    assert_eq!(get_segments_connected_to_pin(&one, &graph).unwrap(), vec![(li1, vec![0])]);
}

/// A net whose pin is on li1 under a via stack to a long met2 wire (level 3), violating on met2:
/// the DFS climbs the stack, the scan finds one window after the via, and the jumper lands one
/// tile past it — the route then holds the jumper's segments, the split-off piece, and the
/// original shortened to start after the jumper.
fn one_net() -> (Vec<NetViolations>, BTreeMap<String, Vec<GSegment>>) {
    let route = vec![
        GSegment::new(50, 50, 1, 50, 50, 2),
        GSegment::new(50, 50, 2, 50, 50, 3),
        GSegment::new(50, 50, 3, 1050, 50, 3),
    ];
    let gate = GatePin { name: "u1/A".into(), inst_rect: Rect::new(0, 0, 100, 100), pin_boxes: vec![(0, Rect::new(40, 40, 60, 60))] };
    let v = vec![NetViolations { net: "n".into(), violations: vec![AntViolation { routing_level: 3, gates: vec![gate] }] }];
    (v, BTreeMap::from([("n".to_string(), route)]))
}

fn tuples(route: &[GSegment]) -> Vec<(i32, i32, i32, i32, i32, i32)> {
    route.iter().map(|s| (s.init_x, s.init_y, s.init_layer, s.final_x, s.final_y, s.final_layer)).collect()
}

#[test]
fn jumper_splits_the_segment_and_moves_usage_up_two_layers() {
    let tech = tech();
    let (v, mut routes) = one_net();
    let mut g = g3(10);
    let lec = BTreeMap::new();
    let inp = JumperInputs { tech: &tech, grid: grid(), max_routing_layer: 6 };
    let (mut trees, ids) = (Vec::new(), BTreeMap::new());
    let mut router = FastRouteJumpers { g3: &mut g, grid: grid(), layer_edge_cost: &lec, trees: &mut trees, ids: &ids };
    let res = jumper_insertion(&v, &mut routes, &inp, &mut router, None).unwrap();
    assert_eq!((res.total_jumpers, res.net_with_jumpers, res.modified_nets.clone()), (1, 1, vec!["n".to_string()]));
    assert_eq!(
        tuples(&routes["n"]),
        vec![
            (50, 50, 1, 50, 50, 2),
            (50, 50, 2, 50, 50, 3),
            (350, 50, 3, 1050, 50, 3), // the original, now starting after the jumper
            (150, 50, 3, 150, 50, 4),
            (350, 50, 3, 350, 50, 4),
            (150, 50, 4, 150, 50, 5),
            (350, 50, 4, 350, 50, 5),
            (150, 50, 5, 350, 50, 5), // the jumper
            (50, 50, 3, 150, 50, 3),  // the piece before it, appended LAST
        ]
    );
    // Usage moves over tiles 1..3 (exclusive): off layer 3 — a uint16 WRAP from 0 — onto layer 5.
    let row = |l: usize| g.h_usage[l][..4].to_vec();
    assert_eq!(row(2), vec![0, u16::MAX, u16::MAX, 0]);
    assert_eq!(row(4), vec![0, 1, 1, 0]);
}

/// The scan reads headroom two layers up: with none there, no window survives and nothing changes.
#[test]
fn no_headroom_no_jumper() {
    let tech = tech();
    let (v, mut routes) = one_net();
    let before = routes.clone();
    let mut g = g3(10);
    g.h_cap[4] = vec![0; 100];
    let lec = BTreeMap::new();
    let inp = JumperInputs { tech: &tech, grid: grid(), max_routing_layer: 6 };
    let (mut trees, ids) = (Vec::new(), BTreeMap::new());
    let mut router = FastRouteJumpers { g3: &mut g, grid: grid(), layer_edge_cost: &lec, trees: &mut trees, ids: &ids };
    let res = jumper_insertion(&v, &mut routes, &inp, &mut router, None).unwrap();
    assert_eq!(res.total_jumpers, 0);
    assert_eq!(routes, before);
}

/// A router that refuses some jumpers — CUGR's shape; FastRoute never refuses.
struct Refusing {
    no_fit: BTreeSet<i32>,
    reject_update: bool,
    restored: usize,
}

impl JumperRouter for Refusing {
    fn has_available_resources(&mut self, _: bool, _: i32, _: i32, _: i32, _: &str) -> bool {
        true
    }
    fn has_jumper_resources(&mut self, init: (i32, i32), _: (i32, i32), _: i32, _: &str) -> bool {
        !self.no_fit.contains(&init.0)
    }
    fn update_jumpered_route(&mut self, _: (i32, i32), _: (i32, i32), _: i32, _: i32, _: &str) -> bool {
        !self.reject_update
    }
    fn restore_net_demand(&mut self, _: &str) {
        self.restored += 1;
    }
    fn headroom(&self, _: bool, _: i32, _: i32, _: i32, _: &str) -> (i32, i32, i32) {
        (1, 0, 1)
    }
}

/// ⛔ Golden-blind (CUGR only): every parent-nearest candidate rejected → the windows' tile-aligned
/// positions, the candidates already tried left out, sorted by the same distance.
#[test]
fn rejected_candidate_falls_back_to_the_window() {
    let tech = tech();
    let (v, mut routes) = one_net();
    let inp = JumperInputs { tech: &tech, grid: grid(), max_routing_layer: 6 };
    let mut router = Refusing { no_fit: BTreeSet::from([150]), reject_update: false, restored: 0 };
    jumper_insertion(&v, &mut routes, &inp, &mut router, None).unwrap();
    // window 150..=750: 150 was the candidate; 250 is next nearest the parent at 50.
    assert_eq!(tuples(&routes["n"])[7], (250, 50, 5, 450, 50, 5));
}

/// ⛔ Golden-blind (CUGR only): a jumper the router will not adopt is undone — the route back to
/// its length and the segment to its start — and the net's demand restored.
#[test]
fn rejected_update_rolls_the_route_back() {
    let tech = tech();
    let (v, mut routes) = one_net();
    let before = routes.clone();
    let inp = JumperInputs { tech: &tech, grid: grid(), max_routing_layer: 6 };
    let mut router = Refusing { no_fit: BTreeSet::new(), reject_update: true, restored: 0 };
    let res = jumper_insertion(&v, &mut routes, &inp, &mut router, None).unwrap();
    assert_eq!(res.total_jumpers, 0);
    assert!(res.modified_nets.is_empty());
    assert_eq!(routes, before);
    assert_eq!(router.restored, 1);
}

/// ⛔ Golden-blind: two positions on one segment. The second is SKIPPED when it is within one
/// jumper length of the last one inserted; and once the split has shrunk the segment below five
/// tiles the loop BREAKS, dropping every later position.
#[test]
fn positions_on_one_segment_skip_and_stop() {
    let s = JumperSizes::new(T);
    let mut router = Refusing { no_fit: BTreeSet::new(), reject_update: false, restored: 0 };
    // Horizontal wire 50..2050 on layer 3 at route index 0.
    let fresh = || vec![GSegment::new(50, 50, 3, 2050, 50, 3)];

    // 150 then 300: 300 is within 2 tiles of 150 → skipped.
    let mut route = fresh();
    let n = add_jumper_on_segments(&BTreeMap::from([(0, BTreeSet::from([150, 300]))]), &mut route, "n", &s, &mut router, None);
    assert_eq!(n, 1);
    // ⛔ Exactly one jumper length (200) apart is still "within": skipped.
    let mut route = fresh();
    let n = add_jumper_on_segments(&BTreeMap::from([(0, BTreeSet::from([150, 350]))]), &mut route, "n", &s, &mut router, None);
    assert_eq!(n, 1);

    // 150 then 1650: after the first jumper the segment is 350..2050 (1700 ≥ 500) → both land;
    // after the second it is 1850..2050.
    let mut route = fresh();
    let n = add_jumper_on_segments(&BTreeMap::from([(0, BTreeSet::from([150, 1650]))]), &mut route, "n", &s, &mut router, None);
    assert_eq!(n, 2);
    assert_eq!((route[0].init_x, route[0].final_x), (1850, 2050));

    // 1450, then 1700 (over 2 tiles on): after the first the segment is 1650..2050 = 400 < 500 →
    // BREAK before the second is even tried.
    let mut route = fresh();
    let n = add_jumper_on_segments(&BTreeMap::from([(0, BTreeSet::from([1450, 1700]))]), &mut route, "n", &s, &mut router, None);
    assert_eq!(n, 1);
}

// ---- stage 4a: diode placement ----
//
// End to end, `grt-diode-score.py` replays the reference's trace on the three diode scripts: the
// fixed set, every try, and all 16 diodes, exact, on our own violations. Every diode there landed
// legally within a few tries; the rules below are the ones it never reaches.

use vyges_grt::repair_antennas::{check_diode_loc, place_diode, row_orient, DiodeFloor, DiodeGate, DiodeRow};

fn floor(rows: Vec<DiodeRow>) -> DiodeFloor {
    DiodeFloor { rows, core: Rect::new(0, 0, 10_000, 10_000), site_width: 100, pad_left: 2, pad_right: 2, diode_width: 200, diode_height: 1000 }
}

fn row(y: i32, orient: &str) -> DiodeRow {
    DiodeRow { bbox: Rect::new(0, y, 10_000, y + 1000), orient: orient.into() }
}

fn gate_at(x: i32, y: i32) -> DiodeGate {
    DiodeGate { rect: Rect::new(x, y, x + 500, y + 1000), orient: "MX".into(), block_or_pad: false, is_block: false }
}

/// Beside the gate, alternating: left (flush), right (flush), left one site further, …, each
/// taking the GATE's orientation, until the padded box touches no fixed cell.
#[test]
fn diode_tries_left_then_right_moving_out_a_site_each_time() {
    let f = floor(vec![row(1000, "R0")]);
    let fixed = [Rect::new(0, 1000, 4800, 2000), Rect::new(5500, 1000, 5900, 2000)];
    let p = place_diode(&gate_at(5000, 1000), false, &f, &fixed);
    let xs: Vec<i32> = p.tries.iter().map(|t| t.0).collect();
    assert_eq!(&xs[..4], &[4800, 5500, 4700, 5600]);
    assert!(p.tries.iter().all(|t| t.2 == "MX"));
    assert_eq!((p.x, p.legal, p.status), (*xs.last().unwrap(), true, "FIRM"));
}

/// ⛔ A vertical violation layer tries ON the gate's row TWICE — the downward and upward
/// sequences both start at offset 0 — then one row below, one above, … — and takes the
/// orientation of the row under the diode's centre, not the gate's.
#[test]
fn vertical_diode_starts_on_the_gate_and_takes_the_row_orientation() {
    let f = floor(vec![row(0, "R0"), row(1000, "MX"), row(2000, "R0")]);
    let p = place_diode(&gate_at(5000, 1000), true, &f, &[Rect::new(4000, 1000, 6000, 2000)]);
    let ys: Vec<(i32, String)> = p.tries.iter().map(|t| (t.1, t.2.clone())).collect();
    assert_eq!(&ys[..3], &[(1000, "MX".to_string()), (1000, "MX".to_string()), (0, "R0".to_string())]);
}

/// ⛔ Golden-blind: no clear spot in 50 tries — the diode stays at the last try, marked PLACED
/// (not FIRM) so the detailed placer may move it.
#[test]
fn a_diode_with_nowhere_to_go_is_left_placed_after_fifty_tries() {
    let f = floor(vec![row(1000, "R0")]);
    let p = place_diode(&gate_at(5000, 1000), false, &f, &[Rect::new(0, 0, 10_000, 10_000)]);
    assert_eq!((p.tries.len(), p.legal, p.status), (50, false, "PLACED"));
}

/// The padded query is widened by BOTH paddings on each side and shrunk by one unit, and it is
/// INCLUSIVE — a fixed cell exactly `pad` sites away still conflicts; one unit further does not.
#[test]
fn padded_query_is_inclusive_at_its_edge() {
    let f = floor(vec![row(1000, "R0")]);
    let diode = Rect::new(5000, 1000, 5200, 2000);
    // (2 + 2) sites of 100 → 400 either side, less 1: the query reaches x = 4601 .. 5599.
    assert!(!check_diode_loc(&diode, &f, &[Rect::new(4000, 1000, 4601, 2000)]));
    assert!(check_diode_loc(&diode, &f, &[Rect::new(4000, 1000, 4600, 2000)]));
    assert_eq!(row_orient(&f.rows, (5100, 1500)), "R0");
    assert_eq!(row_orient(&f.rows, (5100, 1000)), "R0"); // on the row's edge: no row strictly contains it → default R0
    // … and that is visible only on a row that is NOT R0: its edge still reads the default.
    assert_eq!(row_orient(&[row(1000, "MX")], (5100, 1000)), "R0");
    assert_eq!(row_orient(&[row(1000, "MX")], (5100, 1500)), "MX");
}

// ---- stage 3 corners the corpus never reaches (each found by a surviving mutation) ----

use vyges_grt::repair_antennas::select_jumper_position;

/// The pin under a via stack at (50, 50) to a wire on met2 (level 3), violating there.
fn net_with(wires: &[GSegment]) -> (Vec<NetViolations>, BTreeMap<String, Vec<GSegment>>) {
    let mut route = vec![GSegment::new(50, 50, 1, 50, 50, 2), GSegment::new(50, 50, 2, 50, 50, 3)];
    route.extend_from_slice(wires);
    let gate = GatePin { name: "u1/A".into(), inst_rect: Rect::new(0, 0, 100, 100), pin_boxes: vec![(0, Rect::new(40, 40, 60, 60))] };
    let v = vec![NetViolations { net: "n".into(), violations: vec![AntViolation { routing_level: 3, gates: vec![gate] }] }];
    (v, BTreeMap::from([("n".to_string(), route)]))
}

fn jumpers_on(wires: &[GSegment], blocked_tile: Option<usize>) -> (usize, Vec<GSegment>) {
    let tech = tech();
    let (v, mut routes) = net_with(wires);
    let mut g = g3(10);
    if let Some(x) = blocked_tile {
        g.h_cap[4][x] = 0; // level 5 — two up from the violation layer — on row 0
    }
    let lec = BTreeMap::new();
    let inp = JumperInputs { tech: &tech, grid: grid(), max_routing_layer: 6 };
    let (mut trees, ids) = (Vec::new(), BTreeMap::new());
    let mut router = FastRouteJumpers { g3: &mut g, grid: grid(), layer_edge_cost: &lec, trees: &mut trees, ids: &ids };
    let res = jumper_insertion(&v, &mut routes, &inp, &mut router, None).unwrap();
    (res.total_jumpers, routes.remove("n").unwrap())
}

/// `findPosToJumper` skips a wire SHORTER than five tiles; one of exactly five is a candidate.
#[test]
fn a_wire_of_exactly_the_minimum_length_takes_a_jumper() {
    assert_eq!(jumpers_on(&[GSegment::new(50, 50, 3, 550, 50, 3)], None).0, 1);
}

/// A free window must hold the jumper (two tiles) plus ONE TILE OF CLEARANCE AT EACH END — four
/// tiles. A headroom block at tile 3 leaves two windows of three: no jumper at all. Unblocked, the
/// same wire takes one.
#[test]
fn a_window_needs_a_tile_of_clearance_at_both_ends() {
    let wire = [GSegment::new(50, 50, 3, 650, 50, 3)];
    assert_eq!(jumpers_on(&wire, None).0, 1);
    assert_eq!(jumpers_on(&wire, Some(3)).0, 0);
}

/// ⛔ The parent position is SNAPPED to its cell centre before it steers the jumper. A three-tile
/// met2 wire (too short for a jumper) passes its MIDPOINT (200, 50) — on a cell boundary — to the
/// overlapping wire beyond it; snapped to (250, 50), the jumper goes one tile past it, at 350 —
/// not at 300.
#[test]
fn the_parent_position_is_snapped_to_the_grid() {
    let (n, route) = jumpers_on(&[GSegment::new(50, 50, 3, 350, 50, 3), GSegment::new(150, 50, 3, 1150, 50, 3)], None);
    assert_eq!(n, 1);
    let jumper = route.iter().find(|s| s.init_layer == 5 && s.final_layer == 5).expect("a jumper wire");
    assert_eq!((jumper.init_x, jumper.final_x), (350, 550));
}

/// ⛔ Candidates are ordered by `|pos + tile − target|` — the distance from the jumper's first TILE
/// past its start, not from its start: target 300, candidates 150 (50 by that measure, 150 by the
/// plain one) and 400 (200, 100) — 150 wins.
#[test]
fn candidates_are_ordered_by_the_distance_one_tile_in() {
    let s = JumperSizes::new(T);
    let mut router = Refusing { no_fit: BTreeSet::new(), reject_update: false, restored: 0 };
    let pos = select_jumper_position(&mut [150, 400], &[], true, (300, 50), (50, 50), 3, "n", &s, &mut router);
    assert_eq!(pos, 150);
}

/// `updateJumperedRoute` hands `updateRouteGridsLayer` the span in TILES and both layers 0-BASED:
/// a jumper from level 3 to level 5 moves the tree's grid points on layer 2 to layer 4.
#[test]
fn a_jumper_relayers_the_nets_tree_zero_based() {
    use vyges_grt::brk_rsmt::NetState;
    use vyges_grt::full3d::{Point3D, RouteType};
    use vyges_grt::maze3d::{Edge3D, Tree3D};
    let grids: Vec<Point3D> = (0..5).map(|x| Point3D { x, y: 0, layer: 2 }).collect();
    let e = Edge3D { n1: 0, n2: 1, n1a: 0, n2a: 1, len: 4, route_type: RouteType::MazeRoute, routelen: 4, grids };
    let mut trees = vec![NetState { tree3d: Some(Tree3D { num_terminals: 2, num_layers: 6, pin_layers: vec![0, 0], nodes: Vec::new(), edges: vec![e] }), ..NetState::default() }];
    let ids = BTreeMap::from([("n".to_string(), 0usize)]);
    let (mut g, lec) = (g3(10), BTreeMap::new());
    let mut router = FastRouteJumpers { g3: &mut g, grid: grid(), layer_edge_cost: &lec, trees: &mut trees, ids: &ids };
    assert!(router.update_jumpered_route((150, 50), (350, 50), 3, 5, "n"));
    let got: Vec<(i16, i16)> = trees[0].tree3d.as_ref().unwrap().edges[0].grids.iter().map(|p| (p.x, p.layer)).collect();
    assert_eq!(got, vec![(0, 2), (1, 2), (1, 4), (2, 4), (3, 4), (3, 2), (4, 2)]);
}
