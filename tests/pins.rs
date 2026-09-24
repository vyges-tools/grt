// SPDX-License-Identifier: Apache-2.0
//! Ie — I13: `findNets`' filter, and every pin `updateNetPins` builds (terminal boxes → pin →
//! on-grid position and connection layer).
//!
//! Golden `pins.json`: per run the findNets contexts (grid, tracks, the candidate nets) and nets
//! chosen for their features plus a sample of plain ones; for complete runs, the log's
//! GRT-0280/0281/0034/0035/0036 lines, reproduced in order. The exhaustive replay runs every net.

use std::collections::BTreeMap;

use serde_json::Value;
use vyges_grt::{
    find_nets, find_pin, is_pin_reachable, make_bterm_pin, make_iterm_pin, Direction, MasterClass, NetCandidate, NetPin,
    PinEdge, PinGrid, Rect, TermBox,
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

fn rect(v: &Value) -> Rect {
    Rect::new(int(&v[0]), int(&v[1]), int(&v[2]), int(&v[3]))
}

fn grid(ctx: &Value) -> PinGrid {
    let mut directions = BTreeMap::new();
    let mut tracks = BTreeMap::new();
    for l in arr(&ctx["layers"]) {
        let d = match l[1].as_str().expect("dir") {
            "H" => Some(Direction::Horizontal),
            "V" => Some(Direction::Vertical),
            _ => None,
        };
        directions.insert(int(&l[0]), d);
        tracks.insert(int(&l[0]), (int(&l[2]), int(&l[3])));
    }
    PinGrid {
        die: rect(&ctx["die"]),
        tile_size: int(&ctx["tile"]),
        x_grids: int(&ctx["xg"]),
        y_grids: int(&ctx["yg"]),
        directions,
        tracks,
        use_cugr: int(&ctx["cugr"]) == 1,
    }
}

fn boxes(t: &Value) -> Vec<TermBox> {
    arr(&t["boxes"])
        .iter()
        .map(|b| TermBox {
            pin: int(&b[0]),
            level: int(&b[1]),
            routing: int(&b[2]) == 1,
            rect: Rect::new(int(&b[3]), int(&b[4]), int(&b[5]), int(&b[6])),
        })
        .collect()
}

fn edge_ordinal(e: PinEdge) -> i32 {
    match e {
        PinEdge::North => 0,
        PinEdge::South => 1,
        PinEdge::East => 2,
        PinEdge::West => 3,
        PinEdge::None => 4,
    }
}

#[derive(Default)]
struct Seen {
    nets: usize,
    pins: usize,
    aps: usize,
    reach: usize,
    log_runs: usize,
}

/// One net: build its pins, then place each; compare with the reference's final pins. Returns
/// false where the engine stopped with an error — which must be where the reference did (a net with
/// no final pins recorded).
fn replay_net(n: &Value, ctx: &Value, seen: &mut Seen, log: &mut Vec<String>, who: &str) -> bool {
    let aborted = arr(&n["pins"]).is_empty() && !arr(&n["terms"]).is_empty();
    let g = grid(ctx);
    let verbose = int(&ctx["verbose"]) == 1;
    let mut pins: Vec<NetPin> = Vec::new();
    for t in arr(&n["terms"]) {
        let name = t["name"].as_str().expect("name");
        if t["kind"].as_str() == Some("iterm") {
            let class = match t["class"].as_str().expect("class") {
                "pad" => MasterClass::Pad,
                "block" => MasterClass::Block,
                "cover" => MasterClass::Cover,
                _ => MasterClass::Core,
            };
            match make_iterm_pin(name, class, int(&t["core"]) == 1, int(&t["placed"]) == 1, rect(&t["inst_bbox"]), &boxes(t),
                                 g.die, int(&n["max_layer"]), &g.directions, verbose, log) {
                Ok(p) => pins.push(p),
                Err(e) => {
                    assert!(aborted, "{who}: {name}: the engine stopped ({e:?}), the reference did not");
                    return false;
                }
            }
        } else {
            match make_bterm_pin(name, int(&t["placed"]) == 1, &boxes(t), g.die, &g.directions, int(&ctx["check"]) == 1, verbose, log) {
                Ok(Some(p)) => pins.push(p),
                Ok(None) => {}
                Err(e) => {
                    assert!(aborted, "{who}: {name}: the engine stopped ({e:?}), the reference did not");
                    return false;
                }
            }
        }
    }
    assert!(!aborted, "{who}: net {}: the reference stopped here, the engine did not", n["name"]);
    let found = arr(&n["found"]);
    assert_eq!(pins.len(), found.len(), "{who}: net {} pin count", n["name"]);
    for (pin, f) in pins.iter_mut().zip(found) {
        let aps: Vec<(i32, i32, i32)> = arr(&f["aps"]).iter().map(|a| (int(&a[0]), int(&a[1]), int(&a[2]))).collect();
        seen.aps += aps.len();
        let mut reach = arr(&f["reach"]).iter();
        let mut reachable = |p: &NetPin, pos: (i32, i32)| {
            if matches!(p.edge, PinEdge::East | PinEdge::North) {
                return is_pin_reachable(&g, p, pos, &|_, _, _, _, _| unreachable!("east/north never reads a capacity"));
            }
            let r = reach.next().unwrap_or_else(|| panic!("{who}: {}: a reachability check the reference did not make", p.name));
            assert_eq!((int(&r[0]), int(&r[1]), int(&r[2])), (p.connection_layer, pos.0, pos.1), "{who}: {}: reachability query", p.name);
            let got = is_pin_reachable(&g, p, pos, &|_, _, _, _, _| int(&r[3]));
            assert_eq!(got, int(&r[4]) == 1, "{who}: {}: reachable", p.name);
            seen.reach += 1;
            got
        };
        find_pin(&g, pin, &aps, &mut reachable);
        assert!(reach.next().is_none(), "{who}: {}: the reference made more reachability checks", pin.name);
    }
    let want = arr(&n["pins"]);
    assert_eq!(pins.len(), want.len(), "{who}: net {} final pin count", n["name"]);
    for (p, w) in pins.iter().zip(want) {
        let wl: Vec<i32> = arr(&w["layers"]).iter().map(int).collect();
        assert_eq!(
            (p.name.as_str(), p.is_port, p.position, &p.layers, edge_ordinal(p.edge), p.connection_layer, p.on_grid, p.connected_to_pad_or_macro),
            (
                w["name"].as_str().expect("name"),
                int(&w["port"]) == 1,
                (int(&w["pos"][0]), int(&w["pos"][1])),
                &wl,
                int(&w["edge"]),
                int(&w["conn"]),
                (int(&w["ongrid"][0]), int(&w["ongrid"][1])),
                int(&w["connected"]) == 1
            ),
            "{who}: net {} pin {}",
            n["name"],
            p.name
        );
        seen.pins += 1;
    }
    seen.nets += 1;
    true
}

fn replay(g: &Value, full_runs_only_log: bool) -> Seen {
    let mut seen = Seen::default();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        let contexts = arr(&r["contexts"]);
        let complete = r.get("complete").map_or(true, |c| c.as_bool().expect("flag"));
        let mut log = Vec::new();
        // Nets per context, in order.
        let mut by_ctx: BTreeMap<i32, Vec<&Value>> = BTreeMap::new();
        for n in arr(&r["nets"]) {
            by_ctx.entry(int(&n["ctx"])).or_default().push(n);
        }
        for (ci, ctx) in contexts.iter().enumerate() {
            let cands: Vec<NetCandidate> = arr(&ctx["candidates"])
                .iter()
                .map(|c| NetCandidate {
                    name: c[0].as_str().expect("name").into(),
                    is_supply: int(&c[1]) == 1,
                    is_special: int(&c[2]) == 1,
                    term_count: int(&c[3]),
                    has_special_wires: int(&c[4]) == 1,
                    connected_by_abutment: int(&c[5]) == 1,
                })
                .collect();
            let nets = by_ctx.get(&(ci as i32)).cloned().unwrap_or_default();
            // findNets interleaves: a candidate's GRT-280/281, then (if added) its pins' warnings.
            let mut next = nets.iter().peekable();
            for (i, c) in cands.iter().enumerate() {
                let mut one = Vec::new();
                let added = find_nets(std::slice::from_ref(c), int(&ctx["skip"]), &mut one);
                log.extend(one);
                if added.is_empty() {
                    continue;
                }
                if complete {
                    let n = next.next().unwrap_or_else(|| panic!("{who}: candidate {i} {} added, but the reference built no pins", c.name));
                    assert_eq!(n["name"].as_str(), Some(c.name.as_str()), "{who}: added nets, in order");
                    if !replay_net(n, ctx, &mut seen, &mut log, who) {
                        break;
                    }
                } else if next.peek().is_some_and(|n| n["name"].as_str() == Some(c.name.as_str())) {
                    if !replay_net(next.next().expect("peeked"), ctx, &mut seen, &mut log, who) {
                        break;
                    }
                }
            }
            // updateNetPins from later callers (incremental): no findNets around them.
            for n in next {
                if !replay_net(n, ctx, &mut seen, &mut log, who) {
                    break;
                }
            }
        }
        if complete && !r["quiet"].as_bool().expect("flag") && (!full_runs_only_log || complete) {
            let want: Vec<&str> = arr(&r["log"]).iter().map(|l| l.as_str().expect("line")).collect();
            assert_eq!(log, want, "{who}: GRT-0280/0281/0034/0035/0036 lines");
            seen.log_runs += 1;
        }
    }
    seen
}

#[test]
fn the_nets_and_pins_match_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/pins.json", env!("CARGO_MANIFEST_DIR"))), true);
    assert!(s.nets >= 900 && s.pins >= 2000, "nets {}, pins {}", s.nets, s.pins);
    assert!(s.aps > 0 && s.reach > 0 && s.log_runs >= 40, "access points {}, reachability {}, log runs {}", s.aps, s.reach, s.log_runs);
}

/// GRT_PINS_FULL=/path/to/ie-all.json cargo test --release --test pins -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_nets_and_pins_match_the_reference_exhaustively() {
    let path = std::env::var("GRT_PINS_FULL").expect("set GRT_PINS_FULL");
    let s = replay(&read(&path), false);
    eprintln!("exhaustive: {} nets, {} pins, {} access points, {} reachability checks, {} runs' logs", s.nets, s.pins, s.aps, s.reach, s.log_runs);
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

use vyges_grt::{check_pin_placement, determine_edge};

fn port(name: &str, layer: i32, pos: (i32, i32)) -> NetPin {
    NetPin {
        name: name.into(),
        is_port: true,
        position: pos,
        layers: vec![layer],
        boxes: BTreeMap::from([(layer, vec![Rect::new(pos.0, pos.1, pos.0 + 10, pos.1 + 10)])]),
        edge: PinEdge::None,
        connection_layer: layer,
        connected_to_pad_or_macro: false,
        is_core: false,
        on_grid: (0, 0),
    }
}

/// ⛔ I13b: two ports at one RAW position on one layer warn GRT-31 (once per earlier port there)
/// and fail with GRT-80; the same position on another layer is fine. No corpus run has either.
#[test]
fn ports_sharing_a_position_on_a_layer_are_invalid() {
    let (a, b, c, d) = (port("a", 2, (1000, 2000)), port("b", 3, (1000, 2000)), port("c", 2, (1000, 2000)), port("d", 2, (1000, 2000)));
    let mut log = Vec::new();
    assert!(check_pin_placement(&[&a, &b], &|l| format!("m{l}"), 1000, &mut log).is_ok());
    assert!(log.is_empty());
    assert!(check_pin_placement(&[&a, &b, &c, &d], &|l| format!("m{l}"), 1000, &mut log).is_err());
    assert_eq!(log.len(), 3, "c meets a; d meets a and c");
    assert_eq!(log[0], "[WARNING GRT-0031] At least 2 pins in position (1.00um, 2.00um), layer m2, port c.");
}

/// ⛔ The pin position is the LAST routing box's lower-left — across pins too: `last_layer` is never
/// assigned, so "the highest layer" test always passes. Every corpus terminal has ONE pin with boxes.
#[test]
fn the_pin_position_is_the_last_routing_box() {
    let b = |pin, level, x| TermBox { pin, level, routing: true, rect: Rect::new(x, 0, x + 10, 10) };
    let boxes = [b(0, 3, 100), b(0, 1, 200), b(1, 2, 300), TermBox { pin: 1, level: 0, routing: false, rect: Rect::new(900, 0, 910, 10) }];
    let p = make_iterm_pin("u/A", MasterClass::Core, true, true, Rect::new(0, 0, 1000, 1000), &boxes, Rect::new(0, 0, 1000, 1000), 3,
                           &BTreeMap::new(), false, &mut Vec::new()).expect("pin");
    assert_eq!(p.position, (300, 0), "the last ROUTING box, not the top layer's");
    assert_eq!((p.layers.clone(), p.connection_layer), (vec![1, 2, 3], 3));
}

/// ⛔ A block terminal outside the die is clipped only when VERBOSE — quiet, it keeps its box.
#[test]
fn a_bterm_outside_the_die_is_clipped_only_when_verbose() {
    let die = Rect::new(0, 0, 1000, 1000);
    let boxes = [TermBox { pin: 0, level: 2, routing: true, rect: Rect::new(900, 500, 1100, 600) }];
    let quiet = make_bterm_pin("p", true, &boxes, die, &BTreeMap::new(), true, false, &mut Vec::new()).expect("ok").expect("pin");
    let loud = make_bterm_pin("p", true, &boxes, die, &BTreeMap::new(), true, true, &mut Vec::new()).expect("ok").expect("pin");
    assert_eq!(quiet.boxes[&2], vec![Rect::new(900, 500, 1100, 600)]);
    assert_eq!(loud.boxes[&2], vec![Rect::new(900, 500, 1000, 600)]);
}

/// ⛔ Edge voting: ties go north, south, east, west; the connection layer is the HIGHEST layer
/// running across the edge (vertical for north/south), else the top layer.
#[test]
fn edge_votes_break_ties_north_first_and_pick_the_highest_crossing_layer() {
    let bounds = Rect::new(0, 0, 100, 100);
    let dirs = BTreeMap::from([(1, Some(Direction::Horizontal)), (2, Some(Direction::Vertical)), (3, Some(Direction::Horizontal))]);
    // One box touching north, one touching south: a tie → north; vertical layer 2 is the best.
    let boxes = BTreeMap::from([(1, vec![Rect::new(40, 90, 50, 100)]), (3, vec![Rect::new(40, 0, 50, 10)])]);
    assert_eq!(determine_edge(bounds, &boxes, &[1, 2, 3], &dirs), (PinEdge::North, 2));
    // East side: horizontal wanted → the highest horizontal layer, 3.
    let boxes = BTreeMap::from([(1, vec![Rect::new(90, 40, 100, 50)])]);
    assert_eq!(determine_edge(bounds, &boxes, &[1, 2, 3], &dirs), (PinEdge::East, 3));
    // No layer runs the wanted way → the top layer.
    assert_eq!(determine_edge(bounds, &boxes, &[2], &dirs), (PinEdge::East, 2));
}

use vyges_grt::find_on_grid_positions;

fn cand(name: &str, supply: bool, special: bool, terms: i32) -> NetCandidate {
    NetCandidate { name: name.into(), is_supply: supply, is_special: special, term_count: terms, has_special_wires: false, connected_by_abutment: false }
}

/// ⛔ The fanout checks exempt only nets that are supply AND special; the skip is STRICTLY above
/// `skip_large_fanout`. The corpus skips one net, far above its threshold.
#[test]
fn only_supply_and_special_nets_escape_the_fanout_skip() {
    let mut log = Vec::new();
    let added = find_nets(&[cand("s", true, false, 50), cand("at", false, false, 10), cand("pg", true, true, 50)], 10, &mut log);
    assert_eq!(log, ["[INFO GRT-0280] Skipping net s with 50 terminals."], "supply alone is skipped; 10 is not above 10");
    assert_eq!(added, [1], "`at` is added; `pg` passes the skip but addNet rejects supply");
}

/// ⛔ A terminal's LAYERS stop at the max routing layer, its BOXES do not — and the boxes above
/// still vote for a pad/macro pin's edge. No corpus terminal has boxes above the max.
#[test]
fn boxes_above_the_max_layer_stay_but_are_not_layers() {
    let tb = |level, r: Rect| TermBox { pin: 0, level, routing: true, rect: r };
    let die = Rect::new(0, 0, 1000, 1000);
    let dirs = BTreeMap::from([(1, Some(Direction::Horizontal)), (2, Some(Direction::Vertical)), (3, Some(Direction::Horizontal))]);
    // Level 1 box nearest SOUTH; two level-3 boxes nearest NORTH outvote it.
    let boxes = [tb(1, Rect::new(40, 0, 50, 10)), tb(3, Rect::new(10, 990, 20, 1000)), tb(3, Rect::new(60, 990, 70, 1000))];
    let p = make_iterm_pin("m/A", MasterClass::Block, false, true, Rect::new(0, 0, 100, 1000), &boxes, die, 2, &dirs, false, &mut Vec::new())
        .expect("pin");
    assert_eq!(p.layers, vec![1]);
    assert!(p.boxes.contains_key(&3));
    assert_eq!(p.edge, PinEdge::North, "the level-3 boxes vote");
    assert_eq!(p.connection_layer, 1, "no vertical layer among the pin's layers → its top layer");
}

/// ⛔ Ties, per box and in the count: north, then south, then east, then west. The highest layer
/// running across the edge wins. The corpus never ties, and its macro pins offer one layer.
#[test]
fn edge_ties_follow_north_south_east_west() {
    let bounds = Rect::new(0, 0, 100, 100);
    let dirs = BTreeMap::from([(2, Some(Direction::Vertical)), (4, Some(Direction::Vertical)), (3, Some(Direction::Horizontal))]);
    // A box equidistant from all four sides votes NORTH.
    let mid = BTreeMap::from([(2, vec![Rect::new(40, 40, 60, 60)])]);
    assert_eq!(determine_edge(bounds, &mid, &[2, 4], &dirs), (PinEdge::North, 4), "north; the higher vertical layer");
    // One south vote, one east vote: the count ties → SOUTH (before east).
    let se = BTreeMap::from([(3, vec![Rect::new(40, 0, 50, 5), Rect::new(95, 40, 100, 50)])]);
    assert_eq!(determine_edge(bounds, &se, &[3], &dirs).0, PinEdge::South);
}

fn test_grid() -> PinGrid {
    PinGrid {
        die: Rect::new(0, 0, 1000, 1000),
        tile_size: 100,
        x_grids: 10,
        y_grids: 10,
        directions: BTreeMap::from([(1, Some(Direction::Horizontal)), (2, Some(Direction::Vertical)), (3, Some(Direction::Horizontal))]),
        tracks: BTreeMap::from([(1, (5, 10)), (2, (5, 10)), (3, (5, 10))]),
        use_cugr: false,
    }
}

fn core_pin(conn: i32, boxes: Vec<Rect>) -> NetPin {
    NetPin {
        name: "u/A".into(),
        is_port: false,
        position: (boxes[0].x_min, boxes[0].y_min),
        layers: vec![conn],
        boxes: BTreeMap::from([(conn, boxes)]),
        edge: PinEdge::None,
        connection_layer: conn,
        connected_to_pad_or_macro: false,
        is_core: true,
        on_grid: (0, 0),
    }
}

/// ⛔ Access points are taken by LAYER ascending (a `std::map`), whatever order they come in; the
/// vote's first maximum wins, so the lowest layer's first point is chosen.
#[test]
fn access_points_are_ordered_by_layer() {
    let g = test_grid();
    let pin = core_pin(3, vec![Rect::new(0, 0, 10, 10)]);
    let (pos, has, _) = find_on_grid_positions(&g, &pin, &[(3, 750, 750), (1, 250, 250)], &mut |_, _| true);
    assert!(has);
    assert_eq!(pos, vec![(250, 250, 1), (750, 750, 3)]);
}

/// ⛔ The vote's FIRST maximum wins a tie; the single-track correction then applies only WITHOUT
/// access points, and only when the track cell differs ACROSS the layer's direction (y on a
/// horizontal layer). The corpus's committed nets never tie, nor move along the direction.
#[test]
fn the_first_vote_wins_and_the_track_correction_crosses_the_direction() {
    let g = test_grid();
    // Two boxes on horizontal layer 1, cells x 0 and x 3, one track (105) strictly inside y 100..112.
    let mut pin = core_pin(1, vec![Rect::new(0, 100, 10, 112), Rect::new(300, 100, 310, 112)]);
    find_pin(&g, &mut pin, &[], &mut |_, _| true);
    assert_eq!(pin.on_grid, (50, 150), "tie → the first box; the track cell differs only in x → no move");
    // With an access point the correction is skipped.
    let mut pin = core_pin(1, vec![Rect::new(0, 100, 10, 112)]);
    find_pin(&g, &mut pin, &[(1, 50, 250)], &mut |_, _| true);
    assert_eq!(pin.on_grid, (50, 250), "the access point's cell stands");
}

// Upstream rule (GlobalRouter `findOnGridPositions`): a pad or macro pin that cannot reach its
// on-grid position is moved toward its instance's edge — but only under FastRoute. CUGR has no
// FastRoute capacities to ask, so with `use_cugr` the reachability test is NEVER made and the pin
// stays at its box's centre. No pad or macro pin reaches a CUGR route in the suite.
#[test]
fn cugr_never_asks_a_pad_pin_whether_it_is_reachable() {
    let mut directions = BTreeMap::new();
    directions.insert(1, Some(Direction::Horizontal));
    directions.insert(2, Some(Direction::Vertical));
    let mut tracks = BTreeMap::new();
    tracks.insert(1, (100, 200));
    tracks.insert(2, (100, 200));
    let die = Rect::new(0, 0, 10000, 10000);
    let mut log = Vec::new();
    let boxes = [TermBox { pin: 0, level: 2, routing: true, rect: Rect::new(4200, 4200, 4400, 4400) }];
    let pin = vyges_grt::make_iterm_pin("pad/P", vyges_grt::MasterClass::Pad, false, true, Rect::new(4000, 4000, 6000, 6000), &boxes, die, 2, &directions, false, &mut log).unwrap();
    for use_cugr in [false, true] {
        let grid = PinGrid { die, tile_size: 1000, x_grids: 10, y_grids: 10, directions: directions.clone(), tracks: tracks.clone(), use_cugr };
        let mut p = pin.clone();
        let mut asked = false;
        vyges_grt::find_pin(&grid, &mut p, &[], &mut |_, _| {
            asked = true;
            false
        });
        assert_eq!(asked, !use_cugr, "use_cugr = {use_cugr}");
    }
}
