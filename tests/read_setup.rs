// SPDX-License-Identifier: Apache-2.0
//! The database READER against the facts the reference's own setup stages were given.
//!
//! Each design is loaded from the reference test suite's own LEF/DEF into OUR database, read by
//! [`vyges_grt::read`], and every fact compared with what the reference fed the same stage in the
//! same run (the per-stage goldens). A mismatch here is a wrong READ — the rules are gated
//! separately — so this is where a database walk is proven before any rule consumes it.
//!
//! Needs the reference suite's test directory (LEF/DEF): set `GRT_REF_TESTS` to it; skipped
//! otherwise, as CI has no design files.
#![cfg(feature = "odb")]

use serde_json::Value;
use vyges_grt::read::read_tech;
use vyges_opendb::Db;

/// The no-Liberty, non-CUGR guide scripts with no macros, DEF obstructions or routed wires:
/// `(script, LEFs, DEF, set_routing_layers -signal)`.
const TIER_A: &[(&str, &[&str], &str, Option<(&str, &str)>)] = &[
    ("colocated_pins", &["Nangate45/Nangate45.lef", "colocated_pins.lef"], "colocated_pins.def", Some(("metal2", "metal10"))),
    ("congestion1", &["Nangate45/Nangate45.lef"], "gcd.def", Some(("metal2", "metal10"))),
    ("congestion2", &["Nangate45/Nangate45.lef"], "gcd.def", Some(("metal2", "metal10"))),
    ("congestion7", &["Nangate45/Nangate45.lef"], "gcd.def", Some(("metal2", "metal10"))),
    ("gcd_flute", &["Nangate45/Nangate45.lef"], "gcd.def", None),
    ("gcd", &["Nangate45/Nangate45.lef"], "gcd.def", None),
    ("multiple_calls", &["Nangate45/Nangate45.lef"], "multiple_calls.def", None),
    ("pd3", &["Nangate45/Nangate45.lef"], "gcd.def", None),
    ("region_adjustment", &["Nangate45/Nangate45.lef"], "region_adjustment.def", None),
    ("set_nets_to_route1", &["Nangate45/Nangate45.lef"], "gcd.def", Some(("metal2", "metal8"))),
    ("silence", &["Nangate45/Nangate45.lef"], "gcd.def", None),
    ("skip_large_fanout1", &["Nangate45/Nangate45.lef"], "gcd.def", None),
    ("top_level_term1", &["sky130hs/sky130hs.tlef", "sky130hs/sky130hs_std_cell.lef"], "top_level_term1.def", Some(("met1", "met4"))),
    ("top_level_term2", &["sky130hs/sky130hs.tlef", "sky130hs/sky130hs_std_cell.lef"], "top_level_term2.def", Some(("met1", "met4"))),
    ("top_level_term3", &["sky130hs/sky130hs.tlef", "sky130hs/sky130hs_std_cell.lef"], "top_level_term3.def", Some(("met1", "met4"))),
    ("upper_layer_net", &["Nangate45/Nangate45.lef"], "upper_layer_net.def", Some(("metal1", "metal9"))),
];

fn read(path: &str) -> Value {
    serde_json::from_str(&std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"))).expect("golden parses")
}

fn int(v: &Value) -> i32 {
    v.as_i64().expect("integer") as i32
}

fn arr(v: &Value) -> &Vec<Value> {
    v.as_array().expect("array")
}

/// The design as the script loads it: LEFs, DEF, then `set_routing_layers -signal` (the block's
/// min/max routing LEVEL).
fn load(dir: &str, lefs: &[&str], def: &str, signal: Option<(&str, &str)>) -> Db {
    let mut db = Db::new();
    for lef in lefs {
        db.read_lef(format!("{dir}/{lef}")).unwrap_or_else(|e| panic!("{lef}: {e}"));
    }
    db.read_def(format!("{dir}/{def}"), "default").unwrap_or_else(|e| panic!("{def}: {e}"));
    if let Some((lo, hi)) = signal {
        let (lo, hi) = (db.layer_get_routing_level(lo), db.layer_get_routing_level(hi));
        db.block_set_min_routing_layer(lo).expect("min");
        db.block_set_max_routing_layer(hi).expect("max");
    }
    db
}

#[test]
fn the_technology_reads_match_what_the_reference_fed_i6() {
    let Ok(dir) = std::env::var("GRT_REF_TESTS") else {
        eprintln!("GRT_REF_TESTS unset: skipped");
        return;
    };
    let gold = read(&format!("{}/examples/grt_gate/tracks.json", env!("CARGO_MANIFEST_DIR")));
    let runs: std::collections::HashMap<&str, &Value> = arr(&gold["runs"]).iter().map(|r| (r["design"].as_str().expect("design"), r)).collect();
    let (mut designs, mut layers_checked, mut vias_checked) = (0, 0, 0);
    for (script, lefs, def, signal) in TIER_A {
        let db = load(&dir, lefs, def, *signal);
        let facts = read_tech(&db).unwrap_or_else(|e| panic!("{script}: {e}"));
        let run = runs.get(format!("{script}-plain").as_str()).unwrap_or_else(|| panic!("{script}: no tracks golden"));
        // I6's track patterns, per layer the reference reached.
        for t in arr(&run["calls"][0]["tracks"]) {
            let name = t["name"].as_str().expect("name");
            let ours = facts.tracks.iter().find(|l| l.name == name).unwrap_or_else(|| panic!("{script}: layer {name} not read"));
            assert_eq!(ours.index, int(&t["index"]), "{script} {name}: index");
            let grid = ours.grid.as_ref().unwrap_or_else(|| panic!("{script} {name}: no track grid"));
            let pats = |v: &Value| -> Vec<(i32, i32, i32)> { arr(v).iter().map(|p| (int(&p[0]), int(&p[1]), int(&p[2]))).collect() };
            let mine = |v: &[vyges_grt::TrackPattern]| -> Vec<(i32, i32, i32)> { v.iter().map(|p| (p.origin, p.count, p.step)).collect() };
            assert_eq!((mine(&grid.x), mine(&grid.y)), (pats(&t["x"]), pats(&t["y"])), "{script} {name}: track patterns");
            layers_checked += 1;
        }
        // calcLayerPitches' inputs: the pitch case with this technology's layers.
        let case = arr(&gold["pitch_cases"])
            .iter()
            .find(|c| arr(&c["layers"]).iter().zip(&facts.pitch_layers).all(|(l, p)| l["name"] == p.name.as_str()) && arr(&c["layers"]).len() <= facts.pitch_layers.len())
            .unwrap_or_else(|| panic!("{script}: no pitch case for this technology"));
        for v in arr(&case["vias"]) {
            let name = v["name"].as_str().expect("name");
            let ours = facts.vias.iter().find(|x| x.name == name).unwrap_or_else(|| panic!("{script}: via {name} not read"));
            let boxes: Vec<(i32, i32, i32)> = arr(&v["boxes"]).iter().map(|b| (int(&b[0]), int(&b[1]), int(&b[2]))).collect();
            let bottom = (int(&v["bottom"]) >= 0).then(|| int(&v["bottom"]));
            assert_eq!((ours.bottom, ours.or_default, &ours.boxes), (bottom, int(&v["or_default"]) == 1, &boxes), "{script}: via {name}");
            vias_checked += 1;
        }
        assert_eq!(facts.vias.len(), arr(&case["vias"]).len(), "{script}: via count");
        for l in arr(&case["layers"]) {
            let p = facts.pitch_layers.iter().find(|p| p.name == l["name"].as_str().expect("name")).expect("layer");
            let v54: Vec<(u32, Option<(u32, u32)>)> = arr(&l["v54"]).iter().map(|r| (int(&r[0]) as u32, (int(&r[1]) == 1).then(|| (int(&r[2]) as u32, int(&r[3]) as u32)))).collect();
            assert_eq!(
                (p.index, p.is_routing, p.routing_level, p.width, p.has_two_widths, p.has_v55, p.v54.iter().map(|r| (r.spacing, r.range)).collect::<Vec<_>>()),
                (int(&l["index"]), l["routing"].as_bool().expect("routing"), int(&l["level"]), int(&l["width"]), int(&l["tw"]) == 1, int(&l["v55"]) == 1, v54),
                "{script}: pitch layer {}",
                p.name
            );
        }
        designs += 1;
    }
    eprintln!("read: {designs} designs, {layers_checked} track layers, {vias_checked} vias");
    assert_eq!(designs, TIER_A.len());
}

/// Unroll `[[value, count], …]`.
fn unrle(v: &Value) -> Vec<i32> {
    arr(v).iter().flat_map(|p| std::iter::repeat(int(&p[0])).take(int(&p[1]) as usize)).collect()
}

/// G3 and I3–I9 over our database — [`vyges_grt::global_route::setup_tech`] — against what the
/// reference computed in the same run: the min/max routing layers (`driver.json`), each layer's
/// average track spacing (`tracks.json`) and, where captured, every 3D edge capacity
/// (`capacities.json`).
#[test]
fn the_technology_setup_matches_the_reference() {
    let Ok(dir) = std::env::var("GRT_REF_TESTS") else {
        eprintln!("GRT_REF_TESTS unset: skipped");
        return;
    };
    let g = |name: &str| read(&format!("{}/examples/grt_gate/{name}.json", env!("CARGO_MANIFEST_DIR")));
    let (driver, tracks, caps) = (g("driver"), g("tracks"), g("capacities"));
    let by_design = |v: &Value| -> std::collections::HashMap<String, Value> { arr(&v["runs"]).iter().map(|r| (r["design"].as_str().expect("d").to_string(), r.clone())).collect() };
    let (driver, tracks, caps) = (by_design(&driver), by_design(&tracks), by_design(&caps));
    let opts = vyges_grt::global_route::RouteOptions::new();
    let (mut minmax, mut track_layers, mut cap_designs) = (0, 0, 0);
    for (script, lefs, def, signal) in TIER_A {
        let mut db = load(&dir, lefs, def, *signal);
        let s = vyges_grt::global_route::setup_tech(&mut db, &opts).unwrap_or_else(|e| panic!("{script}: {e}"));
        let key = format!("{script}-plain");
        if let Some(d) = driver.get(&key) {
            let out = &d["minmax"][0]["out"];
            assert_eq!((s.min_routing_layer, s.max_routing_layer), (int(&out[0]), int(&out[1])), "{script}: getMinMaxLayer");
            minmax += 1;
        }
        for t in arr(&tracks[&key]["calls"][0]["tracks"]) {
            let ours = s.tracks.iter().find(|r| r.layer_index == int(&t["index"])).unwrap_or_else(|| panic!("{script}: no tracks for {}", t["name"]));
            let a = &t["answer"];
            assert_eq!((ours.track_pitch, ours.location, ours.num_tracks), (int(&a[0]), int(&a[1]), int(&a[2])), "{script} {}: average track spacing", t["name"]);
            track_layers += 1;
        }
        if let Some(c) = caps.get(&key) {
            let set = &c["sets"][0];
            let c3 = &s.capacities;
            assert_eq!((c3.x_grid, c3.y_grid, c3.num_layers), (int(&set["x_grids"]), int(&set["y_grids"]), int(&set["num_layers"])), "{script}: grid");
            for (key3, ours) in [("h3", &c3.h3), ("v3", &c3.v3)] {
                for (ly, row) in set[key3].as_object().expect("rows") {
                    let (l, y) = ly.split_once(',').expect("l,y");
                    let (l, y): (i32, i32) = (l.parse().expect("l"), y.parse().expect("y"));
                    let start = ((l * c3.y_grid + y) * c3.x_grid) as usize;
                    let want = unrle(row);
                    let got: Vec<i32> = ours[start..start + want.len()].iter().map(|&v| i32::from(v)).collect();
                    assert_eq!(got, want, "{script}: {key3} layer {l} row {y}");
                }
            }
            cap_designs += 1;
        }
    }
    eprintln!("setup: {minmax} min/max, {track_layers} track layers, {cap_designs} capacity grids");
    assert!(minmax >= 11 && track_layers >= 140 && cap_designs >= 2);
}

/// A script's setup commands, in its order.
#[derive(Clone, Copy)]
enum Cmd {
    /// `set_routing_layers -signal lo-hi`.
    Layers(&'static str, &'static str),
    /// `set_global_routing_layer_adjustment lo[-hi] adj`.
    LayerAdj(&'static str, &'static str, f32),
    /// `set_global_routing_layer_adjustment * adj`.
    GlobalAdj(f32),
    /// `set_global_routing_region_adjustment {x0 y0 x1 y1} -layer l -adjustment a` (microns).
    Region([f64; 4], &'static str, f32),
}

/// Tier A with each script's setup commands (its first `global_route` only).
fn tier_a_commands(script: &str) -> Vec<Cmd> {
    use Cmd::*;
    match script {
        "colocated_pins" => vec![Layers("metal2", "metal10")],
        "congestion1" | "congestion7" => vec![LayerAdj("metal2", "metal2", 0.9), LayerAdj("metal3", "metal3", 0.9), LayerAdj("metal4", "metal10", 1.0), Layers("metal2", "metal10")],
        "congestion2" => vec![LayerAdj("metal2", "metal2", 0.9), LayerAdj("metal3", "metal3", 0.9), LayerAdj("metal4", "metal6", 0.9), LayerAdj("metal7", "metal10", 1.0), Layers("metal2", "metal10")],
        "region_adjustment" => vec![Region([1.4, 2.0, 20.0, 15.5], "metal2", 0.9)],
        "set_nets_to_route1" => vec![Layers("metal2", "metal8")],
        "top_level_term1" | "top_level_term2" | "top_level_term3" => vec![LayerAdj("met1", "met1", 0.8), LayerAdj("met2", "met2", 0.7), GlobalAdj(0.5), Layers("met1", "met4")],
        "upper_layer_net" => vec![Layers("metal1", "metal9")],
        _ => vec![],
    }
}

/// Load a design and apply its commands as the Tcl does.
fn load_with(dir: &str, lefs: &[&str], def: &str, cmds: &[Cmd]) -> (Db, vyges_grt::global_route::RouteOptions) {
    let mut db = load(dir, lefs, def, None);
    let mut opts = vyges_grt::global_route::RouteOptions::new();
    let mut log = Vec::new();
    for c in cmds {
        match *c {
            Cmd::Layers(lo, hi) => {
                let (lo, hi) = (db.layer_get_routing_level(lo), db.layer_get_routing_level(hi));
                db.block_set_min_routing_layer(lo).expect("min");
                db.block_set_max_routing_layer(hi).expect("max");
            }
            Cmd::LayerAdj(lo, hi, a) => {
                for l in db.layer_get_routing_level(lo)..=db.layer_get_routing_level(hi) {
                    vyges_grt::global_route::add_layer_adjustment(&mut db, l, a, false, &mut log).expect("adjustment");
                }
            }
            Cmd::GlobalAdj(a) => opts.adjustment = a,
            Cmd::Region(r, layer, a) => {
                let dbu = f64::from(db.dbu_per_micron());
                let u = |v: f64| (v * dbu) as i32;
                opts.region_adjustments.push((vyges_grt::Rect { x_min: u(r[0]), y_min: u(r[1]), x_max: u(r[2]), y_max: u(r[3]) }, db.layer_get_routing_level(layer), a));
            }
        }
    }
    (db, opts)
}

/// Unroll `[[a, b, count], …]` rows of an R5 entry's usage dump into per-edge reductions.
fn entry_reductions(rows: &Value) -> Vec<Vec<i32>> {
    arr(rows).iter().map(|row| arr(row).iter().flat_map(|p| std::iter::repeat(int(&p[1])).take(int(&p[2]) as usize)).collect()).collect()
}

/// The whole setup up to the router's input — G3, I3–I10 — against run()'s ENTRY as the reference
/// captured it (R5's first call): every 3D edge capacity per layer, every 2D edge reduction, and the
/// h/v capacities; and the 2D capacity at B7.
#[test]
fn the_setup_reaches_the_routers_entry() {
    let Ok(dir) = std::env::var("GRT_REF_TESTS") else {
        eprintln!("GRT_REF_TESTS unset: skipped");
        return;
    };
    let path = std::env::var("GRT_BRK_RSMT_FULL").unwrap_or_else(|_| format!("{}/examples/grt_gate/brk_rsmt.json", env!("CARGO_MANIFEST_DIR")));
    let r7 = read(&path);
    let runs: std::collections::HashMap<&str, &Value> = arr(&r7["runs"]).iter().map(|r| (r["design"].as_str().expect("d"), r)).collect();
    let (mut designs, mut edges) = (0, 0usize);
    for (script, lefs, def, _) in TIER_A {
        let Some(run) = runs.get(format!("{script}-plain").as_str()) else { continue };
        let c = &arr(&run["calls"])[0];
        if !c["small"].as_bool().expect("small") {
            continue;
        }
        let (mut db, opts) = load_with(&dir, lefs, def, &tier_a_commands(script));
        let t = vyges_grt::global_route::setup_tech(&mut db, &opts).unwrap_or_else(|e| panic!("{script}: {e}"));
        let mut log = Vec::new();
        let e = vyges_grt::global_route::setup_adjust(&mut db, &t, &opts, &mut log).unwrap_or_else(|e| panic!("{script}: {e}"));
        let (xg, yg) = (e.x_grid, e.y_grid);
        assert_eq!((xg, yg), (int(&c["xg"]), int(&c["yg"])), "{script}: grid");
        assert_eq!((t.capacities.h_capacity, t.capacities.v_capacity), (int(&c["hcap"]), int(&c["vcap"])), "{script}: h/v capacity");
        for (d, ours) in [("H", &e.h3), ("V", &e.v3)] {
            for (l, layer) in arr(&c["caps"][d]).iter().enumerate() {
                let want: Vec<i32> = arr(layer).iter().flat_map(unrle).collect();
                let got: Vec<i32> = (0..yg).flat_map(|y| (0..xg).map(move |x| (y, x))).map(|(y, x)| i32::from(ours[((l as i32 * yg + y) * xg + x) as usize].cap)).collect();
                assert_eq!(got.len(), want.len(), "{script}: {d} layer {l} size");
                if let Some(k) = (0..got.len()).find(|&k| got[k] != want[k]) {
                    panic!("{script}: 3D capacity {d} layer {l} at (x {}, y {}): ours {}, reference {}", k as i32 % xg, k as i32 / xg, got[k], want[k]);
                }
                edges += got.len();
            }
        }
        for (d, ours, w) in [("H", &e.h2, xg - 1), ("V", &e.v2, xg)] {
            for (y, row) in entry_reductions(&c["entry"][d]).iter().enumerate() {
                for (x, &red) in row.iter().enumerate() {
                    let i = y * w as usize + x;
                    assert_eq!(i32::from(ours[i].red), red, "{script}: 2D reduction {d} ({x}, {y})");
                }
            }
        }
        designs += 1;
    }
    eprintln!("entry: {designs} designs, {edges} 3D edges");
    assert!(designs >= 10, "{designs} designs reached");
}

/// I13 + I14 over our database — every net the router receives — against run()'s entry as the
/// reference captured it (R5's first call): the same nets in the same order (the FastRoute index),
/// and per net its name, on-grid pins, driver, layer range, edge costs and alpha.
#[test]
fn the_nets_reach_the_router_as_the_reference_built_them() {
    let Ok(dir) = std::env::var("GRT_REF_TESTS") else {
        eprintln!("GRT_REF_TESTS unset: skipped");
        return;
    };
    let path = std::env::var("GRT_BRK_RSMT_FULL").unwrap_or_else(|_| format!("{}/examples/grt_gate/brk_rsmt.json", env!("CARGO_MANIFEST_DIR")));
    let r7 = read(&path);
    let runs: std::collections::HashMap<&str, &Value> = arr(&r7["runs"]).iter().map(|r| (r["design"].as_str().expect("d"), r)).collect();
    let (mut designs, mut nets_checked, mut pins_checked) = (0, 0, 0);
    for (script, lefs, def, _) in TIER_A {
        let Some(run) = runs.get(format!("{script}-plain").as_str()) else { continue };
        let c = &arr(&run["calls"])[0];
        let (mut db, mut opts) = load_with(&dir, lefs, def, &tier_a_commands(script));
        match *script {
            "gcd_flute" => opts.alpha = 0.0,
            "skip_large_fanout1" => opts.skip_large_fanout = 30,
            "set_nets_to_route1" => continue, // its pattern list is resolved by the Tcl front end
            _ => {}
        }
        let t = vyges_grt::global_route::setup_tech(&mut db, &opts).unwrap_or_else(|e| panic!("{script}: {e}"));
        let mut log = Vec::new();
        let e = vyges_grt::global_route::setup_adjust(&mut db, &t, &opts, &mut log).unwrap_or_else(|e| panic!("{script}: {e}"));
        let nets = vyges_grt::global_route::setup_nets(&db, &t, &e, &opts, &mut log).unwrap_or_else(|e| panic!("{script}: {e}"));
        // The capture lists `net_ids_` — the ROUTED nets — by their FastRoute id; a local net has
        // an id but is not routed.
        let want = arr(&c["nets"]);
        let routed: Vec<usize> = (0..nets.len()).filter(|&k| !nets[k].is_local).collect();
        assert_eq!(routed, want.iter().map(|w| int(&w["id"]) as usize).collect::<Vec<_>>(), "{script}: the routed nets' ids ({} router nets)", nets.len());
        for w in want {
            let k = int(&w["id"]) as usize;
            let ours = &nets[k];
            let at = format!("{script}: net {k} {}", w["name"]);
            assert_eq!(ours.name, w["name"].as_str().expect("name"), "{at}: name (the order)");
            let pins: Vec<(i32, i32)> = arr(&w["pins"]).iter().map(|p| (int(&p[0]), int(&p[1]))).collect();
            assert_eq!(ours.pins.iter().map(|p| (p.0, p.1)).collect::<Vec<_>>(), pins, "{at}: on-grid pins");
            assert_eq!((ours.min_layer - 1, ours.max_layer - 1, i32::from(ours.edge_cost), ours.root as i32), (int(&w["min"]), int(&w["max"]), int(&w["cost"]), int(&w["driver"])), "{at}: (min, max, cost, driver)");
            assert_eq!(ours.alpha.to_bits(), (w["alpha"].as_f64().expect("alpha") as f32).to_bits(), "{at}: alpha");
            let lec: Vec<i32> = arr(&w["lec"]).iter().map(int).collect();
            let ours_lec: Vec<i32> = (ours.min_layer - 1..=ours.max_layer - 1).map(|l| ours.layer_edge_cost.as_ref().map_or(1, |v| i32::from(v[l as usize]))).collect();
            assert_eq!(ours_lec, lec, "{at}: per-layer edge costs");
            nets_checked += 1;
            pins_checked += pins.len();
        }
        designs += 1;
    }
    eprintln!("nets: {designs} designs, {nets_checked} router nets, {pins_checked} pins");
    assert!(designs >= 12, "{designs} designs");
}

thread_local! {
    static LUT: vyges_stt::flute::lut::Lut = vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
}

fn to_rsmt(t: &vyges_stt::Tree) -> vyges_grt::RsmtTree {
    vyges_grt::RsmtTree { deg: t.deg, length: t.length, branch: t.branch.iter().map(|b| vyges_grt::Branch { x: b.x, y: b.y, n: b.n }).collect() }
}

/// `SteinerTreeBuilder::makeSteinerTree(x, y, drvr, alpha)` — Prim-Dijkstra or FLUTE.
fn stt(x: &[i32], y: &[i32], drvr: usize, alpha: f32) -> vyges_grt::RsmtTree {
    LUT.with(|lut| to_rsmt(&vyges_stt::make_steiner_tree(lut, x, y, drvr, alpha).0.expect("a tree")))
}

/// FastRoute's own pre-sorted FLUTE.
fn flutes(xs: &[i32], ys: &[i32], s: &[usize], acc: i32) -> vyges_grt::RsmtTree {
    LUT.with(|lut| to_rsmt(&vyges_stt::flute::medium_degree::flutes_all_degree_acc(lut, xs.len(), xs, ys, s, acc).expect("flute")))
}

/// Parse a guide file: `net`, `(`, `x0 y0 x1 y1 layer` lines, `)`.
fn parse_guides(text: &str) -> Vec<(String, Vec<(i32, i32, i32, i32, String)>)> {
    let mut out: Vec<(String, Vec<(i32, i32, i32, i32, String)>)> = Vec::new();
    for line in text.lines().map(str::trim).filter(|l| !l.is_empty() && *l != "(" && *l != ")") {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() == 5 {
            let g = (f[0].parse().expect("x"), f[1].parse().expect("y"), f[2].parse().expect("x"), f[3].parse().expect("y"), f[4].to_string());
            out.last_mut().expect("a net").1.push(g);
        } else {
            out.push((line.to_string(), Vec::new()));
        }
    }
    out
}

/// ⭐ THE END-TO-END GATE: the reference's own LEF/DEF → our reader → setup → run() → F → X, against
/// the guide file a FRESH run of the reference wrote for the same script. Every net, every guide,
/// in order.
///
/// Needs `GRT_REF_TESTS` and `GRT_FRESH_GUIDES` (a directory of `<script>.guide` from the clean
/// reference binary).
#[test]
fn the_guides_match_a_fresh_reference_run() {
    let (Ok(dir), Ok(fresh)) = (std::env::var("GRT_REF_TESTS"), std::env::var("GRT_FRESH_GUIDES")) else {
        eprintln!("GRT_REF_TESTS / GRT_FRESH_GUIDES unset: skipped");
        return;
    };
    let mut report = Vec::new();
    let (mut exact, mut attempted) = (0, 0);
    for (script, lefs, def, _) in TIER_A {
        // multiple_calls writes after a SECOND global_route; set_nets_to_route1 needs its patterns
        // resolved — both held out until the driver models them.
        if matches!(*script, "multiple_calls" | "set_nets_to_route1") {
            continue;
        }
        attempted += 1;
        let (mut db, mut opts) = load_with(&dir, lefs, def, &tier_a_commands(script));
        match *script {
            "gcd_flute" => opts.alpha = 0.0,
            "pd3" => opts.min_fanout_alpha = Some((9, 0.9)),
            "skip_large_fanout1" => opts.skip_large_fanout = 30,
            "congestion1" | "congestion2" | "congestion7" => opts.allow_congestion = true,
            _ => {}
        }
        opts.verbose = *script != "silence";
        let res = match vyges_grt::global_route::route_design(&mut db, &opts, &stt, &flutes) {
            Ok(r) => r,
            Err(e) => {
                report.push(format!("{script}: ERROR {e}"));
                continue;
            }
        };
        let want = parse_guides(&std::fs::read_to_string(format!("{fresh}/{script}.guide")).expect("fresh guide"));
        let want_map: std::collections::HashMap<&str, &Vec<(i32, i32, i32, i32, String)>> = want.iter().map(|(n, g)| (n.as_str(), g)).collect();
        let (mut nets_ok, mut first_bad) = (0, None);
        for ng in &res.guides {
            let got: Vec<(i32, i32, i32, i32, String)> = ng.guides.iter().map(|g| (g.box_.x_min, g.box_.y_min, g.box_.x_max, g.box_.y_max, res.layer_names[&g.layer].clone())).collect();
            match want_map.get(ng.net.as_str()) {
                Some(w) if **w == got => nets_ok += 1,
                w => {
                    if first_bad.is_none() {
                        first_bad = Some(format!("net {}: ours {} guides {:?}…, reference {:?}", ng.net, got.len(), got.first(), w.map(|w| (w.len(), w.first()))));
                    }
                }
            }
        }
        let ok = nets_ok == want.len() && res.guides.len() == want.len();
        exact += usize::from(ok);
        report.push(format!(
            "{script}: {} — {nets_ok}/{} nets exact (ours {} nets), overflow {}{}",
            if ok { "EXACT" } else { "DIFF" },
            want.len(),
            res.guides.len(),
            res.total_overflow,
            first_bad.map(|b| format!("; first: {b}")).unwrap_or_default()
        ));
    }
    eprintln!("gate:\n  {}", report.join("\n  "));
    eprintln!("gate: {exact}/{attempted} designs exact");
    assert_eq!(exact, attempted, "the end-to-end guide gate");
}

/// ⛔ `dbTransform::apply(Rect&)`, all eight orientations, worked from odb's point rules: the box
/// (1, 2)–(3, 5) under each, moved by (100, 200). `MYR90` / `MXR90` mirror FIRST, then rotate.
/// The designs exercise only the row orientations (R0, MX); this is the witness for the rest.
#[test]
fn instance_transforms_follow_odb() {
    use vyges_grt::read::transform_rect;
    use vyges_grt::Rect;
    let r = Rect { x_min: 1, y_min: 2, x_max: 3, y_max: 5 };
    let t = |o: &str| {
        let q = transform_rect(o, (100, 200), r);
        (q.x_min, q.y_min, q.x_max, q.y_max)
    };
    assert_eq!(t("R0"), (101, 202, 103, 205));
    assert_eq!(t("R90"), (95, 201, 98, 203)); // (x, y) → (-y, x)
    assert_eq!(t("R180"), (97, 195, 99, 198)); // → (-x, -y)
    assert_eq!(t("R270"), (102, 197, 105, 199)); // → (y, -x)
    assert_eq!(t("MY"), (97, 202, 99, 205)); // → (-x, y)
    assert_eq!(t("MX"), (101, 195, 103, 198)); // → (x, -y)
    assert_eq!(t("MYR90"), (95, 197, 98, 199)); // (-x, y) then R90 → (-y, -x)
    assert_eq!(t("MXR90"), (102, 201, 105, 203)); // (x, -y) then R90 → (y, x)
}
