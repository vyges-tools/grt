// SPDX-License-Identifier: Apache-2.0
//! `vyges-grt` — global routing over a LEF/DEF (or `.odb`) design: route guides out.
//!
//! One command, `route <job.json>`: the design's inputs and an ordered list of steps — the same
//! sequence a routing script runs (layer settings, adjustments, one or more `global_route`s,
//! `write_guides`). Order matters: a per-layer adjustment is judged against the max routing layer
//! AT THE TIME it is given, and a second `global_route` runs on the database the first left.
//!
//! Exit status: 0 routed (report on stdout), 1 refused (a feature this engine does not model yet —
//! named in the report), 2 usage or read error.

use std::collections::BTreeMap;
use std::process::ExitCode;

use serde_json::{json, Value};
use vyges_grt::global_route::{add_layer_adjustment, route_design, RouteOptions};
use vyges_opendb::Db;

const USAGE: &str = "\
vyges-grt — global routing: route guides from a placed design

USAGE:
  vyges-grt route <job.json>
  vyges-grt --help

JOB (JSON):
  { \"lefs\": [..], \"liberty\": [..], \"def\": \"..\" | \"db\": \"..\", \"timer_slacks\": \"..\",
    \"steps\": [ STEP, .. ] }
  timer_slacks: slacks captured from a reference timer, `<call> <net> <f32 hex bits>` per line.
  With liberty and no clock every net is unconstrained; with a clock, a pass that reads slacks is refused.
  STEP is one of
    { \"cmd\": \"set_routing_layers\", \"signal\": [lo, hi], \"clock\": [lo, hi] }
    { \"cmd\": \"layer_adjustment\", \"layers\": [lo, hi], \"value\": f }      (one layer: lo == hi)
    { \"cmd\": \"global_adjustment\", \"value\": f }                         (the `*` form)
    { \"cmd\": \"region_adjustment\", \"rect_um\": [x0, y0, x1, y1], \"layer\": l, \"value\": f }
    { \"cmd\": \"routing_alpha\", \"alpha\": f, \"nets\": [..] | \"min_fanout\": n | \"min_hpwl\": um | \"clock_nets\": true }
    { \"cmd\": \"nets_to_route\", \"patterns\": [..] }                        (Tcl globs)
    { \"cmd\": \"global_route\", \"verbose\": b, \"allow_congestion\": b, \"grid_origin\": [x, y],
      \"skip_large_fanout\": n, \"congestion_iterations\": n, \"critical_nets_percentage\": f,
      \"resistance_aware\": b, \"res_aware_nets_percentage\": f }
    { \"cmd\": \"create_clock\", \"ports\": [..] }                        (clock network only)
    { \"cmd\": \"set_layer_rc\", \"layer\" | \"via\": name, \"resistance\": f } (user units)
    { \"cmd\": \"propagated_clock\" }
    { \"cmd\": \"write_parasitics\", \"path\": \"..\" }       (the networks the slacks are read from)
    { \"cmd\": \"write_spef\", \"path\": \"..\", \"source\": \"partial\" | \"routed\" }
    { \"cmd\": \"write_guides\", \"path\": \"..\" }

EXIT STATUS:
  0  routed    every step ran; the report lists each global_route
  1  refused   a step needs a feature not modelled yet (named)
  2  error     usage, unreadable input, or a failed write
";

thread_local! {
    static LUT: vyges_stt::flute::lut::Lut = vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
}

fn to_rsmt(t: &vyges_stt::Tree) -> vyges_grt::RsmtTree {
    vyges_grt::RsmtTree { deg: t.deg, length: t.length, branch: t.branch.iter().map(|b| vyges_grt::Branch { x: b.x, y: b.y, n: b.n }).collect() }
}

/// `SteinerTreeBuilder::makeSteinerTree(x, y, drvr, alpha)`.
fn stt(x: &[i32], y: &[i32], drvr: usize, alpha: f32) -> vyges_grt::RsmtTree {
    LUT.with(|lut| to_rsmt(&vyges_stt::make_steiner_tree(lut, x, y, drvr, alpha).0.expect("a Steiner tree")))
}

/// FastRoute's pre-sorted FLUTE.
fn flutes(xs: &[i32], ys: &[i32], s: &[usize], acc: i32) -> vyges_grt::RsmtTree {
    LUT.with(|lut| to_rsmt(&vyges_stt::flute::medium_degree::flutes_all_degree_acc(lut, xs.len(), xs, ys, s, acc).expect("flute")))
}

/// A Tcl `string match` glob: `*` any run, `?` one character.
fn glob(pattern: &str, name: &str) -> bool {
    fn m(p: &[u8], n: &[u8]) -> bool {
        match (p.first(), n.first()) {
            (None, None) => true,
            (Some(b'*'), _) => m(&p[1..], n) || (!n.is_empty() && m(p, &n[1..])),
            (Some(b'?'), Some(_)) => m(&p[1..], &n[1..]),
            (Some(a), Some(b)) if a == b => m(&p[1..], &n[1..]),
            _ => false,
        }
    }
    m(pattern.as_bytes(), name.as_bytes())
}

enum Fail {
    Refused(String),
    Error(String),
}

fn err<E: std::fmt::Display>(e: E) -> Fail {
    Fail::Error(e.to_string())
}

/// A message from the engine naming something it does not model yet.
fn classify(msg: String) -> Fail {
    if ["not wired", "not bound", "none bound", "not modelled"].iter().any(|k| msg.contains(k)) {
        Fail::Refused(msg)
    } else {
        Fail::Error(msg)
    }
}

fn level(db: &Db, v: &Value) -> Result<i32, Fail> {
    let name = v.as_str().ok_or_else(|| Fail::Error(format!("a layer name, got {v}")))?;
    let l = db.layer_get_routing_level(name);
    if l <= 0 {
        return Err(Fail::Error(format!("{name} is not a routing layer")));
    }
    Ok(l)
}

/// `dbBlock::writeGuides`: nets with guides, sorted by name (byte order), each `net ( x0 y0 x1 y1
/// layer … )`.
fn write_guides(path: &str, store: &BTreeMap<String, Vec<(i32, i32, i32, i32, String)>>) -> Result<(), Fail> {
    let mut names: Vec<&String> = store.keys().filter(|n| !store[*n].is_empty()).collect();
    names.sort_by(|a, b| a.as_bytes().cmp(b.as_bytes()));
    let mut text = String::new();
    for n in names {
        text.push_str(n);
        text.push_str("\n(\n");
        for (x0, y0, x1, y1, l) in &store[n] {
            text.push_str(&format!("{x0} {y0} {x1} {y1} {l}\n"));
        }
        text.push_str(")\n");
    }
    std::fs::write(path, text).map_err(err)
}

fn run(job: &Value) -> Result<Value, Fail> {
    let mut db = Db::new();
    for lef in job["lefs"].as_array().map(Vec::as_slice).unwrap_or(&[]) {
        db.read_lef(lef.as_str().ok_or_else(|| err("a LEF path"))?).map_err(err)?;
    }
    if let Some(def) = job["def"].as_str() {
        db.read_def(def, "default").map_err(err)?;
    } else if let Some(odb) = job["db"].as_str() {
        db = Db::open(odb).map_err(err)?;
    }
    let mut opts = RouteOptions::new();
    // read_liberty — the libraries in read order; a cell in two resolves to the first.
    for lib in job["liberty"].as_array().map(Vec::as_slice).unwrap_or(&[]) {
        let path = lib.as_str().ok_or_else(|| err("a liberty path"))?;
        let text = if path.ends_with(".gz") {
            let mut s = String::new();
            std::io::Read::read_to_string(&mut flate2::read::MultiGzDecoder::new(std::fs::File::open(path).map_err(err)?), &mut s).map_err(err)?;
            s
        } else {
            std::fs::read_to_string(path).map_err(err)?
        };
        opts.liberty.get_or_insert_with(Default::default).read(&text).map_err(|e| Fail::Refused(format!("{path}: {e}")))?;
    }
    // timer_slacks — the reference timer's slacks, captured per call: `P <call> <net> <bits>` for
    // a partial-slack call (`CalculatePartialSlack`), `U <call> <net> <bits>` for an `updateSlacks`
    // call (a bare `<call> <net> <bits>` is a P line). An ORACLE: no timing is computed here.
    if let Some(path) = job["timer_slacks"].as_str() {
        let (mut partial, mut update): (Vec<BTreeMap<String, f32>>, Vec<BTreeMap<String, f32>>) = (Vec::new(), Vec::new());
        for (n, line) in std::fs::read_to_string(path).map_err(err)?.lines().enumerate() {
            let f: Vec<&str> = line.split_whitespace().collect();
            let bad = || err(format!("{path}:{}: expected `[P|U] <call> <net> <bits>`", n + 1));
            let (calls, k, net, bits) = match f[..] {
                ["U", k, net, bits] => (&mut update, k, net, bits),
                ["P", k, net, bits] | [k, net, bits] => (&mut partial, k, net, bits),
                _ => return Err(bad()),
            };
            let k: usize = k.parse().map_err(|_| bad())?;
            let v = f32::from_bits(u32::from_str_radix(bits, 16).map_err(|_| bad())?);
            if k > calls.len() {
                return Err(bad());
            }
            if k == calls.len() {
                calls.push(BTreeMap::new());
            }
            calls[k].insert(net.to_string(), v);
        }
        opts.captured_slacks = Some(partial);
        opts.captured_update_slacks = Some(update);
    }
    let mut guides: BTreeMap<String, Vec<(i32, i32, i32, i32, String)>> = BTreeMap::new();
    let mut parasitics: BTreeMap<String, vyges_grt::parasitics::Network> = BTreeMap::new();
    let mut parasitic_pins: BTreeMap<String, Vec<vyges_grt::parasitics::PinGridLocation>> = BTreeMap::new();
    let mut routed_parasitics: BTreeMap<String, vyges_grt::parasitics::Network> = BTreeMap::new();
    let mut planar_routes: BTreeMap<String, Vec<vyges_grt::parasitics::Segment>> = BTreeMap::new();
    let mut snapshot_edges: BTreeMap<String, Vec<vyges_grt::global_route::SnapshotEdge>> = BTreeMap::new();
    let mut calls = Vec::new();
    let mut log = Vec::new();
    for step in job["steps"].as_array().ok_or_else(|| err("steps"))? {
        match step["cmd"].as_str().unwrap_or("") {
            "set_routing_layers" => {
                if let Some([lo, hi]) = step["signal"].as_array().map(Vec::as_slice) {
                    let (lo, hi) = (level(&db, lo)?, level(&db, hi)?);
                    db.block_set_min_routing_layer(lo).map_err(err)?;
                    db.block_set_max_routing_layer(hi).map_err(err)?;
                }
                if let Some([lo, hi]) = step["clock"].as_array().map(Vec::as_slice) {
                    let (lo, hi) = (level(&db, lo)?, level(&db, hi)?);
                    db.block_set_min_layer_for_clock(lo).map_err(err)?;
                    db.block_set_max_layer_for_clock(hi).map_err(err)?;
                }
            }
            "layer_adjustment" => {
                let [lo, hi] = step["layers"].as_array().map(Vec::as_slice).ok_or_else(|| err("layers"))? else { return Err(err("layers: [lo, hi]")) };
                let v = step["value"].as_f64().ok_or_else(|| err("value"))? as f32;
                for l in level(&db, lo)?..=level(&db, hi)? {
                    add_layer_adjustment(&mut db, l, v, opts.verbose, &mut log).map_err(err)?;
                }
            }
            "global_adjustment" => opts.adjustment = step["value"].as_f64().ok_or_else(|| err("value"))? as f32,
            "region_adjustment" => {
                let r: Vec<f64> = step["rect_um"].as_array().ok_or_else(|| err("rect_um"))?.iter().filter_map(Value::as_f64).collect();
                let dbu = f64::from(db.dbu_per_micron());
                let u = |v: f64| (v * dbu) as i32;
                let layer = level(&db, &step["layer"])?;
                opts.region_adjustments.push((vyges_grt::Rect { x_min: u(r[0]), y_min: u(r[1]), x_max: u(r[2]), y_max: u(r[3]) }, layer, step["value"].as_f64().unwrap_or(0.0) as f32));
            }
            // stt's set_routing_alpha: -net, else -min_fanout, else -min_hpwl, else -clock_nets, else
            // the global alpha.
            "routing_alpha" => {
                let a = step["alpha"].as_f64().ok_or_else(|| err("alpha"))? as f32;
                if let Some(nets) = step["nets"].as_array() {
                    for n in nets {
                        let n = n.as_str().ok_or_else(|| err("a net name"))?;
                        if !db.net_names().iter().any(|m| m == n) {
                            return Err(err(format!("net {n} not found")));
                        }
                        opts.net_alpha.insert(n.to_string(), a);
                    }
                } else if let Some(n) = step["min_fanout"].as_i64() {
                    opts.min_fanout_alpha = Some((n as i32, a));
                } else if let Some(um) = step["min_hpwl"].as_f64() {
                    // microns_to_dbu: std::round, half away from zero.
                    let dbu = (um * f64::from(db.tech_get_db_units_per_micron())).round() as i32;
                    opts.min_hpwl_alpha = Some((dbu, a));
                } else if step["clock_nets"].as_bool() == Some(true) {
                    // filter_clk_nets: the nets typed CLOCK in the database right now.
                    let clk: Vec<String> = db.net_names().into_iter().filter(|n| db.net_sigtype(n) == "CLOCK").collect();
                    if clk.is_empty() {
                        return Err(Fail::Error("STT-0006: Clock nets for set_routing_alpha command were not found".into()));
                    }
                    for n in clk {
                        opts.net_alpha.insert(n, a);
                    }
                } else {
                    opts.alpha = a;
                }
            }
            "nets_to_route" => {
                let names = db.net_names();
                let pats: Vec<&str> = step["patterns"].as_array().ok_or_else(|| err("patterns"))?.iter().filter_map(Value::as_str).collect();
                opts.nets_to_route = Some(pats.iter().flat_map(|p| names.iter().filter(|n| glob(p, n)).cloned().collect::<Vec<_>>()).collect());
            }
            "global_route" => {
                opts.verbose = step["verbose"].as_bool().unwrap_or(false);
                opts.allow_congestion = step["allow_congestion"].as_bool().unwrap_or(false);
                if let Some([x, y]) = step["grid_origin"].as_array().map(Vec::as_slice) {
                    opts.grid_origin = (x.as_i64().unwrap_or(0) as i32, y.as_i64().unwrap_or(0) as i32);
                }
                if let Some(n) = step["skip_large_fanout"].as_i64() {
                    opts.skip_large_fanout = n as i32;
                }
                if let Some(n) = step["congestion_iterations"].as_i64() {
                    opts.congestion_iterations = n as i32;
                }
                // setCriticalNetsPercentage: zeroed without a liberty library (GRT-301); it persists
                // into later calls like the router's member.
                // setResistanceAware; setResAwareNetsPercentage — zeroed (GRT-308) when not enabled,
                // and FIXED from then on either way.
                opts.resistance_aware = step["resistance_aware"].as_bool().unwrap_or(false);
                if let Some(p) = step["res_aware_nets_percentage"].as_f64() {
                    opts.res_aware_nets_percentage = Some(if opts.resistance_aware { p as f32 } else { 0.0 });
                }
                if let Some(p) = step["critical_nets_percentage"].as_f64() {
                    opts.critical_nets_percentage = if opts.liberty.is_some() { p as f32 } else { 0.0 };
                }
                let res = route_design(&mut db, &opts, &stt, &flutes).map_err(|e| classify(e.to_string()))?;
                // saveGuides replaces the guides of every net it routes; the others keep theirs.
                parasitics = res.parasitics.clone();
                parasitic_pins = res.parasitic_pins.clone();
                routed_parasitics = res.routed_parasitics.clone();
                planar_routes = res.planar_routes.clone();
                snapshot_edges = res.snapshot_edges.clone();
                for ng in &res.guides {
                    guides.insert(ng.net.clone(), ng.guides.iter().map(|g| (g.box_.x_min, g.box_.y_min, g.box_.x_max, g.box_.y_max, res.layer_names[&g.layer].clone())).collect());
                }
                calls.push(json!({ "nets": res.guides.len(), "total_overflow": res.total_overflow, "congested": res.guide_is_congested, "clock_nets": res.clock_nets }));
                if res.total_overflow > 0 && !opts.allow_congestion {
                    // GRT-116: the reference ends the command in error after writing the guides.
                    return Err(Fail::Error("GRT-0116: Global routing finished with congestion".into()));
                }
            }
            // set_layer_rc — ⛔ it WRITES the technology's resistance (set_dblayer_wire_rc /
            // set_dbvia_wire_r), which resistance-aware routing reads. Converted through the timer's
            // units (the first liberty's; float scales) in the Tcl's double arithmetic, in its order.
            // The capacitance reaches no router input; `-corner` writes no database value.
            "set_layer_rc" => {
                let units = opts.liberty.as_ref().and_then(|l| l.units).ok_or_else(|| Fail::Refused("set_layer_rc before a liberty library: the timer's default units are not modelled".into()))?;
                let res_ui = step["resistance"].as_f64();
                if let Some(layer) = step["layer"].as_str() {
                    // ⛔ No -resistance is 0.0, and set_dblayer_wire_rc still WRITES it.
                    let r = vyges_grt::pricing::set_dblayer_wire_r(res_ui.unwrap_or(0.0), units.resistance, units.distance, db.layer_get_width(layer) as i32, db.tech_get_db_units_per_micron());
                    db.layer_set_resistance(layer, r).map_err(err)?;
                    // …and the ESTIMATOR's own table, which the parasitics read first: ohm/m and
                    // F/m, the Tcl's `[unit_ui_sta $v] / [distance_ui_sta 1.0]`.
                    let d = f64::from(units.distance);
                    // ⛔ `set_layer_rc_cmd(layer, corner, float res, float cap)` — the Tcl's double
                    // is NARROWED to a float on the way into the table (892900.0022543557 is stored
                    // as 892900.0), which is why the table and the database hold different values.
                    let res_per_m = f64::from(((res_ui.unwrap_or(0.0) * f64::from(units.resistance)) / (1.0 * d)) as f32);
                    let cap_per_m = f64::from(((step["capacitance"].as_f64().unwrap_or(0.0) * f64::from(units.capacitance)) / (1.0 * d)) as f32);
                    opts.layer_rc.insert(db.layer_get_routing_level(layer), (res_per_m, cap_per_m));
                    // set_dblayer_wire_rc also writes the layer's capacitance and ZEROES its edge
                    // capacitance, so a later fallback reads the user's value and nothing else.
                    if let Some(c) = step["capacitance"].as_f64() {
                        let per_square = vyges_grt::pricing::set_dblayer_wire_c(c, units.capacitance, units.distance, db.layer_get_width(layer) as i32, db.tech_get_db_units_per_micron());
                        db.layer_set_capacitance(layer, per_square).map_err(err)?;
                        db.layer_set_edge_capacitance(layer, 0.0).map_err(err)?;
                    }
                } else if let Some(via) = step["via"].as_str() {
                    let res_ui = res_ui.ok_or_else(|| err("set_layer_rc -via needs -resistance"))?;
                    let r = vyges_grt::pricing::set_dbvia_wire_r(res_ui, units.resistance);
                    db.layer_set_resistance(via, r).map_err(err)?;
                    opts.via_rc.insert(via.to_string(), res_ui * f64::from(units.resistance));
                } else {
                    return Err(err("set_layer_rc needs a layer or via"));
                }
            }
            // create_clock on top-level ports; the clock's period and waveform do not reach the
            // clock network.
            "create_clock" => {
                for p in step["ports"].as_array().ok_or_else(|| err("ports"))? {
                    opts.clock_sources.push(p.as_str().ok_or_else(|| err("a port name"))?.to_string());
                }
            }
            // set_propagated_clock: only the slacks read it, and with a clock none are bound.
            "propagated_clock" => {}
            // The parasitics the router's own slacks are read from, in the reference's dump shape:
            // `<net>|node|<node>|<farads>` and `<net>|res|<n1>|<n2>|<ohms>`.
            "write_parasitics" => {
                let path = step["path"].as_str().ok_or_else(|| err("path"))?;
                let parasitics: &BTreeMap<String, vyges_grt::parasitics::Network> = match step["source"].as_str().unwrap_or("partial") {
                    "partial" => &parasitics,
                    "routed" => &routed_parasitics,
                    other => return Err(err(format!("write_parasitics source {other:?}: expected partial or routed"))),
                };
                let mut text = String::new();
                for (net, edges) in &snapshot_edges {
                    for (e, ed) in edges.iter().enumerate() {
                        let g: String = ed.grids.iter().map(|(x, y)| format!("{x},{y};")).collect();
                        text.push_str(&format!("{net}|edge|{e}|n1={}|n2={}|len={}|rl={}|g={g}\n", ed.n1, ed.n2, ed.len, ed.routelen));
                    }
                }
                for (net, route) in &planar_routes {
                    for sg in route {
                        text.push_str(&format!("{net}|seg|{}|{}|{}|{}|{}|{}\n", sg.init_x, sg.init_y, sg.init_layer, sg.final_x, sg.final_y, sg.final_layer));
                    }
                }
                for (net, g) in parasitics {
                    let name = |n: &vyges_grt::parasitics::NodeId| match n {
                        vyges_grt::parasitics::NodeId::Pin(p) => p.clone(),
                        // ⚠️ The reference names a net node from ONE: `ensureParasiticNode(…, node_map.size(), …)`
                        // with a size of 0 prints as `<net>:1`.
                        vyges_grt::parasitics::NodeId::Point(i) => format!("{net}:{}", i + 1),
                    };
                    for (node, cap) in &g.nodes {
                        text.push_str(&format!("{net}|node|{}|{:.8e}\n", name(node), cap));
                    }
                    for (n1, n2, res) in &g.resistors {
                        text.push_str(&format!("{net}|res|{}|{}|{:.8e}\n", name(n1), name(n2), res));
                    }
                }
                std::fs::write(path, text).map_err(err)?;
            }
            // The same networks as SPEF, for a timer that reads one.
            //
            // ⛔ Every node's ground capacitance is written, pin nodes included — OpenROAD's own
            // writer emits `*CAP` only for nodes with no pin, so its file understates the load
            // and no longer sums to its own `*D_NET` total (OpenROAD #11482).
            "write_spef" => {
                let path = step["path"].as_str().ok_or_else(|| err("path"))?;
                // "partial" (the default): the planar routes the router's own slacks were read
                // from. "routed": the saved routes, as `estimate_parasitics -global_routing`.
                let parasitics: &BTreeMap<String, vyges_grt::parasitics::Network> = match step["source"].as_str().unwrap_or("partial") {
                    "partial" => &parasitics,
                    "routed" => &routed_parasitics,
                    other => return Err(err(format!("write_spef source {other:?}: expected partial or routed"))),
                };
                let mut text = String::from(
                    "*SPEF \"ieee 1481-1999\"\n*DESIGN \"vyges-grt\"\n*DATE \"\"\n*VENDOR \"Vyges\"\n*PROGRAM \"vyges-grt\"\n*VERSION \"1.0\"\n\
                     *DESIGN_FLOW \"NAME_SCOPE LOCAL\" \"PIN_CAP NONE\"\n*DIVIDER /\n*DELIMITER :\n*BUS_DELIMITER []\n\
                     *T_UNIT 1 NS\n*C_UNIT 1 PF\n*R_UNIT 1 KOHM\n*L_UNIT 1 HENRY\n\n",
                );
                for (net, g) in parasitics {
                    // A parasitic node is `<instance>:<terminal>` for a pin, `<net>:<id>` otherwise.
                    let pin_name = |p: &str| match p.rfind('/') {
                        Some(i) => format!("{}:{}", &p[..i], &p[i + 1..]),
                        None => p.to_string(),
                    };
                    let node_name = |n: &vyges_grt::parasitics::NodeId| match n {
                        vyges_grt::parasitics::NodeId::Pin(p) => pin_name(p),
                        vyges_grt::parasitics::NodeId::Point(i) => format!("{net}:{}", i + 1),
                    };
                    // ⚠️ Only `$` is escaped; the database's names already carry their bracket escapes.
                    let esc = |s: &str| s.replace('$', "\\$");
                    let total: f64 = g.nodes.iter().map(|(_, c)| f64::from(*c)).sum();
                    text.push_str(&format!("*D_NET {} {}\n*CONN\n", esc(net), total * 1e12));
                    let pins = parasitic_pins.get(net).cloned().unwrap_or_default();
                    for (n, _) in &g.nodes {
                        if let vyges_grt::parasitics::NodeId::Pin(p) = n {
                            let pin = pins.iter().find(|q| q.name == *p);
                            let dir = if pin.is_some_and(|q| q.is_driver) { "O" } else { "I" };
                            // A port is `*P`, an instance terminal `*I`; a port's direction is
                            // the other way round, since a driving port feeds the net.
                            if pin.is_some_and(|q| q.is_port) {
                                text.push_str(&format!("*P {} {}\n", esc(p), if dir == "O" { "I" } else { "O" }));
                            } else {
                                text.push_str(&format!("*I {} {}\n", esc(&pin_name(p)), dir));
                            }
                        }
                    }
                    let caps: Vec<&(vyges_grt::parasitics::NodeId, f32)> = g.nodes.iter().filter(|(_, c)| *c != 0.0).collect();
                    if !caps.is_empty() {
                        text.push_str("*CAP\n");
                        for (i, (n, c)) in caps.iter().enumerate() {
                            text.push_str(&format!("{} {} {}\n", i + 1, esc(&node_name(n)), f64::from(*c) * 1e12));
                        }
                    }
                    if !g.resistors.is_empty() {
                        text.push_str("*RES\n");
                        for (i, (a, b, v)) in g.resistors.iter().enumerate() {
                            text.push_str(&format!("{} {} {} {}\n", i + 1, esc(&node_name(a)), esc(&node_name(b)), f64::from(*v) / 1000.0));
                        }
                    }
                    text.push_str("*END\n\n");
                }
                std::fs::write(path, text).map_err(err)?;
            }
            "write_guides" => write_guides(step["path"].as_str().ok_or_else(|| err("path"))?, &guides)?,
            other => return Err(Fail::Refused(format!("step {other:?} is not modelled"))),
        }
    }
    Ok(json!({ "status": "routed", "global_route": calls }))
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
        ["route", path] => {
            let job: Value = match std::fs::read_to_string(path).map_err(|e| e.to_string()).and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string())) {
                Ok(j) => j,
                Err(e) => {
                    eprintln!("vyges-grt: {path}: {e}");
                    return ExitCode::from(2);
                }
            };
            match run(&job) {
                Ok(report) => {
                    println!("{report}");
                    ExitCode::SUCCESS
                }
                Err(Fail::Refused(why)) => {
                    println!("{}", json!({ "status": "refused", "reason": why }));
                    ExitCode::from(1)
                }
                Err(Fail::Error(why)) => {
                    println!("{}", json!({ "status": "error", "reason": why }));
                    ExitCode::from(2)
                }
            }
        }
        ["--help"] | ["-h"] => {
            print!("{USAGE}");
            ExitCode::SUCCESS
        }
        _ => {
            eprint!("{USAGE}");
            ExitCode::from(2)
        }
    }
}
