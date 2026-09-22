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
  { \"lefs\": [..], \"liberty\": [..], \"def\": \"..\" | \"db\": \"..\", \"steps\": [ STEP, .. ] }
  With liberty and no clock defined (there is no clock step), every net is unconstrained.
  STEP is one of
    { \"cmd\": \"set_routing_layers\", \"signal\": [lo, hi], \"clock\": [lo, hi] }
    { \"cmd\": \"layer_adjustment\", \"layers\": [lo, hi], \"value\": f }      (one layer: lo == hi)
    { \"cmd\": \"global_adjustment\", \"value\": f }                         (the `*` form)
    { \"cmd\": \"region_adjustment\", \"rect_um\": [x0, y0, x1, y1], \"layer\": l, \"value\": f }
    { \"cmd\": \"routing_alpha\", \"alpha\": f, \"min_fanout\": n }
    { \"cmd\": \"nets_to_route\", \"patterns\": [..] }                        (Tcl globs)
    { \"cmd\": \"global_route\", \"verbose\": b, \"allow_congestion\": b, \"grid_origin\": [x, y],
      \"skip_large_fanout\": n, \"congestion_iterations\": n, \"critical_nets_percentage\": f }
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
    if msg.contains("not wired") || msg.contains("not bound") {
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
        let text = std::fs::read_to_string(path).map_err(err)?;
        opts.liberty.get_or_insert_with(Default::default).read(&text).map_err(|e| Fail::Refused(format!("{path}: {e}")))?;
    }
    let mut guides: BTreeMap<String, Vec<(i32, i32, i32, i32, String)>> = BTreeMap::new();
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
            "routing_alpha" => {
                let a = step["alpha"].as_f64().ok_or_else(|| err("alpha"))? as f32;
                match step["min_fanout"].as_i64() {
                    Some(n) => opts.min_fanout_alpha = Some((n as i32, a)),
                    None => opts.alpha = a,
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
                if let Some(p) = step["critical_nets_percentage"].as_f64() {
                    opts.critical_nets_percentage = if opts.liberty.is_some() { p as f32 } else { 0.0 };
                }
                let res = route_design(&mut db, &opts, &stt, &flutes).map_err(|e| classify(e.to_string()))?;
                // saveGuides replaces the guides of every net it routes; the others keep theirs.
                for ng in &res.guides {
                    guides.insert(ng.net.clone(), ng.guides.iter().map(|g| (g.box_.x_min, g.box_.y_min, g.box_.x_max, g.box_.y_max, res.layer_names[&g.layer].clone())).collect());
                }
                calls.push(json!({ "nets": res.guides.len(), "total_overflow": res.total_overflow, "congested": res.guide_is_congested }));
                if res.total_overflow > 0 && !opts.allow_congestion {
                    // GRT-116: the reference ends the command in error after writing the guides.
                    return Err(Fail::Error("GRT-0116: Global routing finished with congestion".into()));
                }
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
