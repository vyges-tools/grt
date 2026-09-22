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
    { \"cmd\": \"antenna_wires\", \"path\": \"..\" }      (the wires antenna checking synthesises from the guides)
    { \"cmd\": \"repair_antennas\", \"violations\": \"..\", \"jumper_only\": b, \"diode_only\": b, \"iterations\": n,
      \"allow_congestion\": b, \"trace\": \"..\" }
      violations: the reference checker's, captured (`VYGA|viol|…` lines) — an ORACLE; the
      violation check itself is not modelled. Diode insertion is refused.

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

/// The name of a routing level.
fn db_layer_name(db: &Db, level: i32) -> String {
    db.tech_get_layers().into_iter().find(|l| db.layer_get_type(l).is_ok_and(|t| t == "ROUTING") && db.layer_get_routing_level(l) == level).unwrap_or_default()
}

/// What the antenna wire builder reads from the database: every net in block order, with its
/// terminals as `makeWireFromGuides` / `makeWireToTerm` see them; the min routing layer; and
/// `dbBlock::getDefaultVias` by bottom routing level.
#[allow(clippy::type_complexity)]
fn ant_nets(db: &Db, db_guides: &BTreeMap<String, Vec<vyges_grt::Guide>>) -> Result<(Vec<vyges_grt::wire_builder::AntNet>, vyges_grt::wire_builder::WireTech, BTreeMap<i32, String>), Fail> {
    use vyges_grt::wire_builder::{AntNet, TermFacts, WireTech};
    let level_of = |n: i64| -> (i32, bool) {
        let name = db.layer_name_by_number(n);
        let routing = db.layer_get_type(&name).is_ok_and(|t| t == "ROUTING");
        (if routing { db.layer_get_routing_level(&name) } else { 0 }, routing)
    };
    // A terminal from its shapes: (level, is ROUTING, placed rect), in pin -> shape order.
    let facts = |name: String, shapes: &[(i32, bool, vyges_grt::Rect)]| -> TermFacts {
        let top_level = shapes.iter().filter(|s| s.1).map(|s| s.0).max().unwrap_or(0);
        let mut bbox: Option<vyges_grt::Rect> = None;
        for (_, _, r) in shapes {
            bbox = Some(match bbox {
                None => *r,
                Some(b) => vyges_grt::Rect::new(b.x_min.min(r.x_min), b.y_min.min(r.y_min), b.x_max.max(r.x_max), b.y_max.max(r.y_max)),
            });
        }
        let top_rects = shapes.iter().filter(|s| s.1 && s.0 == top_level).map(|s| s.2).collect();
        // An empty terminal's box is inverted in the reference and overlaps nothing.
        TermFacts { name, top_level, bbox: bbox.unwrap_or(vyges_grt::Rect { x_min: i32::MAX, y_min: i32::MAX, x_max: i32::MIN, y_max: i32::MIN }), top_rects }
    };
    let mut nets = Vec::new();
    for n in db.net_names() {
        let mut iterms = Vec::new();
        for it in db.net_iterms(&n) {
            let (inst, pin) = it.rsplit_once('/').ok_or_else(|| err(format!("iterm {it}")))?;
            let master = db.inst_master(inst);
            let origin = (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst));
            let orient = db.inst_get_orient(inst);
            let shapes: Vec<(i32, bool, vyges_grt::Rect)> = db
                .mterm_pin_boxes(&master, pin)
                .map_err(err)?
                .into_iter()
                .map(|(l, x0, y0, x1, y1)| {
                    let (level, routing) = level_of(l);
                    (level, routing, vyges_grt::read::transform_rect(&orient, origin, vyges_grt::Rect::new(x0, y0, x1, y1)))
                })
                .collect();
            iterms.push(facts(it.clone(), &shapes));
        }
        let mut bterms = Vec::new();
        for bt in db.net_bterms(&n) {
            let (_, boxes) = vyges_grt::read::read_bterm(db, &bt).map_err(|e| err(format!("{e:?}")))?;
            bterms.push(facts(bt.clone(), &boxes));
        }
        nets.push(AntNet {
            is_special: db.net_is_special(&n),
            is_connected_by_abutment: db.net_is_connected_by_abutment(&n),
            term_count: db.net_get_term_count(&n),
            is_detailed_routed: db.net_get_wire_type(&n) == "ROUTED" && db.net_has_wire(&n),
            guides: db_guides.get(&n).cloned().unwrap_or_default(),
            iterms,
            bterms,
            name: n,
        });
    }
    let min = db.block_get_min_routing_layer();
    let dir = db.layers_with_direction().map_err(err)?.into_iter().find(|(l, _)| db.layer_get_routing_level(l) == min).map(|(_, d)| d).unwrap_or_default();
    // getDefaultVias: the OR_DEFAULT vias by bottom layer, the LAST in tech order winning; with
    // none at all, the FIRST via per bottom routing layer.
    let mut vias: BTreeMap<i32, String> = BTreeMap::new();
    let bottom = |v: &str| db.tech_via_layer(v, "bottom").map(|l| db.layer_get_routing_level(&l)).unwrap_or(0);
    for v in db.tech_get_vias() {
        if db.techvia_has_string_property(&v, "OR_DEFAULT") {
            vias.insert(bottom(&v), v);
        }
    }
    if vias.is_empty() {
        for v in db.tech_get_vias() {
            let b = bottom(&v);
            if b != 0 {
                vias.entry(b).or_insert(v);
            }
        }
    }
    Ok((nets, WireTech { min_routing_layer: min, min_layer_vertical: dir == "VERTICAL" }, vias))
}

/// The database as the wire codec reads it: routing layers by level, the default vias by bottom
/// level, and every tech layer's name by its position.
struct DbCodec {
    layers: BTreeMap<i32, vyges_grt::wire_codec::CodecLayer>,
    vias: BTreeMap<i32, vyges_grt::wire_codec::CodecVia>,
    tech_names: Vec<String>,
}

impl DbCodec {
    fn read(db: &Db, default_vias: &BTreeMap<i32, String>) -> Result<Self, Fail> {
        let tech_names = db.tech_get_layers();
        let mut layers = BTreeMap::new();
        for (name, dir) in db.layers_with_direction().map_err(err)? {
            let level = db.layer_get_routing_level(&name);
            if level > 0 {
                layers.insert(level, vyges_grt::wire_codec::CodecLayer {
                    width: db.layer_get_width(&name) as i32,
                    wrong_way_width: db.layer_get_wrong_way_width(&name) as i32,
                    vertical: dir == "VERTICAL",
                    horizontal: dir == "HORIZONTAL",
                });
            }
        }
        let mut vias = BTreeMap::new();
        for (&bottom, name) in default_vias {
            let top = db.tech_via_layer(name, "top").map(|l| db.layer_get_routing_level(&l)).map_err(err)?;
            let mut boxes = Vec::new();
            let mut bbox: Option<vyges_grt::Rect> = None;
            for (n, x0, y0, x1, y1) in db.tech_via_boxes(name).map_err(err)? {
                let layer = db.layer_name_by_number(n);
                let t = tech_names.iter().position(|m| *m == layer).ok_or_else(|| err(format!("via {name}: layer {layer}")))?;
                let r = vyges_grt::Rect::new(x0, y0, x1, y1);
                bbox = Some(match bbox {
                    None => r,
                    Some(b) => vyges_grt::Rect::new(b.x_min.min(r.x_min), b.y_min.min(r.y_min), b.x_max.max(r.x_max), b.y_max.max(r.y_max)),
                });
                boxes.push((t, r));
            }
            vias.insert(bottom, vyges_grt::wire_codec::CodecVia { name: name.clone(), bottom, top, bbox, boxes });
        }
        Ok(DbCodec { layers, vias, tech_names })
    }
}

impl vyges_grt::wire_codec::CodecTech for DbCodec {
    fn layer(&self, level: i32) -> &vyges_grt::wire_codec::CodecLayer {
        &self.layers[&level]
    }
    fn via(&self, bottom: i32) -> Option<&vyges_grt::wire_codec::CodecVia> {
        self.vias.get(&bottom)
    }
}

/// `checkAntennaViolations`' check over the whole block: a wire for every net from its guides, and
/// each wired net checked — its violations as `(net, routing level, gate names)`, nets in block
/// order (the violation map's order), each net's in the checker's order.
#[allow(clippy::type_complexity)]
fn check_design(db: &Db, db_guides: &BTreeMap<String, Vec<vyges_grt::Guide>>, with_diode: bool, ratio_margin: f32) -> Result<Vec<Viol>, Fail> {
    let (nets, wtech, vias) = ant_nets(db, db_guides)?;
    let wires = vyges_grt::wire_builder::make_net_wires_from_guides(&nets, db.block_get_g_cell_tile_size(), &wtech).map_err(|e| Fail::Refused(format!("{e:?}")))?;
    let codec = DbCodec::read(db, &vias)?;
    let tech = tech_layers(db)?;
    let mut checker = Checker::read(db, with_diode, ratio_margin)?;
    let mut out = Vec::new();
    for w in &wires {
        let ops = vyges_grt::wire_codec::encode(&w.ops, &codec).map_err(|e| Fail::Refused(format!("net {}: {e}", w.net)))?;
        let shapes = vyges_grt::wire_codec::decode(&ops, &codec);
        let pins = net_pin_facts(db, &w.net, &codec.tech_names)?;
        let boxes: Vec<(usize, vyges_grt::polygon90::R)> = pins.iter().flat_map(|p| p.boxes.iter().copied()).collect();
        let mut nodes = vyges_grt::antenna_check::build_layer_maps(&shapes, &boxes, &tech).map_err(Fail::Refused)?;
        vyges_grt::antenna_check::save_gates(&mut nodes, &pins, &tech);
        for (level, gates, diodes) in checker.check(db, &w.net, &nodes, &tech)? {
            out.push((w.net.clone(), level, gates, diodes));
        }
    }
    Ok(out)
}

/// The checker's side of a run: the layers' antenna rules, the diode repair would use, and the
/// lines written so far.
struct Checker {
    layers: Vec<vyges_grt::antenna_check::LayerAntenna>,
    dbu_per_micron: f64,
    diode_diff_area: Option<f64>,
    ratio_margin: f32,
    text: String,
}

impl Checker {
    fn read(db: &Db, with_diode: bool, ratio_margin: f32) -> Result<Self, Fail> {
        use vyges_grt::antenna_check::{AntennaRule, LayerAntenna};
        use vyges_opendb::DiffCurve;
        let layers = db
            .tech_get_layers()
            .iter()
            .map(|l| LayerAntenna {
                rule: db.layer_has_default_antenna_rule(l).then(|| AntennaRule {
                    area_factor: db.layerantenna_get_area_factor(l),
                    area_factor_diff_use_only: db.layerantenna_is_area_factor_diff_use_only(l),
                    side_area_factor: db.layerantenna_get_side_area_factor(l),
                    side_area_factor_diff_use_only: db.layerantenna_is_side_area_factor_diff_use_only(l),
                    area_minus_diff_factor: db.layerantenna_get_area_minus_diff_factor(l),
                    gate_plus_diff_factor: db.layerantenna_get_gate_plus_diff_factor(l),
                    gate_plus_diff_pwl: db.layerantenna_diff_pwl(l, DiffCurve::GatePlusDiff),
                    area_diff_reduce: db.layerantenna_diff_pwl(l, DiffCurve::AreaDiffReduce),
                    par: db.layerantenna_get_p_a_r(l),
                    psr: db.layerantenna_get_p_s_r(l),
                    car: db.layerantenna_get_c_a_r(l),
                    csr: db.layerantenna_get_c_s_r(l),
                    diff_par: db.layerantenna_diff_pwl(l, DiffCurve::Par),
                    diff_psr: db.layerantenna_diff_pwl(l, DiffCurve::Psr),
                    diff_car: db.layerantenna_diff_pwl(l, DiffCurve::Car),
                    diff_csr: db.layerantenna_diff_pwl(l, DiffCurve::Csr),
                }),
                thickness_dbu: db.layer_thickness(l).max(0) as u32,
            })
            .collect();
        // findDiodeMTerm: the first CORE ANTENNACELL master's first terminal with diffusion area.
        let mut diode_diff_area = None;
        if with_diode {
            'masters: for (m, t) in db.masters_with_types().map_err(err)? {
                if t == "CORE ANTENNACELL" {
                    for (term, _) in db.master_mterms(&m).map_err(err)? {
                        let d = db.mterm_antenna_diff_area(&m, &term);
                        if d > 0.0 {
                            diode_diff_area = Some(d);
                            break 'masters;
                        }
                    }
                }
            }
        }
        Ok(Checker { layers, dbu_per_micron: f64::from(db.tech_get_db_units_per_micron()), diode_diff_area, ratio_margin, text: String::new() })
    }

    /// `checkNet` after the layer maps: areas, PAR, CAR, then the gates.
    fn check(&mut self, db: &Db, net: &str, nodes: &vyges_grt::antenna_check::LayerNodes, tech: &vyges_grt::repair_antennas::TechLayers) -> Result<Vec<(i32, Vec<String>, i32)>, Fail> {
        use vyges_grt::antenna_check::{calculate_areas, calculate_car, calculate_par, check_gates, fmt_g17, GateFacts};
        let mut gates = Vec::new();
        for it in db.net_iterms(net) {
            let (inst, pin) = it.rsplit_once('/').ok_or_else(|| err(format!("iterm {it}")))?;
            let master = db.inst_master(inst);
            let gate_area = db.mterm_antenna_gate_area(&master, pin);
            gates.push(GateFacts {
                name: it.clone(),
                pin_name: format!("  {inst}/{pin} ({master})"),
                id: db.iterm_id(inst, pin).map_err(err)?,
                is_valid: db.mterm_get_io_type(&master, pin) == "INPUT" && gate_area > 0.0,
                gate_area,
                diff_area: db.mterm_antenna_diff_area(&master, pin),
                is_antenna_cell: db.master_get_type(&master).map_err(err)? == "CORE ANTENNACELL",
            });
        }
        let mut info = calculate_areas(nodes, &gates, &self.layers, tech, self.dbu_per_micron);
        calculate_par(&mut info, &self.layers, tech);
        calculate_car(&mut info, tech);
        let by_id: BTreeMap<u32, usize> = gates.iter().enumerate().map(|(i, g)| (g.id, i)).collect();
        for (id, per_layer) in &info {
            for (t, i) in per_layer {
                let iterms: String = i.iterms.iter().map(|&g| format!("{},", gates[g].name)).collect();
                let f = fmt_g17;
                self.text.push_str(&format!(
                    "VYGC|{net}|info|{}|{}|area={}|side={}|ga={}|da={}|par={}|psr={}|dpar={}|dpsr={}|car={}|csr={}|dcar={}|dcsr={}|iterms={iterms}\n",
                    gates[by_id[id]].name, tech.0[*t].name, f(i.area), f(i.side_area), f(i.iterm_gate_area), f(i.iterm_diff_area),
                    f(i.par), f(i.psr), f(i.diff_par), f(i.diff_psr), f(i.car), f(i.csr), f(i.diff_car), f(i.diff_csr)
                ));
            }
        }
        let (_, violations) = check_gates(&mut info, &gates, &self.layers, tech, self.diode_diff_area, self.ratio_margin);
        let mut out = Vec::new();
        for v in violations {
            let g: String = v.gates.iter().map(|&g| format!("{},", gates[g].name)).collect();
            self.text.push_str(&format!("VYGC|{net}|viol|level={}|excess={}|diodes={}|gates={g}\n", v.routing_level, fmt_g17(v.excess_ratio), v.diode_count_per_gate));
            out.push((v.routing_level, v.gates.iter().map(|&g| gates[g].name.clone()).collect(), v.diode_count_per_gate));
        }
        Ok(out)
    }
}

/// The net's instance terminals as the checker reads them (`avoidPinIntersection`, `saveGates`):
/// each named as `PinType` names it, with its ROUTING-layer boxes, placed, by tech layer index —
/// `getITerms()` → MPin → geometry order.
fn net_pin_facts(db: &Db, net: &str, tech_names: &[String]) -> Result<Vec<vyges_grt::antenna_check::PinFacts>, Fail> {
    let mut out = Vec::new();
    for it in db.net_iterms(net) {
        let (inst, pin) = it.rsplit_once('/').ok_or_else(|| err(format!("iterm {it}")))?;
        let mut boxes = Vec::new();
        for w in db.iterm_pin_boxes(inst, pin) {
            let layer = db.layer_name_by_number(w.layer);
            let t = tech_names.iter().position(|m| *m == layer).ok_or_else(|| err(format!("layer {layer}")))?;
            boxes.push((t, (w.x0, w.y0, w.x1, w.y1)));
        }
        out.push(vyges_grt::antenna_check::PinFacts { name: format!("  {inst}/{pin} ({})", db.inst_master(inst)), boxes });
    }
    Ok(out)
}

/// `dbTech`'s layer stack as the jumper graph walks it: every layer, cut layers included, with
/// `getUpperLayer` / `getLowerLayer` resolved to positions.
fn tech_layers(db: &Db) -> Result<vyges_grt::repair_antennas::TechLayers, Fail> {
    let names = db.tech_get_layers();
    let at = |n: String| names.iter().position(|m| *m == n);
    let mut layers = Vec::with_capacity(names.len());
    for n in &names {
        let is_routing = db.layer_get_type(n).map_err(err)? == "ROUTING";
        layers.push(vyges_grt::repair_antennas::TechLayer {
            name: n.clone(),
            routing_level: if is_routing { db.layer_get_routing_level(n) } else { 0 },
            is_routing,
            upper: at(db.layer_get_upper_layer(n)),
            lower: at(db.layer_get_lower_layer(n)),
        });
    }
    Ok(vyges_grt::repair_antennas::TechLayers(layers))
}

/// Captured violations: `VYGA|viol|<net>|level=<n>|excess=<r>|diodes=<n>|gates=<inst>/<pin>,…`, in
/// blocks — one per checker call, each ended by any other line.
/// One violation as the repair reads it: `(net, routing level, gate names, diodes per gate)`.
type Viol = (String, i32, Vec<String>, i32);

fn read_violation_blocks(path: &str) -> Result<Vec<Vec<Viol>>, Fail> {
    let mut blocks: Vec<Vec<Viol>> = Vec::new();
    let mut open = false;
    for line in std::fs::read_to_string(path).map_err(err)?.lines() {
        let Some(rest) = line.strip_prefix("VYGA|viol|") else {
            open = false;
            continue;
        };
        let f: Vec<&str> = rest.split('|').collect();
        let bad = || err(format!("{path}: bad violation line {line:?}"));
        let level = f.get(1).and_then(|v| v.strip_prefix("level=")).and_then(|v| v.parse().ok()).ok_or_else(bad)?;
        let gates = f.get(4).and_then(|v| v.strip_prefix("gates=")).ok_or_else(bad)?.split(',').filter(|g| !g.is_empty()).map(str::to_string).collect();
        if !open {
            blocks.push(Vec::new());
            open = true;
        }
        let diodes = f.get(3).and_then(|v| v.strip_prefix("diodes=")).and_then(|v| v.parse().ok()).ok_or_else(bad)?;
        blocks.last_mut().expect("a block").push((f[0].to_string(), level, gates, diodes));
    }
    Ok(blocks)
}

/// `repair_antennas` as `GlobalRouter::repairAntennas` runs it, with the CHECKER's answers captured.
///
/// One iteration: the violations of the first check → `jumperInsertion` (unless `-diode_only`) →
/// `saveGuides` over the nets that got jumpers → the second check. Diode insertion, and a second
/// iteration with violations left, are refused.
#[allow(clippy::too_many_arguments)]
fn repair_antennas(db: &mut Db, step: &Value, state: &mut vyges_grt::global_route::AfterRoute, total_overflow: i32, db_guides: &BTreeMap<String, Vec<vyges_grt::Guide>>, from_db: bool, padding: (i32, i32), log: &mut Vec<String>) -> Result<Vec<vyges_grt::NetGuides>, Fail> {
    use vyges_grt::repair_antennas::{jumper_insertion, AntViolation, FastRouteJumpers, GatePin, JumperInputs, NetViolations};
    let jumper_only = step["jumper_only"].as_bool().unwrap_or(false);
    let diode_only = step["diode_only"].as_bool().unwrap_or(false);
    let iterations = step["iterations"].as_i64().unwrap_or(1);
    // The Tcl sets the router's allow_congestion from THIS command's flag.
    let allow_congestion = step["allow_congestion"].as_bool().unwrap_or(false);
    // findDiodeMTerm: the first CORE ANTENNACELL master, in library order, with a terminal that has
    // diffusion area. Without one the command warns (GRT-246) and does nothing at all.
    let mut has_diode = false;
    for (m, t) in db.masters_with_types().map_err(err)? {
        // ⚠️ `dbMasterType::getString` spells it with a SPACE.
        if t == "CORE ANTENNACELL" && db.master_mterms(&m).map_err(err)?.iter().any(|(term, _)| db.mterm_antenna_diff_area(&m, term) > 0.0) {
            has_diode = true;
            break;
        }
    }
    if !has_diode {
        log.push("GRT-0246: No diode with LEF class CORE ANTENNACELL found.".into());
        return Ok(Vec::new());
    }
    let ratio_margin = step["ratio_margin"].as_f64().unwrap_or(0.0) as f32;
    // The checker's answers: ours, or — with "violations" — the reference's, captured.
    let oracle = step["violations"].as_str().map(read_violation_blocks).transpose()?;
    if oracle.is_none() && from_db {
        return Err(Fail::Refused("antenna checking a database: its terminals may carry access points, which are not modelled".into()));
    }
    // antenna_violations_ is a PtrMap: by net ID, which is the block's net order.
    let order = db.net_names();
    let mut first = match &oracle {
        Some(blocks) => blocks.first().cloned().unwrap_or_default(),
        None => check_design(db, db_guides, true, ratio_margin)?,
    };
    first.sort_by_key(|(n, ..)| order.iter().position(|m| m == n).unwrap_or(usize::MAX));
    let tech = tech_layers(db)?;
    let number_to_index: BTreeMap<i64, usize> = tech.0.iter().enumerate().map(|(i, l)| (i64::from(db.layer_get_number(&l.name)), i)).collect();
    let mut by_net: Vec<NetViolations> = Vec::new();
    for (net, level, gates, _) in first.clone() {
        let mut pins = Vec::new();
        for g in gates {
            let (inst, pin) = g.rsplit_once('/').ok_or_else(|| err(format!("gate {g}: expected <inst>/<pin>")))?;
            let master = db.inst_master(inst);
            if db.master_get_type(&master).map_err(err)?.starts_with("BLOCK") {
                return Err(Fail::Refused(format!("gate {g} is on a block: getInstRect's pin-box rule is not modelled")));
            }
            let b = db.inst_bbox(inst).map_err(err)?;
            let pin_boxes = db
                .iterm_pin_boxes(inst, pin)
                .into_iter()
                .map(|w| Ok((*number_to_index.get(&w.layer).ok_or_else(|| err(format!("layer number {}", w.layer)))?, vyges_grt::Rect::new(w.x0, w.y0, w.x1, w.y1))))
                .collect::<Result<Vec<_>, Fail>>()?;
            pins.push(GatePin { name: g.clone(), inst_rect: vyges_grt::Rect::new(b[0], b[1], b[2], b[3]), pin_boxes });
        }
        let v = AntViolation { routing_level: level, gates: pins };
        match by_net.last_mut() {
            Some(nv) if nv.net == net => nv.violations.push(v),
            _ => by_net.push(NetViolations { net, violations: vec![v] }),
        }
    }
    log.push(format!("GRT-0012: Found {} antenna violations.", by_net.len()));
    let mut saved = Vec::new();
    // hasNewViolations: every net is new on the first iteration.
    if !diode_only && !by_net.is_empty() {
        let g3 = state.final_3d.as_mut().ok_or_else(|| Fail::Refused("jumper insertion without the router's 3D edges".into()))?;
        let mut routes: BTreeMap<String, Vec<vyges_grt::GSegment>> = state.net_routes.iter().map(|n| (n.name.clone(), n.segments.clone())).collect();
        let inp = JumperInputs { tech: &tech, grid: state.jumper_grid, max_routing_layer: state.max_routing_layer };
        let mut router = FastRouteJumpers { g3, grid: state.jumper_grid, layer_edge_cost: &state.layer_edge_cost };
        let mut trace = step["trace"].as_str().map(|_| Vec::new());
        let res = jumper_insertion(&by_net, &mut routes, &inp, &mut router, trace.as_mut()).map_err(Fail::Refused)?;
        if let (Some(p), Some(t)) = (step["trace"].as_str(), &trace) {
            std::fs::write(p, t.iter().map(|l| format!("VYGJ|{l}\n")).collect::<String>()).map_err(err)?;
        }
        log.push(format!("GRT-0302: Inserted {} jumpers for {} nets.", res.total_jumpers, res.net_with_jumpers));
        // saveGuides(nets_with_jumpers), with the congestion mark as the command left it.
        let mut opts = state.save_options;
        opts.guide_is_congested = total_overflow > 0 && !allow_congestion;
        let mut modified = Vec::new();
        for name in &res.modified_nets {
            let nr = state.net_routes.iter_mut().find(|n| &n.name == name).ok_or_else(|| err(format!("net {name} has no route")))?;
            nr.segments = routes[name].clone();
            modified.push(nr.clone());
        }
        saved = vyges_grt::save_guides(&modified, &state.jumper_grid.grid, &opts).map_err(|e| err(format!("{e:?}")))?;
    }
    // antenna_violations_ as the diodes see it: the first check's when no jumper pass ran, else the
    // second check's, on the guides the jumpers left.
    let jumpers_ran = !diode_only && !by_net.is_empty();
    let mut second: Vec<Viol> = match &oracle {
        Some(blocks) => if jumpers_ran { blocks.get(1) } else { blocks.first() }.cloned().unwrap_or_default(),
        None if !jumpers_ran => first,
        None => {
            // The second check, on the guides as the jumpers left them.
            let mut after = db_guides.clone();
            for ng in &saved {
                after.insert(ng.net.clone(), ng.guides.clone());
            }
            check_design(db, &after, true, ratio_margin)?
        }
    };
    second.sort_by_key(|(n, ..)| order.iter().position(|m| m == n).unwrap_or(usize::MAX));
    if !second.is_empty() && !jumper_only {
        insert_diodes(db, &second, padding, step["diode_trace"].as_str(), log)?;
        return Err(Fail::Refused(format!("detailed placement after diode insertion is not modelled ({} nets got diodes)", second.iter().map(|v| &v.0).collect::<std::collections::BTreeSet<_>>().len())));
    }
    if !second.is_empty() && iterations > 1 {
        return Err(Fail::Refused("a second repair iteration is not modelled".into()));
    }
    Ok(saved)
}

/// `RepairAntennas::repairAntennas` up to `legalizePlacedCells`: the violating gates made FIRM, the
/// fixed instances and hard blockages collected, then per violation, per gate, per diode needed, a
/// diode created beside the gate (`insertDiode`), placed, marked, connected — and itself added to
/// the fixed set the next one avoids.
fn insert_diodes(db: &mut Db, violations: &[Viol], padding: (i32, i32), trace: Option<&str>, log: &mut Vec<String>) -> Result<(), Fail> {
    use vyges_grt::repair_antennas::{place_diode, DiodeFloor, DiodeGate, DiodeRow};
    let r = |v: &[i32]| vyges_grt::Rect { x_min: v[0], y_min: v[1], x_max: v[2], y_max: v[3] };
    // findDiodeMTerm
    let mut diode = None;
    'm: for (m, t) in db.masters_with_types().map_err(err)? {
        if t == "CORE ANTENNACELL" {
            for (term, _) in db.master_mterms(&m).map_err(err)? {
                if db.mterm_antenna_diff_area(&m, &term) > 0.0 {
                    diode = Some((m.clone(), term));
                    break 'm;
                }
            }
        }
    }
    let (diode_master, diode_term) = diode.ok_or_else(|| err("no diode master"))?;
    let mut rows = Vec::new();
    let mut site_width = -1;
    for i in 0..db.num_rows().map_err(err)? {
        let Some((bbox, site, orient)) = db.nth_row(i).map_err(err)? else { continue };
        if db.site_get_class(&site).map_err(err)? != "PAD" {
            let w = db.site_get_width(&site);
            if site_width == -1 {
                site_width = w;
            } else if site_width != w {
                log.push("GRT-0027: Design has rows with different site widths.".into());
            }
        }
        rows.push(DiodeRow { bbox: r(&bbox), orient });
    }
    let floor = DiodeFloor {
        rows,
        core: vyges_grt::Rect { x_min: db.block_get_core_area_x_min(), y_min: db.block_get_core_area_y_min(), x_max: db.block_get_core_area_x_max(), y_max: db.block_get_core_area_y_max() },
        site_width,
        pad_left: padding.0,
        pad_right: padding.1,
        diode_width: db.master_get_width(&diode_master) as i32,
        diode_height: db.master_get_height(&diode_master) as i32,
    };
    let is_block = |db: &Db, inst: &str| -> Result<bool, Fail> { Ok(db.master_get_type(&db.inst_master(inst)).map_err(err)?.starts_with("BLOCK")) };
    // setDiodesAndGatesPlacementStatus(FIRM)
    for (_, _, gates, _) in violations {
        for g in gates {
            let inst = g.rsplit_once('/').map(|p| p.0).unwrap_or(g);
            if !is_block(db, inst)? {
                db.inst_set_placement_status(inst, "FIRM").map_err(err)?;
            }
        }
    }
    // getFixedInstances, getPlacementBlockages
    let mut fixed = Vec::new();
    for inst in db.inst_names() {
        let st = db.inst_get_placement_status(&inst);
        if st == "FIRM" || st == "LOCKED" {
            fixed.push(r(&db.inst_bbox(&inst).map_err(err)?));
        }
    }
    for (i, b) in db.blockage_boxes().map_err(err)?.into_iter().enumerate() {
        if !db.blockage_is_soft(i) {
            fixed.push(vyges_grt::Rect { x_min: b.0, y_min: b.1, x_max: b.2, y_max: b.3 });
        }
    }
    let mut text = String::new();
    for (i, f) in fixed.iter().enumerate() {
        text.push_str(&format!("VYGD|fixed|{i}|{},{},{},{}\n", f.x_min, f.y_min, f.x_max, f.y_max));
    }
    let mut index = 1;
    let names: std::collections::BTreeSet<String> = db.inst_names().into_iter().collect();
    while names.contains(&format!("ANTENNA_{index}")) {
        index += 1;
    }
    let mut failures = false;
    let tech = tech_layers(db)?;
    let dirs: BTreeMap<String, String> = db.layers_with_direction().map_err(err)?.into_iter().collect();
    for (net, level, gates, diodes) in violations {
        if *diodes <= 0 {
            failures = true;
            continue;
        }
        let layer = tech.find_routing_layer(*level).ok_or_else(|| Fail::Refused(format!("a violation on routing level {level}: the reference dereferences a null layer")))?;
        let place_vertically = dirs.get(&tech.0[layer].name).is_some_and(|d| d == "VERTICAL");
        for g in gates {
            let inst = g.rsplit_once('/').map(|p| p.0).unwrap_or(g);
            let master = db.inst_master(inst);
            let ty = db.master_get_type(&master).map_err(err)?;
            if ty.starts_with("BLOCK") {
                return Err(Fail::Refused(format!("gate {g} is on a block: getInstRect's pin-box rule is not modelled")));
            }
            let gate = DiodeGate { rect: r(&db.inst_bbox(inst).map_err(err)?), orient: db.inst_get_orient(inst), block_or_pad: ty.starts_with("PAD"), is_block: false };
            for _ in 0..*diodes {
                let name = format!("ANTENNA_{index}");
                index += 1;
                let p = place_diode(&gate, place_vertically, &floor, &fixed);
                db.create_inst(&diode_master, &name).map_err(err)?;
                db.set_inst_orient(&name, &p.orient).map_err(err)?;
                db.set_inst_location(&name, p.x, p.y).map_err(err)?;
                db.inst_set_placement_status(&name, p.status).map_err(err)?;
                db.connect(&name, &diode_term, net).map_err(err)?;
                fixed.push(r(&db.inst_bbox(&name).map_err(err)?));
                for (k, (x, y, o, legal)) in p.tries.iter().enumerate() {
                    text.push_str(&format!("VYGD|try|{name}|{g}|{k}|{x},{y}|{o}|legal={}\n", i32::from(*legal)));
                }
                text.push_str(&format!("VYGD|pad|{name}|{}|{}\n", padding.0, padding.1));
                text.push_str(&format!(
                    "VYGD|diode|{name}|{diode_master}|{},{}|{}|legal={}|inrow={}|status={}|net={net}|gate={g}\n",
                    p.x, p.y, p.orient, i32::from(p.legal), i32::from(p.in_row), p.status
                ));
            }
        }
    }
    if failures {
        log.push("GRT-0243: Unable to repair antennas on net with diodes.".into());
    }
    if let Some(path) = trace {
        std::fs::write(path, text).map_err(err)?;
    }
    Ok(())
}

fn run(job: &Value) -> Result<Value, Fail> {
    let mut db = Db::new();
    // A design from a database may carry pin access points; one from DEF carries none.
    let from_db = job["db"].is_string();
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
    // The same guides whole — via layer and pin flags included — for antenna checking.
    let mut db_guides: BTreeMap<String, Vec<vyges_grt::Guide>> = BTreeMap::new();
    let mut parasitics: BTreeMap<String, vyges_grt::parasitics::Network> = BTreeMap::new();
    let mut parasitic_pins: BTreeMap<String, Vec<vyges_grt::parasitics::PinGridLocation>> = BTreeMap::new();
    let mut routed_parasitics: BTreeMap<String, vyges_grt::parasitics::Network> = BTreeMap::new();
    let mut planar_routes: BTreeMap<String, Vec<vyges_grt::parasitics::Segment>> = BTreeMap::new();
    let mut snapshot_edges: BTreeMap<String, Vec<vyges_grt::global_route::SnapshotEdge>> = BTreeMap::new();
    let mut calls = Vec::new();
    let mut log = Vec::new();
    // The router as a later command finds it: the last global_route's state.
    let mut after: Option<(vyges_grt::global_route::AfterRoute, i32)> = None;
    // set_placement_padding -global: opendp's padding, in sites, left and right.
    let mut padding = (0, 0);
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
                    db_guides.insert(ng.net.clone(), ng.guides.clone());
                    guides.insert(ng.net.clone(), ng.guides.iter().map(|g| (g.box_.x_min, g.box_.y_min, g.box_.x_max, g.box_.y_max, res.layer_names[&g.layer].clone())).collect());
                }
                calls.push(json!({ "nets": res.guides.len(), "total_overflow": res.total_overflow, "congested": res.guide_is_congested, "clock_nets": res.clock_nets }));
                after = Some((res.after.clone(), res.total_overflow));
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
            // ant::WireBuilder::makeNetWiresFromGuides over the block's nets, in block order —
            // `VYGA|wire|<net>|<x1>,<y1>,<l1>|<x2>,<y2>,<l2>` per segment, in creation order.
            // With "encoder": that path gets the encoder calls, `VYGW|<net>|…` as the reference's
            // stage-2 trace prints them (less its pass number).
            "antenna_wires" => {
                if from_db {
                    return Err(Fail::Refused("antenna wires from a database: its terminals may carry access points, which are not modelled".into()));
                }
                let path = step["path"].as_str().ok_or_else(|| err("path"))?;
                let (nets, tech, vias) = ant_nets(&db, &db_guides)?;
                let wires = vyges_grt::wire_builder::make_net_wires_from_guides(&nets, db.block_get_g_cell_tile_size(), &tech).map_err(|e| Fail::Refused(format!("{e:?}")))?;
                let mut text = String::new();
                for w in &wires {
                    for sg in &w.route {
                        text.push_str(&format!("VYGA|wire|{}|{},{},{}|{},{},{}\n", w.net, sg.pt1.x, sg.pt1.y, sg.pt1.layer, sg.pt2.x, sg.pt2.y, sg.pt2.layer));
                    }
                }
                std::fs::write(path, text).map_err(err)?;
                if let Some(enc) = step["encoder"].as_str() {
                    let name = |l: i32| db_layer_name(&db, l);
                    let mut text = String::new();
                    for w in &wires {
                        let net = nets.iter().find(|n| n.name == w.net).expect("a built net");
                        for (pt, pins) in &w.pt_pins {
                            let it: String = pins.iterms.iter().map(|&i| format!("{},", net.iterms[i].name)).collect();
                            let bt: String = pins.bterms.iter().map(|&b| format!("{},", net.bterms[b].name)).collect();
                            text.push_str(&format!("VYGW|{}|ptpin|{},{},{}|iterms={it}|bterms={bt}\n", w.net, pt.x, pt.y, pt.layer));
                        }
                        for op in &w.ops {
                            text.push_str(&match op {
                                vyges_grt::wire_builder::WireOp::Path(l) => format!("VYGW|{}|path|{}\n", w.net, name(*l)),
                                vyges_grt::wire_builder::WireOp::Point(x, y) => format!("VYGW|{}|point|{x},{y}\n", w.net),
                                vyges_grt::wire_builder::WireOp::Via(l) => format!("VYGW|{}|via|{}\n", w.net, vias.get(l).cloned().unwrap_or_else(|| "(null)".into())),
                            });
                        }
                    }
                    std::fs::write(enc, text).map_err(err)?;
                }
                // With "shapes": what the database decodes from that wire, `VYGC|<net>|shape|…` and
                // `…|vbox|…` as the reference's stage-2 trace prints them.
                if let Some(out) = step["shapes"].as_str() {
                    let codec = DbCodec::read(&db, &vias)?;
                    let mut text = String::new();
                    for w in &wires {
                        let ops = vyges_grt::wire_codec::encode(&w.ops, &codec).map_err(|e| Fail::Refused(format!("net {}: {e}", w.net)))?;
                        for sh in vyges_grt::wire_codec::decode(&ops, &codec) {
                            let r = |r: &vyges_grt::Rect| format!("{},{},{},{}", r.x_min, r.y_min, r.x_max, r.y_max);
                            match sh {
                                vyges_grt::wire_codec::Shape::Segment { level, rect } => text.push_str(&format!("VYGC|{}|shape|{}|{}|via=-\n", w.net, db_layer_name(&db, level), r(&rect))),
                                vyges_grt::wire_codec::Shape::Via { name, rect, boxes } => {
                                    text.push_str(&format!("VYGC|{}|shape|-|{}|via={name}\n", w.net, r(&rect)));
                                    for (t, b) in boxes {
                                        text.push_str(&format!("VYGC|{}|vbox|{}|{}\n", w.net, codec.tech_names[t], r(&b)));
                                    }
                                }
                            }
                        }
                    }
                    std::fs::write(out, text).map_err(err)?;
                }
                // With "nodes": the checker's polygons per layer (`buildLayerMaps`), `VYGC|<net>|node|…`.
                if step["nodes"].is_string() || step["checker"].is_string() {
                    let out = step["nodes"].as_str();
                    // With "checker": the ratios and the verdict, `VYGC|<net>|info|…` and `…|viol|…`.
                    // "checker_diode" answers as repair_antennas does (with its diode), otherwise as
                    // check_antennas (none).
                    let mut checker = step["checker"].as_str().map(|_| Checker::read(&db, step["checker_diode"].as_bool().unwrap_or(false), step["ratio_margin"].as_f64().unwrap_or(0.0) as f32)).transpose()?;
                    let codec = DbCodec::read(&db, &vias)?;
                    let tech = tech_layers(&db)?;
                    let mut text = String::new();
                    for w in &wires {
                        let ops = vyges_grt::wire_codec::encode(&w.ops, &codec).map_err(|e| Fail::Refused(format!("net {}: {e}", w.net)))?;
                        let shapes = vyges_grt::wire_codec::decode(&ops, &codec);
                        let pins = net_pin_facts(&db, &w.net, &codec.tech_names)?;
                        let boxes: Vec<(usize, vyges_grt::polygon90::R)> = pins.iter().flat_map(|p| p.boxes.iter().copied()).collect();
                        let mut nodes = vyges_grt::antenna_check::build_layer_maps(&shapes, &boxes, &tech).map_err(Fail::Refused)?;
                        vyges_grt::antenna_check::save_gates(&mut nodes, &pins, &tech);
                        if let Some(c) = checker.as_mut() {
                            c.check(&db, &w.net, &nodes, &tech)?;
                        }
                        for (t, list) in &nodes {
                            for n in list {
                                let pts: String = n.pol.iter().map(|(x, y)| format!("{x},{y};")).collect();
                                let low: String = n.low_adj.iter().map(|l| format!("{l},")).collect();
                                text.push_str(&format!("VYGC|{}|node|{}|{}|{pts}|low={low}\n", w.net, tech.0[*t].name, n.id));
                                let gates: String = n.gates.iter().map(|g| format!("{g};")).collect();
                                text.push_str(&format!("VYGC|{}|gates|{}|{gates}\n", w.net, n.id));
                            }
                        }
                    }
                    if let Some(out) = out {
                        std::fs::write(out, text).map_err(err)?;
                    }
                    if let (Some(path), Some(c)) = (step["checker"].as_str(), checker) {
                        std::fs::write(path, c.text).map_err(err)?;
                    }
                }
            }
            "placement_padding" => {
                padding = (step["left"].as_i64().unwrap_or(0) as i32, step["right"].as_i64().unwrap_or(0) as i32);
            }
            "repair_antennas" => {
                // ⚠️ Not GRT-45: the reference also repairs from routes a database brought with it
                // (`have_routes` after read_db). That path is not modelled.
                let (state, total_overflow) = after.as_mut().ok_or_else(|| Fail::Refused("repair_antennas without a global_route in this session: routes read from a database are not modelled".into()))?;
                let repaired = repair_antennas(&mut db, step, state, *total_overflow, &db_guides, from_db, padding, &mut log)?;
                for ng in repaired {
                    guides.insert(ng.net.clone(), ng.guides.iter().map(|g| (g.box_.x_min, g.box_.y_min, g.box_.x_max, g.box_.y_max, db_layer_name(&db, g.layer))).collect());
                    db_guides.insert(ng.net.clone(), ng.guides);
                }
            }
            "write_guides" => write_guides(step["path"].as_str().ok_or_else(|| err("path"))?, &guides)?,
            other => return Err(Fail::Refused(format!("step {other:?} is not modelled"))),
        }
    }
    Ok(json!({ "status": "routed", "global_route": calls, "log": log }))
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
