// SPDX-License-Identifier: Apache-2.0
//! The global router's side of `global_route -use_cugr` (`GlobalRouter::initCUGR`), over the
//! database.

use std::collections::BTreeSet;

use vyges_opendb::Db;

use super::pattern_route::SteinerBuilder;
use super::read::read_design_facts;
use super::{init, Cugr, StageError};
use crate::driver::get_min_max_layer;
use crate::global_route::RouteOptions;
use crate::init::{is_non_leaf_clock, ITermClockFacts};
use crate::pins::{find_nets, NetCandidate};
use crate::read::{read_nets, read_tech};

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// The router after `initCUGR`, with what its callers report.
pub struct CugrInit {
    pub cugr: Cugr,
    pub min_routing_layer: i32,
    pub max_routing_layer: i32,
    pub clock_nets: BTreeSet<String>,
}

/// `initCUGR`'s sequence, as far as it decides CUGR's model.
///
/// 1. `getMinMaxLayer` — an unset max routing layer is computed and WRITTEN BACK to the block.
/// 2. `computeUserGlobalAdjustments` — the global adjustment is written INTO each tech layer in
///    range that has none of its own; `MetalLayer` reads it back from there, and it persists.
/// 3. `initNets(true)` — with a liberty library the timer's clock nets are re-typed CLOCK in the
///    database; `findClockNets` then takes the admitted nets that are non-leaf clocks.
/// 4. `CUGR::init`.
///
/// The grid, track and net-degree reports `initCUGR` also makes are not modelled here.
pub fn init_cugr(db: &mut Db, opts: &RouteOptions) -> Res<CugrInit> {
    let tech = read_tech(db)?;
    let has_track_grid: Vec<bool> = tech.routing_layers.iter().map(|l| l.has_track_grid).collect();
    let (block_max, min, max) = get_min_max_layer(tech.block_max_routing_layer, &has_track_grid, tech.block_min_routing_layer, tech.min_layer_for_clock, tech.max_layer_for_clock)
        .ok_or("GRT-0701: the lowest routing layer has no track grid")?;
    if block_max != tech.block_max_routing_layer {
        db.block_set_max_routing_layer(block_max)?;
    }
    if opts.adjustment != 0.0 {
        for l in min..=max {
            let name = tech.routing_layers.iter().find(|r| r.routing_level == l).map(|r| r.name.clone()).ok_or("a routing level with no layer")?;
            if db.layer_get_layer_adjustment(&name) == 0.0 {
                db.layer_set_layer_adjustment(&name, opts.adjustment)?;
            }
        }
    }
    if let Some(lib) = &opts.liberty {
        for net in crate::clk_network::find_clk_nets(db, lib, &opts.clock_sources)? {
            db.net_set_sig_type(&net, "CLOCK")?;
        }
    }
    let db_nets = read_nets(db);
    let candidates: Vec<NetCandidate> = db_nets
        .iter()
        .map(|n| NetCandidate {
            name: n.name.clone(),
            is_supply: n.is_supply(),
            is_special: n.is_special,
            term_count: n.term_count,
            has_special_wires: n.has_special_wires,
            connected_by_abutment: n.connected_by_abutment,
        })
        .collect();
    let mut log = Vec::new();
    let no_liberty = ITermClockFacts { has_liberty_port: false, is_reg_clk: false, cell_is_pad: false };
    let mut clock_nets = BTreeSet::new();
    for i in find_nets(&candidates, opts.skip_large_fanout, &mut log) {
        let n = &db_nets[i];
        let facts: Vec<ITermClockFacts> = match &opts.liberty {
            Some(lib) if n.sig_type == "CLOCK" => n.iterms.iter().map(|(inst, mterm)| lib.iterm_facts(&db.inst_master(inst), mterm)).collect(),
            _ => vec![no_liberty; n.iterms.len()],
        };
        if is_non_leaf_clock(n.sig_type == "CLOCK", &facts) {
            clock_nets.insert(n.name.clone());
        }
    }
    let (facts, drivers) = read_design_facts(db, &clock_nets)?;
    let cugr = init(&facts, &drivers, min, max).map_err(|e| format!("{e:?}"))?;
    Ok(CugrInit { cugr, min_routing_layer: min, max_routing_layer: max, clock_nets })
}

/// `computeNdrCosts(net)`: 1 on every layer without a rule, from the net's non-default rule.
fn net_ndr_costs(db: &Db, net: &str, num_layers: usize) -> Res<Vec<f64>> {
    let ndr = db.net_get_non_default_rule(net);
    if ndr.is_empty() {
        return Ok(vec![1.0; num_layers]);
    }
    let rules: Vec<super::NdrRuleFacts> = db
        .ndr_layer_rules(&ndr)?
        .into_iter()
        .map(|(layer, width, spacing)| super::NdrRuleFacts {
            is_routing: db.layer_get_type(&layer).map(|t| t == "ROUTING").unwrap_or(false),
            routing_level: db.layer_get_routing_level(&layer),
            width,
            spacing,
            default_width: db.layer_get_width(&layer) as i32,
            default_pitch: db.layer_get_pitch(&layer),
        })
        .collect();
    Ok(super::ndr_costs(num_layers, &rules))
}

/// Each net's costs as `CUGR::init` leaves them, whichever command set CUGR up: its rule's
/// factors (`computeNdrCosts`), the constant stage-1 slack, and its Steiner alpha. Returns the
/// alphas and the slack.
///
/// Upstream rule (`setInitialNetSlacks` / `updateCriticalNets`): slacks are set only with a
/// non-zero critical-net percentage (forced to 0 without a liberty library). With no clock every
/// net is unconstrained (`1e+30`, and none is ever demoted: the percentile threshold is `1e+30`
/// too).
fn init_net_costs(ci: &mut CugrInit, db: &Db, opts: &RouteOptions) -> Res<(Vec<f32>, f32)> {
    let num_layers = ci.cugr.grid.num_layers;
    for n in &mut ci.cugr.nets {
        n.ndr_costs = net_ndr_costs(db, &n.name, num_layers)?;
    }
    let constant = if opts.liberty.is_none() || opts.critical_nets_percentage == 0.0 { 0.0 } else { 1.0e30f32 };
    let mut alphas = Vec::with_capacity(ci.cugr.nets.len());
    for n in &mut ci.cugr.nets {
        n.slack = constant;
        alphas.push(crate::global_route::net_steiner_alpha(db, opts, &n.name)?);
    }
    Ok((alphas, constant))
}

/// What `route_cugr` leaves.
pub struct CugrRoute {
    pub init: CugrInit,
    /// Each net's Steiner alpha, and the constant stage-1 slack (none where captured).
    pub alphas: Vec<f32>,
    pub slack: f32,
    /// The stages' log lines (GRT-0274, GRT-0277, GRT-0305, GRT-0118).
    pub log: Vec<String>,
}

/// `CUGR::route(false)`: `initCUGR`, then stage 1, then — each only while congested nets remain —
/// stage 3 (detours), stage 4 (maze), stage 5 (rip-up and re-route). Stage 2 (resistance-aware)
/// needs `-resistance_aware`, refused before here.
///
pub fn route_cugr(db: &mut Db, opts: &RouteOptions, call: usize, stt: SteinerBuilder<'_>, trace: Option<&mut Vec<String>>) -> Res<CugrRoute> {
    if db.block_access_point_count()? > 0 {
        return Err("cugr: pin access points in the database — findODBAccessPoints is not modelled".into());
    }
    let timed = opts.liberty.is_some() && !opts.clock_sources.is_empty();
    let oracle = opts.cugr_slacks.as_ref().map(|calls| calls.get(call));
    if timed && oracle.is_none() {
        return Err("cugr: a clock with a liberty library — the critical-net slacks are not modelled".into());
    }
    let mut ci = init_cugr(db, opts)?;
    let (alphas, constant) = init_net_costs(&mut ci, db, opts)?;
    // With a clock the slacks are the timer's, refreshed and demoted before each later sort, and
    // come from the capture — per sort, at the sort (`sort_net_indices`).
    if timed && opts.critical_nets_percentage != 0.0 {
        match oracle {
            Some(Some(sorts)) => ci.cugr.sort_slacks = Some(sorts.clone()),
            _ => return Err(format!("cugr: no captured slacks for CUGR call {call} — not modelled").into()),
        }
    }
    let mut log = Vec::new();
    let mut trace = trace;
    // The model's records are those of `CUGR::init` — before any stage changes a net (RRR's
    // soft-NDR demotion resets its factors).
    if let Some(t) = trace.as_deref_mut() {
        t.extend(super::trace::model(&ci.cugr, ci.min_routing_layer, ci.max_routing_layer, ci.clock_nets.len()));
    }
    let fail = |e: StageError| format!("cugr: {e:?}");
    if let Some(t) = trace.as_deref_mut() {
        t.push("VYGC|route|0".into());
    }
    let mut all: Vec<usize> = (0..ci.cugr.nets.len()).collect();
    ci.cugr.pattern_route(&mut all, &alphas, stt, &mut log, trace.as_deref_mut()).map_err(fail)?;
    // updateCongestedNets after stage 1 — the trace tags it stage 2 (patternRouteResAware sets its
    // tag before returning).
    let mut nets = ci.cugr.congested_nets(2, trace.as_deref_mut());
    ci.cugr.pattern_route_with_detours(&mut nets, &alphas, stt, &mut log, trace.as_deref_mut()).map_err(fail)?;
    let mut nets = ci.cugr.congested_nets(3, trace.as_deref_mut());
    ci.cugr.maze_route(&mut nets, 4, &mut log, trace.as_deref_mut()).map_err(fail)?;
    let mut nets = ci.cugr.congested_nets(4, trace.as_deref_mut());
    ci.cugr.iterative_rrr(&mut nets, opts.congestion_iterations, &mut log, trace.as_deref_mut()).map_err(fail)?;
    if let Some(t) = trace.as_deref_mut() {
        t.push("VYGC|routed".into());
    }
    Ok(CugrRoute { init: ci, log, alphas, slack: constant })
}

/// What a CUGR `global_route` saves, and the global router's state a later command reads.
pub struct CugrGuides {
    pub guides: Vec<crate::NetGuides>,
    /// Routing level → layer name, as the guide writer names layers.
    pub layer_names: std::collections::BTreeMap<i32, String>,
    /// `routes_`: every net's route after `addRemainingGuides` (what antenna repair edits).
    pub routes: std::collections::BTreeMap<String, Vec<crate::GSegment>>,
    /// Each net as `saveGuides` reads it — its pins AFTER `updatePinAccessPoints`.
    pub net_routes: Vec<crate::NetRoute>,
    pub jumper_grid: crate::repair_antennas::JumperGrid,
    pub max_routing_layer: i32,
    pub save: crate::SaveOptions,
    /// GlobalRouter's pins per net as they stand (names kept: `updatePinAccessPoints` finds each
    /// pin's CUGR access point by terminal).
    pub pins: std::collections::BTreeMap<String, Vec<crate::pins::NetPin>>,
    /// What an incremental route re-reads: the clock nets, the routing range, the RRR budget of
    /// the last `global_route`, each net's Steiner alpha, and the constant stage-1 slack.
    pub clock_nets: BTreeSet<String>,
    pub min_routing_layer: i32,
    pub iterations: i32,
    pub alphas: Vec<f32>,
    pub slack: f32,
}

/// The global router's tail after `CUGR::route`: `findRoutingCugr` (each net's route from CUGR,
/// then `updatePinAccessPoints`), `addRemainingGuides`, then `saveGuides`.
///
/// Upstream rules: the pins are GlobalRouter's own (`initNets(true)`), found with no FastRoute
/// capacities. A pin CUGR chose a cell for moves to the grid position of that cell's LOW corner
/// and — unless it sits above the max routing layer — to the top of its layer interval. A net CUGR
/// exports nothing for (local, or under two pins) is left to `addRemainingGuides`. Unlike
/// FastRoute's tail there is no `connectPadPins` and no `mergeSegments`, and a CUGR guide is never
/// marked congested. `isLocal` is asked of the pins AFTER they moved.
pub fn cugr_guides(db: &mut Db, opts: &RouteOptions, cugr: &Cugr, clock_nets: &BTreeSet<String>, alphas: &[f32], slack: f32, log: &mut Vec<String>) -> Res<CugrGuides> {
    use crate::findrouting::{add_remaining_guides, RemainingNet};
    if opts.nets_to_route.is_some() {
        return Err("cugr: set_nets_to_route (incremental routing) is not modelled".into());
    }
    // initRoutingGrid: layers, tracks and the core grid the guide boxes are cut on.
    let t = crate::global_route::setup_tech(db, opts)?;
    let db: &Db = db;
    let all = read_nets(db);
    let db_nets: Vec<&crate::read::NetFacts> = all.iter().collect();
    let candidates: Vec<NetCandidate> = db_nets
        .iter()
        .map(|n| NetCandidate {
            name: n.name.clone(),
            is_supply: n.is_supply(),
            is_special: n.is_special,
            term_count: n.term_count,
            has_special_wires: n.has_special_wires,
            connected_by_abutment: n.connected_by_abutment,
        })
        .collect();
    let mut nets = crate::global_route::discover_net_pins(db, &t, &db_nets, &candidates, opts, None, log)?;
    let pin_grid = crate::pins::PinGrid {
        die: t.core.area,
        tile_size: t.core.tile_size,
        x_grids: t.core.x_grids,
        y_grids: t.core.y_grids,
        directions: t.tech.routing_layers.iter().map(|l| (l.routing_level, l.direction)).collect(),
        tracks: t.tracks.iter().map(|r| (r.layer_index, (r.location, r.track_pitch))).collect(),
        use_cugr: true,
    };
    let by_name: std::collections::HashMap<&str, usize> = cugr.nets.iter().enumerate().map(|(k, n)| (n.name.as_str(), k)).collect();
    // findRoutingCugr
    let mut routes: std::collections::BTreeMap<String, Vec<crate::GSegment>> = std::collections::BTreeMap::new();
    for (n, pins) in &mut nets {
        let Some(&k) = by_name.get(n.name.as_str()) else { continue };
        let net = &cugr.nets[k];
        let route = cugr.net_route(net);
        if !route.is_empty() {
            routes.insert(n.name.clone(), route);
        }
        for (pin, _) in pins.iter_mut() {
            update_pin_access_point(cugr, net, pin, &pin_grid, t.max_routing_layer);
        }
    }
    let grid_pins = |pins: &[(crate::pins::NetPin, bool)]| -> Vec<crate::findrouting::GridPin> { pins.iter().map(|(p, _)| (p.on_grid.0, p.on_grid.1, p.connection_layer)).collect() };
    let mut remaining = Vec::with_capacity(nets.len());
    for (n, pins) in &nets {
        // Net::hasStackedVias — only a net of vias and no wire segments reads the decoded via
        // points (refused: not wired); any other wired net has none.
        let (wire_cnt, via_cnt) = (db.net_get_wire_count_wire_cnt(&n.name), db.net_get_wire_count_via_cnt(&n.name));
        if n.has_wire && wire_cnt == 0 && via_cnt > 0 {
            return Err(format!("net {}: a via-only wire — hasStackedVias' via points are not wired", n.name).into());
        }
        remaining.push(RemainingNet { name: n.name.clone(), made: crate::makes_fastroute_net(pins.len(), n.has_wire, || false), pins: grid_pins(pins) });
    }
    add_remaining_guides(&mut routes, &remaining, t.min_routing_layer, t.max_routing_layer, db.block_get_max_routing_layer()).map_err(|e| format!("{e:?}"))?;
    // saveGuides, over the block's nets in its order.
    let net_routes: Vec<crate::NetRoute> = db
        .net_names()
        .iter()
        .filter_map(|name| {
            let (_, pins) = nets.iter().find(|(n, _)| &n.name == name)?;
            let on_grid: Vec<(i32, i32)> = pins.iter().map(|(p, _)| p.on_grid).collect();
            Some(crate::NetRoute {
                name: name.clone(),
                segments: routes.get(name).cloned().unwrap_or_default(),
                pins: pins.iter().map(|(p, _)| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect(),
                is_local: on_grid.split_first().is_none_or(|(first, rest)| rest.iter().all(|p| p == first)),
            })
        })
        .collect();
    let grid = crate::Grid { tile_size: t.core.tile_size, area: t.core.area };
    let save = crate::SaveOptions { guide_is_congested: false, origin_x: opts.grid_origin.0, origin_y: opts.grid_origin.1, min_routing_layer: t.min_routing_layer };
    let guides = crate::save_guides(&net_routes, &grid, &save).map_err(|e| format!("{e:?}"))?;
    let layer_names = t.tech.routing_layers.iter().map(|l| (l.routing_level, l.name.clone())).collect();
    let jumper_grid = crate::repair_antennas::JumperGrid { grid, x_grids: t.core.x_grids, y_grids: t.core.y_grids };
    let pins = nets.iter().map(|(n, p)| (n.name.clone(), p.iter().map(|(pin, _)| pin.clone()).collect())).collect();
    Ok(CugrGuides {
        guides,
        layer_names,
        routes,
        net_routes,
        jumper_grid,
        max_routing_layer: t.max_routing_layer,
        save,
        pins,
        clock_nets: clock_nets.clone(),
        min_routing_layer: t.min_routing_layer,
        iterations: opts.congestion_iterations,
        alphas: alphas.to_vec(),
        slack,
    })
}

/// `repairAntennas` in a session that routed nothing, over a database a CUGR route was saved to
/// (the block's `grt_use_cugr` property selects the engine): the global router's routes and pins
/// come from the guides, then CUGR is set up and adopts each route as the net's demand. In the
/// reference's order:
///
/// 1. `loadGuidesFromDB` (reached through `check_antennas` → `haveRoutes`): GlobalRouter's nets
///    and pins (found with no FastRoute capacities), then per net in block order each guide
///    [`box_to_global_routing`](crate::restore::box_to_global_routing), then `dedupViaSegments`,
///    `addImplicitVias`, `mergeSegments`, and `ensurePinsPositions`;
/// 2. `initCUGR` — the same model a route builds ([`init_cugr`]), each net's rule factors, the
///    constant slack; the rip-up budget is GlobalRouter's (50 unless this session set one);
/// 3. demand adoption: per net in block order that has a route, `restoreNetRoute` — a refusal
///    is counted, not an error.
///
/// Upstream rules: `ensurePinsPositions` moves a pin no restored segment covers only through the
/// database's access points — a covered one (`findCoveredAccessPoint`), or the recomputation
/// `findOnGridPositions(…, true)`, which without access points reads the same shapes `findPins`
/// read. With none in the database (the only case modelled) no pin moves. A saved CUGR guide is
/// never marked congested (`saveGuides`: `&& !use_cugr_`), so a congested guide changes nothing.
///
/// ⛔ Refused rather than guessed: access points in the database, a clock with a liberty library
/// (the timer's slacks), a detail-routed net (`makeRouteFromWires`), a guide on a net the global
/// router does not know (GRT-0127), and a route on a net CUGR does not hold (`restoreNetRoute`
/// re-admits it).
pub fn restore_cugr_for_repair(db: &mut Db, opts: &RouteOptions, mut trace: Option<&mut Vec<String>>) -> Res<(Cugr, CugrGuides)> {
    if db.block_access_point_count()? > 0 {
        return Err("cugr: pin access points in the database — ensurePinsPositions' access-point branch is not modelled".into());
    }
    if opts.liberty.is_some() && !opts.clock_sources.is_empty() {
        return Err("cugr: a clock with a liberty library — the critical-net slacks are not modelled".into());
    }
    // 1. loadGuidesFromDB
    let t = crate::global_route::setup_tech(db, opts)?;
    let mut log = Vec::new();
    let (routes, pins, order) = {
        let db: &Db = db;
        let all = read_nets(db);
        let db_nets: Vec<&crate::read::NetFacts> = all.iter().collect();
        let candidates: Vec<NetCandidate> = db_nets
            .iter()
            .map(|n| NetCandidate {
                name: n.name.clone(),
                is_supply: n.is_supply(),
                is_special: n.is_special,
                term_count: n.term_count,
                has_special_wires: n.has_special_wires,
                connected_by_abutment: n.connected_by_abutment,
            })
            .collect();
        let nets = crate::global_route::discover_net_pins(db, &t, &db_nets, &candidates, opts, None, &mut log)?;
        let pins: std::collections::BTreeMap<String, Vec<crate::pins::NetPin>> = nets.iter().map(|(n, p)| (n.name.clone(), p.iter().map(|(pin, _)| pin.clone()).collect())).collect();
        let order = db.net_names();
        let tile = t.core.tile_size;
        let block_min = db.block_get_min_routing_layer();
        let mut routes: std::collections::BTreeMap<String, Vec<crate::GSegment>> = std::collections::BTreeMap::new();
        for name in &order {
            for k in 0..db.num_net_get_guides(name) {
                let bx = (db.guide_get_box_x_min(name, k), db.guide_get_box_y_min(name, k), db.guide_get_box_x_max(name, k), db.guide_get_box_y_max(name, k));
                let layer = db.layer_get_routing_level(&db.guide_get_layer(name, k));
                let via_layer = db.layer_get_routing_level(&db.guide_get_via_layer(name, k));
                crate::restore::box_to_global_routing(bx, layer, via_layer, tile, routes.entry(name.clone()).or_default());
            }
        }
        for (name, route) in routes.iter_mut() {
            let p = pins.get(name).ok_or_else(|| format!("[ERROR GRT-0127] net_id for db_net {name} not found — not modelled"))?;
            let grid_pins: Vec<crate::findrouting::GridPin> = p.iter().map(|p| (p.on_grid.0, p.on_grid.1, p.connection_layer)).collect();
            crate::restore::finish_loaded_route(route, &grid_pins, block_min);
        }
        // ensurePinsPositions: no access points in the database (refused above) — no pin moves.
        if let Some(tr) = trace.as_deref_mut() {
            for name in order.iter().filter(|n| routes.contains_key(*n)) {
                tr.push(super::trace::loaded_route(name, &routes[name]));
            }
            for name in order.iter().filter(|n| routes.contains_key(*n)) {
                tr.push(super::trace::loaded_pins(name, &pins[name]));
            }
        }
        (routes, pins, order)
    };
    // 2. initCUGR
    let mut ci = init_cugr(db, opts)?;
    let (alphas, constant) = init_net_costs(&mut ci, db, opts)?;
    if let Some(tr) = trace.as_deref_mut() {
        tr.extend(super::trace::model(&ci.cugr, ci.min_routing_layer, ci.max_routing_layer, ci.clock_nets.len()));
    }
    // 3. Demand adoption, per net in block order.
    let index: std::collections::HashMap<String, usize> = ci.cugr.nets.iter().enumerate().map(|(k, n)| (n.name.clone(), k)).collect();
    let mut commits = Vec::new();
    for name in &order {
        if db.net_get_wire_type(name) == "ROUTED" && !db.net_is_special(name) && db.net_has_wire(name) {
            return Err(format!("cugr: net {name} is detail-routed — makeRouteFromWires is not modelled").into());
        }
        let Some(route) = routes.get(name) else { continue };
        let Some(&k) = index.get(name) else {
            return Err(format!("cugr: net {name} has a route but CUGR does not hold it — restoreNetRoute's re-admission is not modelled").into());
        };
        let r = ci.cugr.restore_net_route(k, route, &mut commits).map_err(|e| format!("cugr: restoring net {name}: {e:?}"))?;
        if let Some(tr) = trace.as_deref_mut() {
            tr.push(super::trace::restore(name, &r));
        }
    }
    let net_routes: Vec<crate::NetRoute> = order
        .iter()
        .filter_map(|name| {
            let p = pins.get(name)?;
            let on_grid: Vec<(i32, i32)> = p.iter().map(|p| p.on_grid).collect();
            Some(crate::NetRoute {
                name: name.clone(),
                segments: routes.get(name).cloned().unwrap_or_default(),
                pins: p.iter().map(|p| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect(),
                is_local: on_grid.split_first().is_none_or(|(first, rest)| rest.iter().all(|p| p == first)),
            })
        })
        .collect();
    let grid = crate::Grid { tile_size: t.core.tile_size, area: t.core.area };
    let save = crate::SaveOptions { guide_is_congested: false, origin_x: opts.grid_origin.0, origin_y: opts.grid_origin.1, min_routing_layer: t.min_routing_layer };
    let guides = CugrGuides {
        guides: Vec::new(),
        layer_names: t.tech.routing_layers.iter().map(|l| (l.routing_level, l.name.clone())).collect(),
        routes,
        net_routes,
        jumper_grid: crate::repair_antennas::JumperGrid { grid, x_grids: t.core.x_grids, y_grids: t.core.y_grids },
        max_routing_layer: t.max_routing_layer,
        save,
        pins,
        clock_nets: ci.clock_nets.clone(),
        min_routing_layer: t.min_routing_layer,
        iterations: opts.congestion_iterations,
        alphas,
        slack: constant,
    };
    Ok((ci.cugr, guides))
}

/// `updatePinAccessPoints`, one pin: CUGR's chosen access point replaces the pin's grid position,
/// and its layer too unless the pin sits above the max routing layer.
fn update_pin_access_point(cugr: &Cugr, net: &super::grnet::GrNet, pin: &mut crate::pins::NetPin, pin_grid: &crate::pins::PinGrid, max_routing_layer: i32) {
    if let Some((x, y, z)) = cugr.pin_access_point(net, &pin.name, pin.is_port) {
        if pin.connection_layer <= max_routing_layer {
            pin.connection_layer = z;
        }
        pin.on_grid = pin_grid.position_on_grid((x, y));
    }
}

/// `updateDirtyRoutesCugr`'s pin test: a dirty net moved when the MULTISET of (grid position,
/// connection layer) now read from the database differs from the one last saved — which is the
/// CUGR-updated one (`updatePinAccessPoints`), not the database's.
fn pins_moved(before: &[crate::pins::NetPin], after: &[crate::pins::NetPin]) -> bool {
    let key = |p: &crate::pins::NetPin| (p.on_grid, p.connection_layer);
    let mut before: Vec<_> = before.iter().map(key).collect();
    let mut after: Vec<_> = after.iter().map(key).collect();
    before.sort();
    after.sort();
    before != after
}

/// `GlobalRouter::updateDirtyRoutesCugr` then `findRoutingCugr(dirty, incremental)`, after a
/// command changed the design under a CUGR route (antenna repair's diodes). Returns the NetGuides
/// `saveGuides(nets_to_repair)` writes for every dirty net — a rerouted net left with no route
/// gets none (its guides are cleared).
///
/// Upstream rules, per dirty net in `dirty_nets_` order (a `PtrSet`: the block's net order):
/// its GlobalRouter pins are rebuilt from the database (`updateNetPins`, CUGR's pin rules); it is
/// rerouted when it has no route, or when its pins, as a multiset of (on-grid position,
/// connection layer), differ from those saved when it was dirtied — the ones CUGR's access points
/// had set, so a net can count as moved with nothing moved. A rerouted net is released and
/// rebuilt in CUGR (`CUGR::updateNet`: its pins re-read, its rule's factors, soft-NDR kept), then
/// all are routed incrementally (`route(true)`). Each rerouted net's route replaces its old one
/// (empty: removed), its pins take CUGR's access points, and `addRemainingGuides` covers the
/// rerouted nets.
#[allow(clippy::too_many_arguments)]
pub fn update_dirty_routes_cugr(
    db: &mut Db,
    opts: &RouteOptions,
    cugr: &mut Cugr,
    cg: &mut CugrGuides,
    dirty: &[String],
    stt: SteinerBuilder<'_>,
    log: &mut Vec<String>,
    mut trace: Option<&mut Vec<String>>,
) -> Res<Vec<crate::NetGuides>> {
    use crate::findrouting::{add_remaining_guides, RemainingNet};
    if opts.liberty.is_some() && !opts.clock_sources.is_empty() {
        return Err("cugr: an incremental route of a clocked liberty design reads the timer's slacks — not modelled".into());
    }
    let t = crate::global_route::setup_tech(db, opts)?;
    let db: &Db = db;
    let all = read_nets(db);
    let dirty_facts: Vec<&crate::read::NetFacts> = dirty.iter().filter_map(|n| all.iter().find(|f| &f.name == n)).collect();
    let candidates: Vec<NetCandidate> = dirty_facts
        .iter()
        .map(|n| NetCandidate {
            name: n.name.clone(),
            is_supply: n.is_supply(),
            is_special: n.is_special,
            term_count: n.term_count,
            has_special_wires: n.has_special_wires,
            connected_by_abutment: n.connected_by_abutment,
        })
        .collect();
    let discovered = crate::global_route::discover_net_pins(db, &t, &dirty_facts, &candidates, opts, None, log)?;
    let (facts, drivers) = read_design_facts(db, &cg.clock_nets)?;
    let fresh = super::design::Design::new(&facts, &cugr.constants, cg.min_routing_layer, cg.max_routing_layer).map_err(|e| format!("{e:?}"))?;
    let index: std::collections::HashMap<String, usize> = cugr.nets.iter().enumerate().map(|(k, n)| (n.name.clone(), k)).collect();
    let mut queued = Vec::new();
    let mut rerouted: Vec<String> = Vec::new();
    for name in dirty {
        let Some((_, new_pins)) = discovered.iter().find(|(n, _)| &n.name == name) else {
            return Err(format!("cugr: dirty net {name} is not one the global router holds — not modelled").into());
        };
        let new_pins: Vec<crate::pins::NetPin> = new_pins.iter().map(|(p, _)| p.clone()).collect();
        let moved = pins_moved(cg.pins.get(name).map_or(&[][..], |v| v.as_slice()), &new_pins);
        let has_route = cg.routes.get(name).is_some_and(|r| !r.is_empty());
        let reroute = !has_route || moved;
        if let Some(tr) = trace.as_deref_mut() {
            tr.push(format!("VYGC|dirty|{name}|route={}|moved={}|reroute={}", i32::from(has_route), i32::from(moved), i32::from(reroute)));
        }
        cg.pins.insert(name.clone(), new_pins);
        if reroute {
            let Some(&k) = index.get(name) else {
                return Err(format!("cugr: rerouting net {name}, which CUGR does not hold, is not modelled").into());
            };
            let fi = facts.nets.iter().position(|f| &f.name == name).ok_or("a dirty net missing from the database")?;
            let dn = fresh.nets.iter().find(|n| &n.name == name).ok_or_else(|| format!("cugr: net {name} is no longer routable — not modelled"))?;
            let ndr = net_ndr_costs(db, name, cugr.grid.num_layers)?;
            cugr.update_net(k, dn.pins.clone(), dn.layer_range, &drivers[fi], ndr, &mut queued).map_err(|e| format!("cugr: {e:?}"))?;
            cugr.nets[k].slack = cg.slack;
            rerouted.push(name.clone());
        }
    }
    cugr.route_incremental(queued, cg.iterations, &cg.alphas, stt, log, trace.as_deref_mut()).map_err(|e| format!("cugr: {e:?}"))?;
    // findRoutingCugr over the rerouted nets.
    let grid_pins = |pins: &[crate::pins::NetPin]| -> Vec<crate::findrouting::GridPin> { pins.iter().map(|p| (p.on_grid.0, p.on_grid.1, p.connection_layer)).collect() };
    let pin_grid = crate::pins::PinGrid {
        die: t.core.area,
        tile_size: t.core.tile_size,
        x_grids: t.core.x_grids,
        y_grids: t.core.y_grids,
        directions: t.tech.routing_layers.iter().map(|l| (l.routing_level, l.direction)).collect(),
        tracks: t.tracks.iter().map(|r| (r.layer_index, (r.location, r.track_pitch))).collect(),
        use_cugr: true,
    };
    let mut remaining = Vec::new();
    for name in &rerouted {
        let k = index[name];
        let route = cugr.net_route(&cugr.nets[k]);
        if route.is_empty() {
            cg.routes.remove(name);
        } else {
            cg.routes.insert(name.clone(), route);
        }
        let pins = cg.pins.get_mut(name).expect("inserted above");
        for pin in pins.iter_mut() {
            update_pin_access_point(cugr, &cugr.nets[k], pin, &pin_grid, t.max_routing_layer);
        }
        let f = all.iter().find(|f| &f.name == name).expect("a dirty net");
        remaining.push(RemainingNet { name: name.clone(), made: crate::makes_fastroute_net(pins.len(), f.has_wire, || false), pins: grid_pins(pins) });
    }
    add_remaining_guides(&mut cg.routes, &remaining, t.min_routing_layer, t.max_routing_layer, db.block_get_max_routing_layer()).map_err(|e| format!("{e:?}"))?;
    // saveGuides(nets_to_repair): every dirty net with a route; a rerouted one without is cleared.
    let mut saved = Vec::new();
    let mut to_save = Vec::new();
    for name in dirty {
        let pins = &cg.pins[name];
        let on_grid: Vec<(i32, i32)> = pins.iter().map(|p| p.on_grid).collect();
        let nr = crate::NetRoute {
            name: name.clone(),
            segments: cg.routes.get(name).cloned().unwrap_or_default(),
            pins: pins.iter().map(|p| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect(),
            is_local: on_grid.split_first().is_none_or(|(first, rest)| rest.iter().all(|p| p == first)),
        };
        if let Some(slot) = cg.net_routes.iter_mut().find(|n| &n.name == name) {
            *slot = nr.clone();
        }
        if cg.routes.contains_key(name) {
            to_save.push(nr);
        } else if rerouted.contains(name) {
            saved.push(crate::NetGuides { net: name.clone(), guides: Vec::new(), jumper_count: 0 });
        }
    }
    saved.extend(crate::save_guides(&to_save, &cg.jumper_grid.grid, &cg.save).map_err(|e| format!("{e:?}"))?);
    Ok(saved)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pins::{NetPin, PinEdge};

    fn pin(on_grid: (i32, i32), layer: i32) -> NetPin {
        NetPin {
            name: "p".into(),
            is_port: false,
            position: on_grid,
            layers: vec![layer],
            boxes: Default::default(),
            edge: PinEdge::None,
            connection_layer: layer,
            connected_to_pad_or_macro: false,
            is_core: true,
            on_grid,
        }
    }

    // Upstream rule (`updateDirtyRoutesCugr`): a net moved when the MULTISET of (grid position,
    // connection layer) differs — order ignored, repeats counted. Two pins sharing a cell, one
    // of them now elsewhere, is a move even though the SET of positions is unchanged. The
    // corpus's diodes never land on a cell a pin of the same net already holds.
    #[test]
    fn a_move_is_a_multiset_difference() {
        let (a, b) = (pin((1, 1), 2), pin((2, 1), 2));
        assert!(!pins_moved(&[a.clone(), b.clone()], &[b.clone(), a.clone()]), "order is not a move");
        assert!(pins_moved(&[a.clone(), a.clone(), b.clone()], &[a.clone(), b.clone(), b.clone()]), "same set, other multiset");
        assert!(pins_moved(&[a.clone()], &[pin((1, 1), 3)]), "a layer change is a move");
        assert!(pins_moved(&[a.clone()], &[a.clone(), b]), "an added pin (a diode) is a move");
    }
}
