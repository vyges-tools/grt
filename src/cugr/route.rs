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

/// How far `route_cugr` got.
pub struct CugrRoute {
    pub init: CugrInit,
    /// Stage 1's log lines (GRT-0274).
    pub log: Vec<String>,
    /// Nets whose stage-1 tree is congested; non-empty means stage 3 would run.
    pub congested: Vec<usize>,
}

/// `CUGR::route(false)` as far as it is modelled: `initCUGR`, then stage 1.
///
/// Refused up front, where stage 1 would read what is not modelled:
/// - detailed-router ACCESS POINTS in the database (`findODBAccessPoints`' path);
/// - a CLOCK defined with a liberty library and no captured slacks: the stage-1 order then reads
///   the timer's. Without a clock every net is unconstrained — `1e+30` with a library (the
///   critical-net percentage defaults to 10), `0` without (forced to 0) — one constant, so the
///   order is the bounding boxes'. `call` counts this session's `-use_cugr` routes, from 0.
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
    // computeNdrCosts, per net (`CUGR::init`).
    let num_layers = ci.cugr.grid.num_layers;
    for n in &mut ci.cugr.nets {
        let ndr = db.net_get_non_default_rule(&n.name);
        if ndr.is_empty() {
            continue;
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
        n.ndr_costs = super::ndr_costs(num_layers, &rules);
    }
    // setInitialNetSlacks — only with a non-zero critical-net percentage (forced to 0 without a
    // liberty library). With no clock every net is unconstrained (`1e+30`); with one, the slacks
    // are the timer's and come from the oracle, every net or none.
    let slack_of = |name: &str| -> Res<f32> {
        if opts.liberty.is_none() || opts.critical_nets_percentage == 0.0 {
            return Ok(0.0);
        }
        match oracle {
            Some(Some(m)) => m.get(name).copied().ok_or_else(|| format!("cugr: net {name} has no captured slack — not modelled").into()),
            Some(None) if timed => Err(format!("cugr: no captured slacks for CUGR call {call} — not modelled").into()),
            _ => Ok(1.0e30),
        }
    };
    let mut alphas = Vec::with_capacity(ci.cugr.nets.len());
    for n in &mut ci.cugr.nets {
        n.slack = slack_of(&n.name)?;
        alphas.push(crate::global_route::net_steiner_alpha(db, opts, &n.name)?);
    }
    let mut log = Vec::new();
    ci.cugr.pattern_route(&alphas, stt, &mut log, trace).map_err(|e: StageError| format!("cugr: {e:?}"))?;
    let congested = ci.cugr.congested_nets();
    Ok(CugrRoute { init: ci, log, congested })
}

/// What a CUGR `global_route` saves.
pub struct CugrGuides {
    pub guides: Vec<crate::NetGuides>,
    /// Routing level → layer name, as the guide writer names layers.
    pub layer_names: std::collections::BTreeMap<i32, String>,
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
pub fn cugr_guides(db: &mut Db, opts: &RouteOptions, cugr: &Cugr, log: &mut Vec<String>) -> Res<CugrGuides> {
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
        // updatePinAccessPoints
        for (pin, _) in pins.iter_mut() {
            if let Some((x, y, z)) = cugr.pin_access_point(net, &pin.name, pin.is_port) {
                if pin.connection_layer <= t.max_routing_layer {
                    pin.connection_layer = z;
                }
                pin.on_grid = pin_grid.position_on_grid((x, y));
            }
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
    Ok(CugrGuides { guides, layer_names })
}
