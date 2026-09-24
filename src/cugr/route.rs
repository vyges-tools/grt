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
/// - a net with a NON-DEFAULT RULE (`computeNdrCosts`);
/// - a CLOCK defined with a liberty library: the stage-1 order then reads the timer's slacks.
///   Without one every net is unconstrained — `1e+30` with a library (the critical-net
///   percentage defaults to 10), `0` without (forced to 0) — one constant, so the order is the
///   bounding boxes'.
pub fn route_cugr(db: &mut Db, opts: &RouteOptions, stt: SteinerBuilder<'_>, trace: Option<&mut Vec<String>>) -> Res<CugrRoute> {
    if db.block_access_point_count()? > 0 {
        return Err("cugr: pin access points in the database — findODBAccessPoints is not modelled".into());
    }
    if opts.liberty.is_some() && !opts.clock_sources.is_empty() {
        return Err("cugr: a clock with a liberty library — the critical-net slacks are not modelled".into());
    }
    let mut ci = init_cugr(db, opts)?;
    for n in &ci.cugr.nets {
        if !db.net_get_non_default_rule(&n.name).is_empty() {
            return Err(format!("cugr: net {} has a non-default rule — computeNdrCosts is not modelled", n.name).into());
        }
    }
    let slack = if opts.liberty.is_some() && opts.critical_nets_percentage != 0.0 { 1.0e30f32 } else { 0.0 };
    let mut alphas = Vec::with_capacity(ci.cugr.nets.len());
    for n in &mut ci.cugr.nets {
        n.slack = slack;
        alphas.push(crate::global_route::net_steiner_alpha(db, opts, &n.name)?);
    }
    let mut log = Vec::new();
    ci.cugr.pattern_route(&alphas, stt, &mut log, trace).map_err(|e: StageError| format!("cugr: {e:?}"))?;
    let congested = ci.cugr.congested_nets();
    Ok(CugrRoute { init: ci, log, congested })
}
