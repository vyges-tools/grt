// SPDX-License-Identifier: Apache-2.0
//! G — `globalRoute` over the design database: the setup (I), with the reads [`crate::read`] makes
//! interleaved where the reference makes them.
//!
//! ⛔ **A sequencer and nothing else.** Every rule is its own function elsewhere, and this file only
//! calls them in the reference's order. It exists because some reads must follow a write: odb's
//! `getGCellTileSize` reads the block's max routing layer, which `getMinMaxLayer` computes and
//! writes back first.

use vyges_opendb::Db;

use crate::capacity::{
    check_adjacent_layers_direction, init_routing_layers, mirror_grid_to_fast_route, set_capacities, CapacityLayer, EdgeCapacities, FastRouteGrid,
    RoutingLayer,
};
use crate::driver::get_min_max_layer;
use crate::init::{config_fast_route, init_grid, report_layer_settings, CoreGrid, FastRouteConfig, SetupOptions};
use crate::adjust::{
    apply_obstruction_adjustment, compute_region_adjustments, compute_user_global_adjustments, compute_user_layer_adjustments, init_blocked_intervals,
    save_resources_before_adjustments, EdgeState, RouterEdges,
};
use crate::init::{is_non_leaf_clock, order_nets, DiscoveredNet, ITermClockFacts};
use crate::netlist::{compute_track_consumption, find_fastroute_pins, get_net_layer_range, makes_fastroute_net, net_max_routing_layer, NetlistGrid, RouterPinFacts};
use crate::pins::{find_nets, find_pin, is_pin_reachable, make_bterm_pin, make_iterm_pin, MasterClass, NetCandidate, NetPin, PinGrid, TermBox};
use crate::read::{read_bterm, read_master_shapes, read_nets, read_tech, read_tile_size, transform_rect, DbSpacing, MasterShapes, NetFacts, TechFacts};
use crate::finalize::{Graph3d, NetLayerAttrs};
use crate::Rect;
use crate::tracks::{calc_layer_pitches, init_routing_tracks, RoutingTracks};

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// The run's options the database does not hold.
#[derive(Debug, Clone, PartialEq)]
pub struct RouteOptions {
    pub verbose: bool,
    pub has_liberty: bool,
    pub critical_nets_percentage: f32,
    /// `set_global_routing_layer_adjustment *`.
    pub adjustment: f32,
    pub grid_origin: (i32, i32),
    /// `global_route -infinite_cap`.
    pub infinite_capacity: bool,
    /// `set_global_routing_region_adjustment`: `(region in dbu, routing level, adjustment)`.
    pub region_adjustments: Vec<(crate::Rect, i32, f32)>,
    /// `global_route -skip_large_fanout_nets` (default: none skipped).
    pub skip_large_fanout: i32,
    /// `set_nets_to_route`, resolved to net names in its order; `None` routes every net.
    pub nets_to_route: Option<Vec<String>>,
    /// `set_routing_alpha` — the Steiner tree builder's global alpha (default 0.3).
    pub alpha: f32,
    /// `set_routing_alpha <a> -min_fanout <n>` — `(n, a)`.
    pub min_fanout_alpha: Option<(i32, f32)>,
    /// `global_route -allow_congestion`.
    pub allow_congestion: bool,
    /// `global_route -congestion_iterations` (default 50).
    pub congestion_iterations: i32,
}

impl RouteOptions {
    /// The reference's defaults.
    pub fn new() -> Self {
        RouteOptions {
            verbose: false,
            has_liberty: false,
            critical_nets_percentage: 10.0,
            adjustment: 0.0,
            grid_origin: (0, 0),
            infinite_capacity: false,
            region_adjustments: Vec::new(),
            skip_large_fanout: i32::MAX,
            nets_to_route: None,
            alpha: 0.3,
            min_fanout_alpha: None,
            allow_congestion: false,
            congestion_iterations: 50,
        }
    }
}

impl Default for RouteOptions {
    fn default() -> Self {
        Self::new()
    }
}

/// What the technology half of the setup (G3, I3–I9) leaves.
#[derive(Debug, Clone)]
pub struct TechSetup {
    pub tech: TechFacts,
    pub min_routing_layer: i32,
    pub max_routing_layer: i32,
    pub config: FastRouteConfig,
    /// I4's index → layer.
    pub routing_layers: Vec<(i32, RoutingLayer)>,
    pub tracks: Vec<RoutingTracks>,
    pub core: CoreGrid,
    pub fast_route: FastRouteGrid,
    pub capacities: EdgeCapacities,
    pub log: Vec<String>,
}

/// G3 `getMinMaxLayer`, then `initFastRoute` up to `setCapacities` (I3–I9).
pub fn setup_tech(db: &mut Db, opts: &RouteOptions) -> Res<TechSetup> {
    let mut log = Vec::new();
    let tech = read_tech(db)?;
    // G3 — an unset max routing layer is computed and WRITTEN BACK to the block.
    let has_track_grid: Vec<bool> = tech.routing_layers.iter().map(|l| l.has_track_grid).collect();
    let (block_max, min, max) = get_min_max_layer(
        tech.block_max_routing_layer,
        &has_track_grid,
        tech.block_min_routing_layer,
        tech.min_layer_for_clock,
        tech.max_layer_for_clock,
    )
    .ok_or("GRT-0701: the lowest routing layer has no track grid")?;
    if block_max != tech.block_max_routing_layer {
        db.block_set_max_routing_layer(block_max)?;
    }
    let block_min = tech.block_min_routing_layer;
    let name_of = |level: i32| tech.routing_layers.iter().find(|l| l.routing_level == level).map(|l| l.name.clone()).unwrap_or_default();
    let setup = SetupOptions {
        verbose: opts.verbose,
        has_liberty: opts.has_liberty,
        critical_nets_percentage: opts.critical_nets_percentage,
        adjustment: opts.adjustment,
        grid_origin: opts.grid_origin,
        min_layer_name: name_of(min),
        max_layer_name: name_of(max),
    };
    // I3
    let config = config_fast_route(&setup, &mut log);
    // I4
    let routing_layers = init_routing_layers(&tech.routing_layers, min, max)?;
    let by_level = |level: i32| tech.routing_layers.iter().find(|l| l.routing_level == level).cloned();
    check_adjacent_layers_direction(&by_level, min, max)?;
    // I5
    report_layer_settings(&setup, &mut log);
    // I6
    let pitches = calc_layer_pitches(&tech.pitch_layers, max, block_min, block_max, tech.routing_layer_count, &tech.vias, &DbSpacing(db));
    let tracks = init_routing_tracks(&tech.tracks, max, &pitches, tech.dbu_per_micron, opts.verbose, &mut log).map_err(|e| format!("{e:?}"))?;
    // I7 — the tile size AFTER G3's write-back.
    let core = init_grid(tech.die, read_tile_size(db), routing_layers.len() as i32, max);
    // I8
    let directions: Vec<_> = (1..=core.num_layers).map(|level| by_level(level).and_then(|l| l.direction)).collect();
    let fast_route = mirror_grid_to_fast_route(&core, &directions);
    // I9 — `getRoutingTracksByIndex(level)`: the FIRST entry with the index.
    let layers: Vec<CapacityLayer> = (1..=core.num_layers)
        .map(|level| CapacityLayer { direction: directions[(level - 1) as usize], tracks: tracks.iter().find(|t| t.layer_index == level).copied() })
        .collect();
    let capacities = set_capacities(&fast_route, &core, &layers, min, max, opts.infinite_capacity);
    Ok(TechSetup { tech, min_routing_layer: min, max_routing_layer: max, config, routing_layers, tracks, core, fast_route, capacities, log })
}

/// I10 — `applyAdjustments`, over the technology setup's capacities.
///
/// ⛔ Refused until the producers are wired (tier B): a macro or pad master (the macro branch —
/// extension, layer±1 blocking, transition layers — and `has_macros_or_pads_`), and a net with
/// routed wires (`findNetsObstructions` decodes them). `perturbCapacities` is inert at its default
/// (0%); `findLayerExtensions` feeds only the macro branch.
pub fn setup_adjust(db: &mut Db, t: &TechSetup, opts: &RouteOptions, log: &mut Vec<String>) -> Res<RouterEdges> {
    let c = &t.capacities;
    let (min, max) = (t.min_routing_layer, t.max_routing_layer);
    let state = |cap: &[u16]| cap.iter().map(|&cap| EdgeState { cap, red: 0, real_cap: 0 }).collect::<Vec<_>>();
    let mut e = RouterEdges {
        die: t.core.area,
        tile_size: t.core.tile_size,
        x_grid: c.x_grid,
        y_grid: c.y_grid,
        num_layers: c.num_layers,
        // `grid_->getTrackPitches()` — per routing level, from I6's tracks.
        track_pitches: (1..=t.core.num_layers).map(|l| t.tracks.iter().find(|r| r.layer_index == l).map_or(0, |r| r.track_pitch)).collect(),
        h3: state(&c.h3),
        v3: state(&c.v3),
        h2: state(&c.h2),
        v2: state(&c.v2),
        horizontal_blocked: Default::default(),
        vertical_blocked: Default::default(),
        verbose: opts.verbose,
        log: Vec::new(),
    };
    let dir = |level: i32| t.tech.routing_layers.iter().find(|l| l.routing_level == level).and_then(|l| l.direction);
    let in_range = |level: i32| min <= level && level <= max;
    let die = t.core.area;
    let contains = |r: &Rect| die.x_min <= r.x_min && die.y_min <= r.y_min && r.x_max <= die.x_max && r.y_max <= die.y_max;
    // findObstructions — the block's own (DEF) obstructions.
    for (n, x0, y0, x1, y1) in db.obstruction_boxes()? {
        let level = db.layer_get_routing_level(&db.layer_name_by_number(n));
        if in_range(level) {
            let rect = Rect { x_min: x0, y_min: y0, x_max: x1, y_max: y1 };
            if !contains(&rect) && opts.verbose {
                log.push("[WARNING GRT-0037] Found blockage outside die area.".into());
            }
            apply_obstruction_adjustment(&mut e, rect, level, dir(level), false);
        }
    }
    // findInstancesObstructions — every instance, in the block's order; masters read once.
    let mut masters: std::collections::HashMap<String, MasterShapes> = std::collections::HashMap::new();
    let mut pins_out_of_die = 0;
    for inst in db.inst_names() {
        let master = db.inst_master(&inst);
        if !masters.contains_key(&master) {
            masters.insert(master.clone(), read_master_shapes(db, &master)?);
        }
        let m = &masters[&master];
        if m.is_block || m.is_pad {
            return Err(format!("instance {inst}: macro/pad master {master} — the macro obstruction path is not wired").into());
        }
        let (orient, origin) = (db.inst_get_orient(&inst), (db.inst_get_origin_x(&inst), db.inst_get_origin_y(&inst)));
        for &(level, r) in &m.obstructions {
            if in_range(level) {
                let rect = transform_rect(&orient, origin, r);
                if !contains(&rect) && opts.verbose {
                    log.push(format!("[WARNING GRT-0038] Found blockage outside die area in instance {inst}."));
                }
                apply_obstruction_adjustment(&mut e, rect, level, dir(level), false);
            }
        }
        for (term, supply, routing, level, r) in &m.pins {
            if !*routing || !in_range(*level) {
                continue;
            }
            let rect = transform_rect(&orient, origin, *r);
            if !contains(&rect) && !*supply {
                log.push(format!("[WARNING GRT-0039] Found pin {term} outside die area in instance {inst}."));
                pins_out_of_die += 1;
            }
            apply_obstruction_adjustment(&mut e, rect, *level, dir(*level), false);
        }
    }
    if pins_out_of_die > 0 && opts.verbose {
        return Err(format!("GRT-0028: Found {pins_out_of_die} pins outside die area.").into());
    }
    // findNetsObstructions — a net with routed wires is refused (see above).
    let nets = db.net_names();
    if nets.is_empty() {
        return Err("GRT-0094: Design with no nets.".into());
    }
    if let Some(n) = nets.iter().find(|n| db.net_get_wire_count_wire_cnt(n) > 0) {
        return Err(format!("net {n} has routed wires — findNetsObstructions is not wired").into());
    }
    // findTransitionLayers / adjustTransitionLayers: only macro obstructions are adjusted, and a
    // macro is refused above — nothing to do.
    init_blocked_intervals(&mut e);
    save_resources_before_adjustments(&mut e);
    // computeUserGlobalAdjustments — WRITES the layer adjustment into the database.
    let mut layer_adjustment: Vec<f32> = std::iter::once(0.0).chain((1..=max.max(t.core.num_layers)).map(|l| {
        t.tech.routing_layers.iter().find(|r| r.routing_level == l).map_or(0.0, |r| db.layer_get_layer_adjustment(&r.name))
    })).collect();
    let before = layer_adjustment.clone();
    compute_user_global_adjustments(&mut layer_adjustment, opts.adjustment, min, max);
    for l in 1..layer_adjustment.len() {
        if layer_adjustment[l] != before[l] {
            let name = &t.tech.routing_layers.iter().find(|r| r.routing_level == l as i32).expect("layer").name;
            db.layer_set_layer_adjustment(name, layer_adjustment[l])?;
        }
    }
    let dirs: Vec<_> = std::iter::once(None).chain((1..layer_adjustment.len() as i32).map(dir)).collect();
    compute_user_layer_adjustments(&mut e, &layer_adjustment, &dirs, min, max);
    for &(region, layer, adjustment) in &opts.region_adjustments {
        let use_pitch = t.tracks.iter().find(|r| r.layer_index == layer).map_or(-1, |r| r.use_pitch());
        compute_region_adjustments(&mut e, region, layer, adjustment, dir(layer), use_pitch).map_err(|_| format!("GRT: region adjustment on layer {layer} outside the die"))?;
    }
    log.extend(e.log.drain(..));
    Ok(e)
}

/// `addLayerAdjustment(level, adjustment)` — what `set_global_routing_layer_adjustment <layer> <adj>`
/// does for one layer: stored ON THE TECH LAYER, unless the layer is above the block's max routing
/// layer AT THE TIME OF THE CALL (and that max is set), when it is ignored (GRT-30, verbose).
pub fn add_layer_adjustment(db: &mut Db, level: i32, adjustment: f32, verbose: bool, log: &mut Vec<String>) -> Res<()> {
    let name_of = |db: &Db, level: i32| -> Res<String> {
        let tech = read_tech(db)?;
        Ok(tech.routing_layers.iter().find(|l| l.routing_level == level).map(|l| l.name.clone()).unwrap_or_default())
    };
    let max = db.block_get_max_routing_layer();
    if level > max && max > 0 {
        if verbose {
            log.push(format!(
                "[WARNING GRT-0030] Specified layer {} for adjustment is greater than max routing layer {} and will be ignored.",
                name_of(db, level)?,
                name_of(db, max)?
            ));
        }
        return Ok(());
    }
    let name = name_of(db, level)?;
    db.layer_set_layer_adjustment(&name, adjustment)?;
    Ok(())
}

/// One net as the router receives it (`FastRouteCore::addNet` and its pins).
#[derive(Debug, Clone, PartialEq)]
pub struct RouterNet {
    pub name: String,
    /// `(x, y, routing level)` on the grid — `fr_net->addPin(x, y, level - 1)`.
    pub pins: Vec<(i32, i32, i32)>,
    pub root: usize,
    pub is_clock: bool,
    /// `Net::isLocal()` — every pin at one on-grid position. ⛔ A local net is ADDED to the router
    /// (it takes an id) but not to `net_ids_`: it is never routed, only merged into the guides.
    pub is_local: bool,
    /// Routing levels, 1-based (the router stores `- 1`).
    pub min_layer: i32,
    pub max_layer: i32,
    pub edge_cost: i8,
    pub layer_edge_cost: Option<Vec<i8>>,
    /// `stt_builder_->getAlpha(net)`.
    pub alpha: f32,
    /// Each pin as `updateNetPins` left it, in the net's order.
    pub net_pins: Vec<NetPin>,
    /// `dbNet::getTermCount()` — the Steiner builder's min-fanout test reads it.
    pub term_count: i32,
}

/// I13 `initNets` (`findNets`: discovery, pins, the order) and I14 `initNetlist`.
///
/// ⛔ Refused as in I10: a pad or macro terminal, a net with a wire. A block terminal skipped for
/// having no routing geometry is the Rudy path's leniency, reproduced (`check_pin_placement` off).
pub fn setup_nets(db: &Db, t: &TechSetup, e: &RouterEdges, opts: &RouteOptions, log: &mut Vec<String>) -> Res<Vec<RouterNet>> {
    let (min, max) = (t.min_routing_layer, t.max_routing_layer);
    let (clk_min, clk_max) = (t.tech.min_layer_for_clock, t.tech.max_layer_for_clock);
    let directions: std::collections::BTreeMap<i32, Option<crate::capacity::Direction>> =
        t.tech.routing_layers.iter().map(|l| (l.routing_level, l.direction)).collect();
    let die = t.core.area;
    // findNets — initClockNets needs a liberty library; none here.
    let all = read_nets(db);
    let db_nets: Vec<&NetFacts> = match &opts.nets_to_route {
        None => all.iter().collect(),
        Some(names) => names.iter().map(|n| all.iter().find(|f| &f.name == n).ok_or_else(|| format!("net {n} not found"))).collect::<Result<_, _>>()?,
    };
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
    let added = find_nets(&candidates, opts.skip_large_fanout, log);
    // addNet → updateNetPins: every terminal's pin, then findPins.
    let mut masters: std::collections::HashMap<String, MasterShapes> = std::collections::HashMap::new();
    let pin_grid = PinGrid {
        die,
        tile_size: t.core.tile_size,
        x_grids: t.core.x_grids,
        y_grids: t.core.y_grids,
        directions: directions.clone(),
        tracks: t.tracks.iter().map(|r| (r.layer_index, (r.location, r.track_pitch))).collect(),
        use_cugr: false,
    };
    let mut nets: Vec<(&NetFacts, Vec<(NetPin, bool)>)> = Vec::new();
    for &i in &added {
        let n = db_nets[i];
        let is_clock = n.sig_type == "CLOCK";
        let max_for_pins = if is_clock && clk_max > 0 { clk_max } else { max };
        let mut pins = Vec::new();
        for (inst, term) in &n.iterms {
            let master = db.inst_master(inst);
            if !masters.contains_key(&master) {
                masters.insert(master.clone(), read_master_shapes(db, &master)?);
            }
            let m = &masters[&master];
            if m.is_block || m.is_pad {
                return Err(format!("{inst}/{term}: pad/macro terminal — not wired").into());
            }
            let class = if db.master_is_cover(&master) { MasterClass::Cover } else { MasterClass::Core };
            let (orient, origin) = (db.inst_get_orient(inst), (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst)));
            let boxes: Vec<TermBox> = m.pins.iter().filter(|p| &p.0 == term).map(|p| TermBox { pin: 0, level: p.3, routing: p.2, rect: transform_rect(&orient, origin, p.4) }).collect();
            let name = format!("{inst}/{term}");
            // The instance box is read only for a pad or macro pin (its edge), refused above.
            let pin = make_iterm_pin(&name, class, db.master_is_core(&master), db.inst_is_placed(inst), die, &boxes, die, max_for_pins, &directions, opts.verbose, log)
                .map_err(|e| format!("{e:?}"))?;
            let io = db.mterm_get_io_type(&master, term);
            pins.push((pin, io == "OUTPUT" || io == "INOUT"));
        }
        for bterm in &n.bterms {
            let (placed, bx) = read_bterm(db, bterm)?;
            let boxes: Vec<TermBox> = bx.into_iter().map(|(level, routing, rect)| TermBox { pin: 0, level, routing, rect }).collect();
            if let Some(pin) = make_bterm_pin(bterm, placed, &boxes, die, &directions, false, opts.verbose, log).map_err(|e| format!("{e:?}"))? {
                pins.push((pin, db.bterm_get_io_type(bterm) == "INPUT"));
            }
        }
        let cap = |layer: i32, x1: i32, y1: i32, x2: i32, y2: i32| e.get_edge_capacity(x1, y1, x2, y2, layer);
        for (pin, _) in &mut pins {
            find_pin(&pin_grid, pin, &[], &mut |p, pos| is_pin_reachable(&pin_grid, p, pos, &cap));
        }
        nets.push((n, pins));
    }
    // The order: non-leaf clock nets first, each group by name. With no liberty library no
    // terminal is a clock terminal, so every CLOCK-typed net is a non-leaf clock.
    let no_liberty = ITermClockFacts { has_liberty_port: false, is_reg_clk: false, cell_is_pad: false };
    let non_leaf = |n: &NetFacts| is_non_leaf_clock(n.sig_type == "CLOCK", &vec![no_liberty; n.iterms.len()]);
    let order = order_nets(&nets.iter().map(|(n, _)| DiscoveredNet { name: n.name.clone(), is_non_leaf_clock: non_leaf(n) }).collect::<Vec<_>>());
    // I14 initNetlist — no seed: the order stands.
    let grid = NetlistGrid { x_min: die.x_min, y_min: die.y_min, tile_size: t.core.tile_size, x_grids: t.core.x_grids, y_grids: t.core.y_grids, num_layers: t.core.num_layers };
    let mut out = Vec::new();
    for name in order {
        let (n, pins) = nets.iter().find(|(n, _)| n.name == name).expect("ordered from these");
        let conn: Vec<i32> = pins.iter().map(|(p, _)| p.connection_layer).collect();
        let (lo, hi) = get_net_layer_range(&conn, non_leaf(n), min, max, clk_min, clk_max);
        if n.has_wire {
            return Err(format!("net {} has a wire — hasStackedVias is not wired", n.name).into());
        }
        if !makes_fastroute_net(pins.len(), n.has_wire, || false) {
            continue;
        }
        let facts: Vec<RouterPinFacts> = pins.iter().map(|(p, d)| RouterPinFacts { on_grid: p.on_grid, connection_layer: p.connection_layer, is_driver: *d }).collect();
        let (on_grid, root) = find_fastroute_pins(&facts, grid, net_max_routing_layer(n.sig_type == "CLOCK", clk_max, max));
        // No NDR on any tier-A net: the edge cost is 1 and there is no per-layer vector.
        let (edge_cost, lec) = compute_track_consumption(None, min, max, t.core.num_layers).map_err(|e| format!("{e:?}"))?;
        out.push(RouterNet {
            name: n.name.clone(),
            pins: on_grid,
            root,
            is_clock: n.sig_type == "CLOCK",
            is_local: pins.split_first().is_none_or(|(first, rest)| rest.iter().all(|(p, _)| p.on_grid == first.0.on_grid)),
            min_layer: lo,
            max_layer: hi,
            edge_cost,
            layer_edge_cost: lec,
            alpha: opts.alpha,
            net_pins: pins.iter().map(|(p, _)| p.clone()).collect(),
            term_count: n.term_count,
        });
    }
    Ok(out)
}

/// What a whole `global_route` produced.
#[derive(Debug, Clone)]
pub struct RouteResult {
    /// `saveGuides`' records, per net in the block's order.
    pub guides: Vec<crate::NetGuides>,
    /// Routing level → layer name, for writing guides.
    pub layer_names: std::collections::BTreeMap<i32, String>,
    /// The router's total overflow after R19, and whether the guides are marked congested.
    pub total_overflow: i32,
    pub guide_is_congested: bool,
    pub log: Vec<String>,
}

/// The Steiner tree builder as R5 calls it: pins, driver index, the net's alpha.
pub type SteinerBuilder<'a> = &'a dyn Fn(&[i32], &[i32], usize, f32) -> crate::brk_rsmt::RsmtTree;

/// `globalRoute` end to end over the database: setup (I) → `run()` (R) → `findRouting`'s
/// post-processing (F) → `saveGuides` (X). The Steiner tree builder and FLUTE are injected.
pub fn route_design(db: &mut Db, opts: &RouteOptions, stt: SteinerBuilder<'_>, flutes: crate::brk_rsmt::Flutes<'_>) -> Res<RouteResult> {
    use crate::brk_rsmt::{CapLayer, Caps3D, NetState, RsmtNet};
    use crate::run::{fastroute_run, RunEnd, RunInputs, RunObserver, Stage};
    let t = setup_tech(db, opts)?;
    let mut log = t.log.clone();
    let e = setup_adjust(db, &t, opts, &mut log)?;
    let nets = setup_nets(db, &t, &e, opts, &mut log)?;
    let (xg, yg) = (e.x_grid as usize, e.y_grid as usize);
    // The router's grid, in run()'s layout (`[y * xg + x]` for both directions).
    let (mut red_h, mut red_v, mut cap_h, mut cap_v) = (vec![0u16; xg * yg], vec![0u16; xg * yg], vec![0u16; xg * yg], vec![0u16; xg * yg]);
    for y in 0..yg {
        for x in 0..xg {
            if x + 1 < xg {
                let s = e.h2[y * (xg - 1) + x];
                (red_h[y * xg + x], cap_h[y * xg + x]) = (s.red, s.cap);
            }
            if y + 1 < yg {
                let s = e.v2[y * xg + x];
                (red_v[y * xg + x], cap_v[y * xg + x]) = (s.red, s.cap);
            }
        }
    }
    let layer_of = |v: &[crate::adjust::EdgeState], l: usize| -> Vec<i32> { (0..xg * yg).map(|i| i32::from(v[l * xg * yg + i].cap)).collect() };
    let caps = Caps3D { x_grid: xg, layers: (0..e.num_layers as usize).map(|l| CapLayer { h: layer_of(&e.h3, l), v: layer_of(&e.v3, l) }).collect() };
    // The nets, indexed by FastRoute id; the routed ones exclude local nets.
    let pins: Vec<(Vec<i32>, Vec<i32>)> = nets.iter().map(|n| n.pins.iter().map(|p| (p.0, p.1)).unzip()).collect();
    let lecs: Vec<Vec<i8>> = nets.iter().map(|n| vec![1; (n.max_layer - n.min_layer + 1).max(0) as usize]).collect();
    let rnets: Vec<RsmtNet<'_>> = nets
        .iter()
        .enumerate()
        .map(|(k, n)| RsmtNet {
            pins_x: &pins[k].0,
            pins_y: &pins[k].1,
            alpha: n.alpha,
            edge_cost: n.edge_cost,
            min_layer: (n.min_layer - 1) as usize,
            max_layer: (n.max_layer - 1) as usize,
            layer_edge_cost: &lecs[k],
        })
        .collect();
    let net_ids: Vec<usize> = (0..nets.len()).filter(|&k| !nets[k].is_local).collect();
    let num_layers = e.num_layers as usize;
    let attrs: Vec<NetLayerAttrs> = nets
        .iter()
        .map(|n| NetLayerAttrs {
            pin_layers: n.pins.iter().map(|p| (p.2 - 1) as i16).collect(),
            has_ndr: false,
            is_clock: n.is_clock,
            is_res_aware: false,
            layer_edge_cost: vec![1; num_layers],
            sta_slack: 0.0,
        })
        .collect();
    let slack = vec![(0.0f32, false); nets.len()];
    // makeSteinerTree(net, …): the net's alpha — the min-fanout rule when set.
    let stt_net = |id: usize| {
        let n = &nets[id];
        let alpha = match opts.min_fanout_alpha {
            Some((min_fanout, a)) if min_fanout > 0 && n.term_count - 1 >= min_fanout => a,
            _ => n.alpha,
        };
        stt(&pins[id].0, &pins[id].1, n.root, alpha)
    };
    let layer_dir: Vec<crate::layertable::LayerDir> = (1..=num_layers as i32)
        .map(|l| match t.tech.routing_layers.iter().find(|r| r.routing_level == l).and_then(|r| r.direction) {
            Some(crate::capacity::Direction::Horizontal) => crate::layertable::LayerDir::Horizontal,
            Some(crate::capacity::Direction::Vertical) => crate::layertable::LayerDir::Vertical,
            None => crate::layertable::LayerDir::Other,
        })
        .collect();
    let db_id: Vec<u32> = (0..nets.len() as u32).collect();
    let inp = RunInputs {
        x_grid: xg,
        y_grid: yg,
        h_capacity: t.capacities.h_capacity,
        v_capacity: t.capacities.v_capacity,
        red_h: &red_h,
        red_v: &red_v,
        cap_h: &cap_h,
        cap_v: &cap_v,
        entry: crate::estimate::EstimateGrid::new(xg, yg),
        caps: &caps,
        net_ids: &net_ids,
        nets: &rnets,
        attrs: &attrs,
        slack: &slack,
        stt: &stt_net,
        flutes,
        overflow_iterations: opts.congestion_iterations,
        critical_nets_percentage: t.config.critical_nets_percentage as i32,
        layer_dir: &layer_dir,
        resistance_aware: false,
        liberty: opts.has_liberty,
        origin: crate::routes::GridOrigin { tile_size: t.core.tile_size, x_corner: t.core.area.x_min, y_corner: t.core.area.y_min },
        db_id: &db_id,
    };
    // The run's final overflow (after R19), for the congestion verdict.
    struct Overflow(i32);
    impl RunObserver for Overflow {
        fn stage(&mut self, s: Stage<'_>, _: &crate::graph2d::Graph2d, _: Option<&Graph3d>, _: &[NetState]) -> bool {
            if let Stage::B19(fin) = s {
                self.0 = fin.overflow.total;
            }
            true
        }
    }
    let mut state = vec![NetState::default(); nets.len()];
    let mut ov = Overflow(0);
    let routes = match fastroute_run(&inp, &mut state, &mut ov)? {
        RunEnd::Routed(r) => r,
        RunEnd::Stopped => return Err("run() stopped".into()),
    };
    // F — findRouting's post-processing: remaining guides, pad pins (inert), then each merge.
    let mut by_name: std::collections::BTreeMap<String, Vec<crate::GSegment>> =
        routes.into_iter().map(|(id, segs)| (nets[id as usize].name.clone(), segs)).collect();
    let grid_pins = |n: &RouterNet| -> Vec<crate::findrouting::GridPin> { n.net_pins.iter().map(|p| (p.on_grid.0, p.on_grid.1, p.connection_layer)).collect() };
    let remaining: Vec<crate::findrouting::RemainingNet> = nets.iter().map(|n| crate::findrouting::RemainingNet { name: n.name.clone(), made: true, pins: grid_pins(n) }).collect();
    let block_max = db.block_get_max_routing_layer();
    crate::findrouting::add_remaining_guides(&mut by_name, &remaining, t.min_routing_layer, t.max_routing_layer, block_max).map_err(|e| format!("{e:?}"))?;
    crate::findrouting::connect_pad_pins(&mut by_name);
    let block_min = db.block_get_min_routing_layer();
    for n in &nets {
        if let Some(route) = by_name.get_mut(&n.name) {
            crate::findrouting::merge_segments(&grid_pins(n), route, block_min);
        }
    }
    // X — saveGuides over the block's nets, in its order.
    let total_overflow = ov.0;
    let guide_is_congested = total_overflow > 0 && !opts.allow_congestion;
    let order = db.net_names();
    let net_routes: Vec<crate::NetRoute> = order
        .iter()
        .filter_map(|name| {
            let n = nets.iter().find(|n| &n.name == name)?;
            Some(crate::NetRoute {
                name: name.clone(),
                segments: by_name.get(name).cloned().unwrap_or_default(),
                pins: n.net_pins.iter().map(|p| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect(),
                is_local: n.is_local,
            })
        })
        .collect();
    let grid = crate::Grid { tile_size: t.core.tile_size, area: t.core.area };
    let save = crate::SaveOptions { guide_is_congested, origin_x: opts.grid_origin.0, origin_y: opts.grid_origin.1, min_routing_layer: t.min_routing_layer };
    let guides = crate::save_guides(&net_routes, &grid, &save).map_err(|e| format!("{e:?}"))?;
    let layer_names = t.tech.routing_layers.iter().map(|l| (l.routing_level, l.name.clone())).collect();
    Ok(RouteResult { guides, layer_names, total_overflow, guide_is_congested, log })
}
