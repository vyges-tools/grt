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
    /// `read_liberty` — the libraries' pad cells and register clocks; `None` without one.
    pub liberty: Option<crate::liberty_clk::LibertyClocks>,
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
    /// `set_macro_extension` — in tiles (default 0).
    pub macro_extension: i32,
}

impl RouteOptions {
    /// The reference's defaults.
    pub fn new() -> Self {
        RouteOptions {
            verbose: false,
            liberty: None,
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
            macro_extension: 0,
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
        has_liberty: opts.liberty.is_some(),
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
pub fn setup_adjust(db: &mut Db, t: &TechSetup, opts: &RouteOptions, log: &mut Vec<String>) -> Res<Adjusted> {
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
    let mut stream: Vec<(Rect, i32, bool)> = Vec::new();
    let contains = |r: &Rect| die.x_min <= r.x_min && die.y_min <= r.y_min && r.x_max <= die.x_max && r.y_max <= die.y_max;
    // findObstructions — the block's own (DEF) obstructions.
    for (n, x0, y0, x1, y1) in db.obstruction_boxes()? {
        let level = db.layer_get_routing_level(&db.layer_name_by_number(n));
        if in_range(level) {
            let rect = Rect { x_min: x0, y_min: y0, x_max: x1, y_max: y1 };
            if !contains(&rect) && opts.verbose {
                log.push("[WARNING GRT-0037] Found blockage outside die area.".into());
            }
            { stream.push((rect, level, false)); apply_obstruction_adjustment(&mut e, rect, level, dir(level), false); }
        }
    }
    // findInstancesObstructions — every instance, in the block's order; masters read once.
    let mut masters: std::collections::HashMap<String, MasterShapes> = std::collections::HashMap::new();
    let mut has_macros_or_pads = false;
    let mut extensions: Option<Vec<i32>> = None;
    let mut layer_obs: std::collections::BTreeMap<i32, Vec<Rect>> = std::collections::BTreeMap::new();
    let mut pins_out_of_die = 0;
    for inst in db.inst_names() {
        let master = db.inst_master(&inst);
        if !masters.contains_key(&master) {
            masters.insert(master.clone(), read_master_shapes(db, &master)?);
        }
        let m = &masters[&master];
        has_macros_or_pads |= m.is_block || m.is_pad;
        let (orient, origin) = (db.inst_get_orient(&inst), (db.inst_get_origin_x(&inst), db.inst_get_origin_y(&inst)));
        if m.is_block {
            // The macro branch: obstructions grouped per layer, layer±1 blocking, then each widened
            // across its layer's direction and applied as a MACRO obstruction.
            let mut per_layer: std::collections::BTreeMap<i32, Vec<Rect>> = std::collections::BTreeMap::new();
            let (mut bottom, mut top) = (i32::MAX, i32::MIN);
            for &(level, r) in &m.obstructions {
                if in_range(level) {
                    per_layer.entry(level).or_default().push(transform_rect(&orient, origin, r));
                    bottom = bottom.min(level);
                    top = top.max(level);
                }
            }
            extend_obstructions(&mut per_layer, bottom, top, min, max);
            if extensions.is_none() {
                extensions = Some(find_layer_extensions(db, t)?);
            }
            let ext = extensions.as_ref().expect("computed");
            for (&layer, obs) in &per_layer {
                let extension = ext[layer as usize] + opts.macro_extension * t.core.tile_size;
                for &o in obs {
                    let mut o = o;
                    match dir(layer) {
                        Some(crate::capacity::Direction::Horizontal) => (o.y_min, o.y_max) = (o.y_min - extension, o.y_max + extension),
                        Some(crate::capacity::Direction::Vertical) => (o.x_min, o.x_max) = (o.x_min - extension, o.x_max + extension),
                        None => {}
                    }
                    layer_obs.entry(layer).or_default().push(o);
                    { stream.push((o, layer, true)); apply_obstruction_adjustment(&mut e, o, layer, dir(layer), true); }
                }
            }
        }
        for &(level, r) in m.obstructions.iter().filter(|_| !m.is_block) {
            if in_range(level) {
                let rect = transform_rect(&orient, origin, r);
                if !contains(&rect) && opts.verbose {
                    log.push(format!("[WARNING GRT-0038] Found blockage outside die area in instance {inst}."));
                }
                { stream.push((rect, level, false)); apply_obstruction_adjustment(&mut e, rect, level, dir(level), false); }
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
            { stream.push((rect, *level, false)); apply_obstruction_adjustment(&mut e, rect, *level, dir(*level), false); }
        }
    }
    if pins_out_of_die > 0 && opts.verbose {
        return Err(format!("GRT-0028: Found {pins_out_of_die} pins outside die area.").into());
    }
    // findNetsObstructions — every net with wires: a supply net's special wires, any other net's
    // routed wire, vias decomposed into their boxes (a cut box, routing level 0, skipped), each
    // through applyNetObstruction.
    let nets = db.net_names();
    if nets.is_empty() {
        return Err("GRT-0094: Design with no nets.".into());
    }
    for net in &nets {
        if db.net_get_wire_count_wire_cnt(net) == 0 {
            continue;
        }
        let sig = db.net_sigtype(net);
        let shapes: Vec<(i64, i32, i32, i32, i32, bool)> = if sig == "POWER" || sig == "GROUND" {
            db.net_swire_shapes(net)?.into_iter().map(|s| (s.0, s.1, s.2, s.3, s.4, s.6)).collect()
        } else {
            db.net_wire_boxes(net).into_iter().map(|b| (b.layer, b.x0, b.y0, b.x1, b.y1, b.from_via)).collect()
        };
        for (n, x0, y0, x1, y1, _) in shapes {
            let level = db.layer_get_routing_level(&db.layer_name_by_number(n));
            // applyNetObstruction — the range test also drops a cut box (level 0).
            if in_range(level) {
                let rect = Rect { x_min: x0, y_min: y0, x_max: x1, y_max: y1 };
                if !contains(&rect) && opts.verbose {
                    log.push(format!("[WARNING GRT-0041] Net {net} has wires/vias outside die area."));
                }
                { stream.push((rect, level, false)); apply_obstruction_adjustment(&mut e, rect, level, dir(level), false); }
            }
        }
    }
    // findTransitionLayers / adjustTransitionLayers — tiles under the MACRO obstructions one layer
    // below each transition layer.
    let transition_layers = find_transition_layers(t);
    for &layer in &transition_layers {
        let mut tiles = std::collections::BTreeSet::new();
        for &obs in layer_obs.get(&(layer - 1)).map(Vec::as_slice).unwrap_or(&[]) {
            let (_, _, first, last) = e.get_blocked_tiles(obs);
            if first == last {
                continue;
            }
            for y in first.1..last.1 {
                for x in first.0..last.0 {
                    tiles.insert((x, y));
                }
            }
        }
        crate::adjust::adjust_tile_set(&mut e, &tiles, layer, dir(layer));
    }
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
    Ok(Adjusted { edges: e, has_macros_or_pads, stream, extensions, transition_layers })
}

/// I10's result: the edges, and whether any instance is a macro or pad (`has_macros_or_pads_`).
pub struct Adjusted {
    pub edges: RouterEdges,
    pub has_macros_or_pads: bool,
    /// Every obstruction applied, in order: `(rect, routing level, is_macro)`.
    pub stream: Vec<(Rect, i32, bool)>,
    /// `findLayerExtensions` (computed at the first macro), per routing level.
    pub extensions: Option<Vec<i32>>,
    /// `findTransitionLayers`.
    pub transition_layers: Vec<i32>,
}

/// `findLayerExtensions`: per routing level in range, the largest of the layer's spacing at the
/// largest width and parallel run, its V5.4 spacings and (refused: not bound) its two-widths
/// table's last entry — the halo a macro obstruction is widened by.
fn find_layer_extensions(db: &Db, t: &TechSetup) -> Res<Vec<i32>> {
    let mut ext = vec![0; t.routing_layers.len() + 1];
    for (level, l) in &t.routing_layers {
        if *level < t.min_routing_layer || *level > t.max_routing_layer {
            continue;
        }
        let mut spacing = db.layer_get_spacing_for(&l.name, i32::MAX, i32::MAX)?;
        for (s, _) in db.layer_v54_spacing_rules(&l.name)? {
            spacing = spacing.max(s as i32);
        }
        if db.layer_has_two_widths_spacing_rules(&l.name) {
            return Err(format!("layer {}: a TWOWIDTHS table — its last entry is not bound", l.name).into());
        }
        ext[*level as usize] = spacing;
    }
    Ok(ext)
}

/// `extendObstructions`: a macro blocking layer±1 blocks the layer between, and the min/max
/// routing layers take their only neighbour's obstructions. ⛔ Layers are visited bottom-up and
/// the map is MUTATED as it goes, so a layer reads its lower neighbour already extended.
fn extend_obstructions(per_layer: &mut std::collections::BTreeMap<i32, Vec<Rect>>, mut bottom: i32, mut top: i32, min: i32, max: i32) {
    if bottom - 1 == min {
        bottom -= 1;
    }
    if top + 1 == max {
        top += 1;
    }
    for layer in bottom..=top {
        per_layer.entry(layer).or_default();
        let mut extended = Vec::new();
        if layer == max {
            if let Some(v) = per_layer.get(&(layer - 1)) {
                extended = v.clone();
            }
        }
        if layer == min {
            if let Some(v) = per_layer.get(&(layer + 1)) {
                extended = v.clone();
            }
        }
        let empty = Vec::new();
        let upper = per_layer.get(&(layer + 1)).unwrap_or(&empty);
        let lower = per_layer.get(&(layer - 1)).unwrap_or(&empty);
        extended.extend(intersection_rectangles(lower, upper));
        if !extended.is_empty() {
            per_layer.get_mut(&layer).expect("inserted").extend(extended);
        }
    }
}

/// `polygon_90_set(lower) & polygon_90_set(upper)`, then `get_rectangles`: the intersection region
/// as horizontal slabs (maximal runs in x per y band, bands merged where identical).
fn intersection_rectangles(lower: &[Rect], upper: &[Rect]) -> Vec<Rect> {
    let mut pieces: Vec<Rect> = Vec::new();
    for a in lower {
        for b in upper {
            let r = Rect { x_min: a.x_min.max(b.x_min), y_min: a.y_min.max(b.y_min), x_max: a.x_max.min(b.x_max), y_max: a.y_max.min(b.y_max) };
            if r.x_min < r.x_max && r.y_min < r.y_max {
                pieces.push(r);
            }
        }
    }
    if pieces.is_empty() {
        return pieces;
    }
    let mut ys: Vec<i32> = pieces.iter().flat_map(|p| [p.y_min, p.y_max]).collect();
    ys.sort_unstable();
    ys.dedup();
    let mut out: Vec<Rect> = Vec::new();
    let mut prev: Vec<(i32, i32)> = Vec::new();
    for w in ys.windows(2) {
        let (y0, y1) = (w[0], w[1]);
        let mut xs: Vec<(i32, i32)> = pieces.iter().filter(|p| p.y_min <= y0 && p.y_max >= y1).map(|p| (p.x_min, p.x_max)).collect();
        xs.sort_unstable();
        let mut runs: Vec<(i32, i32)> = Vec::new();
        for (a, b) in xs {
            match runs.last_mut() {
                Some(l) if a <= l.1 => l.1 = l.1.max(b),
                _ => runs.push((a, b)),
            }
        }
        for &(a, b) in &runs {
            // Extend a slab from the band below when it spans the same x run.
            if prev.contains(&(a, b)) {
                if let Some(r) = out.iter_mut().rev().find(|r| r.x_min == a && r.x_max == b && r.y_max == y0) {
                    r.y_max = y1;
                    continue;
                }
            }
            out.push(Rect { x_min: a, y_min: y0, x_max: b, y_max: y1 });
        }
        prev = runs;
    }
    out
}

/// `findTransitionLayers`: a default via's bottom layer (at or below the max routing layer) whose
/// via is wider, across the layer's direction, than 0.8 of its track pitch.
fn find_transition_layers(t: &TechSetup) -> Vec<i32> {
    let defaults = crate::tracks::get_default_vias(&t.tech.vias);
    let mut bottoms: Vec<(i32, usize)> = defaults.iter().filter_map(|(b, &v)| b.map(|b| (b, v))).collect();
    bottoms.sort_unstable();
    let mut out = Vec::new();
    for (level, via) in bottoms {
        if level > t.max_routing_layer || level < 1 {
            continue;
        }
        let vertical = t.tech.routing_layers.iter().find(|l| l.routing_level == level).and_then(|l| l.direction) == Some(crate::capacity::Direction::Vertical);
        let via_width = t.tech.vias[via].boxes.iter().find(|b| b.0 == level).map_or(0, |b| if vertical { b.2 } else { b.1 });
        let pitch = t.tracks.iter().find(|r| r.layer_index == level).map_or(0, |r| r.track_pitch);
        if f64::from(via_width) / f64::from(pitch) > f64::from(0.8f32) {
            out.push(level);
        }
    }
    out
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
pub fn setup_nets(db: &Db, t: &TechSetup, e: &mut RouterEdges, has_macros_or_pads: bool, opts: &RouteOptions, log: &mut Vec<String>) -> Res<Vec<RouterNet>> {
    let (min, max) = (t.min_routing_layer, t.max_routing_layer);
    let (clk_min, clk_max) = (t.tech.min_layer_for_clock, t.tech.max_layer_for_clock);
    let directions: std::collections::BTreeMap<i32, Option<crate::capacity::Direction>> =
        t.tech.routing_layers.iter().map(|l| (l.routing_level, l.direction)).collect();
    let die = t.core.area;
    // findNets — initClockNets: with a liberty library the timer retypes its clock network's nets
    // to CLOCK; ⛔ with no clock defined (the only case the caller lets through) it finds none.
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
            let class = if m.is_pad {
                MasterClass::Pad
            } else if m.is_block {
                MasterClass::Block
            } else if db.master_is_cover(&master) {
                MasterClass::Cover
            } else {
                MasterClass::Core
            };
            // The instance box — read by a pad or macro pin's edge only.
            let bbox = db.inst_bbox(inst)?;
            let inst_box = if bbox.len() == 4 { Rect { x_min: bbox[0], y_min: bbox[1], x_max: bbox[2], y_max: bbox[3] } } else { die };
            let (orient, origin) = (db.inst_get_orient(inst), (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst)));
            let boxes: Vec<TermBox> = m.pins.iter().filter(|p| &p.0 == term).map(|p| TermBox { pin: 0, level: p.3, routing: p.2, rect: transform_rect(&orient, origin, p.4) }).collect();
            let name = format!("{inst}/{term}");
            let pin = make_iterm_pin(&name, class, db.master_is_core(&master), db.inst_is_placed(inst), inst_box, &boxes, die, max_for_pins, &directions, opts.verbose, log)
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
    // The order: non-leaf clock nets first, each group by name. `isClkTerm` asks the liberty port
    // of each terminal (its instance's master, by name); with no library no terminal is a clock
    // terminal, so every CLOCK-typed net is a non-leaf clock.
    let no_liberty = ITermClockFacts { has_liberty_port: false, is_reg_clk: false, cell_is_pad: false };
    let mut iterm_facts: std::collections::HashMap<&str, Vec<ITermClockFacts>> = std::collections::HashMap::new();
    for (n, _) in &nets {
        let facts = match &opts.liberty {
            Some(lib) if n.sig_type == "CLOCK" => n.iterms.iter().map(|(inst, mterm)| lib.iterm_facts(&db.inst_master(inst), mterm)).collect(),
            _ => vec![no_liberty; n.iterms.len()],
        };
        iterm_facts.insert(n.name.as_str(), facts);
    }
    let non_leaf = |n: &NetFacts| is_non_leaf_clock(n.sig_type == "CLOCK", &iterm_facts[n.name.as_str()]);
    let order = order_nets(&nets.iter().map(|(n, _)| DiscoveredNet { name: n.name.clone(), is_non_leaf_clock: non_leaf(n) }).collect::<Vec<_>>());
    let order_all = order.clone();
    // I14 initNetlist — no seed: the order stands. (addResourcesForPinAccess closes it, below.)
    let grid = NetlistGrid { x_min: die.x_min, y_min: die.y_min, tile_size: t.core.tile_size, x_grids: t.core.x_grids, y_grids: t.core.y_grids, num_layers: t.core.num_layers };
    let mut out = Vec::new();
    for name in order {
        let (n, pins) = nets.iter().find(|(n, _)| n.name == name).expect("ordered from these");
        let conn: Vec<i32> = pins.iter().map(|(p, _)| p.connection_layer).collect();
        let (lo, hi) = get_net_layer_range(&conn, non_leaf(n), min, max, clk_min, clk_max);
        // Net::hasStackedVias — only a net of vias and no wire segments reads the decoded via
        // points (refused: not wired); any other wired net has none.
        let (wire_cnt, via_cnt) = (db.net_get_wire_count_wire_cnt(&n.name), db.net_get_wire_count_via_cnt(&n.name));
        if n.has_wire && wire_cnt == 0 && via_cnt > 0 {
            return Err(format!("net {}: a via-only wire — hasStackedVias' via points are not wired", n.name).into());
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
    // addResourcesForPinAccess — over every net initNets returned, in its order, when the design
    // has macros or pads: one more track on the edge a pad/macro pin faces. ⚠️ After
    // initEdgesCapacityPerLayer, so the NDR ledger's per-layer capacities do not see it (inert
    // without NDR nets).
    if has_macros_or_pads {
        let ordered: Vec<(bool, Vec<crate::netlist::AccessPinFacts>)> = order_all
            .iter()
            .map(|name| {
                let (_, pins) = nets.iter().find(|(n, _)| &n.name == name).expect("ordered from these");
                let facts: Vec<crate::netlist::AccessPinFacts> = pins
                    .iter()
                    .map(|(p, _)| crate::netlist::AccessPinFacts { on_grid: p.on_grid, connection_layer: p.connection_layer, edge: p.edge, connected_to_pad_or_macro: p.connected_to_pad_or_macro })
                    .collect();
                (facts.iter().any(|f| f.connected_to_pad_or_macro), facts)
            })
            .collect();
        let dir = |l: i32| directions.get(&l).copied().flatten();
        for (x1, y1, x2, y2, layer) in crate::netlist::pin_access_edges(&ordered, grid, &dir) {
            let cap = e.get_edge_capacity(x1, y1, x2, y2, layer);
            e.add_adjustment(x1, y1, x2, y2, layer, (cap + 1) as u16, false);
        }
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
    /// R20's routes as run() emitted them, by FastRoute id — before F.
    pub routes: std::collections::BTreeMap<u32, Vec<crate::GSegment>>,
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
    let adj = setup_adjust(db, &t, opts, &mut log)?;
    let mut e = adj.edges;
    let nets = setup_nets(db, &t, &mut e, adj.has_macros_or_pads, opts, &mut log)?;
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
    // getNetSlack: with no clock defined every net is unconstrained — the timer's INF (`1E+30F`).
    let timer_slack = opts.liberty.as_ref().map(|_| vec![1.0e30f32; nets.len()]);
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
        critical_nets_percentage: t.config.critical_nets_percentage,
        layer_dir: &layer_dir,
        resistance_aware: false,
        liberty: opts.liberty.is_some(),
        timer_slack: timer_slack.as_deref(),
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
    let raw_routes = routes.clone();
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
    Ok(RouteResult { guides, layer_names, total_overflow, guide_is_congested, routes: raw_routes, log })
}
