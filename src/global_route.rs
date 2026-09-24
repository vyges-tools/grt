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
use crate::netlist::{compute_track_consumption, find_fastroute_pins, get_net_layer_range, makes_fastroute_net, net_max_routing_layer, NdrLayerRule, NetlistGrid, RouterPinFacts};
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
    /// `create_clock … [get_ports …]` — the clocks' source ports, over every clock defined.
    pub clock_sources: Vec<String>,
    /// The timer's slacks captured from the reference at each partial-slack call, by net name —
    /// the ORACLE for a run with a clock (this engine computes no timing).
    pub captured_slacks: Option<Vec<std::collections::BTreeMap<String, f32>>>,
    /// The same, at each `updateSlacks` call (resistance-aware only).
    pub captured_update_slacks: Option<Vec<std::collections::BTreeMap<String, f32>>>,
    /// `global_route -resistance_aware`.
    pub resistance_aware: bool,
    /// `-res_aware_nets_percentage` — once given, FIXED (`is_fixed_nets_percentage_`).
    pub res_aware_nets_percentage: Option<f32>,
    /// `set_layer_rc -layer` — the ESTIMATOR's table: routing level → (ohm/m, F/m). The parasitics
    /// read it in preference to the technology's own values.
    pub layer_rc: std::collections::BTreeMap<i32, (f64, f64)>,
    /// `set_layer_rc -via` — the estimator's table for a cut layer, by its name (ohms per cut).
    pub via_rc: std::collections::BTreeMap<String, f64>,
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
    /// `set_routing_alpha <a> -min_hpwl <µm>` — `(hpwl in dbu, a)`; the µm are rounded
    /// (`microns_to_dbu`) where the command runs.
    pub min_hpwl_alpha: Option<(i32, f32)>,
    /// `set_routing_alpha <a> -net …` and `-clock_nets` — `setNetAlpha`, by net name.
    pub net_alpha: std::collections::BTreeMap<String, f32>,
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
            clock_sources: Vec::new(),
            captured_slacks: None,
            captured_update_slacks: None,
            resistance_aware: false,
            res_aware_nets_percentage: None,
            layer_rc: std::collections::BTreeMap::new(),
            via_rc: std::collections::BTreeMap::new(),
            critical_nets_percentage: 10.0,
            adjustment: 0.0,
            grid_origin: (0, 0),
            infinite_capacity: false,
            region_adjustments: Vec::new(),
            skip_large_fanout: i32::MAX,
            nets_to_route: None,
            alpha: 0.3,
            min_fanout_alpha: None,
            min_hpwl_alpha: None,
            net_alpha: std::collections::BTreeMap::new(),
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
    /// `computeTrackConsumption`'s per-layer costs, indexed by `level - 1` (`num_layers + 1`
    /// entries); `None` without an NDR.
    pub layer_edge_cost: Option<Vec<i8>>,
    /// `getNonDefaultRule() != nullptr` — the RULE, which a soft-NDR demotion keeps.
    pub has_ndr: bool,
    /// The NDR's layer-rule width per routing level (`getLayerResistance` reads it); `None` without
    /// an NDR.
    pub ndr_widths: Option<std::collections::BTreeMap<i32, i32>>,
    /// `stt_builder_->getAlpha(net)`.
    pub alpha: f32,
    /// Each pin as `updateNetPins` left it, in the net's order.
    pub net_pins: Vec<NetPin>,
    /// Per pin of [`net_pins`](Self::net_pins), whether it drives the net.
    pub pin_is_driver: Vec<bool>,
    /// `dbNet::getTermCount()` — the Steiner builder's min-fanout test reads it.
    pub term_count: i32,
}

/// `getLayerEdgeCost(l)` over the net's own range `min_layer..=max_layer` — what the NDR-aware
/// charge reads (`RsmtNet::layer_edge_cost`). 1 throughout without an NDR.
fn net_range_edge_costs(n: &RouterNet) -> Vec<i8> {
    let (lo, hi) = ((n.min_layer - 1).max(0) as usize, (n.max_layer - 1).max(0) as usize);
    match &n.layer_edge_cost {
        Some(v) if n.max_layer >= n.min_layer => v[lo..=hi].to_vec(),
        _ => vec![1; (n.max_layer - n.min_layer + 1).max(0) as usize],
    }
}

/// `getLayerEdgeCost(l)` for every layer `0..num_layers` (layer assignment's view).
fn all_layer_edge_costs(n: &RouterNet, num_layers: usize) -> Vec<i8> {
    n.layer_edge_cost.as_ref().map_or_else(|| vec![1; num_layers], |v| v[..num_layers].to_vec())
}

/// `findNets` → `addNet` → `updateNetPins` → `findPins`: every admitted net with its pins placed
/// on the grid, each with whether it drives, in discovery order.
///
/// `edge_capacity` is FastRoute's: a pad or macro pin that cannot reach its on-grid position is
/// moved toward its instance's edge. `None` is CUGR's side, which has no FastRoute capacities and
/// skips that heuristic (`findOnGridPositions`, `!use_cugr_`).
pub(crate) fn discover_net_pins<'a>(
    db: &Db,
    t: &TechSetup,
    db_nets: &[&'a NetFacts],
    candidates: &[NetCandidate],
    opts: &RouteOptions,
    edge_capacity: Option<&dyn Fn(i32, i32, i32, i32, i32) -> i32>,
    log: &mut Vec<String>,
) -> Res<Vec<(&'a NetFacts, Vec<(NetPin, bool)>)>> {
    let max = t.max_routing_layer;
    let clk_max = t.tech.max_layer_for_clock;
    let directions: std::collections::BTreeMap<i32, Option<crate::capacity::Direction>> =
        t.tech.routing_layers.iter().map(|l| (l.routing_level, l.direction)).collect();
    let die = t.core.area;
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
        use_cugr: edge_capacity.is_none(),
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
        for (pin, _) in &mut pins {
            // With no capacities (CUGR) `find_pin` never asks: `use_cugr` skips the reachability test.
            find_pin(&pin_grid, pin, &[], &mut |p, pos| edge_capacity.is_some_and(|cap| is_pin_reachable(&pin_grid, p, pos, cap)));
        }
        nets.push((n, pins));
    }
    Ok(nets)
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
    let cap = |layer: i32, x1: i32, y1: i32, x2: i32, y2: i32| e.get_edge_capacity(x1, y1, x2, y2, layer);
    let nets = discover_net_pins(db, t, &db_nets, &candidates, opts, Some(&cap), log)?;
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
        // computeTrackConsumption: the net's NDR (`getNonDefaultRule`), each layer rule read with
        // its layer's default width and the track pitch of that routing level. No NDR: cost 1.
        let ndr = db.net_get_non_default_rule(&n.name);
        let ndr_layer_rules = if ndr.is_empty() { Vec::new() } else { db.ndr_layer_rules(&ndr)? };
        let ndr_widths = (!ndr.is_empty())
            .then(|| ndr_layer_rules.iter().map(|(layer, width, _)| (db.layer_get_routing_level(layer), *width)).collect());
        let rules: Option<Vec<NdrLayerRule>> = if ndr.is_empty() {
            None
        } else {
            Some(ndr_layer_rules.iter().cloned().map(|(layer, width, spacing)| {
                let level = db.layer_get_routing_level(&layer);
                NdrLayerRule {
                    level,
                    default_width: db.layer_get_width(&layer) as i32,
                    default_pitch: t.tracks.iter().find(|r| r.layer_index == level).map_or(0, |r| r.track_pitch),
                    ndr_spacing: spacing,
                    ndr_width: width,
                }
            }).collect())
        };
        let (edge_cost, lec) = compute_track_consumption(rules.as_deref(), min, max, t.core.num_layers)
            .map_err(|e| format!("[ERROR GRT-0272] NDR consumption {} exceeds 127 and is unsupported", e.0))?;
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
            has_ndr: !ndr.is_empty(),
            ndr_widths,
            // getAlpha: the net's own alpha, else the global one — FastRoute's path gate reads it.
            alpha: opts.net_alpha.get(&n.name).copied().unwrap_or(opts.alpha),
            net_pins: pins.iter().map(|(p, _)| p.clone()).collect(),
            pin_is_driver: pins.iter().map(|(_, d)| *d).collect(),
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
    /// The nets `initClockNets` re-typed CLOCK (`findClkNets`), by name.
    pub clock_nets: std::collections::BTreeSet<String>,
    /// `est::estimateAllGlobalRouteParasitics` at the router's FIRST partial-slack call: each
    /// net's RC network, built from the planar routes as they stood then. Empty when the run
    /// reached no such call.
    pub parasitics: std::collections::BTreeMap<String, crate::parasitics::Network>,
    /// The same, over the routes the run SAVED (`estimate_parasitics -global_routing`).
    pub routed_parasitics: std::collections::BTreeMap<String, crate::parasitics::Network>,
    /// Each net's pins as the parasitics saw them — for a SPEF `*CONN` section.
    pub parasitic_pins: std::collections::BTreeMap<String, Vec<crate::parasitics::PinGridLocation>>,
    /// The planar routes those networks were built from (`getPlanarRoutes`), by net.
    pub planar_routes: std::collections::BTreeMap<String, Vec<crate::parasitics::Segment>>,
    /// The 2D tree those routes were read from, per net: each edge's ends, length and grid points
    /// IN ORDER — what a route-by-route comparison against the reference needs.
    pub snapshot_edges: std::collections::BTreeMap<String, Vec<SnapshotEdge>>,
    pub log: Vec<String>,
    /// The router's state as a later command in the same session reads it.
    pub after: AfterRoute,
}

/// What stays alive in the router after `global_route` for a later command — antenna repair's
/// jumper pass and the `saveGuides` that follows it.
#[derive(Debug, Clone)]
pub struct AfterRoute {
    /// The 3D edges at the end of the run (`h_edges_3D_` / `v_edges_3D_`). `None` when the run did
    /// not reach R19.
    pub final_3d: Option<Graph3d>,
    /// The 2D graph at the end of the run (usage, estimate, history, used sets).
    pub final_2d: Option<crate::graph2d::Graph2d>,
    /// Every net's router state at the end of the run, by FastRoute id — its final 3D tree is what a
    /// rip-up releases.
    pub final_state: Vec<crate::brk_rsmt::NetState>,
    /// The 2D edge reductions and the 3D capacities the run used: an incremental run keeps them
    /// (`initFastRoute` is not repeated).
    pub red_h: Vec<u16>,
    pub red_v: Vec<u16>,
    pub caps: crate::brk_rsmt::Caps3D,
    /// The 3D edges as the adjustments left them (`cap` reduced, `red` the reduction), per layer
    /// `[l * x_grid * y_grid + y * x_grid + x]`.
    pub edges_3d: (Vec<crate::adjust::EdgeState>, Vec<crate::adjust::EdgeState>),
    /// The nets as the router received them, by FastRoute id, and the ids it routed, in order.
    pub router_nets: Vec<RouterNet>,
    pub net_ids: Vec<usize>,
    /// `h_capacity_` / `v_capacity_`, each routing level's direction, and the last run's final
    /// overflow (`totalOverflow()`).
    pub h_capacity: i32,
    pub v_capacity: i32,
    pub layer_dir: Vec<crate::layertable::LayerDir>,
    pub total_overflow: i32,
    /// `saveGuides`' input: `routes_` after F, per net in block order, with the pin facts.
    pub net_routes: Vec<crate::NetRoute>,
    pub jumper_grid: crate::repair_antennas::JumperGrid,
    pub save_options: crate::SaveOptions,
    /// `getLayerEdgeCost` per net, by 0-based layer.
    pub layer_edge_cost: std::collections::BTreeMap<String, Vec<i8>>,
    pub max_routing_layer: i32,
}

/// `makeSteinerTree(net, …)`'s alpha: the net's own, else — ⛔ an else-if chain — the min-HPWL rule
/// whenever one is set (the min-fanout rule is then never consulted), else the min-fanout rule.
/// `hpwl` is the net's [`compute_hpwl`], asked only when the min-HPWL rule decides.
fn steiner_alpha(opts: &RouteOptions, n: &RouterNet, hpwl: Option<i32>) -> f32 {
    if opts.net_alpha.contains_key(&n.name) {
        n.alpha
    } else if let Some((min, a)) = opts.min_hpwl_alpha.filter(|&(h, _)| h > 0) {
        if hpwl.expect("computed for every net the min-HPWL rule decides") >= min { a } else { n.alpha }
    } else {
        match opts.min_fanout_alpha {
            Some((min_fanout, a)) if min_fanout > 0 && n.term_count - 1 >= min_fanout => a,
            _ => n.alpha,
        }
    }
}

/// `makeSteinerTree(net, …)`'s alpha for a net by name, as CUGR's pattern route asks it: the same
/// precedence as [`steiner_alpha`] — the net's own, else the min-HPWL rule when set, else the
/// min-fanout rule — over the global alpha.
pub(crate) fn net_steiner_alpha(db: &Db, opts: &RouteOptions, net: &str) -> Result<f32, String> {
    if let Some(&a) = opts.net_alpha.get(net) {
        return Ok(a);
    }
    if let Some((min, a)) = opts.min_hpwl_alpha.filter(|&(h, _)| h > 0) {
        return Ok(if compute_hpwl(db, net)? >= min { a } else { opts.alpha });
    }
    Ok(match opts.min_fanout_alpha {
        Some((min_fanout, a)) if min_fanout > 0 && db.net_get_term_count(net) as i32 - 1 >= min_fanout => a,
        _ => opts.alpha,
    })
}

/// `SteinerTreeBuilder::computeHPWL`: the bounding box of every instance terminal's average pin
/// location (`getAvgXY`) and every block terminal's first pin location.
///
/// ⛔ An instance that is NONE or UNPLACED is STT-0004, an error. The block-terminal guard is
/// `status != NONE || status != UNPLACED` — always true — so a block terminal never errors.
fn compute_hpwl(db: &Db, net: &str) -> Result<i32, String> {
    let (iterms, bterms) = (db.net_iterms(net), db.net_bterms(net));
    if iterms.is_empty() && bterms.is_empty() {
        return Ok(0);
    }
    let (mut x0, mut y0, mut x1, mut y1) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
    let mut add = |x: i32, y: i32| {
        (x0, x1, y0, y1) = (x0.min(x), x1.max(x), y0.min(y), y1.max(y));
    };
    for it in &iterms {
        // ⛔ The LAST '/': a flattened hierarchical instance name contains '/' itself.
        let (inst, term) = it.rsplit_once('/').ok_or("an instance terminal without a '/'")?;
        let status = db.inst_get_placement_status(inst);
        if status == "NONE" || status == "UNPLACED" {
            return Err(format!("STT-0004: connected to unplaced instance {inst}"));
        }
        // getAvgXY's failure (ODB-0034) leaves the coordinates unset in the reference: refused.
        let (x, y) = db.iterm_avg_xy(inst, term).ok_or_else(|| format!("{it}: no pin shape for getAvgXY — not modelled"))?;
        add(x, y);
    }
    for b in &bterms {
        let (x, y) = db.bterm_first_pin_location(b).ok_or_else(|| format!("{b}: no pin location — not modelled"))?;
        add(x, y);
    }
    Ok((x1 - x0) + (y1 - y0))
}

/// One 2D tree edge at the snapshot: its ends, length, and the route's grid points in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SnapshotEdge {
    pub n1: usize,
    pub n2: usize,
    pub len: i32,
    pub routelen: i32,
    pub grids: Vec<(i32, i32)>,
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
    // initClockNets (findNets(true), at the head of net discovery): with a liberty library the
    // timer's clock nets are re-typed CLOCK — ⛔ IN the database, so the retype persists.
    let mut clock_nets = std::collections::BTreeSet::new();
    if let Some(lib) = &opts.liberty {
        clock_nets = crate::clk_network::find_clk_nets(db, lib, &opts.clock_sources)?;
        for net in &clock_nets {
            db.net_set_sig_type(net, "CLOCK")?;
        }
    }
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
    let lecs: Vec<Vec<i8>> = nets.iter().map(net_range_edge_costs).collect();
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
            has_ndr: n.has_ndr,
            is_clock: n.is_clock,
            is_res_aware: false,
            layer_edge_cost: all_layer_edge_costs(n, num_layers),
            sta_slack: 0.0,
        })
        .collect();
    let slack = vec![(0.0f32, false); nets.len()];
    // getNetSlack: with no clock defined every net is unconstrained — the timer's INF (`1E+30F`).
    // ⛔ With a clock the slacks are the timer's: only a capture answers them (a routed net missing
    // from it is refused); without one a partial-slack pass is refused.
    let unconstrained = vec![1.0e30f32; nets.len()];
    let captured: Vec<Vec<f32>> = match (&opts.liberty, opts.clock_sources.is_empty(), &opts.captured_slacks) {
        (Some(_), false, Some(calls)) => calls
            .iter()
            .map(|by_name| {
                (0..nets.len())
                    .map(|k| match by_name.get(&nets[k].name) {
                        Some(&s) => Ok(s),
                        None if nets[k].is_local => Ok(0.0), // never routed, never read
                        None => Err(format!("net {}: no captured slack — not bound", nets[k].name)),
                    })
                    .collect::<Result<Vec<f32>, String>>()
            })
            .collect::<Result<_, _>>()?,
        _ => Vec::new(),
    };
    // The same for updateSlacks' reads (resistance-aware only).
    let captured_update: Vec<Vec<f32>> = match (&opts.liberty, opts.clock_sources.is_empty(), &opts.captured_update_slacks) {
        (Some(_), false, Some(calls)) => calls
            .iter()
            .map(|by_name| {
                (0..nets.len())
                    .map(|k| match by_name.get(&nets[k].name) {
                        Some(&s) => Ok(s),
                        None if nets[k].is_local => Ok(0.0),
                        None => Err(format!("net {}: no captured updateSlacks slack — not bound", nets[k].name)),
                    })
                    .collect::<Result<Vec<f32>, String>>()
            })
            .collect::<Result<_, _>>()?,
        _ => Vec::new(),
    };
    let update_slacks = match (&opts.liberty, opts.clock_sources.is_empty(), &opts.captured_update_slacks) {
        (Some(_), true, _) => crate::congestion_loop::TimerSlack::Every(&unconstrained),
        (Some(_), false, Some(_)) => crate::congestion_loop::TimerSlack::PerCall(&captured_update),
        _ => crate::congestion_loop::TimerSlack::None,
    };
    // preProcessTechLayers: each routing layer (by level, up to the router's layers) and the cut
    // layer above it — their widths and resistances as the database holds them now (after any
    // set_layer_rc).
    // ⬜ Resistance-aware pricing reads an NDR net's width PER LAYER (`getWireResistance`); the
    // pricing's `WireNet` carries one width, so the pair is refused rather than priced at the
    // default width. No upstream case combines them.
    if opts.resistance_aware && nets.iter().any(|n| n.has_ndr) {
        return Err("resistance-aware routing of a net with an NDR: the per-layer NDR width is not wired".into());
    }
    let res_aware = if opts.resistance_aware {
        let mut tech = crate::pricing::TechLayers { dbu_per_micron: db.tech_get_db_units_per_micron(), width: Vec::new(), resistance: Vec::new(), via_resistance: Vec::new() };
        for level in 1..=num_layers as i32 {
            let l = t.tech.routing_layers.iter().find(|r| r.routing_level == level).ok_or_else(|| format!("no routing layer at level {level}"))?;
            tech.width.push(db.layer_get_width(&l.name) as i32);
            tech.resistance.push(db.layer_get_resistance(&l.name));
            let cut = db.layer_get_upper_layer(&l.name);
            tech.via_resistance.push((!cut.is_empty()).then(|| db.layer_get_resistance(&cut)));
        }
        Some(crate::run::ResAwareInputs { tech, tile_size: t.core.tile_size, fixed_percentage: opts.res_aware_nets_percentage, update_slacks })
    } else {
        None
    };
    let timer_slack = match (&opts.liberty, opts.clock_sources.is_empty(), &opts.captured_slacks) {
        (Some(_), true, _) => crate::congestion_loop::TimerSlack::Every(&unconstrained),
        (Some(_), false, Some(_)) => crate::congestion_loop::TimerSlack::PerCall(&captured),
        _ => crate::congestion_loop::TimerSlack::None,
    };
    // makeSteinerTree(net, …): the net's own alpha, else — ⛔ an else-if chain — the min-HPWL rule
    // whenever one is set (the min-fanout rule is then never consulted), else the min-fanout rule.
    let min_hpwl = opts.min_hpwl_alpha.filter(|&(h, _)| h > 0);
    let hpwl: Vec<Option<i32>> = match min_hpwl {
        Some(_) => nets
            .iter()
            .enumerate()
            .map(|(k, n)| {
                // Asked only of routed nets with no alpha of their own (every one reaches R5).
                if n.is_local || opts.net_alpha.contains_key(&n.name) || n.alpha <= 0.0 { Ok(None) } else { compute_hpwl(db, &n.name).map(Some) }.map_err(|e| format!("net {}: {e}", nets[k].name))
            })
            .collect::<Result<_, _>>()?,
        None => Vec::new(),
    };
    let stt_net = |id: usize| stt(&pins[id].0, &pins[id].1, nets[id].root, steiner_alpha(opts, &nets[id], hpwl.get(id).copied().flatten()));
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
        resistance_aware: opts.resistance_aware,
        liberty: opts.liberty.is_some(),
        timer_slack,
        res_aware,
        origin: crate::routes::GridOrigin { tile_size: t.core.tile_size, x_corner: t.core.area.x_min, y_corner: t.core.area.y_min },
        db_id: &db_id,
        resume: None,
    };
    // ⛔ The parasitics read the estimator's table where it has a value, and the technology's own
    // only where it has none (`MakeWireParasitics::layerRC`).
    let layer_rc = |db: &Db| -> crate::parasitics::LayerRC {
        let mut rc = crate::parasitics::LayerRC { dbu_per_micron: db.tech_get_db_units_per_micron(), ..Default::default() };
        for l in &t.tech.routing_layers {
            let (lvl, name) = (l.routing_level, &l.name);
            rc.width.insert(lvl, db.layer_get_width(name) as i32);
            rc.resistance.insert(lvl, db.layer_get_resistance(name));
            rc.capacitance.insert(lvl, db.layer_get_capacitance(name));
            rc.edge_capacitance.insert(lvl, db.layer_get_edge_capacitance(name));
            if let Some(&(res, cap)) = opts.layer_rc.get(&lvl) {
                rc.table_res.insert(lvl, res);
                rc.table_cap.insert(lvl, cap);
            }
            let cut = db.layer_get_upper_layer(name);
            if !cut.is_empty() {
                rc.cut_resistance.insert(lvl, db.layer_get_resistance(&cut));
                if let Some(&res) = opts.via_rc.get(&cut) {
                    rc.cut_table_res.insert(lvl, res);
                }
            }
        }
        rc
    };
    // The run's final overflow (after R19), for the congestion verdict — and the 2D trees at the
    // first partial-slack call, which is the state `estimateAllGlobalRouteParasitics` reads
    // (`getPartialRoutes` → `getPlanarRoutes`) when the router asks the timer for slacks.
    struct Observer {
        overflow: i32,
        cnp: f32,
        trees: Option<Vec<Option<crate::brk_rsmt::StTree>>>,
        /// The 3D edges at R19 — nothing after it changes them, so they are what antenna repair's
        /// jumper pass reads (`hasAvailableResources`) and charges.
        g3: Option<Graph3d>,
        /// The 2D graph at R19, for the same reason: an incremental re-route starts from it.
        g2d: Option<crate::graph2d::Graph2d>,
    }
    impl RunObserver for Observer {
        fn stage(&mut self, s: Stage<'_>, g2d: &crate::graph2d::Graph2d, g3: Option<&Graph3d>, st: &[NetState]) -> bool {
            match s {
                Stage::B19(fin) => {
                    self.overflow = fin.overflow.total;
                    self.g3 = g3.cloned();
                    self.g2d = Some(g2d.clone());
                }
                Stage::Loop(crate::congestion_loop::LoopEvent::Before { params, .. }) if params.ordering && self.cnp != 0.0 && self.trees.is_none() => {
                    self.trees = Some(st.iter().map(|n| n.tree.clone()).collect());
                }
                _ => {}
            }
            true
        }
    }
    let mut state = vec![NetState::default(); nets.len()];
    let mut ov = Observer { overflow: 0, cnp: t.config.critical_nets_percentage, trees: None, g3: None, g2d: None };
    let routes = match fastroute_run(&inp, &mut state, &mut ov)? {
        RunEnd::Routed(r) => r,
        RunEnd::Stopped => return Err("run() stopped".into()),
    };
    // est::estimateAllGlobalRouteParasitics, on the planar routes the first partial-slack call saw.
    let mut parasitics = std::collections::BTreeMap::new();
    let mut parasitic_pins: std::collections::BTreeMap<String, Vec<crate::parasitics::PinGridLocation>> = std::collections::BTreeMap::new();
    let mut planar_routes: std::collections::BTreeMap<String, Vec<crate::parasitics::Segment>> = std::collections::BTreeMap::new();
    let mut snapshot_edges: std::collections::BTreeMap<String, Vec<SnapshotEdge>> = std::collections::BTreeMap::new();
    if let Some(trees) = &ov.trees {
        let rc = layer_rc(db);
        let origin = crate::routes::GridOrigin { tile_size: t.core.tile_size, x_corner: t.core.area.x_min, y_corner: t.core.area.y_min };
        for &id in &net_ids {
            let n = &nets[id];
            let Some(tree) = trees[id].as_ref() else { continue };
            let edges: Vec<crate::planar_route::PlanarEdge<'_>> = tree
                .edges
                .iter()
                .zip(&tree.routes)
                .map(|(e, route)| crate::planar_route::PlanarEdge { len: e.len, routelen: route.routelen, grids: &route.grids })
                .collect();
            let route = crate::planar_route::planar_route(&edges, (n.min_layer - 1) as usize, &layer_dir, origin);
            let pins: Vec<crate::parasitics::PinGridLocation> = n
                .net_pins
                .iter()
                .zip(&n.pin_is_driver)
                .map(|(p, &is_driver)| crate::parasitics::PinGridLocation { name: p.name.clone(), is_port: p.is_port, is_driver, pt: p.position, grid_pt: p.on_grid, conn_layer: p.connection_layer })
                .collect();
            let np = crate::parasitics::NetParasitics {
                name: &n.name,
                route: &route,
                pins: &pins,
                net_min_layer: n.min_layer,
                min_routing_layer: t.min_routing_layer,
                ndr_width: n.ndr_widths.as_ref(),
                attach: crate::parasitics::PinAttach::Planar,
            };
            parasitics.insert(n.name.clone(), crate::parasitics::estimate_net(&np, &rc));
            parasitic_pins.insert(n.name.clone(), pins.clone());
            planar_routes.insert(n.name.clone(), route);
            snapshot_edges.insert(
                n.name.clone(),
                tree.edges
                    .iter()
                    .zip(&tree.routes)
                    .map(|(e, rt)| SnapshotEdge { n1: e.n1, n2: e.n2, len: e.len, routelen: rt.routelen, grids: rt.grids[..=(rt.routelen.max(0) as usize).min(rt.grids.len().saturating_sub(1))].to_vec() })
                    .collect(),
            );
        }
    }
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
    // `estimate_parasitics -global_routing` after the run: the same builder over the SAVED routes,
    // where each pin attaches from its own connection layer.
    let mut routed_parasitics = std::collections::BTreeMap::new();
    if ov.trees.is_some() {
        let rc = layer_rc(db);
        for n in &nets {
            let Some(segs) = by_name.get(&n.name) else { continue };
            let route: Vec<crate::parasitics::Segment> = segs
                .iter()
                .map(|g| crate::parasitics::Segment { init_x: g.init_x, init_y: g.init_y, init_layer: g.init_layer, final_x: g.final_x, final_y: g.final_y, final_layer: g.final_layer })
                .collect();
            // ⛔ Over the SAVED routes, which include the local nets `getPartialRoutes` leaves out.
            let pins: Vec<crate::parasitics::PinGridLocation> = n
                .net_pins
                .iter()
                .zip(&n.pin_is_driver)
                .map(|(p, &is_driver)| crate::parasitics::PinGridLocation { name: p.name.clone(), is_port: p.is_port, is_driver, pt: p.position, grid_pt: p.on_grid, conn_layer: p.connection_layer })
                .collect();
            parasitic_pins.entry(n.name.clone()).or_insert_with(|| pins.clone());
            let np = crate::parasitics::NetParasitics {
                name: &n.name,
                route: &route,
                pins: &pins,
                net_min_layer: n.min_layer,
                min_routing_layer: t.min_routing_layer,
                ndr_width: n.ndr_widths.as_ref(),
                attach: crate::parasitics::PinAttach::Routed,
            };
            routed_parasitics.insert(n.name.clone(), crate::parasitics::estimate_net(&np, &rc));
        }
    }
    // X — saveGuides over the block's nets, in its order.
    let total_overflow = ov.overflow;
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
    let jumper_grid = crate::repair_antennas::JumperGrid { grid, x_grids: t.core.x_grids, y_grids: t.core.y_grids };
    // A net the run demoted to soft NDR (`setSoftNDR`) carries edge cost 1 and per-layer cost 1 into
    // every later command that re-routes or releases it.
    let soft = |k: usize| state.get(k).is_some_and(|s| s.soft_ndr);
    let layer_edge_cost = nets.iter().zip(&attrs).enumerate()
        .map(|(k, (n, a))| (n.name.clone(), if soft(k) { vec![1; a.layer_edge_cost.len()] } else { a.layer_edge_cost.clone() }))
        .collect();
    let router_nets: Vec<RouterNet> = nets.iter().enumerate()
        .map(|(k, n)| if soft(k) { RouterNet { edge_cost: 1, ..n.clone() } } else { n.clone() })
        .collect();
    let after = AfterRoute {
        final_3d: ov.g3,
        final_2d: ov.g2d,
        final_state: state,
        red_h,
        red_v,
        caps,
        edges_3d: (e.h3.clone(), e.v3.clone()),
        router_nets,
        net_ids,
        h_capacity: t.capacities.h_capacity,
        v_capacity: t.capacities.v_capacity,
        layer_dir: layer_dir.clone(),
        total_overflow,
        net_routes,
        jumper_grid,
        save_options: save,
        layer_edge_cost,
        max_routing_layer: t.max_routing_layer,
    };
    Ok(RouteResult { guides, layer_names, total_overflow, guide_is_congested, routes: raw_routes, clock_nets, parasitics, routed_parasitics, parasitic_pins, planar_routes, snapshot_edges, log, after })
}

/// `VYGI|<tag>|…` — FastRoute's whole state in the format `grt-incr-trace.py` patches into
/// `FastRouteCore::run` (`end` at a full run's exit, `incr` / `incrend` at an incremental run's
/// entry and exit): every 2D edge, the used-grid sets, every 3D edge, then `net_ids_` in order.
/// `updateNetResources(net, release)` → `updateResources` per wire segment →
/// `updateEdge2DAnd3DUsage(x0, y0, x1, y1, layer, used, net)`: the segment's span in TILES
/// (`dbuToTile` of its min and max corners), walked `x0..x1` (or `y0..y1`) exclusive; each edge
/// charged `used × edge cost` in 2D through the NDR-aware cost and `used × the layer's edge cost` in
/// 3D. A via is skipped.
///
/// ⚠️ Horizontal is tested first (`y1 == y2`), so a span inside one tile is "horizontal" and
/// charges nothing. The 3D usage is `uint16_t` charged by `int8_t` products — it wraps.
fn update_net_resources(g2: &mut crate::graph2d::Graph2d, g3: &mut Graph3d, grid: &crate::repair_antennas::JumperGrid, n: &RouterNet, id: usize, lec: &[i8], segments: &[crate::GSegment], used: i32) {
    let (lo, hi) = ((n.min_layer - 1).max(0) as usize, (n.max_layer - 1).max(0) as usize);
    let nn = crate::ndr_cost::NdrCostNet { id, edge_cost: n.edge_cost, min_layer: lo, max_layer: hi, layer_edge_cost: Some(net_range_edge_costs(n)), soft_ndr: false };
    let xg = g3.x_grid;
    for s in segments.iter().filter(|s| !s.is_via()) {
        let x0 = grid.dbu_to_tile(s.init_x.min(s.final_x), true);
        let y0 = grid.dbu_to_tile(s.init_y.min(s.final_y), false);
        let x1 = grid.dbu_to_tile(s.final_x.max(s.init_x), true);
        let y1 = grid.dbu_to_tile(s.final_y.max(s.init_y), false);
        let k = (s.final_layer - 1) as usize;
        let d3 = (used * i32::from(lec[k])) as u16;
        let d2 = f64::from(used * i32::from(n.edge_cost));
        let mut u = g2.for_net(&nn);
        if y0 == y1 {
            for x in x0..x1 {
                crate::estimate::Usage2d::update_usage_h(&mut u, x, y0, d2);
                let c = &mut g3.h_usage[k][y0 as usize * xg + x as usize];
                *c = c.wrapping_add(d3);
            }
        } else if x0 == x1 {
            for y in y0..y1 {
                crate::estimate::Usage2d::update_usage_v(&mut u, x0, y, d2);
                let c = &mut g3.v_usage[k][y as usize * xg + x0 as usize];
                *c = c.wrapping_add(d3);
            }
        }
    }
}

/// `repairAntennas` with no route in the session (`!initialized_`): the routes come from the
/// database's guides and the router is set up around them — the state antenna repair then starts
/// from. In the reference's order:
///
/// 1. `loadGuidesFromDB` (reached through `check_antennas` → `haveRoutes`): per net in block order,
///    each guide [`box_to_global_routing`](crate::restore::box_to_global_routing), then
///    `dedupViaSegments`, `addImplicitVias`, `mergeSegments`, and `ensurePinsPositions`;
/// 2. `initFastRoute` — the same setup a route makes (layers, tracks, grid, capacities,
///    adjustments, the nets), with EMPTY usage: `fastroute_->clear()` drops the 3D usage
///    `updateEdgesUsage` charged in step 1, so that charge is not modelled;
/// 3. per net of `routes_` (odb-id order) that is not detail-routed, `updateNetResources`: every
///    wire segment's tiles charged once — 2D through the NDR-aware cost at the net's edge cost, 3D
///    at the layer's edge cost — and the net marked `areSegmentsRestored`.
///
/// ⛔ Refused rather than guessed: a pin no restored segment covers (`ensurePinsPositions`' repair
/// of pin positions is not modelled), a detail-routed net, an NDR net
/// (`disableCongestedNDRNetsFromRoutes`), a congested guide, and a guide on a net the router does
/// not know (GRT-0127).
pub fn restore_for_repair(db: &mut Db, opts: &RouteOptions) -> Res<AfterRoute> {
    use crate::brk_rsmt::{CapLayer, Caps3D, NetState};
    let t = setup_tech(db, opts)?;
    let mut log = t.log.clone();
    let adj = setup_adjust(db, &t, opts, &mut log)?;
    let mut e = adj.edges;
    if let Some(lib) = &opts.liberty {
        for net in crate::clk_network::find_clk_nets(db, lib, &opts.clock_sources)? {
            db.net_set_sig_type(&net, "CLOCK")?;
        }
    }
    let nets = setup_nets(db, &t, &mut e, adj.has_macros_or_pads, opts, &mut log)?;
    let (xg, yg) = (e.x_grid as usize, e.y_grid as usize);
    // The router's grid, as route_design lays it out.
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
    let num_layers = e.num_layers as usize;
    let max_layer = nets.iter().filter(|n| !n.is_local).map(|n| (n.max_layer - 1) as usize).max().unwrap_or(0);
    let mut g2 = crate::run::initial_graph_2d(xg, yg, &caps, &cap_h, &cap_v, num_layers.max(max_layer + 1));
    let mut g3 = crate::run::graph_3d(&caps, xg, yg);

    // 1. loadGuidesFromDB — routes_ per net, in block order.
    let tile = t.core.tile_size;
    let block_min = db.block_get_min_routing_layer();
    let pins_of = |n: &RouterNet| -> Vec<crate::Pin> { n.net_pins.iter().map(|p| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect() };
    let mut routes: std::collections::BTreeMap<String, Vec<crate::GSegment>> = std::collections::BTreeMap::new();
    let order = db.net_names();
    for name in &order {
        for k in 0..db.num_net_get_guides(name) {
            if db.guide_is_congested(name, k) {
                return Err(format!("net {name}: a congested guide — restoring a congested routing is not modelled").into());
            }
            let bx = (db.guide_get_box_x_min(name, k), db.guide_get_box_y_min(name, k), db.guide_get_box_x_max(name, k), db.guide_get_box_y_max(name, k));
            let layer = db.layer_get_routing_level(&db.guide_get_layer(name, k));
            let via_layer = db.layer_get_routing_level(&db.guide_get_via_layer(name, k));
            crate::restore::box_to_global_routing(bx, layer, via_layer, tile, routes.entry(name.clone()).or_default());
        }
    }
    for (name, route) in routes.iter_mut() {
        let n = nets.iter().find(|n| &n.name == name).ok_or_else(|| format!("[ERROR GRT-0127] net_id for db_net {name} not found — not modelled"))?;
        crate::restore::dedup_via_segments(route);
        crate::restore::add_implicit_vias(route);
        let grid_pins: Vec<crate::findrouting::GridPin> = n.net_pins.iter().map(|p| (p.on_grid.0, p.on_grid.1, p.connection_layer)).collect();
        crate::findrouting::merge_segments(&grid_pins, route, block_min);
        // ensurePinsPositions: only a net some pin of which no segment covers has anything to do.
        let uncovered = crate::restore::net_is_covered(route, &pins_of(n));
        if !uncovered.is_empty() {
            return Err(format!("net {name}: {} pin(s) not covered by the restored guides — ensurePinsPositions is not modelled", uncovered.len()).into());
        }
    }

    // 3. updateNetResources per net of routes_, in odb-id (block) order.
    let jumper_grid = crate::repair_antennas::JumperGrid { grid: crate::Grid { tile_size: tile, area: t.core.area }, x_grids: t.core.x_grids, y_grids: t.core.y_grids };
    let mut state = vec![NetState::default(); nets.len()];
    for name in order.iter().filter(|n| routes.contains_key(*n)) {
        if db.net_get_wire_type(name) == "ROUTED" && !db.net_is_special(name) && db.net_has_wire(name) {
            return Err(format!("net {name}: detail-routed — its usage from wires is not modelled").into());
        }
        let id = nets.iter().position(|n| &n.name == name).expect("checked above");
        let n = &nets[id];
        if n.has_ndr {
            return Err(format!("net {name}: an NDR net restored from guides — disableCongestedNDRNetsFromRoutes is not modelled").into());
        }
        update_net_resources(&mut g2, &mut g3, &jumper_grid, n, id, &all_layer_edge_costs(n, num_layers), &routes[name], 1);
        state[id].segments_restored = true;
    }

    let layer_dir: Vec<crate::layertable::LayerDir> = (1..=num_layers as i32)
        .map(|l| match t.tech.routing_layers.iter().find(|r| r.routing_level == l).and_then(|r| r.direction) {
            Some(crate::capacity::Direction::Horizontal) => crate::layertable::LayerDir::Horizontal,
            Some(crate::capacity::Direction::Vertical) => crate::layertable::LayerDir::Vertical,
            None => crate::layertable::LayerDir::Other,
        })
        .collect();
    let net_routes: Vec<crate::NetRoute> = order
        .iter()
        .filter_map(|name| {
            let n = nets.iter().find(|n| &n.name == name)?;
            Some(crate::NetRoute { name: name.clone(), segments: routes.get(name).cloned().unwrap_or_default(), pins: pins_of(n), is_local: n.is_local })
        })
        .collect();
    let save_options = crate::SaveOptions { guide_is_congested: false, origin_x: opts.grid_origin.0, origin_y: opts.grid_origin.1, min_routing_layer: t.min_routing_layer };
    let layer_edge_cost = nets.iter().map(|n| (n.name.clone(), all_layer_edge_costs(n, num_layers))).collect();
    let net_ids: Vec<usize> = (0..nets.len()).filter(|&k| !nets[k].is_local).collect();
    Ok(AfterRoute {
        final_3d: Some(g3),
        final_2d: Some(g2),
        final_state: state,
        red_h,
        red_v,
        caps,
        edges_3d: (e.h3.clone(), e.v3.clone()),
        router_nets: nets,
        net_ids,
        h_capacity: t.capacities.h_capacity,
        v_capacity: t.capacities.v_capacity,
        layer_dir,
        total_overflow: 0,
        net_routes,
        jumper_grid,
        save_options,
        layer_edge_cost,
        max_routing_layer: t.max_routing_layer,
    })
}

pub fn router_state_text(tag: &str, a: &AfterRoute) -> Result<String, String> {
    use std::fmt::Write;
    let (g2, g3) = match (&a.final_2d, &a.final_3d) {
        (Some(g2), Some(g3)) => (g2, g3),
        _ => return Err("the run did not reach R19".into()),
    };
    let (xg, yg, nl) = (g2.est.x_grids, g2.est.y_grids, g3.num_layers);
    let mut t = String::new();
    let _ = writeln!(t, "VYGI|{tag}|grid|{xg}|{yg}|{nl}");
    for x in 0..xg.saturating_sub(1) {
        for y in 0..yg {
            let i = y * xg + x;
            let est = crate::antenna_check::fmt_g17(g2.est.h(x, y));
            let _ = writeln!(t, "VYGI|{tag}|h|{x},{y}|cap={}|usage={}|red={}|est={est}|last={}|cong={}", g2.cap_h[i], g2.est.usage_h(x, y), a.red_h[i], g2.est.last_usage_h(x, y), g2.est.cong_cnt_h(x, y));
        }
    }
    for x in 0..xg {
        for y in 0..yg.saturating_sub(1) {
            let i = y * xg + x;
            let est = crate::antenna_check::fmt_g17(g2.est.v(x, y));
            let _ = writeln!(t, "VYGI|{tag}|v|{x},{y}|cap={}|usage={}|red={}|est={est}|last={}|cong={}", g2.cap_v[i], g2.est.usage_v(x, y), a.red_v[i], g2.est.last_usage_v(x, y), g2.est.cong_cnt_v(x, y));
        }
    }
    let used = |s: &std::collections::BTreeSet<(i32, i32)>| s.iter().map(|(x, y)| format!("{x},{y};")).collect::<String>();
    let _ = writeln!(t, "VYGI|{tag}|usedh|{}", used(&g2.used_h));
    let _ = writeln!(t, "VYGI|{tag}|usedv|{}", used(&g2.used_v));
    let n = xg * yg;
    for k in 0..nl {
        for y in 0..yg {
            for x in 0..xg {
                let i = y * xg + x;
                let (h, v) = (&a.edges_3d.0[k * n + i], &a.edges_3d.1[k * n + i]);
                let _ = writeln!(t, "VYGI|{tag}|h3|{k}|{x},{y}|cap={}|usage={}|red={}", g3.h_cap[k][i], g3.h_usage[k][i], h.red);
                let _ = writeln!(t, "VYGI|{tag}|v3|{k}|{x},{y}|cap={}|usage={}|red={}", g3.v_cap[k][i], g3.v_usage[k][i], v.red);
            }
        }
    }
    for &id in &a.net_ids {
        let r = &a.router_nets[id];
        let lc: String = match a.layer_edge_cost.get(&r.name) {
            Some(v) => v.iter().map(|c| format!("{c},")).collect(),
            None => (0..nl).map(|_| "1,".to_string()).collect(),
        };
        let pins: String = r.pins.iter().map(|(x, y, l)| format!("{x},{y},{};", l - 1)).collect();
        let _ = writeln!(t, "VYGI|{tag}|net|{id}|{}|drv={}|min={}|max={}|cost={}|lcost={lc}|clock={}|pins={pins}", r.name, r.root, r.min_layer - 1, r.max_layer - 1, r.edge_cost, r.is_clock as i32);
    }
    Ok(t)
}

/// Where the incremental re-route lets an observer look: `(tag, state)` at `run()`'s entry
/// (`incr`) and exit (`incrend`), as the reference's trace does.
pub type IncrObserver<'a> = &'a mut dyn FnMut(&str, &AfterRoute);

/// `updateDirtyRoutesFastRoute`, after a diode insertion and its legalization dirtied `dirty`
/// (`dirty_nets_`: a `PtrSet`, so in dbNet order — the caller's). Returns the nets it re-routed.
///
/// The reference's call sequence and nothing else; each stage is its own function below, named
/// after the reference's.
///
/// ⛔ Refused where the reference goes on: a resistance-aware net (`isResAware` skips the filter) and
/// overflow after the re-route (the incremental congestion loop). A jumpered net releases through
/// the tree the jumper pass relayered ([`crate::repair_antennas::update_route_grids_layer`]).
pub fn update_dirty_routes_fast_route(db: &mut Db, opts: &RouteOptions, a: &mut AfterRoute, dirty: &[String], stt: SteinerBuilder<'_>, flutes: crate::brk_rsmt::Flutes<'_>, obs: IncrObserver<'_>) -> Res<Vec<String>> {
    if dirty.is_empty() {
        return Ok(Vec::new());
    }
    // clearNetsToRoute, then updateDirtyNets.
    let fresh = update_net_pins(db, opts, a)?;
    let dirty_nets = update_dirty_nets(a, &fresh, dirty)?;
    if dirty_nets.is_empty() {
        return Ok(Vec::new());
    }
    // setCriticalNetsPercentage(0); initFastRouteIncr; findRouting; mergeResults.
    init_fast_route_incr(a, &fresh, &dirty_nets);
    let routes = find_routing(db, opts, a, &dirty_nets, stt, flutes, obs)?;
    merge_results(a, routes);
    if a.total_overflow > 0 && !opts.allow_congestion {
        return Err("the incremental re-route left overflow: its congestion loop is not modelled".into());
    }
    Ok(dirty_nets.iter().map(|&id| a.router_nets[id].name.clone()).collect())
}

/// `updateNetPins`, for every net: the router's nets as the database stands now (discovery, pins,
/// layer ranges), by FastRoute id. ⚠️ Recomputed whole — the database has moved under it — and
/// checked to keep the ids the first run gave (a diode joins an existing net; it adds none).
fn update_net_pins(db: &mut Db, opts: &RouteOptions, a: &AfterRoute) -> Res<Vec<RouterNet>> {
    let t = setup_tech(db, opts)?;
    let mut log = t.log.clone();
    let adj = setup_adjust(db, &t, opts, &mut log)?;
    let mut e = adj.edges;
    let nets = setup_nets(db, &t, &mut e, adj.has_macros_or_pads, opts, &mut log)?;
    if nets.len() != a.router_nets.len() || nets.iter().zip(&a.router_nets).any(|(n, m)| n.name != m.name) {
        return Err("the router's nets changed under the incremental re-route — new nets are not modelled".into());
    }
    Ok(nets)
}

/// `updateDirtyNets`: of the dirty nets, those whose pins moved — `pinPositionsChanged`, the
/// multiset of `(on-grid x, y, connection layer)` against the positions the net was dirtied with
/// (`saveLastPinPositions`, from the router's own stale pins: the first run's) — are released
/// (`clearNetRoute`) and their routes cleared; the rest keep theirs. In dbNet order.
fn update_dirty_nets(a: &mut AfterRoute, fresh: &[RouterNet], dirty: &[String]) -> Res<Vec<usize>> {
    let mut out = Vec::new();
    for name in dirty {
        let Some(id) = a.router_nets.iter().position(|n| &n.name == name) else { continue }; // not in db_net_map_
        let key = |n: &RouterNet| n.net_pins.iter().map(|p| (p.on_grid.0, p.on_grid.1, p.connection_layer)).collect::<Vec<_>>();
        if crate::netlist::pin_positions_changed(&key(&a.router_nets[id]), &key(&fresh[id])) {
            // A net restored from guides has no tree: `updateNetResources(net, true)` over its
            // current routes_ releases it, and it is restored no longer.
            if a.final_state[id].segments_restored {
                let segs = a.net_routes.iter().find(|r| &r.name == name).map(|r| r.segments.clone()).unwrap_or_default();
                let lec = a.layer_edge_cost.get(name).cloned().unwrap_or_default();
                if let (Some(g2), Some(g3)) = (a.final_2d.as_mut(), a.final_3d.as_mut()) {
                    update_net_resources(g2, g3, &a.jumper_grid, &a.router_nets[id], id, &lec, &segs, -1);
                }
                a.final_state[id].segments_restored = false;
            } else {
                clear_net_route(a, id);
            }
            if let Some(r) = a.net_routes.iter_mut().find(|r| &r.name == name) {
                r.segments.clear();
            }
            out.push(id);
        }
    }
    Ok(out)
}

/// `clearNetRoute` → `releaseNetResources`: walk the net's 3D tree and take back, per unit step on
/// one layer, `edgeCost` from the 2D edge (the NDR-aware `updateUsage`) and the layer's edge cost
/// from the 3D edge; then drop the tree.
fn clear_net_route(a: &mut AfterRoute, id: usize) {
    let (Some(g2), Some(g3)) = (a.final_2d.as_mut(), a.final_3d.as_mut()) else { return };
    let r = &a.router_nets[id];
    let lec = a.layer_edge_cost.get(&r.name).cloned().unwrap_or_default();
    let (min, max) = ((r.min_layer - 1) as usize, (r.max_layer - 1) as usize);
    let net = crate::ndr_cost::NdrCostNet { id, edge_cost: r.edge_cost, min_layer: min, max_layer: max, layer_edge_cost: Some(lec.get(min..=max).map_or_else(|| vec![1; max + 1 - min], <[i8]>::to_vec)), soft_ndr: false };
    let st = &mut a.final_state[id];
    if let Some(t) = st.tree3d.as_ref() {
        let xg = g3.x_grid;
        for e in &t.edges {
            for i in 0..e.routelen.max(0) as usize {
                let (p, q) = (e.grids[i], e.grids[i + 1]);
                if p.layer != q.layer {
                    continue;
                }
                let k = p.layer as usize;
                let cost = i32::from(lec.get(k).copied().unwrap_or(1));
                let mut u = g2.for_net(&net);
                if p.x == q.x {
                    let y = p.y.min(q.y);
                    crate::estimate::Usage2d::update_usage_v(&mut u, i32::from(p.x), i32::from(y), -f64::from(r.edge_cost));
                    let c = &mut g3.v_usage[k][y as usize * xg + p.x as usize];
                    *c = (i32::from(*c) - cost) as u16;
                } else if p.y == q.y {
                    let x = p.x.min(q.x);
                    crate::estimate::Usage2d::update_usage_h(&mut u, i32::from(x), i32::from(p.y), -f64::from(r.edge_cost));
                    let c = &mut g3.h_usage[k][p.y as usize * xg + x as usize];
                    *c = (i32::from(*c) - cost) as u16;
                }
            }
        }
    }
    st.tree = None;
    st.tree3d = None;
}

/// `initFastRouteIncr` → `initNetlist(nets, true)`: `net_ids_` becomes the re-routed nets in order,
/// each re-added (`addNet` keeps its id, resets it) with its pins as they stand — a net with fewer
/// than two pins, or a local one, is added but not routed.
fn init_fast_route_incr(a: &mut AfterRoute, fresh: &[RouterNet], dirty_nets: &[usize]) {
    a.net_ids.clear();
    for &id in dirty_nets {
        a.router_nets[id] = fresh[id].clone();
        a.final_state[id] = crate::brk_rsmt::NetState::default();
        let n = &a.router_nets[id];
        if n.net_pins.len() > 1 && !n.is_local {
            a.net_ids.push(id);
        }
    }
}

/// `findRouting(dirty_nets, …)`: `run()` from the state the first run, the jumpers and the rip-ups
/// left, then the post-processing over those nets (remaining guides, pad pins, `mergeSegments`).
fn find_routing(db: &Db, opts: &RouteOptions, a: &mut AfterRoute, dirty_nets: &[usize], stt: SteinerBuilder<'_>, flutes: crate::brk_rsmt::Flutes<'_>, obs: IncrObserver<'_>) -> Res<std::collections::BTreeMap<String, Vec<crate::GSegment>>> {
    use crate::brk_rsmt::RsmtNet;
    use crate::run::{fastroute_run, RunEnd, RunInputs, RunObserver, Stage};
    obs("incr", a);
    let nets = a.router_nets.clone();
    let (xg, yg) = (a.jumper_grid.x_grids as usize, a.jumper_grid.y_grids as usize);
    let num_layers = a.caps.layers.len();
    let pins: Vec<(Vec<i32>, Vec<i32>)> = nets.iter().map(|n| n.pins.iter().map(|p| (p.0, p.1)).unzip()).collect();
    let lecs: Vec<Vec<i8>> = nets.iter().map(net_range_edge_costs).collect();
    let rnets: Vec<RsmtNet<'_>> = nets
        .iter()
        .enumerate()
        .map(|(k, n)| RsmtNet { pins_x: &pins[k].0, pins_y: &pins[k].1, alpha: n.alpha, edge_cost: n.edge_cost, min_layer: (n.min_layer - 1) as usize, max_layer: (n.max_layer - 1) as usize, layer_edge_cost: &lecs[k] })
        .collect();
    let attrs: Vec<NetLayerAttrs> = nets
        .iter()
        .map(|n| NetLayerAttrs { pin_layers: n.pins.iter().map(|p| (p.2 - 1) as i16).collect(), has_ndr: n.has_ndr, is_clock: n.is_clock, is_res_aware: false, layer_edge_cost: all_layer_edge_costs(n, num_layers), sta_slack: 0.0 })
        .collect();
    let slack = vec![(0.0f32, false); nets.len()];
    // The min-HPWL rule reads the instances where the legalization left them.
    let hpwl: Vec<Option<i32>> = match opts.min_hpwl_alpha.filter(|&(h, _)| h > 0) {
        Some(_) => (0..nets.len())
            .map(|k| {
                let n = &nets[k];
                if !a.net_ids.contains(&k) || opts.net_alpha.contains_key(&n.name) || n.alpha <= 0.0 { Ok(None) } else { compute_hpwl(db, &n.name).map(Some) }
            })
            .collect::<Result<_, _>>()?,
        None => Vec::new(),
    };
    let stt_net = |id: usize| stt(&pins[id].0, &pins[id].1, nets[id].root, steiner_alpha(opts, &nets[id], hpwl.get(id).copied().flatten()));
    let layer_dir = a.layer_dir.clone();
    let db_id: Vec<u32> = (0..nets.len() as u32).collect();
    let (g2, g3) = (a.final_2d.clone().ok_or("no 2D graph to resume")?, a.final_3d.clone().ok_or("no 3D graph to resume")?);
    let net_ids = a.net_ids.clone();
    let inp = RunInputs {
        x_grid: xg,
        y_grid: yg,
        h_capacity: a.h_capacity,
        v_capacity: a.v_capacity,
        red_h: &a.red_h,
        red_v: &a.red_v,
        cap_h: &g2.cap_h,
        cap_v: &g2.cap_v,
        entry: g2.est.clone(),
        caps: &a.caps,
        net_ids: &net_ids,
        nets: &rnets,
        attrs: &attrs,
        slack: &slack,
        stt: &stt_net,
        flutes,
        overflow_iterations: opts.congestion_iterations,
        // setCriticalNetsPercentage(0) for the incremental run.
        critical_nets_percentage: 0.0,
        layer_dir: &layer_dir,
        resistance_aware: false,
        liberty: opts.liberty.is_some(),
        timer_slack: crate::congestion_loop::TimerSlack::None,
        res_aware: None,
        origin: crate::routes::GridOrigin { tile_size: a.jumper_grid.grid.tile_size, x_corner: a.jumper_grid.grid.area.x_min, y_corner: a.jumper_grid.grid.area.y_min },
        db_id: &db_id,
        resume: Some((&g2, &g3)),
    };
    struct Observer {
        overflow: i32,
        g2d: Option<crate::graph2d::Graph2d>,
        g3: Option<Graph3d>,
    }
    impl RunObserver for Observer {
        fn stage(&mut self, s: Stage<'_>, g2d: &crate::graph2d::Graph2d, g3: Option<&Graph3d>, _: &[crate::brk_rsmt::NetState]) -> bool {
            if let Stage::B19(fin) = s {
                self.overflow = fin.overflow.total;
                (self.g2d, self.g3) = (Some(g2d.clone()), g3.cloned());
            }
            true
        }
    }
    let mut ov = Observer { overflow: 0, g2d: None, g3: None };
    let mut state = std::mem::take(&mut a.final_state);
    let end = fastroute_run(&inp, &mut state, &mut ov);
    a.final_state = state;
    let routes = match end? {
        RunEnd::Routed(r) => r,
        RunEnd::Stopped => return Err("run() stopped".into()),
    };
    (a.final_2d, a.final_3d, a.total_overflow) = (ov.g2d, ov.g3, ov.overflow);
    obs("incrend", a);
    // addRemainingGuides(routes, dirty_nets, …), connectPadPins, mergeSegments per route.
    let mut by_name: std::collections::BTreeMap<String, Vec<crate::GSegment>> = routes.into_iter().map(|(id, segs)| (nets[id as usize].name.clone(), segs)).collect();
    let grid_pins = |n: &RouterNet| -> Vec<crate::findrouting::GridPin> { n.net_pins.iter().map(|p| (p.on_grid.0, p.on_grid.1, p.connection_layer)).collect() };
    let remaining: Vec<crate::findrouting::RemainingNet> = dirty_nets.iter().map(|&id| crate::findrouting::RemainingNet { name: nets[id].name.clone(), made: true, pins: grid_pins(&nets[id]) }).collect();
    let (min, max) = (a.save_options.min_routing_layer, a.max_routing_layer);
    crate::findrouting::add_remaining_guides(&mut by_name, &remaining, min, max, db.block_get_max_routing_layer()).map_err(|e| format!("{e:?}"))?;
    crate::findrouting::connect_pad_pins(&mut by_name);
    let block_min = db.block_get_min_routing_layer();
    for (name, route) in by_name.iter_mut() {
        if let Some(n) = nets.iter().find(|n| &n.name == name) {
            crate::findrouting::merge_segments(&grid_pins(n), route, block_min);
        }
    }
    Ok(by_name)
}

/// `mergeResults`: each net's new route replaces its old one; the re-added nets' pins are the ones
/// `updateNetPins` read.
fn merge_results(a: &mut AfterRoute, routes: std::collections::BTreeMap<String, Vec<crate::GSegment>>) {
    for r in a.net_routes.iter_mut() {
        let Some(segs) = routes.get(&r.name) else { continue };
        r.segments = segs.clone();
        if let Some(n) = a.router_nets.iter().find(|n| n.name == r.name) {
            r.pins = n.net_pins.iter().map(|p| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect();
        }
    }
}
