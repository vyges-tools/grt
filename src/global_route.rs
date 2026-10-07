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
use crate::pins::{find_nets, find_pin, is_pin_reachable, make_bterm_pin, make_iterm_pin, AccessPoint, MasterClass, NetCandidate, NetPin, PinGrid, TermBox};
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
    /// The timer's libraries and constraints: with a clock and no captured slacks, the slacks are
    /// computed at each read.
    pub timing: Option<crate::timer::Timing>,
    /// Where to write every computed read (`<P|U> <call> <net> <f32 bits>`), if anywhere.
    pub timer_trace: Option<String>,
    /// `estimate_parasitics -placement` ran: a timer read with no route estimate of its own sees
    /// those parasitics.
    pub placement_parasitics: bool,
    /// Their networks by net (corner 0), when the estimator made any.
    pub placement_networks: Option<std::collections::BTreeMap<String, vyges_est::network::Parasitic>>,
    /// `global_route -resistance_aware`.
    pub resistance_aware: bool,
    /// `-res_aware_nets_percentage` — once given, FIXED (`is_fixed_nets_percentage_`).
    pub res_aware_nets_percentage: Option<f32>,
    /// The reference's CUGR timer slacks at each `updateNetSlacks`, per `-use_cugr` call — an
    /// ORACLE, like `cugr_slacks`.
    pub cugr_raw_slacks: Option<Vec<Vec<std::collections::BTreeMap<String, f32>>>>,
    /// `set_layer_rc -layer` — the ESTIMATOR's table: routing level → (ohm/m, F/m). The parasitics
    /// read it in preference to the technology's own values.
    pub layer_rc: std::collections::BTreeMap<i32, (f64, f64)>,
    /// `set_layer_rc -via` — the estimator's table for a cut layer, by its name (ohms per cut).
    pub via_rc: std::collections::BTreeMap<String, f64>,
    pub critical_nets_percentage: f32,
    /// CUGR's slacks as each net-order sort reads them, captured from the reference: per
    /// `-use_cugr` call, per sort (stage 1's first, then stage 3's, 4's and each RRR round's),
    /// net → slack. An ORACLE: CUGR orders by them where a clock makes them the timer's.
    pub cugr_slacks: Option<Vec<Vec<std::collections::BTreeMap<String, f32>>>>,
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
            timing: None,
            timer_trace: None,
            placement_parasitics: false,
            placement_networks: None,
            resistance_aware: false,
            res_aware_nets_percentage: None,
            cugr_raw_slacks: None,
            layer_rc: std::collections::BTreeMap::new(),
            via_rc: std::collections::BTreeMap::new(),
            critical_nets_percentage: 10.0,
            cugr_slacks: None,
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
    let tracks = init_routing_tracks(&tech.tracks, max, &pitches, tech.dbu_per_micron, opts.verbose, &mut log).map_err(|e| e.to_string())?;
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
/// `findPinAccessPointPositions` for an instance terminal: a core pin's PREFERRED access points
/// (`getPrefAccessPoints`), offset by the instance's location (orientation R0 — the router stores
/// them oriented), as `(routing level, x, y)`.
///
/// ⛔ A non-core pin reads EVERY access point, from a map keyed by master-pin pointer — an order
/// not reproduced here: refused when it has any.
fn iterm_access_points(db: &Db, inst: &str, term: &str, is_core: bool) -> Res<Vec<AccessPoint>> {
    if !is_core {
        if db.iterm_access_point_count(inst, term)? > 0 {
            return Err(format!("{inst}/{term}: a non-core terminal's access points (every master pin's, pointer-ordered) are not modelled").into());
        }
        return Ok(Vec::new());
    }
    let (ix, iy) = db.inst_location(inst);
    Ok(db.iterm_pref_access_points(inst, term)?.into_iter().map(|(x, y, l)| (l, x + ix, y + iy)).collect())
}

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
    let layer_name = |level: i32| t.tech.routing_layers.iter().find(|l| l.routing_level == level).map(|l| l.name.clone()).unwrap_or_default();
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
        // `findPinAccessPointPositions`: each pin's detailed-router access points, as `(layer, x, y)`.
        let mut aps: Vec<Vec<AccessPoint>> = Vec::new();
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
                .map_err(|e| e.text(&layer_name))?;
            let io = db.mterm_get_io_type(&master, term);
            aps.push(iterm_access_points(db, inst, term, pin.is_core)?);
            pins.push((pin, io == "OUTPUT" || io == "INOUT"));
        }
        for bterm in &n.bterms {
            let (placed, bx) = read_bterm(db, bterm)?;
            let boxes: Vec<TermBox> = bx.into_iter().map(|(level, routing, rect)| TermBox { pin: 0, level, routing, rect }).collect();
            // `check_pin_placement_` is true in every session this engine can build: only the Rudy
            // congestion path (`initFastRoute(.., false)`) clears it, and there is no Rudy step. So a
            // port with no routing-layer geometry is GRT-42, not skipped.
            if let Some(pin) = make_bterm_pin(bterm, placed, &boxes, die, &directions, true, opts.verbose, log).map_err(|e| e.text(&layer_name))? {
                let per_pin = (0..db.num_bterm_get_b_pins(bterm)).map(|b| Ok(db.bpin_access_points(bterm, b)?.into_iter().map(|(x, y, l)| (l, x, y)).collect())).collect::<Res<Vec<Vec<AccessPoint>>>>()?;
                aps.push(crate::pins::port_access_points(per_pin));
                pins.push((pin, db.bterm_get_io_type(bterm) == "INPUT"));
            }
        }
        for ((pin, _), aps) in pins.iter_mut().zip(&aps) {
            // With no capacities (CUGR) `find_pin` never asks: `use_cugr` skips the reachability test.
            find_pin(&pin_grid, pin, aps, &mut |p, pos| edge_capacity.is_some_and(|cap| is_pin_reachable(&pin_grid, p, pos, cap)));
        }
        nets.push((n, pins));
    }
    // initNets → checkPinPlacement, after every net's pins: ports sharing a position on a layer
    // warn GRT-31 and fail the run (GRT-80).
    let ports: Vec<&NetPin> = nets.iter().flat_map(|(_, pins)| pins.iter().map(|(p, _)| p)).filter(|p| p.is_port).collect();
    crate::pins::check_pin_placement(&ports, &layer_name, t.tech.dbu_per_micron, log).map_err(|e| e.to_string())?;
    Ok(nets)
}

/// I13 `initNets` (`findNets`: discovery, pins, the order) and I14 `initNetlist`.
///
/// ⛔ Refused as in I10: a pad or macro terminal, a net with a wire. A block terminal with no
/// routing geometry is GRT-42 (`check_pin_placement_` is true: only the Rudy path clears it, and
/// this engine has no Rudy step), and `checkPinPlacement` runs after the pins (GRT-31 / GRT-80).
pub fn setup_nets(db: &Db, t: &TechSetup, e: &mut RouterEdges, has_macros_or_pads: bool, opts: &RouteOptions, log: &mut Vec<String>) -> Res<Vec<RouterNet>> {
    Ok(setup_nets_counted(db, t, e, has_macros_or_pads, opts, log)?.0)
}

/// [`setup_nets`], and how many routable nets it left out because they already have wiring.
pub fn setup_nets_counted(db: &Db, t: &TechSetup, e: &mut RouterEdges, has_macros_or_pads: bool, opts: &RouteOptions, log: &mut Vec<String>) -> Res<(Vec<RouterNet>, usize)> {
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
    let mut already_wired = 0usize;
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
            already_wired += usize::from(crate::already_wired(pins.len(), n.has_wire));
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
    Ok((out, already_wired))
}

/// `preProcessTechLayers`: each routing layer (by level, up to the router's layers) and the cut layer
/// above it — their widths and resistances as the database holds them now (after any `set_layer_rc`).
fn pricing_tech(db: &Db, t: &TechSetup, num_layers: usize) -> Res<crate::pricing::TechLayers> {
    let mut tech = crate::pricing::TechLayers { dbu_per_micron: db.tech_get_db_units_per_micron(), width: Vec::new(), resistance: Vec::new(), via_resistance: Vec::new() };
    for level in 1..=num_layers as i32 {
        let l = t.tech.routing_layers.iter().find(|r| r.routing_level == level).ok_or_else(|| format!("no routing layer at level {level}"))?;
        tech.width.push(db.layer_get_width(&l.name) as i32);
        tech.resistance.push(db.layer_get_resistance(&l.name));
        let cut = db.layer_get_upper_layer(&l.name);
        tech.via_resistance.push((!cut.is_empty()).then(|| db.layer_get_resistance(&cut)));
    }
    Ok(tech)
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
    /// Routable nets left out because they already have wiring (the reference's initNetlist skip).
    pub already_wired: usize,
    /// The router's state as a later command in the same session reads it.
    pub after: AfterRoute,
    /// `FastRouteCore::updateDbCongestion`'s gcell grid, as it writes it: per axis (origin, count,
    /// step) = (`x_corner_`, `x_grid_`, `tile_size_`) — the core's low corner, grid count and tile.
    pub gcell_grid: ((i32, i32, i32), (i32, i32, i32)),
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
    /// `MakeWireParasitics`' layer table and the router's min routing layer: what
    /// `estimateGlobalRouteRC(db_net)` reads on a route as it stands ([`routed_network`]).
    pub parasitic_rc: crate::parasitics::LayerRC,
    pub min_routing_layer: i32,
    /// FastRoute ids whose net was destroyed (`removeNet` → `deleteNet` nulls the slot; an id is
    /// never given again — a net re-created under the same name takes a new one).
    pub dead: std::collections::BTreeSet<usize>,
    /// Nets GlobalRouter holds (`db_net_map_`) that FastRoute has no id for — made by `addNet`
    /// during a repair, or left with fewer than two pins — with their pins as the last
    /// `updateNetPins` left them (`addNet`'s runs before any terminal is connected: none).
    pub held_unrouted: std::collections::BTreeMap<String, Vec<(i32, i32, i32)>>,
    /// Each net's guides as the database holds them (`dbNet::getGuides`' list order). `saveGuides`
    /// creates each at the head and then REVERSES the list, so it is the creation order.
    pub db_guides: std::collections::BTreeMap<String, Vec<crate::Guide>>,
    /// `Net::restoreRouteFromGuides`: set by `inDbNetPostGuideRestore`, read (and cleared) by the
    /// next `updateDirtyNets`.
    pub restore_from_guides: std::collections::BTreeSet<String>,
    /// Per open eco level (`beginEco`), the guide edits it journaled, in order.
    pub eco: Vec<Vec<GuideEdit>>,
    /// `Net::isMergedNet` / `getMergedNet`: set on both nets by a routing merge (buffer removal),
    /// cleared when `updateDirtyNets` next passes the net.
    pub merged: std::collections::BTreeMap<String, String>,
    /// `GlobalRouter::resistance_aware_` (`setResistanceAware`): once set, every later incremental
    /// run is resistance-aware.
    pub resistance_aware: bool,
    /// `Net::isResAware` (GlobalRouter's flag, `setNetIsResAware`): such a net is routed again
    /// whatever its pins. ⚠️ Not FastRoute's own flag (`NetState::res_aware`), which `updateSlacks`
    /// sets and an `addNet` reset keeps.
    pub res_aware_nets: std::collections::BTreeSet<String>,
    /// `preProcessTechLayers`' table, for resistance-aware pricing and a net's resistance.
    pub res_tech: crate::pricing::TechLayers,
    /// `tile_size_`.
    pub tile_size: i32,
}

/// `GlobalRouter::setResistanceAware(true)`.
pub fn set_resistance_aware(a: &mut AfterRoute) {
    a.resistance_aware = true;
}

/// `GlobalRouter::setNetIsResAware(db_net, true)` (GRT-0103, a warning, for a net the router does
/// not hold — nothing set).
pub fn set_net_res_aware(a: &mut AfterRoute, net: &str) {
    if holds(a, net) {
        a.res_aware_nets.insert(net.to_string());
    }
}

/// `GlobalRouter::isNetResAware(db_net)`.
pub fn is_net_res_aware(a: &AfterRoute, net: &str) -> bool {
    a.res_aware_nets.contains(net)
}

/// `FastRouteCore::getNetResistanceOnLayer(db_net, layer)` on the net's routed (3D) tree: each
/// wire on `layer` when given, else on its own; each layer change a via stack. 0 for a net
/// FastRoute has no tree for (one restored from guides).
pub fn net_resistance_on_layer(a: &AfterRoute, net: &str, layer: Option<i32>) -> f32 {
    let Some(id) = live_id(a, net) else { return 0.0 };
    let Some(t) = a.final_state[id].tree3d.as_ref() else { return 0.0 };
    let n = &a.router_nets[id];
    let (lo, hi) = a.final_state[id].layer_range.map_or(((n.min_layer - 1) as usize, (n.max_layer - 1) as usize), |r| r);
    let wn = crate::pricing::WireNet { ndr_width: None, min_layer: lo as i32, max_layer: hi as i32 };
    let mut total = 0.0f32;
    for e in &t.edges {
        if e.len == 0 && e.routelen == 0 {
            continue;
        }
        for i in 0..e.routelen.max(0) as usize {
            let (p, q) = (e.grids[i], e.grids[i + 1]);
            if p.layer == q.layer {
                let seg = i32::from((p.x - q.x).abs() + (p.y - q.y).abs());
                let wire_layer = layer.unwrap_or(i32::from(p.layer));
                total += crate::pricing::get_wire_resistance(&a.res_tech, wire_layer, seg * a.tile_size, wn);
            } else {
                total += crate::pricing::get_via_resistance(&a.res_tech, i32::from(p.layer), i32::from(q.layer));
            }
        }
    }
    total
}

/// `GlobalRouter::getFRNetResistanceOnMinResistanceLayer`: the net's resistance with every wire on
/// the routing layer (min..max routing layer) of least resistance per width — the first of equals.
pub fn net_resistance_on_min_resistance_layer(a: &AfterRoute, net: &str) -> f32 {
    let mut min_res_layer = a.min_routing_layer;
    let mut min_res_per_width = f32::MAX;
    for level in a.min_routing_layer..=a.max_routing_layer {
        let k = (level - 1) as usize;
        let (Some(&w), Some(&r)) = (a.res_tech.width.get(k), a.res_tech.resistance.get(k)) else { continue };
        let (w, r) = (w as f32, r as f32);
        if w > 0.0 && r > 0.0 && r / w < min_res_per_width {
            min_res_per_width = r / w;
            min_res_layer = level;
        }
    }
    net_resistance_on_layer(a, net, Some(min_res_layer - 1))
}

/// One journaled guide edit.
#[derive(Debug, Clone, PartialEq)]
pub enum GuideEdit {
    /// `dbGuide::destroy` of each guide, in list order (`clearGuides`, a net's destruction).
    Deleted(String, Vec<crate::Guide>),
    /// `dbGuide::create` of each (`saveGuides` after an incremental re-route).
    Created(String, Vec<crate::Guide>),
}

/// `dbDatabase::beginEco`, the router's side: a level that records each guide deleted under it.
pub fn begin_eco(a: &mut AfterRoute) {
    a.eco.push(Vec::new());
}

/// `commitEco`: the level's deletions stay; an enclosing level takes them over.
pub fn commit_eco(a: &mut AfterRoute) {
    if let Some(level) = a.eco.pop() {
        if let Some(parent) = a.eco.last_mut() {
            parent.extend(level);
        }
    }
}

/// `undoEco`, the router's side: each guide the level deleted is created again — the journal
/// replayed backwards, each at the head of its net's list, so the list comes back in its old order
/// — and each fires `inDbNetPostGuideRestore`: a held net is to be restored from its guides
/// (`setRestoreRouteFromGuides`). Returns the net of every guide created, in that order — the
/// caller marks each dirty (`addDirtyNet`). The database's own undo runs FIRST (a destroyed net is
/// created again before its guides).
///
/// Undoing a guide's CREATION destroys it and fires nothing.
pub fn undo_eco(a: &mut AfterRoute) -> Vec<String> {
    let Some(level) = a.eco.pop() else { return Vec::new() };
    let mut fired = Vec::new();
    for edit in level.into_iter().rev() {
        match edit {
            GuideEdit::Created(net, guides) => {
                if let Some(list) = a.db_guides.get_mut(&net) {
                    for g in &guides {
                        if let Some(k) = list.iter().position(|x| x == g) {
                            list.remove(k);
                        }
                    }
                    if list.is_empty() {
                        a.db_guides.remove(&net);
                    }
                }
            }
            GuideEdit::Deleted(net, guides) => {
                for g in guides.iter().rev() {
                    a.db_guides.entry(net.clone()).or_default().insert(0, *g);
                    if holds(a, &net) {
                        a.restore_from_guides.insert(net.clone());
                        // `fr_net->setIsResAware(false)` (GlobalRouter's net).
                        a.res_aware_nets.remove(&net);
                    }
                    fired.push(net.clone());
                }
            }
        }
    }
    fired
}

/// `dbNet::clearGuides` (or a net's destruction): every guide deleted, recorded under the open eco
/// level.
fn clear_guides(a: &mut AfterRoute, net: &str) {
    let gone = a.db_guides.remove(net).unwrap_or_default();
    if gone.is_empty() {
        return;
    }
    if let Some(level) = a.eco.last_mut() {
        level.push(GuideEdit::Deleted(net.to_string(), gone));
    }
}

/// `saveGuides(modified_nets)` for one net after an incremental re-route: its old guides cleared
/// (none are left: `updateDirtyNets` cleared them) and new ones made from the route, journaled.
fn save_net_guides(a: &mut AfterRoute, net: &str) -> Res<()> {
    let Some(r) = a.net_routes.iter().find(|r| r.name == net).cloned() else { return Ok(()) };
    if r.segments.is_empty() {
        return Ok(());
    }
    clear_guides(a, net);
    let saved = crate::save_guides(std::slice::from_ref(&r), &a.jumper_grid.grid, &a.save_options).map_err(|e| format!("{e:?}"))?;
    let guides: Vec<crate::Guide> = saved.into_iter().flat_map(|ng| ng.guides).collect();
    a.db_guides.insert(net.to_string(), guides.clone());
    if let Some(level) = a.eco.last_mut() {
        level.push(GuideEdit::Created(net.to_string(), guides));
    }
    Ok(())
}

/// A net's FastRoute id, the live one (a destroyed net's id is dead).
pub fn live_id(a: &AfterRoute, net: &str) -> Option<usize> {
    a.router_nets.iter().enumerate().find(|(k, n)| n.name == net && !a.dead.contains(k)).map(|(k, _)| k)
}

/// Whether GlobalRouter holds the net (`db_net_map_`): `addDirtyNet` marks only such a net.
pub fn holds(a: &AfterRoute, net: &str) -> bool {
    live_id(a, net).is_some() || a.held_unrouted.contains_key(net)
}

/// `GRouteDbCbk::inDbNetCreate` → `GlobalRouter::addNet`: a routable net (not supply, not special,
/// no special wires, not connected by abutment) is held from now on, with no pins yet. Returns
/// whether it was (`made`).
pub fn add_net(db: &Db, a: &mut AfterRoute, net: &str) -> bool {
    let sig = db.net_sigtype(net);
    let made = crate::init::is_routable(sig == "POWER" || sig == "GROUND", db.net_is_special(net), db.num_net_get_s_wires(net) > 0, db.net_is_connected_by_abutment(net));
    if made {
        a.held_unrouted.insert(net.to_string(), Vec::new());
    }
    made
}

/// `GRouteDbCbk::inDbNetDestroy` → `GlobalRouter::removeNet`, the router's side (the caller drops
/// the net from its dirty set): with a FastRoute id, the route's usage is released now —
/// `clearNetRoute`, or `updateNetResources(net, true)` for a route restored from guides — and the
/// slot is nulled (`deleteNet`); `routes_` loses the net.
///
/// ⛔ A merged net (`isMergedNet`) takes other branches: refused.
pub fn remove_net(a: &mut AfterRoute, net: &str) -> Res<()> {
    // `dbNet::destroy` deletes the net's guides (journaled).
    clear_guides(a, net);
    a.restore_from_guides.remove(net);
    a.res_aware_nets.remove(net);
    a.held_unrouted.remove(net);
    let Some(id) = live_id(a, net) else { return Ok(()) };
    // A net merged into its survivor (`isMergedNet`): FastRoute's `mergeNet` moves its tree into
    // the survivor's (nodes and edges appended; its usage stays, now the survivor's) and drops it.
    if let Some(preserved) = a.merged.get(net).cloned() {
        let pid = live_id(a, &preserved).ok_or_else(|| format!("net {net}: merged into {preserved}, which the router no longer holds"))?;
        if a.final_state[id].segments_restored || a.final_state[pid].segments_restored {
            return Err(format!("net {net}: a merged net with segments restored from guides — not modelled").into());
        }
        let moved = a.final_state[id].tree3d.take();
        match (moved, a.final_state[pid].tree3d.as_mut()) {
            (Some(t2), Some(t1)) => {
                t1.num_terminals += t2.num_terminals;
                t1.nodes.extend(t2.nodes);
                t1.edges.extend(t2.edges);
            }
            (None, _) => {}
            (Some(_), None) => return Err(format!("net {preserved}: no 3D tree to merge {net} into — GRT-0013").into()),
        }
        a.final_state[id].tree = None;
        a.merged.remove(net);
        a.dead.insert(id);
        a.net_routes.retain(|r| r.name != net);
        return Ok(());
    }
    if a.final_state[id].segments_restored {
        let segs = a.net_routes.iter().find(|r| r.name == net).map(|r| r.segments.clone()).unwrap_or_default();
        let lec = a.layer_edge_cost.get(net).cloned().unwrap_or_default();
        if let (Some(g2), Some(g3)) = (a.final_2d.as_mut(), a.final_3d.as_mut()) {
            update_net_resources(g2, g3, &a.jumper_grid, &a.router_nets[id], id, &lec, &segs, -1);
        }
        a.final_state[id].segments_restored = false;
    } else {
        clear_net_route(a, id);
    }
    a.dead.insert(id);
    a.net_routes.retain(|r| r.name != net);
    Ok(())
}

/// `GRouteDbCbk::inDbNetPostMerge(preserved, removed)` → `GlobalRouter::mergeNetsRouting`: a buffer
/// removed, its two nets' routes joined (`connectRouting`) and the survivor's guides saved again;
/// both nets marked merged. `Ok(false)` when they could not be joined — the caller marks the
/// survivor dirty (`addDirtyNet`), to be routed again.
pub fn merge_nets_routing(db: &mut Db, opts: &RouteOptions, a: &mut AfterRoute, net1: &str, net2: &str) -> Res<bool> {
    if !connect_routing(db, opts, a, net1, net2)? {
        return Ok(false);
    }
    save_net_guides(a, net1)?;
    a.merged.insert(net1.to_string(), net2.to_string());
    a.merged.insert(net2.to_string(), net1.to_string());
    Ok(true)
}

/// `GlobalRouter::connectRouting(net1, net2)`, the reference's stages in order:
/// `findBufferPinPostions` (from the router's stale pins: the buffer is still on them); either
/// routes empty → not joined; pins in different gcells → `findTopLayerOverPosition` on each route,
/// `createConnectionForPositions`, `hasAvailableResources` per wire of it (none → not joined),
/// `addTreeEdge` per wire, net1's route += net2's + the connection; the same gcell → the layer gap
/// bridged with vias, net1's route += net2's; then `updateNetPins(net1)`, `netIsCovered` (not →
/// not joined: the extended route stays) and `isConnected` (not → GRT-0298).
///
/// ⛔ A net with segments restored from guides takes `updateResources` instead of `addTreeEdge`:
/// refused.
fn connect_routing(db: &mut Db, opts: &RouteOptions, a: &mut AfterRoute, net1: &str, net2: &str) -> Res<bool> {
    let (Some(id1), Some(id2)) = (live_id(a, net1), live_id(a, net2)) else {
        return Err(format!("merging {net2} into {net1}: a net with no FastRoute net — not modelled").into());
    };
    // findBufferPinPostions: the last pin pair on one instance (the inner loop breaks, the outer
    // does not).
    let inst_of = |p: &crate::pins::NetPin| p.name.rsplit_once('/').map(|(i, _)| i.to_string());
    let (mut pos1, mut pos2) = ((0, 0), (0, 0));
    for p1 in a.router_nets[id1].net_pins.iter().filter(|p| !p.is_port) {
        for p2 in a.router_nets[id2].net_pins.iter().filter(|p| !p.is_port) {
            if inst_of(p1) == inst_of(p2) {
                (pos1, pos2) = (p1.on_grid, p2.on_grid);
                break;
            }
        }
    }
    let route_of = |a: &AfterRoute, n: &str| a.net_routes.iter().find(|r| r.name == n).map(|r| r.segments.clone()).unwrap_or_default();
    let (mut route1, route2) = (route_of(a, net1), route_of(a, net2));
    if route1.is_empty() || route2.is_empty() {
        return Ok(false);
    }
    if pos1 != pos2 {
        let layer1 = top_layer_over_position(pos1, &route1)?;
        let layer2 = top_layer_over_position(pos2, &route2)?;
        let connection = create_connection_for_positions(a, pos1, pos2, layer1, layer2);
        let g = &a.jumper_grid;
        let tiles = |s: &crate::GSegment| (g.dbu_to_tile(s.init_x.min(s.final_x), true), g.dbu_to_tile(s.init_y.min(s.final_y), false), g.dbu_to_tile(s.init_x.max(s.final_x), true), g.dbu_to_tile(s.init_y.max(s.final_y), false));
        for seg in connection.iter().filter(|s| !s.is_via()) {
            let (x1, y1, x2, y2) = tiles(seg);
            if !has_available_resources(a, x1, y1, x2, y2, seg.init_layer, id1) {
                return Ok(false);
            }
        }
        if a.final_state[id1].segments_restored || a.final_state[id2].segments_restored {
            return Err(format!("merging {net2} into {net1}: segments restored from guides (updateResources) — not modelled").into());
        }
        let (xmin, ymin, tile) = (a.jumper_grid.grid.area.x_min, a.jumper_grid.grid.area.y_min, a.jumper_grid.grid.tile_size);
        for seg in connection.iter().filter(|s| !s.is_via()) {
            let x1 = (seg.init_x.min(seg.final_x) - xmin) / tile;
            let y1 = (seg.init_y.min(seg.final_y) - ymin) / tile;
            let x2 = (seg.init_x.max(seg.final_x) - xmin) / tile;
            let y2 = (seg.init_y.max(seg.final_y) - ymin) / tile;
            add_tree_edge(a, x1, y1, x2, y2, seg.init_layer, id1)?;
        }
        route1.extend(route2);
        route1.extend(connection);
    } else {
        let (min1, max1) = layer_range_over_position(pos1, &route1);
        let (min2, max2) = layer_range_over_position(pos1, &route2);
        if max1 != -1 && max2 != -1 {
            if max1 < min2 {
                insert_vias_for_connection(&mut route1, pos1, max1, min2);
            } else if max2 < min1 {
                insert_vias_for_connection(&mut route1, pos1, max2, min1);
            }
        }
        route1.extend(route2);
    }
    if let Some(r) = a.net_routes.iter_mut().find(|r| r.name == net1) {
        r.segments = route1.clone();
    }
    // updateNetPins(net1)
    let fresh = fresh_net_pins(db, opts)?;
    match fresh.get(net1) {
        Some(n) => {
            a.router_nets[id1].net_pins = n.net_pins.clone();
            a.router_nets[id1].pin_is_driver = n.pin_is_driver.clone();
        }
        None => {
            a.router_nets[id1].net_pins.clear();
            a.router_nets[id1].pin_is_driver.clear();
        }
    }
    let pins: Vec<crate::Pin> = a.router_nets[id1].net_pins.iter().map(|p| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect();
    if !crate::restore::net_is_covered(&route1, &pins).is_empty() {
        return Ok(false);
    }
    if !is_connected(&route1)? {
        return Err(format!("[ERROR GRT-0298] Net {net1} has disconnected segments after merge.").into());
    }
    Ok(true)
}

/// `findTopLayerOverPosition`: the highest layer of a segment with an END at the point. ⛔ None is
/// GRT-0703.
fn top_layer_over_position(pos: (i32, i32), route: &[crate::GSegment]) -> Res<i32> {
    let top = route
        .iter()
        .filter(|s| (s.init_x, s.init_y) == pos || (s.final_x, s.final_y) == pos)
        .map(|s| s.init_layer.max(s.final_layer))
        .max()
        .unwrap_or(-1);
    if top == -1 {
        return Err("[ERROR GRT-0703] No segment was found in the routing that connects to the pin position.".into());
    }
    Ok(top)
}

/// `findLayerRangeOverPosition`: the layers of the segments whose box holds the point;
/// `(i32::MAX, -1)` for none.
fn layer_range_over_position(pos: (i32, i32), route: &[crate::GSegment]) -> (i32, i32) {
    let (mut lo, mut hi) = (i32::MAX, -1);
    for s in route {
        let (x0, x1) = (s.init_x.min(s.final_x), s.init_x.max(s.final_x));
        let (y0, y1) = (s.init_y.min(s.final_y), s.init_y.max(s.final_y));
        if (x0..=x1).contains(&pos.0) && (y0..=y1).contains(&pos.1) {
            lo = lo.min(s.init_layer).min(s.final_layer);
            hi = hi.max(s.init_layer).max(s.final_layer);
        }
    }
    (lo, hi)
}

/// `insertViasForConnection`: a via per layer step between the two layers, at the point.
fn insert_vias_for_connection(route: &mut Vec<crate::GSegment>, pos: (i32, i32), layer: i32, conn_layer: i32) {
    let (lo, hi) = (layer.min(conn_layer), layer.max(conn_layer));
    for l in lo..hi {
        route.push(crate::GSegment::new(pos.0, pos.1, l, pos.0, pos.1, l + 1));
    }
}

/// `createConnectionForPositions`: in one line, a wire on the higher layer — moved one layer off
/// (down above the min routing layer, else up) when that layer runs the other way; else an L:
/// horizontal on the horizontal one of the pair, a via, vertical on the vertical one, with via
/// stacks from each route up to them; then vias at each pin from its route's layer to the
/// connection layer.
fn create_connection_for_positions(a: &AfterRoute, p1: (i32, i32), p2: (i32, i32), layer1: i32, layer2: i32) -> Vec<crate::GSegment> {
    use crate::layertable::LayerDir;
    let mut c = Vec::new();
    let mut conn_layer = layer1.max(layer2);
    let dir = a.layer_dir.get((conn_layer - 1) as usize).copied().unwrap_or(LayerDir::Other);
    let (vertical, horizontal) = (p1.0 == p2.0, p1.1 == p2.1);
    let min_layer = a.min_routing_layer;
    if vertical || horizontal {
        let (x1, x2) = (p1.0.min(p2.0), p1.0.max(p2.0));
        let (y1, y2) = (p1.1.min(p2.1), p1.1.max(p2.1));
        if (vertical && dir != LayerDir::Vertical) || (horizontal && dir != LayerDir::Horizontal) {
            if conn_layer > min_layer {
                conn_layer -= 1;
            } else {
                conn_layer += 1;
            }
        }
        c.push(crate::GSegment::new(x1, y1, conn_layer, x2, y2, conn_layer));
    } else {
        let fix = if conn_layer <= min_layer { 1 } else { -1 };
        let hor = if dir == LayerDir::Horizontal { conn_layer } else { conn_layer + fix };
        let ver = if dir == LayerDir::Vertical { conn_layer } else { conn_layer + fix };
        let (x1, y1, x2, y2) = (p1.0, p1.1, p2.0, p2.1);
        c.push(crate::GSegment::new(x1, y1, hor, x2, y1, hor));
        c.push(crate::GSegment::new(x2, y1, conn_layer + fix, x2, y1, conn_layer));
        c.push(crate::GSegment::new(x2, y1, ver, x2, y2, ver));
        for l in layer1..hor {
            c.push(crate::GSegment::new(x1, y1, l, x1, y1, l + 1));
        }
        for l in layer2..ver {
            c.push(crate::GSegment::new(x2, y2, l, x2, y2, l + 1));
        }
    }
    insert_vias_for_connection(&mut c, p1, layer1, conn_layer);
    insert_vias_for_connection(&mut c, p2, layer2, conn_layer);
    c
}

/// `FastRouteCore::hasAvailableResources`: on each unit of the wire, the 3D edge's free capacity at
/// least the net's layer edge cost and the 2D edge's at least its edge cost.
fn has_available_resources(a: &AfterRoute, x1: i32, y1: i32, x2: i32, y2: i32, layer: i32, id: usize) -> bool {
    let (Some(g2), Some(g3)) = (a.final_2d.as_ref(), a.final_3d.as_ref()) else { return false };
    let k = (layer - 1) as usize;
    let n = &a.router_nets[id];
    let lec = i32::from(a.layer_edge_cost.get(&n.name).and_then(|v| v.get(k).copied()).unwrap_or(1));
    let ec = i32::from(n.edge_cost);
    let xg = g3.x_grid;
    if y1 == y2 {
        (x1..x2).all(|x| {
            let at = y1 as usize * xg + x as usize;
            i32::from(g3.h_cap[k][at]) - i32::from(g3.h_usage[k][at]) >= lec && i32::from(g2.cap_h[at]) - i32::from(g2.est.usage_h(x as usize, y1 as usize)) >= ec
        })
    } else if x1 == x2 {
        (y1..y2).all(|y| {
            let at = y as usize * xg + x1 as usize;
            i32::from(g3.v_cap[k][at]) - i32::from(g3.v_usage[k][at]) >= lec && i32::from(g2.cap_v[at]) - i32::from(g2.est.usage_v(x1 as usize, y as usize)) >= ec
        })
    } else {
        true
    }
}

/// `FastRouteCore::addTreeEdge`: a straight edge charged to the net (2D: its edge cost, NDR-aware;
/// 3D: its layer edge cost) and appended to its tree. ⛔ Not straight: GRT-0216.
fn add_tree_edge(a: &mut AfterRoute, x1: i32, y1: i32, x2: i32, y2: i32, layer: i32, id: usize) -> Res<()> {
    use crate::full3d::Point3D;
    let k = layer - 1;
    let r = a.router_nets[id].clone();
    let lec = a.layer_edge_cost.get(&r.name).and_then(|v| v.get(k as usize).copied()).unwrap_or(1);
    let (min, max) = ((r.min_layer - 1) as usize, (r.max_layer - 1) as usize);
    let net = crate::ndr_cost::NdrCostNet { id, edge_cost: r.edge_cost, min_layer: min, max_layer: max, layer_edge_cost: Some(net_range_edge_costs(&r)), soft_ndr: false };
    let (Some(g2), Some(g3)) = (a.final_2d.as_mut(), a.final_3d.as_mut()) else { return Err("no graphs to charge a tree edge to".into()) };
    let xg = g3.x_grid;
    let mut grids = Vec::new();
    let p = |x: i32, y: i32| Point3D { x: x as i16, y: y as i16, layer: k as i16 };
    if x1 == x2 {
        for y in y1..y2 {
            crate::estimate::Usage2d::update_usage_v(&mut g2.for_net(&net), x1, y, f64::from(r.edge_cost));
            let c = &mut g3.v_usage[k as usize][y as usize * xg + x1 as usize];
            *c = c.wrapping_add(lec as u16);
            grids.push(p(x1, y));
        }
        grids.push(p(x2, y2));
    } else if y1 == y2 {
        for x in x1..x2 {
            crate::estimate::Usage2d::update_usage_h(&mut g2.for_net(&net), x, y1, f64::from(r.edge_cost));
            let c = &mut g3.h_usage[k as usize][y1 as usize * xg + x as usize];
            *c = c.wrapping_add(lec as u16);
            grids.push(p(x, y1));
        }
        grids.push(p(x2, y1));
    } else {
        return Err("[ERROR GRT-0216] Cannot add tree edge: edge is not vertical or horizontal.".into());
    }
    let edge = crate::maze3d::Edge3D { n1: 0, n2: 0, n1a: 0, n2a: 0, len: 0, route_type: crate::full3d::RouteType::NoRoute, routelen: grids.len() as i32 - 1, grids };
    a.final_state[id].tree3d.as_mut().ok_or("a tree edge added to a net with no 3D tree — not modelled")?.edges.push(edge);
    Ok(())
}

/// `GlobalRouter::isConnected`: union-find over the route's segments, each joined to an earlier
/// one its box touches (in 3D). ⛔ A segment that is not a line or a via is GRT-0264/0265.
fn is_connected(route: &[crate::GSegment]) -> Res<bool> {
    let n = route.len();
    if n == 0 {
        return Ok(true);
    }
    let is_line = |s: &crate::GSegment| i32::from(s.init_x != s.final_x) + i32::from(s.init_y != s.final_y) + i32::from(s.init_layer != s.final_layer) == 1;
    let connect = |a: &crate::GSegment, b: &crate::GSegment| {
        let r = |p: i32, q: i32| (p.min(q), p.max(q));
        let ((ax0, ax1), (ay0, ay1), (az0, az1)) = (r(a.init_x, a.final_x), r(a.init_y, a.final_y), r(a.init_layer, a.final_layer));
        let ((bx0, bx1), (by0, by1), (bz0, bz1)) = (r(b.init_x, b.final_x), r(b.init_y, b.final_y), r(b.init_layer, b.final_layer));
        ax1 >= bx0 && ax0 <= bx1 && ay1 >= by0 && ay0 <= by1 && az1 >= bz0 && az0 <= bz1
    };
    if !is_line(&route[0]) {
        return Err("[ERROR GRT-0264] a route segment is not a horizontal/vertical line or via".into());
    }
    let mut parent: Vec<usize> = (0..n).collect();
    let mut rank = vec![0u32; n];
    fn find(p: &mut [usize], x: usize) -> usize {
        if p[x] != x {
            let r = find(p, p[x]);
            p[x] = r;
        }
        p[x]
    }
    let mut groups = 1;
    for i in 1..n {
        if !is_line(&route[i]) {
            return Err("[ERROR GRT-0265] a route segment is not a horizontal/vertical line or via".into());
        }
        groups += 1;
        let mut j = i;
        while j > 0 && groups > 1 {
            j -= 1;
            if connect(&route[i], &route[j]) {
                let (ru, rv) = (find(&mut parent, i), find(&mut parent, j));
                if ru != rv {
                    if rank[ru] > rank[rv] {
                        parent[rv] = ru;
                    } else if rank[ru] < rank[rv] {
                        parent[ru] = rv;
                    } else {
                        parent[rv] = ru;
                        rank[ru] += 1;
                    }
                    groups -= 1;
                }
            }
        }
    }
    Ok(groups == 1)
}

/// `EstimateParasitics::estimateGlobalRouteRC(db_net)`: the net's RC network from its route as
/// `routes_` holds it now, its pins as the router's last `updateNetPins` left them. `None` with no
/// route or an empty one — the estimator then builds nothing, and the net keeps the parasitics it
/// had.
pub fn routed_network(a: &AfterRoute, net: &str) -> Option<crate::parasitics::Network> {
    let r = a.net_routes.iter().find(|r| r.name == net)?;
    if r.segments.is_empty() {
        return None;
    }
    let n = &a.router_nets[live_id(a, net)?];
    let route: Vec<crate::parasitics::Segment> = r
        .segments
        .iter()
        .map(|g| crate::parasitics::Segment { init_x: g.init_x, init_y: g.init_y, init_layer: g.init_layer, final_x: g.final_x, final_y: g.final_y, final_layer: g.final_layer })
        .collect();
    Some(net_network(n, &route, &a.parasitic_rc, a.min_routing_layer, crate::parasitics::PinAttach::Routed).1)
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
/// `MakeWireParasitics::layerRC`: the estimator's table where it has a value, the technology's
/// own only where it has none.
pub(crate) fn layer_rc_for(db: &Db, t: &TechSetup, opts: &RouteOptions) -> crate::parasitics::LayerRC {
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
}

/// `estimateAllGlobalRouteParasitics` for one net on its planar route: the route's segments, the
/// pins where they attach, and the RC network.
fn planar_net_network(
    n: &RouterNet,
    tree: &crate::brk_rsmt::StTree,
    rc: &crate::parasitics::LayerRC,
    layer_dir: &[crate::layertable::LayerDir],
    origin: crate::routes::GridOrigin,
    min_routing_layer: i32,
) -> (Vec<crate::parasitics::Segment>, Vec<crate::parasitics::PinGridLocation>, crate::parasitics::Network) {
    let edges: Vec<crate::planar_route::PlanarEdge<'_>> = tree
        .edges
        .iter()
        .zip(&tree.routes)
        .map(|(e, route)| crate::planar_route::PlanarEdge { len: e.len, routelen: route.routelen, grids: &route.grids })
        .collect();
    let route = crate::planar_route::planar_route(&edges, (n.min_layer - 1) as usize, layer_dir, origin);
    let (pins, network) = net_network(n, &route, rc, min_routing_layer, crate::parasitics::PinAttach::Planar);
    (route, pins, network)
}

/// `MakeWireParasitics::estimateParasitics` for one net on a route: the pins where it attaches and
/// the RC network (`attach`: planar, or by the pins' real layers on a route after layer assignment).
fn net_network(
    n: &RouterNet,
    route: &[crate::parasitics::Segment],
    rc: &crate::parasitics::LayerRC,
    min_routing_layer: i32,
    attach: crate::parasitics::PinAttach,
) -> (Vec<crate::parasitics::PinGridLocation>, crate::parasitics::Network) {
    let pins: Vec<crate::parasitics::PinGridLocation> = n
        .net_pins
        .iter()
        .zip(&n.pin_is_driver)
        .map(|(p, &is_driver)| crate::parasitics::PinGridLocation { name: p.name.clone(), is_port: p.is_port, is_driver, pt: p.position, grid_pt: p.on_grid, conn_layer: p.connection_layer })
        .collect();
    let np = crate::parasitics::NetParasitics {
        name: &n.name,
        route,
        pins: &pins,
        net_min_layer: n.min_layer,
        min_routing_layer,
        ndr_width: n.ndr_widths.as_ref(),
        attach,
    };
    let network = crate::parasitics::estimate_net(&np, rc);
    (pins, network)
}

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
    let (nets, already_wired) = setup_nets_counted(db, &t, &mut e, adj.has_macros_or_pads, opts, &mut log)?;
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
    // ⛔ The parasitics read the estimator's table where it has a value, and the technology's own
    // only where it has none (`MakeWireParasitics::layerRC`).
    let layer_rc = |db: &Db| -> crate::parasitics::LayerRC { layer_rc_for(db, &t, opts) };
    let layer_dir: Vec<crate::layertable::LayerDir> = (1..=num_layers as i32)
        .map(|l| match t.tech.routing_layers.iter().find(|r| r.routing_level == l).and_then(|r| r.direction) {
            Some(crate::capacity::Direction::Horizontal) => crate::layertable::LayerDir::Horizontal,
            Some(crate::capacity::Direction::Vertical) => crate::layertable::LayerDir::Vertical,
            None => crate::layertable::LayerDir::Other,
        })
        .collect();
    // With a clock and no capture, the timer itself answers each read, on the routes as they stand:
    // est::estimateAllGlobalRouteParasitics over the planar routes, then vyges-sta.
    let origin = crate::routes::GridOrigin { tile_size: t.core.tile_size, x_corner: t.core.area.x_min, y_corner: t.core.area.y_min };
    let timer_rc = layer_rc(db);
    let timer_netlist = opts.timing.as_ref().map(|_| crate::timer::netlist(db));
    // `timer_trace`: every computed read, as `<P|U> <call> <net> <f32 bits>` (a capture's format).
    let timer_trace = std::cell::RefCell::new(String::new());
    let compute = |read: crate::congestion_loop::SlackRead, k: usize, st: &[NetState]| -> Result<Vec<f32>, String> {
        let (Some(timing), Some(nl)) = (&opts.timing, &timer_netlist) else { return Err("no timer".into()) };
        let mut par = std::collections::BTreeMap::new();
        // getPlanarRoutes: the planar routes before layer assignment, the 3D routes (estimated
        // with the pins attached by their real layers) in a 3D pass.
        let is_3d = read == (crate::congestion_loop::SlackRead::Update { is_3d_step: true });
        for &id in &net_ids {
            let n = &nets[id];
            if is_3d {
                let tree = st[id].tree3d.as_ref().ok_or_else(|| format!("net {}: no 3D tree in a 3D pass", n.name))?;
                let (px, py): (Vec<i32>, Vec<i32>) = n.pins.iter().map(|p| (p.0, p.1)).unzip();
                let pl: Vec<i16> = n.pins.iter().map(|p| (p.2 - 1) as i16).collect();
                let route = crate::planar_route::route_3d(tree, crate::planar_route::NetPinsGrid { x: &px, y: &py, layer: &pl }, origin);
                par.insert(n.name.clone(), net_network(n, &route, &timer_rc, t.min_routing_layer, crate::parasitics::PinAttach::Routed).1);
            } else if let Some(tree) = st[id].tree.as_ref() {
                par.insert(n.name.clone(), planar_net_network(n, tree, &timer_rc, &layer_dir, origin, t.min_routing_layer).2);
            }
        }
        let by_name = crate::timer::net_slacks(timing, nl, &par)?;
        if opts.timer_trace.is_some() {
            let tag = if read == crate::congestion_loop::SlackRead::Partial { "P" } else { "U" };
            let mut tr = timer_trace.borrow_mut();
            for &id in &net_ids {
                if let Some(v) = by_name.get(&nets[id].name) {
                    tr.push_str(&format!("{tag} {k} {} {:08x}\n", nets[id].name, v.to_bits()));
                }
            }
        }
        (0..nets.len())
            .map(|k| match by_name.get(&nets[k].name) {
                Some(&s) => Ok(s),
                None if nets[k].is_local => Ok(0.0),
                None => Err(format!("net {}: not in the timing netlist", nets[k].name)),
            })
            .collect()
    };
    let computed = opts.timing.is_some();
    let update_slacks = match (&opts.liberty, opts.clock_sources.is_empty(), &opts.captured_update_slacks) {
        (Some(_), true, _) => crate::congestion_loop::TimerSlack::Every(&unconstrained),
        (Some(_), false, Some(_)) => crate::congestion_loop::TimerSlack::PerCall(&captured_update),
        (Some(_), false, None) if computed => crate::congestion_loop::TimerSlack::Compute(&compute),
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
    let res_tech = pricing_tech(db, &t, num_layers)?;
    let res_aware = if opts.resistance_aware {
        Some(crate::run::ResAwareInputs { tech: res_tech.clone(), tile_size: t.core.tile_size, fixed_percentage: opts.res_aware_nets_percentage, update_slacks })
    } else {
        None
    };
    let timer_slack = match (&opts.liberty, opts.clock_sources.is_empty(), &opts.captured_slacks) {
        (Some(_), true, _) => crate::congestion_loop::TimerSlack::Every(&unconstrained),
        (Some(_), false, Some(_)) => crate::congestion_loop::TimerSlack::PerCall(&captured),
        (Some(_), false, None) if computed => crate::congestion_loop::TimerSlack::Compute(&compute),
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
    // The run's final overflow (after R19), for the congestion verdict — and the 2D trees at the
    // router's first timer read, which is the state `estimateAllGlobalRouteParasitics` reads
    // (`getPartialRoutes` → `getPlanarRoutes`): the first partial-slack call, else — a
    // resistance-aware run with none — `layerAssignment`'s `updateSlacks`, whose trees are those
    // after R15's `removeLoops` (B15).
    struct Observer {
        overflow: i32,
        cnp: f32,
        res_aware: bool,
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
                Stage::Scan { tag: "B15", .. } if self.res_aware && self.trees.is_none() => {
                    self.trees = Some(st.iter().map(|n| n.tree.clone()).collect());
                }
                _ => {}
            }
            true
        }
    }
    let mut state = vec![NetState::default(); nets.len()];
    let mut ov = Observer { overflow: 0, cnp: t.config.critical_nets_percentage, res_aware: opts.resistance_aware, trees: None, g3: None, g2d: None };
    let run = fastroute_run(&inp, &mut state, &mut ov);
    if let Some(path) = &opts.timer_trace {
        std::fs::write(path, timer_trace.borrow().as_str())?;
    }
    let routes = match run? {
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
        for &id in &net_ids {
            let n = &nets[id];
            let Some(tree) = trees[id].as_ref() else { continue };
            let (route, pins, network) = planar_net_network(n, tree, &rc, &layer_dir, origin, t.min_routing_layer);
            parasitics.insert(n.name.clone(), network);
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
    //
    // Upstream rule (`EstimateParasitics::estimateGlobalRouteRC`): every net of `getRoutes()` with a
    // non-empty route gets a network — whether or not the run ever read the timer. A run with no
    // congestion loop (no partial-slack read) still has routes to estimate.
    let mut routed_parasitics = std::collections::BTreeMap::new();
    {
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
        parasitic_rc: layer_rc(db),
        min_routing_layer: t.min_routing_layer,
        dead: std::collections::BTreeSet::new(),
        held_unrouted: std::collections::BTreeMap::new(),
        // saveGuides' creation order (its closing `reverse()` undoes the head insertion).
        db_guides: guides.iter().map(|ng| (ng.net.clone(), ng.guides.clone())).collect(),
        restore_from_guides: std::collections::BTreeSet::new(),
        eco: Vec::new(),
        merged: std::collections::BTreeMap::new(),
        resistance_aware: opts.resistance_aware,
        res_aware_nets: std::collections::BTreeSet::new(),
        res_tech,
        tile_size: t.core.tile_size,
    };
    let gcell_grid = ((t.core.area.x_min, t.core.x_grids, t.core.tile_size), (t.core.area.y_min, t.core.y_grids, t.core.tile_size));
    Ok(RouteResult { guides, layer_names, total_overflow, guide_is_congested, routes: raw_routes, clock_nets, parasitics, routed_parasitics, parasitic_pins, planar_routes, snapshot_edges, log, already_wired, after, gcell_grid })
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
        let grid_pins: Vec<crate::findrouting::GridPin> = n.net_pins.iter().map(|p| (p.on_grid.0, p.on_grid.1, p.connection_layer)).collect();
        crate::restore::finish_loaded_route(route, &grid_pins, block_min);
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
        parasitic_rc: layer_rc_for(db, &t, opts),
        min_routing_layer: t.min_routing_layer,
        dead: std::collections::BTreeSet::new(),
        held_unrouted: std::collections::BTreeMap::new(),
        // ⚠️ Not kept here: antenna repair runs no journal, so no guide is restored from them.
        db_guides: std::collections::BTreeMap::new(),
        restore_from_guides: std::collections::BTreeSet::new(),
        eco: Vec::new(),
        merged: std::collections::BTreeMap::new(),
        resistance_aware: opts.resistance_aware,
        res_aware_nets: std::collections::BTreeSet::new(),
        res_tech: pricing_tech(db, &t, num_layers)?,
        tile_size: tile,
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

/// The timer's slack of a net by name (`sta_->slack(net, max)`; `sta::INF` when unconstrained), as
/// a resistance-aware incremental run's `updateSlacks` reads it.
pub type NetSlack<'a> = &'a dyn Fn(&str) -> f32;

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
///
/// `save_guides`: `saveGuides(modified_nets)` after the run — `IncrementalGRoute::updateRoutes`'
/// default (true); the antenna repair's own call passes false.
#[allow(clippy::too_many_arguments)]
pub fn update_dirty_routes_fast_route(db: &mut Db, opts: &RouteOptions, a: &mut AfterRoute, dirty: &[String], stt: SteinerBuilder<'_>, flutes: crate::brk_rsmt::Flutes<'_>, obs: IncrObserver<'_>, save_guides: bool, net_slack: Option<NetSlack<'_>>) -> Res<Vec<String>> {
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
    let routes = find_routing(db, opts, a, &dirty_nets, stt, flutes, obs, net_slack)?;
    merge_results(a, routes);
    if a.total_overflow > 0 && !opts.allow_congestion {
        return Err("the incremental re-route left overflow: its congestion loop is not modelled".into());
    }
    if save_guides {
        let names: Vec<String> = dirty_nets.iter().map(|&id| a.router_nets[id].name.clone()).collect();
        for name in names {
            save_net_guides(a, &name)?;
        }
    }
    Ok(dirty_nets.iter().map(|&id| a.router_nets[id].name.clone()).collect())
}

/// `updateNetPins`, for every net: the router's nets as the database stands now (discovery, pins,
/// layer ranges), by name — only the nets FastRoute would route (two pins or more, unwired); a held
/// net missing here has fewer than two pins. ⚠️ Recomputed whole — the database has moved under it
/// — and checked: every such net is one the router holds (the first run's, or one `addNet` made).
fn update_net_pins(db: &mut Db, opts: &RouteOptions, a: &AfterRoute) -> Res<std::collections::BTreeMap<String, RouterNet>> {
    let fresh = fresh_net_pins(db, opts)?;
    if let Some(n) = fresh.keys().find(|n| !holds(a, n)) {
        return Err(format!("net {n}: routable now, and not one the router holds — not modelled").into());
    }
    Ok(fresh)
}

/// Every net FastRoute would route, read from the database now, by name.
fn fresh_net_pins(db: &mut Db, opts: &RouteOptions) -> Res<std::collections::BTreeMap<String, RouterNet>> {
    let t = setup_tech(db, opts)?;
    let mut log = t.log.clone();
    let adj = setup_adjust(db, &t, opts, &mut log)?;
    let mut e = adj.edges;
    let nets = setup_nets(db, &t, &mut e, adj.has_macros_or_pads, opts, &mut log)?;
    Ok(nets.into_iter().map(|n| (n.name.clone(), n)).collect())
}

/// A net's pins as `pinPositionsChanged` compares them: `(on-grid x, y, connection layer)`.
fn pin_key(n: &RouterNet) -> Vec<(i32, i32, i32)> {
    n.net_pins.iter().map(|p| (p.on_grid.0, p.on_grid.1, p.connection_layer)).collect()
}

/// `updateDirtyNets`: of the dirty nets, those whose pins moved — `pinPositionsChanged`, the
/// multiset of `(on-grid x, y, connection layer)` against the positions the net was dirtied with
/// (`saveLastPinPositions`, from the router's own stale pins: the first run's) — are released
/// (`clearNetRoute`) and their routes cleared; the rest keep theirs. In dbNet order.
///
/// A held net FastRoute has no id for (one `addNet` made) is compared against the pins it was held
/// with; when they changed and it now has two pins or more, `initNetlist` → `makeFastrouteNet` gives
/// it the next id (`FastRouteCore::addNet` appends) — assigned here, in the same order.
fn update_dirty_nets(a: &mut AfterRoute, fresh: &std::collections::BTreeMap<String, RouterNet>, dirty: &[String]) -> Res<Vec<usize>> {
    let mut out = Vec::new();
    for name in dirty {
        let now = fresh.get(name).map(pin_key).unwrap_or_default();
        // `isResAware ||` comes first: a net a reroute marked is routed again whatever its pins (and
        // its guides are not read).
        let res_aware = a.res_aware_nets.contains(name);
        // `!loadRoutingFromDBGuides(db_net)` is the next test: a net an undo gave its guides back
        // takes its route from them, and is not re-routed.
        if !res_aware && a.restore_from_guides.contains(name) && a.db_guides.get(name).is_some_and(|g| !g.is_empty()) {
            load_routing_from_db_guides(a, fresh, name)?;
            continue;
        }
        let Some(id) = live_id(a, name) else {
            let Some(stale) = a.held_unrouted.get(name) else { continue }; // not in db_net_map_
            if !crate::netlist::pin_positions_changed(stale, &now) {
                a.held_unrouted.insert(name.clone(), now);
                continue;
            }
            let Some(n) = fresh.get(name) else {
                return Err(format!("net {name}: a held net with fewer than two pins re-routed — addRemainingGuides over it is not modelled").into());
            };
            let id = a.router_nets.len();
            a.router_nets.push(n.clone());
            a.final_state.push(crate::brk_rsmt::NetState::default());
            a.layer_edge_cost.insert(name.clone(), all_layer_edge_costs(n, a.caps.layers.len()));
            let pins = n.net_pins.iter().map(|p| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect();
            a.net_routes.push(crate::NetRoute { name: name.clone(), segments: Vec::new(), pins, is_local: n.is_local });
            a.held_unrouted.remove(name);
            out.push(id);
            continue;
        };
        let key = pin_key;
        let changed = res_aware || crate::netlist::pin_positions_changed(&key(&a.router_nets[id]), &now);
        // `(!isMergedNet || !netIsCovered)`: a merged net its joined route still covers keeps it
        // (and must be connected: GRT-0267). The merged flag is cleared on every pass.
        let merged = a.merged.remove(name).is_some();
        let changed = if changed && merged && !res_aware {
            let pins: Vec<crate::Pin> = fresh.get(name).map(|n| n.net_pins.iter().map(|p| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect()).unwrap_or_default();
            let route = a.net_routes.iter().find(|r| &r.name == name).map(|r| r.segments.clone()).unwrap_or_default();
            if crate::restore::net_is_covered(&route, &pins).is_empty() {
                if !is_connected(&route)? {
                    return Err(format!("[ERROR GRT-0267] Net {name} has disconnected segments.").into());
                }
                false
            } else {
                true
            }
        } else {
            changed
        };
        // A diagnostic: each dirty net's pins as compared (`VYGES_GRT_INCR_TRACE`, appended).
        if let Ok(path) = std::env::var("VYGES_GRT_INCR_TRACE") {
            if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                let _ = std::io::Write::write_all(&mut f, format!("{name}|last={:?}|now={:?}|changed={}\n", key(&a.router_nets[id]), now, i32::from(changed)).as_bytes());
            }
        }
        // Upstream rule: `updateNetPins(net)` runs for EVERY dirty net, before the test — a net
        // that keeps its route still has its pins as they stand now, and the estimator attaches
        // them where they are (a pin replaced inside the same gcell moves its attachment).
        if !changed {
            if let Some(n) = fresh.get(name) {
                a.router_nets[id].net_pins = n.net_pins.clone();
                a.router_nets[id].pin_is_driver = n.pin_is_driver.clone();
            }
            continue;
        }
        // A net restored from guides has no tree: `updateNetResources(net, true)` over its
        // current routes_ releases it, and it is restored no longer.
        if a.final_state[id].segments_restored {
            let segs = a.net_routes.iter().find(|r| &r.name == name).map(|r| r.segments.clone()).unwrap_or_default();
            let lec = a.layer_edge_cost.get(name).cloned().unwrap_or_default();
            if let (Some(g2), Some(g3)) = (a.final_2d.as_mut(), a.final_3d.as_mut()) {
                update_net_resources(g2, g3, &a.jumper_grid, &a.router_nets[id], id, &lec, &segs, -1);
            }
            a.final_state[id].segments_restored = false;
        } else if !merged {
            // ⚠️ A merged net routed again keeps its tree's usage (the reference skips the release).
            clear_net_route(a, id);
        }
        if let Some(r) = a.net_routes.iter_mut().find(|r| &r.name == name) {
            r.segments.clear();
        }
        // `routes_[db_net].clear(); db_net->clearGuides();`
        clear_guides(a, name);
        out.push(id);
    }
    Ok(out)
}

/// `GlobalRouter::loadRoutingFromDBGuides(db_net)` on a net with guides and the restore flag: its
/// current routing released (`updateNetResources(net, true)` for a route restored before, else
/// `clearNetRoute`); `routes_` rebuilt from the guides in list order (`boxToGlobalRouting`), then
/// `dedupViaSegments` and `addImplicitVias` — ⚠️ no `mergeSegments`, unlike the full guide load;
/// the flag cleared, the net restored, `makeFastrouteNet` (its id reset, or the next one for a net
/// the router has none for — one an undo created again) and `updateNetResources(net, false)`.
///
/// ⛔ A pin the rebuilt route does not cover (`updateUncoveredPinsPositions`' repair, or its
/// GRT-0304 fallback to a re-route) is refused.
fn load_routing_from_db_guides(a: &mut AfterRoute, fresh: &std::collections::BTreeMap<String, RouterNet>, name: &str) -> Res<()> {
    let id_now = live_id(a, name);
    if let Some(id) = id_now {
        if a.final_state[id].segments_restored {
            let segs = a.net_routes.iter().find(|r| r.name == name).map(|r| r.segments.clone()).unwrap_or_default();
            let lec = a.layer_edge_cost.get(name).cloned().unwrap_or_default();
            if let (Some(g2), Some(g3)) = (a.final_2d.as_mut(), a.final_3d.as_mut()) {
                update_net_resources(g2, g3, &a.jumper_grid, &a.router_nets[id], id, &lec, &segs, -1);
            }
        } else {
            clear_net_route(a, id);
        }
    }
    let tile = a.jumper_grid.grid.tile_size;
    let mut route = Vec::new();
    for g in &a.db_guides[name] {
        crate::restore::box_to_global_routing((g.box_.x_min, g.box_.y_min, g.box_.x_max, g.box_.y_max), g.layer, g.via_layer, tile, &mut route);
    }
    crate::restore::dedup_via_segments(&mut route);
    crate::restore::add_implicit_vias(&mut route);
    let n = fresh.get(name).ok_or_else(|| format!("net {name}: restored from guides with fewer than two pins — not modelled"))?.clone();
    let pins: Vec<crate::Pin> = n.net_pins.iter().map(|p| crate::Pin { connection_layer: p.connection_layer, on_grid_x: p.on_grid.0, on_grid_y: p.on_grid.1 }).collect();
    let uncovered = crate::restore::net_is_covered(&route, &pins);
    if !uncovered.is_empty() {
        return Err(format!("net {name}: {} pin(s) not covered by the guides an undo restored — updateUncoveredPinsPositions is not modelled", uncovered.len()).into());
    }
    a.restore_from_guides.remove(name);
    let id = match id_now {
        Some(id) => {
            a.router_nets[id] = n.clone();
            a.final_state[id] = crate::brk_rsmt::NetState::default();
            id
        }
        None => {
            a.router_nets.push(n.clone());
            a.final_state.push(crate::brk_rsmt::NetState::default());
            a.layer_edge_cost.insert(name.to_string(), all_layer_edge_costs(&n, a.caps.layers.len()));
            a.held_unrouted.remove(name);
            a.router_nets.len() - 1
        }
    };
    a.final_state[id].segments_restored = true;
    match a.net_routes.iter_mut().find(|r| r.name == name) {
        Some(r) => {
            r.segments = route.clone();
            r.pins = pins;
        }
        None => a.net_routes.push(crate::NetRoute { name: name.to_string(), segments: route.clone(), pins, is_local: n.is_local }),
    }
    let lec = a.layer_edge_cost.get(name).cloned().unwrap_or_default();
    if let (Some(g2), Some(g3)) = (a.final_2d.as_mut(), a.final_3d.as_mut()) {
        update_net_resources(g2, g3, &a.jumper_grid, &a.router_nets[id], id, &lec, &route, 1);
    }
    Ok(())
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
fn init_fast_route_incr(a: &mut AfterRoute, fresh: &std::collections::BTreeMap<String, RouterNet>, dirty_nets: &[usize]) {
    a.net_ids.clear();
    for &id in dirty_nets {
        // A net down to fewer than two pins is not re-added (`initNetlist` skips it): no pins.
        let name = a.router_nets[id].name.clone();
        a.router_nets[id] = fresh.get(&name).cloned().unwrap_or_else(|| RouterNet { pins: Vec::new(), net_pins: Vec::new(), pin_is_driver: Vec::new(), ..a.router_nets[id].clone() });
        // `FrNet::reset` keeps `is_res_aware_` (FastRoute's flag, which `updateSlacks` sets).
        let res_aware = a.final_state[id].res_aware;
        a.final_state[id] = crate::brk_rsmt::NetState { res_aware, ..Default::default() };
        let n = &a.router_nets[id];
        if n.net_pins.len() > 1 && !n.is_local {
            a.net_ids.push(id);
        }
    }
}

/// `findRouting(dirty_nets, …)`: `run()` from the state the first run, the jumpers and the rip-ups
/// left, then the post-processing over those nets (remaining guides, pad pins, `mergeSegments`).
#[allow(clippy::too_many_arguments)]
fn find_routing(db: &Db, opts: &RouteOptions, a: &mut AfterRoute, dirty_nets: &[usize], stt: SteinerBuilder<'_>, flutes: crate::brk_rsmt::Flutes<'_>, obs: IncrObserver<'_>, net_slack: Option<NetSlack<'_>>) -> Res<std::collections::BTreeMap<String, Vec<crate::GSegment>>> {
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
    // `setResistanceAware(resistance_aware_)`: once a reroute turned it on, every incremental run is
    // resistance-aware, and its `updateSlacks` asks the timer for each routed net's slack.
    // ⚠️ With one net routed the slack decides only whether it is constrained (INF or not): its
    // ordering score has nothing to order. Several nets are refused — the timer's mid-update slacks
    // would order them.
    let res_slacks: Vec<f32>;
    let ra_inputs = if a.resistance_aware {
        if a.net_ids.len() > 1 {
            return Err(format!("a resistance-aware incremental re-route of {} nets: the timer's slacks mid-update order them — not modelled", a.net_ids.len()).into());
        }
        let f = net_slack.ok_or("a resistance-aware incremental re-route with no timer bound")?;
        res_slacks = (0..nets.len()).map(|k| if a.net_ids.contains(&k) { f(&nets[k].name) } else { 1e30 }).collect();
        if let Some(k) = (0..nets.len()).find(|&k| res_slacks[k].is_nan()) {
            return Err(format!("net {}: routed resistance-aware with no slack the timer can give mid-update — not modelled", nets[k].name).into());
        }
        Some(crate::run::ResAwareInputs { tech: a.res_tech.clone(), tile_size: a.tile_size, fixed_percentage: opts.res_aware_nets_percentage, update_slacks: crate::congestion_loop::TimerSlack::Every(&res_slacks) })
    } else {
        None
    };
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
        resistance_aware: a.resistance_aware,
        liberty: opts.liberty.is_some(),
        timer_slack: crate::congestion_loop::TimerSlack::None,
        res_aware: ra_inputs,
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
