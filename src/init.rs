// SPDX-License-Identifier: Apache-2.0
//! Setting up a routing run: the grid the router works on, and the nets it will route.
//!
//! This is the stage that produces the inputs the guide writer consumes. It reads in the same
//! order the published stage runs — the functions below are named after that stage's own, and
//! [`init_fast_route`] is a sequencer and nothing else, so a divergence can be pointed at one
//! line rather than bisected.

use crate::{Grid, Pin, Rect};

/// The routing grid, as the setup stage derives it.
///
/// ⚠️ Wider than [`Grid`], which carries only what guide geometry reads. The cell counts and the
/// regularity flags are consumed by the router, not by the guide writer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoreGrid {
    pub area: Rect,
    pub tile_size: i32,
    pub x_grids: i32,
    pub y_grids: i32,
    /// Whether the cells tile the die exactly, with no partial cell at the top edge.
    pub perfect_regular_x: bool,
    pub perfect_regular_y: bool,
    pub num_layers: i32,
}

impl CoreGrid {
    /// The subset guide geometry needs.
    pub fn grid(&self) -> Grid {
        Grid { tile_size: self.tile_size, area: self.area }
    }
}

/// Derive the routing grid from the die area and the cell size.
///
/// ⚠️ **The DIE area, not the core area.** Guide boxes are snapped against this rectangle, so
/// using the core area would move every guide that lands near the boundary.
///
/// ⚠️ **`dx / tile_size` is a truncating divide, then clamped to at least 1.** A die narrower
/// than one cell still gets one column rather than none, and a die that is not a whole number of
/// cells across loses the remainder — which is exactly what `perfect_regular_x` then records.
///
/// `max_layer` of `-1` means "every routing layer"; otherwise it caps the count.
pub fn init_grid(area: Rect, tile_size: i32, routing_layer_count: i32, max_layer: i32) -> CoreGrid {
    let dx = area.x_max - area.x_min;
    let dy = area.y_max - area.y_min;

    let x_grids = std::cmp::max(1, dx / tile_size);
    let y_grids = std::cmp::max(1, dy / tile_size);

    CoreGrid {
        area,
        tile_size,
        x_grids,
        y_grids,
        perfect_regular_x: x_grids * tile_size == dx,
        perfect_regular_y: y_grids * tile_size == dy,
        num_layers: if max_layer > -1 { max_layer } else { routing_layer_count },
    }
}

impl CoreGrid {
    /// Snap a coordinate to the centre of the grid cell that contains it.
    ///
    /// ⛔ **Everything about pin placement funnels through this**, so an error here moves every
    /// pin and therefore every guide that covers one.
    ///
    /// ⚠️ **The cell index is a truncating divide**, matching the reference's integer division —
    /// Rust and C++ both truncate toward zero, so a coordinate below the die origin behaves the
    /// same in both rather than flooring.
    ///
    /// ⚠️ **A point in the partial cell past the last full one is pulled BACK into it.** The die
    /// need not be a whole number of cells across (that is what `perfect_regular_*` records), so
    /// a coordinate near the top edge can land at index `x_grids`, which is one past the end. The
    /// decrement is the counterpart of that remainder, not an off-by-one guard.
    ///
    /// ⚠️ `tile_size / 2` truncates too: on an odd cell size the centre sits half a unit low.
    pub fn position_on_grid(&self, x: i32, y: i32) -> (i32, i32) {
        let mut gcell_id_x = (x - self.area.x_min) / self.tile_size;
        let mut gcell_id_y = (y - self.area.y_min) / self.tile_size;

        if gcell_id_x >= self.x_grids {
            gcell_id_x -= 1;
        }
        if gcell_id_y >= self.y_grids {
            gcell_id_y -= 1;
        }

        (
            gcell_id_x * self.tile_size + self.tile_size / 2 + self.area.x_min,
            gcell_id_y * self.tile_size + self.tile_size / 2 + self.area.y_min,
        )
    }
}

/// Whether a net sits entirely on one grid point.
///
/// ⛔ **Position only — the LAYER is not compared.** Two pins at the same `(x, y)` on different
/// layers make a local net. That is what decides the two-guide via form, so comparing the layer
/// as well would change the guide COUNT on every such net.
///
/// ⚠️ **A net with no pins is local.** The empty case returns true, not false.
pub fn is_local(pins: &[Pin]) -> bool {
    match pins.split_first() {
        None => true,
        Some((first, rest)) => rest
            .iter()
            .all(|p| p.on_grid_x == first.on_grid_x && p.on_grid_y == first.on_grid_y),
    }
}

/// Whether a database net is routable at all, or skipped before anything else looks at it.
///
/// ⚠️ **Four conditions, all required**, and this is *not* the same predicate as the one guarding
/// the large-fanout skip further down the stage — that one is `isSupply() && isSpecial()`, an AND
/// of two of these. Conflating them changes which nets are reported.
pub fn is_routable(
    is_supply: bool,
    is_special: bool,
    has_special_wires: bool,
    connected_by_abutment: bool,
) -> bool {
    !is_supply && !is_special && !has_special_wires && !connected_by_abutment
}

// `ITermClockFacts`, `is_clk_term` and `is_non_leaf_clock` live in the estimator crate
// (`vyges_est::liberty`), which needs them too and which this crate links.
pub use vyges_est::liberty::{is_clk_term, is_non_leaf_clock, ITermClockFacts};

/// A net as net discovery sees it, before ordering.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiscoveredNet {
    pub name: String,
    pub is_non_leaf_clock: bool,
}

/// Order the nets the way the published stage hands them to the router.
///
/// ⛔ **Non-leaf clock nets first, then everything else, each group sorted by NAME.** Upstream
/// does this deliberately — "to ensure stable results" — so the database's own order is *erased*
/// before anything routes. An engine that kept database order would route in a different sequence
/// and produce different rip-up decisions downstream.
///
/// ⚠️ **The partition is not implied by the sort.** On many designs the clock nets happen to sort
/// first anyway and the two are indistinguishable; on others they do not. Sorting the whole list
/// by name is a different function.
///
/// ⚠️ The comparison is plain byte order on the name, not a natural or numeric ordering, so
/// `net10` sorts before `net9`.
pub fn order_nets(nets: &[DiscoveredNet]) -> Vec<String> {
    let mut clk: Vec<&str> = nets.iter().filter(|n| n.is_non_leaf_clock)
        .map(|n| n.name.as_str()).collect();
    let mut rest: Vec<&str> = nets.iter().filter(|n| !n.is_non_leaf_clock)
        .map(|n| n.name.as_str()).collect();
    clk.sort_unstable();
    rest.sort_unstable();
    clk.into_iter().chain(rest).map(str::to_string).collect()
}

/// The run's options that the first setup stages read (I3, I5).
#[derive(Debug, Clone, PartialEq)]
pub struct SetupOptions {
    pub verbose: bool,
    /// Whether any Liberty library is loaded (`defaultLibertyLibrary() != nullptr`).
    pub has_liberty: bool,
    /// The router's critical-nets percentage going IN — it is router state, and survives across
    /// runs: `clear()` does not reset it.
    pub critical_nets_percentage: f32,
    /// `adjustment_` — a `float`, set by the global layer adjustment.
    pub adjustment: f32,
    pub grid_origin: (i32, i32),
    /// The names of the min and max routing layers, as indexed by I4.
    pub min_layer_name: String,
    pub max_layer_name: String,
}

/// What `configFastRoute` leaves in the router that later stages read.
#[derive(Debug, Clone, PartialEq)]
pub struct FastRouteConfig {
    pub critical_nets_percentage: f32,
}

/// The critical-nets percentage is a MEMBER of each router, not an option of the call: FastRoute's
/// and CUGR's are separate, both 10 at construction, and they persist across `global_route` calls.
///
/// - `setCriticalNetsPercentage` (the `-critical_nets_percentage` flag, set by the Tcl AFTER
///   `set_use_cugr`) writes the CURRENT router's member — 0 without a liberty library (GRT-301).
/// - `configFastRoute` (every FastRoute call) zeroes FastRoute's without a liberty library.
/// - `initCUGR` zeroes CUGR's without a liberty library, after GRT-0309 when it was not yet 0 —
///   so a second CUGR call in the session does not warn again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CriticalNetsPercentages {
    pub fast_route: f32,
    pub cugr: f32,
}

impl Default for CriticalNetsPercentages {
    fn default() -> Self {
        CriticalNetsPercentages { fast_route: 10.0, cugr: 10.0 }
    }
}

impl CriticalNetsPercentages {
    /// The current router's member.
    pub fn get(&self, use_cugr: bool) -> f32 {
        if use_cugr { self.cugr } else { self.fast_route }
    }

    /// `setCriticalNetsPercentage`: the current router's member, 0 without a liberty library.
    pub fn set_critical_nets_percentage(&mut self, use_cugr: bool, percentage: f32, has_liberty: bool) {
        let p = if has_liberty { percentage } else { 0.0 };
        if use_cugr { self.cugr = p } else { self.fast_route = p }
    }

    /// What a call leaves in the member without a liberty library: `configFastRoute` zeroes
    /// FastRoute's, `initCUGR` CUGR's.
    pub fn after_call(&mut self, use_cugr: bool, has_liberty: bool) {
        if !has_liberty {
            if use_cugr { self.cugr = 0.0 } else { self.fast_route = 0.0 }
        }
    }
}

/// I3 — `configFastRoute`: push the run's options into the router.
///
/// ⛔ With no Liberty loaded the critical-nets percentage is FORCED to 0, with GRT-300 — every run,
/// whatever it was (a warning even when it already was 0). The value persists in the router.
pub fn config_fast_route(opts: &SetupOptions, log: &mut dyn crate::log::Sink) -> FastRouteConfig {
    let mut critical_nets_percentage = opts.critical_nets_percentage;
    if !opts.has_liberty {
        log.warn("GRT", 300, "Timing is not available, setting critical nets percentage to 0.");
        critical_nets_percentage = 0.0;
    }
    FastRouteConfig { critical_nets_percentage }
}

/// I5 — `reportLayerSettings`: GRT-20 … GRT-23, verbose only.
///
/// ⛔ `int(adjustment_ * 100)` multiplies in `float` and TRUNCATES: 0.29f × 100 is 29.0f (29%),
/// where the same product in `double` is 28.99999… (28%).
pub fn report_layer_settings(opts: &SetupOptions, log: &mut dyn crate::log::Sink) {
    if opts.verbose {
        log.info("GRT", 20, format!("Min routing layer: {}", opts.min_layer_name));
        log.info("GRT", 21, format!("Max routing layer: {}", opts.max_layer_name));
        log.info("GRT", 22, format!("Global adjustment: {}%", (opts.adjustment * 100.0f32) as i32));
        log.info("GRT", 23, format!("Grid origin: ({}, {})", opts.grid_origin.0, opts.grid_origin.1));
    }
}

/// `reportMacrosAndBlockages`' counts, read from the database alone (the CUGR path's source; the
/// FastRoute path counts the same while it applies the obstructions): every block obstruction
/// whose layer's routing level is in `[min, max]`; per INSTANCE, one macro if its master is a block
/// and each of the master's obstruction boxes in range. `block_obs` and each master's are routing
/// levels (0 for a layer that is not a routing layer); `insts` is one entry per instance.
pub fn count_macros_and_blockages<'a>(min: i32, max: i32, block_obs: &[i32], insts: impl IntoIterator<Item = (bool, &'a [i32])>) -> (usize, usize) {
    let in_range = |l: &&i32| min <= **l && **l <= max;
    let mut blockages = block_obs.iter().filter(in_range).count();
    let mut macros = 0;
    for (is_block, obs) in insts {
        macros += usize::from(is_block);
        blockages += obs.iter().filter(in_range).count();
    }
    (macros, blockages)
}

/// `reportMacrosAndBlockages`: GRT-3 / GRT-4, verbose only, after the obstruction adjustments.
pub fn report_macros_and_blockages(verbose: bool, macros: usize, blockages: usize, log: &mut dyn crate::log::Sink) {
    if verbose {
        log.info("GRT", 3, format!("Macros: {macros}"));
        log.info("GRT", 4, format!("Blockages: {blockages}"));
    }
}

/// `initClockNets`: GRT-19, verbose only — the timer's clock nets, none without a Liberty library.
pub fn report_clock_nets(verbose: bool, found: usize, log: &mut dyn crate::log::Sink) {
    if verbose {
        log.info("GRT", 19, format!("Found {found} clock nets."));
    }
}

/// A stage of the setup sequence that this engine does not implement yet.
///
/// 🔑 **Named, not omitted.** A stage that simply is not called produces no diff to chase — it
/// produces a subtly wrong answer somewhere else — so the sequencer runs the whole published
/// order and reports the gaps by name instead of silently skipping them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AbsentStage {
    /// I4 — validate layer directions and track grids.
    InitRoutingLayers,
    /// I6 — per-layer track pitch and line-to-via pitch. The stage itself is implemented
    /// ([`crate::init_routing_tracks`]); it is not wired here until I4 hands it the technology.
    InitRoutingTracks,
    /// I8 — mirror the grid into the router's own coordinates. Implemented
    /// ([`crate::mirror_grid_to_fast_route`]); wired once I4 hands the sequencer the technology.
    MirrorGridToFastRoute,
    /// I9 — per-edge capacity from the track counts. Implemented ([`crate::set_capacities`]).
    SetCapacities,
    /// I10 — obstructions, blockages and the user's layer/region adjustments. The adjustments
    /// themselves are implemented ([`crate::adjust`]); the database walk that PRODUCES the
    /// obstruction rectangles (instances, pins, wires, macros) is not.
    ApplyAdjustments,
    /// I11 — seeded capacity perturbation. ⚠️ Inert unless a seed is set, and no published case
    /// exercises it; the distributions it draws through are implementation-defined besides.
    PerturbCapacities,
    /// I12 — per-layer edge capacity roll-up. Implemented ([`crate::init_edges_capacity_per_layer`]).
    InitEdgesCapacityPerLayer,
    /// I13a — reading the nets off the database. The ORDER ([`order_nets`]), the filter
    /// ([`crate::find_nets`]) and every pin ([`crate::pins`]) are implemented; what is absent is
    /// the database walk that hands them their facts.
    FindNetsFromDatabase,
    /// I13b — reject ports sharing a position on a layer. Implemented
    /// ([`crate::check_pin_placement`]); not wired until the nets are.
    CheckPinPlacement,
    /// I14 — build the router's netlist, its degrees and its pin-access resources. Implemented
    /// ([`crate::netlist`]); not wired until the nets are.
    InitNetlist,
}

/// What a setup run produced, and what it did not.
#[derive(Debug, Clone)]
pub struct SetupReport {
    pub config: FastRouteConfig,
    pub grid: CoreGrid,
    /// The reference's log lines the stages emit, in order.
    pub log: Vec<crate::log::Msg>,
    /// ⬜ The stages of the published order this engine does not run yet, in that order.
    pub absent: Vec<AbsentStage>,
}

/// The setup sequence, in the published order.
///
/// ⛔ **This is a sequencer and does no work of its own.** Each stage is its own function above;
/// the order here is the order there. Keeping the shape means a trace of the two can be read side
/// by side instead of bisected — which is a debugging property, not a stylistic one.
///
/// The nets (I13a) are supplied by the caller until net discovery is implemented.
pub fn init_fast_route(
    opts: &SetupOptions,
    area: Rect,
    tile_size: i32,
    routing_layer_count: i32,
    max_layer: i32,
) -> SetupReport {
    use AbsentStage::*;
    let mut log = Vec::new();
    // I1, I2 — clearing the router: this engine builds its state fresh, and nothing observable
    // survives `clear()` except the options it does not touch (see `SetupOptions`).
    let config = config_fast_route(opts, &mut log); // I3 ✅
    // I4 ⬜ (`init_routing_layers` exists; the layer names come in through the options)
    report_layer_settings(opts, &mut log); // I5 ✅
    // I6 ⬜
    let grid = init_grid(area, tile_size, routing_layer_count, max_layer); // I7 ✅
    // I8 ⬜  I9 ⬜  I10 ⬜  I11 n/a  I12 ⬜  I13b ⬜  I14 ⬜
    SetupReport {
        config,
        grid,
        log,
        absent: vec![
            InitRoutingLayers,
            InitRoutingTracks,
            MirrorGridToFastRoute,
            SetCapacities,
            ApplyAdjustments,
            PerturbCapacities,
            InitEdgesCapacityPerLayer,
            FindNetsFromDatabase,
            CheckPinPlacement,
            InitNetlist,
        ],
    }
}

#[cfg(test)]
mod critical_nets_tests {
    use super::CriticalNetsPercentages;

    /// Upstream rule (grt at the pin): each router owns its critical-nets percentage, 10 at
    /// construction; the flag writes only the current router's (0 without liberty, GRT-301);
    /// initCUGR / configFastRoute zero their own router's without liberty, and nothing resets it
    /// between calls — so a second CUGR call without liberty sees 0 and does not warn GRT-0309.
    #[test]
    fn each_router_keeps_its_own_percentage_across_calls() {
        let mut c = CriticalNetsPercentages::default();
        assert_eq!((c.get(false), c.get(true)), (10.0, 10.0));
        // With liberty: the flag sets only the current router's member.
        c.set_critical_nets_percentage(true, 30.0, true);
        assert_eq!((c.get(false), c.get(true)), (10.0, 30.0));
        c.after_call(true, true);
        assert_eq!(c.get(true), 30.0, "a call with liberty keeps the member");
        // Without liberty: the flag writes 0, for that router only.
        let mut c = CriticalNetsPercentages::default();
        c.set_critical_nets_percentage(false, 30.0, false);
        assert_eq!((c.get(false), c.get(true)), (0.0, 10.0));
        // A CUGR call without liberty zeroes CUGR's member; the next CUGR call sees 0.
        let mut c = CriticalNetsPercentages::default();
        assert_eq!(c.get(true), 10.0, "first CUGR call sees 10 and warns GRT-0309");
        c.after_call(true, false);
        assert_eq!((c.get(false), c.get(true)), (10.0, 0.0), "second sees 0; FastRoute's untouched");
    }
}
