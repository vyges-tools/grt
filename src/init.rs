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

/// What the liberty lookup says about one instance terminal on a net.
///
/// ⚠️ These are **inputs**, not something this crate derives: they come from the timing library,
/// which is a different substrate entirely.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ITermClockFacts {
    /// Whether the terminal resolves to a liberty port at all.
    pub has_liberty_port: bool,
    /// Whether that port is a register's clock input.
    pub is_reg_clk: bool,
    /// Whether the cell it belongs to is a pad.
    pub cell_is_pad: bool,
}

/// Whether a terminal counts as a clock terminal.
///
/// ⚠️ **A pad terminal counts even when it is not a register clock pin.** The two conditions are
/// an OR, and both are gated on the port existing at all — a terminal with no liberty port is
/// never a clock terminal, whatever its cell.
pub fn is_clk_term(f: ITermClockFacts) -> bool {
    f.has_liberty_port && (f.is_reg_clk || f.cell_is_pad)
}

/// Whether a net is a clock net **above the leaves**.
///
/// ⛔ **"Clock net" is not the same as "clock-typed net".** A net typed as clock that reaches any
/// clock terminal is a LEAF and answers false; only one that reaches none of them is a non-leaf
/// clock. Getting this backwards puts the leaf nets at the front of the routing order.
///
/// ⚠️ A net that is not clock-typed answers false without looking at its terminals.
pub fn is_non_leaf_clock(sig_type_is_clock: bool, iterms: &[ITermClockFacts]) -> bool {
    sig_type_is_clock && !iterms.iter().copied().any(is_clk_term)
}

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

/// I3 — `configFastRoute`: push the run's options into the router.
///
/// ⛔ With no Liberty loaded the critical-nets percentage is FORCED to 0, with GRT-300 — every run,
/// whatever it was (a warning even when it already was 0). The value persists in the router.
pub fn config_fast_route(opts: &SetupOptions, log: &mut Vec<String>) -> FastRouteConfig {
    let mut critical_nets_percentage = opts.critical_nets_percentage;
    if !opts.has_liberty {
        log.push("[WARNING GRT-0300] Timing is not available, setting critical nets percentage to 0.".into());
        critical_nets_percentage = 0.0;
    }
    FastRouteConfig { critical_nets_percentage }
}

/// I5 — `reportLayerSettings`: GRT-20 … GRT-23, verbose only.
///
/// ⛔ `int(adjustment_ * 100)` multiplies in `float` and TRUNCATES: 0.29f × 100 is 29.0f (29%),
/// where the same product in `double` is 28.99999… (28%).
pub fn report_layer_settings(opts: &SetupOptions, log: &mut Vec<String>) {
    if opts.verbose {
        log.push(format!("[INFO GRT-0020] Min routing layer: {}", opts.min_layer_name));
        log.push(format!("[INFO GRT-0021] Max routing layer: {}", opts.max_layer_name));
        log.push(format!("[INFO GRT-0022] Global adjustment: {}%", (opts.adjustment * 100.0f32) as i32));
        log.push(format!("[INFO GRT-0023] Grid origin: ({}, {})", opts.grid_origin.0, opts.grid_origin.1));
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
    /// I14 — build the router's netlist, its degrees and its pin-access resources.
    InitNetlist,
}

/// What a setup run produced, and what it did not.
#[derive(Debug, Clone)]
pub struct SetupReport {
    pub config: FastRouteConfig,
    pub grid: CoreGrid,
    /// The reference's log lines the stages emit, in order.
    pub log: Vec<String>,
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
