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

/// A stage of the setup sequence that this engine does not implement yet.
///
/// 🔑 **Named, not omitted.** A stage that simply is not called produces no diff to chase — it
/// produces a subtly wrong answer somewhere else — so the sequencer runs the whole published
/// order and reports the gaps by name instead of silently skipping them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AbsentStage {
    /// I1/I2 — reset the router's edge state and its per-position net maps.
    ClearRouterState,
    /// I3 — push the run's options into the router.
    ConfigFastRoute,
    /// I4/I5 — validate layer directions and track grids, then report the layer settings.
    InitRoutingLayers,
    /// I6 — per-layer track pitch and line-to-via pitch.
    InitRoutingTracks,
    /// I8 — mirror the grid into the router's own coordinates.
    MirrorGridToFastRoute,
    /// I9 — per-edge capacity from the track counts.
    SetCapacities,
    /// I10 — obstructions, blockages and the user's layer/region adjustments.
    ApplyAdjustments,
    /// I11 — seeded capacity perturbation. ⚠️ Inert unless a seed is set, and no published case
    /// exercises it; the distributions it draws through are implementation-defined besides.
    PerturbCapacities,
    /// I12 — per-layer edge capacity roll-up.
    InitEdgesCapacityPerLayer,
    /// I13b — reject pins that cannot be reached on their own layer.
    CheckPinPlacement,
    /// I14 — build the router's netlist, its degrees and its pin-access resources.
    InitNetlist,
}

/// What a setup run produced, and what it did not.
#[derive(Debug, Clone)]
pub struct SetupReport {
    pub grid: CoreGrid,
    /// ⬜ The stages of the published order this engine does not run yet, in that order.
    pub absent: Vec<AbsentStage>,
}

/// The setup sequence, in the published order.
///
/// ⛔ **This is a sequencer and does no work of its own.** Each stage is its own function above;
/// the order here is the order there. Keeping the shape means a trace of the two can be read side
/// by side instead of bisected — which is a debugging property, not a stylistic one.
///
/// Today it derives the grid (I7) and reports every other stage as absent. The nets (I13a) are
/// supplied by the caller until net discovery is implemented.
pub fn init_fast_route(
    area: Rect,
    tile_size: i32,
    routing_layer_count: i32,
    max_layer: i32,
) -> SetupReport {
    use AbsentStage::*;
    SetupReport {
        // I1, I2 ⬜  I3 ⬜  I4, I5 ⬜  I6 ⬜
        // I7 ✅ — the grid, and its track pitches are part of the absent I6.
        grid: init_grid(area, tile_size, routing_layer_count, max_layer),
        // I8 ⬜  I9 ⬜  I10 ⬜  I11 n/a  I12 ⬜  I13b ⬜  I14 ⬜
        absent: vec![
            ClearRouterState,
            ConfigFastRoute,
            InitRoutingLayers,
            InitRoutingTracks,
            MirrorGridToFastRoute,
            SetCapacities,
            ApplyAdjustments,
            PerturbCapacities,
            InitEdgesCapacityPerLayer,
            CheckPinPlacement,
            InitNetlist,
        ],
    }
}
