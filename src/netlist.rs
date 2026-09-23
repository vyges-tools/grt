// SPDX-License-Identifier: Apache-2.0
//! I14 — `initNetlist`: which nets reach the router, with what layer range, pins, root and edge
//! costs; the net degree; and the extra pin-access resources for pad and macro pins.
//!
//! In the reference's order: (the seeded shuffle — inert, see [`initial_net_order_is_kept`]), per
//! net [`get_net_layer_range`] and the gate [`makes_fastroute_net`] (→ [`find_fastroute_pins`],
//! [`compute_track_consumption`]), then [`compute_net_degree`] / [`report_net_degree`], then
//! [`pin_access_edges`] applied with `addAdjustment` — only with macros or pads, never incremental.

use crate::adjust::RouterEdges;
use crate::capacity::Direction;
use crate::pins::PinEdge;

/// The seeded shuffle (`utl::shuffle` over `std::mt19937`) runs only when `seed_ != 0` and there
/// is more than one net. ⚠️ No corpus test sets a seed (`set_global_routing_random`): the order is
/// always kept, and this engine implements only that branch.
pub fn initial_net_order_is_kept(seed: i32, net_count: usize) -> bool {
    net_count <= 1 || seed == 0
}

/// `getNetLayerRange` → `(min, max)`.
///
/// ⛔ The minimum is raised to the LOWEST pin connection layer — a net never routes below its
/// lowest pin. A non-leaf clock net takes the clock layers when they are set (> 0).
pub fn get_net_layer_range(
    pin_connection_layers: &[i32],
    is_non_leaf_clock: bool,
    block_min: i32,
    block_max: i32,
    clock_min: i32,
    clock_max: i32,
) -> (i32, i32) {
    let pin_min = pin_connection_layers.iter().copied().min().unwrap_or(i32::MAX);
    let min = if is_non_leaf_clock && clock_min > 0 { clock_min } else { block_min };
    let max = if is_non_leaf_clock && clock_max > 0 { clock_max } else { block_max };
    (min.max(pin_min), max)
}

/// `Net::hasStackedVias(max_routing_layer)`: a pre-routed net made ONLY of vias (no wire
/// segments), with exactly as many via points as it has block terminals lying wholly above the max
/// layer — the stacks that bring those terminals down, which the router must still connect.
///
/// A terminal's lowest level counts every box (a non-routing box is level 0); none → `INT_MAX`.
pub fn has_stacked_vias(wire_cnt: u32, via_cnt: u32, via_points: usize, bterm_bottom_levels: &[i32], max_level: i32) -> bool {
    let above = bterm_bottom_levels.iter().filter(|&&b| b > max_level).count();
    if wire_cnt != 0 || via_cnt == 0 {
        return false;
    }
    via_points == above
}

/// `pinPositionsChanged` (the incremental re-route's filter, `updateDirtyNets`): the pins' `(on-grid x, y, connection layer)` against the last positions,
/// as MULTISETS (`std::map<RoutePt, int>` counts up, then down) — order is ignored, multiplicity is
/// not.
pub fn pin_positions_changed(last: &[(i32, i32, i32)], now: &[(i32, i32, i32)]) -> bool {
    let sorted = |v: &[(i32, i32, i32)]| {
        let mut v = v.to_vec();
        v.sort_unstable();
        v
    };
    sorted(last) != sorted(now)
}

/// The gate: a net reaches the router with more than one pin and either no wires, or only the via
/// stacks of [`has_stacked_vias`]. ⚠️ A net with ordinary pre-routed wires is silently absent.
pub fn makes_fastroute_net(pin_count: usize, has_wires: bool, stacked_vias: impl FnOnce() -> bool) -> bool {
    pin_count > 1 && (!has_wires || stacked_vias())
}

/// `getNetMaxRoutingLayer`: the clock max for a net whose SIGNAL TYPE is clock (not the non-leaf
/// test [`get_net_layer_range`] uses), when set.
pub fn net_max_routing_layer(sig_is_clock: bool, clock_max: i32, block_max: i32) -> i32 {
    if sig_is_clock && clock_max > 0 { clock_max } else { block_max }
}

/// A pin as `findFastRoutePins` reads it: on-grid position, connection layer, driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RouterPinFacts {
    pub on_grid: (i32, i32),
    pub connection_layer: i32,
    pub is_driver: bool,
}

/// The grid `findFastRoutePins` indexes: lower-left, cell size, counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetlistGrid {
    pub x_min: i32,
    pub y_min: i32,
    pub tile_size: i32,
    pub x_grids: i32,
    pub y_grids: i32,
    pub num_layers: i32,
}

/// `findFastRoutePins` → `(pins as (x, y, level), root index)`.
///
/// Each pin's cell, its connection layer CAPPED at the net's max; kept when on the grid and not a
/// duplicate of an earlier kept pin. The root is the LAST driver kept (0 if none).
///
/// ⛔ The bounds are asymmetric: `x >= 0` but `y >= -1`.
pub fn find_fastroute_pins(pins: &[RouterPinFacts], grid: NetlistGrid, max_routing_layer: i32) -> (Vec<(i32, i32, i32)>, usize) {
    let mut out: Vec<(i32, i32, i32)> = Vec::new();
    let mut root = 0;
    for p in pins {
        let conn = p.connection_layer.min(max_routing_layer);
        let x = (p.on_grid.0 - grid.x_min) / grid.tile_size;
        let y = (p.on_grid.1 - grid.y_min) / grid.tile_size;
        if x >= 0 && x < grid.x_grids && y >= -1 && y < grid.y_grids && conn <= grid.num_layers && conn > 0 {
            if !out.contains(&(x, y, conn)) {
                out.push((x, y, conn));
                if p.is_driver {
                    root = out.len() - 1;
                }
            }
        }
    }
    (out, root)
}

/// One NDR layer rule as `computeTrackConsumption` reads it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NdrLayerRule {
    pub level: i32,
    pub default_width: i32,
    /// The layer's track pitch (`getRoutingTracksByIndex(level)`).
    pub default_pitch: i32,
    pub ndr_spacing: i32,
    pub ndr_width: i32,
}

/// NDR consumption above `int8_t`'s range (GRT-272).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NdrConsumptionTooLarge(pub i32);

/// `computeTrackConsumption` → `(the net's edge cost, per-layer costs)`; `None` per-layer without
/// an NDR.
///
/// Per rule on a level inside the BLOCK's routing range (not the net's):
/// `ndr_pitch = (ndr_width + 2 * ndr_spacing + default_width) / 2` (integer), and
/// `consumption = 2 * ceil((float) ndr_pitch / default_pitch) - 1` — both sides of the wire. The
/// per-layer vector has `num_layers + 1` entries, all 1, indexed by `level - 1`.
pub fn compute_track_consumption(
    ndr_rules: Option<&[NdrLayerRule]>,
    block_min: i32,
    block_max: i32,
    num_layers: i32,
) -> Result<(i8, Option<Vec<i8>>), NdrConsumptionTooLarge> {
    let Some(rules) = ndr_rules else { return Ok((1, None)) };
    let mut per_layer = vec![1i8; num_layers as usize + 1];
    let mut track_consumption = 1i8;
    for r in rules {
        if r.level > block_max || r.level < block_min {
            continue;
        }
        let ndr_pitch = (r.ndr_width + 2 * r.ndr_spacing + r.default_width) / 2;
        let consumption = (2.0 * (ndr_pitch as f32 / r.default_pitch as f32).ceil() - 1.0) as i32;
        if consumption > i8::MAX as i32 {
            return Err(NdrConsumptionTooLarge(consumption));
        }
        per_layer[(r.level - 1) as usize] = consumption as i8;
        track_consumption = track_consumption.max(consumption as i8);
    }
    Ok((track_consumption, Some(per_layer)))
}

/// `computeNetDegree` over `(pin count, made)` → `(min, max)`: over the nets the gate lets through,
/// `min` from `INT_MAX`, `max` from 1 (never below, so the router's vectors can be sized).
pub fn compute_net_degree(nets: &[(usize, bool)]) -> (i32, i32) {
    let (mut min, mut max) = (i32::MAX, 1);
    for &(pins, made) in nets {
        if made {
            min = min.min(pins as i32);
            max = max.max(pins as i32);
        }
    }
    (min, max)
}

/// `reportNetDegree`: GRT-1 / GRT-2 when verbose — zeros when no net passed the gate.
pub fn report_net_degree(nets: &[(usize, bool)], verbose: bool, log: &mut Vec<String>) {
    if !verbose {
        return;
    }
    let (mut min, mut max) = compute_net_degree(nets);
    if nets.is_empty() || min == i32::MAX {
        min = 0;
        max = 0;
    }
    log.push(format!("[INFO GRT-0001] Minimum degree: {min}"));
    log.push(format!("[INFO GRT-0002] Maximum degree: {max}"));
}

/// A pin as `addResourcesForPinAccess` reads it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AccessPinFacts {
    pub on_grid: (i32, i32),
    pub connection_layer: i32,
    pub edge: PinEdge,
    pub connected_to_pad_or_macro: bool,
}

/// `addResourcesForPinAccess`: the edges that gain one track, in order — for each pad/macro pin
/// with an edge (of a net connected to a pad or macro), the edge leaving its cell TOWARD the
/// instance edge it faces: up for a north pin on a vertical layer (down otherwise), right for an
/// east pin on a horizontal one (left otherwise); none off the grid's low side.
///
/// Returns `(x1, y1, x2, y2, layer)`; apply each with `addAdjustment(cap + 1, is_reduce = false)`.
pub fn pin_access_edges(
    nets: &[(bool, Vec<AccessPinFacts>)],
    grid: NetlistGrid,
    directions: &dyn Fn(i32) -> Option<Direction>,
) -> Vec<(i32, i32, i32, i32, i32)> {
    let mut out = Vec::new();
    for (net_connected, pins) in nets {
        if !net_connected {
            continue;
        }
        for p in pins {
            if !(p.connected_to_pad_or_macro && p.edge != PinEdge::None) {
                continue;
            }
            let px = (p.on_grid.0 - grid.x_min) / grid.tile_size;
            let py = (p.on_grid.1 - grid.y_min) / grid.tile_size;
            let layer = p.connection_layer;
            if directions(layer) == Some(Direction::Vertical) {
                let north = p.edge == PinEdge::North;
                let (y1, y2) = if north { (py, py + 1) } else { (py - 1, py) };
                if y1 < 0 {
                    continue;
                }
                out.push((px, y1, px, y2, layer));
            } else {
                let east = p.edge == PinEdge::East;
                let (x1, x2) = if east { (px, px + 1) } else { (px - 1, px) };
                if x1 < 0 {
                    continue;
                }
                out.push((x1, py, x2, py, layer));
            }
        }
    }
    out
}

/// Apply [`pin_access_edges`]: each edge gains one track (`addAdjustment(cap + 1, false)`).
pub fn add_resources_for_pin_access(e: &mut RouterEdges, edges: &[(i32, i32, i32, i32, i32)]) {
    for &(x1, y1, x2, y2, layer) in edges {
        let cap = e.get_edge_capacity(x1, y1, x2, y2, layer);
        e.add_adjustment(x1, y1, x2, y2, layer, (cap + 1) as u16, false);
    }
}
