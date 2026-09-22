// SPDX-License-Identifier: Apache-2.0
//! I10 — `applyAdjustments`: obstructions, the blocked-interval reductions, the resource save,
//! the user's global and per-layer adjustments, and the region adjustments.
//!
//! In the reference's order: `computeObstructionsAdjustments` (every obstruction, cell obstruction,
//! pin shape and net wire in range → [`apply_obstruction_adjustment`]; macro transition layers →
//! [`adjust_tile_set`]), [`init_blocked_intervals`], [`save_resources_before_adjustments`],
//! [`compute_user_global_adjustments`], [`compute_user_layer_adjustments`], then each
//! [`compute_region_adjustments`].
//!
//! ⚠️ The PRODUCERS — which rectangles the database walk hands over (instances, pins, wires,
//! macros) — are not here; this module takes that stream as its input.

use std::collections::{BTreeSet, HashMap};

use crate::capacity::Direction;
use crate::Rect;

/// One 3D or 2D edge's capacity state — all `uint16_t` in the reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EdgeState {
    pub cap: u16,
    pub red: u16,
    pub real_cap: u16,
}

/// A blocked-interval key: `(x, y, layer)`.
pub type Tile = (i32, i32, i32);

/// A boost ICL `interval_set<int>` of right-open intervals: sorted, disjoint, and JOINING — two
/// intervals that overlap or merely touch become one. An empty interval adds nothing.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct IntervalSet(pub Vec<(i32, i32)>);

impl IntervalSet {
    /// `set += [lo, hi)`.
    pub fn add(&mut self, lo: i32, hi: i32) {
        if lo >= hi {
            return;
        }
        let (mut lo, mut hi) = (lo, hi);
        let mut out = Vec::with_capacity(self.0.len() + 1);
        for &(a, b) in &self.0 {
            if b < lo || a > hi {
                out.push((a, b));
            } else {
                lo = lo.min(a);
                hi = hi.max(b);
            }
        }
        out.push((lo, hi));
        out.sort_unstable();
        self.0 = out;
    }

    /// `blockedTrackCount`: the covered length over the track pitch, rounded UP (0 for no pitch).
    pub fn blocked_track_count(&self, track_space: i32) -> i32 {
        if track_space <= 0 {
            return 0;
        }
        let blocked_length: i64 = self.0.iter().map(|&(a, b)| (b - a).abs() as i64).sum();
        (blocked_length as f64 / track_space as f64).ceil() as i32
    }
}

/// The router's edges while I10 adjusts them, and the grid it measures against.
#[derive(Debug, Clone, PartialEq)]
pub struct RouterEdges {
    pub die: Rect,
    pub tile_size: i32,
    pub x_grid: i32,
    pub y_grid: i32,
    pub num_layers: i32,
    /// `grid_->getTrackPitches()`, per layer `level - 1`.
    pub track_pitches: Vec<i32>,
    /// `h_edges_3D_[l][y][x]`, flattened `(l * y_grid + y) * x_grid + x`.
    pub h3: Vec<EdgeState>,
    pub v3: Vec<EdgeState>,
    /// The 2D edges, `[y][x]`: horizontal `x < x_grid - 1`, vertical `y < y_grid - 1`.
    pub h2: Vec<EdgeState>,
    pub v2: Vec<EdgeState>,
    pub horizontal_blocked: HashMap<Tile, IntervalSet>,
    pub vertical_blocked: HashMap<Tile, IntervalSet>,
    pub verbose: bool,
    /// GRT-113 / GRT-114, in order.
    pub log: Vec<String>,
}

impl RouterEdges {
    fn i3(&self, l: i32, x: i32, y: i32) -> usize {
        ((l * self.y_grid + y) * self.x_grid + x) as usize
    }

    /// `getEdgeCapacity(x1, y1, x2, y2, layer)`: the 3D edge named by its first cell.
    pub fn get_edge_capacity(&self, x1: i32, y1: i32, x2: i32, y2: i32, layer: i32) -> i32 {
        let i = self.i3(layer - 1, x1, y1);
        if y1 == y2 {
            self.h3[i].cap as i32
        } else if x1 == x2 {
            self.v3[i].cap as i32
        } else {
            panic!("GRT-214: edge is not vertical or horizontal")
        }
    }

    /// `Graph2D::addCapH/V` (`cap += delta`, `uint16_t`) and `addRedH/V` (`red = max(red + d, 0)`).
    fn add_2d(e: &mut EdgeState, cap_delta: i32, red_delta: i32) {
        e.cap = (e.cap as i32 + cap_delta) as u16;
        e.red = (e.red as i32 + red_delta).max(0) as u16;
    }

    /// `addAdjustment(x1, y1, x2, y2, layer, reducedCap, isReduce)`: set the 3D edge's capacity to
    /// `reduced_cap` and carry the difference into its reduction and the 2D edge.
    ///
    /// ⛔ `reducedCap` is a `uint16_t` PARAMETER: a negative request arrives wrapped (−3 → 65533).
    /// ⛔ The 2D edge is touched only for a REAL edge (`x1 < x_grid - 1` / `y1 < y_grid - 1`); the
    /// 3D edge is written regardless.
    #[allow(clippy::too_many_arguments)]
    pub fn add_adjustment(&mut self, x1: i32, y1: i32, x2: i32, y2: i32, layer: i32, reduced_cap: u16, is_reduce: bool) {
        let k = layer - 1;
        let horizontal = y1 == y2;
        if !horizontal && x1 != x2 {
            return;
        }
        let i = self.i3(k, x1, y1);
        let cap = if horizontal { self.h3[i].cap } else { self.v3[i].cap } as i32;
        let reduced = reduced_cap as i32;
        let reduce = if cap - reduced < 0 {
            if is_reduce && self.verbose {
                let id = if horizontal { 113 } else { 114 };
                self.log.push(format!("[WARNING GRT-0{id}] Underflow in reduce: cap, reducedCap: {cap}, {reduced}"));
            }
            0
        } else {
            cap - reduced
        };
        let real_edge = if horizontal { x1 < self.x_grid - 1 } else { y1 < self.y_grid - 1 };
        let i2 = if horizontal { (y1 * (self.x_grid - 1) + x1) as usize } else { (y1 * self.x_grid + x1) as usize };
        {
            let e = if horizontal { &mut self.h3[i] } else { &mut self.v3[i] };
            e.cap = reduced_cap;
        }
        if !is_reduce {
            let increase = reduced - cap;
            if real_edge {
                let e2 = if horizontal { &mut self.h2[i2] } else { &mut self.v2[i2] };
                Self::add_2d(e2, increase, -increase);
            }
            let e = if horizontal { &mut self.h3[i] } else { &mut self.v3[i] };
            e.red = (e.red as i32 - increase).max(0) as u16;
        } else {
            let e = if horizontal { &mut self.h3[i] } else { &mut self.v3[i] };
            e.red = (e.red as i32 + reduce) as u16;
        }
        if real_edge {
            let e2 = if horizontal { &mut self.h2[i2] } else { &mut self.v2[i2] };
            Self::add_2d(e2, -reduce, reduce);
        }
    }

    /// `Grid::getPositionOnGrid`: the centre of the cell holding the point, the last cell clamped.
    pub fn get_position_on_grid(&self, x: i32, y: i32) -> (i32, i32) {
        let mut gx = (x - self.die.x_min) / self.tile_size;
        let mut gy = (y - self.die.y_min) / self.tile_size;
        if gx >= self.x_grid {
            gx -= 1;
        }
        if gy >= self.y_grid {
            gy -= 1;
        }
        (gx * self.tile_size + self.tile_size / 2 + self.die.x_min, gy * self.tile_size + self.tile_size / 2 + self.die.y_min)
    }

    /// `Grid::getBlockedTiles` → `(first_tile_bds, last_tile_bds, first_tile, last_tile)`.
    ///
    /// The last tile's box reaches the die's edge when less than a cell remains
    /// (`(die_max - ur) / tile < 1`, integer division).
    pub fn get_blocked_tiles(&self, obs: Rect) -> (Rect, Rect, (i32, i32), (i32, i32)) {
        let t = self.tile_size;
        let lower = self.get_position_on_grid(obs.x_min, obs.y_min);
        let upper = self.get_position_on_grid(obs.x_max, obs.y_max);
        let first = ((lower.0 - self.die.x_min) / t, (lower.1 - self.die.y_min) / t);
        let last = ((upper.0 - self.die.x_min) / t, (upper.1 - self.die.y_min) / t);
        let first_bds = Rect::new(lower.0 - t / 2, lower.1 - t / 2, lower.0 + t / 2, lower.1 + t / 2);
        let (mut urx, mut ury) = (upper.0 + t / 2, upper.1 + t / 2);
        if (self.die.x_max - urx) / t < 1 {
            urx = self.die.x_max;
        }
        if (self.die.y_max - ury) / t < 1 {
            ury = self.die.y_max;
        }
        let last_bds = Rect::new(upper.0 - t / 2, upper.1 - t / 2, urx, ury);
        (first_bds, last_bds, first, last)
    }
}

/// `Grid::computeTileReduceInterval`: the part of an edge tile's span the obstruction covers,
/// across the layer's routing direction.
///
/// ⛔ For a MACRO, when blocking leaves exactly one track (`layer_cap - ceil((float) len / pitch)
/// == 1`, in `float`), the interval is widened by one more pitch.
#[allow(clippy::too_many_arguments)]
pub fn compute_tile_reduce_interval(
    obs: Rect,
    tile: Rect,
    track_space: i32,
    first: bool,
    direction: Option<Direction>,
    layer_cap: i32,
    is_macro: bool,
) -> (i32, i32) {
    let (lo, hi, tlo, thi) = if direction == Some(Direction::Vertical) {
        (obs.x_min, obs.x_max, tile.x_min, tile.x_max)
    } else {
        (obs.y_min, obs.y_max, tile.y_min, tile.y_max)
    };
    let (start_point, mut end_point) = if lo >= tlo && hi <= thi {
        (lo, hi)
    } else if first {
        (lo, thi)
    } else {
        (tlo, hi)
    };
    let interval_length = (end_point - start_point).abs();
    if is_macro {
        let blocked_tracks = (interval_length as f32 / track_space as f32).ceil() as i32;
        if layer_cap - blocked_tracks == 1 {
            end_point += track_space;
        }
    }
    (start_point, end_point)
}

/// `applyObstructionAdjustment(obstruction, layer, is_macro, release = false)`.
///
/// Edge tiles collect blocked INTERVALS (reduced later, in [`init_blocked_intervals`]); tiles
/// strictly between them are blocked outright (capacity 0). An obstruction inside one cell along
/// the routing direction blocks the edge to the NEXT cell, when there is one.
///
/// ⛔ An obstruction that does not strictly OVERLAP the die (outside it, or only touching an edge)
/// is not skipped: its rectangle stays the DEFAULT `Rect` — (0, 0, 0, 0) — and blocks the cells at
/// the origin (`inst_pin_out_of_die`: 6 shapes).
pub fn apply_obstruction_adjustment(
    e: &mut RouterEdges,
    obstruction: Rect,
    layer: i32,
    direction: Option<Direction>,
    is_macro: bool,
) {
    let die = e.die;
    let overlaps = obstruction.x_max > die.x_min
        && obstruction.x_min < die.x_max
        && obstruction.y_max > die.y_min
        && obstruction.y_min < die.y_max;
    let obstruction_rect = if overlaps {
        let r = Rect::new(
            die.x_min.max(obstruction.x_min),
            die.y_min.max(obstruction.y_min),
            die.x_max.min(obstruction.x_max),
            die.y_max.min(obstruction.y_max),
        );
        if r.x_min > r.x_max || r.y_min > r.y_max {
            return;
        }
        r
    } else {
        Rect::new(0, 0, 0, 0)
    };
    let (first_box, last_box, first, mut last) = e.get_blocked_tiles(obstruction_rect);
    let vertical = direction == Some(Direction::Vertical);
    let track_space = e.track_pitches[(layer - 1) as usize];
    let (first_cap, last_cap) = if vertical {
        (
            e.get_edge_capacity(first.0, first.1, first.0, first.1 + 1, layer),
            e.get_edge_capacity(last.0, last.1, last.0, last.1 + 1, layer),
        )
    } else {
        (
            e.get_edge_capacity(first.0, first.1, first.0 + 1, first.1, layer),
            e.get_edge_capacity(last.0, last.1, last.0 + 1, last.1, layer),
        )
    };
    let fi = compute_tile_reduce_interval(obstruction_rect, first_box, track_space, true, direction, first_cap, is_macro);
    let li = compute_tile_reduce_interval(obstruction_rect, last_box, track_space, false, direction, last_cap, is_macro);
    let grid_limit = if vertical { e.y_grid } else { e.x_grid };
    if !vertical {
        if first.0 == last.0 && last.0 + 1 < grid_limit {
            last.0 += 1;
        }
        add_horizontal_adjustments(e, first, last, layer, fi, li);
    } else {
        if first.1 == last.1 && last.1 + 1 < grid_limit {
            last.1 += 1;
        }
        add_vertical_adjustments(e, first, last, layer, fi, li);
    }
}

/// `addHorizontalAdjustments` (no release): `x` in `first..last`, `y` in `first..=last`.
fn add_horizontal_adjustments(e: &mut RouterEdges, first: (i32, i32), last: (i32, i32), layer: i32, fi: (i32, i32), li: (i32, i32)) {
    for x in first.0..last.0 {
        for y in first.1..=last.1 {
            if y == first.1 {
                e.horizontal_blocked.entry((x, y, layer)).or_default().add(fi.0, fi.1);
            } else if y == last.1 {
                e.horizontal_blocked.entry((x, y, layer)).or_default().add(li.0, li.1);
            } else {
                e.add_adjustment(x, y, x + 1, y, layer, 0, true);
            }
        }
    }
}

/// `addVerticalAdjustments` (no release): `x` in `first..=last`, `y` in `first..last`.
fn add_vertical_adjustments(e: &mut RouterEdges, first: (i32, i32), last: (i32, i32), layer: i32, fi: (i32, i32), li: (i32, i32)) {
    for x in first.0..=last.0 {
        for y in first.1..last.1 {
            if x == first.0 {
                e.vertical_blocked.entry((x, y, layer)).or_default().add(fi.0, fi.1);
            } else if x == last.0 {
                e.vertical_blocked.entry((x, y, layer)).or_default().add(li.0, li.1);
            } else {
                e.add_adjustment(x, y, x, y + 1, layer, 0, true);
            }
        }
    }
}

/// `adjustTileSet`: halve each tile's edge on a transition layer — `floor(float cap * 0.5)`, at
/// least 1 where there was capacity.
pub fn adjust_tile_set(e: &mut RouterEdges, tiles: &BTreeSet<(i32, i32)>, layer: i32, direction: Option<Direction>) {
    for &(x, y) in tiles {
        let (ex, ey) = if direction == Some(Direction::Horizontal) { (x + 1, y) } else { (x, y + 1) };
        let edge_cap = e.get_edge_capacity(x, y, ex, ey, layer) as f32;
        let mut new_cap = (edge_cap as f64 * 0.5).floor() as i32;
        if edge_cap > 0.0 {
            new_cap = new_cap.max(1);
        }
        e.add_adjustment(x, y, ex, ey, layer, new_cap as u16, true);
    }
}

/// `initBlockedIntervals`: each blocked tile's edge loses the tracks its intervals cover
/// (clamped at 0) — only edges that still have capacity. Vertical tiles, then horizontal; the
/// tiles are independent, so the maps' order does not matter.
pub fn init_blocked_intervals(e: &mut RouterEdges) {
    for vertical in [true, false] {
        let map = if vertical { &e.vertical_blocked } else { &e.horizontal_blocked };
        let mut work: Vec<(Tile, i32)> = Vec::new();
        for (&(x, y, layer), ivs) in map {
            let reduce = if layer > 0 && layer as usize <= e.track_pitches.len() {
                ivs.blocked_track_count(e.track_pitches[(layer - 1) as usize])
            } else {
                0
            };
            work.push(((x, y, layer), reduce));
        }
        for ((x, y, layer), reduce) in work {
            let (x2, y2) = if vertical { (x, y + 1) } else { (x + 1, y) };
            let edge_cap = e.get_edge_capacity(x, y, x2, y2, layer);
            if edge_cap > 0 {
                let cap = (edge_cap - reduce).max(0);
                e.add_adjustment(x, y, x2, y2, layer, cap as u16, true);
            }
        }
    }
}

/// `saveResourcesBeforeAdjustments`: every REAL edge's capacity becomes its real capacity, 2D and
/// per layer. (The 3D edges past the grid keep whatever they had.)
pub fn save_resources_before_adjustments(e: &mut RouterEdges) {
    let (xg, yg) = (e.x_grid, e.y_grid);
    for x in 0..xg - 1 {
        for y in 0..yg {
            let i2 = (y * (xg - 1) + x) as usize;
            e.h2[i2].real_cap = e.h2[i2].cap;
            for l in 0..e.num_layers {
                let i = e.i3(l, x, y);
                e.h3[i].real_cap = e.h3[i].cap;
            }
        }
    }
    for x in 0..xg {
        for y in 0..yg - 1 {
            let i2 = (y * xg + x) as usize;
            e.v2[i2].real_cap = e.v2[i2].cap;
            for l in 0..e.num_layers {
                let i = e.i3(l, x, y);
                e.v3[i].real_cap = e.v3[i].cap;
            }
        }
    }
}

/// `computeUserGlobalAdjustments`: a layer in range with NO adjustment of its own takes the global
/// one.
///
/// ⛔ This WRITES THE TECHNOLOGY (`setLayerAdjustment`): the layer keeps it for every later run in
/// the session. `layer_adjustment` is indexed by routing level and owned by the caller.
pub fn compute_user_global_adjustments(layer_adjustment: &mut [f32], adjustment: f32, min_routing_layer: i32, max_routing_layer: i32) {
    if adjustment == 0.0 {
        return;
    }
    for l in min_routing_layer..=max_routing_layer {
        if layer_adjustment[l as usize] == 0.0 {
            layer_adjustment[l as usize] = adjustment;
        }
    }
}

/// `computeUserLayerAdjustments`: scale every edge of each adjusted layer in range.
///
/// ⛔ `floor((float) cap * (1 - adjustment))` in `float`; at least 1 where there was capacity,
/// unless the adjustment is exactly 1; a NEGATIVE adjustment is an INCREASE (`is_reduce` false).
pub fn compute_user_layer_adjustments(
    e: &mut RouterEdges,
    layer_adjustment: &[f32],
    directions: &[Option<Direction>],
    min_routing_layer: i32,
    max_routing_layer: i32,
) {
    let (xg, yg) = (e.x_grid, e.y_grid);
    for layer in 1..=max_routing_layer {
        let inside_layer_range = layer >= min_routing_layer && layer <= max_routing_layer;
        let adjustment = layer_adjustment[layer as usize];
        let is_reduce = adjustment > 0.0;
        if adjustment == 0.0 || !inside_layer_range {
            continue;
        }
        let scaled = |cap: i32| {
            let n = (cap as f32 * (1.0 - adjustment)).floor() as i32;
            if cap > 0 && adjustment != 1.0 { n.max(1) } else { n }
        };
        match directions[layer as usize] {
            Some(Direction::Horizontal) => {
                for y in 1..=yg {
                    for x in 1..xg {
                        let cap = e.get_edge_capacity(x - 1, y - 1, x, y - 1, layer);
                        e.add_adjustment(x - 1, y - 1, x, y - 1, layer, scaled(cap) as u16, is_reduce);
                    }
                }
            }
            Some(Direction::Vertical) => {
                for x in 1..=xg {
                    for y in 1..yg {
                        let cap = e.get_edge_capacity(x - 1, y - 1, x - 1, y, layer);
                        e.add_adjustment(x - 1, y - 1, x - 1, y, layer, scaled(cap) as u16, is_reduce);
                    }
                }
            }
            None => {}
        }
    }
}

/// A region adjustment rejected by the reference.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegionOutsideDie;

/// `Grid::computeTileReduce`: how many tracks an edge tile loses — `ceil(|span| / track_space)`
/// with `track_space` a `double`.
pub fn compute_tile_reduce(obs: Rect, tile: Rect, track_space: f64, first: bool, direction: Option<Direction>) -> i32 {
    let (lo, hi, tlo, thi) = if direction == Some(Direction::Vertical) {
        (obs.x_min, obs.x_max, tile.x_min, tile.x_max)
    } else {
        (obs.y_min, obs.y_max, tile.y_min, tile.y_max)
    };
    let span = if lo >= tlo && hi <= thi {
        hi - lo
    } else if first {
        thi - lo
    } else {
        hi - tlo
    };
    (span.abs() as f64 / track_space).ceil() as i32
}

/// `computeRegionAdjustments(region, layer, reduction)`: scale every edge the region covers on one
/// layer; its first and last tiles lose `reduce * (1 - reduction)` tracks instead.
///
/// ⛔ Rules that decide values:
/// - GRT-72 only when the region sticks out on BOTH axes at one corner;
/// - the track space is `getUsePitch()` — the line-to-via pitches count;
/// - the loops run to the last tile INCLUSIVE and write the edge LEAVING it, so a region reaching
///   the top row writes the vertical edge past the grid;
/// - an edge tile's new capacity is `cap - reduce * (1 - r)` in floating point, truncated — it can
///   go NEGATIVE where the capacity is 0, is not clamped there (the `max(…, 1)` needs capacity), and
///   arrives in `addAdjustment` WRAPPED to ~65 533 (upstream finding 13).
pub fn compute_region_adjustments(
    e: &mut RouterEdges,
    region: Rect,
    layer: i32,
    reduction_percentage: f32,
    direction: Option<Direction>,
    use_pitch: i32,
) -> Result<(), RegionOutsideDie> {
    let die = e.die;
    if (die.x_min > region.x_min && die.y_min > region.y_min) || (die.x_max < region.x_max && die.y_max < region.y_max) {
        return Err(RegionOutsideDie);
    }
    let vertical = direction == Some(Direction::Vertical);
    let (first_box, last_box, first, last) = e.get_blocked_tiles(region);
    let track_space = use_pitch as f64;
    let first_tile_reduce = compute_tile_reduce(region, first_box, track_space, true, direction);
    let last_tile_reduce = compute_tile_reduce(region, last_box, track_space, false, direction);
    let keep = 1.0f32 - reduction_percentage;
    for x in first.0..=last.0 {
        for y in first.1..=last.1 {
            let edge_cap = if vertical {
                e.get_edge_capacity(x, y, x, y + 1, layer)
            } else {
                e.get_edge_capacity(x, y, x + 1, y, layer)
            } as f64;
            let mut new_cap = (edge_cap * keep as f64).floor() as i32;
            if x == first.0 || y == first.1 {
                new_cap = (edge_cap - (first_tile_reduce as f32 * keep) as f64) as i32;
            } else if x == last.0 || y == last.1 {
                new_cap = (edge_cap - (last_tile_reduce as f32 * keep) as f64) as i32;
            }
            if edge_cap > 0.0 && reduction_percentage != 1.0 {
                new_cap = new_cap.max(1);
            }
            if vertical {
                e.add_adjustment(x, y, x, y + 1, layer, new_cap as u16, true);
            } else {
                e.add_adjustment(x, y, x + 1, y, layer, new_cap as u16, true);
            }
        }
    }
    Ok(())
}
