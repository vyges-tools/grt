// SPDX-License-Identifier: Apache-2.0
//! Routing layers, and how much track capacity each grid cell has.
//!
//! Stage I4 (validate and index the routing layers) and the core of I9 (per-edge capacity).
//! I6 — track pitches — is [`crate::tracks`].

use crate::init::CoreGrid;
use crate::Rect;

/// A layer's preferred routing direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Horizontal,
    Vertical,
}

/// A routing layer, as the setup stage reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RoutingLayer {
    pub name: String,
    /// The technology's own routing level. ⚠️ **Not** the index this layer is stored under.
    pub routing_level: i32,
    pub direction: Option<Direction>,
    pub has_track_grid: bool,
    /// Backside layers are exempt from the adjacent-direction check.
    pub is_backside: bool,
}

/// What rejected a technology's layer stack.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LayerError {
    /// A layer with no preferred direction.
    NoDirection { layer: String },
    /// A layer inside the routing range with no track grid.
    NoTrackGrid { layer: String },
    /// Two adjacent layers preferring the same direction.
    SameDirection { a: String, b: String, direction: Direction },
}

impl std::fmt::Display for LayerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LayerError::NoDirection { layer } => {
                write!(f, "layer {layer} does not have a valid direction")
            }
            LayerError::NoTrackGrid { layer } => write!(f, "layer {layer} does not have a track grid"),
            LayerError::SameDirection { a, b, direction } => write!(
                f,
                "layers {a} and {b} have the same preferred routing direction ({direction:?})"
            ),
        }
    }
}

impl std::error::Error for LayerError {}

/// Index the routing layers, validating each.
///
/// ⛔ **The index is a running COUNTER, not the routing level.** Layers whose routing level is 0
/// are skipped entirely and do not consume an index, so on a technology where the two diverge
/// every later lookup by index shifts — including the two in the guide writer. Transcribe the
/// counter; do not "simplify" it to the level.
///
/// ⚠️ **The track-grid requirement applies only INSIDE the routing range.** A layer outside
/// `min..=max` may have no track grid and that is not an error.
///
/// Returns the layers keyed by their index, starting at 1.
pub fn init_routing_layers(
    layers: &[RoutingLayer],
    min_routing_layer: i32,
    max_routing_layer: i32,
) -> Result<Vec<(i32, RoutingLayer)>, LayerError> {
    let mut out = Vec::new();
    let mut valid_layers = 1;
    for layer in layers {
        if layer.routing_level == 0 {
            continue;
        }
        if layer.direction.is_none() {
            return Err(LayerError::NoDirection { layer: layer.name.clone() });
        }
        if !layer.has_track_grid
            && layer.routing_level >= min_routing_layer
            && layer.routing_level <= max_routing_layer
        {
            return Err(LayerError::NoTrackGrid { layer: layer.name.clone() });
        }
        out.push((valid_layers, layer.clone()));
        valid_layers += 1;
    }
    Ok(out)
}

/// Adjacent routing layers must prefer different directions.
///
/// ⚠️ **The range is half-open**: it compares `l` with `l + 1` for `l` in `min..max`, so the pair
/// `(max, max + 1)` is never examined. Widening it to an inclusive range reports errors upstream
/// does not.
///
/// ⚠️ **A backside layer next to a frontside one is skipped**, not checked — they route in
/// separate stacks, so sharing a direction is legal there.
///
/// `by_level` looks a layer up by its routing LEVEL, which is what this check uses (unlike the
/// index in [`init_routing_layers`]).
pub fn check_adjacent_layers_direction(
    by_level: &dyn Fn(i32) -> Option<RoutingLayer>,
    min_routing_layer: i32,
    max_routing_layer: i32,
) -> Result<(), LayerError> {
    for l in min_routing_layer..max_routing_layer {
        let (a, b) = match (by_level(l), by_level(l + 1)) {
            (Some(a), Some(b)) => (a, b),
            _ => continue,
        };
        if a.is_backside != b.is_backside {
            continue;
        }
        if a.direction == b.direction {
            if let Some(direction) = a.direction {
                return Err(LayerError::SameDirection { a: a.name, b: b.name, direction });
            }
        }
    }
    Ok(())
}

impl CoreGrid {
    /// The centre of grid cell `(x, y)`, by index.
    ///
    /// ⚠️ The reference computes this as `tile_size * (x + 0.5) + origin` in **floating point**,
    /// truncating on assignment. The integer form below is equal for every positive tile size —
    /// `floor(t*(x + 0.5)) == t*x + floor(t/2)` because `t*x` is exact — and avoids the precision
    /// question entirely. Stated rather than assumed, because "it is the same" is the kind of
    /// claim that turns out to be false at the edges.
    pub fn position_from_grid_point(&self, x: i32, y: i32) -> (i32, i32) {
        (
            x * self.tile_size + self.tile_size / 2 + self.area.x_min,
            y * self.tile_size + self.tile_size / 2 + self.area.y_min,
        )
    }

    /// The rectangle grid cell `(x, y)` covers.
    ///
    /// ⚠️ **Only the upper corner is snapped to the die edge**, by the same truncating-divide test
    /// the guide writer uses. So every cell is exactly one tile wide except those at the top and
    /// right, which absorb the remainder.
    pub fn gcell_rect(&self, x: i32, y: i32) -> Rect {
        let (cx, cy) = self.position_from_grid_point(x, y);
        let half = self.tile_size / 2;
        let (x_min, y_min) = (cx - half, cy - half);
        let mut x_max = cx + half;
        let mut y_max = cy + half;
        if (self.area.x_max - x_max) / self.tile_size < 1 {
            x_max = self.area.x_max;
        }
        if (self.area.y_max - y_max) / self.tile_size < 1 {
            y_max = self.area.y_max;
        }
        Rect::new(x_min, y_min, x_max, y_max)
    }
}

/// How many routing tracks fall inside one grid cell.
///
/// This is the capacity of a cell's edge on one layer: the number of tracks crossing it.
///
/// ⛔ **The two bounds are NOT symmetric, and that asymmetry is the rule.**
///
/// - the first track is a **ceiling**, written as `(a + pitch - 1) / pitch`, guarded by
///   `min_bound <= track_init` so the numerator is never negative (where that idiom breaks);
/// - the last track is a **floor over `max_bound - track_init - 1`** — note the extra `- 1`,
///   which makes the upper bound **exclusive**. A track sitting exactly on `max_bound` belongs to
///   the next cell, not this one. Dropping that `- 1` over-counts by one on every cell boundary a
///   track lands on.
///
/// Both are then clamped into `0..=track_count - 1`, and a cell whose range inverts has no
/// capacity at all rather than a negative one.
pub fn compute_gcell_capacity(
    grid: &CoreGrid,
    x: i32,
    y: i32,
    track_init: i32,
    track_pitch: i32,
    track_count: i32,
    horizontal: bool,
) -> i32 {
    let r = grid.gcell_rect(x, y);
    let (min_bound, max_bound) = if horizontal {
        (r.y_min, r.y_max)
    } else {
        (r.x_min, r.x_max)
    };

    let mut first_track = if min_bound <= track_init {
        0
    } else {
        (min_bound - track_init + track_pitch - 1) / track_pitch // ceil
    };
    let mut last_track = if max_bound < track_init {
        -1
    } else {
        (max_bound - track_init - 1) / track_pitch // floor, upper bound EXCLUSIVE
    };

    first_track = first_track.max(0);
    last_track = last_track.min(track_count - 1);

    if first_track > last_track {
        return 0;
    }
    last_track - first_track + 1
}

/// The order `setCapacities` walks a layer's cells, and the edge each step writes.
///
/// ⛔ **The loops are TRANSPOSED between the two directions, and the inner bound is half-open
/// while the outer is inclusive.** A horizontal layer walks `y` in `1..=y_grids` outside and `x`
/// in `1..x_grids` inside, writing the edge from `(x-1, y-1)` to `(x, y-1)`; a vertical layer
/// walks `x` outside and `y` inside, writing `(x-1, y-1)` to `(x-1, y)`. Writing one loop and
/// swapping the arguments produces a different set of edges.
///
/// Returns `(from, to)` cell pairs in the order they are written.
pub fn capacity_edge_order(x_grids: i32, y_grids: i32, horizontal: bool) -> Vec<((i32, i32), (i32, i32))> {
    let mut out = Vec::new();
    if horizontal {
        for y in 1..=y_grids {
            for x in 1..x_grids {
                out.push(((x - 1, y - 1), (x, y - 1)));
            }
        }
    } else {
        for x in 1..=x_grids {
            for y in 1..y_grids {
                out.push(((x - 1, y - 1), (x - 1, y)));
            }
        }
    }
    out
}

/// The capacity used when the run is told to ignore congestion entirely.
///
/// ⚠️ **`i16::MAX / 10`, not `i32::MAX`.** The router stores capacity in 16 bits, so the
/// "infinite" value is deliberately a tenth of that range to leave headroom for the additions
/// downstream. A wider constant would overflow where the reference does not.
pub const INFINITE_CAPACITY: i32 = (i16::MAX as i32) / 10;

/// I8 — what `mirrorGridToFastRoute` leaves in the router.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FastRouteGrid {
    pub x_grid: i32,
    pub y_grid: i32,
    pub num_layers: i32,
    /// `x_range_ = y_range_`: the larger grid dimension, but never below 1000 (`setGridsAndLayers`).
    pub x_range: i32,
    pub y_range: i32,
    pub regular_x: bool,
    pub regular_y: bool,
    /// The grid's lower-left corner — the DIE's.
    pub x_corner: i32,
    pub y_corner: i32,
    pub tile_size: i32,
    /// `getGridArea().xMax()/yMax()` — the die's upper-right.
    pub x_grid_max: i32,
    pub y_grid_max: i32,
    /// Per router layer `l - 1`, the direction of routing level `l`.
    pub directions: Vec<Option<Direction>>,
}

/// I8 — `mirrorGridToFastRoute(max_routing_layer)`: copy the grid into the router's own fields,
/// and each routing level's direction into router layer `level - 1`.
pub fn mirror_grid_to_fast_route(grid: &CoreGrid, directions: &[Option<Direction>]) -> FastRouteGrid {
    let range = grid.x_grids.max(grid.y_grids).max(1000);
    let mut dirs = vec![None; grid.num_layers as usize];
    for (l, d) in directions.iter().enumerate().take(grid.num_layers as usize) {
        dirs[l] = *d;
    }
    FastRouteGrid {
        x_grid: grid.x_grids,
        y_grid: grid.y_grids,
        num_layers: grid.num_layers,
        x_range: range,
        y_range: range,
        regular_x: grid.perfect_regular_x,
        regular_y: grid.perfect_regular_y,
        x_corner: grid.area.x_min,
        y_corner: grid.area.y_min,
        tile_size: grid.tile_size,
        x_grid_max: grid.area.x_max,
        y_grid_max: grid.area.y_max,
        directions: dirs,
    }
}

/// One routing level as `setCapacities` reads it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapacityLayer {
    pub direction: Option<Direction>,
    /// `getRoutingTracksByIndex(level)` — `None` when no entry carries the index.
    pub tracks: Option<crate::tracks::RoutingTracks>,
}

/// The router's capacities after `setCapacities`.
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeCapacities {
    pub x_grid: i32,
    pub y_grid: i32,
    pub num_layers: i32,
    /// `h_edges_3D_[l][y][x].cap`, flattened `(l * y_grid + y) * x_grid + x` — `uint16_t`.
    pub h3: Vec<u16>,
    /// `v_edges_3D_[l][y][x].cap`, flattened the same way.
    pub v3: Vec<u16>,
    /// The 2D horizontal caps, `[y][x]` with `x < x_grid - 1` — `uint16_t`, summed over layers.
    pub h2: Vec<u16>,
    /// The 2D vertical caps, `[y][x]` with `y < y_grid - 1`.
    pub v2: Vec<u16>,
    /// Per router layer, the smallest edge capacity — `int16_t` (`addHCapacity(int16_t, int)`).
    pub h_capacity_3d: Vec<i16>,
    pub v_capacity_3d: Vec<i16>,
    /// Their sums, `int`.
    pub h_capacity: i32,
    pub v_capacity: i32,
    /// `0.9f * capacity`, `float`.
    pub h_capacity_lb: f32,
    pub v_capacity_lb: f32,
}

impl EdgeCapacities {
    fn at3(&self, l: i32, x: i32, y: i32) -> usize {
        ((l * self.y_grid + y) * self.x_grid + x) as usize
    }
}

/// I9 — `setCapacities(min, max)`: every edge's track capacity per layer, the 2D sums, each
/// layer's minimum, and the lower bounds.
///
/// ⛔ Rules that decide values:
/// - the walk is [`capacity_edge_order`] — transposed between the directions, half-open inside;
/// - a layer whose direction is not HORIZONTAL takes the VERTICAL branch;
/// - outside `min..=max` a layer's edges get 0; with `infinite_capacity` every edge gets
///   [`INFINITE_CAPACITY`], inside the range or not;
/// - the 3D edge is ASSIGNED, the 2D edge ADDED to (both `uint16_t`);
/// - a layer's minimum starts at `INT_MAX` and becomes 0 if the layer wrote no edge (a one-cell-wide
///   grid), then narrows to `int16_t`;
/// - the lower bounds are `0.9f * sum` in `float`.
pub fn set_capacities(
    fr: &FastRouteGrid,
    core: &CoreGrid,
    layers: &[CapacityLayer],
    min_routing_layer: i32,
    max_routing_layer: i32,
    infinite_capacity: bool,
) -> EdgeCapacities {
    let (xg, yg, nl) = (fr.x_grid, fr.y_grid, fr.num_layers);
    let cells = (nl * yg * xg) as usize;
    let mut c = EdgeCapacities {
        x_grid: xg,
        y_grid: yg,
        num_layers: nl,
        h3: vec![0; cells],
        v3: vec![0; cells],
        h2: vec![0; ((xg - 1).max(0) * yg) as usize],
        v2: vec![0; (xg * (yg - 1).max(0)) as usize],
        h_capacity_3d: vec![0; nl as usize],
        v_capacity_3d: vec![0; nl as usize],
        h_capacity: 0,
        v_capacity: 0,
        h_capacity_lb: 0.0,
        v_capacity_lb: 0.0,
    };
    for layer in 1..=core.num_layers {
        let info = layers[(layer - 1) as usize];
        let inside_layer_range = layer >= min_routing_layer && layer <= max_routing_layer;
        let tracks = info.tracks.unwrap_or_default();
        let horizontal = info.direction == Some(Direction::Horizontal);
        let mut min_cap = i32::MAX;
        for ((x1, y1), _) in capacity_edge_order(xg, yg, horizontal) {
            let cap = if infinite_capacity {
                INFINITE_CAPACITY
            } else if inside_layer_range {
                compute_gcell_capacity(core, x1, y1, tracks.location, tracks.track_pitch, tracks.num_tracks, horizontal)
            } else {
                0
            };
            min_cap = min_cap.min(cap);
            // setEdgeCapacity(x1, y1, x2, y2, layer, cap): the edge is named by its first cell.
            let k = layer - 1;
            let i3 = c.at3(k, x1, y1);
            if horizontal {
                let i2 = (y1 * (xg - 1) + x1) as usize;
                c.h2[i2] = c.h2[i2].wrapping_add(cap as u16);
                c.h3[i3] = cap as u16;
            } else {
                let i2 = (y1 * xg + x1) as usize;
                c.v2[i2] = c.v2[i2].wrapping_add(cap as u16);
                c.v3[i3] = cap as u16;
            }
        }
        let min_cap = if min_cap == i32::MAX { 0 } else { min_cap } as i16;
        if horizontal {
            c.h_capacity_3d[(layer - 1) as usize] = min_cap;
            c.h_capacity += min_cap as i32;
        } else {
            c.v_capacity_3d[(layer - 1) as usize] = min_cap;
            c.v_capacity += min_cap as i32;
        }
    }
    // initLowerBoundCapacities: `const float LB = 0.9; lb = LB * capacity`.
    c.v_capacity_lb = 0.9f32 * c.v_capacity as f32;
    c.h_capacity_lb = 0.9f32 * c.h_capacity as f32;
    c
}

/// I12 — `initEdgesCapacityPerLayer`: the per-layer capacity the 2D phases monitor, copied from the
/// 3D edges into `Cap3D { cap, cap_ndr }` — indexed `[layer][x][y]`, TRANSPOSED from the 3D edges'
/// `[layer][y][x]`. Only real edges are copied (`x < x_grid - 1` horizontal, `y < y_grid - 1`
/// vertical); the rest stay zero.
///
/// Returns `(horizontal, vertical)`, each flattened `(l * x_grid + x) * y_grid + y`, as
/// `(cap, cap_ndr)`.
pub fn init_edges_capacity_per_layer(c: &EdgeCapacities) -> (Vec<(u16, f64)>, Vec<(u16, f64)>) {
    let (xg, yg, nl) = (c.x_grid, c.y_grid, c.num_layers);
    let n = (nl * xg * yg) as usize;
    let (mut h, mut v) = (vec![(0u16, 0.0f64); n], vec![(0u16, 0.0f64); n]);
    for y in 0..yg {
        for x in 0..xg {
            for l in 0..nl {
                let i = ((l * xg + x) * yg + y) as usize;
                if x < xg - 1 {
                    // updateCap3D(..., const double cap): cap = cap_ndr = the edge's cap.
                    let cap = c.h3[c.at3(l, x, y)];
                    h[i] = (cap, cap as f64);
                }
                if y < yg - 1 {
                    let cap = c.v3[c.at3(l, x, y)];
                    v[i] = (cap, cap as f64);
                }
            }
        }
    }
    (h, v)
}
