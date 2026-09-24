// SPDX-License-Identifier: Apache-2.0
//! I13 — net discovery's filter, and every pin a net is given: `findNets`' skip rules, then per net
//! `updateNetPins` = `makeItermPins`, `makeBtermPins`, `findPins` (`findOnGridPositions` +
//! `computePinPositionOnGrid`), and `checkPinPlacement` over the ports.
//!
//! The database walk hands over FACTS (a net's flags, an instance terminal's boxes with the
//! transform applied, a block terminal's boxes, the access points, the edge capacities
//! `isPinReachable` reads); everything decided from them is here, in the reference's order.

use std::collections::BTreeMap;

use crate::capacity::Direction;
use crate::Rect;

// ─── findNets ────────────────────────────────────────────────────────────────────────────────

/// What `findNets` reads about one candidate database net.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetCandidate {
    pub name: String,
    pub is_supply: bool,
    pub is_special: bool,
    pub term_count: i32,
    pub has_special_wires: bool,
    pub connected_by_abutment: bool,
}

/// `findNets`' walk over the candidates, in order: the large-fanout skip (GRT-280) and warning
/// (GRT-281), then `addNet`'s routability test. Returns the indices of the nets added.
///
/// ⛔ Two DIFFERENT predicates. The fanout checks exempt only nets that are supply AND special;
/// `addNet` rejects a net that is supply OR special OR has special wires OR is connected by
/// abutment. The warning threshold is a hard-coded 1000, the skip is `skip_large_fanout`.
pub fn find_nets(candidates: &[NetCandidate], skip_large_fanout: i32, log: &mut Vec<String>) -> Vec<usize> {
    const LARGE_FANOUT_THRESHOLD: i32 = 1000;
    let mut added = Vec::new();
    for (i, n) in candidates.iter().enumerate() {
        let is_special = n.is_supply && n.is_special;
        if !is_special && n.term_count > skip_large_fanout {
            log.push(format!("[INFO GRT-0280] Skipping net {} with {} terminals.", n.name, n.term_count));
            continue;
        }
        if !is_special && n.term_count > LARGE_FANOUT_THRESHOLD {
            log.push(format!("[WARNING GRT-0281] Net {} has a large fanout of {} terminals.", n.name, n.term_count));
        }
        if crate::is_routable(n.is_supply, n.is_special, n.has_special_wires, n.connected_by_abutment) {
            added.push(i);
        }
    }
    added
}

// ─── Pins ───────────────────────────────────────────────────────────────────────────────────

/// `PinEdge`, in the reference's declaration order (the capture prints its ordinal).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinEdge {
    North,
    South,
    East,
    West,
    None,
}

/// One box of a terminal: `(pin index, routing level, is a ROUTING layer, rect)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TermBox {
    pub pin: i32,
    pub level: i32,
    pub routing: bool,
    pub rect: Rect,
}

/// A pin as `updateNetPins` leaves it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetPin {
    pub name: String,
    pub is_port: bool,
    /// `position_` — see [`make_iterm_pin`] for which box it is.
    pub position: (i32, i32),
    /// Routing levels, sorted.
    pub layers: Vec<i32>,
    /// Boxes per routing level (every level the terminal has boxes on).
    pub boxes: BTreeMap<i32, Vec<Rect>>,
    pub edge: PinEdge,
    pub connection_layer: i32,
    pub connected_to_pad_or_macro: bool,
    pub is_core: bool,
    pub on_grid: (i32, i32),
}

/// What stops pin construction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PinError {
    /// GRT-10: an instance terminal on an unplaced instance.
    InstanceNotPlaced { pin: String },
    /// GRT-11: a block terminal with an unplaced pin.
    PinNotPlaced { pin: String },
    /// GRT-29: an instance terminal with no geometry at or below the max routing layer.
    NoGeometryBelowMax { pin: String },
    /// GRT-42: a block terminal with no routing-layer geometry, when pin placement is checked.
    NoRoutingGeometry { pin: String },
    /// GRT-209: a block terminal completely outside the die.
    OutsideDie { pin: String },
}

/// The master class of an instance terminal's cell, as `makeItermPins` reads it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MasterClass {
    Pad,
    Block,
    Cover,
    Core,
}

/// `makeItermPins` for ONE instance terminal.
///
/// ⛔ Rules that decide values:
/// - `position` is the lower-left of the LAST routing box of the LAST pin that has one. The loop
///   means "the box on the highest layer" (`if (level > last_layer)`), but `last_layer` is never
///   assigned, so every box passes.
/// - the pin's LAYERS are the levels at or below `max_routing_layer`; its BOXES keep every level;
/// - a pad or macro pin takes its connection layer from [`determine_edge`] against the INSTANCE's
///   box; any other takes its top layer.
#[allow(clippy::too_many_arguments)]
pub fn make_iterm_pin(
    name: &str,
    class: MasterClass,
    is_core: bool,
    placed: bool,
    inst_bbox: Rect,
    boxes: &[TermBox],
    die: Rect,
    max_routing_layer: i32,
    directions: &BTreeMap<i32, Option<Direction>>,
    verbose: bool,
    log: &mut Vec<String>,
) -> Result<NetPin, PinError> {
    if class == MasterClass::Cover && verbose {
        log.push("[WARNING GRT-0034] Net connected to instance of class COVER added for routing.".into());
    }
    let connected = matches!(class, MasterClass::Pad | MasterClass::Block);
    if !placed {
        return Err(PinError::InstanceNotPlaced { pin: name.into() });
    }
    let mut pin_boxes: BTreeMap<i32, Vec<Rect>> = BTreeMap::new();
    let mut pin_pos = (0, 0);
    for b in boxes.iter().filter(|b| b.routing) {
        if !contains(die, b.rect) && verbose {
            log.push(format!("[WARNING GRT-0035] Pin {name} is outside die area."));
        }
        pin_boxes.entry(b.level).or_default().push(b.rect);
        pin_pos = (b.rect.x_min, b.rect.y_min);
    }
    let layers: Vec<i32> = pin_boxes.keys().copied().filter(|&l| l <= max_routing_layer).collect();
    if layers.is_empty() {
        return Err(PinError::NoGeometryBelowMax { pin: name.into() });
    }
    let (edge, connection_layer) = if connected {
        determine_edge(inst_bbox, &pin_boxes, &layers, directions)
    } else {
        (PinEdge::None, *layers.last().expect("non-empty"))
    };
    Ok(NetPin {
        name: name.into(),
        is_port: false,
        position: pin_pos,
        layers,
        boxes: pin_boxes,
        edge,
        connection_layer,
        connected_to_pad_or_macro: connected,
        is_core,
        on_grid: (0, 0),
    })
}

/// `makeBtermPins` for ONE block terminal. `Ok(None)` — skipped without error (no routing geometry,
/// pin placement not checked: the Rudy path).
///
/// ⛔ A box outside the die is CLIPPED to it — but only when VERBOSE (the clip sits inside the
/// warning's `if`). A quiet run keeps the box as it is. Same never-assigned `last_layer` as
/// [`make_iterm_pin`]: the position is the last box's lower-left, after any clip.
pub fn make_bterm_pin(
    name: &str,
    placed: bool,
    boxes: &[TermBox],
    die: Rect,
    directions: &BTreeMap<i32, Option<Direction>>,
    check_pin_placement: bool,
    verbose: bool,
    log: &mut Vec<String>,
) -> Result<Option<NetPin>, PinError> {
    // ⚠️ Checked per pin BEFORE its boxes; the capture folds it to "every pin placed".
    if !placed {
        return Err(PinError::PinNotPlaced { pin: name.into() });
    }
    let mut pin_boxes: BTreeMap<i32, Vec<Rect>> = BTreeMap::new();
    let mut pin_pos = (0, 0);
    for b in boxes.iter().filter(|b| b.routing) {
        let mut rect = b.rect;
        if !contains(die, rect) && verbose {
            log.push(format!("[WARNING GRT-0036] Pin {name} is outside die area."));
            rect = intersection(rect, die);
            if (rect.x_max - rect.x_min) as i64 * (rect.y_max - rect.y_min) as i64 == 0 {
                return Err(PinError::OutsideDie { pin: name.into() });
            }
        }
        pin_boxes.entry(b.level).or_default().push(rect);
        pin_pos = (rect.x_min, rect.y_min);
    }
    let layers: Vec<i32> = pin_boxes.keys().copied().collect();
    if layers.is_empty() {
        return if check_pin_placement { Err(PinError::NoRoutingGeometry { pin: name.into() }) } else { Ok(None) };
    }
    let (edge, connection_layer) = determine_edge(die, &pin_boxes, &layers, directions);
    Ok(Some(NetPin {
        name: name.into(),
        is_port: true,
        position: pin_pos,
        layers,
        boxes: pin_boxes,
        edge,
        connection_layer,
        connected_to_pad_or_macro: false,
        is_core: false,
        on_grid: (0, 0),
    }))
}

/// `odb::Rect::contains`: inclusive on every side.
fn contains(outer: Rect, r: Rect) -> bool {
    outer.x_min <= r.x_min && outer.y_min <= r.y_min && outer.x_max >= r.x_max && outer.y_max >= r.y_max
}

/// `odb::Rect::intersection`: the overlap, or (0, 0, 0, 0) when they do not intersect (touching
/// counts as intersecting).
fn intersection(a: Rect, b: Rect) -> Rect {
    if !(b.x_max >= a.x_min && b.x_min <= a.x_max && b.y_max >= a.y_min && b.y_min <= a.y_max) {
        return Rect::new(0, 0, 0, 0);
    }
    Rect::new(a.x_min.max(b.x_min), a.y_min.max(b.y_min), a.x_max.min(b.x_max), a.y_max.min(b.y_max))
}

/// `Pin::determineEdge`: each box votes for the bounds edge it is nearest (ties: north, south,
/// east, west), the most votes wins (same tie order); the connection layer is the HIGHEST of the
/// pin's layers running across that edge (vertical for north/south), else its top layer.
///
/// ⚠️ Every box votes — including boxes on levels above the max routing layer.
pub fn determine_edge(
    bounds: Rect,
    boxes: &BTreeMap<i32, Vec<Rect>>,
    layers: &[i32],
    directions: &BTreeMap<i32, Option<Direction>>,
) -> (PinEdge, i32) {
    let (mut n, mut s, mut e, mut w) = (0, 0, 0, 0);
    for rects in boxes.values() {
        for b in rects {
            let n_dist = bounds.y_max - b.y_max;
            let s_dist = b.y_min - bounds.y_min;
            let e_dist = bounds.x_max - b.x_max;
            let w_dist = b.x_min - bounds.x_min;
            let min = n_dist.min(s_dist).min(w_dist).min(e_dist);
            if n_dist == min {
                n += 1;
            } else if s_dist == min {
                s += 1;
            } else if e_dist == min {
                e += 1;
            } else {
                w += 1;
            }
        }
    }
    let most = n.max(s).max(e).max(w);
    let edge = if most == n {
        PinEdge::North
    } else if most == s {
        PinEdge::South
    } else if most == e {
        PinEdge::East
    } else {
        PinEdge::West
    };
    let want = if matches!(edge, PinEdge::North | PinEdge::South) { Direction::Vertical } else { Direction::Horizontal };
    let best = layers.iter().copied().filter(|l| directions.get(l).copied().flatten() == Some(want)).max();
    (edge, best.unwrap_or_else(|| *layers.iter().max().expect("non-empty")))
}

// ─── findPins ───────────────────────────────────────────────────────────────────────────────

/// The grid `findPins` measures against, and the tracks per level.
#[derive(Debug, Clone, PartialEq)]
pub struct PinGrid {
    pub die: Rect,
    pub tile_size: i32,
    pub x_grids: i32,
    pub y_grids: i32,
    pub directions: BTreeMap<i32, Option<Direction>>,
    /// Per level: `(track location, track pitch)`.
    pub tracks: BTreeMap<i32, (i32, i32)>,
    pub use_cugr: bool,
}

impl PinGrid {
    /// `Grid::getPositionOnGrid`.
    pub fn position_on_grid(&self, (x, y): (i32, i32)) -> (i32, i32) {
        let mut gx = (x - self.die.x_min) / self.tile_size;
        let mut gy = (y - self.die.y_min) / self.tile_size;
        if gx >= self.x_grids {
            gx -= 1;
        }
        if gy >= self.y_grids {
            gy -= 1;
        }
        (gx * self.tile_size + self.tile_size / 2 + self.die.x_min, gy * self.tile_size + self.tile_size / 2 + self.die.y_min)
    }
}

/// `getRectMiddle`: `xMin + (xMax - xMin) / 2.0` in `double`, truncated back to `int`.
pub fn rect_middle(r: Rect) -> (i32, i32) {
    (
        (r.x_min as f64 + (r.x_max - r.x_min) as f64 / 2.0) as i32,
        (r.y_min as f64 + (r.y_max - r.y_min) as f64 / 2.0) as i32,
    )
}

/// `Pin::getPositionNearInstEdge`: the box middle, moved to the box's side facing the edge.
pub fn position_near_inst_edge(edge: PinEdge, b: Rect, middle: (i32, i32)) -> (i32, i32) {
    match edge {
        PinEdge::North => (middle.0, b.y_max),
        PinEdge::South => (middle.0, b.y_min),
        PinEdge::East => (b.x_max, middle.1),
        PinEdge::West => (b.x_min, middle.1),
        PinEdge::None => middle,
    }
}

/// An access point as `findPinAccessPointPositions` hands it over, in its order: `(level, x, y)`,
/// the position already moved to the instance's location.
pub type AccessPoint = (i32, i32, i32);

/// `findPinAccessPointPositions` for a port: each block pin's access points inserted at the FRONT,
/// so the last pin's lead (CUGR's `findODBAccessPoints` appends instead).
pub fn port_access_points(per_pin: Vec<Vec<AccessPoint>>) -> Vec<AccessPoint> {
    let mut out = Vec::new();
    for these in per_pin {
        out.splice(0..0, these);
    }
    out
}

/// `findOnGridPositions` → `(positions on grid as (x, y, level), has_access_points, pos_on_grid)`.
///
/// With access points: every one, by LEVEL ascending (a `std::map`), each in its order; the last
/// becomes `pos_on_grid`. Without: one position per box on the connection layer — the box middle,
/// or for a pad/macro pin whose box is at least a cell long, the point nearest the instance edge;
/// a pad/macro pin whose middle position is UNREACHABLE also moves to the edge point.
pub fn find_on_grid_positions(
    grid: &PinGrid,
    pin: &NetPin,
    access_points: &[AccessPoint],
    reachable: &mut dyn FnMut(&NetPin, (i32, i32)) -> bool,
) -> (Vec<(i32, i32, i32)>, bool, (i32, i32)) {
    let mut out = Vec::new();
    let mut pos_on_grid = (0, 0);
    if !access_points.is_empty() {
        let mut by_layer: BTreeMap<i32, Vec<(i32, i32)>> = BTreeMap::new();
        for &(l, x, y) in access_points {
            by_layer.entry(l).or_default().push(grid.position_on_grid((x, y)));
        }
        for (l, ps) in by_layer {
            for p in ps {
                pos_on_grid = p;
                out.push((p.0, p.1, l));
            }
        }
        return (out, true, pos_on_grid);
    }
    let conn = pin.connection_layer;
    for &b in &pin.boxes[&conn] {
        let middle = rect_middle(b);
        let len = (b.x_max - b.x_min).max(b.y_max - b.y_min);
        if pin.edge != PinEdge::None && len >= grid.tile_size {
            pos_on_grid = grid.position_on_grid(position_near_inst_edge(pin.edge, b, middle));
        } else {
            pos_on_grid = grid.position_on_grid(middle);
            if !grid.use_cugr && pin.connected_to_pad_or_macro && !reachable(pin, pos_on_grid) {
                pos_on_grid = grid.position_on_grid(position_near_inst_edge(pin.edge, b, middle));
            }
        }
        out.push((pos_on_grid.0, pos_on_grid.1, conn));
    }
    (out, false, pos_on_grid)
}

/// `pinOverlapsWithSingleTrack`: for a pin at most 3 pitches wide across the layer's direction,
/// with exactly one of the two tracks below its top edge STRICTLY inside it, move `track_position`
/// onto that track. All in `float`.
pub fn pin_overlaps_with_single_track(grid: &PinGrid, pin: &NetPin, track_position: &mut (i32, i32)) -> bool {
    let conn = pin.connection_layer;
    let rects = &pin.boxes[&conn];
    let mut r = rects[0];
    for b in &rects[1..] {
        r = Rect::new(r.x_min.min(b.x_min), r.y_min.min(b.y_min), r.x_max.max(b.x_max), r.y_max.max(b.y_max));
    }
    let (loc, pitch) = grid.tracks.get(&conn).copied().unwrap_or((0, 0));
    let horizontal = grid.directions.get(&conn).copied().flatten() == Some(Direction::Horizontal);
    let (min, max) = if horizontal { (r.y_min, r.y_max) } else { (r.x_min, r.x_max) };
    if (max - min) as f32 / pitch as f32 <= 3.0 {
        let nearest = ((max - loc) as f32 / pitch as f32).floor() as i32 * pitch + loc;
        let nearest2 = ((max - loc) as f32 / pitch as f32 - 1.0).floor() as i32 * pitch + loc;
        if (nearest >= min && nearest <= max) && (nearest2 >= min && nearest2 <= max) {
            return false;
        }
        for t in [nearest, nearest2] {
            if t > min && t < max {
                *track_position = if horizontal { (track_position.0, t) } else { (t, track_position.1) };
                return true;
            }
        }
    }
    false
}

/// `computePinPositionOnGrid`: the MOST FREQUENT position (the first to reach a count, in list
/// order, wins ties); then, without access points, a single-track pin moves onto its track's cell
/// when that differs from the vote across the layer's direction.
pub fn compute_pin_position_on_grid(grid: &PinGrid, pin: &mut NetPin, positions: &[(i32, i32, i32)], mut pos_on_grid: (i32, i32), has_access_points: bool) {
    let mut votes = -1;
    let mut chosen = (pin.position.0, pin.position.1, pin.connection_layer);
    for &p in positions {
        let equals = positions.iter().filter(|&&q| q == p).count() as i32;
        if equals > votes {
            chosen = p;
            votes = equals;
        }
    }
    if !has_access_points && pin_overlaps_with_single_track(grid, pin, &mut pos_on_grid) {
        pos_on_grid = grid.position_on_grid(pos_on_grid);
        let dir = grid.directions.get(&pin.connection_layer).copied().flatten();
        if pos_on_grid != (chosen.0, chosen.1)
            && ((dir == Some(Direction::Horizontal) && pos_on_grid.1 != chosen.1)
                || (dir == Some(Direction::Vertical) && pos_on_grid.0 != chosen.0))
        {
            chosen = (pos_on_grid.0, pos_on_grid.1, chosen.2);
        }
    }
    pin.on_grid = (chosen.0, chosen.1);
    pin.connection_layer = chosen.2;
}

/// `findPins` for one pin: [`find_on_grid_positions`] then [`compute_pin_position_on_grid`].
pub fn find_pin(grid: &PinGrid, pin: &mut NetPin, access_points: &[AccessPoint], reachable: &mut dyn FnMut(&NetPin, (i32, i32)) -> bool) {
    let (positions, has_ap, pos_on_grid) = find_on_grid_positions(grid, pin, access_points, reachable);
    compute_pin_position_on_grid(grid, pin, &positions, pos_on_grid, has_ap);
}

/// `isPinReachable`: an east/north pin always is; otherwise the edge INTO its cell on the
/// connection layer must have capacity (none at the grid's first row/column). `edge_capacity`
/// answers `(layer, x1, y1, x2, y2)`.
pub fn is_pin_reachable(grid: &PinGrid, pin: &NetPin, pos_on_grid: (i32, i32), edge_capacity: &dyn Fn(i32, i32, i32, i32, i32) -> i32) -> bool {
    let layer = pin.connection_layer;
    let px = (pos_on_grid.0 - grid.die.x_min) / grid.tile_size;
    let py = (pos_on_grid.1 - grid.die.y_min) / grid.tile_size;
    if matches!(pin.edge, PinEdge::East | PinEdge::North) {
        return true;
    }
    let mut cap = 0;
    if grid.directions.get(&layer).copied().flatten() == Some(Direction::Vertical) {
        if py != 0 {
            cap = edge_capacity(layer, px, py - 1, px, py);
        }
    } else if px != 0 {
        cap = edge_capacity(layer, px - 1, py, px, py);
    }
    cap > 0
}

// ─── checkPinPlacement ──────────────────────────────────────────────────────────────────────

/// GRT-80: two ports share a position on one layer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InvalidPinPlacement;

/// I13b — `checkPinPlacement`: ports (in `db_net_map_` order) on the same connection layer at the
/// same POSITION — `position_`, the raw pin position, not the on-grid one — warn GRT-31 once per
/// earlier port they coincide with, and any such pair fails the run (GRT-80).
///
/// `layer_name` names a routing level; `dbu_per_micron` scales the printed position.
pub fn check_pin_placement(
    ports: &[&NetPin],
    layer_name: &dyn Fn(i32) -> String,
    dbu_per_micron: i32,
    log: &mut Vec<String>,
) -> Result<(), InvalidPinPlacement> {
    let mut invalid = false;
    let mut layer_positions: BTreeMap<i32, Vec<(i32, i32)>> = BTreeMap::new();
    for port in ports {
        let layer = port.connection_layer;
        let seen = layer_positions.entry(layer).or_default();
        for &pos in seen.iter() {
            if pos == port.position {
                let um = |d: i32| d as f64 / dbu_per_micron as f64;
                log.push(format!(
                    "[WARNING GRT-0031] At least 2 pins in position ({:.2}um, {:.2}um), layer {}, port {}.",
                    um(pos.0),
                    um(pos.1),
                    layer_name(layer),
                    port.name
                ));
                invalid = true;
            }
        }
        seen.push(port.position);
    }
    if invalid { Err(InvalidPinPlacement) } else { Ok(()) }
}
