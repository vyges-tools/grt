// SPDX-License-Identifier: Apache-2.0
//! I6 — `initRoutingTracks`: each routing layer's average track spacing, and the line-to-via
//! pitches `calcLayerPitches` derives from the default vias and the layer's spacing rules.
//!
//! In the reference's order: `getDefaultVias` (odb), then per layer `getViaDims` and the spacing
//! rules (`calcLayerPitches`), then per layer `getAverageTrackSpacing` (odb) → `RoutingTracks`.
//!
//! 🔑 **What the pitches are for.** The line-to-via pitch is printed (GRT-88) and — through
//! `getUsePitch`, the max of the track pitch and both line-to-via pitches — read by the REGION
//! adjustments (I10). Nothing else.
//!
//! The spacing-TABLE lookups (`findTwSpacing`, `findV55Spacing`) are odb's; they come in through
//! [`SpacingLookup`] with exactly the arguments this code passes.

use std::collections::HashMap;

use crate::capacity::Direction;

/// One `TRACKS` pattern: `origin`, `count` lines, `step` apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrackPattern {
    pub origin: i32,
    pub count: i32,
    pub step: i32,
}

/// A layer's track grid: its X patterns (vertical tracks) and Y patterns (horizontal tracks).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct TrackGrid {
    pub x: Vec<TrackPattern>,
    pub y: Vec<TrackPattern>,
}

/// What stops the track setup.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TrackError {
    /// GRT-86: a routing layer inside the range has no track grid.
    NoTrackGrid { layer: String },
    /// ODB-414: a horizontal layer with no Y pattern.
    NoHorizontalTracks { layer: String },
    /// ODB-415: a vertical layer with no X pattern.
    NoVerticalTracks { layer: String },
    /// ODB-416: a layer with neither direction.
    InvalidDirection { layer: String },
}

/// `dbTrackGrid::getAverageTrackSpacing` → `(track_step, track_init, num_tracks)`.
///
/// One pattern on the layer's axis: that pattern, as is. Several: `getAverageTrackPattern` over
/// the MERGED coordinates (every pattern expanded, sorted, deduplicated) —
/// `init = front`, `num = count`, and
///
/// ⛔ `step = ceil((float) (back - front) / count)` — the span over the number of TRACKS, not of
/// gaps, and in `float`. So a merged grid of 3 tracks at 0, 10, 20 averages to a step of 7.
pub fn get_average_track_spacing(
    layer: &str,
    direction: Option<Direction>,
    grid: &TrackGrid,
) -> Result<(i32, i32, i32), TrackError> {
    let (patterns, missing) = match direction {
        Some(Direction::Horizontal) => (&grid.y, TrackError::NoHorizontalTracks { layer: layer.into() }),
        Some(Direction::Vertical) => (&grid.x, TrackError::NoVerticalTracks { layer: layer.into() }),
        None => return Err(TrackError::InvalidDirection { layer: layer.into() }),
    };
    match patterns.len() {
        0 => Err(missing),
        1 => Ok((patterns[0].step, patterns[0].origin, patterns[0].count)),
        _ => Ok(get_average_track_pattern(patterns)),
    }
}

/// `_dbTrackGrid::getAverageTrackPattern` over `getGridX` / `getGridY` (expand, `sort_and_unique`).
fn get_average_track_pattern(patterns: &[TrackPattern]) -> (i32, i32, i32) {
    let mut coordinates = Vec::new();
    for p in patterns {
        let mut c = p.origin;
        for _ in 0..p.count {
            coordinates.push(c);
            c += p.step;
        }
    }
    coordinates.sort_unstable();
    coordinates.dedup();
    let span = coordinates[coordinates.len() - 1] - coordinates[0];
    let track_init = coordinates[0];
    let track_step = (span as f32 / coordinates.len() as f32).ceil() as i32;
    (track_step, track_init, coordinates.len() as i32)
}

/// A technology via, as `getDefaultVias` and `getViaDims` read it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TechVia {
    pub name: String,
    /// Routing level of the bottom layer: `0` for a non-routing layer, `None` for no layer.
    pub bottom: Option<i32>,
    /// Whether the via carries the `OR_DEFAULT` property.
    pub or_default: bool,
    /// Boxes on routing layers, in order: `(routing level, dx, dy)`.
    pub boxes: Vec<(i32, i32, i32)>,
}

/// `dbBlock::getDefaultVias`: bottom layer → the index of its default via.
///
/// ⛔ Two OPPOSITE rules. With any `OR_DEFAULT` via, those are the defaults and a LATER one on the
/// same bottom layer REPLACES an earlier one (map assignment). With none, every via whose bottom
/// is a routing layer is a candidate and the FIRST per bottom layer is kept.
pub fn get_default_vias(vias: &[TechVia]) -> HashMap<Option<i32>, usize> {
    let mut default_vias = HashMap::new();
    for (i, via) in vias.iter().enumerate() {
        if via.or_default {
            default_vias.insert(via.bottom, i);
        }
    }
    if default_vias.is_empty() {
        for (i, via) in vias.iter().enumerate() {
            if matches!(via.bottom, Some(l) if l != 0) {
                default_vias.entry(via.bottom).or_insert(i);
            }
        }
    }
    default_vias
}

/// `getViaDims` → `(width_up, prl_up, width_down, prl_down)`, `-1` where there is no via.
///
/// Both vias are measured by their FIRST box on THIS layer: `width = min(dx, dy)`,
/// `prl = max(dx, dy)`. The up via is the one whose bottom is this layer; the down via is the one
/// whose bottom is the routing layer below (`findRoutingLayer(level - 1)` — none at level 1).
pub fn get_via_dims(
    vias: &[TechVia],
    default_vias: &HashMap<Option<i32>, usize>,
    routing_level: i32,
) -> (i32, i32, i32, i32) {
    let dims = |via: usize| {
        vias[via]
            .boxes
            .iter()
            .find(|b| b.0 == routing_level)
            .map(|b| (b.1.min(b.2), b.1.max(b.2)))
            .unwrap_or((-1, -1))
    };
    let below = (routing_level > 1).then_some(routing_level - 1);
    let (width_up, prl_up) = default_vias.get(&Some(routing_level)).map_or((-1, -1), |&v| dims(v));
    let (width_down, prl_down) = default_vias.get(&below).map_or((-1, -1), |&v| dims(v));
    (width_up, prl_up, width_down, prl_down)
}

/// A LEF `SPACING` rule (V5.4 style), with its optional `RANGE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct V54Rule {
    pub spacing: u32,
    pub range: Option<(u32, u32)>,
}

/// A routing layer as `calcLayerPitches` reads it, under its `routing_layers_` index.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PitchLayer {
    /// The `routing_layers_` key — the running counter I4 assigns.
    pub index: i32,
    pub name: String,
    /// `getType() == ROUTING`.
    pub is_routing: bool,
    /// `getRoutingLevel()` — what the via lookups key on.
    pub routing_level: i32,
    pub width: i32,
    pub has_two_widths: bool,
    pub has_v55: bool,
    pub v54: Vec<V54Rule>,
}

/// odb's spacing-table lookups on one layer.
pub trait SpacingLookup {
    /// `findTwSpacing(width1, width2, prl)`.
    fn tw(&self, layer: &PitchLayer, width1: i32, width2: i32, prl: i32) -> i32;
    /// `findV55Spacing(width, prl)`.
    fn v55(&self, layer: &PitchLayer, width: i32, prl: i32) -> i32;
}

/// `calcLayerPitches(max_layer)` → `(L2V_up, L2V_down)` per `routing_layers_` index.
///
/// ⛔ Rules that decide values:
/// - a layer with NO default via either way is skipped and keeps the vector's default `(0, 0)`;
///   one with vias but no spacing rule gets `(-1, -1)`;
/// - the spacing rule is chosen by PRIORITY: two-widths table, else V5.5 table (at
///   `max(layer_width, via_width)`), else the largest V5.4 `SPACING` whose `RANGE` holds the
///   layer width;
/// - `L2V = width / 2 + via_width / 2 + spacing` (each half truncated), but `-1` upward at the
///   BLOCK's max routing layer and downward at its min — `getMaxRoutingLayer()`, not the
///   `max_layer` argument, compared against the INDEX.
pub fn calc_layer_pitches(
    layers: &[PitchLayer],
    max_layer: i32,
    block_min_routing_layer: i32,
    block_max_routing_layer: i32,
    routing_layer_count: i32,
    vias: &[TechVia],
    spacing: &dyn SpacingLookup,
) -> Vec<(i32, i32)> {
    let default_vias = get_default_vias(vias);
    let mut pitches = vec![(0, 0); routing_layer_count as usize + 1];
    for layer in layers {
        if !layer.is_routing {
            continue;
        }
        if layer.index > max_layer && max_layer > -1 {
            break;
        }
        let (width_up, prl_up, width_down, prl_down) = get_via_dims(vias, &default_vias, layer.routing_level);
        let up_via_valid = width_up != -1;
        let down_via_valid = width_down != -1;
        if !up_via_valid && !down_via_valid {
            continue;
        }
        let layer_width = layer.width;
        let (mut l2v_up, mut l2v_down) = (-1, -1);
        let mut min_spc_valid = false;
        let (mut min_spc_up, mut min_spc_down) = (-1, -1);
        if layer.has_two_widths {
            min_spc_valid = true;
            if up_via_valid {
                min_spc_up = spacing.tw(layer, layer_width, width_up, prl_up);
            }
            if down_via_valid {
                min_spc_down = spacing.tw(layer, layer_width, width_down, prl_down);
            }
        } else if layer.has_v55 {
            min_spc_valid = true;
            if up_via_valid {
                min_spc_up = spacing.v55(layer, layer_width.max(width_up), prl_up);
            }
            if down_via_valid {
                min_spc_down = spacing.v55(layer, layer_width.max(width_down), prl_down);
            }
        } else if !layer.v54.is_empty() {
            min_spc_valid = true;
            let mut min_spc = 0i32;
            for rule in &layer.v54 {
                if let Some((rmin, rmax)) = rule.range {
                    // `int < uint32_t` compares as unsigned.
                    if (layer_width as u32) < rmin || (layer_width as u32) > rmax {
                        continue;
                    }
                }
                min_spc = min_spc.max(rule.spacing as i32);
            }
            if up_via_valid {
                min_spc_up = min_spc;
            }
            if down_via_valid {
                min_spc_down = min_spc;
            }
        }
        if min_spc_valid {
            if up_via_valid {
                l2v_up = if layer.index != block_max_routing_layer {
                    layer_width / 2 + width_up / 2 + min_spc_up
                } else {
                    -1
                };
            }
            if down_via_valid {
                l2v_down = if layer.index != block_min_routing_layer {
                    layer_width / 2 + width_down / 2 + min_spc_down
                } else {
                    -1
                };
            }
        }
        pitches[layer.index as usize] = (l2v_up, l2v_down);
    }
    pitches
}

/// `RoutingTracks` — one layer's track spacing and line-to-via pitches.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RoutingTracks {
    pub layer_index: i32,
    pub track_pitch: i32,
    pub line_2_via_pitch_up: i32,
    pub line_2_via_pitch_down: i32,
    pub location: i32,
    pub num_tracks: i32,
}

impl RoutingTracks {
    /// `getLineToViaPitch`: the larger of the two.
    pub fn line_to_via_pitch(&self) -> i32 {
        self.line_2_via_pitch_up.max(self.line_2_via_pitch_down)
    }

    /// `getUsePitch`: the largest of the track pitch and both line-to-via pitches.
    pub fn use_pitch(&self) -> i32 {
        self.track_pitch.max(self.line_2_via_pitch_up).max(self.line_2_via_pitch_down)
    }
}

/// One routing layer's track facts for I6.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TrackLayer {
    pub index: i32,
    pub name: String,
    pub direction: Option<Direction>,
    /// `block_->findTrackGrid(layer)`.
    pub grid: Option<TrackGrid>,
}

/// I6 — `initRoutingTracks(max_routing_layer)`: per layer in index order up to the max, its
/// average track spacing and the pitches from [`calc_layer_pitches`]; GRT-88 when verbose.
///
/// ⚠️ The reference APPENDS to `routing_tracks_`, which only `clear()` empties, and looks a layer
/// up by the FIRST entry with its index — the caller owns that list across calls.
pub fn init_routing_tracks(
    layers: &[TrackLayer],
    max_routing_layer: i32,
    pitches: &[(i32, i32)],
    dbu_per_micron: i32,
    verbose: bool,
    log: &mut Vec<String>,
) -> Result<Vec<RoutingTracks>, TrackError> {
    let mut out = Vec::new();
    for layer in layers {
        if layer.index > max_routing_layer && max_routing_layer > -1 {
            break;
        }
        let grid = layer.grid.as_ref().ok_or_else(|| TrackError::NoTrackGrid { layer: layer.name.clone() })?;
        let (track_step, track_init, num_tracks) = get_average_track_spacing(&layer.name, layer.direction, grid)?;
        let (up, down) = pitches[layer.index as usize];
        let t = RoutingTracks {
            layer_index: layer.index,
            track_pitch: track_step,
            line_2_via_pitch_up: up,
            line_2_via_pitch_down: down,
            location: track_init,
            num_tracks,
        };
        out.push(t);
        if verbose {
            let um = |dbu: i32| dbu as f64 / dbu_per_micron as f64;
            log.push(format!(
                "[INFO GRT-0088] Layer {:<7} Track-Pitch = {:.4}  line-2-Via Pitch: {:.4}",
                layer.name,
                um(t.track_pitch),
                um(t.line_to_via_pitch())
            ));
        }
    }
    Ok(out)
}
