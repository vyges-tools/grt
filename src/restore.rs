// SPDX-License-Identifier: Apache-2.0
//! Routes rebuilt from the database's guides — `GlobalRouter::loadGuidesFromDB`'s per-guide and
//! per-net steps, as `repair_antennas` runs them when no `global_route` ran in the session.
//!
//! Stages in the reference's order: [`box_to_global_routing`] per guide, then per net
//! [`dedup_via_segments`], [`add_implicit_vias`] and `mergeSegments`
//! ([`crate::findrouting::merge_segments`]); [`net_is_covered`] decides whether
//! `ensurePinsPositions` has anything to do.

use crate::GSegment;

/// `boxToGlobalRouting`: one guide box back into tile-centre segments.
///
/// ⛔ The box is snapped with **absolute** coordinates — `tile * (x / tile) + tile / 2` — not
/// relative to the grid's origin. The low corner rounds DOWN to its tile and takes the centre; the
/// high corner rounds down and steps BACK half a tile. A box that collapses to one point is a via
/// from `layer` to `via_layer`; otherwise it is walked one tile at a time, horizontally first, and
/// a box that is neither a row nor a column of tiles yields nothing.
pub fn box_to_global_routing(bx: (i32, i32, i32, i32), layer: i32, via_layer: i32, tile_size: i32, route: &mut Vec<GSegment>) {
    let (x_min, y_min, x_max, y_max) = bx;
    let mut x0 = tile_size * (x_min / tile_size) + tile_size / 2;
    let mut y0 = tile_size * (y_min / tile_size) + tile_size / 2;
    let x1 = tile_size * (x_max / tile_size) - tile_size / 2;
    let y1 = tile_size * (y_max / tile_size) - tile_size / 2;
    if x0 == x1 && y0 == y1 {
        route.push(GSegment::new(x0, y0, layer, x1, y1, via_layer));
    }
    while y0 == y1 && x0 + tile_size <= x1 {
        route.push(GSegment::new(x0, y0, layer, x0 + tile_size, y0, layer));
        x0 += tile_size;
    }
    while x0 == x1 && y0 + tile_size <= y1 {
        route.push(GSegment::new(x0, y0, layer, x0, y0 + tile_size, layer));
        y0 += tile_size;
    }
}

/// `dedupViaSegments`: `saveGuides` writes a guide on EACH layer of a via, so reading them back
/// yields the via twice. Keep the first via per `(x, y, lower layer, upper layer)`, in order; a
/// "via" on one layer, and every wire, is kept whatever it repeats.
pub fn dedup_via_segments(route: &mut Vec<GSegment>) {
    let mut seen = std::collections::BTreeSet::new();
    route.retain(|s| {
        if !s.is_via() || s.init_layer == s.final_layer {
            return true;
        }
        let (lo, hi) = (s.init_layer.min(s.final_layer), s.init_layer.max(s.final_layer));
        seen.insert((s.init_x, s.init_y, lo, hi))
    });
}

/// `addImplicitVias`: at every point where a net has segments on ADJACENT routing levels with no
/// via between them, append that via.
///
/// ⛔ Appended at the END, in `std::map` point order (x, then y) and ascending layer, after the
/// scan — never interleaved. A gap of more than one level is left alone, and a level bridged by
/// any explicit via at the point (however many levels that via spans) needs no bridge.
///
/// ⚠️ **Unreached in the corpus**: on `repair_antennas_from_odb` and the roundtrip reload no bridge
/// is ever added (a probe asserting none passes both) — every transition is an explicit via guide.
/// Pinned by the unit test below only.
pub fn add_implicit_vias(route: &mut Vec<GSegment>) {
    use std::collections::{BTreeMap, BTreeSet};
    if route.is_empty() {
        return;
    }
    let mut layers_at: BTreeMap<(i32, i32), BTreeSet<i32>> = BTreeMap::new();
    let mut existing: BTreeSet<(i32, i32, i32)> = BTreeSet::new();
    for s in route.iter() {
        layers_at.entry((s.init_x, s.init_y)).or_default().insert(s.init_layer);
        layers_at.entry((s.final_x, s.final_y)).or_default().insert(s.final_layer);
        if s.is_via() && s.init_layer != s.final_layer {
            let (lo, hi) = (s.init_layer.min(s.final_layer), s.init_layer.max(s.final_layer));
            for l in lo..hi {
                existing.insert((s.init_x, s.init_y, l));
            }
        }
    }
    let mut bridges = Vec::new();
    for (&(x, y), layers) in &layers_at {
        if layers.len() < 2 {
            continue;
        }
        let mut prev = -1;
        for &l in layers {
            if prev != -1 && l == prev + 1 && !existing.contains(&(x, y, prev)) {
                bridges.push(GSegment::new(x, y, prev, x, y, l));
            }
            prev = l;
        }
    }
    route.extend(bridges);
}

/// `segmentCoversPin`: the pin's on-grid position inside the segment's box (ends inclusive) and its
/// connection layer inside the segment's layer span.
pub fn segment_covers_pin(s: &GSegment, pin: &crate::Pin) -> bool {
    let (min_l, max_l) = (s.init_layer.min(s.final_layer), s.init_layer.max(s.final_layer));
    pin.on_grid_x >= s.init_x.min(s.final_x)
        && pin.on_grid_x <= s.init_x.max(s.final_x)
        && pin.on_grid_y >= s.init_y.min(s.final_y)
        && pin.on_grid_y <= s.init_y.max(s.final_y)
        && pin.connection_layer >= min_l
        && pin.connection_layer <= max_l
}

/// `netIsCovered`: the indices of the pins no segment covers (empty = covered).
pub fn net_is_covered(route: &[GSegment], pins: &[crate::Pin]) -> Vec<usize> {
    (0..pins.len()).filter(|&k| !route.iter().any(|s| segment_covers_pin(s, &pins[k]))).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tile-aligned box one tile high and three long (as `saveGuides` writes it) becomes two
    /// steps between tile centres: low corner up half a tile, high corner back half a tile.
    #[test]
    fn a_row_of_tiles_walks_horizontally_between_centres() {
        let mut r = Vec::new();
        box_to_global_routing((100, 0, 400, 100), 2, 2, 100, &mut r);
        assert_eq!(r, vec![GSegment::new(150, 50, 2, 250, 50, 2), GSegment::new(250, 50, 2, 350, 50, 2)]);
    }

    /// ⛔ The snap is ABSOLUTE and the high corner steps BACK: a box whose top edge is inside the
    /// first tile row has `y1 = -50 != y0 = 50`, so it is neither a row nor a column — nothing.
    #[test]
    fn a_box_not_spanning_whole_tiles_yields_nothing() {
        let mut r = Vec::new();
        box_to_global_routing((120, 20, 380, 90), 2, 2, 100, &mut r);
        assert!(r.is_empty());
    }

    /// A one-tile box is a via to the guide's via layer.
    #[test]
    fn a_single_tile_box_is_a_via() {
        let mut r = Vec::new();
        box_to_global_routing((100, 100, 200, 200), 1, 2, 100, &mut r);
        assert_eq!(r, vec![GSegment::new(150, 150, 1, 150, 150, 2)]);
    }

    /// The second copy of a via (written once per layer) is dropped; a wire repeated is kept.
    #[test]
    fn a_via_read_back_twice_is_kept_once() {
        let v = GSegment::new(50, 50, 1, 50, 50, 2);
        let w = GSegment::new(50, 50, 1, 150, 50, 1);
        let mut r = vec![v, w, GSegment::new(50, 50, 2, 50, 50, 1), w];
        dedup_via_segments(&mut r);
        assert_eq!(r, vec![v, w, w]);
    }

    /// Adjacent levels at a point with no via get one appended; a two-level gap does not.
    #[test]
    fn adjacent_levels_without_a_via_are_bridged_at_the_end() {
        let mut r = vec![GSegment::new(50, 50, 1, 150, 50, 1), GSegment::new(50, 50, 2, 50, 150, 2), GSegment::new(150, 50, 1, 150, 50, 1), GSegment::new(150, 50, 3, 250, 50, 3)];
        add_implicit_vias(&mut r);
        assert_eq!(r.len(), 5);
        assert_eq!(r[4], GSegment::new(50, 50, 1, 50, 50, 2));
    }

    /// A pin is covered by a segment's closed box on a layer inside its span.
    #[test]
    fn coverage_is_inclusive_in_position_and_layer() {
        let s = GSegment::new(50, 50, 1, 150, 50, 1);
        let at = |x, y, l| crate::Pin { connection_layer: l, on_grid_x: x, on_grid_y: y };
        assert!(segment_covers_pin(&s, &at(150, 50, 1)));
        assert!(!segment_covers_pin(&s, &at(150, 50, 2)));
        assert_eq!(net_is_covered(&[s], &[at(50, 50, 1), at(250, 50, 1)]), vec![1]);
    }
}
