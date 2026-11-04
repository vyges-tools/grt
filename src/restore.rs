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

/// `loadGuidesFromDB`'s tail for one net's route, the same under both engines:
/// `dedupViaSegments`, `addImplicitVias`, then `mergeSegments` against the net's pins.
pub fn finish_loaded_route(route: &mut Vec<GSegment>, pins: &[crate::findrouting::GridPin], block_min_routing_layer: i32) {
    dedup_via_segments(route);
    add_implicit_vias(route);
    crate::findrouting::merge_segments(pins, route, block_min_routing_layer);
}

/// A grid-cell node of a route: gcell x, gcell y, layer.
pub type PruneNode = (i32, i32, i32);

/// The geometry of `pruneMergedNetRoute`: the pruned route, or `None` when nothing is pruned (no
/// edge went, the route is empty, or a segment end or a pin — `(on-grid x, y, connection layer)` —
/// is off a gcell centre). The grid's lower-left corner and tile size give the gcell centres.
pub fn pruned_merged_route(route: &[crate::GSegment], pins: &[PruneNode], x_min: i32, y_min: i32, tile: i32) -> Option<Vec<crate::GSegment>> {
    use std::collections::{BTreeMap, BTreeSet};
    if route.is_empty() {
        return None;
    }
    let x_origin = x_min + tile / 2;
    let y_origin = y_min + tile / 2;
    // the gcell index of a gcell centre, or -1 for any other position
    let to_gcell = |coord: i32, origin: i32| -> i32 {
        let offset = coord - origin;
        if offset >= 0 && offset % tile == 0 { offset / tile } else { -1 }
    };
    // `std::map<Edge, bool>`: edge -> is jumper (emplace keeps the first)
    let mut edges: BTreeMap<(PruneNode, PruneNode), bool> = BTreeMap::new();
    let mut total_edges = 0usize;
    let mut add_edge = |mut p: PruneNode, mut q: PruneNode, jumper: bool| {
        if q < p {
            std::mem::swap(&mut p, &mut q);
        }
        edges.entry((p, q)).or_insert(jumper);
        total_edges += 1;
    };
    for seg in route {
        let x0 = to_gcell(seg.init_x.min(seg.final_x), x_origin);
        let y0 = to_gcell(seg.init_y.min(seg.final_y), y_origin);
        let x1 = to_gcell(seg.init_x.max(seg.final_x), x_origin);
        let y1 = to_gcell(seg.init_y.max(seg.final_y), y_origin);
        if x0 < 0 || y0 < 0 || x1 < 0 || y1 < 0 {
            return None;
        }
        if seg.is_via() {
            let (bottom, top) = (seg.init_layer.min(seg.final_layer), seg.init_layer.max(seg.final_layer));
            for layer in bottom..top {
                add_edge((x0, y0, layer), (x0, y0, layer + 1), seg.is_jumper);
            }
        } else {
            let layer = seg.init_layer;
            for x in x0..x1 {
                add_edge((x, y0, layer), (x + 1, y0, layer), seg.is_jumper);
            }
            for y in y0..y1 {
                add_edge((x0, y, layer), (x0, y + 1, layer), seg.is_jumper);
            }
        }
    }
    let mut pin_gcells: BTreeSet<(i32, i32)> = BTreeSet::new();
    let mut pin_nodes: BTreeSet<PruneNode> = BTreeSet::new();
    for &(px, py, layer) in pins {
        let x = to_gcell(px, x_origin);
        let y = to_gcell(py, y_origin);
        if x < 0 || y < 0 {
            return None;
        }
        pin_gcells.insert((x, y));
        pin_nodes.insert((x, y, layer));
    }
    let is_prunable = |node: &PruneNode, neighbors: &BTreeSet<PruneNode>| neighbors.len() == 1 && !pin_gcells.contains(&(node.0, node.1));
    // trim the dangling branches from their open ends
    let mut adjacency: BTreeMap<PruneNode, BTreeSet<PruneNode>> = BTreeMap::new();
    for &(p, q) in edges.keys() {
        adjacency.entry(p).or_default().insert(q);
        adjacency.entry(q).or_default().insert(p);
    }
    let mut open_ends: Vec<PruneNode> = adjacency.iter().filter(|(node, nb)| is_prunable(node, nb)).map(|(node, _)| *node).collect();
    while let Some(node) = open_ends.pop() {
        let neighbors = adjacency.entry(node).or_default();
        if !is_prunable(&node, neighbors) {
            continue;
        }
        let next = *neighbors.iter().next().expect("a prunable node has one neighbour");
        neighbors.clear();
        edges.remove(&if node < next { (node, next) } else { (next, node) });
        let next_neighbors = adjacency.entry(next).or_default();
        next_neighbors.remove(&node);
        if is_prunable(&next, next_neighbors) {
            open_ends.push(next);
        }
    }
    if edges.len() == total_edges {
        return None;
    }
    // rebuild the route: straight segments ending at every branch, via and pin
    let to_dbu = |gcell: i32, origin: i32| origin + gcell * tile;
    let mut pruned: Vec<crate::GSegment> = Vec::new();
    let mut add_segment = |x0: i32, y0: i32, l0: i32, x1: i32, y1: i32, l1: i32, is_jumper: bool| {
        pruned.push(crate::GSegment { init_x: to_dbu(x0, x_origin), init_y: to_dbu(y0, y_origin), init_layer: l0, final_x: to_dbu(x1, x_origin), final_y: to_dbu(y1, y_origin), final_layer: l1, is_jumper });
    };
    // (layer, row or column, is jumper) -> the start of each unit edge
    let mut horizontal: BTreeMap<(i32, i32, bool), Vec<i32>> = BTreeMap::new();
    let mut vertical: BTreeMap<(i32, i32, bool), Vec<i32>> = BTreeMap::new();
    for (&((x0, y0, l0), (x1, y1, l1)), &is_jumper) in &edges {
        if l0 != l1 {
            add_segment(x0, y0, l0, x1, y1, l1, is_jumper);
        } else if y0 == y1 {
            horizontal.entry((l0, y0, is_jumper)).or_default().push(x0);
        } else {
            vertical.entry((l0, x0, is_jumper)).or_default().push(y0);
        }
    }
    for (runs, is_horizontal) in [(&horizontal, true), (&vertical, false)] {
        for (&(layer, line, is_jumper), starts) in runs {
            let mut first = 0usize;
            for i in 1..=starts.len() {
                if i < starts.len() && starts[i] == starts[i - 1] + 1 {
                    let joint = if is_horizontal { (starts[i], line, layer) } else { (line, starts[i], layer) };
                    // `adjacency[joint]`: an absent node has no neighbours
                    let degree = adjacency.get(&joint).map_or(0, BTreeSet::len);
                    if degree == 2 && !pin_nodes.contains(&joint) {
                        continue;
                    }
                }
                let begin = starts[first];
                let end = starts[i - 1] + 1;
                if is_horizontal {
                    add_segment(begin, line, layer, end, line, layer, is_jumper);
                } else {
                    add_segment(line, begin, layer, line, end, layer, is_jumper);
                }
                first = i;
            }
        }
    }
    Some(pruned)
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

    /// Upstream rule (`loadGuidesFromDB`, both engines): every loaded route gets its implicit vias
    /// before the merge. No corpus guide needs one (every transition is an explicit via guide), so
    /// this is the only witness that the shared tail calls it.
    #[test]
    fn a_loaded_route_is_bridged_before_the_merge() {
        let mut r = vec![GSegment::new(50, 50, 1, 150, 50, 1), GSegment::new(50, 50, 2, 50, 150, 2)];
        finish_loaded_route(&mut r, &[], 1);
        assert!(r.contains(&GSegment::new(50, 50, 1, 50, 50, 2)), "{r:?}");
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

#[cfg(test)]
mod prune_merged_route_tests {
    use super::pruned_merged_route;
    use crate::GSegment;

    fn seg(x0: i32, y0: i32, l0: i32, x1: i32, y1: i32, l1: i32) -> GSegment {
        GSegment { init_x: x0, init_y: y0, init_layer: l0, final_x: x1, final_y: y1, final_layer: l1, is_jumper: false }
    }

    /// Rule (`GlobalRouter::pruneMergedNetRoute`, upstream 442e664144c): the branch to a removed
    /// buffer is trimmed back from its open end to the trunk, and the trunk, whose joint now has
    /// two neighbours and no pin, is joined into one segment. Tile 10 from 0: gcell centres 5, 15,
    /// 25. Pins at gcells (0,0) and (2,0) on layer 1; the dead branch climbs from (1,0) to (1,2)
    /// and up a via at its end.
    #[test]
    fn a_branch_to_a_removed_buffer_is_trimmed_and_the_trunk_joined() {
        let route = vec![seg(5, 5, 1, 15, 5, 1), seg(15, 5, 1, 25, 5, 1), seg(15, 5, 1, 15, 25, 1), seg(15, 25, 1, 15, 25, 2)];
        let pins = [(5, 5, 1), (25, 5, 1)];
        let got = pruned_merged_route(&route, &pins, 0, 0, 10).expect("the branch is pruned");
        assert_eq!(got, vec![seg(5, 5, 1, 25, 5, 1)]);
    }

    /// Rule: a branch ending in a pin's gcell is kept (on any layer: the pin access guide), a run is
    /// broken at a pin node, and nothing pruned gives nothing to install.
    #[test]
    fn a_branch_to_a_pin_gcell_is_kept_and_runs_break_at_pins() {
        let route = vec![seg(5, 5, 1, 25, 5, 1), seg(15, 5, 1, 15, 15, 1)];
        // the branch ends in pin gcell (1,1); a pin sits on the trunk at (1,0)
        let pins = [(5, 5, 1), (25, 5, 1), (15, 15, 2), (15, 5, 1)];
        assert_eq!(pruned_merged_route(&route, &pins, 0, 0, 10), None, "nothing to prune");
        // duplicated edges collapse, so an edge count drop alone forces a rebuild
        let dup = vec![seg(5, 5, 1, 25, 5, 1), seg(5, 5, 1, 15, 5, 1)];
        let got = pruned_merged_route(&dup, &[(5, 5, 1), (25, 5, 1), (15, 5, 1)], 0, 0, 10).expect("the duplicate goes");
        assert_eq!(got, vec![seg(5, 5, 1, 15, 5, 1), seg(15, 5, 1, 25, 5, 1)], "broken at the pin node (1,0,1)");
    }

    /// Rule (`std::map::emplace`): a unit edge met twice keeps the FIRST copy's jumper flag.
    #[test]
    fn a_duplicated_edge_keeps_the_first_jumper_flag() {
        let mut jumper = seg(5, 5, 1, 15, 5, 1);
        jumper.is_jumper = true;
        let route = vec![seg(5, 5, 1, 25, 5, 1), jumper];
        let got = pruned_merged_route(&route, &[(5, 5, 1), (25, 5, 1)], 0, 0, 10).expect("the duplicate goes");
        assert_eq!(got, vec![seg(5, 5, 1, 25, 5, 1)]);
        assert!(!got[0].is_jumper, "the first copy was not a jumper");
    }

    /// Rule: a segment end off a gcell centre leaves the route as it is.
    #[test]
    fn an_off_centre_end_leaves_the_route() {
        let route = vec![seg(5, 5, 1, 25, 5, 1), seg(15, 5, 1, 15, 22, 1)];
        assert_eq!(pruned_merged_route(&route, &[(5, 5, 1), (25, 5, 1)], 0, 0, 10), None);
    }
}

