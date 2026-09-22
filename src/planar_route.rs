// SPDX-License-Identifier: Apache-2.0
//! `FastRouteCore::getPlanarRoute` — a net's route as segments BEFORE layer assignment, which is
//! what the parasitics the router's own slacks are read from are built on
//! (`GlobalRouter::getPartialRoutes` while `routes_` is still empty).
//!
//! The 2D tree carries no layers, so each edge is laid on two PSEUDO-LAYERS derived from the net's
//! minimum layer: the one whose direction matches the step, with a via between them wherever the
//! walk changes direction. Segments are de-duplicated as they are made, and a segment is kept in
//! the order it was first made.
//!
//! ⛔ Where two edges of the same net meet at one point on pseudo-layers ONE apart, a bridging via
//! is inserted (`recordLayerAndBridge`) — without it the net's parasitic graph is disconnected
//! there. A wider gap is left alone: its via stack is already in place from an earlier edge.

use std::collections::BTreeMap;

use crate::layertable::LayerDir;
use crate::parasitics::Segment;
use crate::routes::{grid_to_dbu, GridOrigin};

/// One net's 2D tree as this pass reads it: per edge, its length and its route's grid points.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlanarEdge<'a> {
    pub len: i32,
    pub routelen: i32,
    pub grids: &'a [(i32, i32)],
}

/// The two pseudo-layers an edge is laid on: `layer_h` carries horizontal steps, `layer_v`
/// vertical ones. ⚠️ Both are the net's minimum layer or the one above, by its DIRECTION.
fn pseudo_layers(net_min_layer: usize, layer_dir: &[LayerDir]) -> (i32, i32) {
    let min = net_min_layer as i32;
    match layer_dir.get(net_min_layer) {
        Some(LayerDir::Vertical) => (min + 1, min),
        _ => (min, min + 1),
    }
}

/// `recordLayerAndBridge`: remember the pseudo-layer this point was left on, and bridge a
/// one-layer gap against what a previous edge left there.
fn record_layer_and_bridge(x: i32, y: i32, new_layer: i32, seen: &mut BTreeMap<(i32, i32), i32>, out: &mut Vec<Segment>) {
    if let Some(&prev) = seen.get(&(x, y)) {
        if (prev - new_layer).abs() == 1 {
            let (lo, hi) = (prev.min(new_layer), prev.max(new_layer));
            push_unique(out, Segment { init_x: x, init_y: y, init_layer: lo + 1, final_x: x, final_y: y, final_layer: hi + 1 });
        }
    }
    seen.insert((x, y), new_layer);
}

/// `net_segs` is a SET: a segment already made is not made again, and the route keeps the order
/// the segments were first made in.
fn push_unique(out: &mut Vec<Segment>, seg: Segment) {
    if !out.contains(&seg) {
        out.push(seg);
    }
}

/// `getPlanarRoute(db_net, route)` over one net's edges.
///
/// ⚠️ A zero-length edge is not routed and contributes nothing.
pub fn planar_route(edges: &[PlanarEdge<'_>], net_min_layer: usize, layer_dir: &[LayerDir], origin: GridOrigin) -> Vec<Segment> {
    let (layer_h, layer_v) = pseudo_layers(net_min_layer, layer_dir);
    let GridOrigin { tile_size, x_corner, y_corner } = origin;
    let mut out: Vec<Segment> = Vec::new();
    let mut seen: BTreeMap<(i32, i32), i32> = BTreeMap::new();
    for e in edges {
        if e.len <= 0 {
            continue;
        }
        let dbu = |p: (i32, i32)| (grid_to_dbu(p.0 as i16, tile_size, x_corner), grid_to_dbu(p.1 as i16, tile_size, y_corner));
        let (mut last_x, mut last_y) = dbu(e.grids[0]);
        // ⛔ The edge's first step decides which pseudo-layer it starts on — read from grids[1].
        let second_x = grid_to_dbu(e.grids[1].0 as i16, tile_size, x_corner);
        let mut last_l = if last_x == second_x { layer_v } else { layer_h };
        record_layer_and_bridge(last_x, last_y, last_l, &mut seen, &mut out);
        for i in 1..=e.routelen.max(0) as usize {
            let (xreal, yreal) = dbu(e.grids[i]);
            let seg;
            if last_x == xreal {
                // A vertical step: change layer first if the walk was horizontal.
                if last_l == layer_h {
                    push_unique(&mut out, Segment { init_x: last_x, init_y: last_y, init_layer: last_l + 1, final_x: last_x, final_y: last_y, final_layer: layer_v + 1 });
                }
                last_l = layer_v;
                seg = Segment { init_x: last_x, init_y: last_y, init_layer: last_l + 1, final_x: xreal, final_y: yreal, final_layer: last_l + 1 };
            } else {
                if last_l == layer_v {
                    push_unique(&mut out, Segment { init_x: last_x, init_y: last_y, init_layer: last_l + 1, final_x: last_x, final_y: last_y, final_layer: layer_h + 1 });
                }
                last_l = layer_h;
                seg = Segment { init_x: last_x, init_y: last_y, init_layer: last_l + 1, final_x: xreal, final_y: yreal, final_layer: last_l + 1 };
            }
            (last_x, last_y) = (xreal, yreal);
            push_unique(&mut out, seg);
            // Per hop: this edge may pass through a point an earlier edge left on another layer.
            record_layer_and_bridge(last_x, last_y, last_l, &mut seen, &mut out);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const ORIGIN: GridOrigin = GridOrigin { tile_size: 100, x_corner: 0, y_corner: 0 };

    fn dirs() -> Vec<LayerDir> {
        // level 0 horizontal, 1 vertical, 2 horizontal …
        vec![LayerDir::Horizontal, LayerDir::Vertical, LayerDir::Horizontal, LayerDir::Vertical]
    }

    // A straight horizontal edge: one segment on the horizontal pseudo-layer, no via.
    #[test]
    fn a_horizontal_edge_is_one_segment() {
        let grids = [(0, 0), (1, 0), (2, 0)];
        let r = planar_route(&[PlanarEdge { len: 2, routelen: 2, grids: &grids }], 0, &dirs(), ORIGIN);
        assert_eq!(r.len(), 2);
        assert!(r.iter().all(|s| s.init_layer == 1 && s.final_layer == 1), "{r:?}");
        assert_eq!((r[0].init_x, r[0].final_x), (50, 150));
    }

    // ⚠️ The pseudo-layers follow the minimum layer's DIRECTION: with a vertical minimum layer the
    // horizontal steps go one layer up.
    #[test]
    fn the_pseudo_layers_follow_the_minimum_layers_direction() {
        assert_eq!(pseudo_layers(0, &dirs()), (0, 1));
        assert_eq!(pseudo_layers(1, &dirs()), (2, 1));
    }

    // A turn inserts a via between the two pseudo-layers, before the segment that turns.
    #[test]
    fn a_turn_inserts_a_via() {
        let grids = [(0, 0), (1, 0), (1, 1)];
        let r = planar_route(&[PlanarEdge { len: 2, routelen: 2, grids: &grids }], 0, &dirs(), ORIGIN);
        assert_eq!(r.len(), 3);
        let via = r[1];
        assert_eq!((via.init_x, via.init_y, via.init_layer, via.final_layer), (150, 50, 1, 2));
        assert!(via.is_via() && via.length() == 0);
        assert_eq!((r[2].init_layer, r[2].final_layer), (2, 2));
    }

    // Two edges meeting at one point on pseudo-layers one apart are bridged with a via.
    #[test]
    fn two_edges_meeting_on_different_layers_are_bridged() {
        let a = [(0, 0), (1, 0)]; // horizontal, ends at (150, 50) on layer_h
        let b = [(1, 0), (1, 1)]; // vertical from the same point on layer_v
        let r = planar_route(
            &[PlanarEdge { len: 1, routelen: 1, grids: &a }, PlanarEdge { len: 1, routelen: 1, grids: &b }],
            0,
            &dirs(),
            ORIGIN,
        );
        let bridges: Vec<&Segment> = r.iter().filter(|s| s.is_via() && (s.init_x, s.init_y) == (150, 50)).collect();
        assert_eq!(bridges.len(), 1, "{r:?}");
        assert_eq!((bridges[0].init_layer, bridges[0].final_layer), (1, 2));
    }

    // The same segment made twice is kept once, in the order it was first made.
    #[test]
    fn a_repeated_segment_is_kept_once() {
        let g = [(0, 0), (1, 0)];
        let r = planar_route(
            &[PlanarEdge { len: 1, routelen: 1, grids: &g }, PlanarEdge { len: 1, routelen: 1, grids: &g }],
            0,
            &dirs(),
            ORIGIN,
        );
        assert_eq!(r.len(), 1);
    }

    // A zero-length edge is not routed.
    #[test]
    fn a_zero_length_edge_contributes_nothing() {
        let g = [(0, 0), (0, 0)];
        assert!(planar_route(&[PlanarEdge { len: 0, routelen: 0, grids: &g }], 0, &dirs(), ORIGIN).is_empty());
    }
}
