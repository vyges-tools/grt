// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 2 — `ant::AntennaChecker` over the wire the builder encoded, in the
//! reference's order: [`build_layer_maps`] (`wiresToPolygonSetMap`, `avoidPinIntersection`, the
//! nodes, their links through the vias) …
//!
//! The checker's geometry is Boost.Polygon's, and only the REGION of each layer decides it — see
//! [`crate::polygon90`] for why the polygons, their order and their hole slits still come out
//! exactly as the reference's.

use std::collections::BTreeMap;

use crate::polygon90::{get_polygons, Region, R};
use crate::repair_antennas::TechLayers;
use crate::wire_codec::Shape;

/// One polygon of one layer: a `GraphNode`.
#[derive(Debug, Clone)]
pub struct GraphNode {
    pub id: usize,
    pub is_via: bool,
    /// As `polygon_90_data` lists it.
    pub pol: Vec<(i32, i32)>,
    /// Indices into the layer BELOW's nodes (for a routing layer: its cut layer's).
    pub low_adj: Vec<usize>,
}

/// `LayerToGraphNodes`: per tech layer (by index — the `PtrMap`'s order is the layer id's, which is
/// tech order), its nodes in polygon order.
pub type LayerNodes = BTreeMap<usize, Vec<GraphNode>>;

/// `buildLayerMaps`, as far as the nodes and their links (`saveGates` is the next step).
///
/// Per layer the region is every decoded segment box and every via box on it, less every
/// ROUTING-layer pin box of the net's instance terminals (`avoidPinIntersection`) — a pin cuts the
/// metal over it, so charge reaching a gate is counted on the pieces either side.
///
/// ⚠️ `pins` are the net's instance terminals' routing boxes, placed, by tech layer.
pub fn build_layer_maps(shapes: &[Shape], pins: &[(usize, R)], tech: &TechLayers) -> Result<LayerNodes, String> {
    let mut add: BTreeMap<usize, Vec<R>> = BTreeMap::new();
    let mut sub: BTreeMap<usize, Vec<R>> = BTreeMap::new();
    for sh in shapes {
        match sh {
            Shape::Segment { level, rect } => {
                let t = tech.find_routing_layer(*level).ok_or_else(|| format!("no routing level {level}"))?;
                add.entry(t).or_default().push((rect.x_min, rect.y_min, rect.x_max, rect.y_max));
            }
            Shape::Via { boxes, .. } => {
                for (t, b) in boxes {
                    add.entry(*t).or_default().push((b.x_min, b.y_min, b.x_max, b.y_max));
                }
            }
        }
    }
    for &(t, r) in pins {
        sub.entry(t).or_default().push(r);
        add.entry(t).or_default();
    }
    // Nodes: layers in tech order, each layer's polygons in formation order, ids running on.
    let mut nodes: LayerNodes = BTreeMap::new();
    let mut regions: BTreeMap<usize, Vec<Region>> = BTreeMap::new();
    let mut id = 0;
    for (&t, rects) in &add {
        let region = Region::new(rects, sub.get(&t).map(Vec::as_slice).unwrap_or(&[]));
        let is_via = tech.0[t].routing_level == 0;
        let mut list = Vec::new();
        let mut regs = Vec::new();
        for pol in get_polygons(&region) {
            regs.push(polygon_region(&pol));
            list.push(GraphNode { id, is_via, pol, low_adj: Vec::new() });
            id += 1;
        }
        nodes.insert(t, list);
        regions.insert(t, regs);
    }
    // Links through each cut layer: every upper node touching a via lists it; the via lists every
    // lower node touching it.
    let cut_layers: Vec<usize> = add.keys().copied().filter(|&t| tech.0[t].routing_level == 0).collect();
    for t in cut_layers {
        let (lower, upper) = (tech.0[t].lower, tech.0[t].upper);
        let vias = regions.get(&t).cloned().unwrap_or_default();
        for (via_index, via) in vias.iter().enumerate() {
            let lower_index = lower.map(|l| find_nodes_with_intersection(regions.get(&l).map(Vec::as_slice).unwrap_or(&[]), via)).unwrap_or_default();
            let upper_index = upper.map(|u| find_nodes_with_intersection(regions.get(&u).map(Vec::as_slice).unwrap_or(&[]), via)).unwrap_or_default();
            if let Some(u) = upper {
                for up in upper_index {
                    nodes.get_mut(&u).expect("a node found there")[up].low_adj.push(via_index);
                }
            }
            for low in lower_index {
                nodes.get_mut(&t).expect("the via layer")[via_index].low_adj.push(low);
            }
        }
    }
    Ok(nodes)
}

/// `findNodesWithIntersection`: the nodes whose polygon overlaps `pol` grown by 1 (a square
/// Minkowski sum, as Boost's `+= 1` is) in POSITIVE area — so metal one unit away counts, and
/// metal touching only at a corner of the grown shape does not.
pub fn find_nodes_with_intersection(nodes: &[Region], pol: &Region) -> Vec<usize> {
    let grown: Vec<R> = pol.rects().into_iter().map(|r| (r.0 - 1, r.1 - 1, r.2 + 1, r.3 + 1)).collect();
    nodes
        .iter()
        .enumerate()
        .filter(|(_, n)| n.rects().iter().any(|a| grown.iter().any(|b| a.2 > b.0 && a.0 < b.2 && a.3 > b.1 && a.1 < b.3)))
        .map(|(i, _)| i)
        .collect()
}

/// The region a polygon's points enclose (even-odd over its vertical edges — a hole's slit, two
/// coincident opposite edges, encloses nothing).
pub fn polygon_region(pol: &[(i32, i32)]) -> Region {
    let mut xs: Vec<i32> = pol.iter().map(|p| p.0).collect();
    let mut ys: Vec<i32> = pol.iter().map(|p| p.1).collect();
    xs.sort_unstable();
    xs.dedup();
    ys.sort_unstable();
    ys.dedup();
    let nx = xs.len().saturating_sub(1);
    let ny = ys.len().saturating_sub(1);
    let mut cells = vec![vec![false; ny]; nx];
    let n = pol.len();
    for k in 0..n {
        let (a, b) = (pol[k], pol[(k + 1) % n]);
        if a.0 != b.0 || a.1 == b.1 {
            continue;
        }
        let (y0, y1) = (a.1.min(b.1), a.1.max(b.1));
        let i0 = xs.binary_search(&a.0).expect("a vertex x");
        let (j0, j1) = (ys.binary_search(&y0).expect("a vertex y"), ys.binary_search(&y1).expect("a vertex y"));
        for col in &mut cells[i0..] {
            for c in &mut col[j0..j1] {
                *c = !*c;
            }
        }
    }
    Region { xs, ys, cells }
}
