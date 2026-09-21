// SPDX-License-Identifier: Apache-2.0
//! R16 — expanding a routed edge into a full three-dimensional path.
//!
//! Layer assignment leaves each grid point carrying a layer, but two consecutive points may sit on
//! different layers with nothing between them. This pass inserts the intermediate points, so every
//! step of the resulting path moves either in the plane or by exactly one layer.
//!
//! ⛔ **An edge whose length is not positive is left ENTIRELY untouched** — its points, its length
//! and its route type all keep whatever they held. It is not converted and not marked.
//!
//! ⛔ **The inserted points take the NEXT point's coordinates**, not the current one's. So a step
//! that changes layer becomes a move in the plane first and then a stack of vias at the
//! destination — never a stack at the origin followed by a move.
//!
//! ⚠️ **`routelen` is the authority, not the number of points.** The walk reads `routelen` steps
//! and then one final point, so anything in the array past that is dropped rather than copied.

/// One grid point, at the reference's own widths.
///
/// ⚠️ All three fields are 16-bit in the reference. The layer counter that walks between two
/// points is the same width, transcribed rather than widened.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Point3D {
    pub x: i16,
    pub y: i16,
    pub layer: i16,
}

/// How a route was produced. Only the last matters to this pass, which stamps it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteType {
    NoRoute,
    LRoute,
    ZRoute,
    MazeRoute,
}

/// One edge of a net's tree, as this pass sees it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Edge3D {
    /// The Manhattan distance between the edge's two end nodes. ⛔ The gate for this pass.
    pub len: i32,
    pub route_type: RouteType,
    /// Number of steps, which is one less than the number of points the walk reads.
    pub routelen: i32,
    pub grids: Vec<Point3D>,
}

/// Expand one edge in place.
///
/// ⛔ Returns without touching anything when the edge's length is not positive — including the
/// route type, which stays whatever the earlier stage left.
pub fn convert_edge_to_full_3d(edge: &mut Edge3D, num_layers: usize) {
    if edge.len <= 0 {
        return;
    }
    let route_len = edge.routelen.max(0) as usize;
    // ⚠️ Capacity only — the reference reserves this much and then pushes freely. It does not
    // bound how many points may be inserted.
    let mut tmp: Vec<Point3D> = Vec::with_capacity(route_len + num_layers);

    for j in 0..route_len {
        tmp.push(edge.grids[j]);
        let (here, next) = (edge.grids[j].layer, edge.grids[j + 1].layer);
        // ⛔ Both directions start at THIS point's layer and stop before the next point's, so the
        // first inserted point repeats the current layer at the next point's position. The next
        // iteration — or the final push — supplies the destination layer itself.
        if here > next {
            let mut k: i16 = here;
            while k > next {
                tmp.push(Point3D { x: edge.grids[j + 1].x, y: edge.grids[j + 1].y, layer: k });
                k -= 1;
            }
        } else if here < next {
            let mut k: i16 = here;
            while k < next {
                tmp.push(Point3D { x: edge.grids[j + 1].x, y: edge.grids[j + 1].y, layer: k });
                k += 1;
            }
        }
    }
    // The last point, which the loop above never pushes.
    tmp.push(edge.grids[route_len]);

    edge.routelen = tmp.len() as i32 - 1;
    edge.grids = tmp;
    // ⚠️ Stamped unconditionally once the edge qualifies, whatever it was before.
    edge.route_type = RouteType::MazeRoute;
}

/// Expand every edge of every net, in the reference's order.
///
/// A thin sequencer: it decides nothing itself.
pub fn convert_to_full_3d_type2(nets: &mut [Vec<Edge3D>], num_layers: usize) {
    for edges in nets.iter_mut() {
        for edge in edges.iter_mut() {
            convert_edge_to_full_3d(edge, num_layers);
        }
    }
}
