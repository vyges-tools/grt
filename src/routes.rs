// SPDX-License-Identifier: Apache-2.0
//! R20 — handing the routes back: each net's grid paths as database-unit segments, and the run's
//! closing report.
//!
//! `get_routes` is the router's only output. Every earlier stage worked in grid cells and layer
//! indices; this one converts each step of each routed edge into a segment in database units, with
//! layers counted from one, and drops the steps the net has already emitted.
//!
//! ⛔ **What counts as "already emitted" is decided by the segment's CONSTRUCTOR, not by the walk**
//! — see [`GSegment::new`]: a planar step walked backwards is the same segment and is dropped, a
//! via walked downwards is a different segment from the same via upwards, which is why a separate
//! test for the reversed via follows.

use std::collections::{BTreeMap, HashSet};

use crate::full3d::Point3D;
use crate::GSegment;

/// The grid's placement in database units.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridOrigin {
    pub tile_size: i32,
    pub x_corner: i32,
    pub y_corner: i32,
}

/// A grid index to the database unit at its cell's centre.
///
/// ⚠️ Computed in `double` — `tile * (index + 0.5) + corner` — and TRUNCATED back to an integer,
/// so an odd tile size loses its half unit rather than rounding it.
pub fn grid_to_dbu(index: i16, tile_size: i32, corner: i32) -> i32 {
    (f64::from(tile_size) * (f64::from(index) + 0.5) + f64::from(corner)) as i32
}

/// One edge as this stage reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RouteEdge {
    pub len: i32,
    pub routelen: i32,
    pub grids: Vec<Point3D>,
}

/// One net's segments, in the order they are emitted.
///
/// ⛔ The edge gate is `len > 0 || routelen > 0` — the same one pin coverage uses, so the via
/// stacks that pass appended (zero length, positive steps) are emitted too.
///
/// ⚠️ The dedup set spans the whole NET, not one edge: a step two edges share is emitted once, by
/// the first edge in index order.
pub fn get_net_route(edges: &[RouteEdge], origin: GridOrigin) -> Vec<GSegment> {
    let GridOrigin { tile_size, x_corner, y_corner } = origin;
    let mut seen: HashSet<GSegment> = HashSet::new();
    let mut route = Vec::new();
    for edge in edges {
        if !(edge.len > 0 || edge.routelen > 0) {
            continue;
        }
        let g = &edge.grids;
        let mut last_x = grid_to_dbu(g[0].x, tile_size, x_corner);
        let mut last_y = grid_to_dbu(g[0].y, tile_size, y_corner);
        let mut last_l = i32::from(g[0].layer);
        // ⚠️ `routelen` steps, not the array's length; a negative count walks nothing.
        for i in 1..=edge.routelen {
            let p = &g[i as usize];
            let x = grid_to_dbu(p.x, tile_size, x_corner);
            let y = grid_to_dbu(p.y, tile_size, y_corner);
            let segment = GSegment::new(last_x, last_y, last_l + 1, x, y, i32::from(p.layer) + 1);
            (last_x, last_y, last_l) = (x, y, i32::from(p.layer));

            if seen.contains(&segment) {
                continue;
            }
            // ⛔ A via is also dropped when the SAME via walked the other way was emitted. The
            // reversed segment goes back through the constructor, which leaves x and y where
            // they are (they are equal) and swaps only the layers.
            if segment.init_layer != segment.final_layer {
                let reversed = GSegment::new(
                    segment.final_x,
                    segment.final_y,
                    segment.final_layer,
                    segment.init_x,
                    segment.init_y,
                    segment.init_layer,
                );
                if seen.contains(&reversed) {
                    continue;
                }
            }
            seen.insert(segment);
            route.push(segment);
        }
    }
    route
}

/// One net as the whole-run collection reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetForRoutes {
    /// The net's database id — the key the result is ordered by.
    pub db_id: u32,
    pub edges: Vec<RouteEdge>,
}

/// Every net's segments — `getRoutes`.
///
/// ⛔ The result is keyed by the net's DATABASE ID and iterated in ascending id order (the
/// reference's `PtrMap` compares by id), NOT in the router's net order. Nets are visited in the
/// order given, but the map erases it.
///
/// ⚠️ A second net sharing a database id would APPEND to the first one's segments with a fresh
/// dedup set — the reference indexes the map with `operator[]`. Transcribed; no design does it.
pub fn get_routes(nets: &[NetForRoutes], origin: GridOrigin) -> BTreeMap<u32, Vec<GSegment>> {
    let mut routes: BTreeMap<u32, Vec<GSegment>> = BTreeMap::new();
    for net in nets {
        let segments = get_net_route(&net.edges, origin);
        routes.entry(net.db_id).or_default().extend(segments);
    }
    routes
}

/// What the run's closing report says that can be correlated — `reportRunMetrics`.
///
/// ⚠️ The stage's other sixteen metrics are wall-clock timings and snapshot-batch statistics. The
/// timings are this engine's own and cannot match; they are not modelled here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunReport {
    /// The `global_route__vias` metric, always written.
    pub vias_metric: i32,
    /// GRT-0111 and GRT-0112, only in verbose mode.
    pub verbose_lines: Vec<String>,
}

/// ⛔ "Final usage 3D" is the final 2D-and-layer USAGE plus **three per via** — a fixed via
/// weight applied at report time, not a quantity any stage accumulated.
pub fn report_run_metrics(num_vias: i32, final_length: i32, verbose: bool) -> RunReport {
    let verbose_lines = if verbose {
        vec![
            format!("[INFO GRT-0111] Final number of vias: {num_vias}"),
            format!("[INFO GRT-0112] Final usage 3D: {}", final_length + 3 * num_vias),
        ]
    } else {
        Vec::new()
    };
    RunReport { vias_metric: num_vias, verbose_lines }
}
