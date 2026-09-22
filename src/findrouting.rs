// SPDX-License-Identifier: Apache-2.0
//! F — `findRouting`'s post-processing of the router's result: [`add_remaining_guides`] (local
//! nets and top-level pins), [`connect_pad_pins`], and per net [`merge_segments`].
//!
//! The router's routes are keyed by database net and walked in that map's order; every step here
//! works per net, so only the per-net content is decided here.

use std::collections::BTreeMap;

use crate::GSegment;

/// A pin as F reads it: on-grid position and connection layer.
pub type GridPin = (i32, i32, i32);

/// GRT-76: a net the router left without segments whose pins are NOT all on one cell.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NoRouteGuides {
    pub net: String,
}

/// `addGuidesForLocalNets`: a net whose pins all sit on one cell gets a via stack there — from the
/// lower of its lowest pin layer and the min routing layer, up to one past its highest pin layer.
///
/// ⛔ The top is `last_layer` — the highest pin layer, LESS ONE when it is at or above the max
/// routing layer — and the stack's vias run `l → l + 1` for `l` up to it, so a stack normally rises
/// ONE ABOVE the highest pin. GRT-76 when two consecutive pins differ in cell.
pub fn add_guides_for_local_net(
    net: &str,
    pins: &[GridPin],
    min_routing_layer: i32,
    max_routing_layer: i32,
) -> Result<Vec<GSegment>, NoRouteGuides> {
    let mut last_layer = -1;
    let mut min_pin_layer = i32::MAX;
    for (p, pin) in pins.iter().enumerate() {
        if p > 0 && (pin.0 != pins[p - 1].0 || pin.1 != pins[p - 1].1) {
            return Err(NoRouteGuides { net: net.into() });
        }
        min_pin_layer = min_pin_layer.min(pin.2);
        last_layer = last_layer.max(pin.2);
    }
    if last_layer >= max_routing_layer {
        last_layer -= 1;
    }
    let min_layer = min_pin_layer.min(min_routing_layer);
    let (x, y) = (pins[0].0, pins[0].1);
    Ok((min_layer..=last_layer).map(|l| GSegment::new(x, y, l, x, y, l + 1)).collect())
}

/// `connectTopLevelPins`: a via stack from the BLOCK's max routing layer up to each pin connected
/// above it (bumps at the top layer).
pub fn connect_top_level_pins(pins: &[GridPin], block_max_routing_layer: i32) -> Vec<GSegment> {
    let mut out = Vec::new();
    for &(x, y, l) in pins {
        if l > block_max_routing_layer {
            for k in block_max_routing_layer..l {
                out.push(GSegment::new(x, y, k, x, y, k + 1));
            }
        }
    }
    out
}

/// A net as `addRemainingGuides` reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemainingNet {
    pub name: String,
    /// The same gate as `initNetlist`'s ([`crate::makes_fastroute_net`]).
    pub made: bool,
    pub pins: Vec<GridPin>,
}

/// `addRemainingGuides`: for every net through the gate, an EMPTY route (or none — the lookup
/// inserts one) gets [`add_guides_for_local_net`], a non-empty one [`connect_top_level_pins`].
pub fn add_remaining_guides(
    routes: &mut BTreeMap<String, Vec<GSegment>>,
    nets: &[RemainingNet],
    min_routing_layer: i32,
    max_routing_layer: i32,
    block_max_routing_layer: i32,
) -> Result<(), NoRouteGuides> {
    for n in nets.iter().filter(|n| n.made) {
        let route = routes.entry(n.name.clone()).or_default();
        if route.is_empty() {
            route.extend(add_guides_for_local_net(&n.name, &n.pins, min_routing_layer, max_routing_layer)?);
        } else {
            route.extend(connect_top_level_pins(&n.pins, block_max_routing_layer));
        }
    }
    Ok(())
}

/// `connectPadPins` — appends each net's `pad_pins_connections_`. ⛔ Nothing ever WRITES that map
/// (it is cleared in `initNetlist` and read here only), so this is a no-op; kept as the stage it is.
pub fn connect_pad_pins(_routes: &mut BTreeMap<String, Vec<GSegment>>) {}

/// `segmentsConnect(seg0, seg1, new_seg, segs_at_point)`: two collinear planar segments meeting
/// end to end at a point where EXACTLY two things meet (the two segments — no pin, no third) merge
/// into their span, written into `seg1`.
fn segments_connect(seg0: &GSegment, seg1: &mut GSegment, segs_at_point: &BTreeMap<(i32, i32, i32), i32>) -> bool {
    let (ix0, iy0) = (seg0.init_x.min(seg0.final_x), seg0.init_y.min(seg0.final_y));
    let (fx0, fy0) = (seg0.final_x.max(seg0.init_x), seg0.final_y.max(seg0.init_y));
    let (ix1, iy1) = (seg1.init_x.min(seg1.final_x), seg1.init_y.min(seg1.final_y));
    let (fx1, fy1) = (seg1.final_x.max(seg1.init_x), seg1.final_y.max(seg1.init_y));
    let at = |x, y, l| segs_at_point[&(x, y, l)];
    let merge = if ix0 == fx0 && ix1 == fx1 && ix0 == ix1 {
        // vertical, aligned
        if iy0 == fy1 {
            at(ix0, iy0, seg0.init_layer) == 2
        } else if fy0 == iy1 {
            at(ix1, iy1, seg1.init_layer) == 2
        } else {
            false
        }
    } else if iy0 == fy0 && iy1 == fy1 && iy0 == iy1 {
        // horizontal, aligned
        if ix0 == fx1 {
            at(ix0, iy0, seg0.init_layer) == 2
        } else if fx0 == ix1 {
            at(ix1, iy1, seg1.init_layer) == 2
        } else {
            false
        }
    } else {
        false
    };
    if merge {
        seg1.init_x = ix0.min(ix1);
        seg1.init_y = iy0.min(iy1);
        seg1.final_x = fx0.max(fx1);
        seg1.final_y = fy0.max(fy1);
    }
    merge
}

/// `mergeSegments(pins, route)`: fold each segment into the NEXT when both are planar on one layer
/// at or above the block's min routing layer and [`segments_connect`] says they meet end to end.
///
/// ⛔ The point counts are taken ONCE, before any merge (every segment endpoint, every pin), and
/// never updated — a merged segment is judged by the original counts. The last segment is always
/// kept; merged-away segments are dropped in place, order preserved.
pub fn merge_segments(pins: &[GridPin], route: &mut Vec<GSegment>, block_min_routing_layer: i32) {
    if route.is_empty() {
        return;
    }
    let mut segs_at_point: BTreeMap<(i32, i32, i32), i32> = BTreeMap::new();
    for s in route.iter() {
        *segs_at_point.entry((s.init_x, s.init_y, s.init_layer)).or_default() += 1;
        *segs_at_point.entry((s.final_x, s.final_y, s.final_layer)).or_default() += 1;
    }
    for &p in pins {
        *segs_at_point.entry(p).or_default() += 1;
    }
    let (mut read, mut write) = (0, 0);
    while read < route.len() - 1 {
        let seg0 = route[read];
        let mut seg1 = route[read + 1];
        let planar_same_layer = seg0.init_layer == seg0.final_layer
            && seg1.init_layer == seg1.final_layer
            && seg0.init_layer == seg1.init_layer
            && seg0.init_layer >= block_min_routing_layer;
        if planar_same_layer {
            if segments_connect(&seg0, &mut seg1, &segs_at_point) {
                route[read + 1] = seg1;
            } else {
                route[write] = seg0;
                write += 1;
            }
        } else {
            route[write] = seg0;
            write += 1;
        }
        read += 1;
    }
    route[write] = route[read];
    route.truncate(write + 1);
}
