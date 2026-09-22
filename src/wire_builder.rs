// SPDX-License-Identifier: Apache-2.0
//! Antenna repair — the wire `ant::WireBuilder` synthesises for a net FROM ITS GUIDES, as the
//! checker then reads it.
//!
//! `repair_antennas` has no geometry of its own: before the checker charges any area, every net
//! that is not detailed-routed is given a wire built from its guides, one grid-cell step at a time,
//! plus a stub from the grid to each pin a guide is bound to. In the reference's order:
//! [`make_net_wires_from_guides`] → [`make_net_wire`] → [`make_wire_from_guides`] (segments and the
//! guide/pin binding) → per segment [`add_wire_terms`] → [`make_wire_to_term`] → [`make_wire`].
//!
//! ⚠️ The reference writes the result through `dbWireEncoder` because its checker reads it from
//! the database. Here the ENCODER CALLS are the output ([`WireOp`]) — `newPath`, `addPoint`,
//! `addTechVia`, in order — which is exactly what the encoder is given; decoding them into shapes
//! is the next step, not this one.
//!
//! ⚠️ Pins are bound by BOUNDING BOX only: the access-point branch of the binding is taken when a
//! terminal has preferred access points, which a design read from DEF does not. The caller refuses
//! anything else.

use std::collections::{BTreeMap, HashSet};

use crate::{Guide, Rect};

/// A grid-cell centre on a routing layer.
///
/// ⚠️ The reference holds the layer as a `dbTechLayer*` and compares POINTERS; for routing layers
/// that is the same test as comparing routing levels, which is what is held here. The derived
/// order — x, then y, then level — is `GuidePoint::operator<` (`Point`'s defaulted `<=>`, then
/// the routing level), which orders the pin binding map.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GuidePoint {
    pub x: i32,
    pub y: i32,
    pub layer: i32,
}

/// One step of the synthesised wire: a cell-to-cell run on one layer, or a via at one cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GuideSegment {
    pub pt1: GuidePoint,
    pub pt2: GuidePoint,
}

impl GuideSegment {
    /// A via is defined by POSITION: the two points share x and y (`GuideSegment::isVia`).
    pub fn is_via(&self) -> bool {
        self.pt1.x == self.pt2.x && self.pt1.y == self.pt2.y
    }
}

/// What `boxToGuideSegment` hands back beside the segments: the two points a pin may bind to, and
/// the box corner each one's search window is clipped to. Only the pin binding reads them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuideEnds {
    pub endpoints: (GuidePoint, GuidePoint),
    pub box_limits: ((i32, i32), (i32, i32)),
}

/// A terminal as the wire builder reads it — an instance terminal or a block terminal.
#[derive(Debug, Clone)]
pub struct TermFacts {
    /// `inst/pin` for an instance terminal, the port name for a block terminal.
    pub name: String,
    /// The highest routing level among the terminal's ROUTING shapes, 0 when it has none. Both
    /// `check*Connection`'s top layer and `get*TopLayerRects`' index are this number.
    pub top_level: i32,
    /// `getBBox()`: every pin shape, any layer, placed.
    pub bbox: Rect,
    /// `get*TopLayerRects`: the ROUTING shapes on `top_level`, placed, in pin → shape order.
    pub top_rects: Vec<Rect>,
}

/// The binding of one guide point: the terminals bound there, and whether a stub was made.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GuidePtPins {
    /// Indices into the net's `bterms`.
    pub bterms: Vec<usize>,
    /// Indices into the net's `iterms`.
    pub iterms: Vec<usize>,
    pub connected: bool,
}

/// A net as `makeNetWiresFromGuides` filters it and `makeNetWire` builds it.
#[derive(Debug, Clone)]
pub struct AntNet {
    pub name: String,
    pub is_special: bool,
    pub is_connected_by_abutment: bool,
    /// `dbNet::getTermCount`: instance terminals plus block terminals.
    pub term_count: u32,
    /// `getWireType() == ROUTED && getWire()`: the net already has a detailed wire.
    pub is_detailed_routed: bool,
    /// The net's guides in database order — creation order, since `saveGuides` reverses the
    /// prepending list after writing.
    pub guides: Vec<Guide>,
    /// `getITerms()` order.
    pub iterms: Vec<TermFacts>,
    /// `getBTerms()` order.
    pub bterms: Vec<TermFacts>,
}

/// The technology as the builder reads it.
#[derive(Debug, Clone, Copy)]
pub struct WireTech {
    /// `block_->getMinRoutingLayer()`.
    pub min_routing_layer: i32,
    /// The min routing layer's preferred direction is VERTICAL.
    pub min_layer_vertical: bool,
}

/// One `dbWireEncoder` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireOp {
    /// `newPath(layer, ROUTED)`, by routing level.
    Path(i32),
    /// `addPoint(x, y)`.
    Point(i32, i32),
    /// `addTechVia(default_vias_[layer])`, by the routing level of the via's bottom layer.
    Via(i32),
}

/// One net's synthesised wire.
#[derive(Debug, Clone, PartialEq)]
pub struct NetWire {
    pub net: String,
    pub route: Vec<GuideSegment>,
    /// One per guide, in guide order.
    pub ends: Vec<GuideEnds>,
    /// `route_pt_pins` as `makeWireFromGuides` left it (before the stubs mark points connected).
    pub pt_pins: BTreeMap<GuidePoint, GuidePtPins>,
    /// The encoder calls, in order.
    pub ops: Vec<WireOp>,
}

/// What stops the builder, named after the reference's error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireError {
    /// ANT-0015: a segment between layers that are not adjacent.
    NonAdjacentLayers { net: String, from: i32, to: i32 },
    /// A case the reference cannot survive either (a null layer or via dereferenced).
    Unsupported(String),
}

/// `WireBuilder::makeNetWiresFromGuides(nets)`: a wire for every net in the order given, less the
/// nets the filter drops.
///
/// ⛔ The filter is ONE short-circuit `&&` chain and its order matters: `dbNetIsLocal` is only
/// reached for a non-special, non-abutted net with more than one terminal and it dereferences the
/// net's first guide, so it must never be evaluated ahead of those tests.
pub fn make_net_wires_from_guides(nets: &[AntNet], gcell_dimension: i32, tech: &WireTech) -> Result<Vec<NetWire>, WireError> {
    let mut wires = Vec::new();
    for net in nets {
        if !net.is_special
            && !net.is_connected_by_abutment
            && net.term_count > 1
            && !db_net_is_local(&net.guides)
            && !net.is_detailed_routed
        {
            wires.push(make_net_wire(net, gcell_dimension, tech)?);
        }
    }
    Ok(wires)
}

/// `WireBuilder::makeNetWire`: the segments, each ENCODED once — a via as a one-point path and a
/// tech via, a wire as a two-point path with a pin stub at either end where a pin is bound.
///
/// ⛔ `prev_conn_layer` threads through the whole net: a via's path starts on the layer the
/// previous path ENDED on when it can (bottom first, then top), and only a WIRE sets it outright.
/// A via that joins neither starts on its bottom layer and leaves it unchanged.
///
/// ⛔ A via whose bottom layer is below the min routing layer is dropped whole — not encoded and
/// not recorded as done, so a later copy of it is dropped too.
///
/// ⚠️ "Bottom" and "top" are chosen with STRICT comparisons, so for a same-layer segment both are
/// `pt2`'s layer.
pub fn make_net_wire(net: &AntNet, gcell_dimension: i32, tech: &WireTech) -> Result<NetWire, WireError> {
    let (route, ends, mut pt_pins) = make_wire_from_guides(net, gcell_dimension);
    let pt_pins_bound = pt_pins.clone();
    let mut ops = Vec::new();
    let mut wire_segments: HashSet<GuideSegment> = HashSet::new();
    let mut prev_conn_layer = -1;
    for seg in &route {
        let (l1, l2) = (seg.pt1.layer, seg.pt2.layer);
        let bottom = if l1 < l2 { l1 } else { l2 };
        let top = if l1 > l2 { l1 } else { l2 };
        if (l1 - l2).abs() > 1 {
            return Err(WireError::NonAdjacentLayers { net: net.name.clone(), from: bottom, to: top });
        }
        if wire_segments.contains(seg) {
            continue;
        }
        let (x1, y1) = (seg.pt1.x, seg.pt1.y);
        if seg.is_via() {
            if bottom >= tech.min_routing_layer {
                if bottom == prev_conn_layer {
                    ops.push(WireOp::Path(bottom));
                    prev_conn_layer = l1.max(l2);
                } else if top == prev_conn_layer {
                    ops.push(WireOp::Path(top));
                    prev_conn_layer = l1.min(l2);
                } else {
                    ops.push(WireOp::Path(bottom));
                }
                ops.push(WireOp::Point(x1, y1));
                ops.push(WireOp::Via(bottom));
                add_wire_terms(net, &route, x1, y1, bottom, &mut pt_pins, &mut ops, false, tech)?;
                wire_segments.insert(*seg);
            }
        } else {
            let (x2, y2) = (seg.pt2.x, seg.pt2.y);
            if x1 != x2 || y1 != y2 {
                add_wire_terms(net, &route, x1, y1, l1, &mut pt_pins, &mut ops, true, tech)?;
                ops.push(WireOp::Path(l1));
                ops.push(WireOp::Point(x1, y1));
                ops.push(WireOp::Point(x2, y2));
                add_wire_terms(net, &route, x2, y2, l1, &mut pt_pins, &mut ops, true, tech)?;
                wire_segments.insert(*seg);
                prev_conn_layer = l1;
            }
        }
    }
    Ok(NetWire { net: net.name.clone(), route, ends, pt_pins: pt_pins_bound, ops })
}

/// `WireBuilder::addWireTerms`: at a segment end, a stub to every terminal bound at that point —
/// on the segment's layer, and ALSO the layer below when the segment is on the min routing layer.
///
/// ⛔ A point's terminals are stubbed ONCE: the first end to reach it marks it connected, block
/// terminals first, then instance terminals.
#[allow(clippy::too_many_arguments)]
pub fn add_wire_terms(
    net: &AntNet,
    route: &[GuideSegment],
    grid_x: i32,
    grid_y: i32,
    layer: i32,
    pt_pins: &mut BTreeMap<GuidePoint, GuidePtPins>,
    ops: &mut Vec<WireOp>,
    connect_to_segment: bool,
    tech: &WireTech,
) -> Result<(), WireError> {
    let mut layers = vec![layer];
    if layer == tech.min_routing_layer {
        if layer - 1 < 1 {
            // findRoutingLayer(0) is null, and the map's comparator dereferences it.
            return Err(WireError::Unsupported(format!("net {}: min routing layer 1 has no layer below", net.name)));
        }
        layers.push(layer - 1);
    }
    for l in layers {
        let key = GuidePoint { x: grid_x, y: grid_y, layer: l };
        let Some(entry) = pt_pins.get_mut(&key) else { continue };
        if entry.connected {
            continue;
        }
        let (bterms, iterms) = (entry.bterms.clone(), entry.iterms.clone());
        entry.connected = true;
        for t in bterms.iter().map(|&b| &net.bterms[b]).chain(iterms.iter().map(|&i| &net.iterms[i])) {
            if t.top_level < 1 {
                return Err(WireError::Unsupported(format!("net {}: terminal {} has no routing shape", net.name, t.name)));
            }
            make_wire_to_term(route, layer, t.top_level, &t.top_rects, (key.x, key.y), ops, connect_to_segment, tech)?;
        }
    }
    Ok(())
}

/// `WireBuilder::makeWireToTerm`: the stub from a grid point to a terminal's pin.
///
/// The target is the grid point itself when the pin already covers it (or a same-layer segment of
/// the route touches the pin); otherwise the CENTRE of the pin shape nearest the grid point
/// (Manhattan, first strictly smaller wins).
///
/// ⛔ A pin at or above the min routing layer gets an L on the SEGMENT's layer (x first). A pin
/// below it gets: when connecting to a segment above the min layer, a via stack down to it (vias
/// listed from the min layer UP — the reference's order); then an L on the min layer whose corner
/// carries the min layer's OWN default via (up); then vias down to the pin's layer.
#[allow(clippy::too_many_arguments)]
pub fn make_wire_to_term(
    route: &[GuideSegment],
    layer: i32,
    conn_layer: i32,
    pin_rects: &[Rect],
    grid_pt: (i32, i32),
    ops: &mut Vec<WireOp>,
    connect_to_segment: bool,
    tech: &WireTech,
) -> Result<(), WireError> {
    let mut pin_pt = grid_pt;
    if !pin_overlaps_g_segment(grid_pt, conn_layer, pin_rects, route) {
        let mut min_dist = i64::from(i32::MAX);
        for b in pin_rects {
            let pos = ((b.x_min + b.x_max) / 2, (b.y_min + b.y_max) / 2);
            let dist = i64::from((pos.0 - pin_pt.0).abs()) + i64::from((pos.1 - pin_pt.1).abs());
            if dist < min_dist {
                min_dist = dist;
                pin_pt = pos;
            }
        }
    }
    let min_layer = tech.min_routing_layer;
    if conn_layer >= min_layer {
        ops.push(WireOp::Path(layer));
        ops.push(WireOp::Point(grid_pt.0, grid_pt.1));
        ops.push(WireOp::Point(pin_pt.0, grid_pt.1));
        ops.push(WireOp::Point(pin_pt.0, pin_pt.1));
        return Ok(());
    }
    if connect_to_segment && layer != min_layer {
        ops.push(WireOp::Path(layer));
        ops.push(WireOp::Point(grid_pt.0, grid_pt.1));
        for i in min_layer..layer {
            ops.push(WireOp::Via(i));
        }
    }
    let corner = if tech.min_layer_vertical { (grid_pt.0, pin_pt.1) } else { (pin_pt.0, grid_pt.1) };
    make_wire(ops, min_layer, grid_pt, corner);
    ops.push(WireOp::Via(min_layer));
    make_wire(ops, min_layer, corner, pin_pt);
    let mut i = min_layer - 1;
    while i >= conn_layer {
        ops.push(WireOp::Via(i));
        i -= 1;
    }
    Ok(())
}

/// `WireBuilder::makeWire`: a two-point path.
pub fn make_wire(ops: &mut Vec<WireOp>, layer: i32, start: (i32, i32), end: (i32, i32)) {
    ops.push(WireOp::Path(layer));
    ops.push(WireOp::Point(start.0, start.1));
    ops.push(WireOp::Point(end.0, end.1));
}

/// `WireBuilder::pinOverlapsGSegment`: the grid point lies STRICTLY inside a pin shape
/// (`Rect::overlaps(Point)`), or some same-layer segment of the route on the pin's layer touches a
/// pin shape (`Rect::intersects`, edges included).
///
/// ⚠️ "Same-layer" is `pt1.layer == pt2.layer`, which a one-cell wire guide's same-layer via also
/// satisfies.
pub fn pin_overlaps_g_segment(pin_position: (i32, i32), pin_layer: i32, pin_rects: &[Rect], route: &[GuideSegment]) -> bool {
    let (px, py) = pin_position;
    if pin_rects.iter().any(|b| px > b.x_min && px < b.x_max && py > b.y_min && py < b.y_max) {
        return true;
    }
    for b in pin_rects {
        for seg in route {
            if seg.pt1.layer == seg.pt2.layer && seg.pt1.layer == pin_layer {
                let r = Rect::new(seg.pt1.x, seg.pt1.y, seg.pt2.x, seg.pt2.y);
                if r.x_max >= b.x_min && r.x_min <= b.x_max && r.y_max >= b.y_min && r.y_min <= b.y_max {
                    return true;
                }
            }
        }
    }
    false
}

/// `WireBuilder::makeWireFromGuides`: every guide, in order, turned into segments and appended to
/// ONE route — and, for a guide the router flagged as touching a terminal, each end tested against
/// every instance terminal, then every block terminal.
///
/// ⚠️ Segments are appended as they are made and never deduplicated here — a via guide written
/// twice (the two-guide form over a pin) contributes two via segments. The dedup happens later,
/// while encoding, against a set keyed on all six coordinates.
pub fn make_wire_from_guides(net: &AntNet, gcell_dimension: i32) -> (Vec<GuideSegment>, Vec<GuideEnds>, BTreeMap<GuidePoint, GuidePtPins>) {
    let mut route = Vec::new();
    let mut ends = Vec::with_capacity(net.guides.len());
    let mut pt_pins: BTreeMap<GuidePoint, GuidePtPins> = BTreeMap::new();
    for guide in &net.guides {
        let e = box_to_guide_segment(&guide.box_, guide.layer, guide.via_layer, &mut route, gcell_dimension);
        if guide.is_connected_to_term {
            for (i, t) in net.iterms.iter().enumerate() {
                if check_guide_term_connection(&e.endpoints.0, t, e.box_limits.0, gcell_dimension) {
                    pt_pins.entry(e.endpoints.0).or_default().iterms.push(i);
                }
                if check_guide_term_connection(&e.endpoints.1, t, e.box_limits.1, gcell_dimension) {
                    pt_pins.entry(e.endpoints.1).or_default().iterms.push(i);
                }
            }
            for (i, t) in net.bterms.iter().enumerate() {
                if check_guide_term_connection(&e.endpoints.0, t, e.box_limits.0, gcell_dimension) {
                    pt_pins.entry(e.endpoints.0).or_default().bterms.push(i);
                }
                if check_guide_term_connection(&e.endpoints.1, t, e.box_limits.1, gcell_dimension) {
                    pt_pins.entry(e.endpoints.1).or_default().bterms.push(i);
                }
            }
        }
        ends.push(e);
    }
    (route, ends, pt_pins)
}

/// `checkGuideITermConnection` / `checkGuideBTermConnection`, bounding-box branch (no access
/// points): the terminal's box STRICTLY overlaps the window around the guide point, and the
/// terminal's top routing layer is the point's layer.
///
/// The window is the cell around the point — unless neither of its corners is the guide's box
/// limit, in which case its upper corner becomes that limit (and the box is re-normalised).
pub fn check_guide_term_connection(guide_pt: &GuidePoint, term: &TermFacts, box_limit: (i32, i32), gcell_dimension: i32) -> bool {
    let h = gcell_dimension / 2;
    let ll = (guide_pt.x - h, guide_pt.y - h);
    let mut ur = (guide_pt.x + h, guide_pt.y + h);
    if ll != box_limit && ur != box_limit {
        ur = box_limit;
    }
    let w = Rect::new(ll.0, ll.1, ur.0, ur.1);
    let b = &term.bbox;
    let overlaps = b.x_max > w.x_min && b.x_min < w.x_max && b.y_max > w.y_min && b.y_min < w.y_max;
    overlaps && term.top_level == guide_pt.layer
}

/// `WireBuilder::boxToGuideSegment`: a guide box becomes the chain of cell-centre steps it covers.
///
/// The box's lower corner is snapped DOWN to its cell and moved to that cell's centre; the upper
/// corner is snapped down to a cell boundary and moved back half a cell — so a box covering whole
/// cells yields the centres of its first and last cells.
///
/// ⛔ A box that collapses to ONE cell is a via: its single segment runs from `layer` to
/// `via_layer` at that cell, and BOTH box limits are the upper corner. Any other box is a wire on
/// `layer` alone — `via_layer` is not read — with limits lower-corner / upper-corner.
///
/// ⛔ The two walks are separate `while` loops, x first. The x walk only runs while the centres
/// share a row and the y walk only while they share a column, so a box that differs in both x and
/// y yields NO segments at all.
///
/// ⚠️ Integer division truncates toward zero, as C++'s does; both agree for the non-negative
/// coordinates a placed design has.
pub fn box_to_guide_segment(
    guide_box: &Rect,
    layer: i32,
    via_layer: i32,
    route: &mut Vec<GuideSegment>,
    gcell_dimension: i32,
) -> GuideEnds {
    let g = gcell_dimension;
    let mut x0 = g * (guide_box.x_min / g) + g / 2;
    let mut y0 = g * (guide_box.y_min / g) + g / 2;
    let x1 = g * (guide_box.x_max / g) - g / 2;
    let y1 = g * (guide_box.y_max / g) - g / 2;

    let ends = if x0 == x1 && y0 == y1 {
        let pt1 = GuidePoint { x: x0, y: y0, layer };
        let pt2 = GuidePoint { x: x1, y: y1, layer: via_layer };
        route.push(GuideSegment { pt1, pt2 });
        let ur = (guide_box.x_max, guide_box.y_max);
        GuideEnds { endpoints: (pt1, pt2), box_limits: (ur, ur) }
    } else {
        GuideEnds {
            endpoints: (GuidePoint { x: x0, y: y0, layer }, GuidePoint { x: x1, y: y1, layer }),
            box_limits: ((guide_box.x_min, guide_box.y_min), (guide_box.x_max, guide_box.y_max)),
        }
    };

    while y0 == y1 && x0 + g <= x1 {
        route.push(GuideSegment {
            pt1: GuidePoint { x: x0, y: y0, layer },
            pt2: GuidePoint { x: x0 + g, y: y0, layer },
        });
        x0 += g;
    }

    while x0 == x1 && y0 + g <= y1 {
        route.push(GuideSegment {
            pt1: GuidePoint { x: x0, y: y0, layer },
            pt2: GuidePoint { x: x0, y: y0 + g, layer },
        });
        y0 += g;
    }

    ends
}

/// `WireBuilder::dbNetIsLocal`: every guide of the net has the SAME box as the one before it.
///
/// ⛔ It compares BOXES only, not layers — a via stack at one cell (one box on every layer) is
/// local, and so is a net whose guides are a single box.
///
/// ⚠️ The reference dereferences the first guide unguarded. With no guides at all it reads an
/// invalid object and the loop does not run, so it answers "local"; that is what is returned here.
/// Golden-blind: every net with more than one terminal in the captured designs has guides.
pub fn db_net_is_local(guides: &[Guide]) -> bool {
    let Some(first) = guides.first() else {
        return true;
    };
    let mut last_box = first.box_;
    for guide in guides {
        if last_box != guide.box_ {
            return false;
        }
        last_box = guide.box_;
    }
    true
}
