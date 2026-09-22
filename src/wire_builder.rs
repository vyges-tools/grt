// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 1 — the wire `ant::WireBuilder` synthesises for a net FROM ITS GUIDES.
//!
//! `repair_antennas` has no geometry of its own: before the checker charges any area, every net
//! that is not detailed-routed is given a wire built from its guides, one grid-cell step at a time.
//! This module is that synthesis, as far as the SEGMENTS: [`make_net_wires_from_guides`] →
//! [`make_wire_from_guides`] → [`box_to_guide_segment`], in the reference's order.
//!
//! ⚠️ The reference writes the result into the database (`dbWireEncoder`) because its checker reads
//! it from there. What must match is the segment list and, later, the violations — not the
//! intermediate database state — so the segments are returned instead.
//!
//! ⬜ Not here yet: the pin stubs `addWireTerms` / `makeWireToTerm` add while encoding (stage 2
//! needs them), and ANT-0015 (a segment between non-adjacent layers), which `makeNetWire` raises
//! while encoding.

use crate::{Guide, Rect};

/// A grid-cell centre on a routing layer.
///
/// ⚠️ The reference holds the layer as a `dbTechLayer*` and compares POINTERS; for routing layers
/// that is the same test as comparing routing levels, which is what is held here.
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

/// A net as `makeNetWiresFromGuides` filters it.
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
}

/// One net's synthesised wire.
#[derive(Debug, Clone, PartialEq)]
pub struct NetWire {
    pub net: String,
    pub route: Vec<GuideSegment>,
    /// One per guide, in guide order.
    pub ends: Vec<GuideEnds>,
}

/// `WireBuilder::makeNetWiresFromGuides(nets)`: a wire for every net in the order given, less the
/// nets the filter drops.
///
/// ⛔ The filter is ONE short-circuit `&&` chain and its order matters: `dbNetIsLocal` is only
/// reached for a non-special, non-abutted net with more than one terminal and it dereferences the
/// net's first guide, so it must never be evaluated ahead of those tests.
pub fn make_net_wires_from_guides(nets: &[AntNet], gcell_dimension: i32) -> Vec<NetWire> {
    let mut wires = Vec::new();
    for net in nets {
        if !net.is_special
            && !net.is_connected_by_abutment
            && net.term_count > 1
            && !db_net_is_local(&net.guides)
            && !net.is_detailed_routed
        {
            wires.push(make_net_wire(net, gcell_dimension));
        }
    }
    wires
}

/// `WireBuilder::makeNetWire`, as far as the segments.
fn make_net_wire(net: &AntNet, gcell_dimension: i32) -> NetWire {
    let (route, ends) = make_wire_from_guides(&net.guides, gcell_dimension);
    NetWire { net: net.name.clone(), route, ends }
}

/// `WireBuilder::makeWireFromGuides`: every guide, in order, turned into segments and appended to
/// ONE route.
///
/// ⚠️ Segments are appended as they are made and never deduplicated here — a via guide written
/// twice (the two-guide form over a pin) contributes two via segments. The dedup happens later,
/// while encoding, against a set keyed on all six coordinates.
pub fn make_wire_from_guides(guides: &[Guide], gcell_dimension: i32) -> (Vec<GuideSegment>, Vec<GuideEnds>) {
    let mut route = Vec::new();
    let mut ends = Vec::with_capacity(guides.len());
    for guide in guides {
        ends.push(box_to_guide_segment(&guide.box_, guide.layer, guide.via_layer, &mut route, gcell_dimension));
    }
    (route, ends)
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
