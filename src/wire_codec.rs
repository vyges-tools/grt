// SPDX-License-Identifier: Apache-2.0
//! Antenna repair — what the database makes of the wire `WireBuilder` encodes: `dbWireEncoder`'s
//! opcodes, then `dbWireShapeItr`'s shapes, as far as the builder's three calls reach.
//!
//! The checker never sees the encoder calls, only the SHAPES the iterator decodes from them, so the
//! decode is behaviour: which point runs become a segment box, how far a box extends past its end
//! points, which width a wrong-way run takes, and where a via's box lands.
//!
//! [`encode`] turns [`WireOp`]s into the opcode stream (`addPoint` chooses X / Y / COLINEAR by
//! comparing with the LAST point; `addTechVia` exits through whichever of the via's layers is not
//! the current one); [`decode`] walks it as `dbWireShapeItr::next` does. Only the default-width,
//! no-extension forms occur — the builder passes no rule and no extension — and nothing else is
//! modelled.

use crate::wire_builder::WireOp;
use crate::Rect;

/// A routing layer as the decode reads it, by routing level.
#[derive(Debug, Clone, Copy)]
pub struct CodecLayer {
    /// `getWidth()`.
    pub width: i32,
    /// `getWrongWayWidth()`.
    pub wrong_way_width: i32,
    /// ⚠️ Two flags, not one: the decode tests `== VERTICAL` for an X run and `== HORIZONTAL` for
    /// a Y run, so a layer with neither direction takes the plain half-width both ways.
    pub vertical: bool,
    pub horizontal: bool,
}

/// A tech via: its layers by routing level, `getBBox()`, and `getBoxes()` in order, each on a TECH
/// layer (by index into the caller's layer list — cut layers included).
#[derive(Debug, Clone)]
pub struct CodecVia {
    pub name: String,
    pub bottom: i32,
    pub top: i32,
    pub bbox: Option<Rect>,
    pub boxes: Vec<(usize, Rect)>,
}

/// The two lookups the codec needs.
pub trait CodecTech {
    fn layer(&self, level: i32) -> &CodecLayer;
    /// `default_vias_[layer]`, by the via's bottom routing level.
    fn via(&self, bottom: i32) -> Option<&CodecVia>;
}

/// One opcode, as far as the builder produces them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    /// `kPath`, on a routing level.
    Path(i32),
    X(i32),
    Y(i32),
    Colinear,
    /// `kTechVia`, the via (by bottom level) and whether it exits through its TOP layer.
    TechVia { bottom: i32, exit_top: bool },
}

/// `dbWireEncoder` over the builder's calls: `newPath` → `kPath`; `addPoint` → `kX`+`kY` for a
/// path's first point, else `kColinear` / `kX` / `kY` against the LAST point (`kProperty`, which the
/// decode skips, is not kept); `addTechVia` → `kTechVia`, exiting top when entered from the bottom.
///
/// ⛔ The last point (`x_`, `y_`) is NOT reset by `newPath`; only the point count is. Harmless here —
/// a path's first point always writes both coordinates — but it is why a repeated point is
/// COLINEAR only within a path.
///
/// Refused: a diagonal point (the reference asserts, and in a release build writes nothing), and a
/// via touching neither the current layer (it would write via id 0).
pub fn encode(ops: &[WireOp], tech: &dyn CodecTech) -> Result<Vec<Op>, String> {
    let mut out = Vec::new();
    let (mut x, mut y) = (0, 0);
    let mut point_cnt = 0;
    let mut layer = 0;
    for op in ops {
        match *op {
            WireOp::Path(l) => {
                layer = l;
                point_cnt = 0;
                out.push(Op::Path(l));
            }
            WireOp::Point(px, py) => {
                if point_cnt == 0 {
                    out.push(Op::X(px));
                    out.push(Op::Y(py));
                    (x, y) = (px, py);
                    point_cnt += 1;
                } else if x == px && y == py {
                    out.push(Op::Colinear);
                } else if y == py {
                    out.push(Op::X(px));
                    x = px;
                    point_cnt += 1;
                } else if x == px {
                    out.push(Op::Y(py));
                    y = py;
                    point_cnt += 1;
                } else {
                    return Err(format!("a diagonal step ({x},{y})-({px},{py})"));
                }
            }
            WireOp::Via(bottom) => {
                let v = tech.via(bottom).ok_or_else(|| format!("no default via above level {bottom}"))?;
                if v.top == layer {
                    layer = v.bottom;
                    out.push(Op::TechVia { bottom, exit_top: false });
                } else if v.bottom == layer {
                    layer = v.top;
                    out.push(Op::TechVia { bottom, exit_top: true });
                } else {
                    return Err(format!("via {} does not touch level {layer}", v.name));
                }
            }
        }
    }
    Ok(out)
}

/// One decoded shape.
#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    /// A path segment's box, on a routing level.
    Segment { level: i32, rect: Rect },
    /// A via: its box (the via's bbox at the point) and `getViaBoxes`, each on a tech layer.
    Via { name: String, rect: Rect, boxes: Vec<(usize, Rect)> },
}

/// `dbWireShapeItr::next`, over [`encode`]'s stream.
///
/// ⛔ A segment's HALF-WIDTH is the layer's own for a run along its direction and the WRONG-WAY
/// width's half across it — but the extension past each end point is always the layer's own
/// half-width. A via resets the half-width to its exit layer's.
///
/// ⛔ A COLINEAR point (the same point twice) makes a zero-length segment — a square of the
/// half-width — once the path has more than one point. The count starts at the path's first
/// point, so a colinear SECOND point already makes one; it can never be a path's first point (the
/// encoder writes that one as X and Y).
pub fn decode(ops: &[Op], tech: &dyn CodecTech) -> Vec<Shape> {
    let mut shapes = Vec::new();
    let (mut prev_x, mut prev_y) = (0, 0);
    let mut layer = 0;
    let mut dw = 0;
    let mut point_cnt = 0;
    let mut i = 0;
    while i < ops.len() {
        match ops[i] {
            Op::Path(l) => {
                layer = l;
                point_cnt = 0;
                dw = tech.layer(l).width >> 1;
            }
            Op::X(cur_x) => {
                let cur_y = if point_cnt == 0 {
                    i += 1;
                    match ops[i] {
                        Op::Y(y) => y,
                        other => panic!("a path's first X is followed by {other:?}, not Y"),
                    }
                } else {
                    prev_y
                };
                let first = point_cnt == 0;
                point_cnt += 1;
                if !first {
                    let l = tech.layer(layer);
                    let w = if l.vertical { l.wrong_way_width / 2 } else { dw };
                    shapes.push(Shape::Segment { level: layer, rect: set_segment((prev_x, prev_y), (cur_x, cur_y), w, dw) });
                }
                (prev_x, prev_y) = (cur_x, cur_y);
            }
            Op::Y(cur_y) => {
                point_cnt += 1;
                let cur_x = prev_x;
                let l = tech.layer(layer);
                let w = if l.horizontal { l.wrong_way_width / 2 } else { dw };
                shapes.push(Shape::Segment { level: layer, rect: set_segment((prev_x, prev_y), (cur_x, cur_y), w, dw) });
                (prev_x, prev_y) = (cur_x, cur_y);
            }
            Op::Colinear => {
                point_cnt += 1;
                if point_cnt > 1 {
                    shapes.push(Shape::Segment { level: layer, rect: set_segment((prev_x, prev_y), (prev_x, prev_y), dw, dw) });
                }
            }
            Op::TechVia { bottom, exit_top } => {
                let v = tech.via(bottom).expect("encoded against the same vias");
                layer = if exit_top { v.top } else { v.bottom };
                dw = tech.layer(layer).width >> 1;
                if let Some(b) = v.bbox {
                    let rect = Rect { x_min: b.x_min + prev_x, y_min: b.y_min + prev_y, x_max: b.x_max + prev_x, y_max: b.y_max + prev_y };
                    let boxes = v.boxes.iter().map(|&(t, r)| (t, r.move_delta(prev_x, prev_y))).collect();
                    shapes.push(Shape::Via { name: v.name.clone(), rect, boxes });
                }
            }
        }
        i += 1;
    }
    shapes
}

/// `dbShape::setSegment` without extensions: the box of a run from `prev` to `cur`, `dw` either
/// side of it and `ext` past each end; a zero-length run is a `dw` square.
pub fn set_segment(prev: (i32, i32), cur: (i32, i32), dw: i32, ext: i32) -> Rect {
    let ((px, py), (cx, cy)) = (prev, cur);
    let (x1, y1, x2, y2) = if cx == px {
        let (y1, y2) = if cy > py {
            (py - ext, cy + ext)
        } else if cy < py {
            (cy - ext, py + ext)
        } else {
            (cy - dw, cy + dw)
        };
        (cx - dw, y1, cx + dw, y2)
    } else {
        let (x1, x2) = if cx > px { (px - ext, cx + ext) } else { (cx - ext, px + ext) };
        (x1, cy - dw, x2, cy + dw)
    };
    Rect { x_min: x1, y_min: y1, x_max: x2, y_max: y2 }
}
