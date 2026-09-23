// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 2b — the wire codec on a constructed technology.
//!
//! End to end, `grt-antcheck-score.py --kinds shape,vbox` replays every decoded segment and via box
//! against the reference's trace. The sky130hs corpus cannot see three of the decode's rules: it
//! declares no wrong-way width (so either half-width reads the same), and every via the builder
//! places is followed by a new path (so the layer a via EXITS through, and the half-width it
//! resets, are never read). The technology below makes each of them visible.

use vyges_grt::wire_builder::WireOp;
use vyges_grt::wire_codec::{decode, encode, CodecLayer, CodecTech, CodecVia, Shape};
use vyges_grt::Rect;

/// Level 1: horizontal, width 100, wrong-way 300. Level 2: vertical, width 200, wrong-way 500. One
/// via between them.
struct Tech {
    layers: [CodecLayer; 2],
    via: CodecVia,
}

impl CodecTech for Tech {
    fn layer(&self, level: i32) -> &CodecLayer {
        &self.layers[(level - 1) as usize]
    }
    fn via(&self, bottom: i32) -> Option<&CodecVia> {
        (bottom == 1).then_some(&self.via)
    }
}

fn tech() -> Tech {
    Tech {
        layers: [
            CodecLayer { width: 100, wrong_way_width: 300, vertical: false, horizontal: true },
            CodecLayer { width: 200, wrong_way_width: 500, vertical: true, horizontal: false },
        ],
        via: CodecVia { name: "V12".into(), bottom: 1, top: 2, bbox: Some(Rect::new(-10, -10, 10, 10)), boxes: Vec::new() },
    }
}

fn shapes(ops: &[WireOp]) -> Vec<Shape> {
    let t = tech();
    decode(&encode(ops, &t).expect("encodable"), &t)
}

fn seg(level: i32, x0: i32, y0: i32, x1: i32, y1: i32) -> Shape {
    Shape::Segment { level, rect: Rect::new(x0, y0, x1, y1) }
}

fn via_at(x: i32, y: i32) -> Shape {
    Shape::Via { name: "V12".into(), rect: Rect::new(x - 10, y - 10, x + 10, y + 10), boxes: Vec::new() }
}

/// `dbWireShapeItr`: a run ALONG a layer's direction is its own half-width wide; a run ACROSS it
/// takes half the WRONG-WAY width — tested as `== VERTICAL` for an X run and `== HORIZONTAL` for a
/// Y run. The extension past each end is the layer's own half-width either way.
#[test]
fn a_wrong_way_run_takes_the_wrong_way_half_width() {
    use WireOp::{Path, Point};
    // Horizontal layer: X along (half-width 50), Y across (150, extended by 50).
    assert_eq!(shapes(&[Path(1), Point(0, 0), Point(1000, 0), Point(1000, 2000)]), vec![seg(1, -50, -50, 1050, 50), seg(1, 850, -50, 1150, 2050)]);
    // Vertical layer: X across (250, extended by 100), Y along (100).
    assert_eq!(shapes(&[Path(2), Point(0, 0), Point(1000, 0), Point(1000, 2000)]), vec![seg(2, -100, -250, 1100, 250), seg(2, 900, -100, 1100, 2100)]);
}

/// `addTechVia` going UP exits through the via's top layer; the decode then continues the path on
/// it, with that layer's half-width — ⛔ even though no new path was started.
#[test]
fn a_via_going_up_continues_on_its_top_layer() {
    use WireOp::{Path, Point, Via};
    let got = shapes(&[Path(1), Point(0, 0), Via(1), Point(0, 2000)]);
    // On level 2 (vertical, half-width 100) — not level 1's 150 across / 50 along.
    assert_eq!(got, vec![via_at(0, 0), seg(2, -100, -100, 100, 2100)]);
}

/// … and going DOWN, through its bottom layer.
#[test]
fn a_via_going_down_continues_on_its_bottom_layer() {
    use WireOp::{Path, Point, Via};
    let got = shapes(&[Path(2), Point(0, 0), Via(1), Point(2000, 0)]);
    // On level 1 (horizontal, half-width 50) — not level 2's 250 across.
    assert_eq!(got, vec![via_at(0, 0), seg(1, -50, -50, 2050, 50)]);
}

/// `addPoint` of the point just added writes COLINEAR, not X or Y: the decode draws it as a square
/// of the layer's own half-width. Encoded as an X instead, a vertical layer would take its WRONG-WAY
/// half-width for it.
#[test]
fn a_repeated_point_is_colinear() {
    use WireOp::{Path, Point};
    let got = shapes(&[Path(2), Point(0, 0), Point(0, 1000), Point(0, 1000)]);
    assert_eq!(got, vec![seg(2, -100, -100, 100, 1100), seg(2, -100, 900, 100, 1100)]);
}
