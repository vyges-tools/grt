// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 1 — `ant::WireBuilder`'s segments from guides.
//!
//! End to end, `grt-antenna-wires-score.py` scores the `antenna_wires` step against the
//! reference's own trace on `repair_antennas_only_jumpers` (gcd, sky130hs): 5,414/5,414 segments
//! exact, in order, on both pre-repair calls. That design walks x (1,775) and y (1,473), rises
//! through vias (2,151) and takes the two-guide via form over a pin (15). The rules below are each
//! pinned on a constructed case — the ones marked golden-blind the design never reaches.

use vyges_grt::wire_builder::{
    box_to_guide_segment, db_net_is_local, make_net_wires_from_guides, make_wire_from_guides, AntNet, GuidePoint,
    GuideSegment,
};
use vyges_grt::{Guide, Rect};

const G: i32 = 7200;

fn guide(layer: i32, via_layer: i32, b: Rect) -> Guide {
    Guide { layer, via_layer, box_: b, is_congested: false, is_jumper: false, is_connected_to_term: false }
}

fn pt(x: i32, y: i32, layer: i32) -> GuidePoint {
    GuidePoint { x, y, layer }
}

fn seg(a: GuidePoint, b: GuidePoint) -> GuideSegment {
    GuideSegment { pt1: a, pt2: b }
}

/// One cell: `x0 == x1 && y0 == y1`, so the guide is a via from `layer` to `via_layer` at the
/// cell centre, and BOTH box limits are the upper corner (`box_limits = {ur, ur}`).
#[test]
fn one_cell_box_is_a_via_to_the_via_layer() {
    let mut route = Vec::new();
    let b = Rect::new(3 * G, 5 * G, 4 * G, 6 * G);
    let ends = box_to_guide_segment(&b, 2, 3, &mut route, G);
    let c = (3 * G + G / 2, 5 * G + G / 2);
    assert_eq!(route, vec![seg(pt(c.0, c.1, 2), pt(c.0, c.1, 3))]);
    assert_eq!(ends.endpoints, (pt(c.0, c.1, 2), pt(c.0, c.1, 3)));
    assert_eq!(ends.box_limits, ((4 * G, 6 * G), (4 * G, 6 * G)));
}

/// Golden-blind: a one-cell WIRE guide names its own layer twice, so the "via" it becomes has the
/// same layer at both ends. The design never writes one; the rule is `pt2.layer = via_layer`
/// whatever that is.
#[test]
fn one_cell_wire_guide_is_a_same_layer_via() {
    let mut route = Vec::new();
    box_to_guide_segment(&Rect::new(0, 0, G, G), 4, 4, &mut route, G);
    assert_eq!(route, vec![seg(pt(G / 2, G / 2, 4), pt(G / 2, G / 2, 4))]);
    assert!(route[0].is_via());
}

/// A row of cells walks x one cell at a time on `layer`; `via_layer` is NOT read, and the limits
/// are lower corner / upper corner.
#[test]
fn row_walks_x_on_layer_and_ignores_via_layer() {
    let mut route = Vec::new();
    let b = Rect::new(G, 2 * G, 4 * G, 3 * G);
    let ends = box_to_guide_segment(&b, 3, 9, &mut route, G);
    let y = 2 * G + G / 2;
    let xs = [G + G / 2, 2 * G + G / 2, 3 * G + G / 2];
    assert_eq!(route, vec![seg(pt(xs[0], y, 3), pt(xs[1], y, 3)), seg(pt(xs[1], y, 3), pt(xs[2], y, 3))]);
    assert_eq!(ends.endpoints, (pt(xs[0], y, 3), pt(xs[2], y, 3)));
    assert_eq!(ends.box_limits, ((G, 2 * G), (4 * G, 3 * G)));
}

/// A column walks y, low to high.
#[test]
fn column_walks_y() {
    let mut route = Vec::new();
    box_to_guide_segment(&Rect::new(0, 0, G, 3 * G), 2, 2, &mut route, G);
    let x = G / 2;
    assert_eq!(route, vec![seg(pt(x, G / 2, 2), pt(x, G + G / 2, 2)), seg(pt(x, G + G / 2, 2), pt(x, 2 * G + G / 2, 2))]);
}

/// ⛔ Golden-blind: a box two cells wide AND two tall satisfies neither walk's guard, and is not a
/// single cell either — it yields NO segments, only its endpoints.
#[test]
fn box_spanning_both_axes_yields_nothing() {
    let mut route = Vec::new();
    let ends = box_to_guide_segment(&Rect::new(0, 0, 2 * G, 2 * G), 2, 2, &mut route, G);
    assert!(route.is_empty());
    assert_eq!(ends.endpoints, (pt(G / 2, G / 2, 2), pt(G + G / 2, G + G / 2, 2)));
}

/// The corners are SNAPPED to cells, not just offset: a box whose edges sit inside cells (the
/// last guide in a row, extended to the grid edge) still yields cell centres.
#[test]
fn corners_snap_to_cells() {
    let mut route = Vec::new();
    // x from inside cell 1 to a partial cell 3; y exactly one cell.
    box_to_guide_segment(&Rect::new(G + 100, 0, 3 * G + 5000, G), 2, 2, &mut route, G);
    // x0 snaps DOWN to cell 1's centre; x1 = 3G - G/2, cell 2's centre: one step.
    assert_eq!(route, vec![seg(pt(G + G / 2, G / 2, 2), pt(2 * G + G / 2, G / 2, 2))]);
}

/// Guides append to ONE route in guide order, and nothing is deduplicated here: the two-guide via
/// form over a pin gives an up via AND a down via at the same cell.
#[test]
fn guides_append_in_order_without_dedup() {
    let cell = Rect::new(0, 0, G, G);
    let (route, ends) = make_wire_from_guides(&[guide(1, 2, cell), guide(2, 1, cell), guide(1, 2, cell)], G);
    let c = G / 2;
    assert_eq!(
        route,
        vec![seg(pt(c, c, 1), pt(c, c, 2)), seg(pt(c, c, 2), pt(c, c, 1)), seg(pt(c, c, 1), pt(c, c, 2))]
    );
    assert_eq!(ends.len(), 3);
}

/// `dbNetIsLocal` compares BOXES only: a via stack at one cell is local whatever its layers.
#[test]
fn local_is_every_guide_on_one_box() {
    let cell = Rect::new(0, 0, G, G);
    assert!(db_net_is_local(&[guide(1, 2, cell), guide(2, 3, cell), guide(3, 3, cell)]));
    assert!(!db_net_is_local(&[guide(1, 2, cell), guide(2, 2, Rect::new(0, 0, 2 * G, G))]));
    // ⚠️ No guides: the reference reads its first guide unguarded and its loop never runs — "local".
    assert!(db_net_is_local(&[]));
}

fn net(name: &str, term_count: u32, guides: Vec<Guide>) -> AntNet {
    AntNet {
        name: name.into(),
        is_special: false,
        is_connected_by_abutment: false,
        term_count,
        is_detailed_routed: false,
        guides,
    }
}

/// The filter: special, abutted, one-terminal, local and detailed-routed nets get no wire; the
/// rest keep the order they were given.
#[test]
fn filter_drops_each_excluded_kind_and_keeps_order() {
    let row = Rect::new(0, 0, 2 * G, G);
    let cell = Rect::new(0, 0, G, G);
    let mut special = net("special", 2, vec![guide(2, 2, row)]);
    special.is_special = true;
    let mut abutted = net("abutted", 2, vec![guide(2, 2, row)]);
    abutted.is_connected_by_abutment = true;
    let mut routed = net("routed", 2, vec![guide(2, 2, row), guide(2, 3, cell)]);
    routed.is_detailed_routed = true;
    let nets = vec![
        net("b", 2, vec![guide(2, 2, row), guide(2, 3, cell)]),
        special,
        abutted,
        net("one_term", 1, vec![guide(2, 2, row), guide(2, 3, cell)]),
        net("local", 3, vec![guide(1, 2, cell), guide(2, 1, cell)]),
        routed,
        net("a", 2, vec![guide(2, 2, row), guide(2, 3, cell)]),
    ];
    let names: Vec<String> = make_net_wires_from_guides(&nets, G).into_iter().map(|w| w.net).collect();
    assert_eq!(names, vec!["b", "a"]);
}

/// ⛔ The filter short-circuits BEFORE `dbNetIsLocal`: a one-terminal net with no guides at all is
/// dropped by its terminal count and never reaches the unguarded read.
#[test]
fn filter_short_circuits_before_the_local_test() {
    let wires = make_net_wires_from_guides(&[net("lonely", 1, vec![])], G);
    assert!(wires.is_empty());
}
