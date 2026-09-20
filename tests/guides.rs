// SPDX-License-Identifier: Apache-2.0
//! The guide-writing rules, one test per rule, with the rule stated at the site.

use vyges_grt::*;

fn grid() -> Grid {
    // 100-unit tiles over a 0..1000 square: big enough that the boundary snap is reachable but
    // not so big that every box hits it.
    Grid { tile_size: 100, area: Rect::new(0, 0, 1000, 1000) }
}

fn opts() -> SaveOptions {
    SaveOptions { guide_is_congested: false, origin_x: 0, origin_y: 0, min_routing_layer: 2 }
}

fn wire(x0: i32, y0: i32, x1: i32, y1: i32, layer: i32) -> GSegment {
    GSegment { init_x: x0, init_y: y0, init_layer: layer,
               final_x: x1, final_y: y1, final_layer: layer, is_jumper: false }
}

fn via(x: i32, y: i32, l0: i32, l1: i32) -> GSegment {
    GSegment { init_x: x, init_y: y, init_layer: l0,
               final_x: x, final_y: y, final_layer: l1, is_jumper: false }
}

fn net(segments: Vec<GSegment>) -> NetRoute {
    NetRoute { name: "n1".into(), segments, pins: vec![], is_local: false }
}

#[test]
fn a_via_is_defined_by_position_not_by_layer() {
    // ⛔ The rule the obvious implementation gets wrong. A segment is a via when it does not move
    // in x or y — NOT when its layers differ. A segment that both moves and changes layer is
    // therefore NOT a via, and falls through both branches to produce nothing.
    assert!(via(300, 300, 2, 3).is_via());
    assert!(wire(300, 300, 300, 300, 2).is_via(), "a zero-length wire is a via by this rule");
    assert!(!wire(300, 300, 500, 300, 2).is_via());

    let moving_layer_change = GSegment { init_x: 300, init_y: 300, init_layer: 2,
                                         final_x: 500, final_y: 300, final_layer: 3,
                                         is_jumper: false };
    assert!(!moving_layer_change.is_via());
    let g = guides_for_segment(&net(vec![]), &moving_layer_change, &grid(), &opts()).unwrap();
    assert!(g.is_empty(), "neither a via nor same-layer: the chain has no final else");
}

#[test]
fn a_guide_box_grows_by_half_a_tile_in_each_direction() {
    let seg = wire(300, 300, 500, 300, 2);
    let b = global_routing_to_box(&seg, &grid());
    assert_eq!(b, Rect::new(250, 250, 550, 350));
}

#[test]
fn the_endpoints_are_ordered_before_the_half_tile_is_applied() {
    // A segment may be stored either way round; the half-tile goes on the LOWER and UPPER
    // corners, not on init and final. Both directions must give the same box.
    let forward = global_routing_to_box(&wire(300, 300, 500, 300, 2), &grid());
    let reverse = global_routing_to_box(&wire(500, 300, 300, 300, 2), &grid());
    assert_eq!(forward, reverse);
}

#[test]
fn a_box_within_one_tile_of_the_grid_edge_snaps_out_to_it() {
    // ⚠️ Truncating integer division, and that is the rule: (area_max - ur) / tile < 1.
    // ur_x = 950 leaves a 50-unit gap -> 50/100 == 0 -> snaps to 1000.
    let b = global_routing_to_box(&wire(700, 700, 900, 900, 2), &grid());
    assert_eq!(b.x_max, 1000, "a sub-tile gap is closed, not left as a sliver");
    assert_eq!(b.y_max, 1000);

    // A full tile of clearance is left alone: ur = 850, gap 150 -> 150/100 == 1.
    let b2 = global_routing_to_box(&wire(700, 700, 800, 800, 2), &grid());
    assert_eq!((b2.x_max, b2.y_max), (850, 850));
}

#[test]
fn the_grid_origin_is_added_to_every_guide() {
    let o = SaveOptions { origin_x: 7, origin_y: -3, ..opts() };
    let g = guides_for_segment(&net(vec![]), &wire(300, 300, 500, 300, 2), &grid(), &o).unwrap();
    assert_eq!(g[0].box_, Rect::new(257, 247, 557, 347));
}

#[test]
fn an_ordinary_via_writes_ONE_guide_with_min_and_max_layers() {
    let g = guides_for_segment(&net(vec![]), &via(300, 300, 3, 2), &grid(), &opts()).unwrap();
    assert_eq!(g.len(), 1);
    // written high-to-low on purpose: the guide must still name min then max
    assert_eq!((g[0].layer, g[0].via_layer), (2, 3));
}

#[test]
fn a_via_on_a_LOCAL_net_writes_TWO_guides_in_both_layer_directions() {
    // ⛔ This changes the guide COUNT, not just its contents — the fork is worth its own test.
    let mut n = net(vec![]);
    n.is_local = true;
    let g = guides_for_segment(&n, &via(300, 300, 2, 3), &grid(), &opts()).unwrap();
    assert_eq!(g.len(), 2);
    assert_eq!((g[0].layer, g[0].via_layer), (2, 3));
    assert_eq!((g[1].layer, g[1].via_layer), (3, 2));
    assert_eq!(g[0].box_, g[1].box_, "both guides cover the same box");
}

#[test]
fn a_via_covering_a_pin_also_writes_TWO_guides() {
    // is_covering_pin compares the segment's FINAL point and its TOP layer.
    let mut n = net(vec![]);
    n.pins = vec![Pin { connection_layer: 3, on_grid_x: 300, on_grid_y: 300 }];
    let g = guides_for_segment(&n, &via(300, 300, 2, 3), &grid(), &opts()).unwrap();
    assert_eq!(g.len(), 2, "a via landing on a pin takes the two-guide form");

    // same pin one layer down -> not the segment's top layer -> ordinary single guide
    n.pins = vec![Pin { connection_layer: 2, on_grid_x: 300, on_grid_y: 300 }];
    assert_eq!(guides_for_segment(&n, &via(300, 300, 2, 3), &grid(), &opts()).unwrap().len(), 1);

    // right layer, wrong place
    n.pins = vec![Pin { connection_layer: 3, on_grid_x: 400, on_grid_y: 300 }];
    assert_eq!(guides_for_segment(&n, &via(300, 300, 2, 3), &grid(), &opts()).unwrap().len(), 1);
}

#[test]
fn a_via_between_non_adjacent_layers_is_an_error() {
    let e = guides_for_segment(&net(vec![]), &via(300, 300, 2, 4), &grid(), &opts()).unwrap_err();
    assert!(matches!(e, GuideError::NonAdjacentLayers { from: 2, to: 4, .. }));
    // and adjacent in either direction is fine
    assert!(guides_for_segment(&net(vec![]), &via(300, 300, 4, 3), &grid(), &opts()).is_ok());
}

#[test]
fn a_wire_names_its_own_layer_twice() {
    let g = guides_for_segment(&net(vec![]), &wire(300, 300, 500, 300, 2), &grid(), &opts()).unwrap();
    assert_eq!((g[0].layer, g[0].via_layer), (2, 2));
}

#[test]
fn a_jumper_wire_is_flagged_and_counted() {
    let mut seg = wire(300, 300, 500, 300, 2);
    seg.is_jumper = true;
    let out = save_guides(&[net(vec![seg])], &grid(), &opts()).unwrap();
    assert!(out[0].guides[0].is_jumper);
    assert_eq!(out[0].jumper_count, 1);
}

#[test]
fn a_DIAGONAL_wire_below_the_minimum_routing_layer_is_an_error() {
    // ⚠️ All three conditions are required: below min layer AND moving in x AND moving in y.
    // A straight segment on the same layer is fine, which is why the test checks both.
    let diagonal = wire(300, 300, 500, 500, 1);
    let e = guides_for_segment(&net(vec![]), &diagonal, &grid(), &opts()).unwrap_err();
    assert!(matches!(e, GuideError::BlockedMetal { layer: 1, .. }));

    let straight = wire(300, 300, 500, 300, 1);
    assert!(guides_for_segment(&net(vec![]), &straight, &grid(), &opts()).is_ok(),
            "a straight low-layer segment is allowed; only the diagonal is not");
}

#[test]
fn congestion_is_stamped_on_every_guide_from_the_RUN_not_the_net() {
    let o = SaveOptions { guide_is_congested: true, ..opts() };
    let out = save_guides(
        &[net(vec![wire(300, 300, 500, 300, 2)]), net(vec![via(300, 300, 2, 3)])],
        &grid(), &o).unwrap();
    assert!(out.iter().flat_map(|n| &n.guides).all(|g| g.is_congested));
}

#[test]
fn a_net_with_no_segments_is_skipped_entirely() {
    // ⛔ It must not appear in the output at all: the published stage skips it before touching
    // the database, so such a net keeps whatever guides it already had.
    let out = save_guides(&[net(vec![]), net(vec![wire(300, 300, 500, 300, 2)])],
                          &grid(), &opts()).unwrap();
    assert_eq!(out.len(), 1);
}

#[test]
fn guides_come_out_in_segment_order() {
    // The guide file is compared as an ordered list, so this is load-bearing rather than tidy.
    let segs = vec![wire(100, 100, 200, 100, 2), wire(200, 100, 300, 100, 2),
                    wire(300, 100, 400, 100, 2)];
    let out = save_guides(&[net(segs)], &grid(), &opts()).unwrap();
    let xs: Vec<i32> = out[0].guides.iter().map(|g| g.box_.x_min).collect();
    assert_eq!(xs, vec![50, 150, 250]);
}

#[test]
fn a_rect_normalises_so_corner_order_cannot_change_a_result() {
    assert_eq!(Rect::new(10, 20, 3, 4), Rect::new(3, 4, 10, 20));
}
