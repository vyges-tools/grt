// SPDX-License-Identifier: Apache-2.0
//! The setup stage: the grid, and the rules that classify a net.
//!
//! 🔑 **`is_local` is checked against the reference on a whole design.** The corpus records what
//! the published stage decided for all 563 nets, so this is a correlation of the rule and not a
//! restatement of it.

#![allow(non_snake_case)]

use vyges_grt::*;

const CORPUS: &str = include_str!("../examples/grt_gate/corpus.json");

fn pin(layer: i32, x: i32, y: i32) -> Pin {
    Pin { connection_layer: layer, on_grid_x: x, on_grid_y: y }
}

#[test]
fn is_local_matches_the_REFERENCE_on_every_net_of_a_whole_design() {
    let v: serde_json::Value = serde_json::from_str(CORPUS).unwrap();
    let mut checked = 0;
    let mut local = 0;
    for net in v["nets"].as_array().unwrap() {
        let pins: Vec<Pin> = net["pins"].as_array().unwrap().iter().map(|p| Pin {
            connection_layer: p["connection_layer"].as_i64().unwrap() as i32,
            on_grid_x: p["on_grid_x"].as_i64().unwrap() as i32,
            on_grid_y: p["on_grid_y"].as_i64().unwrap() as i32,
        }).collect();
        let want = net["is_local"].as_bool().unwrap();
        assert_eq!(
            is_local(&pins), want,
            "net {}: is_local", net["name"].as_str().unwrap()
        );
        checked += 1;
        if want { local += 1; }
    }
    // ⚠️ Vacuity guards: an empty corpus, or one where every net fell the same way, would satisfy
    // the loop above without testing the rule.
    assert_eq!(checked, 563, "every net must have been classified");
    assert_eq!(local, 68, "the reference calls this many local; neither all nor none");
}

#[test]
fn a_net_with_NO_pins_is_local() {
    // ⚠️ The empty case returns true, not false. It falls out of "every pin is at the first pin's
    // position" only if that is written as an all-match over the tail; an implementation that
    // demanded at least one pin would answer the opposite.
    assert!(is_local(&[]));
}

#[test]
fn locality_compares_POSITION_ONLY_and_ignores_the_layer() {
    // ⛔ Two pins at the same point on different layers make a LOCAL net. This decides the
    // two-guide via form, so comparing the layer as well would change the guide COUNT.
    assert!(is_local(&[pin(1, 100, 200), pin(5, 100, 200)]));
    assert!(!is_local(&[pin(1, 100, 200), pin(1, 100, 201)]));
}

#[test]
fn a_net_is_routable_only_when_ALL_FOUR_conditions_hold() {
    assert!(is_routable(false, false, false, false));
    for i in 0..4 {
        let f = |n: usize| n == i;
        assert!(!is_routable(f(0), f(1), f(2), f(3)), "condition {i} alone must reject the net");
    }
}

#[test]
fn the_grid_divides_the_DIE_area_by_the_cell_size_truncating() {
    // The real design: 200260 x 201600 on 5700-unit cells.
    // 200260 / 5700 = 35.13 -> 35, and 35 * 5700 = 199500 != 200260, so it is NOT regular.
    let g = init_grid(Rect::new(0, 0, 200_260, 201_600), 5_700, 10, -1);
    assert_eq!((g.x_grids, g.y_grids), (35, 35));
    assert!(!g.perfect_regular_x);
    assert!(!g.perfect_regular_y);
    assert_eq!(g.num_layers, 10, "-1 means every routing layer");
    assert_eq!(g.grid().area, Rect::new(0, 0, 200_260, 201_600));
}

#[test]
fn the_grid_it_derives_is_the_one_the_REFERENCE_used() {
    // The corpus records the grid the published stage built for this design; deriving it from the
    // die area and cell size must land on the same rectangle and cell size.
    let v: serde_json::Value = serde_json::from_str(CORPUS).unwrap();
    let a = v["grid"]["area"].as_array().unwrap();
    let n = |x: &serde_json::Value| x.as_i64().unwrap() as i32;
    let want = Rect::new(n(&a[0]), n(&a[1]), n(&a[2]), n(&a[3]));
    let tile = n(&v["grid"]["tile_size"]);

    let g = init_grid(want, tile, 10, -1);
    assert_eq!(g.grid().area, want);
    assert_eq!(g.grid().tile_size, tile);
}

#[test]
fn a_die_narrower_than_one_cell_still_gets_one() {
    let g = init_grid(Rect::new(0, 0, 100, 100), 5_700, 10, -1);
    assert_eq!((g.x_grids, g.y_grids), (1, 1), "clamped to at least one, never zero");
}

#[test]
fn an_exact_multiple_is_REGULAR() {
    let g = init_grid(Rect::new(0, 0, 11_400, 5_700), 5_700, 10, -1);
    assert_eq!((g.x_grids, g.y_grids), (2, 1));
    assert!(g.perfect_regular_x && g.perfect_regular_y);
}

#[test]
fn max_layer_caps_the_layer_count_and_minus_one_does_not() {
    assert_eq!(init_grid(Rect::new(0, 0, 100, 100), 10, 10, 4).num_layers, 4);
    assert_eq!(init_grid(Rect::new(0, 0, 100, 100), 10, 10, -1).num_layers, 10);
}

#[test]
fn the_sequencer_REPORTS_every_stage_it_does_not_run() {
    // 🔑 A stage that is simply not called produces no diff to chase. The setup run names its
    // gaps so they are visible from the outside rather than discovered later as a wrong answer.
    let r = init_fast_route(Rect::new(0, 0, 200_260, 201_600), 5_700, 10, -1);
    assert_eq!(r.grid.x_grids, 35);
    assert!(r.absent.contains(&AbsentStage::SetCapacities));
    assert!(r.absent.contains(&AbsentStage::InitNetlist));
    assert_eq!(r.absent.len(), 11, "eleven of the fourteen stages are still absent");
}
