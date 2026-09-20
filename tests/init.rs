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

/// The extra designs, each added for a path the main one does not walk.
///
/// ⚠️ **`pin_access1` is why this list exists here too.** The end-to-end gate takes `is_local`
/// from the corpus as an INPUT, so mutating the rule cannot move it — only this test exercises
/// the rule, and on the main design alone it could not tell "compare position" from "compare
/// position and layer". `pin_access1` has a net with two pins at one point on different layers,
/// so here it can.
const EXTRA_CORPORA: &[(&str, &str)] = &[
    ("pin_access1", include_str!("../examples/grt_gate/cases/pin_access1.corpus.json")),
    ("pin_track_not_aligned", include_str!("../examples/grt_gate/cases/pin_track_not_aligned.corpus.json")),
    ("macro_obs_not_aligned", include_str!("../examples/grt_gate/cases/macro_obs_not_aligned.corpus.json")),
    ("modeling_instance_obs", include_str!("../examples/grt_gate/cases/modeling_instance_obs.corpus.json")),
];

#[test]
fn is_local_matches_the_REFERENCE_on_FIVE_designs() {
    // The extra designs first: one of them is the only reason this test can distinguish the rule
    // from a plausible wrong one.
    for (name, corpus) in EXTRA_CORPORA {
        let v: serde_json::Value = serde_json::from_str(corpus).unwrap();
        for net in v["nets"].as_array().unwrap() {
            let pins: Vec<Pin> = net["pins"].as_array().unwrap().iter().map(|p| Pin {
                connection_layer: p["connection_layer"].as_i64().unwrap() as i32,
                on_grid_x: p["on_grid_x"].as_i64().unwrap() as i32,
                on_grid_y: p["on_grid_y"].as_i64().unwrap() as i32,
            }).collect();
            assert_eq!(is_local(&pins), net["is_local"].as_bool().unwrap(),
                "{name}, net {}: is_local", net["name"].as_str().unwrap());
        }
    }

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
    assert!(r.absent.contains(&AbsentStage::FindNetsFromDatabase));
    assert_eq!(r.absent.len(), 12, "twelve stages are still absent; I7 and I13's RULES are done");
}

// ---- I13a: net discovery order, and the clock classification that drives it ---------------

/// The order the reference handed its nets to the router, on a design where the clock-first
/// partition is **distinguishable** from a plain name sort.
const NET_ORDER: &str = include_str!("../examples/grt_gate/net_order.ok");

fn net_order_golden() -> Vec<(bool, String)> {
    NET_ORDER
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let mut f = l.splitn(3, ' ');
            let clk = f.next().unwrap() == "1";
            let _oid = f.next().unwrap();
            (clk, f.next().unwrap().to_string())
        })
        .collect()
}

#[test]
fn net_order_matches_the_REFERENCE_on_a_design_that_DISTINGUISHES_the_partition() {
    // ⛔ The corpus was picked for what it makes different, not for being bigger: on this design
    // the full list is NOT name-sorted, so clock-first and plain-name-sort give different answers
    // and the test can actually fail. `gcd` has no clock nets at all and could not tell them
    // apart; `clock_route` has two, but they happen to sort first anyway.
    let golden = net_order_golden();
    assert_eq!(golden.len(), 348);
    assert_eq!(golden.iter().filter(|(c, _)| *c).count(), 2, "two non-leaf clock nets");

    let names: Vec<&str> = golden.iter().map(|(_, n)| n.as_str()).collect();
    let mut sorted = names.clone();
    sorted.sort_unstable();
    assert_ne!(names, sorted, "this corpus must DISTINGUISH the partition or it proves nothing");

    // Feed them REVERSED, to show the answer depends on the rules and not on input order.
    let mut input: Vec<DiscoveredNet> = golden.iter()
        .map(|(clk, name)| DiscoveredNet { name: name.clone(), is_non_leaf_clock: *clk })
        .collect();
    input.reverse();

    assert_eq!(order_nets(&input), names.iter().map(|s| s.to_string()).collect::<Vec<_>>());
}

#[test]
fn sorting_the_WHOLE_list_by_name_is_a_different_function() {
    // The mutation this guards against: dropping the partition and sorting everything by name.
    let golden = net_order_golden();
    let mut all_by_name: Vec<String> = golden.iter().map(|(_, n)| n.clone()).collect();
    all_by_name.sort();
    let input: Vec<DiscoveredNet> = golden.iter()
        .map(|(clk, name)| DiscoveredNet { name: name.clone(), is_non_leaf_clock: *clk })
        .collect();
    assert_ne!(order_nets(&input), all_by_name);
}

#[test]
fn a_clock_typed_net_reaching_a_clock_terminal_is_a_LEAF_and_sorts_with_the_rest() {
    // ⛔ "clock net" is not "clock-typed net". Reaching ANY clock terminal makes it a leaf.
    let reg_clk = ITermClockFacts { has_liberty_port: true, is_reg_clk: true, cell_is_pad: false };
    let plain = ITermClockFacts { has_liberty_port: true, is_reg_clk: false, cell_is_pad: false };

    assert!(is_non_leaf_clock(true, &[plain, plain]), "no clock terminal -> non-leaf");
    assert!(!is_non_leaf_clock(true, &[plain, reg_clk]), "one clock terminal is enough -> leaf");
    assert!(!is_non_leaf_clock(false, &[plain]), "not clock-typed -> not a clock net at all");
    assert!(is_non_leaf_clock(true, &[]), "no terminals -> vacuously non-leaf");
}

#[test]
fn a_PAD_terminal_counts_as_a_clock_terminal_even_without_a_register() {
    // ⚠️ The two conditions are an OR, and both are gated on the port existing.
    let pad = ITermClockFacts { has_liberty_port: true, is_reg_clk: false, cell_is_pad: true };
    assert!(is_clk_term(pad));

    let no_port = ITermClockFacts { has_liberty_port: false, is_reg_clk: true, cell_is_pad: true };
    assert!(!is_clk_term(no_port), "no liberty port is never a clock terminal, whatever the cell");
}

#[test]
fn the_name_comparison_is_BYTE_order_not_numeric() {
    // ⚠️ `net10` before `net9`. A natural-sort would reorder real designs.
    let nets: Vec<DiscoveredNet> = ["net9", "net10", "net1"].iter()
        .map(|n| DiscoveredNet { name: n.to_string(), is_non_leaf_clock: false })
        .collect();
    assert_eq!(order_nets(&nets), vec!["net1", "net10", "net9"]);
}

// ---- the snap every pin position funnels through ------------------------------------------

fn design_grid() -> CoreGrid {
    // The grid the reference built for the corpus design.
    init_grid(Rect::new(0, 0, 200_260, 201_600), 5_700, 10, -1)
}

#[test]
fn every_REFERENCE_pin_position_is_a_FIXED_POINT_of_the_snap() {
    // 🔑 The strongest check available without re-deriving pin geometry: the reference's on-grid
    // positions are, by construction, cell centres — so snapping one must return it unchanged.
    // A wrong divisor, a missing origin offset or a dropped half-tile all break this on the first
    // pin, across 1,536 real positions.
    let g = design_grid();
    let v: serde_json::Value = serde_json::from_str(CORPUS).unwrap();
    let mut checked = 0;
    for net in v["nets"].as_array().unwrap() {
        for p in net["pins"].as_array().unwrap() {
            let x = p["on_grid_x"].as_i64().unwrap() as i32;
            let y = p["on_grid_y"].as_i64().unwrap() as i32;
            assert_eq!(g.position_on_grid(x, y), (x, y),
                "({x}, {y}) is a reference on-grid position but not a fixed point of the snap");
            checked += 1;
        }
    }
    assert_eq!(checked, 1_536, "every pin position must have been checked");
}

#[test]
fn every_REFERENCE_segment_endpoint_is_a_FIXED_POINT_too() {
    // Segment ends are grid points as well, so the same invariant holds over a much larger set —
    // 3,770 segments, both ends each.
    let g = design_grid();
    let v: serde_json::Value = serde_json::from_str(CORPUS).unwrap();
    let mut checked = 0;
    for net in v["nets"].as_array().unwrap() {
        for s in net["segments"].as_array().unwrap() {
            for (kx, ky) in [("init_x", "init_y"), ("final_x", "final_y")] {
                let x = s[kx].as_i64().unwrap() as i32;
                let y = s[ky].as_i64().unwrap() as i32;
                assert_eq!(g.position_on_grid(x, y), (x, y), "segment endpoint ({x}, {y})");
                checked += 1;
            }
        }
    }
    assert_eq!(checked, 7_540, "3,770 segments, both ends");
}

#[test]
fn the_snap_lands_on_the_CENTRE_of_the_containing_cell() {
    // tile 100 over 0..1000: cell 0 spans [0,100) and its centre is 50.
    let g = init_grid(Rect::new(0, 0, 1_000, 1_000), 100, 10, -1);
    assert_eq!(g.position_on_grid(0, 0), (50, 50));
    assert_eq!(g.position_on_grid(99, 99), (50, 50));
    assert_eq!(g.position_on_grid(100, 100), (150, 150));
}

#[test]
fn the_die_ORIGIN_is_subtracted_before_dividing_and_added_back_after() {
    // ⚠️ A non-zero origin is where a missing offset shows up; with origin 0 the bug is invisible.
    let g = init_grid(Rect::new(1_000, 2_000, 2_000, 3_000), 100, 10, -1);
    assert_eq!(g.position_on_grid(1_000, 2_000), (1_050, 2_050));
    assert_eq!(g.position_on_grid(1_150, 2_150), (1_150, 2_150), "a centre maps to itself");
}

#[test]
fn a_point_in_the_PARTIAL_last_cell_is_pulled_back_into_the_last_full_one() {
    // ⛔ 1,050 wide on 100-unit cells: 10 full cells and a 50-unit remainder. A point at 1,020
    // divides to index 10, which is one past the end, so it is clamped to cell 9 — centre 950.
    let g = init_grid(Rect::new(0, 0, 1_050, 1_000), 100, 10, -1);
    assert_eq!(g.x_grids, 10);
    assert!(!g.perfect_regular_x);
    assert_eq!(g.position_on_grid(1_020, 500).0, 950);
}

#[test]
fn an_ODD_cell_size_puts_the_centre_half_a_unit_LOW() {
    // ⚠️ `tile_size / 2` truncates. Worth pinning: rounding it the other way moves every
    // position by one unit on an odd-sized grid.
    let g = init_grid(Rect::new(0, 0, 900, 900), 101, 10, -1);
    assert_eq!(g.position_on_grid(0, 0), (50, 50), "101/2 == 50, not 51");
}


#[test]
fn the_EXTRA_corpora_close_the_layer_blindness_gap() {
    // ⭐ Asserted, so the coverage cannot quietly regress if a corpus is regenerated. On the main
    // design this count is 0 and the rule is undecidable; here it is not.
    let mut with_two_layers_at_one_point = 0;
    for (_, corpus) in EXTRA_CORPORA {
        let v: serde_json::Value = serde_json::from_str(corpus).unwrap();
        for net in v["nets"].as_array().unwrap() {
            let mut by_xy: std::collections::BTreeMap<(i64, i64), std::collections::BTreeSet<i64>>
                = std::collections::BTreeMap::new();
            for p in net["pins"].as_array().unwrap() {
                by_xy.entry((p["on_grid_x"].as_i64().unwrap(), p["on_grid_y"].as_i64().unwrap()))
                    .or_default()
                    .insert(p["connection_layer"].as_i64().unwrap());
            }
            if by_xy.values().any(|l| l.len() > 1) { with_two_layers_at_one_point += 1; }
        }
    }
    assert_eq!(with_two_layers_at_one_point, 1,
        "exactly one net across the extra designs pins two layers at one point; it is what makes \
         `is_local` decidable");
}
