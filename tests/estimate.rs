// SPDX-License-Identifier: Apache-2.0
//! Stage R6 — the first demand estimate, checked against the reference's own grid.
//!
//! 🔑 **Captured BETWEEN the two passes**, so a mismatch is attributable to the estimate rather
//! than to anything the L-router did afterwards. That separation is the point of the dump.

#![allow(non_snake_case)]

use vyges_grt::*;

const GOLDEN: &str = include_str!("../examples/grt_gate/estimate.json");

fn replay() -> (EstimateGrid, Vec<(usize, usize, f64, f64)>) {
    let v: serde_json::Value = serde_json::from_str(GOLDEN).expect("golden parses");
    let g = v["grid"].as_array().unwrap();
    let (xg, yg) = (g[0].as_u64().unwrap() as usize, g[1].as_u64().unwrap() as usize);

    let mut grid = EstimateGrid::new(xg, yg);
    // ⚠️ Replayed in the order the reference visited them: the accumulation is additive, so the
    // order does not change the total — but replaying it faithfully means a future non-additive
    // rule would be caught rather than hidden.
    for s in v["segments"].as_array().unwrap() {
        let f: Vec<i64> = s.as_array().unwrap().iter().map(|x| x.as_i64().unwrap()).collect();
        estimate_one_seg(&mut grid, &Segment {
            x1: f[1] as i32, y1: f[2] as i32, x2: f[3] as i32, y2: f[4] as i32,
            edge_cost: f[5] as i8,
        });
    }

    let expected = v["estimate"].as_array().unwrap().iter().map(|e| {
        let e = e.as_array().unwrap();
        (e[0].as_u64().unwrap() as usize, e[1].as_u64().unwrap() as usize,
         e[2].as_f64().unwrap(), e[3].as_f64().unwrap())
    }).collect();
    (grid, expected)
}

#[test]
fn every_nonzero_cell_of_the_demand_grid_matches_the_REFERENCE() {
    let (grid, expected) = replay();
    for (x, y, h, v) in &expected {
        if *h != 0.0 { assert_eq!(grid.h(*x, *y), *h, "horizontal demand at ({x}, {y})"); }
        if *v != 0.0 { assert_eq!(grid.v(*x, *y), *v, "vertical demand at ({x}, {y})"); }
    }
    assert!(expected.len() > 900, "every non-zero cell must have been compared, got {}", expected.len());
}

#[test]
fn no_cell_the_reference_left_EMPTY_was_given_demand() {
    // ⚠️ The other half of the comparison. Checking only the non-zero cells would pass an
    // implementation that charged demand everywhere.
    let (grid, expected) = replay();
    let nonzero: std::collections::BTreeSet<(usize, usize)> =
        expected.iter().map(|(x, y, _, _)| (*x, *y)).collect();
    let mut stray = 0;
    // ⚠️ Iterated over the EDGE extents, not the cell grid: h has one fewer column and v one
    // fewer row, and the reference's accessors would read past the end rather than say so.
    for y in 0..grid.y_grids {
        for x in 0..grid.x_grids {
            let h = if x < grid.h_columns() { grid.h(x, y) } else { 0.0 };
            let v = if y < grid.v_rows() { grid.v(x, y) } else { 0.0 };
            if !nonzero.contains(&(x, y)) && (h != 0.0 || v != 0.0) {
                stray += 1;
            }
        }
    }
    assert_eq!(stray, 0, "{stray} cell(s) carry demand the reference left empty");
}

#[test]
fn the_grid_really_does_contain_HALF_values_so_the_diagonal_rule_is_exercised() {
    // 🔑 Vacuity guard. If every value were integral the halving would be untested and an
    // implementation charging the full cost four times would pass the cell comparison.
    let (_, expected) = replay();
    let halves = expected.iter()
        .filter(|(_, _, h, v)| h.fract() != 0.0 || v.fract() != 0.0)
        .count();
    assert!(halves > 100, "expected many half-charged cells, found {halves}");
}

#[test]
fn a_DIAGONAL_segment_charges_HALF_to_BOTH_columns_and_BOTH_rows() {
    // ⛔ The rule. Which way the L will bend is not known yet, so both candidate paths are
    // charged at half weight; charging one in full biases every later decision toward the other.
    let mut g = EstimateGrid::new(10, 10);
    estimate_one_seg(&mut g, &Segment { x1: 2, y1: 3, x2: 5, y2: 7, edge_cost: 2 });

    // both columns, over the y span, half each
    assert_eq!(g.v(2, 3), 1.0);
    assert_eq!(g.v(5, 3), 1.0);
    // both rows, over the x span, half each
    assert_eq!(g.h(2, 3), 1.0);
    assert_eq!(g.h(2, 7), 1.0);
    // and nothing outside the spans
    assert_eq!(g.v(2, 7), 0.0, "the vertical run is half-open at the top");
    assert_eq!(g.h(5, 3), 0.0, "the horizontal run is half-open at the right");
}

#[test]
fn a_STRAIGHT_segment_charges_its_FULL_cost_once() {
    let mut g = EstimateGrid::new(10, 10);
    estimate_one_seg(&mut g, &Segment { x1: 1, y1: 4, x2: 4, y2: 4, edge_cost: 3 });
    assert_eq!((g.h(1, 4), g.h(2, 4), g.h(3, 4)), (3.0, 3.0, 3.0));
    assert_eq!(g.h(4, 4), 0.0, "half-open");
    assert_eq!(g.v(1, 4), 0.0, "a horizontal segment charges no vertical demand");
}

#[test]
fn a_VERTICAL_segment_is_normalised_in_y_but_a_horizontal_one_is_NOT_in_x() {
    // ⚠️ Only y is normalised, because segments arrive ordered by x alone — see the R5/R7 rule.
    // A vertical segment given the other way round still charges the same cells.
    let mut a = EstimateGrid::new(10, 10);
    estimate_one_seg(&mut a, &Segment { x1: 3, y1: 6, x2: 3, y2: 2, edge_cost: 1 });
    let mut b = EstimateGrid::new(10, 10);
    estimate_one_seg(&mut b, &Segment { x1: 3, y1: 2, x2: 3, y2: 6, edge_cost: 1 });
    assert_eq!(a, b, "y is min/maxed, so either order charges the same run");

    // ⛔ but x is NOT: a horizontal segment with x1 > x2 charges NOTHING, because the interval
    // is half-open and empty. That is why the upstream x-ordering is load-bearing here.
    let mut c = EstimateGrid::new(10, 10);
    estimate_one_seg(&mut c, &Segment { x1: 7, y1: 4, x2: 2, y2: 4, edge_cost: 1 });
    assert_eq!(c, EstimateGrid::new(10, 10), "an x-reversed horizontal segment charges nothing");
}

#[test]
fn only_DIAGONAL_segments_need_L_routing() {
    assert!(needs_l_route(&Segment { x1: 1, y1: 1, x2: 2, y2: 2, edge_cost: 1 }));
    assert!(!needs_l_route(&Segment { x1: 1, y1: 1, x2: 5, y2: 1, edge_cost: 1 }));
    assert!(!needs_l_route(&Segment { x1: 1, y1: 1, x2: 1, y2: 5, edge_cost: 1 }));
}

#[test]
fn the_EDGE_arrays_have_different_shapes_and_neither_is_the_cell_grid() {
    // ⛔ Found the hard way: a first capture dumped the horizontal array at x = x_grids - 1,
    // where no edge exists, and the reference's unchecked accessor returned 4.7e170 from past
    // the end. Edges live BETWEEN cells: (x-1) x y horizontal, x x (y-1) vertical.
    let g = EstimateGrid::new(35, 35);
    assert_eq!(g.h_columns(), 34);
    assert_eq!(g.v_rows(), 34);
}

#[test]
#[should_panic(expected = "no horizontal edge")]
fn reading_past_the_last_horizontal_edge_PANICS_here_even_though_it_does_not_there() {
    // ⚠️ The reference reads out of bounds silently. This says so instead — the whole reason the
    // shape error was caught at all.
    let g = EstimateGrid::new(35, 35);
    let _ = g.h(34, 0);
}
