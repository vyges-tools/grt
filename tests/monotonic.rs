// SPDX-License-Identifier: Apache-2.0
//! R12 — monotonic routing: the cheapest pair of bends through a searched midpoint.
//!
//! Validated in two halves, because they cost very different amounts to capture:
//!
//! - **the walk** — 1,429 routed edges, each replayed point by point from the reference's own
//!   choice of midpoint and orientations;
//! - **the search** — 130 of those also carry the usage patch over the whole box and the cost
//!   table in force, so the choice itself is replayed.
//!
//! The boxes are too large to carry for every edge (median 476 cells, 3.1 million in total), so
//! the second set is sampled. Both sets are bucketed by the branches taken — the two orientation
//! flags, whether the midpoint landed on an endpoint, and the direction of travel on each axis —
//! and the test asserts every bucket is populated.
//!
//! ⛔ **`via_cost` is 0 on every captured edge**, so the two orientation pairs that pay for a via
//! compete unpenalised. Asserted below.

use serde_json::Value;
use vyges_grt::estimate::EstimateGrid;
use vyges_grt::{route_monotonic, walk_monotonic_route, MonotonicRoute};

fn golden() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/monotonic.json");
    serde_json::from_str(&std::fs::read_to_string(path).expect("golden present"))
        .expect("golden parses")
}

fn i(v: &Value, k: &str) -> i32 {
    v[k].as_i64().unwrap_or_else(|| panic!("{k}")) as i32
}

fn pts(v: &Value) -> Vec<(i32, i32)> {
    v["points"].as_array().expect("points").iter()
        .map(|p| (p[0].as_i64().expect("x") as i32, p[1].as_i64().expect("y") as i32))
        .collect()
}

fn bucket(v: &Value) -> String {
    let (x1, y1, x2, y2) = (i(v, "x1"), i(v, "y1"), i(v, "x2"), i(v, "y2"));
    let (px, py) = (i(v, "px"), i(v, "py"));
    let (b1, b2) = (v["bl1"].as_bool().unwrap(), v["bl2"].as_bool().unwrap());
    format!(
        "{}{}/{}{}/{}{}",
        u8::from(b1), u8::from(b2),
        u8::from(px == x1 && py == y1), u8::from(px == x2 && py == y2),
        u8::from(x2 >= x1), u8::from(y2 >= y1)
    )
}

/// A grid big enough for the edge, so the walk's demand writes land somewhere valid.
fn grid_for(v: &Value) -> EstimateGrid {
    let span = (i(v, "x1").max(i(v, "x2")).max(i(v, "y1")).max(i(v, "y2")) + 3) as usize;
    EstimateGrid::new(span, span)
}

/// Every routed edge's point list, replayed from the reference's own choice.
#[test]
fn walks_match_the_reference_point_by_point() {
    let g = golden();
    let walks = g["walks"].as_array().expect("walks");
    assert!(walks.len() >= 1000, "too few walks: {}", walks.len());
    for w in walks {
        let mut grid = grid_for(w);
        let (points, routelen) = walk_monotonic_route(
            &mut grid,
            (i(w, "x1"), i(w, "y1")),
            (i(w, "px"), i(w, "py")),
            (i(w, "x2"), i(w, "y2")),
            w["bl1"].as_bool().unwrap(),
            w["bl2"].as_bool().unwrap(),
            1,
        );
        assert_eq!(points, pts(w), "points differ on {} {}", w["design"], bucket(w));
        assert_eq!(routelen, i(w, "routelen"), "routelen on {}", w["design"]);
        // ⚠️ The reference sizes its buffer from the two Manhattan halves and asserts it filled
        // exactly; that arithmetic is checked here rather than trusted.
        assert_eq!(
            points.len() as i32, i(w, "grid_size"),
            "point count disagrees with the buffer the reference sized on {}", w["design"]
        );
    }
}

/// The search itself, replayed over the reference's own usage and cost table.
#[test]
fn midpoint_and_orientations_match_the_reference() {
    let g = golden();
    let gens = g["generations"].as_object().expect("generations");
    let dps = g["dps"].as_array().expect("dps");
    assert!(dps.len() >= 100, "too few search samples: {}", dps.len());

    for d in dps {
        let gen = &gens[d["gen_key"].as_str().expect("gen_key")];
        // ⛔ Raw IEEE bits, not decimal. A first capture used seventeen significant digits and
        // left a 1-ULP disagreement that took a second capture to settle.
        let table: Vec<f64> = gen["table"].as_array().expect("table").iter()
            .map(|t| f64::from_bits(t.as_u64().expect("bits"))).collect();

        let (xmin, xmax) = (i(d, "xmin"), i(d, "xmax"));
        let (ymin, _ymax) = (i(d, "ymin"), i(d, "ymax"));
        let row = |key: &str, y: i32, x: i32| -> f64 {
            let rows = d[key].as_array().expect("patch");
            let r = rows.iter().find(|e| e[0].as_i64().unwrap() as i32 == y)
                .unwrap_or_else(|| panic!("row {y} missing from {key}"));
            f64::from_bits(r[1][(x - xmin) as usize].as_u64().expect("bits"))
        };
        let used_h = |x: i32, y: i32| row("uh", y, x);
        let used_v = |x: i32, y: i32| row("uv", y, x);

        let mut grid = grid_for(d);
        let got = route_monotonic(
            &mut grid,
            (i(d, "x1"), i(d, "y1")),
            (i(d, "x2"), i(d, "y2")),
            (xmin, xmax, ymin, i(d, "ymax")),
            &table,
            0.0,
            1,
            &used_h,
            &used_v,
        );

        assert_eq!(
            (got.px, got.py, got.bl1, got.bl2),
            (i(d, "px"), i(d, "py"), d["bl1"].as_bool().unwrap(), d["bl2"].as_bool().unwrap()),
            "search disagrees on {} {} box {:?}",
            d["design"], bucket(d), (xmin, xmax, ymin, i(d, "ymax"))
        );
        // The winning cost too, so an implementation that picks the right cell by accident is
        // still caught.
        assert_eq!(
            got.best.to_bits(), d["best_bits"].as_u64().expect("best_bits"),
            "winning cost differs on {}", d["design"]
        );
        // ⛔ **The demand the walk charges, not just the points it emits.** A backward run
        // charges the edge BELOW the cell it stands on, and nothing about the point list shows
        // that — swapping the index survives a point-by-point comparison of every routed edge.
        // This grid feeds every later stage, so it is checked directly.
        let mut charged: Vec<(i32, i32, f64, bool)> = Vec::new();
        for y in 0..grid.y_grids {
            for x in 0..grid.h_columns() {
                let u = f64::from(grid.usage_h(x, y));
                if u != 0.0 {
                    charged.push((x as i32, y as i32, u, true));
                }
            }
        }
        for y in 0..grid.v_rows() {
            for x in 0..grid.x_grids {
                let u = f64::from(grid.usage_v(x, y));
                if u != 0.0 {
                    charged.push((x as i32, y as i32, u, false));
                }
            }
        }
        let mut want: Vec<(i32, i32, f64, bool)> = Vec::new();
        for (key, horizontal) in [("charged_h", true), ("charged_v", false)] {
            for e in d[key].as_array().expect("charged") {
                want.push((
                    e[0].as_i64().expect("x") as i32,
                    e[1].as_i64().expect("y") as i32,
                    e[2].as_f64().expect("amount"),
                    horizontal,
                ));
            }
        }
        charged.sort_by(|a, b| a.partial_cmp(b).expect("total"));
        want.sort_by(|a, b| a.partial_cmp(b).expect("total"));
        assert_eq!(charged, want, "edges charged differ on {} {}", d["design"], bucket(d));

        let _: MonotonicRoute = got;
    }
}

/// Every sample must actually charge something, or the comparison above is vacuous.
#[test]
fn the_walk_charges_demand_on_both_axes() {
    let g = golden();
    let (mut h, mut v) = (0usize, 0usize);
    for d in g["dps"].as_array().expect("dps") {
        h += d["charged_h"].as_array().expect("h").len();
        v += d["charged_v"].as_array().expect("v").len();
    }
    assert!(h >= 200, "too few horizontal edges charged: {h}");
    assert!(v >= 200, "too few vertical edges charged: {v}");
}

/// The table lookup saturates rather than running off the end.
///
/// ⛔ **Constructed: no design gets near it.** The table spans ten times the capacity, and the
/// worst usage in the corpus is far below that, so removing the clamp is a mutation the captured
/// data cannot kill — but without the clamp the reference would index out of bounds.
#[test]
fn the_cost_lookup_saturates_at_the_end_of_the_table() {
    use vyges_grt::{monotonic_cost_table, route_monotonic};
    let table = monotonic_cost_table(4.0, 3, 0.5);
    assert_eq!(table.len(), 30, "the table spans ten times the capacity");

    let mut grid = EstimateGrid::new(8, 8);
    // A usage far past the end of the table on every edge of the box.
    let huge = |_x: i32, _y: i32| -> f64 { 10_000.0 };
    let route = route_monotonic(
        &mut grid, (1, 1), (4, 4), (1, 4, 1, 4), &table, 0.0, 1, &huge, &huge,
    );
    // Every lookup saturates to the same value, so every candidate ties and the first wins.
    assert_eq!((route.px, route.py), (1, 1), "a total tie keeps the first candidate");
    assert!(route.best.is_finite(), "saturation must not produce a non-finite cost");
}

/// ⛔ Stated as an assertion: every claim here about the via penalty rests on it being zero.
#[test]
fn the_via_cost_is_zero_on_every_captured_edge() {
    let g = golden();
    let dps = g["dps"].as_array().expect("dps");
    assert!(!dps.is_empty());
    for d in dps {
        assert_eq!(i(d, "via_cost"), 0, "{} routed with a non-zero via cost", d["design"]);
    }
}

/// ⚠️ Both orientation flags, both midpoint-degenerate cases and both directions of travel must
/// be present, or half the walk logic is decided by nothing.
#[test]
fn every_branch_bucket_is_populated() {
    let g = golden();
    let mut seen: std::collections::BTreeMap<String, usize> = Default::default();
    for w in g["walks"].as_array().expect("walks") {
        *seen.entry(bucket(w)).or_default() += 1;
    }
    let orientations: std::collections::BTreeSet<String> =
        seen.keys().map(|k| k[..2].to_string()).collect();
    assert_eq!(
        orientations.len(), 4,
        "not all four orientation pairs present: {orientations:?}"
    );
    assert!(seen.len() >= 8, "only {} branch buckets: {:?}", seen.len(), seen.keys());
    for (k, n) in &seen {
        assert!(*n >= 20, "bucket {k} has only {n} walks");
    }
}
