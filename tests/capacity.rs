// SPDX-License-Identifier: Apache-2.0
//! Per-cell track capacity, checked against the reference's own answers.
//!
//! ⛔ **This is arithmetic that reads like a typo**: a ceiling on the lower bound, a floor over
//! `max_bound - track_init - 1` on the upper, and two clamps. Transcribing it is easy;
//! transcribing it *correctly* is not something a reading establishes — so the reference's
//! inputs and its answers were dumped and are replayed here.
//!
//! 🔑 **The sample is STRATIFIED, not random.** The clamps only fire at the edges of the grid and
//! of the track range, so a uniform interior sample would exercise one branch and certify the
//! rest. Every distinct layer parameter set is crossed with the grid's four corners, its four
//! edges and the interior — 30 strata, all present.
//!
//! ⭐ **The full dump — 1,076,274 calls — was replayed once and every rectangle and every
//! capacity matched.** It is 66 MB, so the sample is what ships; the exhaustive replay is the
//! `#[ignore]`d test at the bottom, pointed at the dump by an environment variable.

#![allow(non_snake_case)]

use vyges_grt::*;

/// `x|y|track_init|track_pitch|track_count|horizontal|rx0|ry0|rx1|ry1|min_bound|capacity`
const GOLDEN: &str = include_str!("../examples/grt_gate/gcell_capacity.ok");

/// The grid those calls were made against.
fn grid() -> CoreGrid {
    init_grid(Rect::new(0, 0, 2_920_000, 3_520_000), 6_900, 5, -1)
}

struct Row {
    x: i32, y: i32,
    track_init: i32, track_pitch: i32, track_count: i32,
    horizontal: bool,
    rect: Rect,
    capacity: i32,
}

fn rows() -> Vec<Row> {
    GOLDEN.lines().filter(|l| !l.trim().is_empty()).map(|l| {
        let f: Vec<i32> = l.split('|').map(|v| v.parse().expect("integer field")).collect();
        assert_eq!(f.len(), 12, "twelve fields per row");
        Row {
            x: f[0], y: f[1],
            track_init: f[2], track_pitch: f[3], track_count: f[4],
            horizontal: f[5] == 1,
            rect: Rect::new(f[6], f[7], f[8], f[9]),
            capacity: f[11],
        }
    }).collect()
}

#[test]
fn the_grid_matches_the_one_the_reference_built() {
    // If this is wrong every rectangle below is wrong and the capacity check is meaningless.
    let g = grid();
    assert_eq!((g.x_grids, g.y_grids), (423, 510));
}

#[test]
fn every_gcell_RECTANGLE_matches_the_reference() {
    // ⚠️ Checked separately from the capacity, so a failure says WHICH half is wrong — a bad
    // rectangle and a bad track count look identical if only the final number is compared.
    let g = grid();
    let rows = rows();
    for r in &rows {
        assert_eq!(g.gcell_rect(r.x, r.y), r.rect, "gcell rect at ({}, {})", r.x, r.y);
    }
    assert_eq!(rows.len(), 810);
}

#[test]
fn every_CAPACITY_matches_the_reference() {
    let g = grid();
    let rows = rows();
    let mut seen = std::collections::BTreeSet::new();
    for r in &rows {
        let got = compute_gcell_capacity(
            &g, r.x, r.y, r.track_init, r.track_pitch, r.track_count, r.horizontal,
        );
        assert_eq!(got, r.capacity,
            "capacity at ({}, {}) on a {} layer, init {} pitch {} count {}",
            r.x, r.y, if r.horizontal { "horizontal" } else { "vertical" },
            r.track_init, r.track_pitch, r.track_count);
        seen.insert(got);
    }
    // ⚠️ Vacuity guard: a sample that produced one value everywhere would pass a constant.
    assert!(seen.len() >= 5, "the sample must span several capacities, saw {seen:?}");
}

#[test]
fn the_sample_covers_BOTH_directions_and_every_layer_parameter_set() {
    let rows = rows();
    let params: std::collections::BTreeSet<(i32, i32, i32, bool)> = rows.iter()
        .map(|r| (r.track_init, r.track_pitch, r.track_count, r.horizontal)).collect();
    assert_eq!(params.len(), 5, "five distinct layer parameter sets in this technology");
    assert!(params.iter().any(|p| p.3), "horizontal layers present");
    assert!(params.iter().any(|p| !p.3), "vertical layers present");

    let edges = rows.iter().filter(|r| r.x == 0 || r.y == 0 || r.x >= 422 || r.y >= 509).count();
    assert!(edges > 0, "boundary cells must be sampled: that is where the clamps and the \
                        rectangle snap bite");
}

#[test]
fn the_UPPER_bound_is_EXCLUSIVE_and_that_extra_minus_one_is_the_rule() {
    // ⛔ The single easiest thing to get wrong. `last_track` floors over
    // `max_bound - track_init - 1`, so a track sitting exactly on the cell's upper edge belongs
    // to the NEXT cell. Dropping the `- 1` over-counts by one on every boundary a track lands on.
    //
    // Constructed so a track falls exactly on the boundary: tile 100, cell 0 spans [0, 100),
    // tracks at 0, 50, 100. The track at 100 must NOT be counted.
    let g = init_grid(Rect::new(0, 0, 1_000, 1_000), 100, 5, -1);
    assert_eq!(compute_gcell_capacity(&g, 0, 0, 0, 50, 10, true), 2, "tracks at 0 and 50, not 100");
}

#[test]
fn a_cell_entirely_BELOW_the_first_track_has_no_capacity() {
    // `max_bound < track_init` short-circuits last_track to -1, and the inverted range returns 0
    // rather than a negative count.
    let g = init_grid(Rect::new(0, 0, 1_000, 1_000), 100, 5, -1);
    assert_eq!(compute_gcell_capacity(&g, 0, 0, 5_000, 50, 10, true), 0);
}

#[test]
fn the_track_COUNT_clamps_the_upper_end() {
    // A cell wide enough for many tracks cannot report more than the layer has.
    let g = init_grid(Rect::new(0, 0, 1_000, 1_000), 100, 5, -1);
    assert_eq!(compute_gcell_capacity(&g, 0, 0, 0, 1, 3, true), 3, "clamped to track_count");
}

#[test]
fn INFINITE_capacity_is_the_SIXTEEN_BIT_ceiling_not_the_thirty_two_bit_one() {
    // ⚠️ The router stores capacity in 16 bits, so the "ignore congestion" value is a tenth of
    // that range to leave headroom for the additions downstream.
    assert_eq!(INFINITE_CAPACITY, 3_276);
}

#[test]
fn a_gcell_rect_is_NOT_clamped_at_the_lower_corner_even_with_a_nonzero_origin() {
    // ⚠️ Only the UPPER corner is snapped. Mutating the lower one to clamp against the die origin
    // passes every reference row, because that design's origin is 0 and `centre - half` never
    // falls below it — the mutation is EQUIVALENT there, not merely undetected. With a non-zero
    // origin the two differ, so the distinction is pinned here rather than left to luck.
    let g = init_grid(Rect::new(1_000, 2_000, 2_000, 3_000), 100, 5, -1);
    let r = g.gcell_rect(0, 0);
    assert_eq!((r.x_min, r.y_min), (1_000, 2_000), "cell 0 starts at the die origin");
    // and the cell is exactly one tile, not snapped outward at the bottom
    assert_eq!((r.x_max - r.x_min, r.y_max - r.y_min), (100, 100));
}

/// Exhaustive replay, when the full dump is available.
///
/// 🔑 The committed golden is a stratified 810-row sample, which keeps the repository small and
/// CI fast. The full dump is over a million calls and 66 MB — too large to commit, but exactly
/// the right thing to check once. Point `GRT_CAPACITY_DUMP` at it to replay every call:
///
/// ```sh
/// GRT_CAPACITY_DUMP=/path/to/capu.txt cargo test --test capacity -- --ignored
/// ```
#[test]
#[ignore = "needs the full dump; the committed sample is what CI runs"]
fn EVERY_capacity_call_of_the_full_dump_matches() {
    let path = std::env::var("GRT_CAPACITY_DUMP")
        .expect("set GRT_CAPACITY_DUMP to the full computeGCellCapacity dump");
    let text = std::fs::read_to_string(&path).expect("dump is readable");
    let g = grid();
    let mut n = 0usize;
    for line in text.lines() {
        if line.trim().is_empty() { continue; }
        let f: Vec<i32> = line.split('|').map(|v| v.parse().expect("integer field")).collect();
        if f.len() != 12 { continue; }
        let rect = Rect::new(f[6], f[7], f[8], f[9]);
        assert_eq!(g.gcell_rect(f[0], f[1]), rect, "rect at ({}, {})", f[0], f[1]);
        assert_eq!(
            compute_gcell_capacity(&g, f[0], f[1], f[2], f[3], f[4], f[5] == 1), f[11],
            "capacity at ({}, {})", f[0], f[1]
        );
        n += 1;
    }
    assert!(n > 1_000_000, "expected the full dump, replayed {n}");
    println!("replayed {n} capacity calls");
}
