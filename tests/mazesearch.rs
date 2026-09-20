// SPDX-License-Identifier: Apache-2.0
//! R14 — the search loop, end to end.
//!
//! 331 complete searches from four designs, **27,672 expansions**. Each drives the heap setup's
//! output, the relaxation, the heap primitives and the stopping rule together, and is checked on
//! the meeting point it arrives at.
//!
//! ⛔ **This is the strongest statement available about the search.** A meeting point that matches
//! after a hundred expansions depends on every pop order, every parent recorded, and every cost
//! looked up along the way — a single relaxation matching says far less.
//!
//! ⚠️ The usage patch extends one cell below the region on each axis, because a step in the
//! negative direction prices the edge *behind* the cell it leaves.

use serde_json::Value;
use vyges_grt::mazecost::CostParams;
use vyges_grt::{maze_search, MazeSearch, RelaxInputs};

struct Search {
    design: String,
    region: (i32, i32, i32, i32),
    l: i32,
    via: f64,
    h_capacity: i32,
    v_capacity: i32,
    params: CostParams,
    x_range: usize,
    src: Vec<(i32, i32)>,
    dest: Vec<(i32, i32)>,
    /// Row -> per-column `[h_used, h_last, v_used, v_last]`, starting one cell below the region.
    usage: Vec<(i32, Vec<[i32; 4]>)>,
    /// Row -> per-column `[horizontal, vertical]` detour flags left by the search.
    hyper: Vec<(i32, Vec<[i32; 2]>)>,
    cross: (i32, i32),
    pops: usize,
}

fn searches() -> Vec<Search> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/mazesearch.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let pts = |v: &Value| -> Vec<(i32, i32)> {
        v.as_array().expect("points").iter()
            .map(|p| (p[0].as_i64().expect("x") as i32, p[1].as_i64().expect("y") as i32))
            .collect()
    };
    v["searches"].as_array().expect("searches").iter().map(|s| {
        let i = |k: &str| s[k].as_i64().unwrap_or_else(|| panic!("{k}")) as i32;
        let r = s["region"].as_array().expect("region");
        let n = |k: usize| r[k].as_i64().expect("bound") as i32;
        Search {
            design: s["design"].as_str().expect("design").to_string(),
            region: (n(0), n(1), n(2), n(3)),
            l: i("l"),
            via: f64::from(i("via")),
            h_capacity: i("h_capacity"),
            v_capacity: i("v_capacity"),
            params: CostParams {
                slope: i("slope"),
                logistic_coef: f64::from_bits(s["logis_bits"].as_u64().expect("logis")),
                cost_height: f64::from_bits(s["height_bits"].as_u64().expect("height")),
            },
            x_range: s["x_range"].as_u64().expect("xr") as usize,
            src: pts(&s["src"]),
            dest: pts(&s["dest"]),
            usage: s["usage"].as_array().expect("usage").iter().map(|row| {
                let y = row[0].as_i64().expect("y") as i32;
                let cells = row[1].as_array().expect("cells").iter().map(|c| {
                    let a = c.as_array().expect("quad");
                    [
                        a[0].as_i64().expect("hu") as i32,
                        a[1].as_i64().expect("hl") as i32,
                        a[2].as_i64().expect("vu") as i32,
                        a[3].as_i64().expect("vl") as i32,
                    ]
                }).collect();
                (y, cells)
            }).collect(),
            hyper: s["hyper"].as_array().expect("hyper").iter().map(|row| {
                let y = row[0].as_i64().expect("y") as i32;
                let cells = row[1].as_array().expect("cells").iter().map(|c| {
                    let a = c.as_array().expect("pair");
                    [a[0].as_i64().expect("h") as i32, a[1].as_i64().expect("v") as i32]
                }).collect();
                (y, cells)
            }).collect(),
            cross: (s["cross"][0].as_i64().expect("x") as i32,
                    s["cross"][1].as_i64().expect("y") as i32),
            pops: s["pops"].as_u64().expect("pops") as usize,
        }
    }).collect()
}

#[test]
fn searches_reach_the_same_meeting_point() {
    let all = searches();
    assert!(all.len() >= 250, "corpus too thin: {}", all.len());
    let (mut expansions, mut multi) = (0usize, 0usize);

    for s in &all {
        let (x1, x2, _y1, y2) = s.region;
        // ⛔ The row stride is the distance grid's allocated width, which the flat cell index is
        // decomposed with — not the design's grid width.
        let mut st = MazeSearch::new(s.x_range, (y2 + 2) as usize);
        st.dist.fill(vyges_grt::BIG_INT);
        for &(x, y) in s.src.iter() {
            let i = st.at(x, y);
            st.dist[i] = 0.0;
            st.heap.push(i);
        }

        // The patch starts one cell below the region on each axis.
        let first_col = (x1 - 1).max(0);
        let cell = |x: i32, y: i32, k: usize| -> i32 {
            let Some((_, row)) = s.usage.iter().find(|(ry, _)| *ry == y) else { return 0 };
            row.get((x - first_col) as usize).map_or(0, |c| c[k])
        };
        let used_h = |x: i32, y: i32| cell(x, y, 0);
        let last_h = |x: i32, y: i32| cell(x, y, 1);
        let used_v = |x: i32, y: i32| cell(x, y, 2);
        let last_v = |x: i32, y: i32| cell(x, y, 3);

        let inp = RelaxInputs {
            l: s.l,
            via: s.via,
            h_capacity: s.h_capacity,
            v_capacity: s.v_capacity,
            params: &s.params,
            used_h: &used_h,
            used_v: &used_v,
            last_h: &last_h,
            last_v: &last_v,
        };

        let got = maze_search(&mut st, &s.dest, s.region, &inp).expect("the search completes");
        assert_eq!(
            got, s.cross,
            "meeting point on {} after {} expansions, region {:?}",
            s.design, s.pops, s.region
        );
        // ⛔ The detour flags the search left behind. Without these the guard deciding whether
        // a detour is even considered is invisible: it changes no distance and no meeting point,
        // only which cells are marked — and those decide the backtrace later.
        for (y, row) in &s.hyper {
            for (k, want) in row.iter().enumerate() {
                let x = x1 + k as i32;
                let i = st.at(x, *y);
                assert_eq!(
                    (i32::from(st.hyper_h[i]), i32::from(st.hyper_v[i])),
                    (want[0], want[1]),
                    "detour flags at ({x},{y}) on {} after {} expansions", s.design, s.pops
                );
            }
        }

        expansions += s.pops;
        multi += usize::from(s.src.len() > 1 || s.dest.len() > 1);
    }
    assert!(expansions >= 20_000, "too few expansions to be a gate: {expansions}");
    // ⚠️ Multi-source or multi-destination searches must be present, or the whole point of the
    // two frontiers is untested.
    assert!(multi >= 100, "too few multi-frontier searches: {multi}");
}

/// ⚠️ The detour flags must actually be set somewhere, or comparing them proves nothing.
#[test]
fn the_searches_actually_mark_detours() {
    let all = searches();
    let marked: usize = all.iter()
        .flat_map(|s| s.hyper.iter())
        .flat_map(|(_, row)| row.iter())
        .filter(|c| c[0] != 0 || c[1] != 0)
        .count();
    assert!(marked >= 1000, "only {marked} detour flags set across the corpus");
}

/// A search whose meeting point is already on the source frontier makes no expansion at all.
#[test]
fn a_search_that_starts_finished_expands_nothing() {
    let all = searches();
    // ⚠️ Present in the corpus only if a source seed is also a destination seed, which the heap
    // setup makes impossible — so this is asserted as the absence it is.
    for s in &all {
        let src: std::collections::HashSet<_> = s.src.iter().collect();
        assert!(
            !s.dest.iter().any(|d| src.contains(d)),
            "the two frontiers overlap on {}, so the search was over before it began", s.design
        );
        assert!(s.pops > 0, "a search with no expansion on {}", s.design);
    }
}
