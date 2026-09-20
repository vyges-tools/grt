// SPDX-License-Identifier: Apache-2.0
//! R14 piece 4 — one step of the search: pricing an edge and relaxing the cell across it.
//!
//! 5,081 relaxations from three designs across **53 branch buckets** — both flags, all four
//! directions, first-visit against improvement, and with and without carried-over usage.
//!
//! ⛔ **The usage is carried as its two components**, this round's and the previous round's,
//! rather than blended. The carried-over component is non-zero on about a tenth of the captured
//! calls, so the blend weight is decided by the corpus rather than assumed away.
//!
//! ⚠️ Everything the step writes is compared: the new distance, **both** parent pairs, the flag
//! saying which pair holds the parent, and the two hyper flags. A step that computes the right
//! distance and records the wrong parent would route correctly and then backtrace wrongly.

use serde_json::Value;
use vyges_grt::{relax_adjacent, MazeSearch, RelaxInputs, BIG_INT};
use vyges_grt::mazecost::CostParams;

struct Relax {
    design: String,
    cur: (i32, i32),
    d: (i32, i32),
    add_via: bool,
    maybe_hyper: bool,
    l: i32,
    via: f64,
    dist_cur: f64,
    dist_adj: f64,
    dist_back: f64,
    u1: i32,
    last1: i32,
    u2: i32,
    last2: i32,
    tmp: f64,
    hyper_h_before: bool,
    hyper_v_before: bool,
    dist_adj_after: f64,
    parent_x1: i32,
    parent_y1: i32,
    parent_x3: i32,
    parent_y3: i32,
    hv: bool,
    hyper_h_after: bool,
    hyper_v_after: bool,
    params: CostParams,
    h_capacity: i32,
    v_capacity: i32,
}

fn relaxations() -> Vec<Relax> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/relax.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["relaxations"].as_array().expect("relaxations").iter().map(|r| {
        let i = |k: &str| r[k].as_i64().unwrap_or_else(|| panic!("{k}")) as i32;
        let b = |k: &str| f64::from_bits(r[k].as_u64().unwrap_or_else(|| panic!("{k}")));
        Relax {
            design: r["design"].as_str().expect("design").to_string(),
            cur: (i("cur_x"), i("cur_y")),
            d: (i("d_x"), i("d_y")),
            add_via: r["add_via"].as_bool().expect("via"),
            maybe_hyper: r["maybe_hyper"].as_bool().expect("hyper"),
            l: i("l"),
            via: f64::from(i("via")),
            dist_cur: b("dist_cur_bits"),
            dist_adj: b("dist_adj_bits"),
            dist_back: b("dist_back_bits"),
            u1: i("u1"), last1: i("last1"), u2: i("u2"), last2: i("last2"),
            tmp: b("tmp_bits"),
            hyper_h_before: i("hyper_h_before") != 0,
            hyper_v_before: i("hyper_v_before") != 0,
            dist_adj_after: b("dist_adj_after_bits"),
            parent_x1: i("parent_x1"), parent_y1: i("parent_y1"),
            parent_x3: i("parent_x3"), parent_y3: i("parent_y3"),
            hv: i("hv") != 0,
            hyper_h_after: i("hyper_h_after") != 0,
            hyper_v_after: i("hyper_v_after") != 0,
            params: CostParams {
                slope: i("slope"),
                logistic_coef: b("logis_bits"),
                cost_height: b("height_bits"),
            },
            h_capacity: i("h_capacity"),
            v_capacity: i("v_capacity"),
        }
    }).collect()
}

/// Drive one captured relaxation over a small grid seeded with its recorded state.
fn run(r: &Relax) -> MazeSearch {
    // The grid only has to hold the cells this step touches, offset so nothing is negative.
    let ox = r.cur.0.min(r.cur.0 + r.d.0).min(r.cur.0 - r.d.0) - 1;
    let oy = r.cur.1.min(r.cur.1 + r.d.1).min(r.cur.1 - r.d.1) - 1;
    let mut s = MazeSearch::new(8, 8);
    let cur = ((r.cur.0 - ox), (r.cur.1 - oy));
    let adj = (cur.0 + r.d.0, cur.1 + r.d.1);
    let back = (cur.0 - r.d.0, cur.1 - r.d.1);

    let ci = s.at(cur.0, cur.1);
    s.dist[ci] = r.dist_cur;
    let ai = s.at(adj.0, adj.1);
    s.dist[ai] = r.dist_adj;
    // ⛔ The hyper test reads the distance of the cell BEHIND the current one — a third
    // distance, distinct from the current and the adjacent. Seeding it with either of those
    // silently changes which routes look like a worthwhile detour.
    let bi = s.at(back.0, back.1);
    s.dist[bi] = r.dist_back;
    s.hyper_h[ci] = r.hyper_h_before;
    s.hyper_v[ci] = r.hyper_v_before;
    // A cell with a finite distance must already be in the heap, or the reference errors.
    if r.dist_adj < BIG_INT {
        s.heap.push(ai);
    }
    s.heap.push(ci);

    // The usage this step reads, replayed by position: the edge crossed and the one behind.
    let is_h = r.d.0 != 0;
    let p1 = (r.cur.0 - i32::from(r.d.0 == -1) - ox, r.cur.1 - i32::from(r.d.1 == -1) - oy);
    let p2 = (r.cur.0 - i32::from(r.d.0 == 1) - ox, r.cur.1 - i32::from(r.d.1 == 1) - oy);
    let used = |x: i32, y: i32| if (x, y) == p1 { r.u1 } else if (x, y) == p2 { r.u2 } else { 0 };
    let last = |x: i32, y: i32| if (x, y) == p1 { r.last1 } else if (x, y) == p2 { r.last2 } else { 0 };
    let zero = |_x: i32, _y: i32| 0;

    let inp = RelaxInputs {
        l: r.l,
        via: r.via,
        h_capacity: r.h_capacity,
        v_capacity: r.v_capacity,
        params: &r.params,
        used_h: if is_h { &used } else { &zero },
        used_v: if is_h { &zero } else { &used },
        last_h: if is_h { &last } else { &zero },
        last_v: if is_h { &zero } else { &last },
    };
    relax_adjacent(&mut s, cur, r.d, r.add_via, r.maybe_hyper, &inp).expect("relaxes");
    s
}

/// The hyper flags, which depend only on captured distances and usage.
#[test]
fn the_hyper_flags_match_the_reference() {
    let rows = relaxations();
    assert!(rows.len() >= 3000, "corpus too thin: {}", rows.len());
    let (mut set, mut both_flags) = (0usize, 0usize);

    for r in &rows {
        let s = run(r);
        let ox = r.cur.0.min(r.cur.0 + r.d.0).min(r.cur.0 - r.d.0) - 1;
        let oy = r.cur.1.min(r.cur.1 + r.d.1).min(r.cur.1 - r.d.1) - 1;
        let ci = s.at(r.cur.0 - ox, r.cur.1 - oy);
        assert_eq!(
            (s.hyper_h[ci], s.hyper_v[ci]),
            (r.hyper_h_after, r.hyper_v_after),
            "hyper flags on {} at {:?} d {:?}", r.design, r.cur, r.d
        );
        set += usize::from(r.hyper_h_after != r.hyper_h_before
            || r.hyper_v_after != r.hyper_v_before);
        both_flags += usize::from(r.add_via && r.maybe_hyper);
    }
    assert!(set >= 200, "the hyper flag is barely ever set: {set}");
    assert!(both_flags >= 1000, "the hyper branch is barely reached: {both_flags}");
}

/// The parent recorded, and which of the two parent grids holds it.
#[test]
fn the_parent_and_direction_flag_match_the_reference() {
    let rows = relaxations();
    let (mut horizontal, mut vertical) = (0usize, 0usize);
    for r in &rows {
        // Only meaningful where the step actually improved the adjacent cell.
        if r.dist_adj_after == r.dist_adj {
            continue;
        }
        let s = run(r);
        let ox = r.cur.0.min(r.cur.0 + r.d.0).min(r.cur.0 - r.d.0) - 1;
        let oy = r.cur.1.min(r.cur.1 + r.d.1).min(r.cur.1 - r.d.1) - 1;
        let ai = s.at(r.cur.0 + r.d.0 - ox, r.cur.1 + r.d.1 - oy);

        assert_eq!(s.hv[ai], r.hv, "direction flag on {} d {:?}", r.design, r.d);
        if r.hv {
            assert_eq!((s.parent_x1[ai] + ox, s.parent_y1[ai] + oy), r.cur,
                       "vertical parent on {}", r.design);
            vertical += 1;
        } else {
            assert_eq!((s.parent_x3[ai] + ox, s.parent_y3[ai] + oy), r.cur,
                       "horizontal parent on {}", r.design);
            horizontal += 1;
        }
    }
    // ⚠️ Both parent grids must be exercised, or one of them is never written in anger.
    assert!(horizontal >= 200, "too few horizontal moves: {horizontal}");
    assert!(vertical >= 200, "too few vertical moves: {vertical}");
}

/// ⛔ **The via guard inside the step is redundant, and only the CALL SEQUENCE shows it.**
///
/// Read alone, the step says "a turn is free at a source" — the via is added only where the
/// current distance is non-zero. Read from its caller, that case never arrives:
///
/// ```text
///   preX = curX; preY = curY;
///   if (d1[cur] != 0) { preX, preY = the recorded parent }
///   relaxAdjacent(cur, -1, 0, preY != curY, ...)
/// ```
///
/// `pre` is initialised **to `cur`** exactly when the distance is zero, so the flag the caller
/// passes is already false there. The guard inside is a second test of the same condition.
///
/// ⟹ Asserted as the absence it is: no captured relaxation asks for a via at a source. The guard
/// is still transcribed, because it is what the reference writes and a future caller could reach
/// it.
#[test]
fn no_relaxation_ever_asks_for_a_via_at_a_source() {
    let rows = relaxations();
    let sources: Vec<&Relax> = rows.iter().filter(|r| r.dist_cur == 0.0).collect();
    // Across the full trace before bucket capping: 1,855 relaxations start from a source and
    // **none** requests a via. The kept sample preserves a few dozen of them.
    assert!(
        sources.len() >= 40,
        "too few relaxations start from a source to say anything: {}", sources.len()
    );
    for r in &sources {
        assert!(
            !r.add_via,
            "a via was requested at a source on {} at {:?} — the caller's guard has changed",
            r.design, r.cur
        );
    }
}

/// The distance written to the adjacent cell, and whether it was written at all.
#[test]
fn the_relaxed_distance_matches_the_reference() {
    let rows = relaxations();
    let (mut improved, mut skipped) = (0usize, 0usize);
    for r in &rows {
        let s = run(r);
        let ox = r.cur.0.min(r.cur.0 + r.d.0).min(r.cur.0 - r.d.0) - 1;
        let oy = r.cur.1.min(r.cur.1 + r.d.1).min(r.cur.1 - r.d.1) - 1;
        let ai = s.at(r.cur.0 + r.d.0 - ox, r.cur.1 + r.d.1 - oy);
        assert_eq!(
            s.dist[ai].to_bits(), r.dist_adj_after.to_bits(),
            "relaxed distance on {} at {:?} d {:?}", r.design, r.cur, r.d
        );
        if r.dist_adj_after == r.dist_adj { skipped += 1 } else { improved += 1 }
    }
    // ⚠️ Both outcomes must be present: a step that always improved would not test the guard
    // that refuses a worse path.
    assert!(improved >= 500, "too few improvements: {improved}");
    assert!(skipped >= 500, "too few refusals: {skipped}");
}

/// ⛔ An equal-cost path is **refused**, not accepted.
///
/// The step returns early when the adjacent cell's existing cost is less than **or equal to** the
/// new one. Only one of the 6,752 captured relaxations offers an exactly equal cost, and on that
/// one the distance is unchanged either way — so the corpus cannot tell acceptance from refusal.
/// What differs is the **parent**: accepting would rewrite it and re-point the backtrace through
/// a different predecessor at no gain.
#[test]
fn an_equal_cost_path_does_not_replace_the_parent() {
    let mut s = MazeSearch::new(8, 8);
    let (cur, adj, other) = ((3, 3), (4, 3), (4, 2));

    // Reach `adj` from below first, then offer the same cost from the left.
    let ai = s.at(adj.0, adj.1);
    let oi = s.at(other.0, other.1);
    s.dist[oi] = 0.0;
    s.heap.push(oi);
    s.update_adjacent(other, adj, 7.0).expect("first arrival");
    assert_eq!(s.dist[ai], 7.0);
    assert_eq!((s.parent_x1[ai], s.parent_y1[ai]), (other.0, other.1));
    assert!(s.hv[ai], "reached by a vertical move");

    let before_heap = s.heap.clone();
    let ci = s.at(cur.0, cur.1);
    s.dist[ci] = 0.0;
    s.update_adjacent(cur, adj, 7.0).expect("equal cost");

    assert_eq!(s.dist[ai], 7.0, "the distance is unchanged either way");
    assert!(s.hv[ai], "an equal-cost arrival must NOT re-point the parent");
    assert_eq!((s.parent_x1[ai], s.parent_y1[ai]), (other.0, other.1));
    assert_eq!(s.parent_x3[ai], -1, "and must not record a horizontal parent");
    assert_eq!(s.heap, before_heap, "nor disturb the heap");

    // A strictly better cost does replace it.
    s.update_adjacent(cur, adj, 6.0).expect("better cost");
    assert_eq!(s.dist[ai], 6.0);
    assert!(!s.hv[ai], "now reached horizontally");
    assert_eq!((s.parent_x3[ai], s.parent_y3[ai]), (cur.0, cur.1));
}
