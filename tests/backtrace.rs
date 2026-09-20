// SPDX-License-Identifier: Apache-2.0
//! R14 piece 5 — walking the recorded parents back from the meeting point.
//!
//! 1,632 walks from four designs, 20,308 steps, **566 of them jumps**.
//!
//! The search stops the moment it pops a cell that already belongs to the destination subtree, so
//! the meeting point is already on that subtree and a single walk back to a source cell is the
//! whole new route. There is no second traversal.
//!
//! ⚠️ Each captured step carries the cell entered, the horizontal jump flag **at that cell**, the
//! vertical jump flag at the cell as it stood **after** the horizontal test, and where it landed.
//! The landing point is what pins where the second read happened — the two tests are sequentially
//! dependent, and assuming otherwise would place the second read at the wrong cell.

use serde_json::Value;
use vyges_grt::{backtrace, MazeSearch};

struct Step {
    cur: (i32, i32),
    hyper_h: bool,
    hyper_v: bool,
    mid_x: i32,
    tmp: (i32, i32),
    hv: bool,
    parent_y1: i32,
    parent_x3: i32,
}

struct Walk {
    design: String,
    cross: (i32, i32),
    steps: Vec<Step>,
    path: Vec<(i32, i32)>,
}

fn walks() -> Vec<Walk> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/backtrace.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let pair = |v: &Value| (v[0].as_i64().expect("x") as i32, v[1].as_i64().expect("y") as i32);
    v["walks"].as_array().expect("walks").iter().map(|w| Walk {
        design: w["design"].as_str().expect("design").to_string(),
        cross: pair(&w["cross"]),
        steps: w["steps"].as_array().expect("steps").iter().map(|s| Step {
            cur: pair(&s["cur"]),
            hyper_h: s["hyper_h"].as_i64().expect("hh") > 0,
            hyper_v: s["hyper_v"].as_i64().expect("hv") > 0,
            mid_x: s["mid_x"].as_i64().expect("mid") as i32,
            tmp: pair(&s["tmp"]),
            hv: s["hv"].as_i64().expect("hvf") != 0,
            parent_y1: s["parent_y1"].as_i64().expect("py1") as i32,
            parent_x3: s["parent_x3"].as_i64().expect("px3") as i32,
        }).collect(),
        path: w["path"].as_array().expect("path").iter().map(pair).collect(),
    }).collect()
}

/// Build a search state holding exactly what this walk reads, offset so nothing is negative.
fn state_for(w: &Walk) -> (MazeSearch, i32, i32) {
    let xs = w.steps.iter().flat_map(|s| [s.cur.0, s.tmp.0, s.mid_x])
        .chain(w.path.iter().map(|p| p.0)).chain([w.cross.0]);
    let ys = w.steps.iter().flat_map(|s| [s.cur.1, s.tmp.1])
        .chain(w.path.iter().map(|p| p.1)).chain([w.cross.1]);
    let (ox, oy) = (xs.min().unwrap_or(0) - 2, ys.min().unwrap_or(0) - 2);
    let span = 4 + w.steps.iter().flat_map(|s| [s.cur.0 - ox, s.cur.1 - oy, s.tmp.0 - ox, s.tmp.1 - oy, s.mid_x - ox])
        .chain(w.path.iter().flat_map(|p| [p.0 - ox, p.1 - oy]))
        .max().unwrap_or(0) as usize;
    let mut s = MazeSearch::new(span, span);
    // ⛔ The loop runs while the distance is non-zero, so every cell it enters must be non-zero
    // and the cell it finally reaches must be zero. The default here is zero, and only the
    // entered cells are raised.
    s.dist.fill(0.0);

    for st in &w.steps {
        let entry = s.at(st.cur.0 - ox, st.cur.1 - oy);
        s.dist[entry] = 1.0;
        s.hyper_h[entry] = st.hyper_h;
        // ⛔ The vertical flag is read at the cell as it stands AFTER the horizontal test, and
        // the trace records that column explicitly. Deriving it instead — from the landing point,
        // say — places the read at the wrong cell whenever a vertical jump fires, which is what
        // the first capture of this stage got wrong.
        let mid = s.at(st.mid_x - ox, st.cur.1 - oy);
        s.hyper_v[mid] = st.hyper_v;
        let here = s.at(st.tmp.0 - ox, st.tmp.1 - oy);
        s.hv[here] = st.hv;
        s.parent_y1[here] = st.parent_y1 - oy;
        s.parent_x3[here] = st.parent_x3 - ox;
    }
    (s, ox, oy)
}

#[test]
fn backtraced_paths_match_the_reference() {
    let walks = walks();
    assert!(walks.len() >= 1000, "corpus too thin: {}", walks.len());
    let mut steps = 0usize;

    for w in &walks {
        let (s, ox, oy) = state_for(w);
        let got = backtrace(&s, (w.cross.0 - ox, w.cross.1 - oy));
        let want: Vec<(i32, i32)> = w.path.iter().map(|p| (p.0 - ox, p.1 - oy)).collect();
        assert_eq!(
            got, want,
            "path on {} net edge {:?} from cross {:?}", w.design, w.path.first(), w.cross
        );
        steps += w.steps.len();
    }
    assert!(steps >= 10_000, "too few steps to be a gate: {steps}");
}

/// ⚠️ The jump path must actually be taken, or the whole hyper mechanism is untested.
#[test]
fn the_corpus_reaches_the_jump_path() {
    let walks = walks();
    let mut jumps = 0usize;
    for w in &walks {
        // ⚠️ Read off the recorded landing point rather than re-derived: a step that consulted
        // a parent lands on the cell it entered.
        for st in &w.steps {
            if st.tmp != st.cur {
                jumps += 1;
            }
        }
    }
    assert!(jumps >= 200, "only {jumps} jumps in the corpus — the mechanism is barely tested");
}

/// ⛔ The meeting point appears exactly once, at the end, and the walk ends on a source cell.
#[test]
fn the_path_ends_at_the_meeting_point_and_starts_on_the_source() {
    for w in &walks() {
        assert_eq!(
            *w.path.last().expect("non-empty"), w.cross,
            "the meeting point must be the last point on {}", w.design
        );
        assert_eq!(
            w.path.iter().filter(|p| **p == w.cross).count(), 1,
            "the meeting point must appear exactly once on {}", w.design
        );
        assert_eq!(
            w.path.len(), w.steps.len() + 1,
            "one point per step plus the meeting point, on {}", w.design
        );
    }
}

/// ⛔ Only one coordinate changes per step: every search move was axis-aligned.
///
/// ⚠️ A jump is the exception the rule needs — it moves the same axis twice, so the step is still
/// axis-aligned but of length two.
#[test]
fn every_step_moves_along_one_axis_only() {
    for w in &walks() {
        for pair in w.path.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            assert!(
                a.0 == b.0 || a.1 == b.1,
                "diagonal step {a:?} -> {b:?} on {}", w.design
            );
        }
    }
}

/// ⛔ The first step never jumps, whatever the flags say.
///
/// The reference guards the jump tests with a counter, and the previous position they compare
/// against is **uninitialised** on that first pass — left over from whichever edge was routed
/// before. The guard is what stops it being read.
///
/// ⚠️ **Constructed, because no captured walk reaches it**: not one of the 1,760 has a jump flag
/// set at its meeting point, so the corpus cannot tell the guard from its absence.
#[test]
fn the_first_step_never_jumps() {
    let mut s = MazeSearch::new(10, 10);
    s.dist.fill(0.0);
    let cross = (5, 5);
    let ci = s.at(cross.0, cross.1);
    s.dist[ci] = 1.0;
    // Both jump flags set at the meeting point — the first step must ignore them.
    s.hyper_h[ci] = true;
    s.hyper_v[ci] = true;
    // Its parent is one cell to the left.
    s.hv[ci] = false;
    s.parent_x3[ci] = 4;

    let path = backtrace(&s, cross);
    assert_eq!(
        path, vec![(4, 5), cross],
        "the first step must consult the parent, not reflect off an unset previous position"
    );
}

/// A jump is followed by a parent step, never by another jump.
///
/// ⚠️ After a jump the cell equals the previous position, so both movement tests are false on the
/// next pass. This is what makes the two tests mutually exclusive within a step as well.
#[test]
fn a_jump_is_always_followed_by_a_parent_step() {
    for w in &walks() {
        let mut prev_jumped = false;
        for st in &w.steps {
            let jumped = st.tmp != st.cur;
            assert!(
                !(prev_jumped && jumped),
                "two jumps in a row on {} at {:?}", w.design, st.cur
            );
            prev_jumped = jumped;
        }
    }
}

/// ⛔ No captured step jumps in both axes at once — and it cannot, by construction.
#[test]
fn no_step_jumps_in_both_axes() {
    let walks = walks();
    let mut jumps = 0usize;
    for w in &walks {
        for st in &w.steps {
            if st.tmp == st.cur {
                continue;
            }
            jumps += 1;
            assert!(
                (st.tmp.0 != st.cur.0) ^ (st.tmp.1 != st.cur.1),
                "a step jumped in both axes on {} at {:?}", w.design, st.cur
            );
        }
    }
    assert!(jumps >= 500, "too few jumps to make the claim: {jumps}");
}
