// SPDX-License-Identifier: Apache-2.0
//! The two rip-up gates, and the demand they give back.
//!
//! 583 one-bend decisions and 742 walked-route decisions from five designs. Each carries what the
//! gate read, its verdict, and — because a gate that says yes also **mutates** the grid and the
//! node statuses — the state afterwards.
//!
//! ⛔ **The gates read different grids and compare differently.** The one-bend gate reads the
//! **estimate** and asks `usage > capacity`, per-edge capacity summed over the net's layers. The
//! walked gate reads **committed** demand and asks `usage >= capacity - threshold`. Feeding
//! either rule to the other passes nothing.
//!
//! ⚠️ The critical-net arm of the walked gate is **live**: 142 of the captured decisions are torn
//! up for a detour rather than for congestion.

use serde_json::Value;
use vyges_grt::estimate::EstimateGrid;
use vyges_grt::{new_ripup_check, new_ripup_congested_l, CriticalCheck, RipupReason};

fn golden() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/ripup_gates.json");
    serde_json::from_str(&std::fs::read_to_string(path).expect("golden present"))
        .expect("golden parses")
}

fn i(v: &Value, k: &str) -> i32 {
    v[k].as_i64().unwrap_or_else(|| panic!("{k}")) as i32
}

/// The one-bend gate: verdict, status decrements, and the demand given back.
#[test]
fn the_one_bend_gate_matches_the_reference() {
    let g = golden();
    let rows = g["cl"].as_array().expect("cl");
    assert!(rows.len() >= 400, "too few one-bend decisions: {}", rows.len());
    let (mut yes, mut no, mut steiner) = (0usize, 0usize, 0usize);

    for c in rows {
        let (x1, y1, x2, y2) = (i(c, "x1"), i(c, "y1"), i(c, "x2"), i(c, "y2"));
        let x_first = c["x_first"].as_bool().expect("x_first");
        let (ymin, ymax) = (y1.min(y2), y1.max(y2));
        let x_check = if x_first { x2 } else { x1 };
        let y_check = if x_first { y1 } else { y2 };

        let v_run: Vec<(f64, i32)> = c["v_run"].as_array().expect("v_run").iter()
            .map(|e| (e[0].as_f64().expect("u"), e[1].as_i64().expect("c") as i32)).collect();
        let h_run: Vec<(f64, i32)> = c["h_run"].as_array().expect("h_run").iter()
            .map(|e| (e[0].as_f64().expect("u"), e[1].as_i64().expect("c") as i32)).collect();

        let span = (x1.max(x2).max(ymax) + 3) as usize;
        let mut grid = EstimateGrid::new(span, span);
        for (k, (u, _)) in v_run.iter().enumerate() {
            grid.update_v(x_check, ymin + k as i32, ymin + k as i32 + 1, *u);
        }
        for (k, (u, _)) in h_run.iter().enumerate() {
            grid.update_h(x1 + k as i32, x1 + k as i32 + 1, y_check, *u);
        }

        let capacity = |x: i32, y: i32, horizontal: bool| -> i32 {
            if horizontal {
                h_run[(x - x1) as usize].1
            } else {
                v_run[(y - ymin) as usize].1
            }
        };

        let (n1, n2) = (i(c, "n1") as usize, i(c, "n2") as usize);
        let n = n1.max(n2) + 1;
        let mut statuses = vec![0i16; n];
        statuses[n1] = i(c, "s1") as i16;
        statuses[n2] = i(c, "s2") as i16;

        let need = new_ripup_congested_l(
            &mut grid, &mut statuses, (n1, n2), (x1, y1), (x2, y2), x_first,
            i(c, "num_terminals") as usize, i(c, "edge_cost") as i8, &capacity,
        );

        assert_eq!(need, c["need"].as_bool().expect("need"), "verdict on {}", c["design"]);
        assert_eq!(statuses[n1], i(c, "s1_after") as i16, "n1 status on {}", c["design"]);
        assert_eq!(statuses[n2], i(c, "s2_after") as i16, "n2 status on {}", c["design"]);

        // ⛔ The demand given back, not just the verdict. A gate that returns the right answer and
        // gives back the wrong cells leaves demand nothing will ever remove.
        for (k, want) in c["v_run_after"].as_array().expect("after").iter().enumerate() {
            assert_eq!(
                grid.v(x_check as usize, (ymin + k as i32) as usize),
                want.as_f64().expect("f64"),
                "vertical demand at row {k} on {}", c["design"]
            );
        }
        for (k, want) in c["h_run_after"].as_array().expect("after").iter().enumerate() {
            assert_eq!(
                grid.h((x1 + k as i32) as usize, y_check as usize),
                want.as_f64().expect("f64"),
                "horizontal demand at column {k} on {}", c["design"]
            );
        }

        if need { yes += 1 } else { no += 1 }
        steiner += usize::from(n1 >= i(c, "num_terminals") as usize
            || n2 >= i(c, "num_terminals") as usize);
    }
    assert!(yes >= 100 && no >= 100, "verdicts are lopsided: {yes} yes, {no} no");
    // ⚠️ The horizontal mark is only given back to a Steiner node, so the corpus must contain some.
    assert!(steiner >= 20, "no Steiner endpoints: the conditional decrement is untested");
}

/// The walked gate: verdict, reason, and the demand given back.
#[test]
fn the_walked_gate_matches_the_reference() {
    let g = golden();
    let rows = g["ck"].as_array().expect("ck");
    assert!(rows.len() >= 500, "too few walked decisions: {}", rows.len());
    let (mut congested, mut critical, mut none) = (0usize, 0usize, 0usize);

    for c in rows {
        let points: Vec<(i32, i32)> = c["points"].as_array().expect("points").iter()
            .map(|p| (p[0].as_i64().expect("x") as i32, p[1].as_i64().expect("y") as i32))
            .collect();
        let routelen = i(c, "routelen") as usize;
        let used: Vec<(String, f64)> = c["used"].as_array().expect("used").iter()
            .map(|e| (e[0].as_str().expect("dir").to_string(), e[1].as_f64().expect("u")))
            .collect();

        // The captured usage is per STEP, so it is looked up by the edge each step crosses.
        let mut h_lookup = std::collections::HashMap::new();
        let mut v_lookup = std::collections::HashMap::new();
        for (k, (dir, u)) in used.iter().enumerate() {
            let ((ax, ay), (bx, by)) = (points[k], points[k + 1]);
            if dir == "V" {
                v_lookup.insert((ax, ay.min(by)), *u);
            } else if dir == "H" {
                h_lookup.insert((ax.min(bx), ay), *u);
            }
        }
        let used_h = |x: i32, y: i32| *h_lookup.get(&(x, y)).unwrap_or(&0.0);
        let used_v = |x: i32, y: i32| *v_lookup.get(&(x, y)).unwrap_or(&0.0);

        let span = points.iter().map(|p| p.0.max(p.1)).max().unwrap_or(0) + 3;
        let mut grid = EstimateGrid::new(span as usize, span as usize);
        for ((x, y), u) in h_lookup.iter() {
            grid.update_usage_h(*x, *y, *u);
        }
        for ((x, y), u) in v_lookup.iter() {
            grid.update_usage_v(*x, *y, *u);
        }

        let critical_check = CriticalCheck {
            enabled: i(c, "critical_enabled") != 0,
            last_routelen: i(c, "last_routelen") as usize,
            critical_slack: c["critical_slack"].as_f64().expect("cslack") as f32,
            slack: c["slack"].as_f64().expect("slack") as f32,
        };

        let got = new_ripup_check(
            &mut grid, &points, routelen, i(c, "threshold"),
            (i(c, "h_capacity"), i(c, "v_capacity")), i(c, "edge_cost") as i8,
            Some(critical_check), &used_h, &used_v,
        );

        let want = match c["reason"].as_str().expect("reason") {
            "none" => None,
            "congested" => Some(RipupReason::Congested),
            "critical" => Some(RipupReason::CriticalDetour),
            other => panic!("unknown reason {other}"),
        };
        assert_eq!(got, want, "verdict on {} routelen {routelen}", c["design"]);

        // The committed demand given back, step by step.
        for (k, (dir, _)) in used.iter().enumerate() {
            let ((ax, ay), (bx, by)) = (points[k], points[k + 1]);
            let after = c["used_after"].as_array().expect("after")[k][1]
                .as_f64().expect("f64");
            let got_now = f64::from(if dir == "V" {
                grid.usage_v(ax as usize, ay.min(by) as usize)
            } else {
                grid.usage_h(ax.min(bx) as usize, ay as usize)
            });
            assert_eq!(got_now, after, "demand at step {k} on {}", c["design"]);
        }

        match want {
            None => none += 1,
            Some(RipupReason::Congested) => congested += 1,
            Some(RipupReason::CriticalDetour) => critical += 1,
        }
    }
    assert!(congested >= 100, "too few congested: {congested}");
    assert!(none >= 100, "too few left alone: {none}");
    // ⚠️ Without this the whole critical-net arm could be missing and the gate still pass.
    assert!(critical >= 50, "the critical-net arm is barely reached: {critical}");
}

// ---------------------------------------------------------------------------
// The critical-net arm's four conditions.
//
// ⚠️ All four are constructed. Each was read out of the reference and then found to survive a
// deliberate mutation across all 742 captured decisions, and measuring says why: the corpus has
// **zero** cases that could separate any of them. It carries only three distinct slack
// thresholds, no sentinel slack at all, no zero previous length alongside the other conditions,
// and no detour ratio between one and two.
// ---------------------------------------------------------------------------

/// A straight vertical route of `steps` steps, with the same usage on each of its edges.
///
/// ⚠️ The detour ratio is the route's length over its previous length, so a one-step route can
/// never be a detour however short its predecessor — the shortest that can be is two steps
/// against one.
fn check(steps: usize, usage: f64, critical: CriticalCheck) -> (Option<RipupReason>, f64) {
    let points: Vec<(i32, i32)> = (0..=steps as i32).map(|k| (1, 1 + k)).collect();
    let zero = |_x: i32, _y: i32| 0.0;
    let used = move |_x: i32, _y: i32| usage;
    let mut grid = EstimateGrid::new(8 + steps, 8 + steps);
    for k in 0..steps as i32 {
        grid.update_usage_v(1, 1 + k, usage);
    }
    let got = new_ripup_check(
        &mut grid, &points, steps, 0, (10, 10), 1, Some(critical), &zero, &used,
    );
    (got, f64::from(grid.usage_v(1, 1)))
}

/// A net inside the critical band but with a route that has not grown.
fn quiet() -> CriticalCheck {
    CriticalCheck { enabled: true, last_routelen: 4, critical_slack: 5.0, slack: 1.0 }
}

/// ⛔ The detour must be at least **double**, and the boundary is inclusive.
#[test]
fn the_critical_detour_needs_the_route_to_have_doubled() {
    assert_eq!(check(2, 0.0, quiet()).0, None, "two steps against four is not a detour");

    let doubled = CriticalCheck { last_routelen: 1, ..quiet() };
    assert_eq!(
        check(2, 0.0, doubled).0,
        Some(RipupReason::CriticalDetour),
        "exactly double is already a detour"
    );

    // ⚠️ A ratio between one and two — which the corpus never produces — must NOT qualify.
    let just_under = CriticalCheck { last_routelen: 2, ..quiet() };
    assert_eq!(check(3, 0.0, just_under).0, None, "a ratio of 1.5 is not a detour");
    assert_eq!(check(2, 0.0, just_under).0, None, "a ratio of exactly one is not a detour");
}

/// ⛔ A net carrying the sentinel slack is excluded — it would otherwise look like the most
/// critical net in the design.
#[test]
fn the_sentinel_slack_is_not_treated_as_critical() {
    let base = CriticalCheck { last_routelen: 1, ..quiet() };
    let sentinel = CriticalCheck { slack: f32::MIN, ..base };
    assert_eq!(check(2, 0.0, sentinel).0, None, "the sentinel must not qualify");

    // The same net with a real, very negative slack does qualify.
    let real = CriticalCheck { slack: -1.0e30, ..base };
    assert_eq!(check(2, 0.0, real).0, Some(RipupReason::CriticalDetour));
}

/// ⚠️ Two of the four conditions only ask whether the feature is configured at all.
#[test]
fn the_critical_arm_is_skipped_when_it_is_not_configured() {
    let base = CriticalCheck { last_routelen: 1, ..quiet() };
    assert_eq!(check(2, 0.0, base).0, Some(RipupReason::CriticalDetour), "the baseline qualifies");

    assert_eq!(check(2, 0.0, CriticalCheck { enabled: false, ..base }).0, None, "disabled outright");
    assert_eq!(
        check(2, 0.0, CriticalCheck { last_routelen: 0, ..base }).0, None,
        "no previous length recorded — and dividing by it would be worse than skipping"
    );
    assert_eq!(
        check(2, 0.0, CriticalCheck { critical_slack: 0.0, ..base }).0, None,
        "no slack threshold for this round"
    );
    assert_eq!(
        check(2, 0.0, CriticalCheck { slack: 9.0, ..base }).0, None,
        "slack outside the band"
    );
}

/// ⛔ Congestion is decided first, and a congested route is reported as congested even when it
/// would also qualify as a detour.
#[test]
fn congestion_takes_precedence_over_the_detour_reason() {
    let both = CriticalCheck { last_routelen: 1, ..quiet() };
    // Usage 10 against a capacity of 10 with a threshold of 0 is congested, and the same net also
    // satisfies every critical condition.
    let (reason, after) = check(2, 10.0, both);
    assert_eq!(reason, Some(RipupReason::Congested), "congestion wins");
    assert_eq!(after, 9.0, "and the demand is given back exactly once");
}

/// Routing a shape and then ripping it up must leave the grid exactly as it was.
///
/// ⛔ **The Z undo has no captured witness.** Neither gate reaches it: the one-bend gate refuses
/// anything that is not a single bend, and the walked gate only ever sees routes already expanded
/// into points. So it is pinned by the property that actually matters — that the undo is the
/// inverse of the charge — rather than left untested.
///
/// ⚠️ This is a real check, not a tautology: the undo is written from the reference's `newRipup`
/// and the charge from its Z router, in different modules. They agree only if both are right.
#[test]
fn ripping_up_a_route_gives_back_exactly_what_routing_charged() {
    use vyges_grt::{newroute_z, RoutedShape, SpiralNode};
    use vyges_grt::new_ripup;

    for (p1, p2) in [((2, 2), (9, 7)), ((2, 7), (9, 2)), ((1, 1), (12, 11))] {
        let mut grid = EstimateGrid::new(16, 16);
        let before = grid.clone();
        let mut nodes: Vec<SpiralNode> = (0..2).map(|d| SpiralNode {
            x: 0, y: 0, top_layer: -1, bot_layer: 0, assigned: false,
            stack_alias: d, status: 0, h_id: 0, l_id: 0, edges: Vec::new(),
        }).collect();
        let zero = |_x: usize, _y: usize| -> u16 { 0 };

        let choice = newroute_z(
            &mut grid, &mut nodes, 0, 1, p1, p2, 1, 0.0, 0.0, 0.0, &zero, &zero,
        );
        assert_ne!(grid, before, "the Z route must have charged something");

        new_ripup(
            &mut grid, p1, p2,
            &RoutedShape::Z { hvh: choice.hvh, z_point: choice.z_point },
            1,
        );
        assert_eq!(grid, before, "the Z undo must be the exact inverse of the Z charge");
    }
}
