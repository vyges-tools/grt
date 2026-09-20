// SPDX-License-Identifier: Apache-2.0
//! R10 `newrouteZ` — re-routing a long congested diagonal with two bends.
//!
//! 285 decisions from four designs, **89 horizontal-first and 196 vertical-first**, replayed end
//! to end: each carries the whole grid patch its cost loops read, so this drives the real cost
//! computation rather than checking an outcome against a captured intermediate.
//!
//! ⛔ **`via_cost_` is an `int` and is 0 whenever this stage runs**, so every via term — both
//! family base costs and both endpoint penalties — contributes nothing. Asserted below. That
//! whole branch is transcribed and unwitnessed, exactly as in the single-bend stage.
//!
//! ⚠️ The unreduced-usage asymmetry **is** witnessed, unlike the via bias: the reference asks for
//! plain rather than reduced usage on one branch alone, blockage differs from zero on half the
//! captured cells, and 135 of the 285 decisions take that branch.

use serde_json::Value;
use vyges_grt::estimate::EstimateGrid;
use vyges_grt::{newroute_z, SpiralNode, ZChoice};

struct Patch {
    /// Column -> values for rows `ymin..ymax`.
    rows: Vec<(i32, Vec<f64>)>,
    ymin: i32,
}

impl Patch {
    fn at(&self, x: i32, y: i32) -> f64 {
        let row = self.rows.iter().find(|(c, _)| *c == x)
            .unwrap_or_else(|| panic!("column {x} not captured"));
        row.1[(y - self.ymin) as usize]
    }
}

struct Case {
    design: String,
    x1: i32, y1: i32, x2: i32, y2: i32,
    n1a: usize, n2a: usize,
    s1: i16, s2: i16,
    h1: i32, l1: i32, h2: i32, l2: i32,
    edge_cost: i8,
    via_cost: f64, v_lb: f32, h_lb: f32,
    vr: Patch, vp: Patch, hr: Patch,
    hvh: bool, z_point: i32,
    s1a: i16, s2a: i16,
    h1a: i32, l1a: i32, h2a: i32, l2a: i32,
}

fn patch(v: &Value, key: &str, ymin: i32) -> Patch {
    let rows = v[key].as_array().unwrap_or_else(|| panic!("{key}")).iter()
        .map(|e| {
            let x = e[0].as_i64().expect("column") as i32;
            let vals = e[1].as_array().expect("values").iter()
                .map(|n| n.as_f64().expect("f64")).collect();
            (x, vals)
        }).collect();
    Patch { rows, ymin }
}

fn cases() -> Vec<Case> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/zroute.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["calls"].as_array().expect("calls").iter().map(|c| {
        let i = |k: &str| c[k].as_i64().unwrap_or_else(|| panic!("{k}"));
        let f = |k: &str| c[k].as_f64().unwrap_or_else(|| panic!("{k}"));
        let (y1, y2) = (i("y1") as i32, i("y2") as i32);
        let ymin = y1.min(y2);
        Case {
            design: c["design"].as_str().expect("design").to_string(),
            x1: i("x1") as i32, y1, x2: i("x2") as i32, y2,
            n1a: i("n1a") as usize, n2a: i("n2a") as usize,
            s1: i("s1") as i16, s2: i("s2") as i16,
            h1: i("h1") as i32, l1: i("l1") as i32,
            h2: i("h2") as i32, l2: i("l2") as i32,
            edge_cost: i("edge_cost") as i8,
            via_cost: f("via_cost"), v_lb: f("v_lb") as f32, h_lb: f("h_lb") as f32,
            vr: patch(c, "vr", ymin), vp: patch(c, "vp", ymin), hr: patch(c, "hr", ymin),
            hvh: c["hvh"].as_bool().expect("hvh"), z_point: i("z_point") as i32,
            s1a: i("s1a") as i16, s2a: i("s2a") as i16,
            h1a: i("h1a") as i32, l1a: i("l1a") as i32,
            h2a: i("h2a") as i32, l2a: i("l2a") as i32,
        }
    }).collect()
}

/// ⛔ Stated as an assertion: if any design ever ran this stage with a non-zero via cost, every
/// claim about the via terms here would be wrong and this would say so.
#[test]
fn the_via_cost_is_zero_on_every_captured_decision() {
    let cases = cases();
    assert!(!cases.is_empty());
    for c in &cases {
        assert_eq!(c.via_cost, 0.0, "{} ran newrouteZ with a non-zero via cost", c.design);
    }
}

/// Every decision replayed end to end, over the reference's own grid state.
#[test]
fn z_decisions_match_the_reference() {
    let cases = cases();
    let (mut hvh_n, mut vhv_n, mut reversed_y, mut blockage) = (0usize, 0usize, 0usize, 0usize);

    for c in &cases {
        let ymin = c.y1.min(c.y2);
        let ymax = c.y1.max(c.y2);
        let span = (c.x1.max(c.x2).max(ymax) + 2) as usize;
        let mut grid = EstimateGrid::new(span, span);

        // The vertical grid carries the PLAIN usage, with the blockage supplied separately, so
        // that the one branch asking for unreduced usage sees a different number from the rest.
        for (x, vals) in &c.vp.rows {
            for (j, val) in vals.iter().enumerate() {
                grid.update_v(*x, ymin + j as i32, ymin + j as i32 + 1, *val);
            }
        }
        // Only the reduced form is ever read horizontally, so it goes in whole.
        for (x, vals) in &c.hr.rows {
            for (j, val) in vals.iter().enumerate() {
                grid.update_h(*x, *x + 1, ymin + j as i32, *val);
            }
        }

        // The blockage is exactly what the reduced form adds over the plain one; measured
        // non-negative and integral on every captured cell.
        let red_v = |x: usize, y: usize| -> u16 {
            (c.vr.at(x as i32, y as i32) - c.vp.at(x as i32, y as i32)) as u16
        };
        let red_h = |_x: usize, _y: usize| -> u16 { 0 };

        let n = c.n1a.max(c.n2a) + 1;
        let mut nodes: Vec<SpiralNode> = (0..n).map(|d| SpiralNode {
            x: 0, y: 0, top_layer: -1, bot_layer: 0, assigned: false,
            stack_alias: d, status: 0, h_id: 0, l_id: 0, edges: Vec::new(),
        }).collect();
        nodes[c.n1a].status = c.s1;
        nodes[c.n2a].status = c.s2;
        nodes[c.n1a].h_id = c.h1;
        nodes[c.n1a].l_id = c.l1;
        nodes[c.n2a].h_id = c.h2;
        nodes[c.n2a].l_id = c.l2;

        let got = newroute_z(
            &mut grid, &mut nodes, c.n1a, c.n2a,
            (c.x1, c.y1), (c.x2, c.y2),
            c.edge_cost, c.via_cost, c.v_lb, c.h_lb, &red_v, &red_h,
        );

        assert_eq!(
            got, ZChoice { hvh: c.hvh, z_point: c.z_point },
            "Z choice on {} edge ({},{})-({},{})", c.design, c.x1, c.y1, c.x2, c.y2
        );
        assert_eq!(nodes[c.n1a].status, c.s1a, "n1a status on {}", c.design);
        assert_eq!(nodes[c.n2a].status, c.s2a, "n2a status on {}", c.design);
        assert_eq!(nodes[c.n1a].h_id, c.h1a, "n1a hID on {}", c.design);
        assert_eq!(nodes[c.n1a].l_id, c.l1a, "n1a lID on {}", c.design);
        assert_eq!(nodes[c.n2a].h_id, c.h2a, "n2a hID on {}", c.design);
        assert_eq!(nodes[c.n2a].l_id, c.l2a, "n2a lID on {}", c.design);

        if c.hvh { hvh_n += 1 } else { vhv_n += 1 }
        reversed_y += usize::from(c.y1 > c.y2);
        for (x, vals) in &c.vr.rows {
            for (j, val) in vals.iter().enumerate() {
                blockage += usize::from(*val != c.vp.at(*x, ymin + j as i32));
            }
        }
    }

    // ⚠️ Both families, both y orderings, and real blockage — otherwise the asymmetric branches
    // are decided by nothing.
    assert!(hvh_n >= 50, "too few horizontal-first decisions: {hvh_n}");
    assert!(vhv_n >= 50, "too few vertical-first decisions: {vhv_n}");
    assert!(reversed_y >= 50, "the unreduced-usage branch is barely reached: {reversed_y}");
    assert!(blockage >= 1000, "blockage never differs from zero: {blockage}");
}
