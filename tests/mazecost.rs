// SPDX-License-Identifier: Apache-2.0
//! The maze router's edge-cost tables.
//!
//! 60 distinct builds captured from three designs — every combination of slope, logistic
//! coefficient, cost height and the two capacities the congestion loop produced — with **every
//! entry** carried as raw IEEE bits.
//!
//! ⛔ **Carrying every entry is the point.** The curve is a logistic, and `exp` is the one place
//! where two correct implementations may legitimately differ in the last place. A sampled check
//! would not answer whether this crate reproduces the reference exactly; the full table does.

use serde_json::Value;
use vyges_grt::{cost_table, get_cost, CostParams};

struct Build {
    design: String,
    params: CostParams,
    h_capacity: i32,
    v_capacity: i32,
    h_table: Vec<u64>,
    v_table: Vec<u64>,
}

fn builds() -> Vec<Build> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/mazecost.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    v["builds"].as_array().expect("builds").iter().map(|b| {
        let bits = |k: &str| b[k].as_u64().unwrap_or_else(|| panic!("{k}"));
        let table = |k: &str| b[k].as_array().expect("table").iter()
            .map(|e| e.as_u64().expect("bits")).collect();
        Build {
            design: b["design"].as_str().expect("design").to_string(),
            params: CostParams {
                slope: b["slope"].as_i64().expect("slope") as i32,
                logistic_coef: f64::from_bits(bits("logis_bits")),
                cost_height: f64::from_bits(bits("height_bits")),
            },
            h_capacity: b["h_capacity"].as_i64().expect("hcap") as i32,
            v_capacity: b["v_capacity"].as_i64().expect("vcap") as i32,
            h_table: table("h_table"),
            v_table: table("v_table"),
        }
    }).collect()
}

/// Every entry of every captured table, bit for bit.
#[test]
fn the_cost_tables_match_the_reference_exactly() {
    let builds = builds();
    assert!(builds.len() >= 15, "too few builds: {}", builds.len());
    let mut entries = 0usize;

    for b in &builds {
        for (want, capacity, dir) in [
            (&b.h_table, b.h_capacity, "horizontal"),
            (&b.v_table, b.v_capacity, "vertical"),
        ] {
            let got = cost_table(capacity, &b.params);
            assert_eq!(
                got.len(), want.len(),
                "{dir} table spans the wrong range on {}", b.design
            );
            for (i, (g, w)) in got.iter().zip(want).enumerate() {
                assert_eq!(
                    g.to_bits(), *w,
                    "{dir} cost at usage {i} on {} (capacity {capacity})", b.design
                );
            }
            entries += want.len();
        }
    }
    assert!(entries >= 40_000, "corpus is thinner than it looks: {entries} entries");
}

/// ⛔ The two directions are priced against their **own** capacities.
///
/// ⚠️ Worth asserting separately: the monotonic stage earlier in the pipeline prices vertical
/// edges from the horizontal table, so the obvious mistake here is to carry that over. The
/// corpus makes it visible because the two capacities differ on every captured design.
#[test]
fn each_direction_is_priced_against_its_own_capacity() {
    let builds = builds();
    let mut differing = 0usize;
    for b in &builds {
        if b.h_capacity == b.v_capacity {
            continue;
        }
        differing += 1;
        let wrong = cost_table(b.h_capacity, &b.params);
        let right = cost_table(b.v_capacity, &b.params);
        assert_ne!(
            wrong.len(), right.len(),
            "the two tables must not coincide on {}", b.design
        );
        assert_eq!(right.iter().map(|c| c.to_bits()).collect::<Vec<_>>(), b.v_table);
    }
    assert!(differing >= 10, "no design has differing capacities: {differing}");
}

/// The ramp past capacity is added on top of the logistic, and meets it continuously.
#[test]
fn the_ramp_starts_at_capacity_and_adds_nothing_there() {
    let params = CostParams { slope: 20, logistic_coef: 0.5, cost_height: 8.0 };
    let capacity = 10;

    // At capacity the ramp term is zero, so the cost is exactly the logistic value.
    let logistic_only = params.cost_height / ((0.0f64).exp() + 1.0) + 1.0;
    assert_eq!(get_cost(capacity, capacity, &params), logistic_only);

    // One past capacity, the ramp contributes exactly one step.
    let step = params.cost_height / f64::from(params.slope);
    let at_next = get_cost(capacity + 1, capacity, &params);
    let logistic_at_next = params.cost_height / ((-params.logistic_coef).exp() + 1.0) + 1.0;
    assert_eq!(at_next, logistic_at_next + step);

    // ⚠️ A larger slope makes overflow CHEAPER, which reads backwards from the name.
    let gentler = CostParams { slope: 40, ..params };
    assert!(get_cost(capacity + 5, capacity, &gentler) < get_cost(capacity + 5, capacity, &params));
}
