// SPDX-License-Identifier: Apache-2.0
//! Ia — the first setup stages: `configFastRoute` (I3) and `reportLayerSettings` (I5).
//!
//! Golden `setup_config.json`: every run of the corpus, both cost modes — each `initFastRoute`
//! call's inputs and the router's critical-nets percentage after I3, and the run's OWN log lines
//! GRT-0300 and GRT-0020..0023, which the replay must reproduce line for line, in order.

use serde_json::Value;
use vyges_grt::{config_fast_route, report_layer_settings, SetupOptions};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn opts(c: &Value) -> SetupOptions {
    SetupOptions {
        verbose: int(&c["verbose"]) == 1,
        has_liberty: int(&c["liberty"]) == 1,
        critical_nets_percentage: c["crit_in"].as_f64().expect("f") as f32,
        adjustment: c["adjustment"].as_f64().expect("f") as f32,
        grid_origin: (int(&c["origin"][0]) as i32, int(&c["origin"][1]) as i32),
        min_layer_name: c["names"][0].as_str().expect("name").to_string(),
        max_layer_name: c["names"][1].as_str().expect("name").to_string(),
    }
}

/// Every run: I3's critical-nets percentage per call, and the log lines of all its calls.
#[test]
fn the_setup_config_and_layer_report_match_the_reference() {
    let g = read(&format!("{}/examples/grt_gate/setup_config.json", env!("CARGO_MANIFEST_DIR")));
    let (mut runs, mut calls, mut warned, mut verbose, mut quiet) = (0, 0, 0, 0, 0);
    for r in g["runs"].as_array().expect("runs") {
        let who = r["design"].as_str().expect("design");
        // ⚠️ The CUGR path prints GRT-0020..0023 without entering initFastRoute.
        if r["cugr"].as_bool().expect("flag") {
            continue;
        }
        let mut log = Vec::new();
        for c in r["calls"].as_array().expect("calls") {
            let o = opts(c);
            let cfg = config_fast_route(&o, &mut log);
            assert_eq!(
                cfg.critical_nets_percentage.to_bits(),
                (c["crit_out"].as_f64().expect("f") as f32).to_bits(),
                "{who}: critical-nets percentage after configFastRoute"
            );
            report_layer_settings(&o, &mut log);
            calls += 1;
            warned += !o.has_liberty as usize;
            verbose += o.verbose as usize;
        }
        // ⚠️ `tee -quiet` around global_route: the run's grt output never reached its log.
        if r["quiet"].as_bool().expect("flag") {
            quiet += 1;
            continue;
        }
        let want: Vec<&str> = r["log"].as_array().expect("log").iter().map(|l| l.as_str().expect("line")).collect();
        assert_eq!(log, want, "{who}: GRT-0300 / GRT-0020..0023 lines");
        runs += 1;
    }
    // ⛔ Not vacuous: both branches of each stage are exercised.
    assert!(runs >= 100 && calls >= runs, "runs {runs}, calls {calls}");
    // Tripwire: the unobservable runs stay the exception (6: three `tee -quiet` tests, two modes).
    assert!(quiet <= 6, "{quiet} runs have no grt output in their log");
    assert!(warned > 0 && warned < calls, "GRT-0300 on {warned} of {calls} calls");
    assert!(verbose > 0 && verbose < calls, "verbose on {verbose} of {calls} calls");
}

/// ⛔ `int(adjustment_ * 100)` is a FLOAT product, truncated: 0.29f × 100 rounds to 29.0f, where
/// the double product 28.999999165… truncates to 28. The corpus's adjustments (0, ±0.5, 0.3, 0.7,
/// 0.8) all land the same either way.
#[test]
fn the_global_adjustment_percentage_is_a_float_product() {
    let o = SetupOptions {
        verbose: true,
        has_liberty: true,
        critical_nets_percentage: 10.0,
        adjustment: 0.29,
        grid_origin: (0, 0),
        min_layer_name: "m1".into(),
        max_layer_name: "m2".into(),
    };
    let mut log = Vec::new();
    report_layer_settings(&o, &mut log);
    assert_eq!(log[2], "[INFO GRT-0022] Global adjustment: 29%");
    assert_eq!((0.29f32 as f64 * 100.0) as i32, 28, "the double reading differs");
}

/// ⛔ With no Liberty, GRT-0300 fires EVERY call, even when the percentage is already 0 — it does
/// not check first (the CUGR path's GRT-0309 does).
#[test]
fn grt_0300_fires_even_when_the_percentage_is_already_zero() {
    let o = SetupOptions {
        verbose: false,
        has_liberty: false,
        critical_nets_percentage: 0.0,
        adjustment: 0.0,
        grid_origin: (0, 0),
        min_layer_name: String::new(),
        max_layer_name: String::new(),
    };
    let mut log = Vec::new();
    assert_eq!(config_fast_route(&o, &mut log).critical_nets_percentage, 0.0);
    assert_eq!(log.len(), 1);
}

/// ⛔ GRT-0022 TRUNCATES toward zero (0.257 → 25%, −0.257 → −25%), and GRT-0023 prints the origin
/// as `(x, y)`. Every captured origin is (0, 0) and every captured percentage is whole.
#[test]
fn the_layer_report_truncates_and_prints_x_then_y() {
    let mut o = SetupOptions {
        verbose: true,
        has_liberty: true,
        critical_nets_percentage: 10.0,
        adjustment: 0.257,
        grid_origin: (100, 250),
        min_layer_name: "m1".into(),
        max_layer_name: "m2".into(),
    };
    let mut log = Vec::new();
    report_layer_settings(&o, &mut log);
    assert_eq!(log[2], "[INFO GRT-0022] Global adjustment: 25%");
    assert_eq!(log[3], "[INFO GRT-0023] Grid origin: (100, 250)");
    o.adjustment = -0.257;
    log.clear();
    report_layer_settings(&o, &mut log);
    assert_eq!(log[2], "[INFO GRT-0022] Global adjustment: -25%");
}
