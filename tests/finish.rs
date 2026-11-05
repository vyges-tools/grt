// SPDX-License-Identifier: Apache-2.0
//! Xa — `finishGlobalRouting`'s reports and verdict: the congestion table (GRT-0096), the total
//! wirelength (GRT-0018), the routed nets (GRT-0014), the adjustment suggestion (GRT-0704), and
//! GRT-0115 / GRT-0116 — line for line against each run's log.

use serde_json::Value;
use vyges_grt::{
    compute_suggested_adjustment, compute_wirelength, congestion_verdict, report_congestion, report_routed_nets,
    suggest_adjustment, CongestedGrid, CongestionLayer,
};

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i32 {
    v.as_i64().expect("integer") as i32
}

fn arr(v: &Value) -> &Vec<Value> {
    v.as_array().expect("array")
}

fn edges(rows: &Value, l: i32, ny: i32) -> Vec<(u16, u16)> {
    (0..ny)
        .flat_map(|y| {
            arr(&rows[format!("{l},{y}")])
                .iter()
                .flat_map(|p| std::iter::repeat_n((int(&p[0]) as u16, int(&p[1]) as u16), int(&p[2]) as usize))
                .collect::<Vec<_>>()
        })
        .collect()
}

#[derive(Default)]
struct Seen {
    warned_704: usize,
    calls: usize,
    tables: usize,
    suggestions: usize,
    log_runs: usize,
}

fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        let mut log = Vec::new();
        for c in arr(&r["calls"]) {
            let (verbose, cugr) = (int(&c["verbose"]) == 1, int(&c["cugr"]) == 1);
            if verbose && !cugr {
                let t = &c["table"];
                let (nl, yg) = (int(&t["nl"]), int(&t["yg"]));
                let h: Vec<Vec<(u16, u16)>> = (0..nl).map(|l| edges(&t["h"], l, yg)).collect();
                let v: Vec<Vec<(u16, u16)>> = (0..nl).map(|l| edges(&t["v"], l, yg - 1)).collect();
                let names = arr(&t["names"]);
                let layers: Vec<CongestionLayer<'_>> = (0..nl as usize)
                    .map(|l| CongestionLayer { name: names[l].as_str().expect("name"), h_edges: &h[l], v_edges: &v[l] })
                    .collect();
                report_congestion(&layers, &mut log);
                seen.tables += 1;
            }
            let lengths: Vec<Vec<i32>> = arr(&c["lengths"]).iter().map(|r| arr(r).iter().map(int).collect()).collect();
            let def_units = int(&c["def_units"]);
            compute_wirelength(&lengths, int(&c["tile"]), vyges_grt::finish::DbUnits { tech_dbu_per_micron: def_units, block_def_units: def_units }, verbose, &mut log);
            report_routed_nets(lengths.len(), verbose, &mut log);
            let congested = int(&c["congested"]) == 1;
            if !cugr {
                // (CUGR's congestion comes from its own overflow, not captured.)
                assert_eq!(congested, int(&c["overflow"]) > 0, "{who}: congested = total overflow > 0");
            }
            if congested && !cugr {
                let s = &c["suggest"];
                let grid = |d: &str| -> Vec<CongestedGrid> {
                    arr(&s["grids"])
                        .iter()
                        .filter(|g| g["dir"].as_str() == Some(d))
                        .map(|g| CongestedGrid {
                            real_cap: int(&g["real"]),
                            usage: int(&g["usage"]),
                            layer_real_caps: arr(&g["layers"]).iter().map(int).collect(),
                        })
                        .collect()
                };
                let suggestion = compute_suggested_adjustment(&grid("H"), &grid("V"));
                let want = &s["result"];
                assert_eq!(suggestion.is_some(), int(&want[0]) == 1, "{who}: a suggestion exists");
                if let Some(v) = suggestion {
                    assert_eq!(v, int(&want[1]), "{who}: the suggested adjustment");
                }
                let adj: Vec<f32> = arr(&s["adjustments"]).iter().map(|a| a.as_f64().expect("f") as f32).collect();
                let before = log.len();
                suggest_adjustment(&adj, suggestion, &mut log);
                seen.warned_704 += log.len() - before;
                seen.suggestions += 1;
            }
            congestion_verdict(congested, int(&c["allow"]) == 1, cugr, &mut log);
            seen.calls += 1;
        }
        // ⚠️ A CUGR run prints its own congestion report (not captured); a quiet run's log is silent.
        if r["quiet"].as_bool().expect("flag") || r["cugr"].as_bool().expect("flag") {
            continue;
        }
        // ⚠️ The corpus harness (`helpers.tcl`) runs `suppress_message GRT 704` (and 303): the logger
        // drops those lines. The engine still emits GRT-704 — its suggestion is checked above against
        // the captured result — and the harness's suppression is applied here, as the logger would.
        log.retain(|l| !l.line().contains("GRT-0704]"));
        let want: Vec<&str> = arr(&r["log"]).iter().map(|l| l.as_str().expect("line")).collect();
        assert_eq!(vyges_grt::log::lines(&log), want, "{who}: GRT-0096 / 0018 / 0014 / 0115 / 0116 lines");
        seen.log_runs += 1;
    }
    seen
}

#[test]
fn the_finish_reports_match_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/finish.json", env!("CARGO_MANIFEST_DIR"))));
    assert!(s.warned_704 > 0, "no GRT-704 raised: the suggestion's warning branch is unexercised");
    assert!(s.calls >= 80 && s.tables >= 25 && s.suggestions >= 16 && s.log_runs >= 35,
            "calls {}, tables {}, suggestions {}, log runs {}", s.calls, s.tables, s.suggestions, s.log_runs);
}

/// GRT_FINISH_FULL=/path/to/x-all.json cargo test --release --test finish -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_finish_reports_match_the_reference_exhaustively() {
    let path = std::env::var("GRT_FINISH_FULL").expect("set GRT_FINISH_FULL");
    let s = replay(&read(&path));
    eprintln!("exhaustive: {} calls, {} tables, {} suggestions, {} runs' logs", s.calls, s.tables, s.suggestions, s.log_runs);
}

// ─── Constructed cases: what the corpus logs cannot show ────────────────────────────────────

/// ⛔ Any overflowing cell whose real capacity is below its usage abandons the WHOLE suggestion —
/// horizontal cells too (the corpus's four abandoned suggestions all fail on vertical cells).
#[test]
fn a_hopeless_horizontal_cell_abandons_the_suggestion() {
    let bad = CongestedGrid { real_cap: 5, usage: 8, layer_real_caps: vec![5] };
    let ok = CongestedGrid { real_cap: 20, usage: 10, layer_real_caps: vec![20] };
    assert_eq!(compute_suggested_adjustment(&[bad], std::slice::from_ref(&ok)), None);
    assert_eq!(compute_suggested_adjustment(std::slice::from_ref(&ok), &[]), Some(50));
}

/// ⛔ GRT-704 — suppressed by the corpus harness (`suppress_message GRT 704`), so its text is pinned
/// here: the smallest NON-ZERO layer adjustment in range, as a float percentage, strictly above the
/// suggestion.
#[test]
fn grt_704_names_the_smallest_nonzero_adjustment() {
    let mut log = Vec::new();
    suggest_adjustment(&[0.0, 0.95, 1.0], Some(84), &mut log);
    assert_eq!(vyges_grt::log::lines(&log), ["[WARNING GRT-0704] Try reduce the layer adjustment from 95% to 84%"]);
    log.clear();
    suggest_adjustment(&[0.84], Some(84), &mut log);
    assert!(log.is_empty(), "equal is not above");
    suggest_adjustment(&[0.95], None, &mut log);
    assert!(log.is_empty(), "no suggestion, no warning");
}

/// ⛔ The verdict: CUGR congestion is only a WARNING even without `-allow_congestion`; FastRoute's is
/// the GRT-116 error. The replay skips CUGR runs' logs, so this pins the CUGR branch.
#[test]
fn cugr_congestion_is_only_a_warning() {
    let mut log = Vec::new();
    assert!(congestion_verdict(true, false, true, &mut log));
    assert!(vyges_grt::log::lines(&log)[0].starts_with("[WARNING GRT-0115]"));
    log.clear();
    assert!(!congestion_verdict(true, false, false, &mut log));
    assert!(vyges_grt::log::lines(&log)[0].starts_with("[ERROR GRT-0116]"));
    log.clear();
    assert!(congestion_verdict(false, false, false, &mut log) && log.is_empty());
}

// Upstream rule (finishGlobalRouting: computeWirelength, then GRT-14 `routes_.size()`): over the
// route map, every entry counts as a routed net — an empty route too — and a route's wirelength is
// its MOVING segments plus a tile each (a via counts nothing); microns by integer division.
#[test]
fn route_map_reports_every_entry_and_moving_segments_only() {
    use vyges_grt::GSegment;
    let seg = |x0, y0, l0, x1, y1, l1| GSegment { init_x: x0, init_y: y0, init_layer: l0, final_x: x1, final_y: y1, final_layer: l1, is_jumper: false };
    let mut routes = std::collections::BTreeMap::new();
    routes.insert("a".to_string(), vec![seg(0, 0, 2, 3000, 0, 2), seg(3000, 0, 2, 3000, 0, 3)]);
    routes.insert("b".to_string(), vec![seg(0, 0, 3, 0, 1500, 3)]);
    routes.insert("c".to_string(), Vec::new());
    let mut log: Vec<vyges_grt::log::Msg> = Vec::new();
    // (3000 + 600) + (1500 + 600) = 5700 dbu at 1000 per micron -> 5 um.
    let units = vyges_grt::finish::DbUnits { tech_dbu_per_micron: 1000, block_def_units: 1000 };
    assert_eq!(vyges_grt::finish::report_route_map(&routes, 600, units, true, &mut log), 5);
    let lines: Vec<String> = log.iter().map(|m| m.line()).collect();
    assert_eq!(lines, ["[INFO GRT-0018] Total wirelength: 5 um", "[INFO GRT-0014] Routed nets: 3"]);
    let mut quiet: Vec<vyges_grt::log::Msg> = Vec::new();
    vyges_grt::finish::report_route_map(&routes, 600, units, false, &mut quiet);
    assert!(quiet.is_empty(), "both are verbose only");
}

// Upstream rule (`computeWirelength`): `total_wirelength / block_->getDefUnits()` — the BLOCK's DEF
// units, not the tech's database units; they differ on asap7 (tech 4000, DEF 1000), where dividing
// by the tech's reads a quarter of the wirelength. Integer division of the int64 total.
#[test]
fn wirelength_microns_divides_by_the_block_def_units() {
    use vyges_grt::finish::{wirelength_microns, DbUnits};
    let asap7 = DbUnits { tech_dbu_per_micron: 4000, block_def_units: 1000 };
    assert_eq!(wirelength_microns(7_364_500, asap7), 7364);
    let finer_def = DbUnits { tech_dbu_per_micron: 1000, block_def_units: 2000 };
    assert_eq!(wirelength_microns(638_999, finer_def), 319, "truncates toward zero");
    assert_eq!(wirelength_microns(i64::from(i32::MAX) * 4, asap7), i64::from(i32::MAX) * 4 / 1000, "the total is int64");
}

// Upstream rule (`finishGlobalRouting` → `computeWirelength`, shared by FastRoute and CUGR): the
// FastRoute total is divided by the BLOCK's DEF units too — one design, tech 4000 / DEF 1000.
#[test]
fn compute_wirelength_divides_by_the_block_def_units_for_either_router() {
    let units = vyges_grt::finish::DbUnits { tech_dbu_per_micron: 4000, block_def_units: 1000 };
    let mut log: Vec<vyges_grt::log::Msg> = Vec::new();
    // (3000 + 500) + (1000 + 500) = 5000 dbu -> 5 um by DEF units (1 by the tech's).
    assert_eq!(compute_wirelength(&[vec![3000, 0], vec![1000]], 500, units, true, &mut log), 5);
    assert_eq!(log.iter().map(|m| m.line()).collect::<Vec<_>>(), ["[INFO GRT-0018] Total wirelength: 5 um"]);
}
