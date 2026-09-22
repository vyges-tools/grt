// SPDX-License-Identifier: Apache-2.0
//! Ga — the driver's own steps: G1 the routable-net scan, G3 `getMinMaxLayer`, G5
//! `reportResources` (the GRT-0053 table, line for line against the run's log).

use serde_json::Value;
use vyges_grt::{get_min_max_layer, has_routable_nets, report_resources, ResourceLayer, GRT_7};

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
                .flat_map(|p| std::iter::repeat((int(&p[0]) as u16, int(&p[1]) as u16)).take(int(&p[2]) as usize))
                .collect::<Vec<_>>()
        })
        .collect()
}

#[derive(Default)]
struct Seen {
    scans: usize,
    minmax: usize,
    computed: usize,
    tables: usize,
    cugr_runs: usize,
    log_runs: usize,
}

fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        let mut log = Vec::new();
        for s in arr(&r["scans"]) {
            // The scan stops at the first routable net, so the counts captured are exactly the ones read.
            let nets: Vec<(usize, usize)> = arr(&s["nets"]).iter().map(|p| (int(&p[0]) as usize, int(&p[1]) as usize)).collect();
            let got = has_routable_nets(nets.iter().copied());
            assert_eq!(got, int(&s["result"]) == 1, "{who}: G1");
            if got {
                assert!(!has_routable_nets(nets[..nets.len() - 1].iter().copied()), "{who}: G1 stops at the FIRST routable net");
            } else {
                log.push(GRT_7.to_string());
            }
            seen.scans += 1;
        }
        for m in arr(&r["minmax"]) {
            let i: Vec<i32> = arr(&m["in"]).iter().map(int).collect();
            let tracks: Vec<bool> = arr(&m["tracks"]).iter().map(|v| int(v) == 1).collect();
            let (block_max, min, max) = get_min_max_layer(i[0], &tracks, i[2], i[3], i[4]).expect("GRT-701 not in the corpus");
            assert_eq!((block_max, min, max), (i[1], int(&m["out"][0]), int(&m["out"][1])), "{who}: G3");
            seen.computed += (i[0] == -1) as usize;
            seen.minmax += 1;
        }
        for t in arr(&r["tables"]) {
            let (nl, xg, yg) = (int(&t["nl"]), int(&t["xg"]), int(&t["yg"]));
            let names = arr(&t["names"]);
            let h: Vec<Vec<(u16, u16)>> = (0..nl).map(|l| edges(&t["h"], l, yg)).collect();
            let v: Vec<Vec<(u16, u16)>> = (0..nl).map(|l| edges(&t["v"], l, yg - 1)).collect();
            assert!(h.iter().all(|e| e.len() == ((xg - 1) * yg) as usize));
            let layers: Vec<ResourceLayer<'_>> = (0..nl as usize)
                .map(|l| ResourceLayer {
                    name: names[l].as_str().expect("name"),
                    horizontal: t["dirs"][l].as_str() == Some("H"),
                    h_edges: &h[l],
                    v_edges: &v[l],
                })
                .collect();
            report_resources(&layers, &mut log);
            seen.tables += 1;
        }
        // ⚠️ A CUGR run (the sweep marks it from the SCRIPT) prints the table from its own engine's
        // resources — not captured.
        // ⚠️ `tee -quiet` runs: their tables never reached the log.
        if r["quiet"].as_bool().expect("flag") {
            continue;
        }
        let uncaptured = int(&r["uncaptured_tables"]);
        assert!(uncaptured >= 0, "{who}: more tables captured than logged");
        if uncaptured > 0 {
            assert!(r["cugr"].as_bool().expect("flag"), "{who}: tables in the log the FastRoute capture lacks, on a non-CUGR run");
            seen.cugr_runs += 1;
            continue;
        }
        let want: Vec<&str> = arr(&r["log"]).iter().map(|l| l.as_str().expect("line")).collect();
        assert_eq!(log, want, "{who}: GRT-0007 / GRT-0053 table lines");
        seen.log_runs += 1;
    }
    seen
}

#[test]
fn the_driver_steps_match_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/driver.json", env!("CARGO_MANIFEST_DIR"))));
    assert!(s.scans >= 200 && s.minmax >= 700 && s.computed > 0 && s.tables >= 50 && s.log_runs >= 150,
            "scans {}, getMinMaxLayer {} ({} computed), tables {}, log runs {}", s.scans, s.minmax, s.computed, s.tables, s.log_runs);
}

/// GRT_DRIVER_FULL=/path/to/g-all.json cargo test --release --test driver -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_driver_steps_match_the_reference_exhaustively() {
    let path = std::env::var("GRT_DRIVER_FULL").expect("set GRT_DRIVER_FULL");
    let s = replay(&read(&path));
    eprintln!("exhaustive: {} scans, {} getMinMaxLayer ({} computed), {} tables, {} runs' logs, {} CUGR runs skipped",
              s.scans, s.minmax, s.computed, s.tables, s.log_runs, s.cugr_runs);
}

// ─── Constructed cases: branches the corpus never reaches ───────────────────────────────────

/// ⛔ G1 accepts a net with two iterms, two bterms, or one of EACH; a design of single-terminal nets
/// has nothing to route (GRT-7). Every corpus design has a routable net.
#[test]
fn only_single_terminal_nets_means_nothing_to_route() {
    assert!(!has_routable_nets([(1, 0), (0, 1), (0, 0)]));
    assert!(has_routable_nets([(1, 1)]));
    assert!(has_routable_nets([(0, 2)]));
}

/// ⛔ The clock layers: the minimum takes the clock minimum only when set (> 0) and only if lower;
/// the maximum is the larger of the two, set or not. No corpus run sets clock layers.
#[test]
fn clock_layers_widen_the_routing_range() {
    assert_eq!(get_min_max_layer(6, &[], 2, 1, 8), Some((6, 1, 8)));
    assert_eq!(get_min_max_layer(6, &[], 2, 3, 4), Some((6, 2, 6)), "a higher clock min does not raise it");
    assert_eq!(get_min_max_layer(-1, &[true, true, false, true], 1, -1, -2), Some((2, 1, 2)), "the unbroken run of track grids");
    assert_eq!(get_min_max_layer(-1, &[false, true], 1, -1, -2), None, "GRT-701");
}

/// ⛔ The reduction is `(1.0 - (float) d / (float) o) * 100` with a DOUBLE `1.0`, stored in a float:
/// 59 of 160 is exactly 63.125 and prints "63.12" (fmt rounds the exact tie to even); computed all
/// in float it is 63.125004 → "63.13". No corpus table lands on such a value.
#[test]
fn the_reduction_is_computed_through_a_double() {
    let h: Vec<(u16, u16)> = vec![(59, 160)];
    let mut log = Vec::new();
    report_resources(&[ResourceLayer { name: "m1", horizontal: true, h_edges: &h, v_edges: &[] }], &mut log);
    assert_eq!(log[5], "m1         Horizontal        160            59          63.12%");
}
