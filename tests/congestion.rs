// SPDX-License-Identifier: Apache-2.0
//! Xb — the congestion the router writes into the database: the gcell grid (capacity, usage per
//! layer and cell) and the congestion markers (cells, source nets, in the seeded-shuffle order).
//!
//! Golden `congestion.json`: whole runs — marker runs first. The exhaustive replay runs every call.

use serde_json::Value;
use vyges_grt::{congestion_markers, update_db_congestion, CongestionEdge, CrossingNet, DbCongestionLayer};

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

fn rows(g: &Value, dir: &str, k: &str, yg: i32) -> Vec<CongestionEdge> {
    (0..yg)
        .flat_map(|y| {
            arr(&g["edges"][format!("{dir},{k},{y}")])
                .iter()
                .flat_map(|p| {
                    let e = CongestionEdge { cap: int(&p[0]) as u16, red: int(&p[1]) as u16, usage: int(&p[2]) as u16 };
                    std::iter::repeat(e).take(int(&p[3]) as usize)
                })
                .collect::<Vec<_>>()
        })
        .collect()
}

#[derive(Default)]
struct Seen {
    grids: usize,
    layers: usize,
    marker_calls: usize,
    markers: usize,
    sourced: usize,
}

fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        for gr in arr(&r["grids"]) {
            let (xg, yg) = (int(&gr["xg"]), int(&gr["yg"]));
            for (k, cells) in gr["cells"].as_object().expect("cells") {
                let (h, v) = (rows(gr, "H", k, yg), rows(gr, "V", k, yg));
                let layer = DbCongestionLayer { horizontal: cells["dir"].as_str() == Some("H"), h: &h, v: &v };
                let got = update_db_congestion(xg, yg, &layer);
                let want: Vec<(f32, f32)> = arr(&cells["rle"])
                    .iter()
                    .flat_map(|p| std::iter::repeat((p[0].as_f64().expect("f") as f32, p[1].as_f64().expect("f") as f32)).take(int(&p[2]) as usize))
                    .collect();
                assert_eq!(got.len(), want.len(), "{who}: layer {k} cell count");
                if let Some(i) = got.iter().zip(&want).position(|(a, b)| a != b) {
                    panic!("{who}: layer {k} cell ({}, {}): engine {:?}, reference {:?}", i as i32 % xg, i as i32 / xg, got[i], want[i]);
                }
                seen.layers += 1;
            }
            seen.grids += 1;
        }
        for m in arr(&r["markers"]) {
            let cells = |d: &str| -> Vec<(i32, i32, i32, i32)> {
                arr(&m["cells"][d]).iter().map(|c| (int(&c[0]), int(&c[1]), int(&c[2]), int(&c[3]))).collect()
            };
            let edges: Vec<Vec<Vec<(i32, i32)>>> = arr(&m["nets"])
                .iter()
                .map(|n| arr(&n["edges"]).iter().map(|e| arr(e).iter().map(|p| (int(&p[0]), int(&p[1]))).collect()).collect())
                .collect();
            let nets: Vec<CrossingNet<'_>> = arr(&m["nets"])
                .iter()
                .zip(&edges)
                .map(|(n, e)| CrossingNet { name: n["name"].as_str().expect("name"), id: int(&n["id"]) as u32, edges: e })
                .collect();
            let (h, v) = congestion_markers(&cells("H"), &cells("V"), &nets, int(&m["tile"]), (int(&m["x_corner"]), int(&m["y_corner"])));
            for (dir, got) in [("H", &h), ("V", &v)] {
                let want = arr(&m["out"][dir]);
                assert_eq!(got.len(), want.len(), "{who}: {dir} marker count");
                for (i, (a, w)) in got.iter().zip(want).enumerate() {
                    let ws: Vec<&str> = arr(&w["sources"]).iter().map(|s| s.as_str().expect("name")).collect();
                    assert_eq!(
                        (a.x, a.y, a.capacity, a.usage, a.sources.iter().map(String::as_str).collect::<Vec<_>>()),
                        (int(&w["x"]), int(&w["y"]), int(&w["cap"]), int(&w["usage"]), ws),
                        "{who}: {dir} marker {i} (iter {})",
                        m["iter"]
                    );
                    seen.sourced += (!a.sources.is_empty()) as usize;
                }
                seen.markers += got.len();
            }
            seen.marker_calls += 1;
        }
    }
    seen
}

#[test]
fn the_database_congestion_matches_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/congestion.json", env!("CARGO_MANIFEST_DIR"))));
    assert!(s.grids >= 30 && s.layers >= 150 && s.markers >= 1000 && s.sourced > 0,
            "grids {}, layers {}, markers {}, with sources {}", s.grids, s.layers, s.markers, s.sourced);
}

/// GRT_CONGESTION_FULL=/path/to/xb-all.json cargo test --release --test congestion -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_database_congestion_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_CONGESTION_FULL").expect("set GRT_CONGESTION_FULL");
    let s = replay(&read(&path));
    eprintln!("exhaustive: {} grid writes ({} layers), {} marker calls, {} markers ({} with sources)", s.grids, s.layers, s.marker_calls, s.markers, s.sourced);
}

// ─── Constructed cases: branches the corpus never reaches ───────────────────────────────────

use vyges_grt::{boost_uniform_int, Mt19937};

fn edge(cap: u16, red: u16, usage: u16) -> CongestionEdge {
    CongestionEdge { cap, red, usage }
}

/// ⛔ Capacity and each direction's usage are narrowed to `uint8_t` (mod 256); the two usages are
/// then summed in `int` (not narrowed again). Only the committed sample's absence of an infinite-
/// capacity design (3276 → 204) and of any usage above 255 leaves this to a constructed case.
#[test]
fn gcell_values_narrow_to_a_byte_per_direction() {
    let h = vec![edge(3276, 0, 200), edge(0, 0, 0), edge(0, 0, 0), edge(0, 0, 0)];
    let v = vec![edge(0, 0, 100), edge(0, 0, 0), edge(0, 0, 0), edge(0, 0, 0)];
    let cells = update_db_congestion(2, 2, &DbCongestionLayer { horizontal: true, h: &h, v: &v });
    assert_eq!(cells[0], (204.0, 300.0), "3276 mod 256 = 204; 200 + 100 summed in int");
}

/// ⛔ boost's bucket step REJECTS a draw past the last full bucket and draws again: for range 2
/// (n = 3) the bucket is floor(2^32 / 3), so 0xFFFF_FFFF lands in bucket 3 — rejected.
#[test]
fn the_uniform_draw_rejects_past_the_last_bucket() {
    let mut draws = [0xFFFF_FFFFu32, 5].into_iter();
    assert_eq!(boost_uniform_int(|| draws.next().expect("a draw"), 2), 0);
}

/// ⛔ A cell centre is `tile * (j + 0.5) + corner` TRUNCATED: with an odd tile, 7 * 0.5 = 3.5 → 3.
/// Every corpus tile is even.
#[test]
fn a_cell_centre_truncates() {
    let (h, _) = congestion_markers(&[(0, 0, 1, 2)], &[], &[], 7, (0, 0));
    assert_eq!((h[0].x, h[0].y), (3, 3));
}

/// The first outputs of `std::mt19937` seeded 42 — a check on the generator itself.
#[test]
fn mt19937_matches_the_standard_sequence() {
    let mut g = Mt19937::new(42);
    assert_eq!([g.next_u32(), g.next_u32(), g.next_u32()], [1608637542, 3421126067, 4083286876]);
}
