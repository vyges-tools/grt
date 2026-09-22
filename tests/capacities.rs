// SPDX-License-Identifier: Apache-2.0
//! Ic — I8 `mirrorGridToFastRoute`, I9 `setCapacities`, I12 `initEdgesCapacityPerLayer`.
//!
//! Golden `capacities.json`: whole calls, chosen for their features (infinite capacity, a one-cell
//! grid, layers below the range, a persisted tracks entry, each regularity combination, each
//! technology) — every 3D and 2D edge capacity, the per-layer minima, sums and lower bounds, and
//! the per-layer copy I12 makes. The exhaustive replay runs the uncapped dump.

use serde_json::Value;
use vyges_grt::{
    init_edges_capacity_per_layer, mirror_grid_to_fast_route, set_capacities, CapacityLayer, CoreGrid, Direction, Rect,
    RoutingTracks,
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

/// `[[value, count], ...]` → the row.
fn unrle(v: &Value) -> Vec<u16> {
    arr(v).iter().flat_map(|p| std::iter::repeat(int(&p[0]) as u16).take(int(&p[1]) as usize)).collect()
}

fn direction(d: &str) -> Option<Direction> {
    match d {
        "H" => Some(Direction::Horizontal),
        "V" => Some(Direction::Vertical),
        _ => None,
    }
}

#[derive(Default)]
struct Seen {
    sets: usize,
    copies: usize,
    infinite: usize,
    one_cell: usize,
    below_range: usize,
}

fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        for s in arr(&r["sets"]) {
            let die = arr(&s["die"]);
            let core = CoreGrid {
                area: Rect::new(int(&die[0]), int(&die[1]), int(&die[2]), int(&die[3])),
                tile_size: int(&s["tile"]),
                x_grids: int(&s["x_grids"]),
                y_grids: int(&s["y_grids"]),
                perfect_regular_x: int(&s["regx"]) == 1,
                perfect_regular_y: int(&s["regy"]) == 1,
                num_layers: int(&s["num_layers"]),
            };
            let layers: Vec<CapacityLayer> = arr(&s["layers"])
                .iter()
                .enumerate()
                .map(|(i, l)| CapacityLayer {
                    direction: direction(l["dir"].as_str().expect("dir")),
                    tracks: (int(&l["entries"]) > 0).then(|| RoutingTracks {
                        layer_index: i as i32 + 1,
                        track_pitch: int(&l["pitch"]),
                        line_2_via_pitch_up: 0,
                        line_2_via_pitch_down: 0,
                        location: int(&l["init"]),
                        num_tracks: int(&l["count"]),
                    }),
                })
                .collect();
            let dirs: Vec<Option<Direction>> = layers.iter().map(|l| l.direction).collect();

            // I8
            let fr = mirror_grid_to_fast_route(&core, &dirs);
            let st: Vec<i32> = arr(&s["state"]).iter().map(int).collect();
            assert_eq!(
                vec![fr.x_grid, fr.y_grid, fr.num_layers, fr.x_range, fr.y_range, fr.regular_x as i32, fr.regular_y as i32,
                     fr.x_corner, fr.y_corner, fr.tile_size, fr.x_grid_max, fr.y_grid_max],
                st,
                "{who}: the mirrored grid"
            );
            let want_dirs: Vec<bool> = arr(&s["dirs"]).iter().map(|d| int(d) == 1).collect();
            let got_dirs: Vec<bool> = fr.directions.iter().map(|d| *d == Some(Direction::Horizontal)).collect();
            assert_eq!(got_dirs, want_dirs, "{who}: router layer directions");

            // I9
            let c = set_capacities(&fr, &core, &layers, int(&s["min"]), int(&s["max"]), int(&s["infinite"]) == 1);
            for (l, m) in arr(&s["cap3d"]).iter().enumerate() {
                assert_eq!(
                    (c.h_capacity_3d[l] as i32, c.v_capacity_3d[l] as i32),
                    (int(&m[0]), int(&m[1])),
                    "{who}: layer {l} minimum capacities"
                );
            }
            let t = arr(&s["totals"]);
            assert_eq!((c.h_capacity, c.v_capacity), (int(&t[0]), int(&t[1])), "{who}: capacity sums");
            assert_eq!(
                (c.h_capacity_lb.to_bits(), c.v_capacity_lb.to_bits()),
                ((t[2].as_f64().expect("f") as f32).to_bits(), (t[3].as_f64().expect("f") as f32).to_bits()),
                "{who}: lower bounds"
            );
            let (xg, yg, nl) = (c.x_grid, c.y_grid, c.num_layers);
            for l in 0..nl {
                for y in 0..yg {
                    let base = ((l * yg + y) * xg) as usize;
                    assert_eq!(c.h3[base..base + xg as usize], unrle(&s["h3"][format!("{l},{y}")])[..], "{who}: H3 {l},{y}");
                    assert_eq!(c.v3[base..base + xg as usize], unrle(&s["v3"][format!("{l},{y}")])[..], "{who}: V3 {l},{y}");
                }
            }
            for y in 0..yg {
                let w = (xg - 1) as usize;
                assert_eq!(c.h2[y as usize * w..(y as usize + 1) * w], unrle(&s["h2"][y.to_string()])[..], "{who}: H2 {y}");
            }
            for y in 0..yg - 1 {
                let w = xg as usize;
                assert_eq!(c.v2[y as usize * w..(y as usize + 1) * w], unrle(&s["v2"][y.to_string()])[..], "{who}: V2 {y}");
            }
            seen.sets += 1;
            seen.infinite += (int(&s["infinite"]) == 1) as usize;
            seen.one_cell += (xg == 1 || yg == 1) as usize;
            seen.below_range += (int(&s["min"]) > 1) as usize;
        }
        // I12 — its input is the 3D caps AFTER the adjustments (I10), captured beside it.
        for cp in arr(&r["copies"]) {
            let (xg, yg, nl) = (int(&cp["x_grid"]), int(&cp["y_grid"]), int(&cp["num_layers"]));
            let mut caps = vyges_grt::EdgeCapacities {
                x_grid: xg,
                y_grid: yg,
                num_layers: nl,
                h3: Vec::new(),
                v3: Vec::new(),
                h2: Vec::new(),
                v2: Vec::new(),
                h_capacity_3d: Vec::new(),
                v_capacity_3d: Vec::new(),
                h_capacity: 0,
                v_capacity: 0,
                h_capacity_lb: 0.0,
                v_capacity_lb: 0.0,
            };
            for l in 0..nl {
                for y in 0..yg {
                    caps.h3.extend(unrle(&cp["h3"][format!("{l},{y}")]));
                    caps.v3.extend(unrle(&cp["v3"][format!("{l},{y}")]));
                }
            }
            let (h, v) = init_edges_capacity_per_layer(&caps);
            for l in 0..nl {
                for x in 0..xg {
                    for (got, key) in [(&h, "c3h"), (&v, "c3v")] {
                        let want: Vec<(u16, f64)> = arr(&cp[key][format!("{l},{x}")])
                            .iter()
                            .flat_map(|p| {
                                std::iter::repeat((int(&p[0]) as u16, p[1].as_f64().expect("f"))).take(int(&p[2]) as usize)
                            })
                            .collect();
                        let base = ((l * xg + x) * yg) as usize;
                        assert_eq!(got[base..base + yg as usize], want[..], "{who}: {key} {l},{x}");
                    }
                }
            }
            seen.copies += 1;
        }
    }
    seen
}

#[test]
fn the_capacities_match_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/capacities.json", env!("CARGO_MANIFEST_DIR"))));
    // ⛔ Not vacuous: every feature the corpus carries is in the committed sample.
    assert!(s.sets >= 16 && s.copies >= 10, "sets {}, copies {}", s.sets, s.copies);
    assert!(s.infinite > 0 && s.one_cell > 0 && s.below_range > 0, "infinite {} one-cell {} below {}", s.infinite, s.one_cell, s.below_range);
}

/// GRT_CAPACITIES_FULL=/path/to/ic-all.json cargo test --release --test capacities -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn the_capacities_match_the_reference_exhaustively() {
    let path = std::env::var("GRT_CAPACITIES_FULL").expect("set GRT_CAPACITIES_FULL");
    let s = replay(&read(&path));
    eprintln!("exhaustive: {} setCapacities, {} initEdgesCapacityPerLayer", s.sets, s.copies);
}

/// ⛔ `setCapacities` tests `direction == HORIZONTAL` and sends EVERY other layer down the vertical
/// branch — including one with no direction (I4 rejects those only on routing levels, so the
/// branch is the rule, not an accident). No corpus layer lacks a direction.
#[test]
fn a_layer_without_a_direction_takes_the_vertical_branch() {
    let core = CoreGrid {
        area: Rect::new(0, 0, 300, 300),
        tile_size: 100,
        x_grids: 3,
        y_grids: 3,
        perfect_regular_x: true,
        perfect_regular_y: true,
        num_layers: 1,
    };
    let tracks = RoutingTracks { layer_index: 1, track_pitch: 10, line_2_via_pitch_up: 0, line_2_via_pitch_down: 0, location: 5, num_tracks: 30 };
    let layers = [CapacityLayer { direction: None, tracks: Some(tracks) }];
    let fr = mirror_grid_to_fast_route(&core, &[None]);
    let c = set_capacities(&fr, &core, &layers, 1, 1, false);
    assert!(c.h3.iter().all(|&v| v == 0), "no horizontal edge written");
    assert_eq!(c.v3[0], 10, "vertical edges, tracks counted across x");
    assert_eq!((c.h_capacity, c.v_capacity), (0, 10));
}
