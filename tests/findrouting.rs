// SPDX-License-Identifier: Apache-2.0
//! F — `findRouting`'s post-processing: the guides `addRemainingGuides` adds (local nets, top-level
//! pins), `connectPadPins` (a no-op), and every route after `mergeSegments`.
//!
//! Golden `findrouting.json`: whole calls — the router's routes going in, the additions, the final
//! routes. The exhaustive replay runs every call of the corpus.

use std::collections::BTreeMap;

use serde_json::Value;
use vyges_grt::{add_remaining_guides, connect_pad_pins, merge_segments, GSegment, GridPin, RemainingNet};

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

/// Segments exactly as captured — NOT through `GSegment::new`, which would re-sort them.
fn segs(v: &Value) -> Vec<GSegment> {
    arr(v)
        .iter()
        .map(|s| GSegment {
            init_x: int(&s[0]),
            init_y: int(&s[1]),
            init_layer: int(&s[2]),
            final_x: int(&s[3]),
            final_y: int(&s[4]),
            final_layer: int(&s[5]),
            is_jumper: false,
        })
        .collect()
}

#[derive(Default)]
struct Seen {
    calls: usize,
    routes: usize,
    added: usize,
    merged_away: usize,
}

fn replay(g: &Value) -> Seen {
    let mut seen = Seen::default();
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        for c in arr(&r["calls"]) {
            let nets: Vec<RemainingNet> = arr(&c["nets"])
                .iter()
                .map(|n| RemainingNet {
                    name: n["name"].as_str().expect("name").into(),
                    made: int(&n["made"]) == 1,
                    pins: arr(&n["grid_pins"]).iter().map(|p| (int(&p[0]), int(&p[1]), int(&p[2]))).collect(),
                })
                .collect();
            let pins_of: BTreeMap<&str, &Vec<GridPin>> = nets.iter().map(|n| (n.name.as_str(), &n.pins)).collect();
            let mut routes: BTreeMap<String, Vec<GSegment>> =
                arr(&c["run"]).iter().map(|e| (e[0].as_str().expect("name").to_string(), segs(&e[1]))).collect();
            let before: BTreeMap<String, usize> = routes.iter().map(|(k, v)| (k.clone(), v.len())).collect();
            add_remaining_guides(&mut routes, &nets, int(&c["min"]), int(&c["max"]), int(&c["block_max"]))
                .unwrap_or_else(|e| panic!("{who}: {e:?}"));
            // Every net whose route grew (or appeared), with exactly the reference's additions.
            let added = c["added"].as_object().expect("added");
            for (name, route) in &routes {
                let from = before.get(name).copied().unwrap_or(0);
                if !before.contains_key(name) || route.len() != from {
                    let want = segs(added.get(name).unwrap_or_else(|| panic!("{who}: {name}: the engine added guides, the reference did not")));
                    assert_eq!(route[from..], want[..], "{who}: {name}: addRemainingGuides");
                    seen.added += 1;
                }
            }
            for name in added.keys() {
                assert!(routes.contains_key(name), "{who}: {name}: the reference added guides, the engine did not");
            }
            connect_pad_pins(&mut routes);
            let merged = arr(&c["merged"]);
            assert_eq!(routes.len(), merged.len(), "{who}: route count");
            for m in merged {
                let name = m[0].as_str().expect("name");
                let route = routes.get_mut(name).unwrap_or_else(|| panic!("{who}: {name}: no route"));
                let pins = pins_of.get(name).unwrap_or_else(|| panic!("{who}: {name}: routed but not in `nets`"));
                let n0 = route.len();
                merge_segments(pins, route, int(&c["block_min"]));
                assert_eq!(*route, segs(&m[1]), "{who}: {name}: mergeSegments");
                seen.merged_away += n0 - route.len();
                seen.routes += 1;
            }
            seen.calls += 1;
        }
    }
    seen
}

#[test]
fn find_routing_post_processing_matches_the_reference() {
    let s = replay(&read(&format!("{}/examples/grt_gate/findrouting.json", env!("CARGO_MANIFEST_DIR"))));
    assert!(s.calls >= 70 && s.routes >= 1500 && s.added > 0 && s.merged_away > 0,
            "calls {}, routes {}, added {}, merged away {}", s.calls, s.routes, s.added, s.merged_away);
}

/// GRT_FINDROUTING_FULL=/path/to/f-all.json cargo test --release --test findrouting -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn find_routing_post_processing_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_FINDROUTING_FULL").expect("set GRT_FINDROUTING_FULL");
    let s = replay(&read(&path));
    eprintln!("exhaustive: {} calls, {} routes, {} nets given guides, {} segments merged away", s.calls, s.routes, s.added, s.merged_away);
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

use vyges_grt::add_guides_for_local_net;

fn seg(x0: i32, y0: i32, l0: i32, x1: i32, y1: i32, l1: i32) -> GSegment {
    GSegment { init_x: x0, init_y: y0, init_layer: l0, final_x: x1, final_y: y1, final_layer: l1, is_jumper: false }
}

/// ⛔ A local net's stack starts at the LOWER of its lowest pin layer and the min routing layer,
/// and rises one above its highest pin; a pin at or above the max stops it one lower. No corpus
/// local net has a pin below the min routing layer.
#[test]
fn a_local_net_stack_starts_at_the_lower_of_pin_and_min_layer() {
    let s = add_guides_for_local_net("n", &[(5, 5, 3), (5, 5, 3)], 2, 6).expect("local");
    assert_eq!(s, vec![seg(5, 5, 2, 5, 5, 3), seg(5, 5, 3, 5, 5, 4)], "from min 2, one above pin 3");
    let s = add_guides_for_local_net("n", &[(5, 5, 1)], 2, 6).expect("local");
    assert_eq!(s, vec![seg(5, 5, 1, 5, 5, 2)], "the pin below the min starts it");
    let s = add_guides_for_local_net("n", &[(5, 5, 6)], 2, 6).expect("local");
    assert_eq!(s.last(), Some(&seg(5, 5, 5, 5, 5, 6)), "a pin at the max: the stack ends at it");
    assert!(add_guides_for_local_net("n", &[(5, 5, 2), (6, 5, 2)], 2, 6).is_err(), "GRT-76");
}

/// ⛔ Merging stops BELOW the block's min routing layer (pin-access guides stay separate).
#[test]
fn segments_below_the_min_layer_are_not_merged() {
    let mut r = vec![seg(0, 0, 1, 1, 0, 1), seg(1, 0, 1, 2, 0, 1)];
    merge_segments(&[], &mut r, 2);
    assert_eq!(r.len(), 2);
    let mut r = vec![seg(0, 0, 2, 1, 0, 2), seg(1, 0, 2, 2, 0, 2)];
    merge_segments(&[], &mut r, 2);
    assert_eq!(r, vec![seg(0, 0, 2, 2, 0, 2)]);
}
