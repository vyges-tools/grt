// SPDX-License-Identifier: Apache-2.0
//! Ib — I6 `initRoutingTracks`: average track spacing, default vias, via dimensions and the
//! line-to-via pitches (`calcLayerPitches`), and GRT-0088.
//!
//! Golden `tracks.json`: every distinct `calcLayerPitches` call of the corpus (both cost modes,
//! both engines' setup — the CUGR path shares it), with the spacing-table lookups the reference
//! made as the oracle; and per run every `initRoutingTracks` call's raw track patterns, the answers,
//! and the run's own GRT-0088 lines.

use serde_json::Value;
use vyges_grt::{
    calc_layer_pitches, get_default_vias, get_via_dims, init_routing_tracks, Direction, PitchLayer, SpacingLookup,
    TechVia, TrackError, TrackGrid, TrackLayer, TrackPattern, V54Rule,
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

/// The lookups the reference made, keyed `index:kind:args`. ⛔ A lookup it never made — different
/// arguments — panics: the arguments are part of what is checked.
struct Oracle<'a>(&'a serde_json::Map<String, Value>);

impl Oracle<'_> {
    fn get(&self, key: String) -> i32 {
        int(self.0.get(&key).unwrap_or_else(|| panic!("the reference never looked up {key}")))
    }
}

impl SpacingLookup for Oracle<'_> {
    fn tw(&self, layer: &PitchLayer, width1: i32, width2: i32, prl: i32) -> i32 {
        self.get(format!("{}:tw:{width1},{width2},{prl}", layer.index))
    }
    fn v55(&self, layer: &PitchLayer, width: i32, prl: i32) -> i32 {
        self.get(format!("{}:v55:{width},{prl}", layer.index))
    }
}

fn vias(c: &Value) -> Vec<TechVia> {
    arr(&c["vias"])
        .iter()
        .map(|v| TechVia {
            name: v["name"].as_str().expect("name").into(),
            bottom: (int(&v["bottom"]) >= 0).then(|| int(&v["bottom"])),
            or_default: int(&v["or_default"]) == 1,
            boxes: arr(&v["boxes"]).iter().map(|b| (int(&b[0]), int(&b[1]), int(&b[2]))).collect(),
        })
        .collect()
}

fn layers(c: &Value) -> Vec<PitchLayer> {
    arr(&c["layers"])
        .iter()
        .map(|l| PitchLayer {
            index: int(&l["index"]),
            name: l["name"].as_str().expect("name").into(),
            is_routing: l["routing"].as_bool().expect("flag"),
            routing_level: int(&l["level"]),
            width: int(&l["width"]),
            has_two_widths: int(&l["tw"]) == 1,
            has_v55: int(&l["v55"]) == 1,
            v54: arr(&l["v54"])
                .iter()
                .map(|r| V54Rule {
                    spacing: int(&r[0]) as u32,
                    range: (int(&r[1]) == 1).then(|| (int(&r[2]) as u32, int(&r[3]) as u32)),
                })
                .collect(),
        })
        .collect()
}

fn patterns(v: &Value) -> Vec<TrackPattern> {
    arr(v).iter().map(|p| TrackPattern { origin: int(&p[0]), count: int(&p[1]), step: int(&p[2]) }).collect()
}

fn direction(d: &str) -> Option<Direction> {
    match d {
        "H" => Some(Direction::Horizontal),
        "V" => Some(Direction::Vertical),
        _ => None,
    }
}

/// One `initRoutingTracks` call's layers, as reached (the reference prints a layer's record only
/// once its track grid is found).
fn track_layers(c: &Value) -> Vec<TrackLayer> {
    arr(&c["tracks"])
        .iter()
        .map(|t| TrackLayer {
            index: int(&t["index"]),
            name: t["name"].as_str().expect("name").into(),
            direction: direction(t["dir"].as_str().expect("dir")),
            grid: Some(TrackGrid { x: patterns(&t["x"]), y: patterns(&t["y"]) }),
        })
        .collect()
}

/// Every distinct pitch case: getViaDims per layer, then the pitches.
fn replay_pitches(g: &Value) -> Vec<Vec<(i32, i32)>> {
    let mut out = Vec::new();
    for (i, c) in arr(&g["pitch_cases"]).iter().enumerate() {
        let vias = vias(c);
        let layers = layers(c);
        let defaults = get_default_vias(&vias);
        let oracle = Oracle(c["lookups"].as_object().expect("lookups"));
        let pitches = calc_layer_pitches(
            &layers,
            int(&c["max_layer"]),
            int(&c["block_min"]),
            int(&c["block_max"]),
            int(&c["count"]),
            &vias,
            &oracle,
        );
        for (l, raw) in layers.iter().zip(arr(&c["layers"])) {
            let d: Vec<i32> = arr(&raw["dims"]).iter().map(int).collect();
            let got = get_via_dims(&vias, &defaults, l.routing_level);
            assert_eq!([got.0, got.1, got.2, got.3], d[..], "case {i}: getViaDims on {}", l.name);
            let want = raw.get("pitch").map_or((0, 0), |p| (int(&p[0]), int(&p[1])));
            assert_eq!(pitches[l.index as usize], want, "case {i}: pitches on {}", l.name);
        }
        out.push(pitches);
    }
    out
}

#[test]
fn the_routing_tracks_match_the_reference() {
    let g = read(&format!("{}/examples/grt_gate/tracks.json", env!("CARGO_MANIFEST_DIR")));
    let pitches = replay_pitches(&g);
    let (mut runs, mut layers_seen, mut errors, mut multi, mut quiet) = (0, 0, 0, 0, 0);
    for r in arr(&g["runs"]) {
        let who = r["design"].as_str().expect("design");
        for c in arr(&r["calls"]) {
            let case = &arr(&g["pitch_cases"])[c["case"].as_u64().expect("case") as usize];
            let recs = arr(&c["tracks"]);
            let tl = track_layers(c);
            multi += recs.iter().filter(|t| arr(&t["x"]).len() > 1 || arr(&t["y"]).len() > 1).count();
            let got = init_routing_tracks(
                &tl,
                int(&case["max_layer"]),
                &pitches[c["case"].as_u64().expect("case") as usize],
                int(&c["dbu"]),
                false,
                &mut Vec::new(),
            );
            let answered: Vec<&Value> = recs.iter().filter(|t| t.get("answer").is_some()).collect();
            match got {
                Ok(tracks) => {
                    assert_eq!(answered.len(), recs.len(), "{who}: the reference errored, the engine did not");
                    for (t, rec) in tracks.iter().zip(&answered) {
                        let a: Vec<i32> = arr(&rec["answer"]).iter().map(int).collect();
                        assert_eq!([t.track_pitch, t.location, t.num_tracks], a[..], "{who}: layer {}", rec["name"]);
                    }
                    layers_seen += tracks.len();
                }
                Err(e) => {
                    assert_eq!(answered.len() + 1, recs.len(), "{who}: the engine errored early: {e:?}");
                    assert!(matches!(e, TrackError::NoHorizontalTracks { .. } | TrackError::NoVerticalTracks { .. }));
                    errors += 1;
                }
            }
        }
        if r["quiet"].as_bool().expect("flag") {
            quiet += 1;
        }
        runs += 1;
    }
    assert!(runs >= 200 && layers_seen >= 2000, "runs {runs}, layers {layers_seen}");
    assert!(errors > 0 && multi > 0, "errors {errors}, multi-pattern grids {multi}");
    assert!(quiet <= 6, "{quiet} quiet runs");
}

/// GRT-0088, line for line against every observable run's log (6 `tee -quiet` runs excepted).
#[test]
fn grt_0088_matches_the_reference_logs() {
    let g = read(&format!("{}/examples/grt_gate/tracks.json", env!("CARGO_MANIFEST_DIR")));
    let pitches = replay_pitches(&g);
    let mut checked = 0;
    for r in arr(&g["runs"]) {
        if r["quiet"].as_bool().expect("flag") {
            continue;
        }
        let want: Vec<&str> = arr(&r["log"]).iter().map(|l| l.as_str().expect("line")).collect();
        let mut log = Vec::new();
        for c in arr(&r["calls"]) {
            let k = c["case"].as_u64().expect("case") as usize;
            let case = &arr(&g["pitch_cases"])[k];
            let _ = init_routing_tracks(
                &track_layers(c),
                int(&case["max_layer"]),
                &pitches[k],
                int(&c["dbu"]),
                int(&c["verbose"]) == 1,
                &mut log,
            );
        }
        assert_eq!(log, want, "{}: GRT-0088 lines", r["design"]);
        checked += want.len();
    }
    assert!(checked >= 1000, "only {checked} GRT-0088 lines checked");
}

// ─── Constructed cases: branches and boundaries the corpus never reaches ───────────────────

use vyges_grt::get_average_track_spacing;

fn pat(origin: i32, count: i32, step: i32) -> TrackPattern {
    TrackPattern { origin, count, step }
}

/// ⛔ Several patterns: the coordinates are MERGED — expanded, SORTED, DEDUPLICATED — and the step
/// is `ceil((float) span / tracks)`, over the TRACKS not the gaps. The one averaged layer in the
/// corpus (asap7 M2, 1,498 tracks) gives 39 under every reading; small grids separate them.
#[test]
fn several_track_patterns_average_over_the_merged_tracks() {
    let h = Some(Direction::Horizontal);
    // Interleaved and out of order: 10, 30 then 0, 20 → 0, 10, 20, 30: span 30 over 4 tracks = 8.
    let g = TrackGrid { x: vec![], y: vec![pat(10, 2, 20), pat(0, 2, 20)] };
    assert_eq!(get_average_track_spacing("m", h, &g), Ok((8, 0, 4)));
    // The same pattern twice: 3 distinct tracks, not 6 — span 20 over 3 = 7.
    let g = TrackGrid { x: vec![], y: vec![pat(0, 3, 10), pat(0, 3, 10)] };
    assert_eq!(get_average_track_spacing("m", h, &g), Ok((7, 0, 3)));
}

fn via(name: &str, bottom: Option<i32>, or_default: bool, boxes: Vec<(i32, i32, i32)>) -> TechVia {
    TechVia { name: name.into(), bottom, or_default, boxes }
}

/// ⛔ With `OR_DEFAULT` vias, a LATER one on the same bottom layer REPLACES the earlier (the
/// fallback — every corpus technology — keeps the FIRST); non-default vias are then ignored. The
/// dims come from the FIRST box on the layer. And a via whose bottom is a non-routing layer is never
/// layer 1's "down" via — `findRoutingLayer(0)` is null.
#[test]
fn or_default_vias_last_wins_and_dims_read_the_first_box() {
    let vias = vec![
        via("plain", Some(1), false, vec![(1, 10, 20)]),
        via("d1", Some(1), true, vec![(1, 30, 40)]),
        via("d2", Some(1), true, vec![(1, 50, 60), (1, 99, 99)]),
        via("poly", Some(0), true, vec![(1, 70, 80)]),
    ];
    let d = get_default_vias(&vias);
    assert_eq!(d[&Some(1)], 2, "the later OR_DEFAULT via");
    assert_eq!(get_via_dims(&vias, &d, 1), (50, 60, -1, -1), "first box; no down via at level 1");
}

/// Answers every lookup with a fixed value per kind, recording the arguments.
struct Fixed {
    tw: i32,
    v55: i32,
    asked: std::cell::RefCell<Vec<String>>,
}

impl SpacingLookup for Fixed {
    fn tw(&self, _: &PitchLayer, width1: i32, width2: i32, prl: i32) -> i32 {
        self.asked.borrow_mut().push(format!("tw {width1} {width2} {prl}"));
        self.tw
    }
    fn v55(&self, _: &PitchLayer, width: i32, prl: i32) -> i32 {
        self.asked.borrow_mut().push(format!("v55 {width} {prl}"));
        self.v55
    }
}

fn fixed() -> Fixed {
    Fixed { tw: 7, v55: 11, asked: Default::default() }
}

fn player(index: i32, width: i32, tw: bool, v55: bool, v54: Vec<V54Rule>) -> PitchLayer {
    PitchLayer { index, name: format!("m{index}"), is_routing: true, routing_level: index, width, has_two_widths: tw, has_v55: v55, v54 }
}

/// Vias between 1-2 and 2-3, each 20 x 40 on both layers.
fn stack() -> Vec<TechVia> {
    vec![via("v12", Some(1), false, vec![(1, 20, 40), (2, 20, 40)]), via("v23", Some(2), false, vec![(2, 20, 40), (3, 20, 40)])]
}

/// ⛔ A layer with NO default via either way keeps the vector's default `(0, 0)`, not `(-1, -1)`.
#[test]
fn a_layer_without_default_vias_keeps_zero_pitches() {
    let p = calc_layer_pitches(&[player(1, 30, false, true, vec![])], -1, 1, 1, 1, &[], &fixed());
    assert_eq!(p[1], (0, 0));
}

/// ⛔ The rule PRIORITY: the two-widths table over V5.5 — `findTwSpacing(layer_width, via_width,
/// prl)`; V5.5 looked up at `max(layer_width, via_width)`. No corpus layer has a two-widths table,
/// and every corpus via is at least as wide as its layer.
#[test]
fn two_widths_beats_v55_and_v55_uses_the_wider_width() {
    let s = fixed();
    let p = calc_layer_pitches(&[player(2, 30, true, true, vec![])], -1, 1, 3, 3, &stack(), &s);
    assert_eq!(*s.asked.borrow(), ["tw 30 20 40", "tw 30 20 40"]);
    assert_eq!(p[2], (15 + 10 + 7, 15 + 10 + 7));
    let s = fixed();
    let _ = calc_layer_pitches(&[player(2, 30, false, true, vec![])], -1, 1, 3, 3, &stack(), &s);
    assert_eq!(*s.asked.borrow(), ["v55 30 40", "v55 30 40"], "max(30, 20) = 30");
}

/// ⛔ V5.4: the LARGEST spacing among rules whose RANGE holds the layer width.
#[test]
fn v54_takes_the_largest_in_range_spacing() {
    let rules = vec![
        V54Rule { spacing: 40, range: None },
        V54Rule { spacing: 90, range: Some((0, 20)) },
        V54Rule { spacing: 60, range: Some((25, 35)) },
    ];
    let p = calc_layer_pitches(&[player(2, 30, false, false, rules)], -1, 1, 3, 3, &stack(), &fixed());
    assert_eq!(p[2], (15 + 10 + 60, 15 + 10 + 60));
}

/// ⛔ `L2V = width / 2 + via / 2 + spacing` — each half truncated (odd 15 + 15 → 7 + 7), and `-1`
/// upward at the BLOCK's max routing layer, compared with the index — not the `max_layer` argument.
#[test]
fn l2v_truncates_each_half_and_stops_at_the_block_max() {
    let vias = vec![via("v23", Some(2), false, vec![(2, 15, 40)]), via("v12", Some(1), false, vec![(2, 15, 40)])];
    let p = calc_layer_pitches(&[player(2, 15, false, false, vec![V54Rule { spacing: 5, range: None }])], 9, 1, 2, 3, &vias, &fixed());
    assert_eq!(p[2], (-1, 7 + 7 + 5), "block max is 2; max_layer is 9");
    let p = calc_layer_pitches(&[player(2, 15, false, false, vec![V54Rule { spacing: 5, range: None }])], 9, 1, 3, 3, &vias, &fixed());
    assert_eq!(p[2], (7 + 7 + 5, 7 + 7 + 5), "below the block max, upward is computed too");
}

/// ⛔ Both loops STOP at the max routing layer (`index > max && max > -1`): a layer above it gets
/// no pitches and no tracks. The capture records only layers reached, so only this case sees it.
#[test]
fn both_loops_stop_at_the_max_routing_layer() {
    let layers = [player(1, 30, false, true, vec![]), player(2, 30, false, true, vec![]), player(3, 30, false, true, vec![])];
    let p = calc_layer_pitches(&layers, 2, 1, 3, 3, &stack(), &fixed());
    assert_eq!(p[3], (0, 0));
    let grid = TrackGrid { x: vec![pat(0, 5, 10)], y: vec![pat(0, 5, 10)] };
    let tl: Vec<TrackLayer> = (1..=3)
        .map(|i| TrackLayer { index: i, name: format!("m{i}"), direction: Some(Direction::Vertical), grid: Some(grid.clone()) })
        .collect();
    let t = init_routing_tracks(&tl, 2, &p, 1000, false, &mut Vec::new()).expect("tracks");
    assert_eq!(t.len(), 2);
}
