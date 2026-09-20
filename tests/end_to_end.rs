// SPDX-License-Identifier: Apache-2.0
//! **The gating test.** One test that drives the whole guide writer over a real design and fails
//! if any of it drifts.
//!
//! 🔑 **The golden is the reference's own output, unmodified.** `examples/grt_gate/grt_gate.ok`
//! is the guide file a published global router writes for this design, and it was verified
//! byte-identical to the one that ships with that router's own test suite before being committed
//! here. So this is not a golden of our own behaviour captured and blessed.
//!
//! 🔑 **`corpus.json` is the input that golden was produced from** — the same run, the same
//! moment — so there is no second spelling of the design to drift against.
//!
//! ⛔ **Two of the inputs appear in no output file**: whether a net fits inside one grid cell,
//! and where each of its pins sits. Both select between the one-guide and two-guide via forms, so
//! a replay without them cannot tell a missing rule from a missing input. They are in the corpus.
//!
//! 🔑 **It needs no database, no container and no network** — the corpus and the golden are
//! committed, so this runs anywhere `cargo test` runs, in under a second.
//!
//! What the corpus reaches:
//!
//! | | |
//! | --- | --- |
//! | 563 nets, 3,770 segments, 1,536 pins | a whole design, not a hand-picked case |
//! | **68 local nets** | the two-guide via form, which changes the guide COUNT |
//! | 3,848 guides (5,537 golden file lines) | every box, layer and ordering decision |
//! | 1,414 of them connected-to-term | the stateful first-claim rule, from a SECOND golden |

// ⚠️ Test names carry the rule; the capitalised word is what must not be missed when the gate
// goes red and the failure line is the first thing read.
#![allow(non_snake_case)]

use std::collections::BTreeMap;
use vyges_grt::*;

const CORPUS: &str = include_str!("../examples/grt_gate/corpus.json");

/// ⭐ Four more designs, each chosen because it walks a path the first one does not.
///
/// `pin_access1` is the one that pays: **15 nets**, and it closes two coverage gaps that 563
/// nets could not — a net whose pin route point is touched twice, and a net with two pins at one
/// point on different layers. `pin_track_not_aligned` is a **single net, two pins**, and is the
/// only case found that reaches the instance-edge path in pin derivation.
const CASES: &[(&str, &str, &str)] = &[
    ("pin_access1",
     include_str!("../examples/grt_gate/cases/pin_access1.corpus.json"),
     include_str!("../examples/grt_gate/cases/pin_access1.guideok")),
    ("pin_track_not_aligned",
     include_str!("../examples/grt_gate/cases/pin_track_not_aligned.corpus.json"),
     include_str!("../examples/grt_gate/cases/pin_track_not_aligned.guideok")),
    ("macro_obs_not_aligned",
     include_str!("../examples/grt_gate/cases/macro_obs_not_aligned.corpus.json"),
     include_str!("../examples/grt_gate/cases/macro_obs_not_aligned.guideok")),
    ("modeling_instance_obs",
     include_str!("../examples/grt_gate/cases/modeling_instance_obs.corpus.json"),
     include_str!("../examples/grt_gate/cases/modeling_instance_obs.guideok")),
];

const CASE_CONNECTED: &[(&str, &str)] = &[
    ("pin_access1", include_str!("../examples/grt_gate/cases/pin_access1.connected.ok")),
    ("pin_track_not_aligned", include_str!("../examples/grt_gate/cases/pin_track_not_aligned.connected.ok")),
    ("macro_obs_not_aligned", include_str!("../examples/grt_gate/cases/macro_obs_not_aligned.connected.ok")),
    ("modeling_instance_obs", include_str!("../examples/grt_gate/cases/modeling_instance_obs.connected.ok")),
];
const GOLDEN: &str = include_str!("../examples/grt_gate/grt_gate.ok");
/// ⛔ A SECOND golden, because the guide file does not record `is_connected_to_term`. One line
/// per net: the net's name and one character per guide, in guide-file order.
const CONNECTED: &str = include_str!("../examples/grt_gate/connected.ok");

/// One guide as the golden spells it: a box and a layer NAME.
type GuideLine = (i32, i32, i32, i32, String);

/// Parse the reference's guide file: `net`, `(`, one `x1 y1 x2 y2 layer` per line, `)`.
fn parse_golden(text: &str) -> BTreeMap<String, Vec<GuideLine>> {
    let mut out: BTreeMap<String, Vec<GuideLine>> = BTreeMap::new();
    let mut cur: Option<String> = None;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line == "(" {
            continue;
        }
        if line == ")" {
            cur = None;
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() == 5 {
            let net = cur.as_ref().expect("a guide line outside any net block");
            out.get_mut(net).unwrap().push((
                f[0].parse().unwrap(), f[1].parse().unwrap(),
                f[2].parse().unwrap(), f[3].parse().unwrap(),
                f[4].to_string(),
            ));
        } else {
            cur = Some(line.to_string());
            out.entry(line.to_string()).or_default();
        }
    }
    out
}

struct Corpus {
    grid: Grid,
    opts: SaveOptions,
    layers: BTreeMap<i32, String>,
    nets: Vec<NetRoute>,
}

fn parse_corpus() -> Corpus {
    parse_corpus_text(CORPUS)
}

fn parse_corpus_text(text: &str) -> Corpus {
    let v: serde_json::Value = serde_json::from_str(text).expect("corpus parses");
    let g = &v["grid"];
    let a = g["area"].as_array().unwrap();
    let n = |x: &serde_json::Value| x.as_i64().unwrap() as i32;
    let grid = Grid {
        tile_size: n(&g["tile_size"]),
        area: Rect::new(n(&a[0]), n(&a[1]), n(&a[2]), n(&a[3])),
    };
    let o = &v["opts"];
    let opts = SaveOptions {
        guide_is_congested: o["guide_is_congested"].as_bool().unwrap(),
        origin_x: n(&o["origin_x"]),
        origin_y: n(&o["origin_y"]),
        min_routing_layer: n(&o["min_routing_layer"]),
    };
    let layers = v["layers"].as_object().unwrap().iter()
        .map(|(k, val)| (k.parse().unwrap(), val.as_str().unwrap().to_string()))
        .collect();
    let nets = v["nets"].as_array().unwrap().iter().map(|net| NetRoute {
        name: net["name"].as_str().unwrap().to_string(),
        is_local: net["is_local"].as_bool().unwrap(),
        pins: net["pins"].as_array().unwrap().iter().map(|p| Pin {
            connection_layer: n(&p["connection_layer"]),
            on_grid_x: n(&p["on_grid_x"]),
            on_grid_y: n(&p["on_grid_y"]),
        }).collect(),
        segments: net["segments"].as_array().unwrap().iter().map(|s| GSegment {
            init_x: n(&s["init_x"]), init_y: n(&s["init_y"]), init_layer: n(&s["init_layer"]),
            final_x: n(&s["final_x"]), final_y: n(&s["final_y"]), final_layer: n(&s["final_layer"]),
            is_jumper: s["is_jumper"].as_bool().unwrap(),
        }).collect(),
    }).collect();
    Corpus { grid, opts, layers, nets }
}

#[test]
fn every_guide_of_a_whole_design_matches_the_REFERENCES_OWN_output() {
    let c = parse_corpus();
    let golden = parse_golden(GOLDEN);
    let produced = save_guides(&c.nets, &c.grid, &c.opts).expect("the corpus routes without error");

    // ⚠️ Compared per net AND in order. Guide order within a net is part of the answer, so a
    // set-equality check here would pass an engine that emitted everything backwards.
    let mut checked_nets = 0;
    let mut checked_guides = 0;
    for net in &produced {
        let want = golden.get(&net.net)
            .unwrap_or_else(|| panic!("net {} is in the corpus but not the golden", net.net));
        let got: Vec<GuideLine> = net.guides.iter().map(|g| (
            g.box_.x_min, g.box_.y_min, g.box_.x_max, g.box_.y_max,
            c.layers.get(&g.layer).expect("a routing level with no layer name").clone(),
        )).collect();

        assert_eq!(
            got.len(), want.len(),
            "net {}: produced {} guides, the reference produced {} — a COUNT mismatch is the \
             one/two-guide via fork, not a geometry slip",
            net.net, got.len(), want.len()
        );
        for (i, (g, w)) in got.iter().zip(want.iter()).enumerate() {
            assert_eq!(g, w, "net {}, guide {i}", net.net);
        }
        checked_nets += 1;
        checked_guides += got.len();
    }

    // ⛔ The check that stops this passing vacuously: a corpus that produced nothing, or a
    // golden that parsed to nothing, would satisfy every assertion above.
    assert_eq!(checked_nets, golden.len(), "every net in the golden must have been checked");
    let golden_guides: usize = golden.values().map(|v| v.len()).sum();
    assert_eq!(
        checked_guides, golden_guides,
        "every guide in the golden must have been compared"
    );
    // ⚠️ A floor as well as the equality: if both sides parsed to nothing the equality above is
    // satisfied and proves nothing. 3,848 guides across 563 nets is this design; the golden FILE
    // has 5,537 lines, the difference being each net's name and its two brackets.
    assert!(checked_guides > 3_000, "expected a whole design, got {checked_guides}");
}

#[test]
fn the_connected_to_term_flag_matches_the_REFERENCE_on_every_guide() {
    // ⛔ This flag appears in NO output file, so the guide comparison above is blind to it — an
    // engine that never set it at all would pass that test completely. It needed its own golden,
    // captured from the reference at the same moment as the guides.
    //
    // 🔑 It is also the only stateful rule in the stage: the FIRST guide to reach a route point
    // where a pin sits claims it, and every later guide over that point is left unmarked. A
    // per-segment implementation cannot express that, so getting it wrong is easy and invisible.
    let c = parse_corpus();
    let produced = save_guides(&c.nets, &c.grid, &c.opts).expect("routes");

    let want: BTreeMap<&str, &str> = CONNECTED
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let (net, bits) = l.rsplit_once(' ').expect("`<net> <bits>` per line");
            (net, bits)
        })
        .collect();

    let mut checked = 0usize;
    let mut connected = 0usize;
    for net in &produced {
        let bits = want.get(net.net.as_str())
            .unwrap_or_else(|| panic!("net {} has no connected-flag golden", net.net));
        let got: String = net.guides.iter()
            .map(|g| if g.is_connected_to_term { '1' } else { '0' })
            .collect();
        assert_eq!(&got, bits, "net {}: connected-to-term flags", net.net);
        checked += got.len();
        connected += got.chars().filter(|c| *c == '1').count();
    }

    // ⚠️ Vacuity guards in both directions: all-zero would pass a string compare against an
    // all-zero golden, and an empty run would pass everything.
    assert_eq!(checked, 3_848, "every guide must have had its flag compared");
    assert_eq!(connected, 1_414, "the reference marks this many; neither all nor none");
}

#[test]
fn ONE_gap_remains_of_the_three_and_the_suite_says_which() {
    // 🔑 Three rules could once each be replaced by a plausible wrong one with identical output.
    // Mutation testing found all three; the passing gate found none. Two are now CLOSED by adding
    // `pin_access1` — 15 nets, where the main design's 563 could not decide either:
    //
    // | rule | closed by | verified |
    // | --- | --- | --- |
    // | first guide to reach a pin point claims it | `pin_access1` (1 repeated point) | the mutation now fails `the_connected_flag_matches_..._FOUR_MORE_DESIGNS` |
    // | `is_local` compares position only | `pin_access1` (1 net, 2 layers at a point) | the mutation now fails `is_local_matches_the_REFERENCE_on_FIVE_designs` |
    // | ⬜ **a net with no pins is local** | **nothing yet** | only a synthetic case covers it |
    //
    // ⚠️ This asserts the REMAINING gap, so adding a design with a pinless net will make it fail
    // and the last gap gets closed rather than forgotten.
    let mut pinless = 0;
    let mut corpora = 1;
    for net in &parse_corpus().nets {
        if net.pins.is_empty() { pinless += 1; }
    }
    for (_, corpus, _) in CASES {
        corpora += 1;
        for net in &parse_corpus_text(corpus).nets {
            if net.pins.is_empty() { pinless += 1; }
        }
    }
    assert_eq!(corpora, 5, "five designs are scored");
    assert_eq!(pinless, 0,
        "a design here now has a pinless net ({pinless}): the empty-net locality rule is \
         witnessed at last — close the gap and update this test");
}

#[test]
fn the_corpus_actually_REACHES_the_two_guide_via_form() {
    // A gate is only worth its runtime if the corpus exercises the fork that is easiest to get
    // wrong. If this ever reads 0, the gate has stopped testing the thing it exists for.
    let c = parse_corpus();
    let local = c.nets.iter().filter(|n| n.is_local).count();
    assert!(local > 0, "no local nets: the two-guide via form is unexercised");
    let with_pins = c.nets.iter().filter(|n| !n.pins.is_empty()).count();
    assert!(with_pins > 0, "no pins: the covering-pin test is unexercised");
}


/// Compare one design's guides, per net and in order. Returns (nets, guides) checked.
fn check_design(name: &str, corpus: &str, golden_text: &str) -> (usize, usize) {
    let c = parse_corpus_text(corpus);
    let golden = parse_golden(golden_text);
    let produced = save_guides(&c.nets, &c.grid, &c.opts)
        .unwrap_or_else(|e| panic!("{name}: {e}"));

    let (mut nets, mut guides) = (0, 0);
    for net in &produced {
        let want = golden.get(&net.net)
            .unwrap_or_else(|| panic!("{name}: net {} is in the corpus but not the golden", net.net));
        let got: Vec<GuideLine> = net.guides.iter().map(|g| (
            g.box_.x_min, g.box_.y_min, g.box_.x_max, g.box_.y_max,
            c.layers.get(&g.layer).expect("a routing level with no layer name").clone(),
        )).collect();
        assert_eq!(got.len(), want.len(),
            "{name}, net {}: produced {} guides, the reference produced {}", net.net, got.len(), want.len());
        for (i, (g, w)) in got.iter().zip(want.iter()).enumerate() {
            assert_eq!(g, w, "{name}, net {}, guide {i}", net.net);
        }
        nets += 1;
        guides += got.len();
    }
    assert_eq!(nets, golden.len(), "{name}: every net in the golden must have been checked");
    (nets, guides)
}

#[test]
fn every_guide_of_FOUR_MORE_DESIGNS_matches_the_reference() {
    // ⭐ Each of these was picked for a path it walks that the main design does not. Together
    // they are 19 nets against that design's 563 — coverage is not size.
    let mut total_nets = 0;
    let mut total_guides = 0;
    for (name, corpus, golden) in CASES {
        let (n, g) = check_design(name, corpus, golden);
        assert!(n > 0, "{name}: produced no nets");
        total_nets += n;
        total_guides += g;
    }
    assert_eq!(total_nets, 19, "four designs, nineteen nets between them");
    assert_eq!(total_guides, 329, "and this many guides");
}

#[test]
fn the_connected_flag_matches_the_reference_on_FOUR_MORE_DESIGNS() {
    let mut total = 0;
    let mut marked = 0;
    for (name, conn_text) in CASE_CONNECTED {
        let corpus = CASES.iter().find(|(n, _, _)| n == name).unwrap().1;
        let c = parse_corpus_text(corpus);
        let produced = save_guides(&c.nets, &c.grid, &c.opts).expect("routes");
        let want: BTreeMap<&str, &str> = conn_text.lines().filter(|l| !l.trim().is_empty())
            .map(|l| l.rsplit_once(' ').expect("`<net> <bits>`")).collect();
        for net in &produced {
            let bits = want.get(net.net.as_str())
                .unwrap_or_else(|| panic!("{name}: net {} has no flag golden", net.net));
            let got: String = net.guides.iter()
                .map(|g| if g.is_connected_to_term { '1' } else { '0' }).collect();
            assert_eq!(&got, bits, "{name}, net {}: connected-to-term flags", net.net);
            total += got.len();
            marked += got.chars().filter(|ch| *ch == '1').count();
        }
    }
    assert_eq!(total, 329);
    assert_eq!(marked, 83, "neither all nor none");
}

#[test]
fn pin_access1_CLOSES_two_of_the_three_gaps_the_main_design_left_open() {
    // 🔑 The point of adding it. 15 nets close what 563 could not:
    //   * a net whose pin route point is touched TWICE -> the first-claim tie-break is witnessed
    //   * a net with two pins at one point on DIFFERENT LAYERS -> is_local's layer-blindness is
    //
    // ⬜ The third gap, a net with no pins at all, is still open on every design here.
    let c = parse_corpus_text(CASES.iter().find(|(n, _, _)| *n == "pin_access1").unwrap().1);

    let mut repeated = 0;
    let mut same_point_other_layer = 0;
    for net in &c.nets {
        let mut by_xy: BTreeMap<(i32, i32), std::collections::BTreeSet<i32>> = BTreeMap::new();
        for p in &net.pins {
            by_xy.entry((p.on_grid_x, p.on_grid_y)).or_default().insert(p.connection_layer);
        }
        if by_xy.values().any(|l| l.len() > 1) { same_point_other_layer += 1; }

        let pts: std::collections::BTreeSet<RoutePt> = net.pins.iter()
            .map(|p| RoutePt { x: p.on_grid_x, y: p.on_grid_y, layer: p.connection_layer }).collect();
        let mut hits: BTreeMap<RoutePt, usize> = BTreeMap::new();
        for sg in &net.segments {
            for pt in [RoutePt { x: sg.init_x, y: sg.init_y, layer: sg.init_layer },
                       RoutePt { x: sg.final_x, y: sg.final_y, layer: sg.final_layer }] {
                if pts.contains(&pt) { *hits.entry(pt).or_default() += 1; }
            }
        }
        repeated += hits.values().filter(|n| **n > 1).count();
    }
    assert_eq!(repeated, 1, "the first-claim tie-break is witnessed here");
    assert_eq!(same_point_other_layer, 1, "is_local's layer-blindness is witnessed here");
}
