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

// ⚠️ Test names carry the rule; the capitalised word is what must not be missed when the gate
// goes red and the failure line is the first thing read.
#![allow(non_snake_case)]

use std::collections::BTreeMap;
use vyges_grt::*;

const CORPUS: &str = include_str!("../examples/grt_gate/corpus.json");
const GOLDEN: &str = include_str!("../examples/grt_gate/grt_gate.ok");

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
    let v: serde_json::Value = serde_json::from_str(CORPUS).expect("corpus parses");
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
fn the_corpus_actually_REACHES_the_two_guide_via_form() {
    // A gate is only worth its runtime if the corpus exercises the fork that is easiest to get
    // wrong. If this ever reads 0, the gate has stopped testing the thing it exists for.
    let c = parse_corpus();
    let local = c.nets.iter().filter(|n| n.is_local).count();
    assert!(local > 0, "no local nets: the two-guide via form is unexercised");
    let with_pins = c.nets.iter().filter(|n| !n.pins.is_empty()).count();
    assert!(with_pins > 0, "no pins: the covering-pin test is unexercised");
}
