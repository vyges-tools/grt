// SPDX-License-Identifier: Apache-2.0
//! R16 — the layer-assignment dynamic program.
//!
//! 1,001 edge assignments from 35 designs, **468 of which change layer somewhere** (so the via
//! transitions actually decide something), across both directions.
//!
//! ⛔ **Captured in BOTH cost modes.** `getWireCost` and `getViaCost` return 0 outright unless
//! the router runs resistance-aware, so a default capture prices every layer identically and
//! cannot distinguish any weight in the program — the first 285 cases here could not tell the
//! wire cost from a constant. A second sweep with `global_route -resistance_aware` added 716
//! cases, 594 with a live wire cost and 237 with a live via cost, and killed that mutation.
//!
//! ⚠️ The availability table is captured rather than the 3D usage it comes from. That splits the
//! stage cleanly: this golden decides the **program**, and the table's own derivation is a
//! separate piece with its own inputs.

use serde_json::Value;
use vyges_grt::layerdp::BIG_INT;
use vyges_grt::{assign_edge_layers, selection_column_witness, LayerDpInputs, LayerEnd};

struct Case {
    design: String,
    process_dir: bool,
    routelen: usize,
    num_layers: usize,
    min_layer: usize,
    max_layer: usize,
    layer_grid: Vec<Vec<i32>>,
    edge_cost: Vec<i64>,
    wire_cost: Vec<i64>,
    via_cost: Vec<Vec<i64>>,
    n1: LayerEnd,
    n2: LayerEnd,
    want: Vec<i32>,
}

fn cases() -> Vec<Case> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/assignedge.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let rows = |v: &Value| -> Vec<Vec<i32>> {
        v.as_array().expect("rows").iter()
            .map(|r| r.as_array().expect("row").iter()
                .map(|c| c.as_i64().expect("int") as i32).collect())
            .collect()
    };
    let end = |v: &Value| LayerEnd {
        assigned: v["assigned"].as_bool().expect("assigned"),
        bot_layer: v["bot"].as_i64().expect("bot") as i32,
        top_layer: v["top"].as_i64().expect("top") as i32,
    };
    v["edges"].as_array().expect("edges").iter().map(|c| {
        let i = |k: &str| c[k].as_i64().unwrap_or_else(|| panic!("{k}"));
        Case {
            design: c["design"].as_str().expect("design").to_string(),
            process_dir: c["process_dir"].as_bool().expect("dir"),
            routelen: i("routelen") as usize,
            num_layers: i("num_layers") as usize,
            min_layer: i("min_layer") as usize,
            max_layer: i("max_layer") as usize,
            layer_grid: rows(&c["layer_grid"]),
            edge_cost: c["edge_cost"].as_array().expect("ec").iter()
                .map(|x| x.as_i64().expect("int")).collect(),
            wire_cost: c["wire_cost"].as_array().expect("wc").iter()
                .map(|x| x.as_i64().expect("int")).collect(),
            via_cost: c["via_cost"].as_array().expect("vc").iter()
                .map(|r| r.as_array().expect("row").iter()
                    .map(|x| x.as_i64().expect("int")).collect()).collect(),
            n1: end(&c["n1"]),
            n2: end(&c["n2"]),
            want: c["layers"].as_array().expect("layers").iter()
                .map(|x| x.as_i64().expect("layer") as i32).collect(),
        }
    }).collect()
}

fn inputs(c: &Case) -> LayerDpInputs<'_> {
    LayerDpInputs {
        num_layers: c.num_layers,
        routelen: c.routelen,
        min_layer: c.min_layer,
        max_layer: c.max_layer,
        layer_grid: &c.layer_grid,
        edge_cost: &c.edge_cost,
        wire_cost: &c.wire_cost,
        via_cost: &c.via_cost,
    }
}

fn run(c: &Case) -> Vec<i32> {
    assign_edge_layers(&inputs(c), c.process_dir, c.n1, c.n2)
}

#[test]
fn the_assigned_layers_match_the_reference() {
    let all = cases();
    assert!(all.len() >= 900, "corpus too thin: {}", all.len());
    let (mut forward, mut backward, mut with_vias) = (0usize, 0usize, 0usize);
    let (mut priced_wire, mut priced_via) = (0usize, 0usize);

    for c in &all {
        let got = run(c);
        assert_eq!(
            got, c.want,
            "layers on {} (direction {}, {} steps, layers {}..={})",
            c.design, u8::from(c.process_dir), c.routelen, c.min_layer, c.max_layer
        );
        if c.process_dir { forward += 1 } else { backward += 1 }
        with_vias += usize::from(c.want.iter().collect::<std::collections::HashSet<_>>().len() > 1);
        priced_wire += usize::from(c.wire_cost.iter().any(|x| *x != 0));
        priced_via += usize::from(c.via_cost.iter().flatten().any(|x| *x != 0));
    }
    // ⚠️ Both directions must be present: they are not mirror images.
    assert!(forward >= 500, "too few of the first direction: {forward}");
    assert!(backward >= 200, "too few of the second: {backward}");
    // ⚠️ And the via transitions must decide something, or the program is only ever picking one
    // layer and holding it.
    assert!(with_vias >= 300, "too few edges change layer: {with_vias}");
    // ⛔ Both cost modes must be present. Without the resistance-aware half every cost weight in
    // the program is multiplied by zero and the golden cannot see it at all.
    assert!(priced_wire >= 400, "too few edges carry a wire cost: {priced_wire}");
    assert!(priced_via >= 150, "too few edges carry a via cost: {priced_via}");
}

/// ⛔ All three cost tiers must be taken, counted the way the program takes them.
///
/// ⚠️ Wrong orientation and out-of-range are **one** tier, not two: the program tests them with a
/// single `or`. An earlier version of this test counted them separately and reported the
/// out-of-range case as unreached, when in fact every net here excludes layer 0 and most of those
/// cells also carry the orientation sentinel.
#[test]
fn the_corpus_exercises_all_three_cost_tiers() {
    let all = cases();
    let (mut usable, mut barred, mut short_of_resources) = (0usize, 0usize, 0usize);
    let mut out_of_range_cells = 0usize;

    for c in &all {
        for (l, row) in c.layer_grid.iter().enumerate() {
            for cell in row.iter().take(c.routelen) {
                if i64::from(*cell) >= c.edge_cost[l] {
                    usable += 1;
                } else if *cell == i32::MIN || l < c.min_layer || l > c.max_layer {
                    barred += 1;
                    out_of_range_cells += usize::from(l < c.min_layer || l > c.max_layer);
                } else {
                    short_of_resources += 1;
                }
            }
        }
    }
    assert!(usable >= 100, "the usable tier is barely reached: {usable}");
    assert!(barred >= 100, "the barred tier is barely reached: {barred}");
    assert!(
        short_of_resources >= 10,
        "the short-of-resources tier is barely reached: {short_of_resources}"
    );
    // ⚠️ Every captured net excludes layer 0, so the range half of the barred tier is live even
    // where the orientation half would also have fired.
    assert!(out_of_range_cells >= 100, "no cell is out of range: {out_of_range_cells}");
}

/// ⚠️ Every assigned layer must lie inside the grid, whatever the program decided.
#[test]
fn every_assigned_layer_is_a_real_layer() {
    for c in &cases() {
        for (k, l) in run(c).iter().enumerate() {
            assert!(
                *l >= 0 && (*l as usize) < c.num_layers,
                "step {k} of {} was assigned layer {l}, outside 0..{}",
                c.design, c.num_layers
            );
        }
    }
}

// ─── The three surviving mutations, and what each one's survival measures ───────────────────
//
// 8 of 13 deliberate mutations of `layerdp.rs` die on the golden. These five live. A surviving
// mutation has three possible causes — a gap in the corpus, code that cannot change the answer,
// or a real defect — and the tests below record which one each survivor turned out to be, so a
// later reader does not have to re-derive it and a later corpus cannot quietly invalidate it.
//
// ⚠️ A sixth, "the wire cost is dropped from a usable step", survived the first 285 cases and
// died once the resistance-aware sweep landed. That is what a corpus gap looks like when it is
// closed rather than recorded, and it is the reason the ones below were measured rather than
// waved through.

/// ⛔ **Survivor 1 — narrowing the running minimum to an `int` is a GAP, not dead code.**
///
/// The reference holds the running minimum in an `int` while the costs are wider, so a cost above
/// `i32::MAX` is truncated and can land negative. Removing the narrowing here changes nothing,
/// because **no captured case puts a cost that large in the column the selection reads** — one
/// barred step costs `2 * BIG_INT`, which still fits, and no captured edge accumulates two of
/// them there.
///
/// ⚠️ **The margin is 7%.** The worst cost reached is 2,000,000,013 against a limit of
/// 2,147,483,647 — a single further barred step at the selection column would overflow. This is
/// a gap the corpus very nearly closes by accident, not a rule that is out of reach.
///
/// ⚠️ This asserts the limitation itself. If a future capture does reach the truncating range the
/// test fails, which is the intent: the mutation would then be killable and this note stale.
#[test]
fn no_captured_case_reaches_the_narrowing_range() {
    let all = cases();
    let mut worst = i64::MIN;
    for c in &all {
        let inp = inputs(c);
        let (costs, _, _) = selection_column_witness(&inp, c.process_dir, c.n1, c.n2);
        for cost in costs {
            worst = worst.max(cost);
        }
    }
    assert!(
        worst <= i64::from(i32::MAX),
        "a cost of {worst} now exceeds i32::MAX at the selection column — the narrowing is \
         reachable, so it must be pinned by a case rather than recorded as a gap"
    );
    // ⚠️ And the column must carry real magnitudes, or "nothing overflows" would be vacuous.
    assert!(worst >= BIG_INT, "the selection column never even reaches BIG_INT: {worst}");
}

/// ⛔ **Survivor 2 — the "still unset" clause is reached, and CANNOT change the answer.**
///
/// When every candidate layer is unreachable the clause makes the sweep accept the last layer it
/// examines rather than keeping the initialiser. 19 of the captured cases reach that state, and
/// on 3 of them the clause does select a different layer — yet the assigned layers are identical
/// either way, because **both directions follow the link at the selection column before recording
/// anything**, and in every such case that link is the same from every candidate. The choice is
/// overwritten before it is ever read.
///
/// ⚠️ So this survivor is equivalent code *on this corpus*, not an untested rule. The clause is
/// still transcribed: a case whose links disagree would expose it, and the assertion below is
/// what would notice such a case arriving.
#[test]
fn the_all_unreachable_state_is_reached_and_its_choice_is_overwritten() {
    let all = cases();
    let mut reached = 0usize;
    for c in &all {
        let inp = inputs(c);
        let (costs, links, _) = selection_column_witness(&inp, c.process_dir, c.n1, c.n2);
        let end = if c.process_dir { c.n2 } else { c.n1 };
        let candidates: Vec<usize> = if end.assigned {
            (end.bot_layer as usize..=end.top_layer as usize).collect()
        } else {
            (0..c.num_layers).collect()
        };
        if !candidates.iter().all(|i| costs[*i] >= BIG_INT) {
            continue;
        }
        reached += 1;
        // ⛔ The proof that the clause cannot matter here.
        //
        // Both directions begin the read-back by stepping through the link at this column, and
        // both fall back to the candidate itself when that link is unset — so each candidate
        // reduces to one effective starting layer, and everything after is a function of it.
        // Dropping the clause leaves the sweep's initialiser, layer 0; keeping it takes the last
        // candidate examined. Here every one of those reduces to the SAME starting layer, so the
        // two forms cannot produce different assignments.
        let effective = |i: usize| -> usize {
            if links[i] == BIG_INT { i } else { links[i] as usize }
        };
        let first = effective(candidates[0]);
        for i in candidates.iter().copied().chain(std::iter::once(0)) {
            assert_eq!(
                effective(i), first,
                "on {} layer {i} now starts the read-back at {} rather than {first} — the \
                 still-unset clause can change the assignment and must be pinned by a case",
                c.design, effective(i)
            );
        }
    }
    assert!(
        reached >= 15,
        "the all-unreachable state is barely reached ({reached}) — the clause has no witness"
    );
}

/// ⛔ **Survivor 3 — the endpoint's cheaper via base is a GAP in the via costs, not in the path.**
///
/// The first via sweep charges **2** per layer crossed at the edge's own end and **3** everywhere
/// else. Levelling the two shifts the costs on 1,000 of the 1,001 cases and changes the chosen
/// layers on none: the sweep's `argmin` is decided by the via table, and no captured table —
/// including the 237 with live, resistance-derived via costs — is asymmetric enough for one unit
/// per layer crossed to flip it.
///
/// ⚠️ The measurement is what matters — the base is *live* (costs move) but *undecisive* (choices
/// do not). A corpus with a steeper via table would kill the mutation.
#[test]
fn the_endpoint_via_base_moves_costs_but_not_choices() {
    let all = cases();
    let (mut shifted, mut reroutes) = (0usize, 0usize);

    // The endpoint sweep, reproduced here alone — the seeded column, before any step cost.
    let sweep = |c: &Case, base: i64| -> (Vec<i64>, Vec<i64>) {
        let nl = c.num_layers;
        let end = if c.process_dir { c.n1 } else { c.n2 };
        let mut cost = vec![BIG_INT; nl];
        let mut link = vec![BIG_INT; nl];
        if end.assigned {
            for l in end.bot_layer..=end.top_layer {
                cost[l as usize] = 0;
            }
        }
        for l in 0..nl {
            for i in 0..nl {
                let via = if i != l { c.via_cost[l][i] } else { 0 };
                let total = via + (i as i64 - l as i64).abs() * base;
                if cost[i] > cost[l] + total {
                    cost[i] = cost[l] + total;
                    link[i] = l as i64;
                }
            }
        }
        (cost, link)
    };

    for c in &all {
        let (c2, l2) = sweep(c, 2);
        let (c3, l3) = sweep(c, 3);
        shifted += usize::from(c2 != c3);
        reroutes += usize::from(l2 != l3);
    }
    assert!(
        shifted >= 900,
        "the endpoint base barely moves any cost ({shifted}) — it is not live in this corpus and \
         the survivor means something else"
    );
    assert_eq!(
        reroutes, 0,
        "the endpoint base now changes {reroutes} sweeps' choices — the mutation is killable and \
         this note is stale"
    );
}

/// ⛔ **Survivor 4 — the backward step's orientation index reads a column the reference NEVER
/// FILLS, and no assignment turns on it.**
///
/// The backward direction prices a step with the availability of `k-1` and the orientation of
/// `k`. Making both read `k-1` changes nothing across 1,001 cases, 251 of them backward — so the
/// asymmetry is transcribed on the strength of the source, not of the golden.
///
/// ⚠️ Part of the reason is a defect upstream: `assignEdge` fills the availability table only
/// for `k < routelen`, so **column `routelen` is never written**. The backward direction's first
/// step reads its orientation from exactly that column, and gets the container's default. The
/// assertion below is what states that: every cell of the last column is zero, so the orientation
/// sentinel can never be seen there and that one step's barred tier is decided by the layer range
/// alone. Held against the reference pending the open question on the programme's issue.
#[test]
fn the_last_column_of_the_availability_table_is_never_written() {
    let all = cases();
    let (mut cells, mut backward) = (0usize, 0usize);
    for c in &all {
        backward += usize::from(!c.process_dir);
        for l in 0..c.num_layers {
            assert_eq!(
                c.layer_grid[l][c.routelen], 0,
                "the last column of {} now carries {} on layer {l} — the reference has started \
                 filling it, and the backward step's orientation read is no longer reading a \
                 default",
                c.design, c.layer_grid[l][c.routelen]
            );
            cells += 1;
        }
    }
    assert!(cells >= 5000, "too few cells to call the last column unwritten: {cells}");
    // ⚠️ And the direction that reads it must actually be present in force.
    assert!(backward >= 200, "too few backward cases to exercise that read: {backward}");
}

/// ⛔ **Survivor 5 — the backward terminal via sweep's unit base is undecisive here.**
///
/// The terminal sweep charges **1** per layer crossed where the main sweeps charge 2 or 3.
/// Raising the backward one to 3 changes no assignment across 251 backward cases, 56 of which
/// carry a live via cost — the sweep's `argmin` is set by the via table, and one unit per layer
/// crossed does not reach it.
///
/// ⚠️ The forward terminal sweep's own unit base **is** killable and is not listed here; only
/// the backward one survives, which is a coverage difference between the two directions rather
/// than a property of the rule.
#[test]
fn the_backward_terminal_sweep_has_live_via_costs_to_price() {
    let all = cases();
    let priced = all
        .iter()
        .filter(|c| !c.process_dir && c.via_cost.iter().flatten().any(|x| *x != 0))
        .count();
    // ⚠️ This is the precondition for the survivor to mean "undecisive" rather than "never
    // reached". If it fails, the backward terminal base has no priced case at all and the
    // survivor means something weaker than recorded above.
    assert!(
        priced >= 40,
        "no backward case carries a via cost ({priced}) — the terminal base is unexercised, not \
         merely undecisive"
    );
}
