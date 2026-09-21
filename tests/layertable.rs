// SPDX-License-Identifier: Apache-2.0
//! R16 — the availability table's derivation.
//!
//! 1,040 edges from 69 designs. ⛔ **The reach-outside path fires on none of them**, so it is
//! pinned by constructed cases below and by a tripwire that asserts the absence, and the golden
//! decides only the path the designs take.

use serde_json::Value;
use vyges_grt::{build_layer_grid, LayerDir, LayerRange, TableInputs, BARRED};

struct Case {
    design: String,
    routelen: usize,
    num_layers: usize,
    range_in: LayerRange,
    range_out: LayerRange,
    net_cost: i32,
    has_2d_overflow: bool,
    step_is_vertical: Vec<bool>,
    layer_dir: Vec<LayerDir>,
    layer_edge_cost: Vec<i32>,
    resources: Vec<Vec<i32>>,
    want: Vec<Vec<i32>>,
}

fn cases() -> Vec<Case> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/grt_gate/layertable.json");
    let text = std::fs::read_to_string(path).expect("golden present");
    let v: Value = serde_json::from_str(&text).expect("golden parses");
    let rows = |v: &Value| -> Vec<Vec<i32>> {
        v.as_array().expect("rows").iter()
            .map(|r| r.as_array().expect("row").iter()
                .map(|c| c.as_i64().expect("int") as i32).collect())
            .collect()
    };
    v["edges"].as_array().expect("edges").iter().map(|c| {
        let i = |k: &str| c[k].as_i64().unwrap_or_else(|| panic!("{k}"));
        Case {
            design: c["design"].as_str().expect("design").to_string(),
            routelen: i("routelen") as usize,
            num_layers: i("num_layers") as usize,
            range_in: LayerRange {
                min_layer: i("min_layer_in") as usize,
                max_layer: i("max_layer_in") as usize,
            },
            range_out: LayerRange {
                min_layer: i("min_layer_out") as usize,
                max_layer: i("max_layer_out") as usize,
            },
            net_cost: i("net_cost") as i32,
            has_2d_overflow: c["has_2d_overflow"].as_bool().expect("ovf"),
            step_is_vertical: c["step_is_vertical"].as_array().expect("dir").iter()
                .map(|x| x.as_bool().expect("bool")).collect(),
            layer_dir: c["layer_dir"].as_array().expect("ld").iter()
                .map(|x| match x.as_i64().expect("dir") {
                    0 => LayerDir::Vertical,
                    1 => LayerDir::Horizontal,
                    _ => LayerDir::Other,
                }).collect(),
            layer_edge_cost: c["layer_edge_cost"].as_array().expect("ec").iter()
                .map(|x| x.as_i64().expect("int") as i32).collect(),
            resources: rows(&c["resources"]),
            want: rows(&c["table"]),
        }
    }).collect()
}

fn run(c: &Case) -> (Vec<Vec<i32>>, LayerRange) {
    let inp = TableInputs {
        num_layers: c.num_layers,
        routelen: c.routelen,
        step_is_vertical: &c.step_is_vertical,
        layer_dir: &c.layer_dir,
        resources: &c.resources,
        layer_edge_cost: &c.layer_edge_cost,
        net_cost: c.net_cost,
        has_2d_overflow: c.has_2d_overflow,
    };
    let mut range = c.range_in;
    let grid = build_layer_grid(&inp, &mut range);
    (grid, range)
}

#[test]
fn the_derived_table_matches_the_reference() {
    let all = cases();
    assert!(all.len() >= 900, "corpus too thin: {}", all.len());
    let (mut mixed, mut overflow, mut narrow) = (0usize, 0usize, 0usize);

    for c in &all {
        let (got, range) = run(c);
        assert_eq!(
            got, c.want,
            "availability table on {} ({} steps, layers {}..={}, overflow {})",
            c.design, c.routelen, c.range_in.min_layer, c.range_in.max_layer, c.has_2d_overflow
        );
        // ⛔ The range is an output as much as the table is.
        assert_eq!(
            range, c.range_out,
            "layer range left behind on {} — the pass widened differently",
            c.design
        );
        mixed += usize::from(c.step_is_vertical.iter().collect::<std::collections::HashSet<_>>().len() > 1);
        overflow += usize::from(c.has_2d_overflow);
        narrow += usize::from(c.range_in.max_layer - c.range_in.min_layer <= 2);
    }
    // ⚠️ An edge that only ever runs one way cannot show that the direction test is per step.
    assert!(mixed >= 300, "too few edges change direction mid-route: {mixed}");
    // ⚠️ Both sides of the guard that decides whether the pass may reach outside the range.
    assert!(overflow >= 200, "too few edges with 2D overflow set: {overflow}");
    assert!(all.len() - overflow >= 400, "too few without it: {}", all.len() - overflow);
    assert!(narrow >= 20, "too few narrow layer ranges: {narrow}");
}

/// ⛔ **The reach-outside path does not fire on any shipped design — measured, not assumed.**
///
/// It needs two things at once: no layer in the net's range can carry the step, **and** two
/// dimensional routing left no overflow behind. Across 29,264 steps of 15,269 edges from 69
/// designs, the first condition is met **494 times and the second never coincides with it** —
/// every resource-exhausted step belongs to a run that still had 2D overflow, so the reference
/// takes the other branch.
///
/// ⚠️ The two are naturally opposed: a design whose per-layer resources are exhausted is usually
/// one whose 2D routing overflowed. The flag is `total_overflow_ > 0`, set once before layer
/// assignment.
///
/// This test asserts the absence. The rules of the path itself are pinned by the constructed
/// cases below, because the corpus cannot pin them.
#[test]
fn no_captured_edge_reaches_outside_its_layer_range() {
    let all = cases();
    let (mut exhausted, mut exhausted_without_overflow) = (0usize, 0usize);

    for c in &all {
        let (_, range) = run(c);
        assert_eq!(
            range, c.range_in,
            "the layer range on {} was widened — the reach-outside path now fires and this \
             note is stale",
            c.design
        );
        for k in 0..c.routelen {
            let vertical = c.step_is_vertical[k];
            let any_fits = (c.range_in.min_layer..=c.range_in.max_layer).any(|l| {
                let matches = match c.layer_dir[l] {
                    LayerDir::Vertical => vertical,
                    LayerDir::Horizontal => !vertical,
                    LayerDir::Other => false,
                };
                matches && c.resources[l][k] >= c.layer_edge_cost[l]
            });
            if !any_fits {
                exhausted += 1;
                exhausted_without_overflow += usize::from(!c.has_2d_overflow);
            }
        }
    }
    // ⚠️ The first condition must be reached, or "it never fires" would be vacuous — it would
    // just mean no step ever runs short.
    assert!(
        exhausted >= 20,
        "no captured step exhausts its layer range ({exhausted}) — the absence below proves \
         nothing"
    );
    assert_eq!(
        exhausted_without_overflow, 0,
        "{exhausted_without_overflow} steps now exhaust their range with no 2D overflow — the \
         reach-outside path is reachable and must be pinned by a captured case, not a \
         constructed one"
    );
}

/// ⚠️ The column past the last step is never written; [`vyges_grt::layerdp`]'s backward direction
/// reads its orientation. Asserted on both sides so the two cannot drift apart.
#[test]
fn the_column_past_the_last_step_is_left_alone() {
    for c in &cases() {
        let (got, _) = run(c);
        for l in 0..c.num_layers {
            assert_eq!(
                got[l][c.routelen], 0,
                "{} wrote {} past the last step on layer {l}",
                c.design, got[l][c.routelen]
            );
        }
    }
}

// ─── The reach-outside path, constructed ────────────────────────────────────────────────────
//
// No shipped design reaches this path, so these cases come from the reference's rules rather than
// from a capture. ⚠️ They are stated as the source states them, and each one fails if the
// corresponding rule is dropped — that is all they can claim. They are not evidence that the
// reference behaves this way on a real design, and the tripwire above is what would notice the
// day one does.

fn starved(num_layers: usize, dirs: &[LayerDir], resources: Vec<Vec<i32>>) -> Vec<Vec<i32>> {
    let costs = vec![10i32; num_layers];
    let inp = TableInputs {
        num_layers,
        routelen: 1,
        step_is_vertical: &[true],
        layer_dir: dirs,
        resources: &resources,
        layer_edge_cost: &costs,
        net_cost: 5,
        has_2d_overflow: false,
    };
    let mut range = LayerRange { min_layer: 2, max_layer: 2 };
    let grid = build_layer_grid(&inp, &mut range);
    // The in-range layer is horizontal on a vertical step, so nothing in range fits.
    assert_eq!(grid[2][0], BARRED, "the in-range layer should be barred by direction");
    grid
}

/// ⛔ The scan below the range runs **downwards**, closest layer first, and the bound names the
/// first layer that reaches the net's cost.
#[test]
fn the_widened_bound_names_the_first_sufficient_layer_below() {
    let dirs = [LayerDir::Vertical, LayerDir::Vertical, LayerDir::Horizontal];
    // Layer 1 is closer and sufficient; layer 0 is also sufficient but must never be reached.
    let inp = TableInputs {
        num_layers: 3,
        routelen: 1,
        step_is_vertical: &[true],
        layer_dir: &dirs,
        resources: &[vec![99], vec![7], vec![0]],
        layer_edge_cost: &[10, 10, 10],
        net_cost: 5,
        has_2d_overflow: false,
    };
    let mut range = LayerRange { min_layer: 2, max_layer: 2 };
    let grid = build_layer_grid(&inp, &mut range);
    assert_eq!(range.min_layer, 1, "the closer sufficient layer must take the bound");
    assert_eq!(grid[1][0], 7, "layer 1 is the one that was priced");
    // ⛔ Layer 0 is barred although it had the larger resource: the scan had already succeeded.
    assert_eq!(
        grid[0][0], BARRED,
        "layer 0 was examined after the scan already met the net's cost and must be barred"
    );
}

/// ⛔ A layer whose direction does not match the step is barred even though it is the only one
/// left — the path does not relax the direction rule, only the range.
#[test]
fn the_reach_outside_scan_still_respects_direction() {
    let dirs = [LayerDir::Horizontal, LayerDir::Horizontal, LayerDir::Horizontal];
    let grid = starved(3, &dirs, vec![vec![99], vec![99], vec![0]]);
    assert_eq!(grid[0][0], BARRED, "a horizontal layer cannot carry a vertical step");
    assert_eq!(grid[1][0], BARRED, "nor this one");
}

/// ⚠️ **The two direction tests differ, and this pins the difference.** The in-range scan asks
/// whether the layer's direction *is* the step's, so a layer with neither direction is barred.
/// The reach-outside scan asks whether `is_vertical` *equals* the step's orientation, so on a
/// horizontal step a layer with neither direction counts as horizontal and is accepted.
#[test]
fn a_layer_with_neither_direction_is_treated_differently_by_the_two_scans() {
    let dirs = [LayerDir::Other, LayerDir::Vertical, LayerDir::Vertical];
    let costs = [10i32, 10, 10];

    // In range, on a horizontal step: barred, because its direction is not HORIZONTAL.
    let inp = TableInputs {
        num_layers: 3, routelen: 1, step_is_vertical: &[false], layer_dir: &dirs,
        resources: &[vec![99], vec![0], vec![0]], layer_edge_cost: &costs,
        net_cost: 5, has_2d_overflow: true,
    };
    let mut range = LayerRange { min_layer: 0, max_layer: 0 };
    assert_eq!(
        build_layer_grid(&inp, &mut range)[0][0], BARRED,
        "the in-range scan bars a layer with neither direction"
    );

    // Outside the range, same layer, same step: accepted, because it is not VERTICAL and the
    // step is not vertical either.
    let inp = TableInputs {
        num_layers: 3, routelen: 1, step_is_vertical: &[false], layer_dir: &dirs,
        resources: &[vec![99], vec![0], vec![0]], layer_edge_cost: &costs,
        net_cost: 5, has_2d_overflow: false,
    };
    let mut range = LayerRange { min_layer: 1, max_layer: 1 };
    let grid = build_layer_grid(&inp, &mut range);
    assert_eq!(
        grid[0][0], 99,
        "the reach-outside scan accepts a layer with neither direction on a horizontal step"
    );
    assert_eq!(range.min_layer, 0, "and it takes the bound");
}

/// ⛔ The best found below carries into the scan above: a sufficient layer below stops the upward
/// scan from pricing anything, and the upper bound is left where it was.
#[test]
fn a_layer_found_below_stops_the_scan_above() {
    let dirs = [LayerDir::Vertical, LayerDir::Horizontal, LayerDir::Vertical];
    let inp = TableInputs {
        num_layers: 3,
        routelen: 1,
        step_is_vertical: &[true],
        layer_dir: &dirs,
        resources: &[vec![9], vec![0], vec![99]],
        layer_edge_cost: &[10, 10, 10],
        net_cost: 5,
        has_2d_overflow: false,
    };
    let mut range = LayerRange { min_layer: 1, max_layer: 1 };
    let grid = build_layer_grid(&inp, &mut range);
    assert_eq!(range.min_layer, 0, "layer 0 sufficed");
    assert_eq!(grid[0][0], 9);
    assert_eq!(
        range.max_layer, 1,
        "the upper bound must not move once the lower scan has already met the net's cost"
    );
    assert_eq!(
        grid[2][0], BARRED,
        "layer 2 must be barred unpriced, although its resource is the largest of the three"
    );
}

/// ⛔ With 2D overflow set the pass never reaches outside, however starved the range is.
#[test]
fn two_dimensional_overflow_blocks_the_reach_outside_entirely() {
    let dirs = [LayerDir::Vertical, LayerDir::Horizontal, LayerDir::Vertical];
    let inp = TableInputs {
        num_layers: 3,
        routelen: 1,
        step_is_vertical: &[true],
        layer_dir: &dirs,
        resources: &[vec![99], vec![0], vec![99]],
        layer_edge_cost: &[10, 10, 10],
        net_cost: 5,
        has_2d_overflow: true,
    };
    let mut range = LayerRange { min_layer: 1, max_layer: 1 };
    let grid = build_layer_grid(&inp, &mut range);
    assert_eq!(range, LayerRange { min_layer: 1, max_layer: 1 }, "the range must not move");
    assert_eq!(grid[0][0], BARRED);
    assert_eq!(grid[2][0], BARRED);
}

/// ⛔ A widening on one step is visible to the next: the range is state threaded through the
/// loop, not a constant read once.
///
/// ⚠️ **Built so the two readings disagree.** An earlier version of this test could not fail:
/// it checked a cell that reads the sentinel either way — barred by direction if the range
/// widened, barred as out-of-range if it did not. Here the second step prices layer 0 only if the
/// first step's widening carried, so the cell holds a resource under one reading and the sentinel
/// under the other.
#[test]
fn a_widened_range_is_visible_to_the_following_step() {
    let dirs = [LayerDir::Vertical, LayerDir::Vertical, LayerDir::Vertical];
    let inp = TableInputs {
        num_layers: 3,
        routelen: 2,
        step_is_vertical: &[true, true],
        layer_dir: &dirs,
        // Step 0 starves in range and widens down to layer 0. Step 1 does not starve.
        resources: &[vec![9, 42], vec![0, 99], vec![0, 0]],
        layer_edge_cost: &[10, 10, 10],
        net_cost: 5,
        has_2d_overflow: false,
    };
    let mut range = LayerRange { min_layer: 1, max_layer: 1 };
    let grid = build_layer_grid(&inp, &mut range);
    assert_eq!(range.min_layer, 0, "the first step widened the range down to layer 0");
    assert_eq!(grid[1][1], 99, "the second step prices its original in-range layer");
    // ⛔ The witness. With the range threaded, layer 0 is in range on the second step and is
    // priced. With the range read fresh from the net each step, it is out of range and barred.
    assert_eq!(
        grid[0][1], 42,
        "the second step did not price layer 0 — the widening from the first step was lost, so          the range is being read as a constant rather than threaded through the loop"
    );
}

/// ⛔ The early-out is `>=`, so a layer that meets the net's cost **exactly** stops the scan.
///
/// ⚠️ Added because a mutation to `>` survived every other case here: each of them cleared the
/// cost with room to spare, so none could see the boundary.
#[test]
fn meeting_the_net_cost_exactly_stops_the_scan() {
    let dirs = [LayerDir::Vertical, LayerDir::Vertical, LayerDir::Horizontal];
    let inp = TableInputs {
        num_layers: 3,
        routelen: 1,
        step_is_vertical: &[true],
        layer_dir: &dirs,
        // Layer 1 hits the net's cost on the nose; layer 0 must never be priced.
        resources: &[vec![99], vec![5], vec![0]],
        layer_edge_cost: &[10, 10, 10],
        net_cost: 5,
        has_2d_overflow: false,
    };
    let mut range = LayerRange { min_layer: 2, max_layer: 2 };
    let grid = build_layer_grid(&inp, &mut range);
    assert_eq!(range.min_layer, 1, "an exact match must take the bound");
    assert_eq!(grid[1][0], 5);
    assert_eq!(grid[0][0], BARRED, "an exact match must stop the scan, not merely tie");
}

/// ⛔ When **no** layer outside the range reaches the net's cost, the bound does not move at all
/// — every one of them is priced, and none is adopted.
///
/// ⚠️ This is the case that separates "the bound names the first sufficient layer" from "the
/// bound names the last layer tried". Where something does suffice the two agree, because the
/// early-out bars everything after it.
#[test]
fn a_scan_that_never_suffices_leaves_the_bound_alone() {
    let dirs = [LayerDir::Vertical, LayerDir::Vertical, LayerDir::Horizontal];
    let inp = TableInputs {
        num_layers: 3,
        routelen: 1,
        step_is_vertical: &[true],
        layer_dir: &dirs,
        resources: &[vec![9], vec![7], vec![0]],
        layer_edge_cost: &[10, 10, 10],
        // Nothing below reaches this.
        net_cost: 50,
        has_2d_overflow: false,
    };
    let mut range = LayerRange { min_layer: 2, max_layer: 2 };
    let grid = build_layer_grid(&inp, &mut range);
    // Both were priced — the scan did run over them.
    assert_eq!(grid[1][0], 7);
    assert_eq!(grid[0][0], 9);
    assert_eq!(
        range.min_layer, 2,
        "no layer met the net's cost, so the bound must stay where it started"
    );
}

/// ⛔ The scan above the range starts **above** it. Starting at the top layer of the range would
/// overwrite a cell the in-range scan has already priced.
#[test]
fn the_scan_above_does_not_revisit_the_range_itself() {
    let dirs = [LayerDir::Vertical, LayerDir::Vertical, LayerDir::Vertical];
    let inp = TableInputs {
        num_layers: 3,
        routelen: 1,
        step_is_vertical: &[true],
        layer_dir: &dirs,
        // Layer 1 is in range and priced at 3 — short of its own cost, so the pass reaches out,
        // but a real value all the same. Layer 0 then meets the net's cost.
        resources: &[vec![9], vec![3], vec![0]],
        layer_edge_cost: &[10, 10, 10],
        net_cost: 5,
        has_2d_overflow: false,
    };
    let mut range = LayerRange { min_layer: 1, max_layer: 1 };
    let grid = build_layer_grid(&inp, &mut range);
    assert_eq!(range.min_layer, 0, "layer 0 met the net's cost");
    assert_eq!(
        grid[1][0], 3,
        "the in-range layer's own price was overwritten — the upward scan started one layer \
         too low"
    );
}

/// ⛔ **The one surviving mutation, and why it survives.**
///
/// Starting the per-step best at zero instead of the least integer changes nothing here. The best
/// is read in exactly two places, and both are `best >= net_cost` — so a starting value can only
/// matter if it can itself reach the net's cost. Every captured net costs **1, 3, 5 or 9**, so
/// zero never does.
///
/// ⚠️ The mutation is not dead code: 28 captured cells hold a **negative** resource, so the `max`
/// is genuinely doing work, and a net costing zero or less would make the two starts disagree
/// immediately — the scan would bar every layer outside the range without pricing one. This test
/// asserts the precondition rather than the consequence, so the day a net costs nothing the gate
/// fails and this note is revisited.
#[test]
fn every_net_costs_at_least_one() {
    let all = cases();
    let mut negative_resources = 0usize;
    for c in &all {
        assert!(
            c.net_cost >= 1,
            "{} has a net cost of {} — a per-step best starting at zero would now reach it, and \
             the starting value stops being unobservable",
            c.design, c.net_cost
        );
        negative_resources += c.resources.iter().flatten().filter(|x| **x < 0).count();
    }
    // ⚠️ Without a negative resource anywhere, "the starting value is unobservable" would be true
    // for an uninteresting reason.
    assert!(
        negative_resources >= 10,
        "no captured resource is negative ({negative_resources}) — the running maximum is never \
         tested against one"
    );
}
