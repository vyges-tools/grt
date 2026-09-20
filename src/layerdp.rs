// SPDX-License-Identifier: Apache-2.0
//! R16 — choosing a layer for every point of one routed edge.
//!
//! A dynamic program over layers. Each grid step can be carried by some set of layers; moving
//! between layers costs a via. The program finds the cheapest assignment from one fixed end of
//! the edge to the other.
//!
//! ⛔ **Three cost tiers, and the ordering between them is the policy.** A step on a usable layer
//! costs one plus that layer's wire cost. A layer whose orientation does not match the step, or
//! which lies outside the net's permitted range, costs **twice** the infinity constant. Anything
//! else short of resources costs it **once**. So when nothing is usable the program still picks a
//! layer — preferring one that merely lacks resources over one that cannot carry the step at all.
//!
//! ⛔ **Every cost weight here is multiplied by zero unless the router runs resistance-aware.**
//! The reference's wire and via cost functions both return 0 outright when that mode is off, so
//! in a default run `wire_cost` and `via_cost` are entirely zero and the program is decided by
//! the availability tiers and the via base costs alone. A correlation corpus taken from default
//! runs therefore cannot see the wire cost at all — the golden for this file is captured in both
//! modes for that reason.
//!
//! ⚠️ **The two directions are not mirror images.** Which end is fixed changes the index the
//! availability test reads, the order the chosen layers are read back in, and whether the far
//! end's layer is overwritten afterwards. Each difference is transcribed rather than levelled.

/// The reference's `BIG_INT`, here as the integer it is declared as.
pub const BIG_INT: i64 = 1_000_000_000;

/// Everything the program prices against, all of it captured from the reference.
pub struct LayerDpInputs<'a> {
    pub num_layers: usize,
    pub routelen: usize,
    pub min_layer: usize,
    pub max_layer: usize,
    /// Per layer, per step: how much of that layer is available for the step.
    ///
    /// ⚠️ The reference stores the sentinel `i32::MIN` here to mean "this layer's orientation
    /// does not match the step", which is a different condition from "not enough left".
    ///
    /// ⛔ **Column `routelen` is never written.** The reference fills this table for `k <
    /// routelen` only, so the last column holds the container's default and means nothing — yet
    /// the backward direction reads its orientation from exactly there on the first step. Left as
    /// the reference leaves it, and pinned by a test.
    pub layer_grid: &'a [Vec<i32>],
    /// Per layer: what one step of this net costs on it.
    pub edge_cost: &'a [i64],
    /// Per layer: the wire cost added to a usable step.
    ///
    /// ⚠️ Zero throughout unless the router runs resistance-aware; see the module note.
    pub wire_cost: &'a [i64],
    /// Per layer pair: the cost of the via between them.
    ///
    /// ⚠️ Zero throughout unless the router runs resistance-aware; see the module note.
    pub via_cost: &'a [Vec<i64>],
}

/// One end of the edge, as the program sees it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LayerEnd {
    pub assigned: bool,
    pub bot_layer: i32,
    pub top_layer: i32,
}

struct Dp {
    cost: Vec<Vec<i64>>,
    link: Vec<Vec<i64>>,
}

impl Dp {
    fn new(num_layers: usize, routelen: usize) -> Self {
        Dp {
            cost: vec![vec![BIG_INT; routelen + 1]; num_layers],
            link: vec![vec![BIG_INT; routelen + 1]; num_layers],
        }
    }

    /// Relax layer transitions within one column.
    ///
    /// ⛔ **In place, and the pair order matters.** The outer layer's cost may already have been
    /// lowered by an earlier pair in the same sweep, so this is not a clean single relaxation
    /// pass — a later pair can chain off an earlier one.
    ///
    /// ⚠️ The base cost per layer crossed is **2** at the edge's own end and **3** elsewhere;
    /// the terminal sweep below uses **1**. Three different weights for the same distance.
    fn propagate_via(&mut self, k: usize, base: i64, via_cost: &[Vec<i64>], num_layers: usize) {
        for l in 0..num_layers {
            for i in 0..num_layers {
                let via = if i != l { via_cost[l][i] } else { 0 };
                let total = via + (i as i64 - l as i64).abs() * base;
                if self.cost[i][k] > self.cost[l][k] + total {
                    self.cost[i][k] = self.cost[l][k] + total;
                    self.link[i][k] = l as i64;
                }
            }
        }
    }

    /// Fill the whole table in the reference's order for this direction, and name the column the
    /// selection then reads.
    ///
    /// ⚠️ **The two directions are separate transcriptions, not one loop with a sign.** The
    /// seeded end, the sweep order, which step the availability and orientation tests read, and
    /// the base cost of the first via sweep all differ.
    fn build(
        inp: &LayerDpInputs<'_>,
        process_dir: bool,
        n1: LayerEnd,
        n2: LayerEnd,
    ) -> (Dp, usize) {
        let (nl, rl) = (inp.num_layers, inp.routelen);
        let mut dp = Dp::new(nl, rl);

        if process_dir {
            if n1.assigned {
                for l in n1.bot_layer..=n1.top_layer {
                    dp.cost[l as usize][0] = 0;
                }
            }
            for k in 0..rl {
                dp.propagate_via(k, if k == 0 { 2 } else { 3 }, inp.via_cost, nl);
                for l in 0..nl {
                    dp.cost[l][k + 1] = dp.cost[l][k] + step_cost(inp, l, k, k);
                }
            }
            dp.propagate_via(rl, 1, inp.via_cost, nl);
            (dp, rl)
        } else {
            if n2.assigned {
                for l in n2.bot_layer..=n2.top_layer {
                    dp.cost[l as usize][rl] = 0;
                }
            }
            for k in (1..=rl).rev() {
                dp.propagate_via(k, if k == rl { 2 } else { 3 }, inp.via_cost, nl);
                for l in 0..nl {
                    // ⛔ Availability of step k-1, orientation of step k.
                    dp.cost[l][k - 1] = dp.cost[l][k] + step_cost(inp, l, k - 1, k);
                }
            }
            dp.propagate_via(0, 1, inp.via_cost, nl);
            (dp, 0)
        }
    }

    /// Pick the cheapest layer in one column.
    ///
    /// ⛔ **`min_result` is an `int` in the reference, assigned from a 64-bit cost.** A cost above
    /// what an `int` holds — which an unreachable layer passes after a single bad step — is
    /// **truncated**, and can land negative. Every later comparison in the sweep then runs against
    /// that truncated value. Transcribed, not widened.
    ///
    /// ⚠️ **The "still unset" clause makes the tie-break direction-dependent.** While the running
    /// minimum is untouched, *any* layer is accepted — so when nothing is reachable the answer is
    /// whichever layer the sweep examines **last**. The constrained sweep descends and the free
    /// one ascends, so they land at opposite ends.
    fn select_min_cost_layer(&self, k: usize, end: LayerEnd, num_layers: usize) -> usize {
        if end.assigned {
            let mut result = 0usize;
            let mut min_result: i32 = BIG_INT as i32;
            let mut i = end.top_layer;
            while i >= end.bot_layer {
                if self.cost[i as usize][k] < i64::from(min_result)
                    || min_result == BIG_INT as i32
                {
                    min_result = self.cost[i as usize][k] as i32;
                    result = i as usize;
                }
                i -= 1;
            }
            result
        } else {
            let mut min_result: i32 = self.cost[0][k] as i32;
            let mut result = 0usize;
            for i in 0..num_layers {
                if self.cost[i][k] < i64::from(min_result) || min_result == BIG_INT as i32 {
                    min_result = self.cost[i][k] as i32;
                    result = i;
                }
            }
            result
        }
    }
}

/// The cost of taking one step on one layer.
///
/// ⛔ The availability test and the orientation test read **different steps** in the backward
/// direction — the first reads the step being priced, the second the one after it. The forward
/// direction reads the same step for both.
fn step_cost(inp: &LayerDpInputs<'_>, l: usize, avail_k: usize, orient_k: usize) -> i64 {
    if i64::from(inp.layer_grid[l][avail_k]) >= inp.edge_cost[l] {
        1 + inp.wire_cost[l]
    } else if inp.layer_grid[l][orient_k] == i32::MIN
        || l < inp.min_layer
        || l > inp.max_layer
    {
        2 * BIG_INT
    } else {
        BIG_INT
    }
}

/// Assign a layer to every point of the edge.
///
/// `process_dir` names which end is fixed: the first when true, the second when false.
pub fn assign_edge_layers(
    inp: &LayerDpInputs<'_>,
    process_dir: bool,
    n1: LayerEnd,
    n2: LayerEnd,
) -> Vec<i32> {
    let (nl, rl) = (inp.num_layers, inp.routelen);
    let (dp, sel_k) = Dp::build(inp, process_dir, n1, n2);
    let end = if process_dir { n2 } else { n1 };
    let end_layer = dp.select_min_cost_layer(sel_k, end, nl);
    let mut layers = vec![0i32; rl + 1];

    if process_dir {
        // ⚠️ Unlike the other direction, the starting layer steps back through the link once
        // before the read-back begins.
        let mut last = if dp.link[end_layer][rl] == BIG_INT {
            end_layer
        } else {
            dp.link[end_layer][rl] as usize
        };
        for k in (0..=rl).rev() {
            layers[k] = last as i32;
            if dp.link[last][k] != BIG_INT {
                last = dp.link[last][k] as usize;
            }
        }
    } else {
        let mut last = end_layer;
        for k in 0..=rl {
            // ⚠️ The link is followed BEFORE the layer is recorded here, and after it in the
            // other direction.
            if dp.link[last][k] != BIG_INT {
                last = dp.link[last][k] as usize;
            }
            layers[k] = last as i32;
        }
        // ⛔ The far end's layer is then overwritten with its neighbour's. The other direction
        // has no such step.
        if rl >= 1 {
            layers[rl] = layers[rl - 1];
        }
    }
    layers
}

/// What the table held in the column the selection reads: every layer's cost, every layer's link,
/// and the layer the program went on to choose.
///
/// ⚠️ **An observability hook, not a second implementation.** It runs the same
/// [`Dp::build`] the driver runs, so it cannot drift from it. The tests use it to measure which
/// branches of the selection a corpus actually reaches — three deliberate mutations of this file
/// survive, and only a witness on this column says whether each is a gap in the corpus or code
/// that cannot change the answer.
#[doc(hidden)]
pub fn selection_column_witness(
    inp: &LayerDpInputs<'_>,
    process_dir: bool,
    n1: LayerEnd,
    n2: LayerEnd,
) -> (Vec<i64>, Vec<i64>, usize) {
    let nl = inp.num_layers;
    let (dp, sel_k) = Dp::build(inp, process_dir, n1, n2);
    let end = if process_dir { n2 } else { n1 };
    let chosen = dp.select_min_cost_layer(sel_k, end, nl);
    (
        (0..nl).map(|l| dp.cost[l][sel_k]).collect(),
        (0..nl).map(|l| dp.link[l][sel_k]).collect(),
        chosen,
    )
}
