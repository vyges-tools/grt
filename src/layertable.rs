// SPDX-License-Identifier: Apache-2.0
//! R16 — deriving the availability table the layer-assignment program prices against.
//!
//! The other half of the reference's `assignEdge`: before the dynamic program in [`crate::layerdp`]
//! runs, this fills one cell per layer per grid step with how much of that layer is free for the
//! step — or a sentinel meaning the layer cannot carry it at all.
//!
//! ⛔ **The net's layer range is MUTABLE STATE threaded through the loop.** When no layer in range
//! can carry a step, the reference reaches outside the range, and **writes the widened bound back
//! onto the net**. Later steps of the same edge then read the widened range, and so does the
//! dynamic program afterwards. Hoisting the range out of the loop as a constant reads naturally
//! and is wrong.
//!
//! ⚠️ **Two different thresholds for "enough".** The in-range scan asks whether a layer's free
//! resource reaches that **layer's** edge cost; the reach-outside scan asks whether the best found
//! so far reaches the **net's** edge cost. They are different numbers and neither substitutes for
//! the other.
//!
//! ⛔ **Column `routelen` is never written** — the loop runs over steps, of which there are
//! `routelen`, while the table is one wider. See the module note in [`crate::layerdp`]: the
//! backward direction reads that unwritten column's orientation.
//!
//! # What is captured and what is derived
//!
//! The per-step free resource is captured; the table is derived from it. The reference reads the
//! resource out of the 3D capacity and usage arrays, indexed by the step's own position — which
//! is a database walk, not a rule. Splitting there keeps this piece about the rules.

/// A routing layer's preferred wire direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LayerDir {
    Vertical,
    Horizontal,
    /// Neither — the reference compares against its two named directions, so such a layer matches
    /// no step and is always barred.
    Other,
}

/// The net's permitted layer range, which this pass may widen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LayerRange {
    pub min_layer: usize,
    pub max_layer: usize,
}

/// Everything the derivation reads.
pub struct TableInputs<'a> {
    pub num_layers: usize,
    /// Number of grid steps. The table has `routelen + 1` columns and only the first `routelen`
    /// are written.
    pub routelen: usize,
    /// True where the step runs vertically, i.e. the two points share an x.
    pub step_is_vertical: &'a [bool],
    pub layer_dir: &'a [LayerDir],
    /// Per layer, per step: capacity less usage at the position that step reads.
    ///
    /// ⚠️ Captured rather than derived; see the module note.
    pub resources: &'a [Vec<i32>],
    /// Per layer: what one step of this net costs on it.
    ///
    /// ⚠️ An `int8_t` in the reference, widened here at the point the reference widens it.
    pub layer_edge_cost: &'a [i32],
    /// The net's own edge cost — a **different** threshold from the per-layer one above.
    pub net_cost: i32,
    /// Whether two-dimensional routing left any overflow behind.
    ///
    /// ⛔ When set, the reference never reaches outside the layer range, however short of
    /// resources the range is.
    pub has_2d_overflow: bool,
}

/// The sentinel meaning "this layer cannot carry this step".
pub const BARRED: i32 = i32::MIN;

/// Consider one layer outside the net's range as a carrier for this step.
///
/// ⛔ **Two ways to be rejected, and they are not the same.** A layer whose direction does not
/// match the step is barred outright. So is *every* layer once the best found so far already
/// reaches the net's cost — the scan stops improving but keeps writing the sentinel over the
/// layers it skips, so a later layer that would have fitted is barred anyway.
///
/// ⚠️ **This direction test is NOT the one the in-range scan uses.** Here a layer matches when
/// `is_vertical` equals the step's own orientation, so a layer with **neither** direction counts
/// as horizontal and is accepted on a horizontal step. The in-range scan tests for the step's
/// direction by name, so it bars such a layer. Transcribed as the two are written, not levelled.
///
/// ⚠️ `bound` moves only when the layer just examined is what pushed the best over the net's
/// cost, so the widened bound names the **first** layer that sufficed, not the last one tried.
fn fix_edge_assignment(
    bound: &mut usize,
    l: usize,
    layer_dir: LayerDir,
    step_is_vertical: bool,
    resource: i32,
    net_cost: i32,
    best_cost: &mut i32,
    cell: &mut i32,
) {
    let is_vertical = layer_dir == LayerDir::Vertical;
    if is_vertical != step_is_vertical || *best_cost >= net_cost {
        *cell = BARRED;
    } else {
        *cell = resource;
        *best_cost = (*best_cost).max(*cell);
        if *best_cost >= net_cost {
            *bound = l;
        }
    }
}

/// Fill the availability table for one edge, widening the net's layer range where the reference
/// widens it.
///
/// `range` is read **and written**: it carries the net's permitted layers in, and carries out
/// whatever this pass widened them to. The dynamic program that follows reads the widened values.
pub fn build_layer_grid(inp: &TableInputs<'_>, range: &mut LayerRange) -> Vec<Vec<i32>> {
    // ⚠️ The reference only resizes this and never clears it, so the column it does not write
    // holds the container's default. Every capture shows that default is zero; reproduced here so
    // the unwritten column reads the same on both sides.
    let mut grid = vec![vec![0i32; inp.routelen + 1]; inp.num_layers];

    for k in 0..inp.routelen {
        let vertical = inp.step_is_vertical[k];
        let mut best_cost = i32::MIN;
        let mut has_available_resources = false;

        // ⚠️ Reads the range live: an earlier step of this same edge may already have widened it.
        for l in range.min_layer..=range.max_layer {
            let matches = match inp.layer_dir[l] {
                LayerDir::Vertical => vertical,
                LayerDir::Horizontal => !vertical,
                LayerDir::Other => false,
            };
            if matches {
                grid[l][k] = inp.resources[l][k];
                best_cost = best_cost.max(grid[l][k]);
                // ⚠️ Against the LAYER's cost here, against the NET's cost in the reach-outside
                // scan below.
                has_available_resources |= grid[l][k] >= inp.layer_edge_cost[l];
            } else {
                grid[l][k] = BARRED;
            }
        }

        if !has_available_resources && !inp.has_2d_overflow {
            // Closest layer below the range first, then closest above.
            let mut min_layer = range.min_layer;
            for l in (0..range.min_layer).rev() {
                fix_edge_assignment(
                    &mut min_layer, l, inp.layer_dir[l], vertical,
                    inp.resources[l][k], inp.net_cost, &mut best_cost, &mut grid[l][k],
                );
            }
            range.min_layer = min_layer;

            // ⛔ Written back BEFORE the upward scan, and `best_cost` carries across the two — a
            // layer found below can stop the scan above from looking at all.
            let mut max_layer = range.max_layer;
            for l in (range.max_layer + 1)..inp.num_layers {
                fix_edge_assignment(
                    &mut max_layer, l, inp.layer_dir[l], vertical,
                    inp.resources[l][k], inp.net_cost, &mut best_cost, &mut grid[l][k],
                );
            }
            range.max_layer = max_layer;
        } else {
            for l in 0..inp.num_layers {
                if l < range.min_layer || l > range.max_layer {
                    grid[l][k] = BARRED;
                }
            }
        }
    }
    grid
}
