// SPDX-License-Identifier: Apache-2.0
//! Stage R11 — turning each edge's symbolic route into an explicit list of grid points.
//!
//! Up to here an edge records only *which shape* it took: a bend and which way it turned, or a
//! Z and where its middle segment sits. The maze router needs the actual cells, so every shape is
//! walked out into a point per step.
//!
//! ⛔ **The point buffer is sized from the edge's length BEFORE it is recomputed**, while the
//! number of points written is fixed by the geometry. Every shape writes exactly
//! `manhattan + 1` points, so the two agree only while the stored length already equals the
//! Manhattan distance — which the very next line then asserts by recomputing it. The corpus
//! checks that they agree rather than assuming it.

use crate::estimate::LShape;
use crate::RoutePt;

/// The symbolic route an edge carries into this stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SymbolicRoute {
    /// Nothing was routed — a degenerate edge.
    NoRoute,
    /// One bend.
    L(LShape),
    /// Two bends, with the middle segment at `z_point`: a column when `hvh`, a row otherwise.
    Z { hvh: bool, z_point: i32 },
}

/// What the expansion leaves on the edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MazeRoute {
    pub grids: Vec<RoutePt>,
    /// The edge's length, recomputed as the Manhattan distance.
    pub len: i32,
    /// The length the edge had on entry, **not** the number of steps written. The reference sets
    /// this last, overwriting the zero it writes for a degenerate edge.
    ///
    /// 🔑 **Indistinguishable from the recomputed length on every captured record**, because the
    /// stored length is always already the Manhattan distance — an invariant the corpus asserts
    /// rather than assumes, since it is also what makes the reference's buffer `resize` safe.
    /// Taking the entry value is what the reference does, and it is the one that would differ if
    /// that invariant ever broke.
    pub routelen: i32,
}

/// Walk one edge's shape out into grid points.
///
/// ⚠️ **`layer` is never set here.** The reference writes only `x` and `y`, leaving the layer at
/// whatever the buffer held; layer assignment is a later stage. Left at zero rather than invented.
///
/// ⚠️ Horizontal runs assume `x1 <= x2` — the reference writes `for (i = x1; i <= x2; i++)` with
/// no ordering, so a reversed edge would produce no horizontal points at all. Vertical runs, by
/// contrast, branch on the ordering explicitly. The asymmetry is the reference's.
pub fn convert_to_mazeroute(
    (x1, y1): (i32, i32),
    (x2, y2): (i32, i32),
    len_on_entry: i32,
    route: SymbolicRoute,
) -> MazeRoute {
    let manhattan = (x1 - x2).abs() + (y1 - y2).abs();
    let mut grids: Vec<RoutePt> = Vec::new();
    let mut push = |x: i32, y: i32| grids.push(RoutePt { x, y, layer: 0 });

    // ⛔ A degenerate edge is given a length of zero explicitly. **That write is redundant and
    // provably so**: the shape is only ever "no route" for an edge whose endpoints coincide, so
    // the recomputed Manhattan distance is already zero. Measured: all 301 such records in the
    // corpus have coincident endpoints and a zero entry length, and removing the write is a
    // mutation nothing can kill.
    //
    // Kept because the reference keeps it, and because it is the write that would hold if an
    // earlier stage ever left the stored length out of step with the geometry.
    let mut len = manhattan;

    match route {
        SymbolicRoute::NoRoute => {
            push(x1, y1);
            len = 0;
        }
        SymbolicRoute::L(LShape::XFirst) => {
            for i in x1..=x2 {
                push(i, y1);
            }
            if y1 <= y2 {
                for i in y1 + 1..=y2 {
                    push(x2, i);
                }
            } else {
                for i in (y2..=y1 - 1).rev() {
                    push(x2, i);
                }
            }
        }
        SymbolicRoute::L(LShape::YFirst) => {
            if y1 <= y2 {
                for i in y1..=y2 {
                    push(x1, i);
                }
            } else {
                for i in (y2..=y1).rev() {
                    push(x1, i);
                }
            }
            for i in x1 + 1..=x2 {
                push(i, y2);
            }
        }
        SymbolicRoute::Z { hvh: true, z_point } => {
            for i in x1..z_point {
                push(i, y1);
            }
            // ⚠️ The turning column is walked in full here, so the two corner points belong to
            // this run rather than to the horizontal ones either side of it.
            if y1 <= y2 {
                for i in y1..=y2 {
                    push(z_point, i);
                }
            } else {
                for i in (y2..=y1).rev() {
                    push(z_point, i);
                }
            }
            for i in z_point + 1..=x2 {
                push(i, y2);
            }
        }
        SymbolicRoute::Z { hvh: false, z_point } => {
            if y1 <= y2 {
                for i in y1..z_point {
                    push(x1, i);
                }
                for i in x1..=x2 {
                    push(i, z_point);
                }
                for i in z_point + 1..=y2 {
                    push(x2, i);
                }
            } else {
                for i in ((z_point + 1)..=y1).rev() {
                    push(x1, i);
                }
                for i in x1..=x2 {
                    push(i, z_point);
                }
                for i in (y2..=z_point - 1).rev() {
                    push(x2, i);
                }
            }
        }
    }

    MazeRoute { grids, len, routelen: len_on_entry }
}
