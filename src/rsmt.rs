// SPDX-License-Identifier: Apache-2.0
//! Turning a net's Steiner tree into the segments the router works on — stages R5 and R7.
//!
//! This is the seam between two engines. The tree comes from a Steiner-tree builder; what this
//! module does is walk it and emit one segment per non-degenerate branch.
//!
//! ⚠️ **Which builder produces the tree is decided per net by its routing alpha**, and the
//! default is **0.3** — greater than zero, so the ordinary path is the Steiner builder's
//! `make_steiner_tree`. The alternative, a coefficient-weighted flute the router owns itself, is
//! reached only when alpha is explicitly set to zero.

/// One branch of a Steiner tree: a point, and the index of the branch it connects to.
///
/// This is the tree's own representation — branch `n` is the parent, and the root points at
/// itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Branch {
    pub x: i32,
    pub y: i32,
    /// Index of the branch this one connects to.
    pub n: usize,
}

/// A segment handed to the router: two points and the net's edge cost.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Segment {
    pub x1: i32,
    pub y1: i32,
    pub x2: i32,
    pub y2: i32,
    pub edge_cost: i8,
}

/// What one net's tree contributes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetSegments {
    pub segments: Vec<Segment>,
    /// Manhattan length over **every** branch, degenerate ones included.
    pub wirelength: i64,
}

/// Walk a tree's branches and emit the router's segments.
///
/// ⛔ **The endpoints are ordered by X ALONE, and that is not the same as ordering them
/// lexicographically.** The rule is `if (x1 < x2) keep else swap`. For a **vertical** segment
/// `x1 == x2`, so the comparison is false and the endpoints are **always swapped** — the branch's
/// own point comes second. An implementation that normalised on `(x, y)` would leave vertical
/// segments the other way round, and every one of them would differ.
///
/// ⚠️ **A degenerate branch — one whose two points coincide — emits NO segment.** Its length is
/// still added to the total, because the accumulation sits before the test, but that placement is
/// **not observable**: a degenerate branch's Manhattan length is zero by definition, so moving the
/// `+=` inside the branch gives the same total. Kept where the reference keeps it, and recorded as
/// equivalent rather than as a rule — a mutation proved the difference undetectable, which is the
/// opposite of what this comment first claimed.
///
/// ⚠️ Every branch is paired with its parent, including the root, whose parent is itself — that
/// pair is degenerate by construction and so contributes nothing but a zero to the total.
pub fn segments_from_tree(branches: &[Branch], edge_cost: i8) -> NetSegments {
    let mut segments = Vec::new();
    let mut wirelength: i64 = 0;

    for b in branches {
        let (x1, y1) = (b.x, b.y);
        let parent = &branches[b.n];
        let (x2, y2) = (parent.x, parent.y);

        wirelength += ((x1 - x2).abs() + (y1 - y2).abs()) as i64;

        if x1 != x2 || y1 != y2 {
            // ⛔ ordered by X only — a vertical segment always swaps
            let seg = if x1 < x2 {
                Segment { x1, y1, x2, y2, edge_cost }
            } else {
                Segment { x1: x2, y1: y2, x2: x1, y2: y1, edge_cost }
            };
            segments.push(seg);
        }
    }

    NetSegments { segments, wirelength }
}

/// The coefficient the congestion-driven pass feeds to the flute path.
///
/// ⚠️ Only consulted when a net's alpha is zero. `1.36` is the plain default; a congestion-driven
/// pass replaces it with a per-net value, and a pass told to ignore adjustments uses `1.2`. The
/// non-congestion pass also drops to `1.2` when the net suits an H-tree.
pub const COEFF_V_DEFAULT: f32 = 1.36;
/// The coefficient used when adjustments are ignored, and for an H-tree-suited net.
pub const COEFF_V_NO_ADJUSTMENTS: f32 = 1.2;

/// The flute accuracy the router asks for when it builds a tree itself.
///
/// ⚠️ **2 here, where the Steiner builder's own façade uses 3.** The router does not inherit the
/// builder's accuracy; it passes its own, and they differ.
pub const ROUTER_FLUTE_ACCURACY: i32 = 2;
