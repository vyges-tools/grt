// SPDX-License-Identifier: Apache-2.0
//! R16 — the net ordering layer assignment walks in.
//!
//! Six keys per net, compared in order, deciding which nets get their layers chosen first.
//!
//! ⛔ **Not the same thing as the congestion ordering**, despite the family resemblance and a
//! shared field name. That one accumulates per-edge overflow into a field called `xmin`, then
//! re-sorts by slack after mutating it; this one takes a real minimum x, sorts once, and mutates
//! nothing. Reading either from the other is how a transcription goes wrong quietly.
//!
//! ⛔ **Two of the six keys are identically zero unless the router runs resistance-aware.** The
//! clock key is `0` outright in a default run, and the score key is `0` for every net that is not
//! resistance-aware — which is all of them. A corpus captured from default runs therefore decides
//! only four of the six keys.

use std::cmp::Ordering;

/// One net's sort keys, as the reference's own record holds them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrderNetPin {
    pub tree_index: usize,
    /// ⚠️ **Stored as a full-width integer, computed as a NARROW one.** See [`netpin_order_inc`]:
    /// the reduction runs in the narrow type, and only the result widens.
    pub min_x: i32,
    /// Total edge length over terminal count. ⚠️ A 32-bit float in the reference, and the
    /// comparison below is on that value — widening it here would change which nets tie.
    pub length_per_pin: f32,
    /// `0` for a net with a non-default rule, `1` otherwise — so NDR nets sort **first**.
    pub ndr_priority: i32,
    pub res_aware_score: f32,
    /// `0` for a clock net, `1` otherwise — so clock nets sort first. ⚠️ Identically `0` when the
    /// router is not resistance-aware, which removes the key rather than reversing it.
    pub clock: i32,
}

/// What one net contributes, before the keys are derived from it.
pub struct NetForOrder<'a> {
    pub net_id: usize,
    /// Per edge, in the tree's own edge order.
    pub edge_len: &'a [i32],
    /// Per edge, the x of its **first** node.
    ///
    /// ⛔ **The first node only.** The second node's x never enters the minimum, so this is not
    /// the net's leftmost point — it is the leftmost of one endpoint per edge. Transcribed as
    /// written.
    pub edge_n1_x: &'a [i16],
    pub num_terminals: i32,
    pub has_ndr: bool,
    pub is_res_aware: bool,
    /// The reference's own score for this net, captured.
    ///
    /// ⚠️ It divides four of the net's properties by four run-wide worst-case values, which are
    /// state this pass does not own. Captured rather than derived, so this piece decides the
    /// ordering and that derivation stays separate.
    pub res_aware_score: f32,
    pub is_clock: bool,
}

/// The value the reduction starts from: the largest value the **narrow** type holds.
///
/// ⛔ Not the largest `int`. A net with no edges keeps this value, and it is 32,767.
pub const MIN_X_INITIAL: i16 = i16::MAX;

/// Compare two nets the way the reference's tuple comparison does.
///
/// ⚠️ **Transcribed as a chain of `<`, not as a total order.** The reference compares tied tuples
/// of six values with `<` on each in turn: if neither side is less, it moves on. Two values that
/// are merely incomparable — which floats can be — therefore fall through to the next key rather
/// than being called equal by a comparator that assumes a total order.
fn compare_net_pins(a: &OrderNetPin, b: &OrderNetPin) -> Ordering {
    macro_rules! key {
        ($f:ident) => {
            if a.$f < b.$f {
                return Ordering::Less;
            } else if b.$f < a.$f {
                return Ordering::Greater;
            }
        };
    }
    key!(ndr_priority);
    key!(clock);
    key!(res_aware_score);
    key!(length_per_pin);
    key!(min_x);
    key!(tree_index);
    Ordering::Equal
}

/// Order the nets for layer assignment.
///
/// `resistance_aware` is the router-wide flag, not a per-net property: it decides whether the
/// clock key exists at all.
pub fn netpin_order_inc(nets: &[NetForOrder<'_>], resistance_aware: bool) -> Vec<OrderNetPin> {
    let mut order: Vec<OrderNetPin> = nets
        .iter()
        .map(|n| {
            // ⛔ The reduction runs in the NARROW type. A coordinate that would not fit is not
            // representable here either, and the starting value is that type's maximum — not the
            // maximum of the field this ends up stored in.
            let mut min_x: i16 = MIN_X_INITIAL;
            let mut total_length: i32 = 0;
            for (len, x) in n.edge_len.iter().zip(n.edge_n1_x.iter()) {
                total_length += *len;
                min_x = min_x.min(*x);
            }
            OrderNetPin {
                tree_index: n.net_id,
                min_x: i32::from(min_x),
                length_per_pin: total_length as f32 / n.num_terminals as f32,
                // ⚠️ Zero is the HIGHER priority here.
                ndr_priority: i32::from(!n.has_ndr),
                res_aware_score: if n.is_res_aware { -n.res_aware_score } else { 0.0 },
                clock: if resistance_aware { i32::from(!n.is_clock) } else { 0 },
            }
        })
        .collect();

    // ⚠️ A stable sort, transcribed as one. The last key is the net's own index, so the order it
    // produces is total and stability cannot change the result — but the reference asks for a
    // stable sort and a later key change could make that matter.
    order.sort_by(compare_net_pins);
    order
}
