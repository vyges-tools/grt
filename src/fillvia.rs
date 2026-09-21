// SPDX-License-Identifier: Apache-2.0
//! R19 — filling in the via stacks at a net's nodes.
//!
//! Layer assignment leaves every edge a path through the grid, but where an edge meets a node the
//! path starts or ends on whatever layer the edge was assigned. The pins at that node may sit on
//! other layers, and so may the node's other edges. This pass prepends and appends the stack of
//! vias that joins them, and gives a zero-length edge between two co-located nodes a stack of its
//! own.
//!
//! It is the only one of R19's five passes that **changes the routes**: it runs first, and the
//! via count, the overflow total, the checker and pin coverage all read what it leaves.
//!
//! ⛔ **Which edge carries a node's stack is decided by layer assignment's bookkeeping, not
//! here.** Each node records the edge at its HIGHEST layer (`h_id`) and, where no edge rises above
//! its own layer, the edge at its lowest (`l_id`). Only that one edge gets the stack; every other
//! edge at the node is left as it was.
//!
//! ⛔ **The two arms read DIFFERENT endpoints.** A positive-length edge reads its endpoints'
//! aliases (`n1a`/`n2a`); a zero-length edge reads the endpoints themselves (`n1`/`n2`), because
//! the aliases are never written for it, and resolves them through the node's `stack_alias` only
//! when the node carries no layers at all.

use crate::full3d::{Point3D, RouteType};

/// The reference's `BIG_INT`, the sentinel layer assignment leaves in `h_id` when no edge rises
/// above a node's own layer.
pub const NO_EDGE: i32 = 1_000_000_000;

/// One pin of a net, as the via-stack lookup reads it.
///
/// ⚠️ The reference holds these as `int` and narrows the layer to 16 bits when it compares.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ViaPin {
    pub x: i32,
    pub y: i32,
    pub layer: i32,
}

/// A tree node as this pass reads it — after layer assignment's bookkeeping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ViaNode {
    pub x: i16,
    pub y: i16,
    pub bot_layer: i16,
    pub top_layer: i16,
    /// The edge at the node's highest layer, or [`NO_EDGE`].
    pub h_id: i32,
    /// The edge at the node's lowest layer, or [`NO_EDGE`].
    pub l_id: i32,
    pub stack_alias: usize,
}

/// A tree edge as this pass reads and rewrites it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViaEdge {
    pub len: i32,
    pub n1: usize,
    pub n2: usize,
    /// The endpoints' aliases. ⚠️ Meaningful only on a positive-length edge.
    pub n1a: usize,
    pub n2a: usize,
    pub route_type: RouteType,
    pub routelen: i32,
    pub grids: Vec<Point3D>,
}

/// One net's tree and pins.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ViaNet {
    pub num_terminals: usize,
    pub nodes: Vec<ViaNode>,
    pub edges: Vec<ViaEdge>,
    pub pins: Vec<ViaPin>,
}

/// The pass's two counters. They reach the log only in verbose mode.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ViaCounts {
    /// "Via related to pin nodes".
    pub pin_nodes: i32,
    /// "Via related Steiner nodes".
    pub steiner_nodes: i32,
}

/// Which end of a positive-length edge took a node's stack, and whether that node is a terminal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndClaim {
    pub node: usize,
    pub terminal: bool,
}

/// What one edge's arm decided, returned so a trace can be lined up against it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeFill {
    /// A positive-length edge: which ends claimed a stack, and the point count it built.
    Positive { first: Option<EndClaim>, second: Option<EndClaim>, points: i32 },
    /// A zero-length edge whose resolved endpoints both carry no layers — skipped.
    ZeroSkipped { effective: (usize, usize) },
    /// A zero-length edge that reached the range test, with the range it computed. Written only
    /// when the top is above the bottom.
    Zero { effective: (usize, usize), bottom: i16, top: i16 },
}

/// The reference aborts the run here ("Edge has no previous routing").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NoPreviousRouting {
    pub net: usize,
    pub edge: usize,
}

/// The layer range of the pins at a node's position — `getViaStackRange`.
///
/// ⚠️ **Matched by POSITION, over every pin of the net**, not by the node's own pin: two pins
/// sharing the node's grid cell on different layers widen the range to both.
///
/// ⛔ **With no pin at the position the range comes back INVERTED** — bottom `i16::MAX`, top `-1`
/// — so folding it into another range with `min`/`max` changes nothing.
pub fn get_via_stack_range(pins: &[ViaPin], node: &ViaNode) -> (i16, i16) {
    let (x, y) = (i32::from(node.x), i32::from(node.y));
    let mut bot = i16::MAX;
    let mut top: i16 = -1;
    for p in pins {
        if p.x == x && p.y == y {
            // ⚠️ `static_cast<int16_t>` in the reference: a truncation, not a clamp.
            bot = bot.min(p.layer as i16);
            top = top.max(p.layer as i16);
        }
    }
    (bot, top)
}

/// Widen `[bot, top]` to take in the pins at `node` — the reference's `extendLayerRange` lambda.
fn extend_layer_range(pins: &[ViaPin], node: &ViaNode, bot: &mut i16, top: &mut i16) {
    let (pin_bot, pin_top) = get_via_stack_range(pins, node);
    *bot = pin_bot.min(*bot);
    *top = pin_top.max(*top);
}

/// Fill every net's via stacks — `fillVIA`.
///
/// A thin sequencer: nets in the given order, edges in index order, each to one of two arms by
/// length. ⛔ The gate is `len > 0`; a zero **or negative** length goes to the zero-length arm.
///
/// Returns the counters and, per net, what each edge's arm decided. On the reference's fatal error
/// the edges already processed stay rewritten, as they do there.
pub fn fill_via(
    nets: &mut [ViaNet],
    num_layers: i16,
) -> Result<(ViaCounts, Vec<Vec<EdgeFill>>), NoPreviousRouting> {
    let mut counts = ViaCounts::default();
    let mut decided = Vec::with_capacity(nets.len());
    for (net_id, net) in nets.iter_mut().enumerate() {
        let mut per_edge = Vec::with_capacity(net.edges.len());
        for edge_id in 0..net.edges.len() {
            let fill = if net.edges[edge_id].len > 0 {
                fill_positive_edge(net, edge_id, &mut counts)
                    .ok_or(NoPreviousRouting { net: net_id, edge: edge_id })?
            } else {
                fill_zero_length_edge(net, edge_id, num_layers)
            };
            per_edge.push(fill);
        }
        decided.push(per_edge);
    }
    Ok((counts, decided))
}

/// Does this edge carry `node`'s stack?
///
/// ⛔ Either it is the node's highest-layer edge, **or** it is the node's lowest-layer edge AND no
/// edge rises above the node AND the node is a terminal. The second arm is how a pin whose every
/// edge sits at or below its own layer still gets a stack; a Steiner node never takes it.
fn claims_stack(node: &ViaNode, node_id: usize, edge_id: usize, num_terminals: usize) -> bool {
    let e = edge_id as i32;
    node.h_id == e || (e == node.l_id && node.h_id == NO_EDGE && node_id < num_terminals)
}

/// A positive-length edge: stack at the first end, the path, stack at the second end.
///
/// Returns `None` where the reference raises its fatal error — a positive length with no steps.
fn fill_positive_edge(net: &mut ViaNet, edge_id: usize, counts: &mut ViaCounts) -> Option<EdgeFill> {
    let num_terminals = net.num_terminals;
    let edge = &net.edges[edge_id];
    let (node1, node2) = (edge.n1a, edge.n2a);
    let route_len = edge.routelen;
    let grids = &edge.grids;
    let mut tmp: Vec<Point3D> = Vec::new();
    let mut first = None;
    let mut second = None;

    if claims_stack(&net.nodes[node1], node1, edge_id, num_terminals) {
        let n = net.nodes[node1];
        let mut bottom = n.bot_layer;
        let mut top = n.top_layer;
        let init = grids[0].layer;
        let (x, y) = (grids[0].x, grids[0].y);
        first = Some(EndClaim { node: node1, terminal: node1 < num_terminals });
        if node1 < num_terminals {
            extend_layer_range(&net.pins, &n, &mut bottom, &mut top);
            // ⛔ UP from the bottom to one below the top, THEN back DOWN from the top to one
            // above the edge's own layer: the stack visits the top and returns. Only the upward
            // run is counted.
            for l in bottom..top {
                tmp.push(Point3D { x, y, layer: l });
                counts.pin_nodes += 1;
            }
            let mut l = top;
            while l > init {
                tmp.push(Point3D { x, y, layer: l });
                l -= 1;
            }
        } else {
            // ⚠️ A Steiner node's stack runs up from its bottom only; its top is not read.
            for l in bottom..init {
                tmp.push(Point3D { x, y, layer: l });
                counts.steiner_nodes += 1;
            }
        }
    }

    for j in 0..=route_len.max(-1) {
        tmp.push(grids[j as usize]);
    }

    // ⚠️ Raised AFTER the first stack and the path are built, so the counter above has already
    // moved. It aborts the run, so nothing below is reached.
    if route_len <= 0 {
        return None;
    }

    if claims_stack(&net.nodes[node2], node2, edge_id, num_terminals) {
        let n = net.nodes[node2];
        let mut bottom = n.bot_layer;
        let top_start = n.top_layer;
        let mut top = top_start;
        second = Some(EndClaim { node: node2, terminal: node2 < num_terminals });
        let last = *tmp.last().expect("the path pushed at least one point");
        if node2 < num_terminals {
            extend_layer_range(&net.pins, &n, &mut bottom, &mut top);
            // ⛔ The bottom is skipped when the path already ends on it, else it would repeat.
            if bottom == last.layer {
                bottom += 1;
            }
            // ⛔ DOWN from one below the path's last layer to one ABOVE the bottom, then UP from
            // the bottom to the top — so a pin range above the arrival layer is reached by going
            // down and coming back. ⚠️ Nothing goes up from an arrival BELOW the bottom: the
            // first run is empty and the second starts at the bottom, skipping the layers between.
            let mut l = last.layer - 1;
            while l > bottom {
                tmp.push(Point3D { x: last.x, y: last.y, layer: l });
                l -= 1;
            }
            for l in bottom..=top {
                tmp.push(Point3D { x: last.x, y: last.y, layer: l });
                counts.pin_nodes += 1;
            }
        } else {
            let mut l = top - 1;
            while l >= bottom {
                tmp.push(Point3D { x: last.x, y: last.y, layer: l });
                // ⛔ Transcribed as the reference has it: the Steiner count at the SECOND end is
                // gated on the FIRST end's node. A Steiner node here whose edge starts at a
                // terminal adds vias and counts none.
                if node1 >= num_terminals {
                    counts.steiner_nodes += 1;
                }
                l -= 1;
            }
        }
    }

    let points = tmp.len() as i32;
    // ⛔ The reference's guard reads "only if there were vias added", but it compares the POINT
    // count to the STEP count, which always differ by at least one — so every positive-length edge
    // is rewritten and restamped as a maze route, vias or not. The array is also trimmed to
    // exactly the points walked.
    if points != route_len {
        let edge = &mut net.edges[edge_id];
        edge.grids = tmp;
        edge.route_type = RouteType::MazeRoute;
        edge.routelen = points - 1;
    }
    Some(EdgeFill::Positive { first, second, points })
}

/// A node that carries no layers: layer assignment never touched it.
///
/// ⚠️ The top test is redundant in every reachable state — no assigned layer equals the layer
/// count — so dropping it is an EQUIVALENT mutation. Kept because the reference asks both.
fn has_no_layers(node: &ViaNode, num_layers: i16) -> bool {
    node.bot_layer == num_layers && node.top_layer == -1
}

/// A zero-length edge: a single stack at the shared position, or nothing.
fn fill_zero_length_edge(net: &mut ViaNet, edge_id: usize, num_layers: i16) -> EdgeFill {
    let num_terminals = net.num_terminals;
    let (node1, node2) = (net.edges[edge_id].n1, net.edges[edge_id].n2);
    // ⚠️ Resolved through the alias only when the node itself carries no layers — layer
    // assignment writes a co-located Steiner node's layers onto its alias, not onto it.
    let resolve = |n: usize| {
        if has_no_layers(&net.nodes[n], num_layers) { net.nodes[n].stack_alias } else { n }
    };
    let (eff1, eff2) = (resolve(node1), resolve(node2));

    if has_no_layers(&net.nodes[eff1], num_layers) && has_no_layers(&net.nodes[eff2], num_layers) {
        return EdgeFill::ZeroSkipped { effective: (eff1, eff2) };
    }

    // ⛔ BOTH ends read the BOTTOM layer — the top of the range is the higher of the two bottoms,
    // not either node's top. ⚠️ So where one end still carries no layers its bottom is the layer
    // COUNT, and that becomes the top.
    let (b1, b2) = (net.nodes[eff1].bot_layer, net.nodes[eff2].bot_layer);
    let mut bottom = b1.min(b2);
    let mut top = b1.max(b2);
    if eff1 < num_terminals {
        let n = net.nodes[eff1];
        extend_layer_range(&net.pins, &n, &mut bottom, &mut top);
    }
    if eff2 < num_terminals {
        let n = net.nodes[eff2];
        extend_layer_range(&net.pins, &n, &mut bottom, &mut top);
    }

    let fill = EdgeFill::Zero { effective: (eff1, eff2), bottom, top };
    if top <= bottom {
        return fill;
    }

    // ⚠️ Placed at the UNRESOLVED first endpoint's position. The edge has zero length and an
    // alias is co-located by definition, so the resolved node gives the same point — an
    // EQUIVALENT mutation, not a gap. Transcribed as the reference reads it.
    let (x, y) = (net.nodes[node1].x, net.nodes[node1].y);
    let edge = &mut net.edges[edge_id];
    edge.grids = (bottom..=top).map(|l| Point3D { x, y, layer: l }).collect();
    edge.route_type = RouteType::MazeRoute;
    edge.routelen = i32::from(top - bottom);
    fill
}
