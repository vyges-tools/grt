// SPDX-License-Identifier: Apache-2.0
//! Stage R9 — preparing each net's tree for layer assignment, and walking it outward from the
//! pins.
//!
//! Four passes, and none of them can be folded into another:
//!
//! 1. reset every node, give terminals their pin layer, and collapse coincident Steiner nodes
//!    onto a single **alias**;
//! 2. register each edge on its endpoints' **alias** nodes, not on the endpoints themselves;
//! 3. rip up the net, then **breadth-first from the terminals outward**, routing each edge as it
//!    is dequeued;
//! 4. copy each alias node's final state back onto the nodes that share it.
//!
//! ⛔ **This stage RESETS `status` to 0 and then sets terminals to 2.** The vertical/horizontal
//! connection state the L-routing pass built is therefore **discarded** here — the field is
//! reused with a different meaning from this point on. Carrying it forward would change the
//! traversal.

use crate::estimate::{congestion_cost, EstimateGrid, LShape};
use crate::lroute::{mark_h, mark_v, via_bias, EdgeRoute};

/// How many edges one node can carry.
///
/// ⚠️ The reference stores `eID` as a fixed `int[10]` and appends with `eID[conCNT++]`, with no
/// bounds check. Aliasing merges several nodes' edges onto one node, so the count here is not
/// bounded by the degree-3 Steiner tree — it is bounded only by this constant. Kept as a named
/// constant so an overflow is a visible assertion rather than a silent one.
pub const MAX_CONNECTIONS: usize = 10;

/// A tree node as this stage needs it.
///
/// ⛔ The widths are the reference's: `x`, `y`, `status`, `botL` and `topL` are all `int16_t`
/// there. Holding them wider here would let a value survive that the reference truncates.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpiralNode {
    pub x: i16,
    pub y: i16,
    pub top_layer: i16,
    pub bot_layer: i16,
    pub assigned: bool,
    /// ⛔ The node whose state this one shares. A Steiner node sitting exactly on an earlier
    /// node's coordinate aliases to it; everything else aliases to itself.
    pub stack_alias: usize,
    pub status: i16,
    /// Counts of edges leaving this node horizontally and "lower" (the reference's `hID` and
    /// `lID`). ⚠️ Declared `-1` in the reference's struct but reset to 0 by this stage, and
    /// incremented only on the **alias** node, never the node itself.
    pub h_id: i32,
    pub l_id: i32,
    /// Edges registered on this node, in registration order.
    pub edges: Vec<usize>,
}

/// How the reset differs between the two stages that perform it.
///
/// ⛔ **Two stages reset a net's nodes in exactly the same way bar two values**: the outward walk
/// before per-edge routing, and layer assignment before the 3D pass. A literal diff of the two
/// reference functions shows every other line identical, so they share one implementation here
/// rather than a copy that can drift.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResetParams {
    /// What the two per-node connection counters start at.
    pub counter_init: i32,
    /// The status a terminal is given.
    pub terminal_status: i16,
}

/// The outward walk's reset: counters at zero, terminals marked **2**.
pub const WALK_RESET: ResetParams = ResetParams { counter_init: 0, terminal_status: 2 };

/// Layer assignment's reset: counters at **infinity**, terminals marked **1**.
///
/// ⚠️ They are not counters in this stage. Layer assignment uses the two fields to hold the
/// **edge id at the node's highest and lowest layer**, and the sentinel means "no edge above (or
/// below) this node's own layer yet" — which is why it starts at infinity rather than zero.
pub const LAYER_RESET: ResetParams =
    ResetParams { counter_init: 1_000_000_000, terminal_status: 1 };

/// Reset the nodes and resolve the aliases — pass 1.
///
/// ⚠️ **Terminals come first and always get their own entry**, because nodes are ordered
/// terminals-then-Steiner and only Steiner nodes look for a coincident predecessor. So a Steiner
/// node landing on a pin aliases to the pin, never the other way round.
///
/// ⚠️ The coincidence search is **linear over the points recorded so far** and takes the FIRST
/// match. With three coincident nodes the second and third both alias to the first, not to each
/// other.
pub fn reset_and_alias(
    coords: &[(i16, i16)],
    num_terminals: usize,
    pin_layers: &[i16],
    num_layers: i16,
    params: ResetParams,
) -> Vec<SpiralNode> {
    let mut nodes: Vec<SpiralNode> = Vec::with_capacity(coords.len());
    // the points recorded so far, as (x, y, node index)
    let mut points: Vec<(i16, i16, usize)> = Vec::new();

    for (d, &(x, y)) in coords.iter().enumerate() {
        let mut node = SpiralNode {
            x,
            y,
            top_layer: -1,
            bot_layer: num_layers,
            assigned: false,
            stack_alias: d,
            status: 0,
            h_id: params.counter_init,
            l_id: params.counter_init,
            edges: Vec::new(),
        };
        if d < num_terminals {
            let layer = pin_layers[d];
            node.bot_layer = layer;
            node.top_layer = layer;
            node.assigned = true;
            // ⚠️ Not 0, and unrelated to the horizontal-connection meaning the same field
            // carries during L-routing. The value differs between the two stages that do this.
            node.status = params.terminal_status;
            points.push((x, y, d));
        } else if let Some(&(_, _, first)) = points.iter().find(|&&(px, py, _)| px == x && py == y)
        {
            node.stack_alias = first;
        } else {
            points.push((x, y, d));
        }
        nodes.push(node);
    }
    nodes
}

/// What pass 2 leaves on one edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeReg {
    /// The endpoints' alias nodes.
    ///
    /// ⛔ **`None` for a zero-length edge, because the reference never writes them.** It assigns
    /// `n1a`/`n2a` only in the positive-length arm; a zero-length edge keeps whatever its fields
    /// were default-initialised to. Every one of the 407 zero-length edges in the corpus reads
    /// back as `0`, but `0` is also a valid node index, so storing it would be indistinguishable
    /// from a real answer. Nothing downstream may read this: the traversal only ever looks at
    /// edges it dequeued, and a zero-length edge is never enqueued.
    pub alias: Option<(usize, usize)>,
    pub assigned: bool,
}

/// Register each edge on its endpoints' alias nodes — pass 2.
///
/// ⚠️ **Zero-length edges are marked assigned and registered nowhere**, so the traversal never
/// visits them. Registering them would put edges in the queue that route to nothing.
pub fn register_edges(
    nodes: &mut [SpiralNode],
    edges: &[(usize, usize, i32)], // (n1, n2, len)
) -> Vec<EdgeReg> {
    let mut out = Vec::with_capacity(edges.len());
    for (edge_id, &(n1, n2, len)) in edges.iter().enumerate() {
        if len > 0 {
            let a1 = nodes[n1].stack_alias;
            let a2 = nodes[n2].stack_alias;
            nodes[a1].edges.push(edge_id);
            nodes[a2].edges.push(edge_id);
            debug_assert!(
                nodes[a1].edges.len() <= MAX_CONNECTIONS
                    && nodes[a2].edges.len() <= MAX_CONNECTIONS,
                "eID overflow: the reference would write past the end of its array"
            );
            out.push(EdgeReg { alias: Some((a1, a2)), assigned: false });
        } else {
            out.push(EdgeReg { alias: None, assigned: true });
        }
    }
    out
}

/// Walk the tree outward from the terminals, yielding edges in the order they are routed — pass 3.
///
/// ⛔ **Breadth-first, seeded from every terminal in index order**, and an edge is marked assigned
/// when it is *enqueued*, not when it is processed. That is what stops an edge being queued twice
/// from its two ends, and it means the order depends on the registration order of pass 2.
///
/// ⚠️ **The next node is the endpoint that is NOT yet assigned**, chosen as
/// `n1a.assigned ? n2a : n1a`. When both are already assigned the expression still picks `n2a`,
/// and the guard that follows discards it — so the traversal never revisits.
///
/// 🔑 **Testing the other endpoint instead is equivalent, and provably so.** An edge is only ever
/// queued from inside the expansion of a node that was just marked assigned, and assignment is
/// never undone, so at least one endpoint is assigned by the time the edge is popped. When
/// exactly one is unassigned both forms select it; when both are assigned the two forms differ
/// only in which already-assigned node they name, and the guard below discards either. So no
/// corpus can distinguish them — swapping them survives every mutation run, and it should.
pub fn traversal_order(
    nodes: &mut [SpiralNode],
    edges: &[EdgeReg],
    num_terminals: usize,
) -> Vec<usize> {
    let mut assigned: Vec<bool> = edges.iter().map(|e| e.assigned).collect();
    let mut queue: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
    let mut order = Vec::new();

    let enqueue_node_edges = |node_alias: usize,
                                  nodes: &[SpiralNode],
                                  assigned: &mut Vec<bool>,
                                  queue: &mut std::collections::VecDeque<usize>| {
        for &eid in &nodes[node_alias].edges {
            if !assigned[eid] {
                queue.push_back(eid);
                assigned[eid] = true;
            }
        }
    };

    for node_id in 0..num_terminals {
        nodes[node_id].assigned = true;
        enqueue_node_edges(node_id, nodes, &mut assigned, &mut queue);
    }

    while let Some(edge_id) = queue.pop_front() {
        order.push(edge_id);
        // Safe by construction: only positive-length edges are ever enqueued, and those are
        // exactly the ones pass 2 gave an alias pair.
        let (a1, a2) = edges[edge_id].alias.expect("dequeued edge has aliases");
        let next = if nodes[a1].assigned { a2 } else { a1 };
        if !nodes[next].assigned {
            enqueue_node_edges(next, nodes, &mut assigned, &mut queue);
            nodes[next].assigned = true;
        }
    }
    order
}

/// Copy each alias node's state back onto the nodes sharing it — pass 4.
///
/// ⚠️ Only `status` is propagated. Layers and assignment stay per node.
pub fn propagate_alias_status(nodes: &mut [SpiralNode]) {
    // Taking a snapshot is equivalent to the reference's in-place loop: an alias node is always
    // its own alias, so the source of every copy is a node this loop never writes to.
    let statuses: Vec<i16> = nodes.iter().map(|n| nodes[n.stack_alias].status).collect();
    for (node, status) in nodes.iter_mut().zip(statuses) {
        node.status = status;
    }
}

/// Mark a node **and its alias** as connected vertically.
///
/// ⛔ The alias is marked too, which is what separates this stage from the earlier L re-route:
/// coincident nodes share a location, so a segment arriving at one arrives at all of them.
fn mark_vertical(nodes: &mut [SpiralNode], n: usize, na: usize) {
    mark_v(&mut nodes[n].status);
    mark_v(&mut nodes[na].status);
}

/// Mark a node **and its alias** as connected horizontally.
fn mark_horizontal(nodes: &mut [SpiralNode], n: usize, na: usize) {
    mark_h(&mut nodes[n].status);
    mark_h(&mut nodes[na].status);
}

/// Route one tree edge as the outward walk reaches it.
///
/// The body is the earlier L re-route's, with three differences that are the whole point of the
/// stage:
///
/// - ⛔ **the via bias is unconditional.** The earlier pass wraps it in `viaGuided`; here there is
///   no guard. ⚠️ **But it is dead in the shipped flow**: `spiralRouteAll` has exactly one call
///   site, and the router sets `via_cost_ = 0` before it and only raises it to 1 much later, in
///   the 3D phase. Measured: all 16,568 captured calls across four designs carry `via_cost_ = 0`,
///   so the bias contributes nothing and the difference from the earlier pass cannot be observed.
///   Transcribed anyway, and marked as unwitnessed rather than claimed as validated.
/// - ⛔ **every mark also lands on the alias**, so coincident nodes share connection state.
/// - ⛔ **`hID` and `lID` are incremented on the ALIAS nodes**, and crossed: the shape that leaves
///   `n1` vertically counts a horizontal arrival at `n2a` and a "lower" departure at `n1a`.
///
/// ⚠️ **The degenerate arm is not a no-op**: a zero-length edge is explicitly set to "no route",
/// not left at whatever the previous iteration wrote.
#[allow(clippy::too_many_arguments)]
pub fn spiral_route(
    grid: &mut EstimateGrid,
    nodes: &mut [SpiralNode],
    edge: (usize, usize, i32),
    edge_cost: i8,
    via_cost: f64,
    v_lb: f32,
    h_lb: f32,
    red_v: &dyn Fn(usize, usize) -> u16,
    red_h: &dyn Fn(usize, usize) -> u16,
) -> EdgeRoute {
    let (n1, n2, len) = edge;
    if len <= 0 {
        return EdgeRoute::None;
    }

    // ⚠️ Widened here because the reference widens here — the fields are `int16_t`, the locals
    // `int`.
    let (x1, y1) = (i32::from(nodes[n1].x), i32::from(nodes[n1].y));
    let (x2, y2) = (i32::from(nodes[n2].x), i32::from(nodes[n2].y));
    let n1a = nodes[n1].stack_alias;
    let n2a = nodes[n2].stack_alias;
    let (ymin, ymax) = (y1.min(y2), y1.max(y2));
    let cost = f64::from(edge_cost);

    if x1 == x2 {
        grid.update_v(x1, ymin, ymax, cost);
        mark_vertical(nodes, n1, n1a);
        mark_vertical(nodes, n2, n2a);
        return EdgeRoute::Vertical;
    }
    if y1 == y2 {
        grid.update_h(x1, x2, y1, cost);
        mark_horizontal(nodes, n1, n1a);
        mark_horizontal(nodes, n2, n2a);
        return EdgeRoute::Horizontal;
    }

    // ⛔ No `viaGuided` guard here, unlike the earlier pass.
    let (mut cost_l1, mut cost_l2) = via_bias(nodes[n1].status, nodes[n2].status, via_cost);

    for j in ymin..ymax {
        cost_l1 += congestion_cost(grid.v(x1 as usize, j as usize), red_v(x1 as usize, j as usize), v_lb);
        cost_l2 += congestion_cost(grid.v(x2 as usize, j as usize), red_v(x2 as usize, j as usize), v_lb);
    }
    // ⚠️ Not `min..max` — the reference writes `for (int j = x1; j < x2; j++)` with no ordering,
    // so this loop contributes nothing at all when `x1 > x2`. The vertical loop above DOES order
    // its bounds. The asymmetry is the reference's.
    for j in x1..x2 {
        cost_l1 += congestion_cost(grid.h(j as usize, y2 as usize), red_h(j as usize, y2 as usize), h_lb);
        cost_l2 += congestion_cost(grid.h(j as usize, y1 as usize), red_h(j as usize, y1 as usize), h_lb);
    }

    if chooses_y_first(cost_l1, cost_l2) {
        mark_vertical(nodes, n1, n1a);
        mark_horizontal(nodes, n2, n2a);
        nodes[n2a].h_id += 1;
        nodes[n1a].l_id += 1;
        grid.update_v(x1, ymin, ymax, cost);
        grid.update_h(x1, x2, y2, cost);
        EdgeRoute::L(LShape::YFirst)
    } else {
        mark_vertical(nodes, n2, n2a);
        mark_horizontal(nodes, n1, n1a);
        nodes[n1a].h_id += 1;
        nodes[n2a].l_id += 1;
        grid.update_h(x1, x2, y1, cost);
        grid.update_v(x2, ymin, ymax, cost);
        EdgeRoute::L(LShape::XFirst)
    }
}

/// Which way the bend goes, given the two candidate costs.
///
/// ⛔ **A tie goes to X-first**, as everywhere else in this router: the reference tests
/// `costL1 < costL2` and takes the else branch on equality. Ties are not rare — with
/// `via_cost_` at zero and an uncongested region both costs are exactly `0.0`.
///
/// Split out so the comparison can be replayed against costs captured from the reference, which
/// is the only part of the L arm a corpus can check without reconstructing the demand grid.
pub fn chooses_y_first(cost_l1: f64, cost_l2: f64) -> bool {
    cost_l1 < cost_l2
}

/// Reset a net's nodes before recording which edges reach its extreme layers.
///
/// ⚠️ **A partial reset, not the full one.** The aliases resolved earlier are left in place —
/// only the per-node layer bookkeeping is cleared. Re-resolving them here would be harmless but
/// wasteful, and the reference does not.
pub fn reset_for_layer_extremes(
    nodes: &mut [SpiralNode],
    num_terminals: usize,
    pin_layers: &[i16],
    num_layers: i16,
) {
    for (d, node) in nodes.iter_mut().enumerate() {
        node.top_layer = -1;
        node.bot_layer = num_layers;
        node.edges.clear();
        node.h_id = LAYER_RESET.counter_init;
        node.l_id = LAYER_RESET.counter_init;
        node.status = 0;
        node.assigned = false;
        if d < num_terminals {
            node.bot_layer = pin_layers[d];
            node.top_layer = pin_layers[d];
            node.assigned = true;
            node.status = LAYER_RESET.terminal_status;
        }
    }
}

/// Which layer each end of one edge arrives on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeLayers {
    pub n1: usize,
    pub n2: usize,
    pub len: i32,
    /// The layer at the edge's first grid point, which meets `n1`.
    pub first: i16,
    /// The layer at its last, which meets `n2`.
    pub last: i16,
}

/// What a node ends up connected to, vertically.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LayerExtremes {
    pub top_layer: i16,
    pub bot_layer: i16,
    /// The edge reaching this node's **highest** layer, or the sentinel if none goes above it.
    pub h_id: i32,
    /// The edge reaching its **lowest**, or the sentinel if none goes below.
    pub l_id: i32,
}

/// Record, per node, which edges reach its highest and lowest layers.
///
/// ⛔ **Both ends of every edge are recorded, on the ALIAS nodes.** An edge contributes the layer
/// of its first grid point to one end and of its last to the other, so a via along the way is
/// invisible here — only where the edge *meets* each node matters.
///
/// ⛔ **The comparisons are strict, so the first edge to reach a layer keeps it.** A later edge
/// arriving on the same layer does not displace it, which is what makes the result depend on edge
/// order rather than only on the layers.
///
/// ⚠️ **A terminal starts at its own pin layer, not at infinity**, so an edge that stays on that
/// layer sets neither field — and the sentinel survives, meaning "nothing above (or below) the
/// pin". A Steiner node starts wide open and is always set by its first edge.
pub fn record_layer_extremes(nodes: &mut [SpiralNode], edges: &[EdgeLayers]) {
    for (edge_id, e) in edges.iter().enumerate() {
        if e.len <= 0 {
            continue;
        }
        for (node, layer) in [(e.n1, e.first), (e.n2, e.last)] {
            let a = nodes[node].stack_alias;
            nodes[a].edges.push(edge_id);
            if layer > nodes[a].top_layer {
                nodes[a].h_id = edge_id as i32;
                nodes[a].top_layer = layer;
            }
            if layer < nodes[a].bot_layer {
                nodes[a].l_id = edge_id as i32;
                nodes[a].bot_layer = layer;
            }
            nodes[a].assigned = true;
        }
    }
}

/// Read one node's layer bookkeeping back out.
pub fn layer_extremes(node: &SpiralNode) -> LayerExtremes {
    LayerExtremes {
        top_layer: node.top_layer,
        bot_layer: node.bot_layer,
        h_id: node.h_id,
        l_id: node.l_id,
    }
}
