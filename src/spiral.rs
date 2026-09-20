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
    /// Edges registered on this node, in registration order.
    pub edges: Vec<usize>,
}

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
            edges: Vec::new(),
        };
        if d < num_terminals {
            let layer = pin_layers[d];
            node.bot_layer = layer;
            node.top_layer = layer;
            node.assigned = true;
            // ⚠️ 2, not 0 — and unrelated to the horizontal-connection meaning the same field
            // carries during L-routing.
            node.status = 2;
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

    let mut enqueue_node_edges = |node_alias: usize,
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
