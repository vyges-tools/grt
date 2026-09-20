// SPDX-License-Identifier: Apache-2.0
//! The maze router — the stage that re-routes congested edges by shortest path.
//!
//! This module grows piece by piece alongside the reference's `maze.cpp`. The edge-cost tables
//! it prices with live in [`crate::mazecost`], built once per congestion iteration.

/// One of a net's edges, paired with the length that decides when it is routed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OrderNetEdge {
    /// ⚠️ The **routed** length — how many steps the current path takes — not the Manhattan
    /// distance between the endpoints. A detour therefore raises an edge's priority here.
    pub length: i32,
    pub edge_id: usize,
}

/// Order one net's edges **longest first** — the reference's `netedgeOrderDec`.
///
/// ⛔ **Stable, and the stability is the specification, not an implementation detail.** Edges of
/// equal routed length keep their index order, and equal lengths are common: a net whose edges
/// are all freshly routed straight lines has many. Sorting unstably would reorder them by
/// whatever the algorithm happened to do.
///
/// ⚠️ **Descending.** The longest edge is routed first, when the grid is least crowded by this
/// net's own new demand.
pub fn netedge_order_dec(routelens: &[i32]) -> Vec<OrderNetEdge> {
    let mut out: Vec<OrderNetEdge> = routelens
        .iter()
        .enumerate()
        .map(|(edge_id, &length)| OrderNetEdge { length, edge_id })
        .collect();
    out.sort_by(|a, b| b.length.cmp(&a.length));
    out
}

/// A node of the net's tree, as the heap setup reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MazeNode {
    pub x: i32,
    pub y: i32,
    /// Adjacent nodes and the tree edge reaching each, in the reference's stored order.
    pub neighbours: Vec<(usize, usize)>,
}

/// One tree edge's current route.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MazeEdge {
    pub n1: usize,
    pub n2: usize,
    pub routelen: usize,
    pub grids: Vec<(i32, i32)>,
}

/// What the heap setup produces: the two frontiers, in push order, and where each point came from.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Heaps {
    /// Seeds of the subtree containing the edge's first endpoint, in push order.
    pub src: Vec<(i32, i32)>,
    /// Seeds of the subtree containing its second endpoint.
    pub dest: Vec<(i32, i32)>,
    /// The tree edge each seeded point was taken from, in the order it was recorded.
    ///
    /// ⚠️ A point reached from two edges keeps the **last** write, which is why this is a list of
    /// assignments rather than a set.
    pub corr_edge: Vec<((i32, i32), usize)>,
}

/// Seed both search frontiers from the two subtrees the edge separates — the reference's
/// `setupHeap`.
///
/// The edge being re-routed splits its net's tree in two. Everything already routed on one side
/// becomes a **source**, everything on the other a **destination**, so the search that follows is
/// multi-source and multi-destination rather than point to point.
///
/// ⛔ **Push ORDER is behaviour.** Every seed is given a distance of zero, so the heap that
/// follows is entirely ties, and which one is popped first is decided by insertion order alone.
///
/// ⚠️ **A two-pin net skips the traversal entirely** — there is nothing else on either side.
/// 🔑 That shortcut is an optimisation, not a different rule: running the traversal on those nets
/// gives the same answer, and it is a mutation nothing can kill. Measured across the corpus,
/// every two-pin net has **exactly two nodes and one edge**, so the traversal has nowhere to go.
///
/// ⚠️ **Only points inside the enlarged region are seeded.** The rest of the subtree exists but
/// is out of reach of this search.
pub fn setup_heap(
    num_terminals: usize,
    nodes: &[MazeNode],
    edges: &[MazeEdge],
    edge_id: usize,
    (region_x1, region_x2, region_y1, region_y2): (i32, i32, i32, i32),
) -> Heaps {
    let mut h = Heaps::default();
    let in_region = |x: i32, y: i32| {
        x >= region_x1 && x <= region_x2 && y >= region_y1 && y <= region_y2
    };

    let (n1, n2) = (edges[edge_id].n1, edges[edge_id].n2);
    let (x1, y1) = (nodes[n1].x, nodes[n1].y);
    let (x2, y2) = (nodes[n2].x, nodes[n2].y);

    if num_terminals == 2 {
        // ⚠️ Seeded without the region test the traversal applies — a two-pin net's endpoints are
        // the edge's own, and the region was built around them.
        h.src.push((x1, y1));
        h.dest.push((x2, y2));
        return h;
    }

    // ⛔ `visited` is marked when a node is DEQUEUED, not when it is enqueued, and the check that
    // skips a neighbour reads it at enqueue time.
    //
    // 🔑 Marking on enqueue instead is a mutation nothing can kill, and the reason is structural:
    // every captured net satisfies `edges == nodes - 1`, so each node has exactly one parent in
    // the traversal and nothing is ever enqueued twice. Transcribed as written rather than
    // tightened — the reference does not enforce that invariant, it relies on it.
    let mut visited = vec![false; nodes.len()];

    let walk = |root: usize,
                    stop_at: usize,
                    seeds: &mut Vec<(i32, i32)>,
                    corr: &mut Vec<((i32, i32), usize)>,
                    visited: &mut Vec<bool>| {
        let (rx, ry) = (nodes[root].x, nodes[root].y);
        seeds.push((rx, ry));
        visited[root] = true;
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(root);

        while let Some(cur) = queue.pop_front() {
            visited[cur] = true;
            for &(nbr, edge) in &nodes[cur].neighbours {
                // ⛔ The far endpoint is the boundary between the two subtrees and is never
                // crossed — that is what keeps the two frontiers disjoint.
                if nbr == stop_at || visited[nbr] {
                    continue;
                }
                let e = &edges[edge];
                if e.routelen > 0 {
                    let (nx, ny) = (nodes[nbr].x, nodes[nbr].y);
                    if in_region(nx, ny) {
                        seeds.push((nx, ny));
                        corr.push(((nx, ny), edge));
                    }
                    // ⚠️ Interior points only: the route's two endpoints are the tree nodes
                    // themselves, and seeding those here would duplicate them.
                    for j in 1..e.routelen {
                        let (gx, gy) = e.grids[j];
                        if in_region(gx, gy) {
                            seeds.push((gx, gy));
                            corr.push(((gx, gy), edge));
                        }
                    }
                }
                queue.push_back(nbr);
            }
        }
    };

    walk(n1, n2, &mut h.src, &mut h.corr_edge, &mut visited);
    walk(n2, n1, &mut h.dest, &mut h.corr_edge, &mut visited);
    h
}
