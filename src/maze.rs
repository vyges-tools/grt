// SPDX-License-Identifier: Apache-2.0
//! The maze router — the stage that re-routes congested edges by shortest path.
//!
//! This module grows piece by piece alongside the reference's `maze.cpp`. The edge-cost tables
//! it prices with live in [`crate::mazecost`], built once per congestion iteration.

use crate::estimate::EstimateGrid;

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
    /// The node whose connection state this one shares — see the outward walk's aliasing.
    pub stack_alias: usize,
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

/// The reference's `BIG_INT` — **1e9**, declared as an `int` and used as infinity.
///
/// ⛔ Not `i32::MAX`, which is roughly twice as large and the plausible-looking guess. Costs here
/// never approach either, so no captured case distinguishes them — which is exactly why the
/// constant has to be read rather than assumed.
pub const BIG_INT: f64 = 1e9;

// ─── The heap — shared by the 2D search (R14, `double` distances) and the 3D search (R18c,
// `int` distances) ───────────────────────────────────────────────────────────────────────────
//
// ⭐ **One implementation, two reference copies.** `maze.cpp`'s `heapify` / `updateHeap` /
// `removeMin` and `maze3D.cpp`'s `heapify3D` / `updateHeap3D` / `removeMin3D` are TEXTUALLY
// IDENTICAL once the element type (`double*` vs `int*`) and the `3D` suffix are normalised — a
// literal diff leaves two comments. The only real difference is the key type, so the functions are
// generic over it rather than copied: a copy is a place for the two to drift. The 2D instance is
// validated against R14's goldens; the 3D instance is the same code over `i32`, and R18e's search
// capture exercises it end to end.

fn parent_index(i: usize) -> usize {
    (i - 1) / 2
}
fn left_index(i: usize) -> usize {
    2 * i + 1
}
fn right_index(i: usize) -> usize {
    2 * i + 2
}

/// Sift the element at `i` up until its parent is no larger — the reference's `updateHeap`.
///
/// ⚠️ Compares the **live** distance each cell currently holds, which is why the heap has to be
/// repaired by position when a cell's distance improves rather than simply re-pushed.
pub fn update_heap<K: PartialOrd + Copy>(heap: &mut [usize], mut i: usize, dist: &[K]) {
    let tmp = heap[i];
    while i > 0 && dist[heap[parent_index(i)]] > dist[tmp] {
        let parent = parent_index(i);
        heap[i] = heap[parent];
        i = parent;
    }
    heap[i] = tmp;
}

/// Sift the root down — the reference's `heapify`.
///
/// ⚠️ Written as a hole being pushed down, comparing children against the value held aside
/// rather than against whatever currently sits in the hole.
pub fn heapify<K: PartialOrd + Copy>(heap: &mut [usize], dist: &[K]) {
    if heap.is_empty() {
        return;
    }
    let heap_size = heap.len();
    let mut i = 0usize;
    let tmp = heap[i];
    loop {
        let (l, r) = (left_index(i), right_index(i));
        let smallest = if l < heap_size && dist[heap[l]] < dist[tmp] {
            if r < heap_size && dist[heap[r]] < dist[heap[l]] {
                r
            } else {
                l
            }
        } else if r < heap_size && dist[heap[r]] < dist[tmp] {
            r
        } else {
            i
        };
        if smallest != i {
            heap[i] = heap[smallest];
            i = smallest;
        } else {
            heap[i] = tmp;
            return;
        }
    }
}

/// Remove the smallest element — the reference's `removeMin`.
///
/// The last element is copied to the root and the heap is repaired **while that element is still
/// present at the end**; only then is the tail dropped.
///
/// ⚠️ **CORRECTED 2026-09-21 — this is EQUIVALENT to the idiomatic swap-pop-sift.** An earlier
/// version of this comment said the difference was observable; a mutation proved otherwise, and
/// the reason is a property of the code: the value held aside IS the tail element, so the stale
/// copy is only ever compared against itself or against a child already smaller than it, and
/// under strict `<` it is never selected. The hole therefore never enters the last slot, and the
/// `pop` removes exactly the stale copy. Transcribed as the reference writes it anyway.
pub fn remove_min<K: PartialOrd + Copy>(heap: &mut Vec<usize>, dist: &[K]) {
    if heap.is_empty() {
        return;
    }
    heap[0] = heap[heap.len() - 1];
    heapify(heap, dist);
    heap.pop();
}

/// The search state one edge's re-route works over.
///
/// ⚠️ **Two separate parent grids, chosen by the direction of the move that reached a cell**, with
/// `hv` recording which of them holds that cell's parent. A single parent grid would lose the
/// distinction the backtrace later depends on.
#[derive(Debug, Clone)]
pub struct MazeSearch {
    pub width: usize,
    pub dist: Vec<f64>,
    /// Parent of a cell reached by a **vertical** move.
    pub parent_x1: Vec<i32>,
    pub parent_y1: Vec<i32>,
    /// Parent of a cell reached by a **horizontal** move.
    pub parent_x3: Vec<i32>,
    pub parent_y3: Vec<i32>,
    /// `true` when the cell was reached vertically, i.e. its parent is in the first pair.
    pub hv: Vec<bool>,
    pub hyper_h: Vec<bool>,
    pub hyper_v: Vec<bool>,
    pub heap: Vec<usize>,
}

impl MazeSearch {
    pub fn new(width: usize, height: usize) -> Self {
        let n = width * height;
        MazeSearch {
            width,
            dist: vec![BIG_INT; n],
            parent_x1: vec![-1; n],
            parent_y1: vec![-1; n],
            parent_x3: vec![-1; n],
            parent_y3: vec![-1; n],
            hv: vec![false; n],
            hyper_h: vec![false; n],
            hyper_v: vec![false; n],
            heap: Vec::new(),
        }
    }

    pub fn at(&self, x: i32, y: i32) -> usize {
        y as usize * self.width + x as usize
    }

    /// Lower the cost of one adjacent cell, if this path reaches it more cheaply — the
    /// reference's `updateAdjacent`.
    ///
    /// ⛔ **The heap is repaired two different ways.** A cell never reached before is pushed and
    /// sifted up from the end; a cell already in the heap is **found by scanning** and sifted from
    /// where it sits. Re-pushing instead would leave a stale entry behind.
    ///
    /// ⚠️ The reference distinguishes those two cases by testing the cell's **previous** distance
    /// against `BIG_INT`. Its second test, that the previous distance exceeds the new one, is
    /// already guaranteed by the early return above it.
    pub fn update_adjacent(
        &mut self,
        (cur_x, cur_y): (i32, i32),
        (adj_x, adj_y): (i32, i32),
        cost: f64,
    ) -> Result<(), String> {
        let adj = self.at(adj_x, adj_y);
        let adj_cost = self.dist[adj];
        if adj_cost <= cost {
            return Ok(());
        }
        self.dist[adj] = cost;

        if cur_x != adj_x {
            self.parent_x3[adj] = cur_x;
            self.parent_y3[adj] = cur_y;
            self.hv[adj] = false;
        } else {
            self.parent_x1[adj] = cur_x;
            self.parent_y1[adj] = cur_y;
            self.hv[adj] = true;
        }

        if adj_cost >= BIG_INT {
            self.heap.push(adj);
            let last = self.heap.len() - 1;
            update_heap(&mut self.heap, last, &self.dist);
        } else {
            match self.heap.iter().position(|&c| c == adj) {
                Some(pos) => update_heap(&mut self.heap, pos, &self.dist),
                // The reference raises an error here and names the net; a cell with a finite
                // distance that is absent from the heap means the two have gone out of step.
                None => return Err(format!("cell ({adj_x},{adj_y}) is not in the heap")),
            }
        }
        Ok(())
    }
}

/// What one relaxation step needs from the grids it prices against.
pub struct RelaxInputs<'a> {
    /// `L` — how heavily the previous round's usage is blended into this one's.
    pub l: i32,
    /// The via penalty for turning.
    pub via: f64,
    pub h_capacity: i32,
    pub v_capacity: i32,
    pub params: &'a crate::mazecost::CostParams,
    pub used_h: &'a dyn Fn(i32, i32) -> i32,
    pub used_v: &'a dyn Fn(i32, i32) -> i32,
    pub last_h: &'a dyn Fn(i32, i32) -> i32,
    pub last_v: &'a dyn Fn(i32, i32) -> i32,
}

/// Relax one step from `cur` in direction `(d_x, d_y)` — the reference's `relaxAdjacent`.
///
/// ⛔ **The edge crossed is indexed at its LOWER endpoint**, so a step in the negative direction
/// prices the edge *behind* the cell, not the one at it. That is the same rule the route walk and
/// the rip-up undo use, stated a third way here as `cur - (d == -1)`.
///
/// ⛔ **The usage looked up blends this round with the previous one**: `usage + L * last_usage`.
/// This is where the carried-over demand the per-round reset clears is actually consumed.
///
/// ⛔ **The via guard here is redundant, and only the CALL SEQUENCE shows it.** Read alone this
/// says "a turn is free at a source". Read from the caller, that case never arrives: the caller
/// derives the flag as `pre != cur`, and initialises `pre` **to `cur`** exactly when the distance
/// is zero — so the flag is already false there. Measured: of 1,855 relaxations starting from a
/// source, **none** requests a via, and removing this guard is a mutation nothing can kill.
///
/// Transcribed anyway, because it is what the reference writes and a future caller could reach it.
///
/// ⛔ **The hyper test truncates to an integer.** The reference stores the competing cost in an
/// `int` before comparing it against a `double`, so any fractional part is discarded and the
/// comparison is coarser than it looks. Transcribed, not corrected.
#[allow(clippy::too_many_arguments)]
pub fn relax_adjacent(
    s: &mut MazeSearch,
    (cur_x, cur_y): (i32, i32),
    (d_x, d_y): (i32, i32),
    add_via: bool,
    maybe_hyper: bool,
    inp: &RelaxInputs<'_>,
) -> Result<(), String> {
    let is_horizontal = d_x != 0;
    let capacity = if is_horizontal { inp.h_capacity } else { inp.v_capacity };

    // p1 is the edge this step crosses; p2 is the one on the far side of `cur`.
    let p1_x = cur_x - i32::from(d_x == -1);
    let p1_y = cur_y - i32::from(d_y == -1);
    let p2_x = cur_x - i32::from(d_x == 1);
    let p2_y = cur_y - i32::from(d_y == 1);

    let usage = |x: i32, y: i32| {
        if is_horizontal {
            (inp.used_h)(x, y) + inp.l * (inp.last_h)(x, y)
        } else {
            (inp.used_v)(x, y) + inp.l * (inp.last_v)(x, y)
        }
    };

    let cur = s.at(cur_x, cur_y);
    let cost1 = crate::mazecost::get_cost(usage(p1_x, p1_y), capacity, inp.params);
    let mut tmp = s.dist[cur] + cost1;

    if add_via && s.dist[cur] != 0.0 {
        tmp += inp.via;

        if maybe_hyper {
            let cost2 = crate::mazecost::get_cost(usage(p2_x, p2_y), capacity, inp.params);
            let back = s.at(cur_x - d_x, cur_y - d_y);
            // ⛔ Truncated to an integer by the reference before the comparison.
            let tmp_cost = (s.dist[back] + cost2) as i32;
            if f64::from(tmp_cost) < s.dist[cur] + inp.via {
                let hyper = if is_horizontal { &mut s.hyper_h } else { &mut s.hyper_v };
                hyper[cur] = true;
            }
        }
    }

    s.update_adjacent((cur_x, cur_y), (cur_x + d_x, cur_y + d_y), tmp)
}

/// Walk the recorded parents back from the meeting point to the source subtree.
///
/// The search stops the moment it pops a cell that already belongs to the destination subtree, so
/// the meeting point is **already on** that subtree and a single walk back to a source cell is
/// the whole new route. There is no second traversal.
///
/// ⛔ **Only one coordinate changes per step.** Every move the search made was axis-aligned, so
/// the parent differs from its child in exactly one axis — and the reference updates only that
/// one, taking the other from where it already is. Copying both from the parent grid would read
/// a coordinate that grid never wrote.
///
/// ⛔ **A "hyper" cell jumps one more step in the direction just travelled** rather than
/// consulting its parent: `cur = 2 * cur - previous` reflects the last step forward again. The
/// parent step is skipped entirely when a jump happens — a jumped cell has no parent to consult.
///
/// ⚠️ **The reflection is what makes the walk terminate.** Shortening the jump so the position
/// does not advance — `cur = previous`, say — leaves the loop spinning rather than producing a
/// wrong path: the guard here is the arithmetic, not a bound on the loop.
///
/// ⚠️ **The two jump tests are sequentially dependent** — the vertical one reads the hyper grid at
/// a column the horizontal one may have just changed.
///
/// 🔑 **But they can never both fire in the same step, and that is structural.** After a parent
/// step the cell differs from the previous one in exactly one axis; after a jump it differs in
/// none. So at most one of the two movement tests can be true. Measured: **0 of 744** captured
/// jumps move both axes. Swapping the two tests is therefore a mutation nothing can kill — an
/// equivalence, not a gap.
///
/// ⚠️ The path is built backwards and reversed, then the meeting point is appended — so the
/// meeting point appears exactly once, at the end, and is never pushed by the loop.
pub fn backtrace(s: &MazeSearch, cross: (i32, i32)) -> Vec<(i32, i32)> {
    let (mut cur_x, mut cur_y) = cross;
    // ⚠️ The reference leaves these uninitialised and relies on the first iteration not reading
    // them. Named here so that reliance is visible rather than accidental.
    let (mut prev_x, mut prev_y) = (i32::MIN, i32::MIN);
    let mut reversed: Vec<(i32, i32)> = Vec::new();
    let mut first = true;

    while s.dist[s.at(cur_x, cur_y)] != 0.0 {
        let mut jumped = false;
        if !first {
            if cur_x != prev_x && s.hyper_h[s.at(cur_x, cur_y)] {
                cur_x = 2 * cur_x - prev_x;
                jumped = true;
            }
            if cur_y != prev_y && s.hyper_v[s.at(cur_x, cur_y)] {
                cur_y = 2 * cur_y - prev_y;
                jumped = true;
            }
        }
        prev_x = cur_x;
        prev_y = cur_y;
        if !jumped {
            let here = s.at(prev_x, prev_y);
            if s.hv[here] {
                cur_y = s.parent_y1[here];
            } else {
                cur_x = s.parent_x3[here];
            }
        }
        reversed.push((cur_x, cur_y));
        first = false;
    }

    let mut grids: Vec<(i32, i32)> = reversed.into_iter().rev().collect();
    grids.push(cross);
    grids
}

/// An edge as the tree surgery reads and rewrites it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SurgeryEdge {
    pub n1: usize,
    pub n2: usize,
    /// The endpoints' alias nodes, carried alongside the endpoints themselves.
    pub n1a: usize,
    pub n2a: usize,
    pub routelen: usize,
    pub grids: Vec<(i32, i32)>,
    /// `false` for a degenerate edge that was never routed.
    pub is_maze_route: bool,
    pub len: i32,
}

/// Read one edge's points out in the direction starting at `from` — the reference's `copyGrids`.
///
/// ⚠️ An edge stores its points in one fixed order; this hands them back starting from whichever
/// endpoint is asked for, reversing when that is the stored second node.
///
/// ⚠️ **A never-routed edge yields a single point — the coordinates of `from`** — rather than an
/// empty list, so the joins below always have something to start from.
pub fn copy_grids(
    nodes: &[MazeNode],
    from: usize,
    edges: &[SurgeryEdge],
    edge_id: usize,
) -> Vec<(i32, i32)> {
    let e = &edges[edge_id];
    if !e.is_maze_route {
        return vec![(nodes[from].x, nodes[from].y)];
    }
    let taken = &e.grids[..=e.routelen];
    if e.n1 == from {
        taken.to_vec()
    } else {
        taken.iter().rev().copied().collect()
    }
}

/// The node moved onto one of its own edges: re-cut that edge at the new position.
///
/// `n1` is moving to `(e1x, e1y)`, which lies on the edge joining it to `a1`. That edge is
/// shortened to end at the new position, and the edge to `a2` takes over everything beyond it —
/// so the node keeps two edges and neither is created or destroyed.
///
/// ⛔ **Each rewritten edge is oriented by X**: the endpoint with the smaller column is stored
/// first, and `n1`/`n2` are set to match. This is the same convention the segment emission uses,
/// and it is why the endpoints are assigned in both arms rather than once.
///
/// ⚠️ The join skips the first point of the second list, which is the node itself and already
/// present as the last point of the first.
#[allow(clippy::too_many_arguments)]
pub fn update_route_type1(
    nodes: &[MazeNode],
    n1: usize,
    a1: usize,
    a2: usize,
    (e1x, e1y): (i32, i32),
    edges: &mut [SurgeryEdge],
    edge_n1a1: usize,
    edge_n1a2: usize,
) -> Result<(), String> {
    let (a1x, a1y) = (nodes[a1].x, nodes[a1].y);
    let (a2x, a2y) = (nodes[a2].x, nodes[a2].y);

    // Both copies are taken before anything is written, because the edges being read are the
    // edges about to be overwritten.
    let from_a1 = copy_grids(nodes, a1, edges, edge_n1a1);
    let from_n1 = copy_grids(nodes, n1, edges, edge_n1a2);

    let e1_pos = from_a1
        .iter()
        .position(|&p| p == (e1x, e1y))
        .ok_or_else(|| format!("({e1x},{e1y}) is not on the edge it is supposed to lie on"))?;

    // The near half: a1 as far as the new position.
    let head: Vec<(i32, i32)> = from_a1[..=e1_pos].to_vec();
    let e = &mut edges[edge_n1a1];
    if a1x <= e1x {
        e.grids = head;
        e.n1 = a1;
        e.n2 = n1;
    } else {
        e.grids = head.into_iter().rev().collect();
        e.n1 = n1;
        e.n2 = a1;
    }
    e.is_maze_route = true;
    e.routelen = e1_pos;
    e.len = (a1x - e1x).abs() + (a1y - e1y).abs();

    // The far half: everything beyond the new position, plus the whole of the other edge.
    let mut tail: Vec<(i32, i32)> = from_a1[e1_pos..].to_vec();
    tail.extend_from_slice(&from_n1[1..]);
    let e = &mut edges[edge_n1a2];
    e.routelen = tail.len() - 1;
    if e1x <= a2x {
        e.grids = tail;
        e.n1 = n1;
        e.n2 = a2;
    } else {
        e.grids = tail.into_iter().rev().collect();
        e.n1 = a2;
        e.n2 = n1;
    }
    e.is_maze_route = true;
    e.len = (a2x - e1x).abs() + (a2y - e1y).abs();
    Ok(())
}

/// The node moved onto a **different** edge: merge its own two edges, and split the one it landed
/// on.
///
/// ⛔ **Three edge slots are recycled, not created.** The node's two edges become the single edge
/// joining its former neighbours, and the edge it landed on becomes the node's two new edges —
/// all in the same three slots, with their roles swapped.
///
/// ⛔ **The merged edge is written into the slot of the edge about to be split**, so the split
/// only works because every list was copied out first. Writing before copying would destroy the
/// points the split needs.
///
/// ⚠️ **No endpoint is assigned here**, unlike the other variant: the caller rewrites all three
/// edges' endpoints and the adjacency of five nodes afterwards. Assigning them here would make
/// the two halves disagree about which had done it.
#[allow(clippy::too_many_arguments)]
pub fn update_route_type2(
    nodes: &[MazeNode],
    n1: usize,
    a1: usize,
    a2: usize,
    c1: usize,
    c2: usize,
    (e1x, e1y): (i32, i32),
    edges: &mut [SurgeryEdge],
    edge_n1a1: usize,
    edge_n1a2: usize,
    edge_c1c2: usize,
) -> Result<(), String> {
    let (a1x, a1y) = (nodes[a1].x, nodes[a1].y);
    let (a2x, a2y) = (nodes[a2].x, nodes[a2].y);
    let (c1x, c1y) = (nodes[c1].x, nodes[c1].y);
    let (c2x, c2y) = (nodes[c2].x, nodes[c2].y);

    let from_a1 = copy_grids(nodes, a1, edges, edge_n1a1);
    let from_n1 = copy_grids(nodes, n1, edges, edge_n1a2);
    let from_c1 = copy_grids(nodes, c1, edges, edge_c1c2);

    // The two edges at the node become one, joining its former neighbours through where it was.
    let mut merged: Vec<(i32, i32)> = from_a1.clone();
    merged.extend_from_slice(&from_n1[1..]);
    let e = &mut edges[edge_c1c2];
    e.routelen = merged.len() - 1;
    e.grids = merged;
    e.is_maze_route = true;
    e.len = (a1x - a2x).abs() + (a1y - a2y).abs();

    // ⚠️ Looked up AFTER the merge has overwritten this edge's slot — which is safe only because
    // its points were copied above.
    let e1_pos = from_c1
        .iter()
        .position(|&p| p == (e1x, e1y))
        .ok_or_else(|| format!("({e1x},{e1y}) is not on the edge it was routed onto"))?;

    let head: Vec<(i32, i32)> = from_c1[..=e1_pos].to_vec();
    let e = &mut edges[edge_n1a1];
    e.routelen = head.len() - 1;
    e.grids = head;
    e.is_maze_route = true;
    e.len = (c1x - e1x).abs() + (c1y - e1y).abs();

    let tail: Vec<(i32, i32)> = from_c1[e1_pos..].to_vec();
    let e = &mut edges[edge_n1a2];
    e.routelen = tail.len() - 1;
    e.grids = tail;
    e.is_maze_route = true;
    e.len = (c2x - e1x).abs() + (c2y - e1y).abs();
    Ok(())
}

/// How far the search may stray from the edge it is re-routing.
///
/// ⛔ **The allowance is capped by the edge's CURRENT route length**, not by its span: an edge
/// already routed the long way round gets a wider search than a short one between the same
/// endpoints. It also grows with the iteration, so later rounds look further afield.
///
/// ⚠️ **Integer division.** `iter / 6` steps every sixth round, not smoothly, and `iter / 7` in
/// the shrink below steps on a different cadence again.
///
/// ⛔ **A critical net gets a NARROWER region, not a wider one.** The shrink is applied inwards on
/// every side, so a net under timing pressure is kept close to its existing path rather than
/// allowed to wander — and it is capped at half the allowance, so the region can never invert.
pub fn maze_edge_region(
    (n1x, n1y): (i32, i32),
    (n2x, n2y): (i32, i32),
    expand: i32,
    iter: i32,
    routelen: i32,
    is_critical: bool,
    (x_grid, y_grid): (i32, i32),
) -> (i32, i32, i32, i32) {
    let (xmin, xmax) = (n1x.min(n2x), n1x.max(n2x));
    let (ymin, ymax) = (n1y.min(n2y), n1y.max(n2y));

    let enlarge = expand.min((iter / 6 + 3) * routelen);
    let decrease = if is_critical {
        ((iter / 7) * 5).min(enlarge / 2)
    } else {
        0
    };

    (
        (xmin - enlarge + decrease).max(0),
        (xmax + enlarge - decrease).min(x_grid - 1),
        (ymin - enlarge + decrease).max(0),
        (ymax + enlarge - decrease).min(y_grid - 1),
    )
}

/// Whether an edge is worth re-routing at all — the driver's first gate.
///
/// ⚠️ **The length is RECOMPUTED from the endpoints here**, not read from the edge. An earlier
/// stage may have moved a node, leaving the stored length stale; the gate uses the live geometry.
pub fn maze_edge_is_long_enough(
    (n1x, n1y): (i32, i32),
    (n2x, n2y): (i32, i32),
    maze_edge_threshold: i32,
) -> Option<i32> {
    let len = (n2x - n1x).abs() + (n2y - n1y).abs();
    (len > maze_edge_threshold).then_some(len)
}

/// Run the search until it reaches the destination subtree, and report where.
///
/// ⛔ **The stopping test is on the cell the heap is ABOUT to pop, not on the one just popped.**
/// The loop reads the heap's minimum, asks whether it belongs to the destination subtree, and
/// only expands it if not — so the cell it stops on is never expanded, and is returned as the
/// meeting point.
///
/// ⛔ **Four relaxations per expansion, each guarded by the region**, and the guards are not
/// symmetric with the detour flags beside them: a step is *taken* while the neighbour is inside
/// the region, but a detour is only *considered* while there is a further cell beyond it.
///
/// ⚠️ **The via flag is derived from the predecessor, not from the step.** A cell reached
/// vertically makes a horizontal step a turn, and vice versa — which is why the horizontal pair
/// tests the row and the vertical pair tests the column.
///
/// 🔑 **The source exemption here and the one inside the relaxation are MUTUALLY redundant.**
/// This loop leaves the predecessor equal to the cell when the distance is zero, so the flag it
/// passes is already false at a source; the relaxation then refuses the via again on the same
/// condition. Removing **either** changes nothing — each is a mutation nothing can kill — and
/// removing both would charge a via for turning at a source. Both are transcribed.
///
/// ⚠️ The cell index is decomposed with the **row stride** of the distance grid, which is the
/// grid's allocated width rather than the design's.
///
/// 🔑 **The reference keeps a second distance grid for the destination subtree, and never reads
/// it.** It is filled with the infinity sentinel, then zeroed at each destination seed — but only
/// its *addresses* are used, to index the flags that say which cells belong to that subtree. Its
/// values are inert. Destinations are therefore held here as coordinates and a flag, which is
/// exactly equivalent and one grid lighter.
pub fn maze_search(
    s: &mut MazeSearch,
    dest_seeds: &[(i32, i32)],
    (region_x1, region_x2, region_y1, region_y2): (i32, i32, i32, i32),
    inp: &RelaxInputs<'_>,
) -> Result<(i32, i32), String> {
    let mut is_dest = vec![false; s.dist.len()];
    for &(x, y) in dest_seeds {
        is_dest[s.at(x, y)] = true;
    }

    let mut ind1 = *s.heap.first().ok_or("the source frontier is empty")?;
    while !is_dest[ind1] {
        let (cur_x, cur_y) = ((ind1 % s.width) as i32, (ind1 / s.width) as i32);

        // Where this cell was reached from; itself when it is a source.
        let (mut pre_x, mut pre_y) = (cur_x, cur_y);
        if s.dist[ind1] != 0.0 {
            if s.hv[ind1] {
                pre_x = s.parent_x1[ind1];
                pre_y = s.parent_y1[ind1];
            } else {
                pre_x = s.parent_x3[ind1];
                pre_y = s.parent_y3[ind1];
            }
        }

        remove_min(&mut s.heap, &s.dist);

        if cur_x > region_x1 {
            relax_adjacent(s, (cur_x, cur_y), (-1, 0), pre_y != cur_y,
                           cur_x < region_x2 - 1, inp)?;
        }
        if cur_x < region_x2 {
            relax_adjacent(s, (cur_x, cur_y), (1, 0), pre_y != cur_y,
                           cur_x > region_x1 + 1, inp)?;
        }
        if cur_y > region_y1 {
            relax_adjacent(s, (cur_x, cur_y), (0, -1), pre_x != cur_x,
                           cur_y < region_y2 - 1, inp)?;
        }
        if cur_y < region_y2 {
            relax_adjacent(s, (cur_x, cur_y), (0, 1), pre_x != cur_x,
                           cur_y > region_y1 + 1, inp)?;
        }

        ind1 = *s.heap.first().ok_or("the search exhausted its frontier")?;
    }
    Ok(((ind1 % s.width) as i32, (ind1 / s.width) as i32))
}

/// Charge the demand a finished route costs — the driver's last act for an edge.
///
/// ⛔ **The edge is indexed at its LOWER endpoint**, stated here as `min` of the two. That is the
/// fourth site in this engine to state the same rule, after the route walk, the rip-up undo and
/// the relaxation.
pub fn charge_route(grid: &mut EstimateGrid, grids: &[(i32, i32)], edge_cost: i8) {
    let cost = f64::from(edge_cost);
    for pair in grids.windows(2) {
        let ((ax, ay), (bx, by)) = (pair[0], pair[1]);
        if ax == bx {
            grid.update_usage_v(ax, ay.min(by), cost);
        } else {
            grid.update_usage_h(ax.min(bx), ay, cost);
        }
    }
}

/// Give a pin a stand-in that can move in its place — the reference's `splitEdge`.
///
/// A pin cannot be relocated, so when the search wants its position to change, a **duplicate node
/// is created at the same coordinates** and joined to the pin by a **zero-length edge**. The
/// duplicate takes over the pin's connections and moves instead; the pin stays where it is.
///
/// ⛔ **The duplicate inherits the pin's alias**, not its own identity, so the two share
/// connection state exactly as coincident nodes do elsewhere.
///
/// ⚠️ **The pin's neighbour list shrinks.** The far node is rebuilt onto the duplicate and the
/// caller's node is dropped entirely, so the pin ends with one fewer neighbour than it began.
///
/// Returns the new node's index — which the caller uses in place of the pin from then on.
pub fn split_edge(
    nodes: &mut Vec<MazeNode>,
    edges: &mut Vec<SurgeryEdge>,
    n1: usize,
    n2: usize,
    edge_n1n2: usize,
) -> usize {
    let (n2x, n2y) = (nodes[n2].x, nodes[n2].y);
    let new_node_id = nodes.len();
    let new_edge_id = edges.len();
    let alias = nodes[n2].stack_alias;

    // ⚠️ The neighbour handed over is the first one that is not the caller — taken by position,
    // so which one it is depends on the stored order.
    let (nbr, edge_n2_nbr) = if nodes[n2].neighbours[0].0 == n1 {
        nodes[n2].neighbours[1]
    } else {
        nodes[n2].neighbours[0]
    };

    // Rebuild the pin's list: the caller is dropped, and the handed-over neighbour is replaced by
    // the duplicate.
    let rebuilt: Vec<(usize, usize)> = nodes[n2]
        .neighbours
        .iter()
        .filter(|(v, _)| *v != n1)
        .map(|&(v, e)| if v == nbr { (new_node_id, new_edge_id) } else { (v, e) })
        .collect();
    nodes[n2].neighbours = rebuilt;

    // Both edges that met at the pin now meet at the duplicate.
    for eid in [edge_n2_nbr, edge_n1n2] {
        if edges[eid].n1 == n2 {
            edges[eid].n1 = new_node_id;
            edges[eid].n1a = alias;
        } else {
            edges[eid].n2 = new_node_id;
            edges[eid].n2a = alias;
        }
    }

    // ⚠️ Only the neighbour ITSELF is re-pointed on these two, not the edge beside it — the edge
    // is unchanged, only which node sits at its far end.
    for node in [nbr, n1] {
        for entry in nodes[node].neighbours.iter_mut() {
            if entry.0 == n2 {
                entry.0 = new_node_id;
            }
        }
    }

    edges.push(SurgeryEdge {
        n1: new_node_id,
        n2,
        n1a: alias,
        n2a: nodes[n2].stack_alias,
        routelen: 0,
        grids: vec![(n2x, n2y)],
        is_maze_route: true,
        len: 0,
    });
    nodes.push(MazeNode {
        x: n2x,
        y: n2y,
        // ⛔ Three neighbours in this order: the handed-over one, the pin, then the caller.
        neighbours: vec![(nbr, edge_n2_nbr), (n2, new_edge_id), (n1, edge_n1n2)],
        stack_alias: alias,
    });
    new_node_id
}

/// Re-point the tree after a node has moved onto a different edge.
///
/// The grids were rewritten by the surgery; this is the other half — the endpoints of the three
/// recycled edges, and the adjacency of the **five** nodes involved.
///
/// ⛔ **The moved node's list is rebuilt wholesale**, not patched: it keeps its link to the far
/// endpoint of the edge being re-routed and takes the two ends of the edge it landed on. Its
/// former neighbours are joined to each other instead.
#[allow(clippy::too_many_arguments)]
pub fn rewire_after_type2(
    nodes: &mut [MazeNode],
    edges: &mut [SurgeryEdge],
    n1: usize,
    n2: usize,
    a1: usize,
    a2: usize,
    c1: usize,
    c2: usize,
    edge_n1n2: usize,
    edge_n1a1: usize,
    edge_n1a2: usize,
    edge_c1c2: usize,
) {
    let (edge_n1c1, edge_n1c2, edge_a1a2) = (edge_n1a1, edge_n1a2, edge_c1c2);

    edges[edge_n1c1].n1 = c1;
    edges[edge_n1c1].n2 = n1;
    edges[edge_n1c2].n1 = n1;
    edges[edge_n1c2].n2 = c2;
    edges[edge_a1a2].n1 = a1;
    edges[edge_a1a2].n2 = a2;

    nodes[n1].neighbours = vec![(n2, edge_n1n2), (c1, edge_n1c1), (c2, edge_n1c2)];

    // ⚠️ Each of the other four keeps its own list and replaces exactly one entry — the first
    // match, which is what the reference's `break` means.
    //
    // 🔑 Replacing every match instead is a mutation nothing can kill, and the reason is
    // structural: a node's neighbours are distinct, so there is never more than one match.
    // Measured across 11,084 captured neighbour lists, not one holds a duplicate. Transcribed as
    // the reference writes it, because the reference relies on that rather than enforcing it.
    //
    // ⛔ The ORDER of these four does matter, and is not structural: 301 of the captured
    // rewirings have two of them touching the same node, where whichever runs first wins.
    for (node, from, to, edge) in [
        (a1, n1, a2, edge_a1a2),
        (a2, n1, a1, edge_a1a2),
        (c1, c2, n1, edge_n1c1),
        (c2, c1, n1, edge_n1c2),
    ] {
        if let Some(entry) = nodes[node].neighbours.iter_mut().find(|e| e.0 == from) {
            *entry = (to, edge);
        }
    }
}

/// Why one edge was not re-routed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EdgeOutcome {
    /// Shorter than the threshold.
    TooShort,
    /// The rip-up gate declined to replace the existing route.
    NotCongested,
    /// Re-routed: the path it took, the region it searched, and the frontiers it seeded.
    ///
    /// ⚠️ The region and the frontiers are reported, not just the path, so that a stage handed
    /// the wrong one is caught. A region one cell too small usually yields the same path anyway.
    Routed {
        path: Vec<(i32, i32)>,
        region: (i32, i32, i32, i32),
        src: Vec<(i32, i32)>,
        dest: Vec<(i32, i32)>,
        /// Every `corr_edge` write the seeding made, in order — the surgery looks the path's ends up
        /// here (the last write for a point wins).
        corr_edge: Vec<((i32, i32), usize)>,
    },
    /// ⛔ The surgery could not place a contact point. The net's tree must be rebuilt and the
    /// **whole net** reprocessed — the reference steps its net index back before breaking out.
    RebuildNet(String),
}

/// What the sequencer needs from the stages around it.
///
/// ⚠️ Passed in rather than reached for, so the order below stays readable as a call sequence and
/// nothing in it does work of its own.
pub struct EdgeContext<'a> {
    pub maze_edge_threshold: i32,
    pub expand: i32,
    pub iter: i32,
    pub is_critical: bool,
    pub grid_size: (i32, i32),
    pub num_terminals: usize,
    pub edge_cost: i8,
    pub relax: &'a RelaxInputs<'a>,
    /// The rip-up gate's answer for this edge, which the caller obtains from the gate stage.
    pub rip_up_says_reroute: bool,
}

/// Re-route one edge: the reference's per-edge call sequence, and nothing else.
///
/// ```text
///   recompute the length      ->  too short?      give up
///   ask the rip-up gate       ->  not congested?  give up
///   compute the search region
///   seed both frontiers from the subtrees the edge separates
///   search until the far subtree is reached
///   walk the parents back to get the path
/// ```
///
/// ⛔ **Every step here is its own function, each gated against the reference separately.** This
/// one does no work of its own — that is what lets a divergence be attributed to a stage rather
/// than bisected out of a loop.
///
/// ⚠️ The tree surgery that follows is left to the caller: which of its two shapes applies depends
/// on where the path's ends landed, and that decision belongs with the tree, not with the search.
///
/// ⚠️ **`search_grid_width` is an allocation detail here, not a behavioural one.** The reference's
/// row stride matters because it decomposes a flat cell index that crosses function boundaries;
/// no flat index escapes this crate, so any width wide enough round-trips. Kept as a parameter so
/// the allocation stays the caller's decision, but changing it is a mutation nothing can kill —
/// and that is a property of this transcription, not of the reference.
pub fn route_one_edge(
    search_grid_width: usize,
    nodes: &[MazeNode],
    edges: &[MazeEdge],
    edge_id: usize,
    ctx: &EdgeContext<'_>,
) -> Result<EdgeOutcome, String> {
    let e = &edges[edge_id];
    let (n1, n2) = (e.n1, e.n2);
    let p1 = (nodes[n1].x, nodes[n1].y);
    let p2 = (nodes[n2].x, nodes[n2].y);

    if maze_edge_is_long_enough(p1, p2, ctx.maze_edge_threshold).is_none() {
        return Ok(EdgeOutcome::TooShort);
    }
    if !ctx.rip_up_says_reroute {
        return Ok(EdgeOutcome::NotCongested);
    }

    let region = maze_edge_region(
        p1, p2, ctx.expand, ctx.iter, e.routelen as i32, ctx.is_critical, ctx.grid_size,
    );

    let heaps = setup_heap(ctx.num_terminals, nodes, edges, edge_id, region);

    // ⚠️ The distances start at "unreached" across the region, and the seeds are what the search
    // begins from — both of which the heap setup decided.
    let mut state = MazeSearch::new(search_grid_width, (region.3 + 2) as usize);
    for &(x, y) in &heaps.src {
        let i = state.at(x, y);
        state.dist[i] = 0.0;
        state.heap.push(i);
    }

    let cross = maze_search(&mut state, &heaps.dest, region, ctx.relax)?;
    Ok(EdgeOutcome::Routed {
        path: backtrace(&state, cross),
        region,
        src: heaps.src,
        dest: heaps.dest,
        corr_edge: heaps.corr_edge,
    })
}

/// What the per-net loop decides to do after one edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfterEdge {
    /// Carry on to the next edge of this net.
    Continue,
    /// ⛔ Abandon the rest of this net, rebuild its tree from the pins, and process the **same**
    /// net again from the start.
    RebuildAndRetry,
}

/// One pass of the maze router over every net — the reference's per-net call sequence.
///
/// ⛔ **The net order is the congestion order when ordering is on, and the router's own net order
/// otherwise.** Those are different lists, not the same list in a different order.
///
/// ⛔ **A failed surgery does not skip the edge — it abandons the net.** The reference steps its
/// net index *back* before breaking out, so the loop's own increment returns it to the same net.
/// And the rebuild is not a retry of the routing: it clears the net's nodes and edges entirely
/// and builds a fresh Steiner tree from the pins, discarding everything routed so far.
///
/// ⚠️ **Nothing bounds the retry.** A net that fails the same way every time is reprocessed
/// forever; the reference relies on the rebuilt tree differing from the one that failed.
///
/// ⚠️ **The edge count is read once**, before the edge loop — so an edge added part-way through
/// by a pin stand-in is not visited in that pass. The ordering is likewise computed once per
/// entry to the net, which means a retry recomputes it against the rebuilt tree.
pub fn maze_route_pass<F>(
    net_order: &[usize],
    mut route_net: F,
) -> Vec<usize>
where
    F: FnMut(usize) -> AfterEdge,
{
    let mut visited = Vec::new();
    let mut i = 0usize;
    while i < net_order.len() {
        let net_id = net_order[i];
        visited.push(net_id);
        match route_net(net_id) {
            AfterEdge::Continue => i += 1,
            // ⚠️ The reference's `nidRPC--` followed by the loop's `++`: the index does not move.
            AfterEdge::RebuildAndRetry => {}
        }
    }
    visited
}

/// Cut any loop out of a routed path, giving back the demand it charged.
///
/// A maze route can revisit a cell — the search is over a grid, not a tree, and nothing in it
/// forbids a path that doubles back. This finds the **first** repeated point, removes everything
/// between the two visits, and gives back exactly the demand that stretch was charged.
///
/// ⛔ **The scan restarts from the beginning after every removal.** The reference resets both
/// indices and lets the loops' own increments carry it back to the start. That is not a
/// conservative choice: removing a stretch shifts everything after it **down**, so a duplicate
/// pair that sat beyond the index can land entirely before it, where a continuing scan would
/// never look again.
///
/// 🔑 **Because it restarts, at most one earlier point can ever match.** When the scan reaches an
/// index, no two earlier points are equal — if they were it would have stopped at the second of
/// them. So taking the last match rather than the first is a mutation nothing can kill, and that
/// is a property of the algorithm rather than a gap in the corpus.
///
/// ⚠️ **A zero-length step charges nothing and gives nothing back.** The vertical arm guards
/// against the two points being identical; the horizontal arm needs no guard, because it is only
/// reached when the columns differ.
///
/// ⚠️ The points beyond the loop are copied **down** over it and the length reduced — the buffer
/// keeps its size, so anything past the new length is stale and must not be read.
pub fn remove_loops(
    grid: &mut EstimateGrid,
    grids: &mut [(i32, i32)],
    routelen: &mut usize,
    edge_cost: i8,
) -> usize {
    let cost = f64::from(edge_cost);
    let mut removed = 0usize;
    let mut i = 1usize;

    while i <= *routelen {
        let mut found = None;
        for j in 0..i {
            if grids[i] == grids[j] {
                found = Some(j);
                break;
            }
        }
        let Some(j) = found else {
            i += 1;
            continue;
        };

        for k in j..i {
            let ((ax, ay), (bx, by)) = (grids[k], grids[k + 1]);
            if ax == bx {
                if ay != by {
                    grid.update_usage_v(ax, ay.min(by), -cost);
                }
            } else {
                grid.update_usage_h(ax.min(bx), ay, -cost);
            }
        }

        let mut cnt = 1usize;
        for k in i + 1..=*routelen {
            grids[j + cnt] = grids[k];
            cnt += 1;
        }
        *routelen -= i - j;
        removed += 1;
        // ⛔ Back to the start, not on from here.
        i = 1;
    }
    removed
}
