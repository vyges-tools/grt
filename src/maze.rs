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

/// The reference's `BIG_INT`, used as "not reached yet".
pub const BIG_INT: f64 = i32::MAX as f64;

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
pub fn update_heap(heap: &mut [usize], mut i: usize, dist: &[f64]) {
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
pub fn heapify(heap: &mut [usize], dist: &[f64]) {
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
/// ⛔ **This is NOT the idiomatic swap-pop-sift, and the difference is observable.** The last
/// element is copied to the root and the heap is repaired **while that element is still present
/// at the end**, so it takes part in comparisons as a child; only then is the tail dropped.
///
/// ⟹ If the sift path reaches the stale slot, the ordering differs from repairing a heap that had
/// already been shortened. Writing the idiomatic version would give a different pop order, and
/// therefore a different route. Transcribed exactly.
pub fn remove_min(heap: &mut Vec<usize>, dist: &[f64]) {
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
