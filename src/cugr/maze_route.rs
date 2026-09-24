// SPDX-License-Identifier: Apache-2.0
//! Stage 4's search (`MazeRoute`): a sparsified two-layer grid graph over the net's access
//! points, a Dijkstra-style search that attaches pins one at a time, and the Steiner tree of the
//! paths found — which pattern routing then lays onto real layers.

use std::collections::HashMap;

use super::geo::{Interval, Point};
use super::grid_graph::{view_sum, GridGraph, View};
use super::layers::{H, V};
use super::pattern_route::{AccessPointMap, SteinerNode, SteinerTree};

/// `SparseGrid`: which rows and columns the sparsified graph keeps — every `interval`-th, from an
/// `offset` that `step` advances after each net.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SparseGrid {
    pub interval: Point,
    pub offset: Point,
}

impl SparseGrid {
    pub fn new(x_interval: i32, y_interval: i32, x_offset: i32, y_offset: i32) -> Self {
        SparseGrid { interval: Point::new(x_interval, y_interval), offset: Point::new(x_offset, y_offset) }
    }
    /// `step()`: each offset advances by one, modulo its interval.
    pub fn step(&mut self) {
        self.offset = Point::new((self.offset.x + 1) % self.interval.x, (self.offset.y + 1) % self.interval.y);
    }
}

/// `SparseGraph`: vertices on the kept rows × columns in two planes (0 horizontal, 1 vertical), a
/// same-plane edge between neighbours priced from the wire-cost view, and a via edge between the
/// planes at every point — free at a pin.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SparseGraph {
    /// The net's selected cells, in (x, y) order.
    pub pseudo_pins: Vec<(Point, Interval)>,
    pub xs: Vec<i32>,
    pub ys: Vec<i32>,
    /// `(plane, x, y)`.
    pub vertices: Vec<(usize, i32, i32)>,
    /// Per vertex: the next vertex along the plane, the previous one, and across the planes
    /// (-1 where none).
    pub edges: Vec<[i32; 3]>,
    pub costs: Vec<[f64; 3]>,
    /// vertex → pin, and pin → vertex (a lookup; the reference's hash map is never iterated).
    pub vertex_pin: HashMap<i32, usize>,
    pub pin_vertex: Vec<i32>,
}

impl SparseGraph {
    fn vertex_index(&self, direction: usize, xi: usize, yi: usize) -> usize {
        direction * self.xs.len() * self.ys.len() + yi * self.xs.len() + xi
    }

    /// The kept lines of one dimension: every `interval`-th from `offset` below `size`, with each
    /// pin line not already a kept line (nor a repeat) inserted in order before the next.
    fn lines(pins: &[i32], interval: i32, offset: i32, size: i32) -> Vec<i32> {
        let mut out: Vec<i32> = Vec::new();
        let mut j = 0;
        let mut i = 0;
        loop {
            let line = i * interval + offset;
            while j < pins.len() && pins[j] <= line {
                if !((!out.is_empty() && pins[j] == *out.last().expect("non-empty")) || pins[j] == line) {
                    out.push(pins[j]);
                }
                j += 1;
            }
            if line < size {
                out.push(line);
            } else {
                break;
            }
            i += 1;
        }
        out
    }

    /// `SparseGraph::init(wire_cost_view, grid)`, from the net's selected cells.
    pub fn init(selected: &AccessPointMap, wire_cost_view: &View<f64>, grid: &SparseGrid, sizes: [usize; 2], unit_via_cost: f64) -> SparseGraph {
        let mut g = SparseGraph { pseudo_pins: selected.iter().map(|(&(x, y), &l)| (Point::new(x, y), l)).collect(), ..Default::default() };
        let mut pxs: Vec<i32> = g.pseudo_pins.iter().map(|(p, _)| p.x).collect();
        let mut pys: Vec<i32> = g.pseudo_pins.iter().map(|(p, _)| p.y).collect();
        pxs.sort();
        pys.sort();
        g.xs = SparseGraph::lines(&pxs, grid.interval.x, grid.offset.x, sizes[0] as i32);
        g.ys = SparseGraph::lines(&pys, grid.interval.y, grid.offset.y, sizes[1] as i32);
        for direction in 0..2 {
            for &y in &g.ys {
                for &x in &g.xs {
                    g.vertices.push((direction, x, y));
                }
            }
        }
        g.edges = vec![[-1; 3]; g.vertices.len()];
        g.costs = vec![[-1.0; 3]; g.vertices.len()];
        let (nx, ny) = (g.xs.len(), g.ys.len());
        let same_layer = |g: &mut SparseGraph, direction: usize, xi: usize, yi: usize| {
            let u = g.vertex_index(direction, xi, yi);
            let v = if direction == H { u + 1 } else { u + nx };
            let a = Point::new(g.xs[xi], g.ys[yi]);
            let b = Point::new(g.xs[xi + 1 - direction], g.ys[yi + direction]);
            g.edges[u][0] = v as i32;
            g.edges[v][1] = u as i32;
            let c = view_sum(wire_cost_view, a, b);
            g.costs[u][0] = c;
            g.costs[v][1] = c;
        };
        for yi in 0..ny {
            for xi in 0..nx.saturating_sub(1) {
                same_layer(&mut g, H, xi, yi);
            }
        }
        for xi in 0..nx {
            for yi in 0..ny.saturating_sub(1) {
                same_layer(&mut g, V, xi, yi);
            }
        }
        for xi in 0..nx {
            for yi in 0..ny {
                let u = g.vertex_index(0, xi, yi);
                let v = u + nx * ny;
                g.edges[u][2] = v as i32;
                g.edges[v][2] = u as i32;
                g.costs[u][2] = unit_via_cost;
                g.costs[v][2] = unit_via_cost;
            }
        }
        let x_to_xi: HashMap<i32, usize> = g.xs.iter().enumerate().map(|(i, &x)| (x, i)).collect();
        let y_to_yi: HashMap<i32, usize> = g.ys.iter().enumerate().map(|(i, &y)| (y, i)).collect();
        g.pin_vertex = vec![-1; g.pseudo_pins.len()];
        for pin in 0..g.pseudo_pins.len() {
            let p = g.pseudo_pins[pin].0;
            let u = g.vertex_index(0, x_to_xi[&p.x], y_to_yi[&p.y]);
            g.vertex_pin.insert(u as i32, pin);
            g.pin_vertex[pin] = u as i32;
            g.costs[u][2] = 0.0;
            g.costs[u + nx * ny][2] = 0.0;
        }
        g
    }
}

/// A search state (`Solution`): its cost, vertex and predecessor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Solution {
    pub cost: f64,
    pub vertex: i32,
    pub prev: Option<usize>,
}

/// `std::priority_queue` as libc++ 22 implements it over a vector: `push_heap` sifts up;
/// `pop_heap` moves the top out, drives the hole to a leaf taking the larger child each level
/// (Floyd), fills it with the last element and sifts that up. ⛔ Not `BinaryHeap`: two states
/// with equal cost AND vertex are ordered by these moves alone.
pub struct LibcxxHeap<'a> {
    pub v: Vec<usize>,
    /// `comp(a, b)`: `a` ranks BELOW `b` (the reference's `compare_solution`).
    less: &'a dyn Fn(usize, usize) -> bool,
}

impl<'a> LibcxxHeap<'a> {
    pub fn new(less: &'a dyn Fn(usize, usize) -> bool) -> Self {
        LibcxxHeap { v: Vec::new(), less }
    }
    /// `__sift_up(first, last, comp, len)`: the element at `len - 1` moves up.
    fn sift_up(&mut self, len: usize) {
        if len <= 1 {
            return;
        }
        let mut last = len - 1;
        let mut parent = (len - 2) / 2;
        if (self.less)(self.v[parent], self.v[last]) {
            let t = self.v[last];
            loop {
                self.v[last] = self.v[parent];
                last = parent;
                if parent == 0 {
                    break;
                }
                parent = (parent - 1) / 2;
                if !(self.less)(self.v[parent], t) {
                    break;
                }
            }
            self.v[last] = t;
        }
    }
    pub fn push(&mut self, x: usize) {
        self.v.push(x);
        let n = self.v.len();
        self.sift_up(n);
    }
    pub fn top(&self) -> usize {
        self.v[0]
    }
    pub fn is_empty(&self) -> bool {
        self.v.is_empty()
    }
    /// `__floyd_sift_down`: the hole at 0 descends to a leaf through the larger child (the right
    /// one only when strictly greater); returns it. (libc++'s `__child_i` and `__child` always
    /// name the same index.)
    fn floyd_sift_down(&mut self, len: usize) -> usize {
        let mut hole = 0usize;
        loop {
            let mut child = 2 * hole + 1;
            if child + 1 < len && (self.less)(self.v[child], self.v[child + 1]) {
                child += 1;
            }
            self.v[hole] = self.v[child];
            hole = child;
            if child > (len - 2) / 2 {
                return hole;
            }
        }
    }
    pub fn pop(&mut self) {
        let len = self.v.len();
        if len > 1 {
            let top = self.v[0];
            let hole = self.floyd_sift_down(len);
            let last = len - 1;
            if hole == last {
                self.v[hole] = top;
            } else {
                self.v[hole] = self.v[last];
                self.v[last] = top;
                self.sift_up(hole + 1);
            }
        }
        self.v.pop();
    }
}

/// `MazeRoute::run()`: from pin 0, repeatedly the cheapest unexplored state, until every pin is
/// reached; each path found is re-seeded at cost 0 so later pins can join it.
///
/// Upstream rules: states rank by cost, then vertex (lower first); a state no cheaper than its
/// vertex's best is not expanded; a move straight back to the predecessor's vertex is skipped; a
/// state is kept only strictly cheaper than the target's best. The re-seed walks the found path
/// until a state of cost 0, each new state keeping the OLD predecessor.
pub fn run(g: &SparseGraph) -> Result<(Vec<Solution>, Vec<usize>), String> {
    let mut min_costs = vec![f64::MAX; g.vertices.len()];
    let mut found_list = Vec::new();
    let num_pins = g.pseudo_pins.len();
    let mut visited = vec![false; num_pins.max(1)];
    visited[0] = true;
    let mut detached = num_pins as i32 - 1;
    // The comparator reads the arena, which grows while the heap holds indices into it.
    let arena_cell: std::cell::RefCell<Vec<Solution>> = std::cell::RefCell::new(Vec::new());
    let less = |a: usize, b: usize| {
        let ar = arena_cell.borrow();
        let (x, y) = (&ar[a], &ar[b]);
        if x.cost != y.cost {
            x.cost > y.cost
        } else {
            x.vertex > y.vertex
        }
    };
    let mut heap = LibcxxHeap::new(&less);
    let push = |heap: &mut LibcxxHeap<'_>, s: Solution, min_costs: &mut Vec<f64>| {
        let id = {
            let mut ar = arena_cell.borrow_mut();
            ar.push(s);
            ar.len() - 1
        };
        heap.push(id);
        let v = s.vertex as usize;
        min_costs[v] = s.cost.min(min_costs[v]);
    };
    push(&mut heap, Solution { cost: 0.0, vertex: g.pin_vertex[0], prev: None }, &mut min_costs);
    while detached > 0 {
        let mut found: Option<usize> = None;
        let mut found_pin: i32 = 0;
        while !heap.is_empty() {
            let id = heap.top();
            heap.pop();
            let s = arena_cell.borrow()[id];
            found_pin = g.vertex_pin.get(&s.vertex).map_or(-1, |&p| p as i32);
            if found_pin != -1 && !visited[found_pin as usize] {
                found = Some(id);
                break;
            }
            if s.cost > min_costs[s.vertex as usize] {
                continue;
            }
            let prev_vertex = s.prev.map(|p| arena_cell.borrow()[p].vertex);
            for e in 0..3 {
                let next = g.edges[s.vertex as usize][e];
                if next == -1 || prev_vertex == Some(next) {
                    continue;
                }
                let next_cost = s.cost + g.costs[s.vertex as usize][e];
                if next_cost < min_costs[next as usize] {
                    push(&mut heap, Solution { cost: next_cost, vertex: next, prev: Some(id) }, &mut min_costs);
                }
            }
        }
        let f = found.ok_or("GRT-0282: failed to find a connected pin")?;
        if found_pin == -1 {
            return Err("GRT-0282: failed to find a connected pin".into());
        }
        found_list.push(f);
        visited[found_pin as usize] = true;
        detached -= 1;
        let mut t = Some(f);
        while let Some(id) = t {
            let s = arena_cell.borrow()[id];
            if s.cost == 0.0 {
                break;
            }
            push(&mut heap, Solution { cost: 0.0, vertex: s.vertex, prev: s.prev }, &mut min_costs);
            t = s.prev;
        }
    }
    drop(heap);
    Ok((arena_cell.into_inner(), found_list))
}

/// `getSteinerTree()`: the found paths merged into one tree rooted at the start pin, then cleaned.
///
/// Upstream rules: a path is walked from its pin back to the first vertex already in the tree,
/// each new vertex's node taking the previous as child; a path's two ends take their pins' layers.
/// Clean-up, in preorder: (1) a child at its parent's point is folded in — its children appended
/// to the parent's (and so visited in the same pass) and its layers taken only when the parent has
/// none (⚠️ where both have layers the reference's `unionWith` result is discarded: the parent's
/// stay); (2) each child is replaced by the end of the straight run of layer-less single-child
/// nodes below it; (3) a remaining co-located child is an error.
pub fn steiner_tree(g: &SparseGraph, arena: &[Solution], found: &[usize]) -> Result<SteinerTree, String> {
    let mut t = SteinerTree::default();
    if g.pseudo_pins.len() == 1 {
        let (p, l) = g.pseudo_pins[0];
        t.nodes.push(SteinerNode { p, fixed: l, children: Vec::new() });
        return Ok(t);
    }
    let point = |v: i32| {
        let (_, x, y) = g.vertices[v as usize];
        Point::new(x, y)
    };
    let mut created: HashMap<i32, usize> = HashMap::new();
    let mut root = None;
    for &f in found {
        let mut temp = Some(f);
        let mut last: Option<usize> = None;
        while let Some(id) = temp {
            let s = arena[id];
            match created.get(&s.vertex) {
                None => {
                    t.nodes.push(SteinerNode { p: point(s.vertex), fixed: Interval::default(), children: Vec::new() });
                    let node = t.nodes.len() - 1;
                    created.insert(s.vertex, node);
                    if let Some(l) = last {
                        t.nodes[node].children.push(l);
                    }
                    if s.prev.is_none() {
                        root = Some(node);
                    }
                    if last.is_none() || s.prev.is_none() {
                        let pin = *g.vertex_pin.get(&s.vertex).ok_or("GRT-0284: pin index not found for a path end")?;
                        t.nodes[node].fixed = g.pseudo_pins[pin].1;
                    }
                    last = Some(node);
                    temp = s.prev;
                }
                Some(&existing) => {
                    if let Some(l) = last {
                        t.nodes[existing].children.push(l);
                    }
                    break;
                }
            }
        }
    }
    t.root = root.ok_or("GRT-0285: Steiner tree construction failed")?;
    clean_up(&mut t);
    // (3) no co-located child may remain.
    for n in t.preorder() {
        if t.nodes[n].children.iter().any(|&c| t.nodes[c].p == t.nodes[n].p) {
            return Err("GRT-0276: duplicated tree nodes encountered".into());
        }
    }
    Ok(t)
}

/// `getSteinerTree`'s clean-up passes (1) and (2), in preorder from the root.
fn clean_up(t: &mut SteinerTree) {
    // (1) fold co-located children, preorder.
    fn fold(t: &mut SteinerTree, n: usize) {
        let mut ci = 0;
        while ci < t.nodes[n].children.len() {
            let child = t.nodes[n].children[ci];
            if t.nodes[n].p == t.nodes[child].p {
                let grand = t.nodes[child].children.clone();
                t.nodes[n].children.extend(grand);
                let cf = t.nodes[child].fixed;
                if cf.is_valid() && !t.nodes[n].fixed.is_valid() {
                    t.nodes[n].fixed = cf;
                }
                t.nodes[n].children.remove(ci);
                continue;
            }
            ci += 1;
        }
        for c in t.nodes[n].children.clone() {
            fold(t, c);
        }
    }
    let root = t.root;
    fold(t, root);
    // (2) skip layer-less single-child straight runs, preorder.
    fn skip(t: &mut SteinerTree, n: usize) {
        for ci in 0..t.nodes[n].children.len() {
            let child = t.nodes[n].children[ci];
            let d = if t.nodes[n].p.y == t.nodes[child].p.y { H } else { V };
            let mut temp = child;
            while !t.nodes[temp].fixed.is_valid() && t.nodes[temp].children.len() == 1 && t.nodes[temp].p.get(1 - d) == t.nodes[t.nodes[temp].children[0]].p.get(1 - d) {
                temp = t.nodes[temp].children[0];
            }
            t.nodes[n].children[ci] = temp;
        }
        for c in t.nodes[n].children.clone() {
            skip(t, c);
        }
    }
    skip(t, root);
}


/// Stage 4's search for one net: the sparsified graph, the search, the tree.
pub fn maze_tree(selected: &AccessPointMap, view: &View<f64>, grid: &SparseGrid, g: &GridGraph) -> Result<(SparseGraph, Vec<Solution>, Vec<usize>, SteinerTree), String> {
    let sg = SparseGraph::init(selected, view, grid, [g.x_size, g.y_size], g.unit_via_cost);
    let (arena, found) = run(&sg)?;
    let tree = steiner_tree(&sg, &arena, &found)?;
    Ok((sg, arena, found, tree))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A heap over plain keys, ordered as a min-heap (`a` ranks below `b` when larger).
    fn pops(keys: &[i32]) -> (Vec<usize>, Vec<usize>) {
        let less = |a: usize, b: usize| keys[a] > keys[b];
        let mut h = LibcxxHeap::new(&less);
        for i in 0..keys.len() {
            h.push(i);
        }
        let layout = h.v.clone();
        let mut out = Vec::new();
        while !h.is_empty() {
            out.push(h.top());
            h.pop();
        }
        (layout, out)
    }

    // Upstream rule (MazeRoute `SparseGraph::init`): every interval-th line from the offset below
    // the size, with each pin line inserted before the next kept line — unless it IS that line or
    // repeats the last one kept.
    #[test]
    fn sparse_lines_merge_pins() {
        assert_eq!(SparseGraph::lines(&[3, 3, 10, 24], 10, 0, 25), vec![0, 3, 10, 20, 24]);
        assert_eq!(SparseGraph::lines(&[0, 1], 10, 1, 12), vec![0, 1, 11]);
        let mut g = SparseGrid::new(10, 3, 9, 2);
        g.step();
        assert_eq!(g.offset, Point::new(0, 0), "each offset advances modulo its interval");
    }

    fn node(p: (i32, i32), fixed: Interval, children: &[usize]) -> SteinerNode {
        SteinerNode { p: Point::new(p.0, p.1), fixed, children: children.to_vec() }
    }

    // Upstream rule (MazeRoute `getSteinerTree` clean-up): (1) a co-located child is folded into
    // its parent — its children appended and visited in the same pass — and gives its layers only
    // to a parent WITHOUT any (the reference's `unionWith` result is discarded); (2) a run of
    // layer-less single-child nodes along one line is skipped to its end.
    #[test]
    fn maze_tree_clean_up() {
        // root (0,0)[pin 1..1] → a (0,0)[pin 2..2] → b (0,0)[none] → c (3,0) → d (6,0)[pin]
        let pin = |l| Interval::point(l);
        let mut t = SteinerTree { nodes: vec![
            node((0, 0), pin(1), &[1]),
            node((0, 0), pin(2), &[2]),
            node((0, 0), Interval::default(), &[3]),
            node((3, 0), Interval::default(), &[4]),
            node((6, 0), pin(1), &[]),
        ], root: 0, ..Default::default() };
        clean_up(&mut t);
        assert_eq!(t.nodes[0].fixed, pin(1), "the parent keeps its own layers: the union is discarded");
        assert_eq!(t.nodes[0].children, vec![4], "folded twice, then (3,0) skipped: straight to (6,0)");
    }
    // Upstream rule (libc++ 22 `push_heap` / `pop_heap`, the reference's `std::priority_queue`):
    // the pop order of EQUAL keys is fixed by the moves — sift-up on push; Floyd's descent to a
    // leaf through the strictly greater child, the last element dropped in and sifted up, on pop.
    // Expected sequences worked by hand from libc++'s source for these inputs.
    #[test]
    fn equal_keys_pop_in_libcxx_order() {
        // four equal keys: push leaves [0,1,2,3]; pop 0 → hole descends 0→1 (left: right not
        // strictly greater)→3; 3 is last → [1,3,2]; pop 1 → hole 0→1 (3), 1 is not last (len 3,
        // last 2): v[1]=2, v[2]=1, sift_up(2) — 3 vs 2 equal, stays → [3,2]; pop 3 → [2].
        assert_eq!(pops(&[5, 5, 5, 5]).1, vec![0, 1, 3, 2]);
        // a strict order pops in order whatever the layout
        assert_eq!(pops(&[3, 1, 2, 0]).1, vec![3, 1, 2, 0]);
        // mixed ties (a literal re-transcription of libc++'s source agrees)
        assert_eq!(pops(&[2, 2, 1, 2, 2, 1, 2]).1, vec![2, 5, 1, 3, 6, 4, 0]);
    }
}
