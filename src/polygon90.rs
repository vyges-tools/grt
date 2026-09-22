// SPDX-License-Identifier: Apache-2.0
//! Boost.Polygon's rectilinear polygon formation, as the antenna checker reaches it.
//!
//! The checker holds each layer's metal as a `std::vector<polygon_90_data<int>>` and grows or cuts
//! it with `+=` / `-=`. Every such operation runs a boolean op and then RE-DERIVES the vector from
//! the resulting region through `get_polygons(…, fracture_holes = true)` — so the polygons depend
//! only on the final REGION, and their order, vertex listing and hole slits are that function's
//! output. This module is that function:
//!
//! 1. [`Region`] — the region, exactly, on a compressed grid;
//! 2. [`get_polygons`] — per scan position (y, ascending: the set is HORIZONTAL, a horizontal
//!    scanline) the maximal LEFT edges (solid to the right, in the scan's own axes) and RIGHT
//!    edges, handed to [`ScanLine::process_edges`], the
//!    classic `ScanLineToPolygonItrs::processEdges` (the vertex-threshold variant is never taken:
//!    the threshold is `SIZE_MAX`);
//! 3. each closed figure written out through `ActiveTail::iterator` (as a hole-less figure of a
//!    HORIZONTAL set), `iterator_compact_to_points`, and `polygon_90_data`'s compact form.
//!
//! ⛔ A hole is FRACTURED into its shell by a slit (`ActiveTail::addHole`), and the slit's two sides
//! are edges of the polygon: `gtl::perimeter` counts them. The side area the checker charges
//! depends on where the slit falls, so the formation is transcribed, not approximated.
//!
//! The partial figures are linked structures in the reference (`PolyLine` chains joined head and
//! tail, `ActiveTail` pairs pointing at each other, assigned by value). Here they live in arenas
//! and are addressed by index; an `ActiveTail` assignment copies the three fields the reference's
//! `operator=` copies and leaves the hole list alone, as it does.

use std::collections::BTreeMap;

// ---- the region ----

/// A rectilinear region on a compressed grid: `cells[i][j]` covers `[xs[i], xs[i+1]) × [ys[j], ys[j+1])`.
#[derive(Debug, Clone)]
pub struct Region {
    pub xs: Vec<i32>,
    pub ys: Vec<i32>,
    pub cells: Vec<Vec<bool>>,
}

/// An axis-aligned rectangle, `(x0, y0, x1, y1)` with `x0 <= x1`, `y0 <= y1`.
pub type R = (i32, i32, i32, i32);

impl Region {
    /// `(∪ add) − (∪ sub)` — what a sequence of `+=` then `-=` leaves, whatever the order within
    /// each. Degenerate rectangles contribute nothing.
    pub fn new(add: &[R], sub: &[R]) -> Region {
        let mut xs: Vec<i32> = add.iter().chain(sub).flat_map(|r| [r.0, r.2]).collect();
        let mut ys: Vec<i32> = add.iter().chain(sub).flat_map(|r| [r.1, r.3]).collect();
        xs.sort_unstable();
        xs.dedup();
        ys.sort_unstable();
        ys.dedup();
        let nx = xs.len().saturating_sub(1);
        let ny = ys.len().saturating_sub(1);
        let mut cells = vec![vec![false; ny]; nx];
        let ix = |v: i32| xs.binary_search(&v).expect("a grid line");
        let iy = |v: i32| ys.binary_search(&v).expect("a grid line");
        for (rects, on) in [(add, true), (sub, false)] {
            for r in rects {
                for col in &mut cells[ix(r.0)..ix(r.2)] {
                    for c in &mut col[iy(r.1)..iy(r.3)] {
                        *c = on;
                    }
                }
            }
        }
        Region { xs, ys, cells }
    }

    /// The same region with x and y exchanged.
    pub fn transposed(&self) -> Region {
        let nx = self.cells.len();
        let ny = self.ys.len().saturating_sub(1);
        let cells = (0..ny).map(|j| (0..nx).map(|i| self.cells[i][j]).collect()).collect();
        Region { xs: self.ys.clone(), ys: self.xs.clone(), cells }
    }

    fn at(&self, i: usize, j: usize) -> bool {
        i < self.cells.len() && self.cells[i][j]
    }

    /// The maximal rectangles of each column run — a decomposition of the region.
    pub fn rects(&self) -> Vec<R> {
        let mut out = Vec::new();
        for (i, col) in self.cells.iter().enumerate() {
            let mut j = 0;
            while j < col.len() {
                if col[j] {
                    let j0 = j;
                    while j < col.len() && col[j] {
                        j += 1;
                    }
                    out.push((self.xs[i], self.ys[j0], self.xs[i + 1], self.ys[j]));
                } else {
                    j += 1;
                }
            }
        }
        out
    }

    /// Area, in grid units squared.
    pub fn area(&self) -> i64 {
        self.rects().iter().map(|r| i64::from(r.2 - r.0) * i64::from(r.3 - r.1)).sum()
    }

    /// Per scan position with a boundary: `(x, left edges, right edges)`, each list the maximal
    /// `[low, high]` intervals in ascending order. A LEFT edge has solid to its right.
    pub fn edges(&self) -> Vec<(i32, Vec<(i32, i32)>, Vec<(i32, i32)>)> {
        let ny = self.ys.len().saturating_sub(1);
        let mut out = Vec::new();
        for i in 0..self.xs.len() {
            let mut left: Vec<(i32, i32)> = Vec::new();
            let mut right: Vec<(i32, i32)> = Vec::new();
            for j in 0..ny {
                let before = i > 0 && self.at(i - 1, j);
                let after = self.at(i, j);
                let list = match (before, after) {
                    (false, true) => &mut left,
                    (true, false) => &mut right,
                    _ => continue,
                };
                let (lo, hi) = (self.ys[j], self.ys[j + 1]);
                match list.last_mut() {
                    Some(last) if last.1 == lo => last.1 = hi,
                    _ => list.push((lo, hi)),
                }
            }
            if !left.is_empty() || !right.is_empty() {
                out.push((self.xs[i], left, right));
            }
        }
        out
    }
}

// ---- the formation ----

const VERTICAL_HEAD: i32 = 1;
const HEAD_TO_TAIL: i32 = 2;
const TAIL_TO_TAIL: i32 = 4;
const HEAD: bool = false;
const TAIL: bool = true;
const HORIZONTAL: bool = false;
const VERTICAL: bool = true;

#[derive(Debug, Clone, Default)]
struct PolyLine {
    ptdata: Vec<i32>,
    headp: Option<usize>,
    tailp: Option<usize>,
    state: i32,
}

#[derive(Debug, Clone, Default)]
struct ActiveTail {
    tailp: usize,
    other: usize,
    holes: Vec<usize>,
    size: i64,
}

/// `ScanLineToPolygonItrs<…, polygon_90_concept>` with `fractureHoles_ = true`.
#[derive(Debug, Default)]
pub struct ScanLine {
    lines: Vec<PolyLine>,
    tails: Vec<ActiveTail>,
    tail_map: BTreeMap<i32, usize>,
    output: Vec<usize>,
}

impl ScanLine {
    // -- PolyLine --
    fn new_line(&mut self, orient: bool, coord: i32, side: bool) -> usize {
        self.lines.push(PolyLine { ptdata: vec![coord], headp: None, tailp: None, state: i32::from(orient) + (i32::from(side) << 3) });
        self.lines.len() - 1
    }
    fn vertical_head(&self, l: usize) -> bool {
        self.lines[l].state & VERTICAL_HEAD != 0
    }
    fn odd_length(&self, l: usize) -> bool {
        (self.lines[l].ptdata.len() - 1) % 2 != 0
    }
    fn tail_orient(&self, l: usize) -> bool {
        self.vertical_head(l) ^ self.odd_length(l)
    }
    fn head_to_tail(&self, l: usize) -> bool {
        self.lines[l].state & HEAD_TO_TAIL != 0
    }
    fn tail_to_tail(&self, l: usize) -> bool {
        self.lines[l].state & TAIL_TO_TAIL != 0
    }
    fn end_connectivity(&self, l: usize, end: bool) -> bool {
        if end {
            self.tail_to_tail(l)
        } else {
            self.head_to_tail(l)
        }
    }
    fn next(&self, l: usize, end: bool) -> Option<usize> {
        if end {
            self.lines[l].tailp
        } else {
            self.lines[l].headp
        }
    }
    fn end_coord(&self, l: usize, end: bool) -> i32 {
        let d = &self.lines[l].ptdata;
        if end {
            *d.last().expect("a coordinate")
        } else {
            d[0]
        }
    }
    fn segment_orient(&self, l: usize, index: usize) -> bool {
        self.vertical_head(l) ^ (index % 2 == 1)
    }
    /// `PolyLine::getPoint`: the coordinate at `index`, completed by the one before it (for the
    /// first, the connected end of the head neighbour).
    fn get_point(&self, l: usize, index: usize) -> (i32, i32) {
        let c = self.lines[l].ptdata[index];
        let prev = if index == 0 {
            let h = self.lines[l].headp.expect("a head neighbour");
            self.end_coord(h, self.head_to_tail(l))
        } else {
            self.lines[l].ptdata[index - 1]
        };
        // set(segmentOrient, prev): VERTICAL sets y, HORIZONTAL sets x.
        if self.segment_orient(l, index) == VERTICAL {
            (c, prev)
        } else {
            (prev, c)
        }
    }
    fn end_point(&self, l: usize) -> (i32, i32) {
        self.get_point(l, self.lines[l].ptdata.len() - 1)
    }
    /// `PolyLine::pushPoint`: a point colinear with the tail REMOVES the tail coordinate instead.
    fn push_point(&mut self, l: usize, p: (i32, i32)) {
        let vertical = self.tail_orient(l);
        if !self.lines[l].ptdata.is_empty() {
            let e = self.end_point(l);
            if if vertical { p.1 == e.1 } else { p.0 == e.0 } {
                self.lines[l].ptdata.pop();
                return;
            }
        }
        self.lines[l].ptdata.push(if vertical { p.1 } else { p.0 });
    }
    fn join_to_(&mut self, a: usize, this_end: bool, b: usize, end: bool) {
        let line = &mut self.lines[a];
        if this_end {
            line.tailp = Some(b);
            line.state &= !TAIL_TO_TAIL;
            line.state |= i32::from(end) << 2;
        } else {
            line.headp = Some(b);
            line.state &= !HEAD_TO_TAIL;
            line.state |= i32::from(end) << 1;
        }
    }
    fn join_to(&mut self, a: usize, this_end: bool, b: usize, end: bool) {
        self.join_to_(a, this_end, b, end);
        self.join_to_(b, end, a, this_end);
    }

    // -- ActiveTail --
    fn new_tail_pair(&mut self, x: i32, y: i32, solid: bool) -> (usize, usize) {
        let a1 = self.tails.len();
        let a2 = a1 + 1;
        let l1 = self.new_line(VERTICAL, x, solid);
        let l2 = self.new_line(HORIZONTAL, y, !solid);
        self.tails.push(ActiveTail { tailp: l1, other: a2, holes: Vec::new(), size: 1 });
        self.tails.push(ActiveTail { tailp: l2, other: a1, holes: Vec::new(), size: 1 });
        // joinHeadToHead
        self.join_to(l1, HEAD, l2, HEAD);
        self.tails[a1].size += 1;
        self.tails[a2].size += 1;
        (a1, a2)
    }
    fn orient(&self, t: usize) -> bool {
        self.tail_orient(self.tails[t].tailp)
    }
    fn coordinate(&self, t: usize) -> i32 {
        self.end_coord(self.tails[t].tailp, TAIL)
    }
    /// `ActiveTail::pushCoordinate`.
    fn push_coordinate(&mut self, t: usize, coord: i32) {
        let l = self.tails[t].tailp;
        // p = (coord, coord), then the perpendicular of the tail's orientation set to its coordinate
        let cur = self.coordinate(t);
        let p = if self.orient(t) == VERTICAL { (cur, coord) } else { (coord, cur) };
        let old = self.lines[l].ptdata.len() as i64;
        self.push_point(l, p);
        let delta = self.lines[l].ptdata.len() as i64 - old;
        self.tails[t].size += delta;
        let o = self.tails[t].other;
        self.tails[o].size += delta;
    }
    /// `ActiveTail::addHole`, fracturing: the hole's horizontal tail steps to the vertical tail's
    /// coordinate, and the hole's chain is joined into this figure. Returns the hole's VERTICAL
    /// tail, which carries on.
    fn add_hole(&mut self, this: usize, hole: usize) -> usize {
        let other = self.tails[hole].other;
        let (h, v) = if self.orient(other) == VERTICAL { (hole, other) } else { (other, hole) };
        let c = self.coordinate(v);
        self.push_coordinate(h, c);
        self.join_chains(this, h, false);
        v
    }
    /// `ActiveTail::joinChains` (the output-buffer form). Closing the figure through SOLID returns
    /// the hole handle; through space it outputs the shell. Otherwise the two chains become one.
    fn join_chains(&mut self, a1: usize, a2: usize, solid: bool) -> Option<usize> {
        if self.tails[a1].other == a2 {
            if solid {
                return Some(a1);
            }
            self.output.push(a1);
            let moved: Vec<usize> = std::mem::take(&mut self.tails[a2].holes);
            self.tails[a1].holes.extend(moved);
            return None;
        }
        let (l1, l2) = (self.tails[a1].tailp, self.tails[a2].tailp);
        self.join_to(l1, TAIL, l2, TAIL);
        let (o1, o2) = (self.tails[a1].other, self.tails[a2].other);
        let (ot1, ot2) = (self.tails[o1].tailp, self.tails[o2].tailp);
        // *(o1) = ActiveTail(o1.tailp, o2); *(o2) = ActiveTail(o2.tailp, o1): tailp, other and
        // size assigned (size 0), holes untouched.
        self.tails[o1].tailp = ot1;
        self.tails[o1].other = o2;
        self.tails[o1].size = 0;
        self.tails[o2].tailp = ot2;
        self.tails[o2].other = o1;
        self.tails[o2].size = 0;
        let accumulate = self.tails[a2].size + self.tails[a1].size;
        self.tails[o1].size = accumulate;
        self.tails[o2].size = accumulate;
        let h1: Vec<usize> = std::mem::take(&mut self.tails[a1].holes);
        let h2: Vec<usize> = std::mem::take(&mut self.tails[a2].holes);
        self.tails[o1].holes.extend(h1);
        self.tails[o1].holes.extend(h2);
        None
    }
    /// `createActiveTailsAsPair`: a new pair — or, with a hole coming up from below, the hole's
    /// two tails stepped to `(x, y)` and reused (the fracture filament).
    fn create_pair(&mut self, x: i32, y: i32, solid: bool, hole: Option<usize>) -> (usize, usize) {
        let Some(h) = hole else {
            return self.new_tail_pair(x, y, solid);
        };
        let at2 = if self.orient(h) == VERTICAL { h } else { self.tails[h].other };
        let at1 = self.tails[at2].other;
        self.push_coordinate(at1, x);
        self.push_coordinate(at2, y);
        (at1, at2)
    }

    // -- the map, with the reference's iterator semantics --
    fn after(&self, key: i32) -> Option<i32> {
        self.tail_map.range(key + 1..).next().map(|(&k, _)| k)
    }
    /// `findAtNext`: from the walk position, the entry at `key` — but a key BELOW the position is
    /// taken as absent, without looking.
    fn find_at_next(&self, pos: Option<i32>, key: i32) -> Option<i32> {
        match pos {
            None => self.tail_map.contains_key(&key).then_some(key),
            Some(p) if p < key => self.tail_map.contains_key(&key).then_some(key),
            Some(p) if p > key => None,
            Some(p) => Some(p),
        }
    }
    /// `std::map::insert`: an existing key keeps its value.
    fn insert(&mut self, key: i32, t: usize) {
        self.tail_map.entry(key).or_insert(t);
    }

    /// `processEdges` at one scan position. Returns the figures it closed, in closing order.
    pub fn process_edges(&mut self, x: i32, left: &[(i32, i32)], right: &[(i32, i32)]) -> Vec<Vec<(i32, i32)>> {
        self.output.clear();
        let (mut li, mut ri) = (0usize, 0usize);
        let mut bottom_done = false;
        let mut current: Option<usize> = None;
        let mut next_pos: Option<i32> = self.tail_map.keys().next().copied();
        const MAX: i32 = i32::MAX;
        while li < left.len() || ri < right.len() {
            let mut edges = [(MAX, MAX), (MAX, MAX)];
            let mut have_next = true;
            if li < left.len() {
                edges[0] = left[li];
            } else {
                have_next = false;
            }
            if ri < right.len() {
                edges[1] = right[ri];
            } else {
                have_next = false;
            }
            let trailing = edges[1].0 < edges[0].0;
            let edge = edges[usize::from(trailing)];
            let next_edge = edges[usize::from(!trailing)];
            if !bottom_done {
                if let Some(k) = self.find_at_next(next_pos, edge.0) {
                    let tail = self.tail_map[&k];
                    let c = match current {
                        Some(ct) => self.add_hole(tail, ct),
                        None => {
                            self.push_coordinate(tail, x);
                            tail
                        }
                    };
                    current = Some(c);
                    next_pos = self.after(k);
                    self.tail_map.remove(&k);
                } else {
                    let (a1, a2) = self.create_pair(x, edge.0, !trailing, current);
                    current = Some(a1);
                    self.insert(edge.0, a2);
                }
            }
            if have_next && edge.1 == next_edge.0 {
                bottom_done = true;
                let Some(k) = self.find_at_next(next_pos, edge.1) else {
                    break; // "assert this should never happen": the reference returns
                };
                if trailing {
                    let tail = self.tail_map[&k];
                    self.join_chains(current.expect("a current tail"), tail, false);
                    let (a1, a2) = self.create_pair(x, edge.1, true, None);
                    current = Some(a1);
                    self.tail_map.insert(k, a2);
                } else {
                    let ct = current.expect("a current tail");
                    self.push_coordinate(ct, edge.1);
                    let t = self.tail_map[&k];
                    self.push_coordinate(t, x);
                    self.tail_map.insert(k, ct);
                    current = Some(t);
                }
                next_pos = self.after(k);
            } else {
                bottom_done = false;
                if let Some(k) = self.find_at_next(next_pos, edge.1) {
                    let tail = self.tail_map[&k];
                    current = self.join_chains(current.expect("a current tail"), tail, !trailing);
                    next_pos = self.after(k);
                    if let Some(ct) = current {
                        let next_y = next_pos.unwrap_or(MAX);
                        let left_y = if li + 1 < left.len() { left[li + 1].0 } else { MAX };
                        let right_y = next_edge.0;
                        if !have_next || (next_y < left_y && next_y < right_y) {
                            let nk = next_pos.expect("a figure above the hole");
                            let above = self.tail_map[&nk];
                            let t = self.add_hole(above, ct);
                            self.push_coordinate(t, nk);
                            self.tail_map.insert(nk, t);
                            current = None;
                        }
                    }
                    self.tail_map.remove(&k);
                } else {
                    let ct = current.expect("a current tail");
                    self.push_coordinate(ct, edge.1);
                    self.insert(edge.1, ct);
                    current = None;
                }
            }
            li += usize::from(!trailing);
            ri += usize::from(trailing);
        }
        let closed: Vec<usize> = std::mem::take(&mut self.output);
        closed.into_iter().map(|t| self.write_out(t)).collect()
    }

    /// A closed figure's points as `polygon_90_data` stores and lists them: `ActiveTail::iterator`
    /// (hole-less figure of a HORIZONTAL set — it starts from the OTHER tail) gives compact
    /// coordinates, `iterator_compact_to_points` turns them into points, and `polygon_90_data::set`
    /// / `begin` round-trip them through its own compact form.
    fn write_out(&mut self, at: usize) -> Vec<(i32, i32)> {
        let compact = self.compact(at);
        let pts = compact_to_points(&compact);
        let coords: Vec<i32> = pts.iter().enumerate().map(|(i, p)| if i % 2 == 0 { p.0 } else { p.1 }).collect();
        compact_to_points(&coords)
    }

    /// `ActiveTail::iterator` from construction to end, for `isHole = true` (the polygon_90 output
    /// type) and a HORIZONTAL scan: `!isHole ^ (orient == HORIZONTAL)` is true, so it starts from
    /// the other tail. Construction JOINS the two tails, closing the chain.
    fn compact(&mut self, at0: usize) -> Vec<i32> {
        // `!isHole ^ (orient == HORIZONTAL)`: isHole true and a HORIZONTAL set — switch tails.
        let at = self.tails[at0].other;
        let mut start_end = TAIL;
        let mut p_line = self.tails[at].tailp;
        let n = self.lines[p_line].ptdata.len();
        let mut index = if n > 0 { n - 1 } else { 0 };
        let (p_end, index_end);
        // (at.orient == HORIZONTAL) ^ (orient == HORIZONTAL), orient HORIZONTAL
        if self.orient(at) != HORIZONTAL {
            p_end = self.tails[at].tailp;
            index_end = self.lines[p_end].ptdata.len() - 1;
            if index == 0 {
                let tl = self.tails[at].tailp;
                p_line = self.next(tl, HEAD).expect("a head neighbour");
                if self.end_connectivity(tl, HEAD) == TAIL {
                    index = self.lines[p_line].ptdata.len() - 1;
                } else {
                    start_end = HEAD;
                    index = 0;
                }
            } else {
                index -= 1;
            }
        } else {
            let o = self.tails[at].other;
            p_end = self.tails[o].tailp;
            let m = self.lines[p_end].ptdata.len();
            index_end = if m > 0 { m - 1 } else { 0 };
        }
        let (a, b) = (self.tails[at].tailp, self.tails[self.tails[at].other].tailp);
        self.join_to(a, TAIL, b, TAIL);
        let mut out = Vec::new();
        let mut line = Some(p_line);
        while let Some(l) = line {
            out.push(self.lines[l].ptdata[index]);
            if l == p_end && index == index_end {
                line = None;
                continue;
            }
            if start_end == HEAD {
                index += 1;
                if index == self.lines[l].ptdata.len() {
                    let end = self.end_connectivity(l, TAIL);
                    let nl = self.next(l, TAIL).expect("a tail neighbour");
                    line = Some(nl);
                    if end == TAIL {
                        start_end = TAIL;
                        index = self.lines[nl].ptdata.len() - 1;
                    } else {
                        index = 0;
                    }
                }
            } else if index == 0 {
                let end = self.end_connectivity(l, HEAD);
                let nl = self.next(l, HEAD).expect("a head neighbour");
                line = Some(nl);
                if end == TAIL {
                    index = self.lines[nl].ptdata.len() - 1;
                } else {
                    start_end = HEAD;
                    index = 0;
                }
            } else {
                index -= 1;
            }
        }
        out
    }
}

/// `iterator_compact_to_points` walked to its end: the first point is `(c0, c1)`, then each next
/// coordinate replaces x or y alternately (y first); at the end, a last point whose x is not the
/// first x gets one more point back at the first x.
pub fn compact_to_points(c: &[i32]) -> Vec<(i32, i32)> {
    if c.is_empty() {
        return Vec::new();
    }
    let first_x = c[0];
    let mut pt = (first_x, if c.len() > 1 { c[1] } else { 0 });
    let mut out = vec![pt];
    let mut i = 1;
    let mut orient_x = true; // orient_ starts HORIZONTAL: the next coordinate sets x
    loop {
        let prev = i;
        i += 1;
        if i >= c.len() {
            if pt.0 != first_x {
                i = prev;
                pt.0 = first_x;
                out.push(pt);
                // the iterator now sits at `prev` with x == firstX: equal to end, stop.
            }
            break;
        }
        if orient_x {
            pt.0 = c[i];
        } else {
            pt.1 = c[i];
        }
        orient_x = !orient_x;
        out.push(pt);
    }
    let _ = i;
    out
}

/// `get_polygons` over a region, as a `std::vector<polygon_90_data>` receives it: a HORIZONTAL set,
/// which in Boost's terms is a horizontal SCANLINE — positions are y, ascending, and each
/// position's edges run along x — so the scan runs over the TRANSPOSED region, and the figures
/// closed at each position come out in closing order.
///
/// ⛔ Found by measurement against the probe (408/408): the transposed scan with the output
/// iterator's HORIZONTAL branches. Every other combination of axis and branch fails most cases.
pub fn get_polygons(region: &Region) -> Vec<Vec<(i32, i32)>> {
    let t = region.transposed();
    let mut sl = ScanLine::default();
    let mut out = Vec::new();
    for (pos, left, right) in t.edges() {
        out.extend(sl.process_edges(pos, &left, &right));
    }
    out
}

/// `gtl::area` of one polygon (shoelace, absolute).
pub fn polygon_area(p: &[(i32, i32)]) -> i64 {
    let n = p.len();
    let mut a: i64 = 0;
    for i in 0..n {
        let (x0, y0) = p[i];
        let (x1, y1) = p[(i + 1) % n];
        a += i64::from(x0) * i64::from(y1) - i64::from(x1) * i64::from(y0);
    }
    (a / 2).abs()
}

/// `gtl::perimeter` of one polygon: every edge, the closing one included — hole slits twice.
pub fn polygon_perimeter(p: &[(i32, i32)]) -> i64 {
    let n = p.len();
    (0..n).map(|i| {
        let (a, b) = (p[i], p[(i + 1) % n]);
        i64::from((a.0 - b.0).abs()) + i64::from((a.1 - b.1).abs())
    }).sum()
}
