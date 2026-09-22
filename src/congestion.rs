// SPDX-License-Identifier: Apache-2.0
//! Xb — the congestion the router leaves in the database: X1 [`update_db_congestion`] (the gcell
//! grid's capacity and usage per layer) and X2's markers ([`congestion_markers`]: every overflowing
//! 2D cell, the nets crossing it, in a seeded-shuffled order).

use std::collections::{BTreeMap, BTreeSet};

/// One 3D edge as the congestion writer reads it (`uint16_t` each).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CongestionEdge {
    pub cap: u16,
    pub red: u16,
    pub usage: u16,
}

/// One routing layer's 3D edges, `[y][x]` flattened `y * x_grid + x`, including the last column /
/// row (they are read).
pub struct DbCongestionLayer<'a> {
    pub horizontal: bool,
    pub h: &'a [CongestionEdge],
    pub v: &'a [CongestionEdge],
}

/// X1 — `FastRouteCore::updateDbCongestion` for one layer → `(capacity, usage)` per gcell, `[y][x]`,
/// as the grid stores them (`float`).
///
/// ⛔ Rules that decide values:
/// - capacity = `cap + red` of the edge along the layer's direction, narrowed to `uint8_t` (mod 256);
///   the LAST cell in that direction repeats the previous cell's value (the walk is x-inner for a
///   horizontal layer, y-inner for a vertical one, and the carried value runs on across rows);
/// - usage = `uint8_t(h.usage + h.red) + uint8_t(v.usage + v.red)` of the cell's own edges — but the
///   TOP-RIGHT cell (on a grid larger than 1 × 1) takes the reductions from its own left / lower edges
///   and the usages from the DIAGONAL cell `(x-1, y-1)`.
pub fn update_db_congestion(x_grid: i32, y_grid: i32, l: &DbCongestionLayer<'_>) -> Vec<(f32, f32)> {
    let (xg, yg) = (x_grid as usize, y_grid as usize);
    let at = |e: &[CongestionEdge], x: usize, y: usize| e[y * xg + x];
    let mut cells = vec![(0.0f32, 0.0f32); xg * yg];
    let mut last = 0i32;
    if l.horizontal {
        for y in 0..yg {
            for x in 0..xg {
                let cap = if x == xg - 1 { last as u8 } else { { let e = at(l.h, x, y); (e.cap as i32 + e.red as i32) as u8 } };
                cells[y * xg + x].0 = cap as f32;
                last = cap as i32;
            }
        }
    } else {
        for x in 0..xg {
            for y in 0..yg {
                let cap = if y == yg - 1 { last as u8 } else { { let e = at(l.v, x, y); (e.cap as i32 + e.red as i32) as u8 } };
                cells[y * xg + x].0 = cap as f32;
                last = cap as i32;
            }
        }
    }
    for y in 0..yg {
        for x in 0..xg {
            let usage = if x == xg - 1 && y == yg - 1 && xg > 1 && yg > 1 {
                let block_h = at(l.h, x - 1, y).red as u8;
                let block_v = at(l.v, x, y - 1).red as u8;
                let usage_h = (at(l.h, x - 1, y - 1).usage as i32 + block_h as i32) as u8;
                let usage_v = (at(l.v, x - 1, y - 1).usage as i32 + block_v as i32) as u8;
                usage_h as i32 + usage_v as i32
            } else {
                let (h, v) = (at(l.h, x, y), at(l.v, x, y));
                let usage_h = (h.usage as i32 + h.red as u8 as i32) as u8;
                let usage_v = (v.usage as i32 + v.red as u8 as i32) as u8;
                usage_h as i32 + usage_v as i32
            };
            cells[y * xg + x].1 = usage as f32;
        }
    }
    cells
}

// ─── The seeded shuffle ─────────────────────────────────────────────────────────────────────

/// `std::mt19937` (MT19937, 32-bit).
pub struct Mt19937 {
    mt: [u32; 624],
    index: usize,
}

impl Mt19937 {
    pub fn new(seed: u32) -> Self {
        let mut mt = [0u32; 624];
        mt[0] = seed;
        for i in 1..624 {
            mt[i] = 1812433253u32.wrapping_mul(mt[i - 1] ^ (mt[i - 1] >> 30)).wrapping_add(i as u32);
        }
        Mt19937 { mt, index: 624 }
    }

    pub fn next_u32(&mut self) -> u32 {
        if self.index >= 624 {
            for i in 0..624 {
                let y = (self.mt[i] & 0x8000_0000) | (self.mt[(i + 1) % 624] & 0x7fff_ffff);
                let mut v = self.mt[(i + 397) % 624] ^ (y >> 1);
                if y & 1 != 0 {
                    v ^= 0x9908_b0df;
                }
                self.mt[i] = v;
            }
            self.index = 0;
        }
        let mut y = self.mt[self.index];
        self.index += 1;
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c_5680;
        y ^= (y << 15) & 0xefc6_0000;
        y ^ (y >> 18)
    }
}

/// boost's `generate_uniform_int(eng, 0, range)` for an engine spanning `[0, 2^32 - 1]` whose
/// result type is 64-bit (`std::mt19937::result_type` = `uint_fast32_t` on x86-64 Linux): the
/// engine range is then NOT the unsigned type's maximum, so the bucket is `(brange + 1) / (range +
/// 1)`, and draws past the last full bucket are rejected.
fn boost_uniform(eng: &mut Mt19937, range: u64) -> u64 {
    boost_uniform_int(|| eng.next_u32(), range)
}

/// [`boost_uniform`] over any 32-bit draw source — the bucket step itself, exposed so the
/// rejection branch (probability ~ `range / 2^32` per draw) can be exercised directly.
pub fn boost_uniform_int(mut draw: impl FnMut() -> u32, range: u64) -> u64 {
    if range == 0 {
        return 0;
    }
    let brange: u64 = 0xffff_ffff;
    let bucket_size = (brange + 1) / (range + 1);
    loop {
        let result = draw() as u64 / bucket_size;
        if result <= range {
            return result;
        }
    }
}

/// `utl::shuffle` — the platform-independent Fisher–Yates: from the back, swap each position `i`
/// with a draw in `[0, i]` (`variate_generator(g, uniform_int)(i + 1)`).
pub fn utl_shuffle<T>(v: &mut [T], g: &mut Mt19937) {
    let n = v.len();
    if n <= 1 {
        return;
    }
    for i in (1..n).rev() {
        let j = boost_uniform(g, i as u64) as usize;
        v.swap(i, j);
    }
}

// ─── The markers ────────────────────────────────────────────────────────────────────────────

/// One overflowing 2D cell: `(j, i, capacity, usage)` in grid indices.
pub type CongestedCell = (i32, i32, i32, i32);

/// A net's routed tree edges (len > 0), each its grid points in order — `findCongestedEdgesNets`.
pub struct CrossingNet<'a> {
    pub name: &'a str,
    /// The database id, which orders a marker's sources (`odb::PtrSet`).
    pub id: u32,
    pub edges: &'a [Vec<(i32, i32)>],
}

/// One marker: its cell centre, capacity, usage and source nets (by database id).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CongestionMarker {
    pub x: i32,
    pub y: i32,
    pub capacity: i32,
    pub usage: i32,
    pub sources: Vec<String>,
}

/// The real position of a cell index: `tile * (j + 0.5) + corner` in `double`, truncated.
fn real(tile: i32, j: i32, corner: i32) -> i32 {
    (tile as f64 * (j as f64 + 0.5) + corner as f64) as i32
}

/// `getCongestionGrid` for one direction: the overflowing cells keyed by their centre (a `std::map`,
/// so sorted by `(x, y)`), each with the nets whose routed steps ALONG that direction start there
/// (a step's congestion lives at its lower-left end: `(min x, min y)`).
fn congestion_grid(cells: &[CongestedCell], nets: &[CrossingNet<'_>], vertical: bool, tile: i32, corner: (i32, i32)) -> Vec<CongestionMarker> {
    let mut map: BTreeMap<(i32, i32), (i32, i32, BTreeSet<(u32, String)>)> = BTreeMap::new();
    for &(j, i, cap, usage) in cells {
        map.insert((real(tile, j, corner.0), real(tile, i, corner.1)), (cap, usage, BTreeSet::new()));
    }
    for n in nets {
        for edge in n.edges {
            let (mut lx, mut ly) = (real(tile, edge[0].0, corner.0), real(tile, edge[0].1, corner.1));
            for p in &edge[1..] {
                let (x, y) = (real(tile, p.0, corner.0), real(tile, p.1, corner.1));
                if lx == x && ly == y {
                    continue;
                }
                if (x == lx) == vertical {
                    if let Some(e) = map.get_mut(&(lx.min(x), ly.min(y))) {
                        e.2.insert((n.id, n.name.to_string()));
                    }
                }
                lx = x;
                ly = y;
            }
        }
    }
    map.into_iter()
        .map(|((x, y), (capacity, usage, s))| CongestionMarker { x, y, capacity, usage, sources: s.into_iter().map(|p| p.1).collect() })
        .collect()
}

/// X2 — `FastRouteCore::saveCongestion`'s marker lists: horizontal then vertical
/// [`congestion_grid`]s, each shuffled by [`utl_shuffle`] with ONE `std::mt19937` seeded 42 (the
/// vertical shuffle continues the horizontal's generator).
pub fn congestion_markers(
    horizontal: &[CongestedCell],
    vertical: &[CongestedCell],
    nets: &[CrossingNet<'_>],
    tile: i32,
    corner: (i32, i32),
) -> (Vec<CongestionMarker>, Vec<CongestionMarker>) {
    let mut h = congestion_grid(horizontal, nets, false, tile, corner);
    let mut v = congestion_grid(vertical, nets, true, tile, corner);
    let mut g = Mt19937::new(42);
    utl_shuffle(&mut h, &mut g);
    utl_shuffle(&mut v, &mut g);
    (h, v)
}
