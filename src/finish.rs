// SPDX-License-Identifier: Apache-2.0
//! X — `finishGlobalRouting`'s reports and verdict: X3 [`report_congestion`] (GRT-96), X4
//! [`compute_wirelength`] (GRT-18), X5 GRT-14, X7 [`suggest_adjustment`] (GRT-704) and
//! [`congestion_verdict`] (GRT-115 / GRT-116).

/// `computeNetWirelength`: every segment that MOVES (`|dx| + |dy| > 0`) counts its length PLUS one
/// cell — vias count nothing. Takes each segment's `|dx| + |dy|`.
pub fn compute_net_wirelength(segment_lengths: &[i32], tile_size: i32) -> i32 {
    segment_lengths.iter().filter(|&&l| l > 0).map(|&l| l + tile_size).sum()
}

/// X4 — `computeWirelength`: the routes' total, in DEF units, divided (INTEGER division, `int64_t`)
/// by the DEF units per micron — GRT-18 when verbose. Returns the reported microns.
pub fn compute_wirelength(routes: &[Vec<i32>], tile_size: i32, def_units: i32, verbose: bool, log: &mut Vec<String>) -> i64 {
    let total: i64 = routes.iter().map(|r| compute_net_wirelength(r, tile_size) as i64).sum();
    let um = total / def_units as i64;
    if verbose {
        log.push(format!("[INFO GRT-0018] Total wirelength: {um} um"));
    }
    um
}

/// X5 — GRT-14, the size of the route map (nets given guides included), when verbose.
pub fn report_routed_nets(routes: usize, verbose: bool, log: &mut Vec<String>) {
    if verbose {
        log.push(format!("[INFO GRT-0014] Routed nets: {routes}"));
    }
}

/// One router layer's edges as the congestion report reads them: `(cap, usage)` per edge.
pub struct CongestionLayer<'a> {
    pub name: &'a str,
    /// Every horizontal 3D edge (`x < x_grid - 1`).
    pub h_edges: &'a [(u16, u16)],
    /// Every vertical 3D edge (`y < y_grid - 1`).
    pub v_edges: &'a [(u16, u16)],
}

/// X3 — `reportCongestion` (after `computeCongestionInformation`): GRT-96 and the table.
///
/// Per layer over BOTH directions: resource = Σ cap, demand = Σ usage, overflow = Σ max(usage −
/// cap, 0); the largest horizontal and vertical overflow; usage `(float) d / (float) r`, then
/// `*= 100` in float (0 without resources). The total's usage is `(float) d / (float) r * 100`.
pub fn report_congestion(layers: &[CongestionLayer<'_>], log: &mut Vec<String>) {
    log.push(String::new());
    log.push("[INFO GRT-0096] Final congestion report:".into());
    log.push("Layer         Resource        Demand        Usage (%)    Max H / Max V / Total Congestion".into());
    log.push("---------------------------------------------------------------------------------------".into());
    let (mut tr, mut td, mut to, mut th, mut tv) = (0i32, 0i32, 0i32, 0i32, 0i32);
    for l in layers {
        let (mut r, mut d, mut o, mut mh, mut mv) = (0i32, 0i32, 0i32, 0i32, 0i32);
        for (dir, edges) in [(true, l.h_edges), (false, l.v_edges)] {
            for &(cap, usage) in edges {
                r += cap as i32;
                d += usage as i32;
                let over = usage as i32 - cap as i32;
                if over > 0 {
                    o += over;
                    if dir { mh = mh.max(over) } else { mv = mv.max(over) }
                }
            }
        }
        let usage = if r == 0 { 0.0f32 } else { (d as f32 / r as f32) * 100.0 };
        tr += r;
        td += d;
        to += o;
        th += mh;
        tv += mv;
        log.push(format!("{:<7}      {:>9}       {:>7}        {:>8.2}%            {:>2} / {:>2} / {:>2}", l.name, r, d, usage, mh, mv, o));
    }
    let total_usage = if tr == 0 { 0.0f32 } else { td as f32 / tr as f32 * 100.0 };
    log.push("---------------------------------------------------------------------------------------".into());
    log.push(format!("Total        {:>9}       {:>7}        {:>8.2}%            {:>2} / {:>2} / {:>2}", tr, td, total_usage, th, tv, to));
    log.push(String::new());
}

/// One overflowing used grid cell as `computeSuggestedAdjustment` reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CongestedGrid {
    /// 2D real capacity and usage.
    pub real_cap: i32,
    pub usage: i32,
    /// Per-layer 3D real capacity of the edge.
    pub layer_real_caps: Vec<i32>,
}

/// `Graph2D::computeSuggestedAdjustment` + `getPrecisionAdjustment` for one cell: `None` when the
/// real capacity is below the usage (and the whole suggestion is then abandoned).
///
/// The first guess is `(1.0 - usage / real) * 100` (float division, double arithmetic, truncated);
/// then up to 5 times, while the adjusted capacity — `Σ (1.0 - adj / 100.0) * real_l`, an `int`
/// accumulated from doubles, TRUNCATED at every addition — is below the usage, the adjustment drops
/// by one.
pub fn cell_suggestion(g: &CongestedGrid) -> Option<i32> {
    let (real, usage) = (g.real_cap as f32, g.usage as f32);
    if real < usage {
        return None;
    }
    let mut adjustment = ((1.0 - (usage / real) as f64) * 100.0) as i32;
    let mut new_cap = 0i32;
    let mut range = 5;
    while range > 0 && new_cap < g.usage {
        for &rc in &g.layer_real_caps {
            new_cap = (new_cap as f64 + (1.0 - adjustment as f64 / 100.0) * rc as f64) as i32;
        }
        if g.usage > new_cap {
            adjustment -= 1;
            new_cap = 0;
        }
        range -= 1;
    }
    Some(adjustment)
}

/// `computeSuggestedAdjustment`: the minimum cell suggestion over the overflowing horizontal cells,
/// then the vertical, each from 100; `None` as soon as any cell has none.
pub fn compute_suggested_adjustment(horizontal: &[CongestedGrid], vertical: &[CongestedGrid]) -> Option<i32> {
    let mut h = 100;
    for g in horizontal {
        h = h.min(cell_suggestion(g)?);
    }
    let mut v = 100;
    for g in vertical {
        v = v.min(cell_suggestion(g)?);
    }
    Some(h.min(v))
}

/// X7 — `suggestAdjustment`: GRT-704 when a suggestion exists BELOW the smallest non-zero layer
/// adjustment in range (as a float percentage, starting from 1.0 = 100%).
pub fn suggest_adjustment(layer_adjustments: &[f32], suggestion: Option<i32>, log: &mut Vec<String>) {
    let mut min_adjustment = 1.0f32;
    for &a in layer_adjustments {
        if a != 0.0 {
            min_adjustment = min_adjustment.min(a);
        }
    }
    min_adjustment *= 100.0;
    if let Some(s) = suggestion {
        if min_adjustment > s as f32 {
            log.push(format!("[WARNING GRT-0704] Try reduce the layer adjustment from {min_adjustment}% to {s}%"));
        }
    }
}

/// X7 — the verdict on a congested run: a warning (GRT-115) with `-allow_congestion` or CUGR, else
/// an error (GRT-116). Returns false on the error.
pub fn congestion_verdict(congested: bool, allow_congestion: bool, use_cugr: bool, log: &mut Vec<String>) -> bool {
    if !congested {
        return true;
    }
    const MSG: &str = "Global routing finished with congestion. Check the congestion regions in the DRC Viewer.";
    if allow_congestion || use_cugr {
        log.push(format!("[WARNING GRT-0115] {MSG}"));
        true
    } else {
        log.push(format!("[ERROR GRT-0116] {MSG}"));
        false
    }
}
