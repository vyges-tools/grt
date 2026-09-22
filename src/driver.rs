// SPDX-License-Identifier: Apache-2.0
//! G — `globalRoute`, the driver around the setup (I), the router (R) and the finish (X):
//! G1 the routable-net scan (GRT-7), G3 `getMinMaxLayer`, G5 `reportResources` (GRT-53).

/// G1 — `globalRoute`'s early return: is there ANY net with at least two instance terminals, at
/// least two block terminals, or one of each? The scan stops at the first. `nets` yields
/// `(iterm count, bterm count)` in database order; `false` means GRT-7 and no routing at all.
pub fn has_routable_nets(nets: impl IntoIterator<Item = (usize, usize)>) -> bool {
    nets.into_iter().any(|(i, b)| i > 1 || b > 1 || (i > 0 && b > 0))
}

/// The GRT-7 warning G1 issues before returning.
pub const GRT_7: &str = "[WARNING GRT-0007] Design does not have any routable net (with at least 2 terms)";

/// `computeMaxRoutingLayer`: the last routing level of the unbroken run, from level 1, that has a
/// track grid (GRT-701 when level 1 has none: `None`).
pub fn compute_max_routing_layer(has_track_grid: &[bool]) -> Option<i32> {
    let n = has_track_grid.iter().take_while(|&&t| t).count() as i32;
    (n > 0).then_some(n)
}

/// G3 — `getMinMaxLayer` → `(new block max, min, max)`.
///
/// An unset (−1) max routing layer is first COMPUTED and stored ([`compute_max_routing_layer`]).
/// ⛔ Asymmetric: the minimum takes the clock minimum only when it is set (> 0), the maximum is
/// `max(block max, clock max)` unconditionally.
pub fn get_min_max_layer(
    block_max: i32,
    has_track_grid: &[bool],
    block_min: i32,
    clock_min: i32,
    clock_max: i32,
) -> Option<(i32, i32, i32)> {
    let block_max = if block_max == -1 { compute_max_routing_layer(has_track_grid)? } else { block_max };
    let min = if clock_min > 0 { block_min.min(clock_min) } else { block_min };
    Some((block_max, min, block_max.max(clock_max)))
}

/// One router layer's edges as `reportResources` reads them: `(cap, real_cap)` per edge.
pub struct ResourceLayer<'a> {
    pub name: &'a str,
    pub horizontal: bool,
    /// Every horizontal 3D edge (`x < x_grid - 1`), any order.
    pub h_edges: &'a [(u16, u16)],
    /// Every vertical 3D edge (`y < y_grid - 1`), any order.
    pub v_edges: &'a [(u16, u16)],
}

/// G5 — `reportResources`: GRT-53 and the table.
///
/// Per layer: ORIGINAL resources = the real capacities along its PREFERRED direction only
/// (`getOriginalResources`); DERATED = the current capacities in BOTH directions
/// (`computeCongestionInformation`'s `cap_per_layer_`); reduction
/// `(1.0 - (float) derated / (float) original) * 100` — the `1.0` is a `double`, the result stored
/// in a `float` — and 0 when there is no original resource.
pub fn report_resources(layers: &[ResourceLayer<'_>], log: &mut Vec<String>) {
    log.push(String::new());
    log.push("[INFO GRT-0053] Routing resources analysis:".into());
    log.push("          Routing      Original      Derated      Resource".into());
    log.push("Layer     Direction    Resources     Resources    Reduction (%)".into());
    log.push("---------------------------------------------------------------".into());
    for l in layers {
        let preferred = if l.horizontal { l.h_edges } else { l.v_edges };
        let original: i32 = preferred.iter().map(|e| e.1 as i32).sum();
        let derated: i32 = l.h_edges.iter().chain(l.v_edges).map(|e| e.0 as i32).sum();
        let reduction = if original > 0 { ((1.0 - (derated as f32 / original as f32) as f64) * 100.0) as f32 } else { 0.0 };
        log.push(format!(
            "{:<7}    {:<10}   {:>8}      {:>8}          {:>3.2}%",
            l.name,
            if l.horizontal { "Horizontal" } else { "Vertical" },
            original,
            derated,
            reduction
        ));
    }
    log.push("---------------------------------------------------------------".into());
    log.push(String::new());
}
