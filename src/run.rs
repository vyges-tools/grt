// SPDX-License-Identifier: Apache-2.0
//! `FastRouteCore::run()` — the router, R5 to R20, as ONE thin sequencer over the shared state.
//!
//! Nothing here decides anything: each line is one reference stage, in the reference's order, and
//! the stages live in their own modules. An observer sees the state at every boundary the
//! correlation captures (and may stop the run there, where a capture was trimmed), so the replay
//! checks THIS sequence rather than a copy of it.
//!
//! What the stages before `gen_brk_RSMT` do is not modelled: `clearUsed` (the used sets start
//! empty here), `preProcessTechLayers` (resistance-aware only) and run()'s constants.

use std::collections::BTreeMap;

use crate::brk_rsmt::{gen_brk_rsmt, BrkFlags, BrkGrid, BrkSummary, Caps3D, Flutes, NetState, RsmtNet, RsmtTree};
use crate::checks3d::Overflow3D;
use crate::congestion_loop::{congestion_loop, LoopEnd, LoopEvent, LoopStart};
use crate::estimate::EstimateGrid;
use crate::finalize::{
    finish_3d, get_overflow_3d_all, get_routes_all, layer_assignment, maze_route_msmd_order_3d_all, remove_loops_all, Finish3d, Graph3d,
    LayerParams, Maze3dParams, NetLayerAttrs,
};
use crate::graph2d::Graph2d;
use crate::layertable::LayerDir;
use crate::maze_phase::{convert_to_mazeroute_all, init_for_congestion_loop, lv_rounds, LvRound};
use crate::ndr_cost::NdrLedger;
use crate::overflow2d::Overflow2DScan;
use crate::route_l::{newroute_l_all, newroute_z_all, route_l_all, spiral_route_all};
use crate::routes::GridOrigin;
use crate::GSegment;

/// The reference's `BIG_INT`.
const BIG_INT: i32 = 1_000_000_000;

/// Everything run() reads that setup (I) produced.
pub struct RunInputs<'a> {
    pub x_grid: usize,
    pub y_grid: usize,
    /// `h_capacity_` / `v_capacity_`.
    pub h_capacity: i32,
    pub v_capacity: i32,
    /// Per 2D edge, `[y * x_grid + x]`: the capacity reduction, the capacity, and the estimated
    /// usage at entry (zero outside an incremental run).
    pub red_h: &'a [u16],
    pub red_v: &'a [u16],
    pub cap_h: &'a [u16],
    pub cap_v: &'a [u16],
    pub entry: EstimateGrid,
    /// The 3D capacities (`h_edges_3D_` / `v_edges_3D_`).
    pub caps: &'a Caps3D,
    pub net_ids: &'a [usize],
    /// Indexed by net id.
    pub nets: &'a [RsmtNet<'a>],
    pub attrs: &'a [NetLayerAttrs],
    /// Each net's slack and critical flag as setup left them (`FrNet::getSlack`, `isCritical`).
    pub slack: &'a [(f32, bool)],
    /// The Steiner-tree engine's tree for a net (R5's `stt` builder) and FLUTE (R5/R7).
    pub stt: &'a dyn Fn(usize) -> RsmtTree,
    pub flutes: Flutes<'a>,
    /// `overflow_iterations_` and `critical_nets_percentage_`.
    pub overflow_iterations: i32,
    pub critical_nets_percentage: f32,
    pub layer_dir: &'a [LayerDir],
    pub resistance_aware: bool,
    pub liberty: bool,
    /// The timer's slack per net id (`getNetSlack`, `sta::INF` = `1E+30F` when unconstrained), read
    /// by the loop's partial-slack pass.
    pub timer_slack: crate::congestion_loop::TimerSlack<'a>,
    /// For R20's database units.
    pub origin: GridOrigin,
    /// Each net's database id, indexed by net id (R20's key).
    pub db_id: &'a [u32],
}

/// One boundary of the run, as the observer sees it.
pub enum Stage<'a> {
    /// After R5 (`gen_brk_RSMT(false, …)`).
    R5(&'a BrkSummary),
    /// After R6 (`routeLAll(true)`) — R7's entry.
    R6,
    /// After R7 (`gen_brk_RSMT(true, true, true, …)`).
    R7(&'a BrkSummary),
    /// A boundary with a 2D overflow scan: B7 (after R7 + `getOverflow2D`), B8, B9 (no new scan —
    /// R8's), B10, B11, B13, B15.
    Scan { tag: &'static str, scan: &'a Overflow2DScan },
    /// One monotonic round of R12 (B12_k). Cannot stop the run.
    Lv { k: usize, round: &'a LvRound },
    /// A congestion-loop boundary (B14pre / B14 / B14b / B14c). Cannot stop the run.
    Loop(&'a LoopEvent),
    /// The loop's end.
    LoopEnd(&'a LoopEnd),
    /// After R16 + `getOverflow3D`.
    B16 { past_cong: i32, overflow: &'a Overflow3D },
    /// After one R18 pass: `B18a` / `B18b`, its `enlarge_` and rip-up bound.
    B18 { tag: &'static str, expand: i32, ripup_ub: i32 },
    /// After R19.
    B19(&'a Finish3d),
}

/// Watches the run. `false` from a stoppable stage ends the run there.
pub trait RunObserver {
    fn stage(&mut self, s: Stage<'_>, g2d: &Graph2d, g3: Option<&Graph3d>, state: &[NetState]) -> bool;
}

/// How a run ended.
#[derive(Debug)]
pub enum RunEnd {
    /// The observer stopped it.
    Stopped,
    /// R20's routes, keyed by database id.
    Routed(BTreeMap<u32, Vec<GSegment>>),
}

/// `FastRouteCore::run()`. `state` is indexed by net id and starts at its default.
///
/// ⛔ Refused (`Err`), where the reference goes on: `check2DEdgesUsage` finding violations (the
/// reference raises GRT-0228/0229 and stops too — returned so the caller can tell), the congestion
/// loop's partial slack, a resistance-aware net layer assignment would price, and R17.
pub fn fastroute_run(inp: &RunInputs<'_>, state: &mut [NetState], obs: &mut dyn RunObserver) -> Result<RunEnd, String> {
    let (xg, yg) = (inp.x_grid, inp.y_grid);
    let (ids, nets) = (inp.net_ids, inp.nets);
    let max_layer = ids.iter().map(|&id| nets[id].max_layer).max().unwrap_or(0);
    let num_layers = inp.caps.layers.len().max(max_layer + 1);
    let mut g2d = graph_2d(inp, num_layers);
    for &id in ids {
        state[id].seglist.clear();
        (state[id].slack, state[id].critical) = inp.slack[id];
    }
    let (rh, rv) = (inp.red_h, inp.red_v);
    let red_h = move |x: usize, y: usize| rh[y * xg + x];
    let red_v = move |x: usize, y: usize| rv[y * xg + x];
    macro_rules! grid {
        () => {
            BrkGrid { g: &mut g2d, red_h: &red_h, red_v: &red_v, caps: inp.caps, h_capacity: inp.h_capacity, v_capacity: inp.v_capacity, via_cost: 0.0 }
        };
    }
    macro_rules! stop_unless {
        ($s:expr, $g3:expr) => {
            if !obs.stage($s, &g2d, $g3, &*state) {
                return Ok(RunEnd::Stopped);
            }
        };
    }
    let no_adj = false;
    // R5
    let sum = gen_brk_rsmt(BrkFlags { congestion_driven: false, re_route: false, gen_tree: false, no_adj }, ids, nets, state, &mut grid!(), inp.stt, inp.flutes)
        .map_err(|e| format!("R5: {e:?}"))?;
    stop_unless!(Stage::R5(&sum), None);
    // R6
    route_l_all(ids, nets, state, &mut grid!());
    stop_unless!(Stage::R6, None);
    // R7
    let sum = gen_brk_rsmt(BrkFlags { congestion_driven: true, re_route: true, gen_tree: true, no_adj }, ids, nets, state, &mut grid!(), inp.stt, inp.flutes)
        .map_err(|e| format!("R7: {e:?}"))?;
    stop_unless!(Stage::R7(&sum), None);
    let scan = g2d.get_overflow_2d();
    stop_unless!(Stage::Scan { tag: "B7", scan: &scan }, None);
    // R8
    newroute_l_all(false, true, ids, nets, state, &mut grid!());
    let scan = g2d.get_overflow_2d();
    stop_unless!(Stage::Scan { tag: "B8", scan: &scan }, None);
    // R9 — no overflow scan follows it.
    let pin_layer = |id: usize, pin: usize| inp.attrs[id].pin_layers.get(pin).copied().unwrap_or(0);
    spiral_route_all(ids, nets, state, &mut grid!(), num_layers as i16, &pin_layer);
    stop_unless!(Stage::Scan { tag: "B9", scan: &scan }, None);
    // R10
    newroute_z_all(10, ids, nets, state, &mut grid!());
    let scan = g2d.get_overflow_2d();
    stop_unless!(Stage::Scan { tag: "B10", scan: &scan }, None);
    // R11
    let viol = convert_to_mazeroute_all(ids, state, &mut g2d, inp.h_capacity, inp.v_capacity);
    if !viol.is_empty() {
        return Err(format!("check2DEdgesUsage (GRT-0228/0229): {viol:?}"));
    }
    stop_unless!(Stage::Scan { tag: "B11", scan: &scan }, None);
    // R12
    let pattern_scan = scan;
    let (mut last, mut last_lc) = (scan, 0.0f32);
    lv_rounds(scan.max_overflow, ids, nets, state, &mut grid!(), &mut |k, round, g, st| {
        let _ = obs.stage(Stage::Lv { k, round }, g, None, st);
        (last, last_lc) = (round.scan, round.logistic_coef);
    });
    // R13
    init_for_congestion_loop(ids, state, &mut g2d);
    stop_unless!(Stage::Scan { tag: "B13", scan: &last }, None);
    // R14
    let start = LoopStart {
        pattern_max_overflow: pattern_scan.max_overflow,
        logistic_coef: last_lc,
        scan: last,
        overflow_iterations: inp.overflow_iterations,
        critical_nets_percentage: inp.critical_nets_percentage,
    };
    let end = congestion_loop(&start, ids, nets, state, &mut grid!(), inp.timer_slack, &mut |ev, g, st| {
        let _ = obs.stage(Stage::Loop(ev), g, None, st);
    })?;
    stop_unless!(Stage::LoopEnd(&end), None);
    // R15 — freeRR drops the loop's own backup; nothing is left here to free.
    remove_loops_all(ids, nets, state, &mut g2d);
    let scan = g2d.get_overflow_2d_maze();
    stop_unless!(Stage::Scan { tag: "B15", scan: &scan }, None);
    // R16
    let mut g3 = graph_3d(inp.caps, xg, yg);
    let layer = LayerParams { layer_dir: inp.layer_dir, resistance_aware: inp.resistance_aware, liberty: inp.liberty, has_2d_overflow: end.has_2d_overflow };
    let mut order = layer_assignment(ids, nets, inp.attrs, state, &mut g3, &layer)?;
    let overflow = get_overflow_3d_all(&g2d, &g3);
    let past_cong = scan.total_overflow;
    stop_unless!(Stage::B16 { past_cong, overflow: &overflow }, Some(&g3));
    // R17
    if past_cong != overflow.total {
        return Err("R17: 2D and 3D overflow differ — disableNDRForCongestedNets is not wired".into());
    }
    // R18 — costheight_ 3, via_cost_ 1; run()'s local enlarge_ as the loop left it.
    if past_cong == 0 {
        let (long, short) = if inp.resistance_aware { (BIG_INT, BIG_INT) } else { (40, 12) };
        for (tag, ub) in [("B18a", long), ("B18b", short)] {
            let mp = Maze3dParams { layer: &layer, expand: end.enlarge, ripup_lb: 0, ripup_ub: ub, via_cost: 1 };
            order = maze_route_msmd_order_3d_all(&order, nets, inp.attrs, state, &mut g2d, &mut g3, &mp)?.0;
            stop_unless!(Stage::B18 { tag, expand: end.enlarge, ripup_ub: ub }, Some(&g3));
        }
    }
    // R19
    let fin = finish_3d(ids, nets, inp.attrs, state, &g2d, &g3)?;
    stop_unless!(Stage::B19(&fin), Some(&g3));
    // R20
    Ok(RunEnd::Routed(get_routes_all(ids, state, inp.db_id, inp.origin)))
}

/// The 2D graph at entry: the estimated usage, the capacities, and the NDR ledger's per-layer
/// capacities as `initEdgesCapacityPerLayer` leaves them (horizontal edges to `x < xg-1`, vertical
/// to `y < yg-1`, no NDR net anywhere).
fn graph_2d(inp: &RunInputs<'_>, num_layers: usize) -> Graph2d {
    let (xg, yg) = (inp.x_grid, inp.y_grid);
    let mut ledger = NdrLedger::new(xg, yg, num_layers);
    for (l, cl) in inp.caps.layers.iter().enumerate() {
        for y in 0..yg {
            for x in 0..xg {
                if x + 1 < xg {
                    ledger.update_cap_3d(x, y, l, true, f64::from(cl.h[y * xg + x]));
                }
                if y + 1 < yg {
                    ledger.update_cap_3d(x, y, l, false, f64::from(cl.v[y * xg + x]));
                }
            }
        }
    }
    let mut g = Graph2d::new(xg, yg, 1);
    g.est = inp.entry.clone();
    g.ndr = ledger;
    g.cap_h = inp.cap_h.to_vec();
    g.cap_v = inp.cap_v.to_vec();
    g
}

/// The 3D edges at layer assignment: the setup's capacities, no usage.
fn graph_3d(caps: &Caps3D, xg: usize, yg: usize) -> Graph3d {
    let layer = |v: &[i32]| -> Vec<u16> { v.iter().map(|&c| c as u16).collect() };
    Graph3d {
        x_grid: xg,
        num_layers: caps.layers.len(),
        h_cap: caps.layers.iter().map(|l| layer(&l.h)).collect(),
        v_cap: caps.layers.iter().map(|l| layer(&l.v)).collect(),
        h_usage: vec![vec![0; xg * yg]; caps.layers.len()],
        v_usage: vec![vec![0; xg * yg]; caps.layers.len()],
    }
}
