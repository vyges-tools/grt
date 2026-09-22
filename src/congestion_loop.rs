// SPDX-License-Identifier: Apache-2.0
//! Stage R14 — run()'s congestion loop: the schedule around [`maze_route_msmd_sequential`].
//!
//! ⛔ The constants ARE the algorithm (flowchart §R14): every threshold, step and switch below is a
//! transcription of run(), in its order. The loop reports each maze pass it makes — the parameters
//! it computed and the state after — so a replay can check the schedule itself.
//!
//! ⚠️ Not transcribed, refused rather than guessed: the snapshot-batched convergence (the sequential
//! kernel keeps `snapshot_cleanup_active_` false), and soft-NDR demotion on a design with NDR nets
//! (reported as an error when reached).

use crate::brk_rsmt::{BrkGrid, NetState, RouteKind, RsmtNet, StTree};
use crate::estimate::Usage2d;
use crate::maze_msmd::{maze_route_msmd_sequential, MsmdParams};
use crate::mazecost::CostParams;
use crate::overflow2d::Overflow2DScan;

/// The reference's `BIG_INT`.
const BIG_INT: i32 = 1_000_000_000;

/// What run() carries into the loop from the pattern and monotonic phases.
#[derive(Debug, Clone, Copy)]
pub struct LoopStart {
    /// `maxOverflow` after R10 — the `> 700` switch in the LV prologue reads it.
    pub pattern_max_overflow: i32,
    /// The last LV round: its logistic coefficient and its maze-overflow scan.
    pub logistic_coef: f32,
    pub scan: Overflow2DScan,
    /// `overflow_iterations_` (`-congestion_iterations`, 50 by default).
    pub overflow_iterations: i32,
    /// `critical_nets_percentage_`.
    pub critical_nets_percentage: i32,
}

/// The schedule's values at one maze pass, beside the parameters it passed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopSchedule {
    pub up_type: i32,
    pub stop_dec: bool,
    pub thresh_m: i32,
    pub cost_step: i32,
    pub max_adj: i32,
    /// `updateCongestionHistory`'s inputs this iteration: `(up_type, ahth, stop_dec)`.
    pub history_args: (i32, i32, bool),
}

/// Which maze pass an event reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PassKind {
    /// The iteration's own pass.
    Main,
    /// The extra pass at `i == 20` (`maxOverflow < 150`, `past_cong > 200`).
    Extra20,
    /// The extra pass after `copyRS` (`i > 140`, or `i > 80` with `past_cong < 20`).
    ExtraCopyRs,
}

/// One maze pass as the loop makes it.
#[derive(Debug, Clone, Copy)]
pub enum LoopEvent {
    /// Before the iteration's own pass: the parameters, and the schedule that produced them.
    Before { params: MsmdParams, schedule: LoopSchedule },
    /// After any pass and its `getOverflow2Dmaze`.
    After { kind: PassKind, iter: i32, scan: Overflow2DScan },
}

/// What the loop leaves for the finalisation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopEnd {
    pub iterations: i32,
    pub scan: Overflow2DScan,
    pub minofl: i32,
    pub minoflrnd: i32,
    pub has_2d_overflow: bool,
    pub enlarge: i32,
    /// How often each branch no corpus loop reaches actually fired — so a replay can ASSERT the
    /// limitation instead of noting it.
    pub rare: RareBranches,
}

/// The loop's branches the corpus never exercises.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RareBranches {
    /// `total_overflow > 15000 && maxOverflow > 400`.
    pub escape_hatch: u32,
    /// Iterations that START in `upType` 4 or with `stopDEC` set — the late / low-congestion modes.
    /// ⚠️ Counted where they are CONSUMED: a 50-iteration loop sets `upType = 4` after its last pass
    /// (`i` reaches 51), which nothing reads.
    pub up_type4: u32,
    /// `copyRS` (`i > 140`, or `i > 80` with `past_cong < 20`).
    pub copy_rs: u32,
    /// `str_accu(25)` / `str_accu(40)` at `i == 35` / `50` on a heavily used grid.
    pub late_accu: u32,
    /// The `mazeRound` (500) cap.
    pub maze_round: u32,
}

/// run()'s congestion loop, from `i = 1` to its break, then `copyBR` when any round beat nothing.
pub fn congestion_loop(
    start: &LoopStart,
    net_ids: &[usize],
    nets: &[RsmtNet<'_>],
    state: &mut [NetState],
    grid: &mut BrkGrid<'_>,
    on: &mut dyn FnMut(&LoopEvent, &crate::graph2d::Graph2d, &[NetState]),
) -> Result<LoopEnd, String> {
    const ENLARGE: i32 = 15;
    const ESTEP1: i32 = 10;
    const ESTEP2: i32 = 5;
    const ESTEP3: i32 = 5;
    const CSTEP2: i32 = 2;
    const CSTEP3: i32 = 5;
    const COSHEIGHT: i32 = 4;
    const RIPVALUE: i32 = -1;
    const TH_STEP1: i32 = 10;
    const TH_STEP2: i32 = 4;
    const MAZE_ROUND: i32 = 500;
    const MAX_OVERFLOW_INCREASES: i32 = 25;
    const SOFT_NDR_STAGNANT_TH: i32 = 10;
    const SOFT_NDR_MAX_ITER: i32 = 15;

    // The LV prologue's `maxOverflow > 700` switch, carried into the loop.
    let big = start.pattern_max_overflow > 700;
    let mut thresh_m: i32 = if big { 0 } else { 20 };
    let cstep1: i32 = if big { 30 } else { 2 };
    let mut via: i32 = if big { 0 } else { 2 };
    let mut logistic_coef = start.logistic_coef;

    let mut scan = start.scan;
    let (mut max_overflow, mut past_cong, mut total_overflow) = (scan.max_overflow, scan.total_overflow, scan.total_overflow);
    let mut i = 1;
    let mut costheight = COSHEIGHT;
    let mut enlarge = ENLARGE;
    let mut ripup_threshold = RIPVALUE;
    let mut minofl = total_overflow;
    let mut minoflrnd = 0;
    let mut stop_dec = false;
    let mut slope = 20;
    let mut l = 1;
    let mut up_type = 1;
    let mut max_adj = 0;
    let (mut bmfl, mut bwcnt) = (BIG_INT, 0);
    let slack_th = f32::MIN;
    let mut overflow_increases = -1;
    let mut last_total_overflow = 0;
    let mut minofl_stagnant = 0;
    let mut backup: Option<Vec<Option<StTree>>> = None;
    let (xg, yg) = (grid.g.est.x_grids as i32, grid.g.est.y_grids as i32);
    let mut rare = RareBranches::default();

    // ⛔ The pass computes an `enlarge_` per edge, but into the router's MEMBER; run()'s loop uses a
    // LOCAL `int enlarge_` that shadows it, so the pass never changes the schedule's value.
    let pass = |grid: &mut BrkGrid<'_>, state: &mut [NetState], p: &MsmdParams| -> Result<(), String> {
        maze_route_msmd_sequential(p, net_ids, nets, state, grid).map(|_| ())
    };

    while total_overflow > 0 && i <= start.overflow_iterations && overflow_increases <= MAX_OVERFLOW_INCREASES {
        if up_type == 4 || stop_dec {
            rare.up_type4 += 1;
        }
        thresh_m = if thresh_m > 15 { thresh_m - TH_STEP1 } else if thresh_m >= 2 { thresh_m - TH_STEP2 } else { 0 };
        thresh_m = thresh_m.max(0);

        let cost_step;
        if total_overflow > 2000 {
            enlarge += ESTEP1;
            cost_step = cstep1;
        } else if total_overflow < 500 {
            cost_step = CSTEP3;
            enlarge += ESTEP3;
            ripup_threshold = -1;
        } else {
            cost_step = CSTEP2;
            enlarge += ESTEP2;
        }
        let history_args = (up_type, scan.ahth, stop_dec);
        max_adj = grid.g.update_congestion_history(up_type, scan.ahth, stop_dec, max_adj);

        if total_overflow > 15000 && max_overflow > 400 {
            rare.escape_hatch += 1;
            enlarge = xg.max(yg) / 30;
            slope = BIG_INT;
            if i == 5 {
                via = 0;
                logistic_coef = 1.33;
                ripup_threshold = -1;
            } else if i > 6 && i % 2 == 0 {
                logistic_coef += 0.5;
            }
            if i > 10 {
                ripup_threshold = 0;
            }
        }

        enlarge = enlarge.min(xg / 2);
        costheight += cost_step;
        let maze_edge_threshold = thresh_m;
        // ⛔ `std::max<float>(2.0 / (1 + log(maxOverflow)), logistic_coef)`: the double narrowed to
        // float first; `log(0)` is -inf, so an empty round contributes -0.0. (`upType == 3` never
        // occurs in run(); its arm is not transcribed.)
        logistic_coef = ((2.0 / (1.0 + f64::from(max_overflow).ln())) as f32).max(logistic_coef);

        if i == 8 {
            l = 0;
            up_type = 2;
            grid.g.est.init_last_usage(up_type);
        }
        if max_overflow == 1 {
            ripup_threshold = -1;
            slope = 5;
        }
        if max_overflow > 300 && past_cong > 15000 {
            l = 0;
        }

        let cnp = start.critical_nets_percentage;
        let params = |i: i32, enlarge: i32, ripup_threshold: i32, via: i32, l: i32, logistic_coef: f32, costheight: i32, slope: i32| MsmdParams {
            iter: i,
            expand: enlarge,
            ripup_threshold,
            maze_edge_threshold,
            ordering: i % 3 == 0,
            via,
            l,
            cost: CostParams { slope, logistic_coef: f64::from(logistic_coef), cost_height: f64::from(costheight) },
            slack_th,
            critical_nets_percentage: cnp,
        };
        let p = params(i, enlarge, ripup_threshold, via, l, logistic_coef, costheight, slope);
        on(&LoopEvent::Before { params: p, schedule: LoopSchedule { up_type, stop_dec, thresh_m, cost_step, max_adj, history_args } }, grid.g, state);
        pass(grid, state, &p)?;
        let mut last_cong = past_cong;
        scan = grid.g.get_overflow_2d_maze();
        (past_cong, max_overflow, total_overflow) = (scan.total_overflow, scan.max_overflow, scan.total_overflow);
        on(&LoopEvent::After { kind: PassKind::Main, iter: i, scan }, grid.g, state);

        if minofl > past_cong {
            minofl = past_cong;
            minoflrnd = i;
            minofl_stagnant = 0;
        } else {
            minofl_stagnant += 1;
        }
        if i == 8 {
            l = 1;
        }
        i += 1;

        if past_cong < 200 && i > 30 && up_type == 2 && max_adj <= 20 {
            up_type = 4;
            stop_dec = true;
        }

        if max_overflow < 150 {
            if i == 20 && past_cong > 200 {
                l = 0;
                slope = 5;
                let p = params(i, enlarge, ripup_threshold, via, l, logistic_coef, costheight, slope);
                pass(grid, state, &p)?;
                last_cong = past_cong;
                scan = grid.g.get_overflow_2d_maze();
                (past_cong, max_overflow, total_overflow) = (scan.total_overflow, scan.max_overflow, scan.total_overflow);
                on(&LoopEvent::After { kind: PassKind::Extra20, iter: i, scan }, grid.g, state);
                grid.g.str_accu(12);
                l = 1;
                stop_dec = false;
                slope = 3;
                up_type = 2;
            }
            if i == 35 && scan.total_usage > 800_000 {
                rare.late_accu += 1;
                grid.g.str_accu(25);
            }
            if i == 50 && scan.total_usage > 800_000 {
                rare.late_accu += 1;
                grid.g.str_accu(40);
            }
        }

        if i > 50 {
            up_type = 4;
            if i > 70 {
                stop_dec = true;
            }
        }
        if f64::from(past_cong) > 0.7 * f64::from(last_cong) {
            costheight += CSTEP3;
        }
        if past_cong >= last_cong {
            via = 0;
        }

        // `checkSnapshotConvergence` is false on the sequential kernel; the cleanup branch runs.
        if past_cong < bmfl {
            bwcnt = 0;
            if i > 140 || (i > 80 && past_cong < 20) {
                rare.copy_rs += 1;
                backup = Some(copy_rs(net_ids, state));
                bmfl = past_cong;
                l = 0;
                let p = params(i, enlarge, ripup_threshold, via, l, logistic_coef, costheight, slope);
                pass(grid, state, &p)?;
                last_cong = past_cong;
                scan = grid.g.get_overflow_2d_maze();
                (past_cong, max_overflow, total_overflow) = (scan.total_overflow, scan.max_overflow, scan.total_overflow);
                on(&LoopEvent::After { kind: PassKind::ExtraCopyRs, iter: i, scan }, grid.g, state);
                if past_cong < last_cong {
                    backup = Some(copy_rs(net_ids, state));
                    bmfl = past_cong;
                }
                l = 1;
                if minofl > past_cong {
                    minofl = past_cong;
                    minoflrnd = i;
                    minofl_stagnant = 0;
                }
            }
        } else {
            bwcnt += 1;
        }
        if bmfl > 10 && ((bmfl > 30 && bmfl < 72 && bwcnt > 50) || (bmfl < 30 && bwcnt > 50)) {
            break;
        }
        if i >= MAZE_ROUND {
            rare.maze_round += 1;
            scan = grid.g.get_overflow_2d_maze();
            total_overflow = scan.total_overflow;
            break;
        }
        if total_overflow > last_total_overflow {
            overflow_increases += 1;
        }
        last_total_overflow = total_overflow;

        // Soft-NDR demotion: only NDR nets can be demoted, so a design without them computes an
        // empty list and nothing happens.
        if total_overflow > 0 && (minofl_stagnant > SOFT_NDR_STAGNANT_TH || i > SOFT_NDR_MAX_ITER) {
            let has_ndr = net_ids.iter().any(|&id| nets[id].edge_cost > 1);
            if has_ndr {
                return Err("soft-NDR demotion on a design with NDR nets: not wired into the loop".into());
            }
        }
    }

    let has_2d_overflow = total_overflow > 0;
    if minofl > 0 {
        copy_br(net_ids, nets, state, grid, backup.as_deref());
    }
    Ok(LoopEnd { iterations: i - 1, scan, minofl, minoflrnd, has_2d_overflow, enlarge, rare })
}

/// `copyRS` — save every net's tree (topology and routes) as the best so far.
pub fn copy_rs(net_ids: &[usize], state: &[NetState]) -> Vec<Option<StTree>> {
    let mut bk = vec![None; state.len()];
    for &id in net_ids {
        bk[id] = state[id].tree.clone();
    }
    bk
}

/// `copyBR` — restore the saved trees, moving the committed usage with them: the current routes are
/// given back, the saved ones charged. ⚠️ A no-op when nothing was saved.
pub fn copy_br(net_ids: &[usize], nets: &[RsmtNet<'_>], state: &mut [NetState], grid: &mut BrkGrid<'_>, backup: Option<&[Option<StTree>]>) {
    let Some(bk) = backup else { return };
    let charge = |grid: &mut BrkGrid<'_>, id: usize, t: &StTree, sign: f64| {
        let nn = nets[id].ndr_net(id);
        let mut g = grid.g.for_net(&nn);
        for (e, r) in t.edges.iter().zip(&t.routes) {
            if e.len <= 0 {
                continue;
            }
            for k in 0..r.routelen.max(0) as usize {
                let ((ax, ay), (bx, by)) = (r.grids[k], r.grids[k + 1]);
                if (ax, ay) == (bx, by) {
                    continue;
                }
                if ax == bx {
                    g.update_usage_v(ax, ay.min(by), sign * f64::from(nn.edge_cost));
                } else {
                    g.update_usage_h(ax.min(bx), ay, sign * f64::from(nn.edge_cost));
                }
            }
        }
    };
    for &id in net_ids {
        if let Some(t) = state[id].tree.clone() {
            charge(grid, id, &t, -1.0);
        }
    }
    for &id in net_ids {
        if let Some(saved) = bk.get(id).cloned().flatten() {
            let mut t = saved;
            for r in t.routes.iter_mut() {
                r.kind = RouteKind::MazeRoute;
            }
            state[id].tree = Some(t);
        }
    }
    for &id in net_ids {
        if let Some(t) = state[id].tree.clone() {
            charge(grid, id, &t, 1.0);
        }
    }
}
