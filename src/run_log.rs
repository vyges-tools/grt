// SPDX-License-Identifier: Apache-2.0
//! `run()`'s own log lines, read off its stage events: the congestion loop's (GRT-101 once, then
//! GRT-102 per iteration), the via filling's (GRT-197 … GRT-199) and `reportRunMetrics`'
//! (GRT-111 / GRT-112). Verbose only. They are
//! collected while the run goes and written after it, in the order they were raised.

use crate::congestion_loop::LoopEvent;
use crate::log::Msg;
use crate::run::Stage;

pub struct RunLines {
    verbose: bool,
    overflow_iterations: i32,
    started: bool,
    lines: Vec<Msg>,
}

impl RunLines {
    pub fn new(verbose: bool, overflow_iterations: i32) -> RunLines {
        RunLines { verbose, overflow_iterations, started: false, lines: Vec::new() }
    }

    fn info(&mut self, tool: &'static str, id: u16, text: String) {
        self.lines.push(Msg::Coded { level: crate::log::Level::Info, tool, id, text });
    }

    /// One stage event.
    ///
    /// Rules:
    /// - GRT-101 is raised before the loop when the overflow is positive and iterations are
    ///   allowed — exactly when the loop runs its first iteration, so it is written then, once
    ///   (a soft-NDR restart does not raise it again);
    /// - GRT-102 opens each iteration with its number, which a soft-NDR restart sets back to 1;
    /// - fillVIA's counters, then its closing line.
    pub fn stage(&mut self, s: &Stage<'_>) {
        if !self.verbose {
            return;
        }
        match s {
            Stage::Loop(LoopEvent::Before { params, .. }) => {
                if !self.started {
                    self.started = true;
                    self.info("GRT", 101, "Running extra iterations to remove congestion.".into());
                }
                self.info("GRT", 102, format!("Start extra iteration {}/{}", params.iter, self.overflow_iterations));
            }
            Stage::B19(fin) => {
                self.info("GRT", 197, format!("Via related to pin nodes: {}", fin.vias.pin_nodes));
                self.info("GRT", 198, format!("Via related Steiner nodes: {}", fin.vias.steiner_nodes));
                self.info("GRT", 199, "Via filling finished.".into());
                // reportRunMetrics: the vias `threeDVIA` counted, and the usage `getOverflow3D`
                // returned plus three per via.
                self.info("GRT", 111, format!("Final number of vias: {}", fin.num_via));
                self.info("GRT", 112, format!("Final usage 3D: {}", fin.overflow.total_usage + 3 * fin.num_via));
            }
            _ => {}
        }
    }

    pub fn write(&mut self, log: &mut dyn crate::log::Sink) {
        for m in self.lines.drain(..) {
            log.put(m);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::congestion_loop::LoopSchedule;
    use crate::fillvia::ViaCounts;
    use crate::finalize::Finish3d;
    use crate::maze_msmd::MsmdParams;
    use crate::mazecost::CostParams;

    fn before(iter: i32) -> LoopEvent {
        let params = MsmdParams { iter, expand: 0, ripup_threshold: 0, maze_edge_threshold: 0, ordering: false, via: 0, l: 0, cost: CostParams { slope: 0, logistic_coef: 0.0, cost_height: 0.0 }, slack_th: 0.0, critical_nets_percentage: 0.0 };
        LoopEvent::Before { params, schedule: LoopSchedule { up_type: 1, stop_dec: false, thresh_m: 0, cost_step: 0, max_adj: 0, history_args: (0, 0, false) } }
    }

    fn texts(r: &mut RunLines) -> Vec<String> {
        let mut v: Vec<Msg> = Vec::new();
        r.write(&mut v);
        v.into_iter().map(|m| match m { Msg::Coded { id, text, .. } => format!("{id} {text}"), other => format!("{other:?}") }).collect()
    }

    /// Rules (run, fillVIA): GRT-101 once, before the first iteration — not again when a soft-NDR
    /// restart sets the count back to 1; each iteration's number over the limit; the via counters.
    /// Silent unless verbose.
    #[test]
    fn run_lines_in_order() {
        let fin = Finish3d { vias: ViaCounts { pin_nodes: 7, steiner_nodes: 3 }, overflow: Default::default(), num_via: 0, defects: vec![], pin_stacks: 0 };
        let mut r = RunLines::new(true, 50);
        let evs = [before(1), before(2), before(1)];
        for e in &evs {
            r.stage(&Stage::Loop(e));
        }
        r.stage(&Stage::B19(&fin));
        assert_eq!(texts(&mut r), ["101 Running extra iterations to remove congestion.", "102 Start extra iteration 1/50", "102 Start extra iteration 2/50", "102 Start extra iteration 1/50", "197 Via related to pin nodes: 7", "198 Via related Steiner nodes: 3", "199 Via filling finished.", "111 Final number of vias: 0", "112 Final usage 3D: 0"]);
        let mut q = RunLines::new(false, 50);
        q.stage(&Stage::Loop(&evs[0]));
        assert!(texts(&mut q).is_empty());
    }
}
