// SPDX-License-Identifier: Apache-2.0
//! The second global router, `global_route -use_cugr`.
//!
//! Reimplemented from the behaviour of CUGR 2.0 (the Chinese University of Hong Kong; the EDGE
//! global router) as the OpenROAD project's global router embeds it — no source is carried over.
//!
//! Built one stage at a time, earliest first. What exists:
//!
//! | stage | reference | here |
//! | --- | --- | --- |
//! | 0 · the model | `Design`, `GridGraph`, `GRNet` construction | [`init`] |
//! | 1 · pattern routing | `CUGR::patternRoute` | [`Cugr::pattern_route`] |
//! | 3 · detours | `CUGR::patternRouteWithDetours` | [`Cugr::pattern_route_with_detours`] |
//! | 4 · maze | `CUGR::mazeRoute` | [`Cugr::maze_route`] |
//! | 5 · rip-up and re-route | `CUGR::iterativeRRR` | [`Cugr::iterative_rrr`] |
//!
//! [`init`] is `CUGR::init`'s call sequence and does no work of its own; so is each stage.

pub mod design;
pub mod geo;
pub mod grid_graph;
pub mod grnet;
pub mod jumpers;
pub mod layers;
pub mod maze_route;
pub mod pattern_route;
pub mod restore;
pub mod trace;
#[cfg(feature = "odb")]
pub mod read;
#[cfg(feature = "odb")]
pub mod route;

use design::{Design, DesignError, DesignFacts};
use grid_graph::{GridError, GridGraph};
use grnet::GrNet;

/// CUGR's tuning constants (`Constants`), at the reference's defaults.
#[derive(Debug, Clone, PartialEq)]
pub struct Constants {
    pub weight_wire_length: f64,
    pub weight_via_number: f64,
    pub weight_short_area: f64,
    /// 0-based; `init` sets it to the min routing layer − 1.
    pub min_routing_layer: i32,
    pub cost_logistic_slope: f64,
    pub max_detour_ratio: f64,
    pub target_detour_count: i32,
    pub via_multiplier: f64,
    pub maze_logistic_slope: f64,
    pub resistance_min_net_length: i32,
    pub resistance_weight: f64,
    pub congestion_gate_penalty: f64,
}

impl Default for Constants {
    fn default() -> Self {
        Constants {
            weight_wire_length: 0.5,
            weight_via_number: 4.0,
            weight_short_area: 500.0,
            min_routing_layer: 1,
            cost_logistic_slope: 1.0,
            max_detour_ratio: 0.25,
            target_detour_count: 20,
            via_multiplier: 2.0,
            maze_logistic_slope: 0.5,
            resistance_min_net_length: 3,
            resistance_weight: 50.0,
            congestion_gate_penalty: 4.0,
        }
    }
}

/// One layer rule of a net's non-default rule, as `computeNdrCosts` reads it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NdrRuleFacts {
    /// The rule's layer: whether it is a ROUTING layer, and its routing level.
    pub is_routing: bool,
    pub routing_level: i32,
    /// `dbTechLayerRule::getWidth` / `getSpacing`.
    pub width: i32,
    pub spacing: i32,
    /// The tech layer's own `getWidth()` and `getPitch()`.
    pub default_width: i32,
    pub default_pitch: i32,
}

/// `computeNdrCosts`: per layer, how many tracks one wire of the rule occupies — 1 everywhere a
/// rule does not reach.
///
/// Upstream rule: `(W + 2S + D) / 2P` in DOUBLE from an INTEGER numerator — the rule's width `W`
/// and spacing `S`, the layer's default width `D` and pitch `P` (the tech layer's, not the
/// tracks') — floored at 1. A rule on a non-routing layer, a layer out of range, or a pitch of 0
/// is skipped.
pub fn ndr_costs(num_layers: usize, rules: &[NdrRuleFacts]) -> Vec<f64> {
    let mut factors = vec![1.0; num_layers];
    for r in rules {
        if !r.is_routing {
            continue;
        }
        let layer = r.routing_level - 1;
        if layer < 0 || layer as usize >= num_layers || r.default_pitch <= 0 {
            continue;
        }
        let f = f64::from(r.width + 2 * r.spacing + r.default_width) / f64::from(2 * r.default_pitch);
        factors[layer as usize] = f.max(1.0);
    }
    factors
}

/// The router after `init`: the design, the graph, and the nets it will route.
#[derive(Debug, Clone, PartialEq)]
pub struct Cugr {
    pub constants: Constants,
    pub design: Design,
    pub grid: GridGraph,
    /// Indexed by net index (every design net is valid at `init`).
    pub nets: Vec<GrNet>,
    /// `GridGraph::cost_multiplier_`: RRR's slope multiplier on the logistic costs (1 otherwise).
    pub cost_multiplier: f64,
    /// Captured slacks per net-order sort, where the timer's are needed; the next sort's index.
    pub sort_slacks: Option<Vec<std::collections::BTreeMap<String, f32>>>,
    pub sorts_done: usize,
    /// `incremental_candidates_`: during an incremental route, the nets it re-routes (its
    /// congested-set scans and RRR rounds are scoped to them).
    pub incremental_candidates: Option<Vec<usize>>,
    /// `resistance_aware_`, `critical_nets_percentage_`, `res_aware_percentage_` (15 unless set).
    pub resistance_aware: bool,
    pub critical_nets_percentage: f32,
    pub res_aware_percentage: f32,
    /// The timer's slack of every net at each `updateNetSlacks` (captured), and the calls made.
    /// With them the engine refreshes, marks and demotes itself; without, sorts after the first
    /// take the per-sort capture.
    pub raw_slacks: Option<Vec<std::collections::BTreeMap<String, f32>>>,
    pub raw_done: usize,
    /// Without a clock the timer answers one slack for every net (unconstrained, `1e+30`): each
    /// refresh then sets it — no capture needed.
    pub constant_slack: Option<f32>,
    /// `worst_slack_`, `worst_resistance_`, `worst_fanout_`, `worst_net_length_`: the res-aware
    /// score's normalisers, as the last marking left them.
    pub worst: (f32, f32, i32, i32),
    /// The timer itself, asked at each `updateNetSlacks` on the routes as they stand (instead of a
    /// capture).
    pub slack_source: SlackSource,
}

/// Every net's slack, by name, from the timer at one `updateNetSlacks`.
pub type SlackFn = dyn FnMut(&Cugr) -> Result<std::collections::BTreeMap<String, f32>, String>;

/// The timer as the router holds it: shared, and invisible to the router's own comparisons.
#[derive(Clone, Default)]
pub struct SlackSource(pub Option<std::rc::Rc<std::cell::RefCell<SlackFn>>>);

impl std::fmt::Debug for SlackSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SlackSource({})", if self.0.is_some() { "timer" } else { "none" })
    }
}

impl PartialEq for SlackSource {
    fn eq(&self, _: &Self) -> bool {
        true
    }
}

/// `kDemotedSlack`: a non-critical net's slack after demotion.
pub const DEMOTED_SLACK: f32 = f32::MAX;

/// A stage that stopped where the reference would have gone on to something not modelled, or
/// would have raised an error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StageError {
    Pattern(pattern_route::PatternError),
    Commit(grid_graph::CommitError),
    Maze(String),
    /// A captured slack the sort needs is missing — not modelled.
    Slacks(String),
}

impl Cugr {
    /// `sortNetIndices(nets, res_aware_order=false)`: a STABLE sort by `(slack, bbox half
    /// perimeter)`, from index order. Where captured slacks are set, each listed net first takes
    /// this sort's (the reference refreshes and demotes them before stages 3 and 4).
    ///
    /// With `res_aware_order` the key is the res-aware score instead (lower first), a stable sort.
    /// With the timer's raw slacks captured, only the first sort takes the per-sort capture: later
    /// sorts read the slacks the engine refreshed and demoted itself.
    pub fn sort_net_indices(&mut self, indices: &mut [usize], res_aware_order: bool, stage: i32, trace: Option<&mut Vec<String>>) -> Result<(), StageError> {
        if let Some(sorts) = &self.sort_slacks {
            // Later sorts read what the refresh left — captured raw slacks, or the timer's own.
            if (self.raw_slacks.is_none() && self.slack_source.0.is_none()) || self.sorts_done == 0 {
                let m = sorts.get(self.sorts_done).ok_or_else(|| StageError::Slacks(format!("no captured slacks for sort {}", self.sorts_done)))?;
                for &k in indices.iter() {
                    self.nets[k].slack = *m.get(&self.nets[k].name).ok_or_else(|| StageError::Slacks(format!("net {} has no captured slack in sort {}", self.nets[k].name, self.sorts_done)))?;
                }
            }
        }
        self.sorts_done += 1;
        if res_aware_order {
            let scores: Vec<f32> = (0..self.nets.len()).map(|k| self.res_aware_score(k)).collect();
            indices.sort_by(|&a, &b| scores[a].partial_cmp(&scores[b]).unwrap_or(std::cmp::Ordering::Equal));
            if let Some(t) = trace {
                for (pos, &k) in indices.iter().enumerate() {
                    t.push(format!("VYGC|rorder|{stage}|{pos}|{k}|{}|{:08x}", self.nets[k].name, scores[k].to_bits()));
                }
            }
        } else {
            self.sort_by_slack_then_hp(indices);
        }
        Ok(())
    }

    /// Stages 3 to 5 sort in the res-aware order when resistance-aware routing has critical nets.
    fn res_aware_order(&self) -> bool {
        self.resistance_aware && self.critical_nets_percentage != 0.0
    }

    /// `getResAwareScore` (lower = more critical), in f32, left to right:
    /// `slack/norm*4 − R/worstR*1 − pins/worstFanout*3 − len/worstLen*2`, `norm = max(|worst
    /// slack|, 1e-9)`.
    pub fn res_aware_score(&self, k: usize) -> f32 {
        let n = &self.nets[k];
        let (worst_slack, worst_resistance, worst_fanout, worst_length) = self.worst;
        let slack_norm = worst_slack.abs().max(1e-9f32);
        let slack_term = n.slack / slack_norm * 4.0f32;
        let resistance_term = n.resistance / worst_resistance * 1.0f32;
        let fanout_term = n.num_pins() as f32 / worst_fanout as f32 * 3.0f32;
        let length_term = n.net_length as f32 / worst_length as f32 * 2.0f32;
        slack_term - resistance_term - fanout_term - length_term
    }

    /// `updateCriticalNets`: every net's slack refreshed from the timer (`updateNetSlacks`, the
    /// next captured call), the res-aware set marked on those real slacks, then — outside an
    /// incremental route — every non-critical net demoted.
    pub fn update_critical_nets(&mut self, stage: i32, mut trace: Option<&mut Vec<String>>) -> Result<(), StageError> {
        match (&self.raw_slacks, self.constant_slack) {
            (Some(raw), _) => {
                let m = raw.get(self.raw_done).ok_or_else(|| StageError::Slacks(format!("no captured timer slacks for update {}", self.raw_done)))?;
                for n in &mut self.nets {
                    n.slack = *m.get(&n.name).ok_or_else(|| StageError::Slacks(format!("net {} has no captured timer slack in update {}", n.name, self.raw_done)))?;
                }
            }
            (None, Some(constant)) => self.nets.iter_mut().for_each(|n| n.slack = constant),
            (None, None) => {
                let Some(src) = self.slack_source.0.clone() else { return Ok(()) };
                let m = (src.borrow_mut())(self).map_err(StageError::Slacks)?;
                for n in &mut self.nets {
                    n.slack = *m.get(&n.name).ok_or_else(|| StageError::Slacks(format!("net {} is not in the timing netlist", n.name)))?;
                }
            }
        }
        self.raw_done += 1;
        self.mark_res_aware_nets(stage, trace.as_deref_mut());
        if self.incremental_candidates.is_none() {
            let th = self.critical_slack_threshold();
            if let Some(t) = trace {
                t.push(format!("VYGC|dthr|{stage}|{:08x}", th.to_bits()));
            }
            for n in &mut self.nets {
                if n.slack > th && !n.res_aware {
                    n.slack = DEMOTED_SLACK;
                }
            }
        }
        Ok(())
    }

    /// `criticalSlackThreshold`: every net's slack, stably sorted; the one at
    /// `ceil(count * percentage / 100)` (f32), clamped to the last.
    fn critical_slack_threshold(&self) -> f32 {
        let mut slacks: Vec<f32> = self.nets.iter().map(|n| n.slack).collect();
        if slacks.is_empty() {
            return 0.0;
        }
        slacks.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let index = (slacks.len() as f32 * self.critical_nets_percentage / 100.0f32).ceil() as usize;
        slacks[index.min(slacks.len() - 1)]
    }

    /// `markResAwareNets` — only with resistance-aware routing. The set only ever grows.
    ///
    /// Upstream rules: pass 1 over every net (the rerouted ones in an incremental route) sets its
    /// resistance (the tree's, f64 → f32) and length (the tree's planar length, else the bounding
    /// box's half perimeter); a net of under two pins, of length ≤ `resistance_min_net_length`,
    /// or of positive slack (a CLOCK-typed net excepted) is skipped; the rest update the
    /// normalisers (from 1, 1, 1 and `f32::MAX`), and a CLOCK or NDR net is marked outright while
    /// an unmarked one becomes a candidate. Pass 2 ranks the candidates by `(score, index)` and
    /// marks the first `ceil(count * res_aware_percentage / 100)` (f32) — every one in an
    /// incremental route.
    fn mark_res_aware_nets(&mut self, stage: i32, mut trace: Option<&mut Vec<String>>) {
        if !self.resistance_aware {
            return;
        }
        let (mut worst_slack, mut worst_resistance, mut worst_fanout, mut worst_length) = (f32::MAX, 1.0f32, 1i32, 1i32);
        let scope: Vec<usize> = match &self.incremental_candidates {
            Some(c) => c.clone(),
            None => (0..self.nets.len()).collect(),
        };
        let mut candidates = Vec::new();
        for k in scope {
            let resistance = self.grid.net_resistance(&self.design, self.nets[k].routing_tree.as_ref(), &self.nets[k].ndr_widths) as f32;
            let length = if self.nets[k].routing_tree.is_some() { GridGraph::tree_length(self.nets[k].routing_tree.as_ref()) } else { self.nets[k].bounding_box.hp() };
            let min_length = self.constants.resistance_min_net_length;
            let n = &mut self.nets[k];
            n.resistance = resistance;
            n.net_length = length;
            let is_positive_slack = n.slack > 0.0 && !n.is_clock_sig;
            let is_short = n.net_length <= min_length;
            let skip = n.num_pins() < 2 || is_short || is_positive_slack;
            if let Some(t) = trace.as_deref_mut() {
                t.push(format!("VYGC|ranet|{stage}|{}|{:08x}|{}|{:08x}|{}|{}", n.name, n.resistance.to_bits(), n.net_length, n.slack.to_bits(), n.num_pins(), i32::from(skip)));
            }
            if skip {
                continue;
            }
            worst_resistance = worst_resistance.max(n.resistance);
            worst_fanout = worst_fanout.max(n.num_pins() as i32);
            worst_length = worst_length.max(n.net_length);
            worst_slack = worst_slack.min(n.slack);
            if n.is_clock_sig || n.has_ndr() {
                n.res_aware = true;
            } else if !n.res_aware {
                candidates.push(k);
            }
        }
        self.worst = (worst_slack, worst_resistance, worst_fanout, worst_length);
        let mut scored: Vec<(usize, f32)> = candidates.iter().map(|&k| (k, self.res_aware_score(k))).collect();
        scored.sort_by(|a, b| (a.1, a.0).partial_cmp(&(b.1, b.0)).unwrap_or(std::cmp::Ordering::Equal));
        if let Some(t) = trace.as_deref_mut() {
            t.push(format!("VYGC|raworst|{stage}|{:08x}|{:08x}|{worst_fanout}|{worst_length}", worst_slack.to_bits(), worst_resistance.to_bits()));
            for (i, &(k, score)) in scored.iter().enumerate() {
                t.push(format!("VYGC|rascore|{stage}|{i}|{}|{:08x}", self.nets[k].name, score.to_bits()));
            }
        }
        let count = if self.incremental_candidates.is_some() { scored.len() } else { (scored.len() as f32 * self.res_aware_percentage / 100.0f32).ceil() as usize };
        for &(k, _) in scored.iter().take(count) {
            self.nets[k].res_aware = true;
        }
        if let Some(t) = trace {
            t.push(format!("VYGC|racount|{stage}|{count}|{}", scored.len()));
        }
    }

    /// `patternRouteResAware` (stage 2), with resistance-aware routing and critical nets only: the
    /// critical nets updated and marked on the stage-1 trees, then every marked net — in the
    /// res-aware order — ripped up and pattern-routed again (no detours), its res-aware costs on.
    pub fn pattern_route_res_aware(&mut self, alphas: &[f32], stt: pattern_route::SteinerBuilder<'_>, log: &mut Vec<String>, mut trace: Option<&mut Vec<String>>) -> Result<(), StageError> {
        if !self.res_aware_order() {
            return Ok(());
        }
        self.update_critical_nets(2, trace.as_deref_mut())?;
        let mut nets: Vec<usize> = (0..self.nets.len()).filter(|&k| self.nets[k].res_aware).collect();
        if nets.is_empty() {
            return Ok(());
        }
        self.sort_net_indices(&mut nets, true, 2, trace.as_deref_mut())?;
        for &k in nets.iter() {
            if self.nets[k].num_pins() < 2 {
                continue;
            }
            let mut ripped = Vec::new();
            self.commit_net(k, true, &mut ripped)?;
            if let Some(t) = trace.as_deref_mut() {
                trace::commits(t, &ripped, 2);
            }
            let cx = pattern_route::CostContext { grid: &self.grid, design: &self.design, constants: &self.constants, cost_multiplier: self.cost_multiplier };
            let route = pattern_route::pattern_route_net(&mut self.nets[k], alphas[k], stt, &cx, None, log).map_err(StageError::Pattern)?;
            let mut commits = Vec::new();
            self.commit_net(k, false, &mut commits)?;
            if let Some(t) = trace.as_deref_mut() {
                trace::net_route(t, &self.nets[k], &route, &commits, 2);
            }
        }
        Ok(())
    }

    fn sort_by_slack_then_hp(&self, indices: &mut [usize]) {
        indices.sort_by(|&a, &b| {
            let (na, nb) = (&self.nets[a], &self.nets[b]);
            (na.slack, na.bounding_box.hp()).partial_cmp(&(nb.slack, nb.bounding_box.hp())).unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    /// `patternRoute` (stage 1): every net in the neutral order; a net of two or more pins is
    /// pattern-routed and its tree's usage committed before the next is routed.
    ///
    /// `alphas[k]` is net `k`'s Steiner alpha. With `trace`, the stage's records are appended.
    ///
    /// ⚠️ The list is sorted IN PLACE, as upstream's is: an incremental route hands the sorted
    /// list on to stages 3 and 4.
    pub fn pattern_route(&mut self, order: &mut Vec<usize>, alphas: &[f32], stt: pattern_route::SteinerBuilder<'_>, log: &mut Vec<String>, mut trace: Option<&mut Vec<String>>) -> Result<(), StageError> {
        self.sort_net_indices(order, false, 1, None)?;
        if let Some(t) = trace.as_deref_mut() {
            trace::order(t, self, order, 1);
        }
        for &k in order.iter() {
            if self.nets[k].num_pins() < 2 {
                continue;
            }
            let cx = pattern_route::CostContext { grid: &self.grid, design: &self.design, constants: &self.constants, cost_multiplier: 1.0 };
            let route = pattern_route::pattern_route_net(&mut self.nets[k], alphas[k], stt, &cx, None, log).map_err(StageError::Pattern)?;
            let mut commits = Vec::new();
            let tree = self.nets[k].routing_tree.clone().expect("set by the route");
            self.grid.commit_tree(&self.design, &tree, false, &self.nets[k].ndr_costs, false, &mut commits).map_err(StageError::Commit)?;
            if let Some(t) = trace.as_deref_mut() {
                trace::net_route(t, &self.nets[k], &route, &commits, 1);
            }
        }
        if let Some(t) = trace {
            trace::demand(t, &self.grid, 1);
        }
        Ok(())
    }

    /// `updateCongestedNets(threshold 1)` over every routed net, in index order: those whose tree
    /// crosses an edge with more demand than capacity. Empty means stages 3 to 5 have nothing to
    /// do. `tag` is the stage the trace records it under.
    pub fn congested_nets(&self, tag: i32, trace: Option<&mut Vec<String>>) -> Vec<usize> {
        let scope: Vec<usize> = match &self.incremental_candidates {
            Some(c) => c.clone(),
            None => (0..self.nets.len()).collect(),
        };
        let out: Vec<usize> = scope
            .into_iter()
            .filter(|&k| self.nets[k].routing_tree.as_ref().is_some_and(|t| self.grid.check_congestion(t, 1.0) > 0))
            .collect();
        if let Some(t) = trace {
            t.push(format!("VYGC|cong|{tag}|{}|{}", out.len(), trace::list(&out)));
        }
        out
    }

    /// Rip up or restore one net's tree, its commits appended.
    fn commit_net(&mut self, k: usize, rip_up: bool, commits: &mut Vec<grid_graph::Commit>) -> Result<(), StageError> {
        let Some(tree) = self.nets[k].routing_tree.clone() else { return Ok(()) };
        self.grid.commit_tree(&self.design, &tree, rip_up, &self.nets[k].ndr_costs, self.nets[k].adopted, commits).map_err(StageError::Commit)
    }

    /// `patternRouteWithDetours` (stage 3): the overflow view taken ONCE, the congested nets in the
    /// neutral order, each ripped up and pattern-routed again with detours through the view.
    pub fn pattern_route_with_detours(&mut self, nets: &mut Vec<usize>, alphas: &[f32], stt: pattern_route::SteinerBuilder<'_>, log: &mut Vec<String>, mut trace: Option<&mut Vec<String>>) -> Result<(), StageError> {
        if nets.is_empty() {
            return Ok(());
        }
        if self.critical_nets_percentage != 0.0 {
            self.update_critical_nets(3, trace.as_deref_mut())?;
        }
        let view = self.grid.extract_congestion_view();
        if let Some(t) = trace.as_deref_mut() {
            trace::congestion_view(t, &view);
        }
        let res_aware = self.res_aware_order();
        self.sort_net_indices(nets, res_aware, 3, trace.as_deref_mut())?;
        if let Some(t) = trace.as_deref_mut() {
            trace::order(t, self, nets, 3);
        }
        for &k in nets.iter() {
            if self.nets[k].num_pins() < 2 {
                continue;
            }
            // The rip-up is recorded before the route, as the reference's trace has it.
            let mut ripped = Vec::new();
            self.commit_net(k, true, &mut ripped)?;
            if let Some(t) = trace.as_deref_mut() {
                trace::commits(t, &ripped, 3);
            }
            let cx = pattern_route::CostContext { grid: &self.grid, design: &self.design, constants: &self.constants, cost_multiplier: self.cost_multiplier };
            let route = pattern_route::pattern_route_net(&mut self.nets[k], alphas[k], stt, &cx, Some(&view), log).map_err(StageError::Pattern)?;
            let mut commits = Vec::new();
            self.commit_net(k, false, &mut commits)?;
            if let Some(t) = trace.as_deref_mut() {
                trace::net_route(t, &self.nets[k], &route, &commits, 3);
            }
        }
        if let Some(t) = trace {
            trace::demand(t, &self.grid, 3);
        }
        Ok(())
    }

    /// `mazeRoute(nets, stage)` (stage 4, and each RRR iteration as stage 5): EVERY listed net
    /// ripped up first, then the wire-cost view taken, the nets in the neutral order, each routed
    /// on a sparsified grid whose offset steps after every net, its tree laid onto layers by
    /// pattern routing, committed, and the view re-priced along it.
    pub fn maze_route(&mut self, nets: &mut Vec<usize>, stage: i32, log: &mut Vec<String>, mut trace: Option<&mut Vec<String>>) -> Result<(), StageError> {
        if nets.is_empty() {
            return Ok(());
        }
        if self.critical_nets_percentage != 0.0 {
            self.update_critical_nets(stage, trace.as_deref_mut())?;
        }
        let mut ripped = Vec::new();
        for &k in nets.iter() {
            self.commit_net(k, true, &mut ripped)?;
        }
        let mut view: grid_graph::View<f64> = Vec::new();
        self.grid.extract_wire_cost_view(&mut view, &[], &self.constants, self.cost_multiplier);
        let res_aware = self.res_aware_order();
        let mut sorted_trace = Vec::new();
        self.sort_net_indices(nets, res_aware, stage, Some(&mut sorted_trace))?;
        if let Some(t) = trace.as_deref_mut() {
            trace::commits(t, &ripped, stage);
            t.extend(sorted_trace);
            trace::order(t, self, nets, stage);
            trace::wire_cost_view(t, &view, stage);
        }
        let mut grid = maze_route::SparseGrid::new(10, 10, 0, 0);
        let mut ndr_view: grid_graph::View<f64> = Vec::new();
        for &k in nets.iter() {
            if self.nets[k].num_pins() < 2 {
                continue;
            }
            let selected = pattern_route::select_access_points(&mut self.nets[k], &self.grid, log).map_err(StageError::Pattern)?;
            let has_ndr = self.nets[k].has_ndr();
            if has_ndr {
                self.grid.extract_wire_cost_view(&mut ndr_view, &self.nets[k].ndr_costs, &self.constants, self.cost_multiplier);
            }
            let (sparse, arena, found, steiner) = maze_route::maze_tree(&selected, if has_ndr { &ndr_view } else { &view }, &grid, &self.grid).map_err(|e| StageError::Maze(e))?;
            let cx = pattern_route::CostContext { grid: &self.grid, design: &self.design, constants: &self.constants, cost_multiplier: self.cost_multiplier };
            let mut route = pattern_route::pattern_route_tree(&mut self.nets[k], steiner, &cx).map_err(StageError::Pattern)?;
            route.maze = Some((sparse, arena, found));
            let mut commits = Vec::new();
            self.commit_net(k, false, &mut commits)?;
            let tree = self.nets[k].routing_tree.clone().expect("set by the route");
            self.grid.update_wire_cost_view(&mut view, &tree, &self.constants, self.cost_multiplier);
            if let Some(t) = trace.as_deref_mut() {
                // The reference's order: the access points (chosen by the sparse graph), the maze
                // (graph, paths, tree), then the DAG, the tree and the commits.
                let mut lines = Vec::new();
                trace::net_route(&mut lines, &self.nets[k], &route, &commits, stage);
                let split = lines.iter().position(|l| !(l.starts_with("VYGC|odbap|") || l.starts_with("VYGC|shapeap|"))).unwrap_or(lines.len());
                let mut maze = Vec::new();
                trace::maze(&mut maze, &self.nets[k], &route, &grid);
                lines.splice(split..split, maze);
                t.extend(lines);
            }
            grid.step();
        }
        if let Some(t) = trace {
            trace::demand(t, &self.grid, stage);
        }
        Ok(())
    }

    /// `iterativeRRR` (stage 5): only with integer overflow left; up to `iterations` rounds, each
    /// taking the congested nets again (stopping when none), demoting an NDR net congested two
    /// rounds running to the default rule, raising the logistic slope by 1 up to 6, and maze-
    /// routing the congested set. The multiplier is reset after.
    pub fn iterative_rrr(&mut self, nets: &mut Vec<usize>, iterations: i32, log: &mut Vec<String>, mut trace: Option<&mut Vec<String>>) -> Result<(), StageError> {
        if self.grid.total_overflow() == 0 {
            return Ok(());
        }
        let mut streak: std::collections::HashMap<usize, i32> = std::collections::HashMap::new();
        let mut multiplier = 1.0f64;
        for i in 1..=iterations {
            *nets = self.congested_nets(5, trace.as_deref_mut());
            if nets.is_empty() {
                break;
            }
            let current: std::collections::HashSet<usize> = nets.iter().copied().collect();
            let mut demoted = Vec::new();
            let mut commits = Vec::new();
            for &k in nets.iter() {
                if !self.nets[k].has_ndr() {
                    continue;
                }
                let s = streak.entry(k).or_insert(0);
                *s += 1;
                if *s >= 2 {
                    self.commit_net(k, true, &mut commits)?;
                    self.nets[k].set_soft_ndr();
                    self.commit_net(k, false, &mut commits)?;
                    demoted.push(self.nets[k].name.clone());
                }
            }
            for (k, s) in streak.iter_mut() {
                if !current.contains(k) {
                    *s = 0;
                }
            }
            if !demoted.is_empty() {
                log.push(format!("[WARNING GRT-0305] Demoted {} NDR net(s) to default rule to reduce congestion (use debug 'softNDR' for net list).", demoted.len()));
            }
            if multiplier < 6.0 {
                multiplier += 1.0;
            }
            self.cost_multiplier = multiplier;
            if let Some(t) = trace.as_deref_mut() {
                trace::commits(t, &commits, 5);
                t.push(format!("VYGC|rrr|{i}|{multiplier}|{}|{}|demoted={}", nets.len(), trace::list(nets.iter()), demoted.iter().map(|d| format!("{d},")).collect::<String>()));
            }
            // Incremental re-mazes every candidate each round, in the candidates' own order.
            match self.incremental_candidates.clone() {
                Some(mut all) => self.maze_route(&mut all, 5, log, trace.as_deref_mut())?,
                None => self.maze_route(nets, 5, log, trace.as_deref_mut())?,
            }
        }
        self.cost_multiplier = 1.0;
        let residual = self.grid.total_overflow();
        if residual > 0 && self.incremental_candidates.is_none() {
            log.push(format!("[WARNING GRT-0118] Iterative RRR finished with congestion remaining ({residual})."));
        }
        Ok(())
    }
}

impl Cugr {
    /// `gridlineCenter(dimension, index)`: a gcell's LOW gridline plus half the NOMINAL gcell —
    /// not the gcell's own centre (the last gcell is wider).
    pub fn gridline_center(&self, dimension: usize, index: i32) -> i32 {
        self.grid.gridlines[dimension][index as usize] + self.design.gridline_spacing / 2
    }

    /// `GRNet::isLocal`: no preferred access point, or every one at the same cell.
    pub fn is_local(net: &grnet::GrNet) -> bool {
        let mut cells = net.preferred_aps.values().map(|&(p, _)| p);
        match cells.next() {
            None => true,
            Some(first) => cells.all(|p| p == first),
        }
    }

    /// `getNetRoute` / `buildNetRoute`: a net of two or more pins that is not local, as DBU
    /// segments at gridline centres, in tree preorder.
    ///
    /// Upstream rule: a same-layer child is one segment from the lower to the higher corner, a
    /// degenerate one (same cell) skipped; a layer change is one via segment per layer crossed, at
    /// the parent's cell, bottom up. Layers are 1-based routing levels.
    pub fn net_route(&self, net: &grnet::GrNet) -> Vec<crate::GSegment> {
        let mut route = Vec::new();
        let Some(tree) = net.routing_tree.as_ref() else { return route };
        if net.num_pins() < 2 || Cugr::is_local(net) {
            return route;
        }
        for n in tree.preorder() {
            let node = &tree.nodes[n];
            for &ch in &node.children {
                let child = &tree.nodes[ch];
                if node.layer == child.layer {
                    if node.p == child.p {
                        continue;
                    }
                    route.push(crate::GSegment {
                        init_x: self.gridline_center(0, node.p.x.min(child.p.x)),
                        init_y: self.gridline_center(1, node.p.y.min(child.p.y)),
                        init_layer: node.layer + 1,
                        final_x: self.gridline_center(0, node.p.x.max(child.p.x)),
                        final_y: self.gridline_center(1, node.p.y.max(child.p.y)),
                        final_layer: child.layer + 1,
                        is_jumper: false,
                    });
                } else {
                    let (x, y) = (self.gridline_center(0, node.p.x), self.gridline_center(1, node.p.y));
                    for l in node.layer.min(child.layer)..node.layer.max(child.layer) {
                        route.push(crate::GSegment { init_x: x, init_y: y, init_layer: l + 1, final_x: x, final_y: y, final_layer: l + 2, is_jumper: false });
                    }
                }
            }
        }
        route
    }

    /// `getITermsAccessPoints` / `getBTermsAccessPoints` for one pin, by terminal name: the chosen
    /// cell's LOW gridlines and the top of the pin's layer interval, as a 1-based layer.
    pub fn pin_access_point(&self, net: &grnet::GrNet, pin_name: &str, is_port: bool) -> Option<(i32, i32, i32)> {
        let design_net = self.design.nets.get(net.index)?;
        let pin = design_net.pins.iter().find(|p| p.name == pin_name && p.is_port == is_port)?;
        let &(p, layers) = net.preferred_aps.get(&pin.index)?;
        Some((self.grid.gridlines[0][p.x as usize], self.grid.gridlines[1][p.y as usize], layers.high + 1))
    }
}

impl Cugr {
    /// `CUGR::updateNet(net)` for a net the router holds: its tree's demand released (with its
    /// adopted flag), the net rebuilt from `net_facts` as new — no preferred access points, slack
    /// 0 — with its rule's factors, kept at 1 if it was soft-demoted; queued for the next route.
    pub fn update_net(&mut self, k: usize, pins: Vec<design::CugrPin>, layer_range: design::LayerRange, driver_term: &str, ndr_costs: Vec<f64>, ndr_widths: Vec<i32>, queue: &mut Vec<usize>) -> Result<(), StageError> {
        let was_soft = self.nets[k].soft_ndr;
        if self.nets[k].routing_tree.is_some() {
            let mut commits = Vec::new();
            self.commit_net(k, true, &mut commits)?;
        }
        self.design.nets[k].pins = pins;
        self.design.nets[k].layer_range = layer_range;
        let mut n = grnet::GrNet::new(&self.design.nets[k], driver_term, &self.grid).map_err(|e| StageError::Maze(format!("{e:?}")))?;
        n.ndr_costs = ndr_costs;
        n.ndr_widths = ndr_widths;
        if was_soft {
            n.set_soft_ndr();
        }
        self.nets[k] = n;
        queue.push(k);
        Ok(())
    }

    /// `CUGR::updateNet`'s branch for a net CUGR does not hold: the design appended it at `k`
    /// ([`design::Design::update_net`]); its GRNet is appended too, with its rule's factors, and
    /// queued.
    pub fn add_net(&mut self, k: usize, driver_term: &str, ndr_costs: Vec<f64>, ndr_widths: Vec<i32>, queue: &mut Vec<usize>) -> Result<(), StageError> {
        if k != self.nets.len() {
            return Err(StageError::Maze(format!("cugr: a new net at {k} with {} held — not aligned", self.nets.len())));
        }
        let mut n = grnet::GrNet::new(&self.design.nets[k], driver_term, &self.grid).map_err(|e| StageError::Maze(format!("{e:?}")))?;
        n.ndr_costs = ndr_costs;
        n.ndr_widths = ndr_widths;
        self.nets.push(n);
        queue.push(k);
        Ok(())
    }

    /// `CUGR::route(true)`: the queued nets only — stage 1 on them (sorted in place), then stages
    /// 3 and 4 on ALL of them (no congested-set narrowing), then RRR scoped to them; no GRT-0118.
    pub fn route_incremental(&mut self, queued: Vec<usize>, iterations: i32, alphas: &[f32], stt: pattern_route::SteinerBuilder<'_>, log: &mut Vec<String>, mut trace: Option<&mut Vec<String>>) -> Result<(), StageError> {
        if let Some(t) = trace.as_deref_mut() {
            t.push("VYGC|route|1".into());
        }
        // Nothing queued (a restore- or remove-only round): no stage runs, and `route` returns
        // before its end.
        if queued.is_empty() {
            return Ok(());
        }
        self.incremental_candidates = Some(queued.clone());
        let mut nets = queued;
        self.pattern_route(&mut nets, alphas, stt, log, trace.as_deref_mut())?;
        self.pattern_route_with_detours(&mut nets, alphas, stt, log, trace.as_deref_mut())?;
        self.maze_route(&mut nets, 4, log, trace.as_deref_mut())?;
        self.iterative_rrr(&mut nets, iterations, log, trace.as_deref_mut())?;
        self.incremental_candidates = None;
        if let Some(t) = trace {
            t.push("VYGC|routed".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InitError {
    Design(DesignError),
    Grid(GridError),
}

/// `CUGR::init(min, max, clock_nets)`: `Design`, then `GridGraph`, then one `GRNet` per net.
///
/// `driver_terms[k]` is the first driver terminal of `facts.nets[k]` (see
/// [`grnet::GrNet::new`]). NDR costs are not modelled yet.
pub fn init(facts: &DesignFacts, driver_terms: &[String], min_routing_layer: i32, max_routing_layer: i32) -> Result<Cugr, InitError> {
    let constants = Constants { min_routing_layer: min_routing_layer - 1, ..Constants::default() };
    let design = Design::new(facts, &constants, min_routing_layer, max_routing_layer).map_err(InitError::Design)?;
    let grid = GridGraph::new(&design, constants.min_routing_layer as usize).map_err(InitError::Grid)?;
    let nets = design
        .nets
        .iter()
        .map(|n| GrNet::new(n, &driver_terms[n.facts_index], &grid))
        .collect::<Result<Vec<_>, _>>()
        .map_err(InitError::Grid)?;
    Ok(Cugr { constants, design, grid, nets, cost_multiplier: 1.0, sort_slacks: None, sorts_done: 0, incremental_candidates: None, resistance_aware: false, critical_nets_percentage: 0.0, res_aware_percentage: 15.0, raw_slacks: None, raw_done: 0, constant_slack: None, worst: (1.0, 1.0, 1, 1), slack_source: SlackSource::default() })
}

#[cfg(test)]
mod tests {
    use super::design::{DesignFacts, TechLayerFacts};
    use super::geo::{Interval, Point};
    use super::grid_graph::GrTree;
    use super::grnet::{GrNet, GrPoint};
    use super::layers::MetalLayerFacts;
    use super::*;

    fn layer(name: &str, level: i32, horizontal: bool) -> TechLayerFacts {
        TechLayerFacts {
            name: name.into(),
            is_routing: true,
            routing_level: level,
            upper_layer: String::new(),
            metal: Some(MetalLayerFacts {
                name: name.into(),
                routing_level: level,
                horizontal,
                width: 100,
                min_width: 100,
                spacing: 100,
                resistance: 0.0,
                via_resistance: 0.0,
                tracks: (200, 100, 30),
                area: 0,
                v55_widths_and_lengths: None,
                v55_table: None,
                adjustment: 0.0,
            }),
        }
    }

    /// Gridlines 0, 1000, …, 4000, 5900: the last gcell is 1900 wide.
    fn router() -> Cugr {
        let f = DesignFacts {
            dbu_per_micron: 1000,
            die: (0, 0, 5900, 5900),
            gcell_tile_size: 1000,
            layers: vec![layer("m1", 1, true), layer("m2", 2, false), layer("m3", 3, true)],
            vias: Vec::new(),
            nets: Vec::new(),
            instance_shapes: Vec::new(),
            design_obstructions: Vec::new(),
            min_layer_for_clock: 0,
            max_layer_for_clock: 0,
        };
        init(&f, &[], 2, 3).unwrap()
    }

    fn net_with(tree: GrTree, aps: &[(usize, Point)]) -> GrNet {
        GrNet {
            index: 0,
            name: "n".into(),
            pin_access_points: vec![vec![GrPoint { layer: 0, p: Point::new(0, 0) }]; 2],
            bounding_box: Default::default(),
            driver_pin_index: -1,
            layer_range: design::LayerRange { min_layer: 1, max_layer: 2 },
            slack: 0.0,
            preferred_aps: aps.iter().map(|&(pin, p)| (pin, (p, Interval::point(0)))).collect(),
            ndr_costs: vec![1.0; 3],
            routing_tree: Some(tree),
            shape_ap_choices: Vec::new(),
            soft_ndr: false,
            adopted: false,
            ndr_widths: Vec::new(),
            res_aware: false,
            resistance: 0.0,
            net_length: 0,
            is_clock_sig: false,
            odb_aps: Vec::new(),
            odb_ap_choices: Vec::new(),
        }
    }

    // Upstream rule (CUGR `buildNetRoute` / `gridlineCenter`): a cell's DBU point is its LOW
    // gridline plus HALF THE NOMINAL gcell — in the 1900-wide last gcell (4000..5900) that is 4500,
    // not its own centre 4950. A same-layer child at the SAME cell is skipped; a layer change is one
    // via segment per layer crossed, at the parent's cell, bottom up; layers are 1-based.
    #[test]
    fn net_route_segments() {
        let c = router();
        let mut t = GrTree::default();
        let root = t.add(0, Point::new(1, 4));
        let up = t.add(2, Point::new(1, 4));
        let same = t.add(2, Point::new(1, 4));
        let far = t.add(2, Point::new(4, 4));
        t.nodes[root].children.push(up);
        t.nodes[up].children.push(same);
        t.nodes[up].children.push(far);
        let n = net_with(t, &[(0, Point::new(1, 4)), (1, Point::new(4, 4))]);
        let r = c.net_route(&n);
        let seg = |s: &crate::GSegment| (s.init_x, s.init_y, s.init_layer, s.final_x, s.final_y, s.final_layer);
        let got: Vec<_> = r.iter().map(seg).collect();
        assert_eq!(got, vec![(1500, 4500, 1, 1500, 4500, 2), (1500, 4500, 2, 1500, 4500, 3), (1500, 4500, 3, 4500, 4500, 3)]);
    }

    // Upstream rule (CUGR `computeNdrCosts`): (W + 2S + D) / 2P from an integer numerator, floored
    // at 1; a non-routing layer, one out of range, or a zero pitch is skipped. A 1w/3s rule on a
    // 140-wide, 380-pitch layer: (140 + 840 + 140) / 760.
    #[test]
    fn ndr_cost_factors() {
        let r = |is_routing, routing_level, width, spacing, default_pitch| NdrRuleFacts { is_routing, routing_level, width, spacing, default_width: 140, default_pitch };
        let f = ndr_costs(3, &[r(true, 2, 140, 420, 380), r(true, 3, 0, 0, 380), r(false, 1, 999, 999, 380), r(true, 4, 999, 999, 380), r(true, 1, 999, 0, 0)]);
        assert_eq!(f, vec![1.0, 1120.0 / 760.0, 1.0]);
    }

    // Upstream rule (CUGR `getNetRoute`, `GRNet::isLocal`): a net whose pins all chose ONE cell
    // exports nothing — `addRemainingGuides` covers it.
    #[test]
    fn a_local_net_exports_nothing() {
        let c = router();
        let mut t = GrTree::default();
        let a = t.add(1, Point::new(2, 2));
        let b = t.add(2, Point::new(2, 2));
        t.nodes[a].children.push(b);
        assert!(c.net_route(&net_with(t.clone(), &[(0, Point::new(2, 2)), (1, Point::new(2, 2))])).is_empty());
        assert_eq!(c.net_route(&net_with(t, &[(0, Point::new(2, 2)), (1, Point::new(2, 3))])).len(), 1);
    }

    /// A net on m3 (horizontal) from cell (1,1) to (3,1), every edge on its way overfull.
    fn congested(c: &mut Cugr) -> GrNet {
        let mut t = GrTree::default();
        let a = t.add(2, Point::new(1, 1));
        let b = t.add(2, Point::new(3, 1));
        t.nodes[a].children.push(b);
        for x in 1..3 {
            let e = &mut c.grid.graph_edges[2][x][1];
            e.demand = e.capacity + 5.0;
        }
        net_with(t, &[(0, Point::new(1, 1)), (1, Point::new(3, 1))])
    }

    // Upstream rule (CUGR `route(true)` → `getCongestedNets`): an incremental route looks for
    // congestion ONLY among its candidates (`incremental_candidates_`), never over every net. In
    // the corpus the diode reroute has no congested net outside its candidates.
    #[test]
    fn incremental_congestion_is_scoped_to_the_candidates() {
        let mut c = router();
        let n = congested(&mut c);
        c.nets = vec![n.clone(), n];
        assert_eq!(c.congested_nets(0, None), vec![0, 1]);
        c.incremental_candidates = Some(vec![1]);
        assert_eq!(c.congested_nets(0, None), vec![1]);
    }

    // Upstream rule (CUGR `updateNet`): the net is rebuilt from its new pins with its rule's
    // factors, but a net marked soft-NDR STAYS soft (its factors back to 1), and it is queued.
    // No corpus net is soft-NDR when diodes land.
    #[test]
    fn update_net_keeps_soft_ndr() {
        let mut c = router();
        let range = design::LayerRange { min_layer: 1, max_layer: 2 };
        c.design.nets.push(design::CugrNet { index: 0, name: "n".into(), pins: Vec::new(), layer_range: range, facts_index: 0 });
        let mut n = net_with(GrTree::default(), &[]);
        n.routing_tree = None;
        n.set_soft_ndr();
        c.nets = vec![n];
        let mut queue = Vec::new();
        c.update_net(0, Vec::new(), range, "", vec![2.0; 3], Vec::new(), &mut queue).unwrap();
        assert!(c.nets[0].soft_ndr);
        assert_eq!(c.nets[0].ndr_costs, vec![1.0; 3]);
        assert_eq!(queue, vec![0]);
        c.nets[0].soft_ndr = false;
        c.update_net(0, Vec::new(), range, "", vec![2.0; 3], Vec::new(), &mut queue).unwrap();
        assert_eq!(c.nets[0].ndr_costs, vec![2.0; 3], "a hard-NDR net keeps its factors");
    }

    // Upstream rule (`GRNet::setSoftNdr`): a soft-demoted net loses its NDR wire widths with its
    // demand factors, so a res-aware cost reads the layer's width again. No soft-NDR net is
    // resistance-aware in the corpus.
    #[test]
    fn soft_ndr_clears_the_wire_widths() {
        let mut n = net_with(GrTree::default(), &[]);
        n.ndr_costs = vec![1.0, 2.0, 1.0];
        n.ndr_widths = vec![0, 300, 0];
        n.set_soft_ndr();
        assert_eq!(n.ndr_widths, vec![0, 0, 0]);
        assert_eq!(n.ndr_width(1), 0);
        assert_eq!(n.ndr_width(9), 0, "off the end");
    }

    /// Two-pin nets on m3 from (0,0) to (4,0): long enough (4 > 3 gcells) to be eligible.
    fn critical_candidate(slack: f32) -> GrNet {
        let mut t = GrTree::default();
        let a = t.add(2, Point::new(0, 0));
        let b = t.add(2, Point::new(4, 0));
        t.nodes[a].children.push(b);
        let mut n = net_with(t, &[]);
        n.slack = slack;
        n
    }

    // Upstream rules (`markResAwareNets`): a CLOCK-typed net is eligible even with a positive
    // slack, and eligible CLOCK nets are marked outright; ranked candidates that tie on score go
    // by index. Every clock net of the stage-2 corpus has a negative slack, and no two candidates
    // tie.
    #[test]
    fn marking_takes_positive_clocks_and_breaks_ties_by_index() {
        let mut c = router();
        c.resistance_aware = true;
        c.res_aware_percentage = 50.0;
        let mut clock = critical_candidate(1.0);
        clock.is_clock_sig = true;
        c.nets = vec![critical_candidate(-1.0), critical_candidate(-1.0), clock];
        c.mark_res_aware_nets(2, None);
        assert!(c.nets[2].res_aware, "a positive-slack CLOCK net is still marked");
        assert!(c.nets[0].res_aware && !c.nets[1].res_aware, "a tie goes to the lower index: ceil(2 * 50%) = 1");
    }
}
