// SPDX-License-Identifier: Apache-2.0
//! R16 — choosing which nets the router treats as resistance-aware.
//!
//! Reads each net's slack, measures its length, sets both on the net, and then — for the nets that
//! qualify — scores them and marks the worst fraction.
//!
//! ⛔ **The whole pass is a no-op unless the router is resistance-aware and a liberty library is
//! loaded.** It returns before touching anything, so in a default run no net's slack, length or
//! resistance is written at all. A corpus from default runs contains nothing of this function.
//!
//! ⛔ **The score divides by worst-case values that are still being accumulated.** Each net's
//! score is computed inside the same loop that updates the worst metrics, so it divides by the
//! worst seen **so far** — the nets before it in order, including itself — not by the final
//! values. Hoisting the accumulation into its own pass reads better and produces different
//! scores, and therefore a different marked set.
//!
//! ⚠️ **Slack and length are written for EVERY net, including the ones the pass then skips.**
//! Only resistance, the worst metrics and candidacy are confined to the survivors.

/// A net of this length or shorter is never made resistance-aware.
pub const SHORT_NET_THRESHOLD: i32 = 3;

/// The score's four weights, in the reference's own order.
pub const RESISTANCE_WEIGHT: f32 = 1.0;
pub const SLACK_WEIGHT: f32 = 4.0;
pub const FANOUT_WEIGHT: f32 = 3.0;
pub const NET_LENGTH_WEIGHT: f32 = 2.0;

/// The run-wide worst values the score divides by.
///
/// ⚠️ Slack takes the **minimum** and the other three the maximum — "worst" means a different
/// direction per field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorstMetrics {
    pub resistance: f32,
    pub slack: f32,
    pub net_length: i32,
    pub fanout: i32,
}

impl WorstMetrics {
    /// ⚠️ Slack resets to the timer's infinity, not to zero like the rest.
    pub fn reset(infinity: f32) -> Self {
        WorstMetrics { resistance: 0.0, slack: infinity, net_length: 0, fanout: 0 }
    }

    fn update(&mut self, resistance: f32, slack: f32, net_length: i32, fanout: i32) {
        self.resistance = self.resistance.max(resistance);
        self.slack = self.slack.min(slack);
        self.net_length = self.net_length.max(net_length);
        self.fanout = self.fanout.max(fanout);
    }
}

/// One net's score, against the worst metrics **as they stand**.
pub fn res_aware_score(
    worst: &WorstMetrics,
    resistance: f32,
    slack: f32,
    num_pins: i32,
    net_length: i32,
) -> f32 {
    resistance / worst.resistance * RESISTANCE_WEIGHT
        + slack / worst.slack * SLACK_WEIGHT
        + num_pins as f32 / worst.fanout as f32 * FANOUT_WEIGHT
        + net_length as f32 / worst.net_length as f32 * NET_LENGTH_WEIGHT
}

/// What one net brings in.
pub struct NetSlackInput<'a> {
    pub net_id: usize,
    /// As the timer reports it.
    pub slack: f32,
    /// Per edge of the net's tree.
    pub edge_len: &'a [i32],
    pub num_pins: i32,
    pub is_clock: bool,
    pub has_ndr: bool,
    /// Whether the net was already resistance-aware on entry.
    pub is_res_aware: bool,
    /// ⚠️ Captured, not derived: the reference reads it from the database, on a layer that depends
    /// on whether this is a three-dimensional step. Only the surviving nets are asked for it.
    pub resistance: f32,
}

/// What the pass wrote onto one net.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NetSlackOutput {
    pub net_id: usize,
    pub slack: f32,
    pub net_length: i32,
    /// `None` for a net the pass skipped — its resistance is never read or written.
    pub resistance: Option<f32>,
    pub is_res_aware: bool,
    /// The score this net was given, and the worst metrics **as they stood** when it was given.
    ///
    /// ⚠️ An observability hook, not an output of the reference: it exposes the one value that
    /// depends on where in the loop the net sits, so a corpus can gate the accumulation order
    /// rather than only the final marked set. `None` for a skipped net.
    pub score: Option<f32>,
    pub worst_at_score: Option<WorstMetrics>,
}

/// Everything the pass produced.
#[derive(Debug, Clone, PartialEq)]
pub struct SlackUpdate {
    pub nets: Vec<NetSlackOutput>,
    /// The candidates, scored and sorted.
    pub candidates: Vec<(usize, f32)>,
    pub worst: WorstMetrics,
    /// How many of the candidates the percentage marked.
    pub marked: usize,
}

/// The run-wide settings this pass reads.
pub struct SlackParams {
    /// A liberty library is loaded **and** the router is resistance-aware.
    pub enabled: bool,
    pub is_incremental: bool,
    /// The configured percentage, as a percentage — the reference divides by 100 here.
    pub percentage: f32,
    /// The timer's infinity, which an unconstrained net's slack equals exactly.
    pub infinity: f32,
}

/// Mark the worst fraction of qualifying nets as resistance-aware.
pub fn update_slacks(nets: &[NetSlackInput<'_>], p: &SlackParams) -> SlackUpdate {
    let mut out: Vec<NetSlackOutput> = nets
        .iter()
        .map(|n| NetSlackOutput {
            net_id: n.net_id,
            slack: n.slack,
            net_length: 0,
            resistance: None,
            is_res_aware: n.is_res_aware,
            score: None,
            worst_at_score: None,
        })
        .collect();

    // ⛔ Before anything is read or written. Every net comes back exactly as it went in.
    if !p.enabled {
        return SlackUpdate {
            nets: out,
            candidates: Vec::new(),
            worst: WorstMetrics::reset(p.infinity),
            marked: 0,
        };
    }

    let mut worst = WorstMetrics::reset(p.infinity);
    let mut candidates: Vec<(usize, f32)> = Vec::new();

    for (n, o) in nets.iter().zip(out.iter_mut()) {
        // ⚠️ Written for every net, whether or not it goes on to qualify.
        let net_length: i32 = n.edge_len.iter().sum();
        o.slack = n.slack;
        o.net_length = net_length;

        let is_short = net_length <= SHORT_NET_THRESHOLD;
        // ⚠️ Exact equality against the timer's infinity, not a magnitude test.
        let is_unconstrained = n.slack == p.infinity && !n.is_clock;
        let is_positive_slack = !p.is_incremental && n.slack > 0.0 && !n.is_clock;
        if is_unconstrained || is_short || is_positive_slack {
            continue;
        }

        o.resistance = Some(n.resistance);
        // ⛔ This net's own values enter the worst metrics BEFORE its score divides by them.
        worst.update(n.resistance, n.slack, net_length, n.num_pins);

        // A clock or NDR net is made resistance-aware outright, and is then not a candidate.
        if n.has_ndr || n.is_clock {
            o.is_res_aware = true;
        }
        // ⛔ Scored HERE, against the partially accumulated worst metrics.
        let score = -res_aware_score(&worst, n.resistance, n.slack, n.num_pins, net_length);
        o.score = Some(score);
        o.worst_at_score = Some(worst);
        if !o.is_res_aware {
            candidates.push((n.net_id, score));
        }
    }

    // ⚠️ Stable, ordered by score then net id — the id makes the order total.
    candidates.sort_by(|a, b| {
        if a.1 < b.1 {
            std::cmp::Ordering::Less
        } else if b.1 < a.1 {
            std::cmp::Ordering::Greater
        } else if a.0 < b.0 {
            std::cmp::Ordering::Less
        } else if b.0 < a.0 {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });

    // ⚠️ An incremental run takes all of them, whatever the configured percentage.
    let percentage = if p.is_incremental { 1.0 } else { p.percentage / 100.0 };
    // ⛔ Rounded UP, so a non-empty candidate list always marks at least one net.
    let take = (candidates.len() as f32 * percentage).ceil() as usize;
    let marked = take.min(candidates.len());
    for (id, _) in candidates.iter().take(marked) {
        if let Some(o) = out.iter_mut().find(|o| o.net_id == *id) {
            o.is_res_aware = true;
        }
    }

    SlackUpdate { nets: out, candidates, worst, marked }
}
