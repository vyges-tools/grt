// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 2 — the checker.
//!
//! End to end, `grt-antcheck-score.py` replays the reference's instrumented checker per net: the
//! decoded shapes, every node (polygon, vertex order, links), each node's gates, every per-gate
//! per-layer record at 17 significant digits, and the violations — on 8 suite scripts, for the
//! `check_antennas` pass (no diode) and the repair pass (with one), all exact. The rules below are
//! the ones those designs never reach, each on a constructed case.

use std::collections::BTreeMap;

use vyges_grt::antenna_check::{check_gates, fmt_g17, get_pwl_factor, AntennaRule, GateFacts, GateInfo, LayerAntenna, NodeInfo};
use vyges_grt::repair_antennas::{TechLayer, TechLayers};

/// `getPwlFactor`: linear inside the table, extrapolated along the LAST segment past its end —
/// and, ⛔ golden-blind, a value BELOW the first index matches no segment, so it too is
/// extrapolated from the table's end, not its start.
#[test]
fn pwl_reads_inside_past_the_end_and_below_the_start() {
    let pwl = [(0.0, 400.0), (0.0125, 2200.0), (0.0225, 2200.0)];
    assert_eq!(get_pwl_factor(&[], 3.0, 7.0), 7.0);
    assert_eq!(get_pwl_factor(&[(5.0, 9.0)], 100.0, 7.0), 9.0);
    assert_eq!(get_pwl_factor(&pwl, 0.00625, 0.0), 400.0 + 0.00625 * ((2200.0 - 400.0) / 0.0125));
    assert_eq!(get_pwl_factor(&pwl, 1.0, 0.0), 2200.0); // flat last segment
    let rising = [(1.0, 10.0), (2.0, 20.0)];
    assert_eq!(get_pwl_factor(&rising, 3.0, 0.0), 30.0);
    assert_eq!(get_pwl_factor(&rising, 0.0, 0.0), 20.0 + (0.0 - 2.0) * 10.0); // from the END: 0.0
    assert_eq!(get_pwl_factor(&rising, -1.0, 0.0), -10.0);
}

/// The trace prints with `%.17g`; the formatter must agree with printf on every shape of number.
#[test]
fn g17_matches_printf() {
    assert_eq!(fmt_g17(0.0), "0");
    assert_eq!(fmt_g17(1.7408108108108109), "1.7408108108108109");
    assert_eq!(fmt_g17(0.0289), "0.028899999999999999");
    assert_eq!(fmt_g17(13.66022), "13.660220000000001");
    assert_eq!(fmt_g17(1.0), "1");
    assert_eq!(fmt_g17(0.222), "0.222");
    assert_eq!(fmt_g17(1e-5), "1.0000000000000001e-05");
    assert_eq!(fmt_g17(123456789012345678.0), "1.2345678901234568e+17");
    assert_eq!(fmt_g17(-2.5), "-2.5");
}

fn tech() -> TechLayers {
    // li1 (1), mcon (cut), met1 (2)
    TechLayers(vec![
        TechLayer { name: "li1".into(), routing_level: 1, is_routing: true, upper: Some(1), lower: None },
        TechLayer { name: "mcon".into(), routing_level: 0, is_routing: false, upper: Some(2), lower: Some(0) },
        TechLayer { name: "met1".into(), routing_level: 2, is_routing: true, upper: None, lower: Some(1) },
    ])
}

fn gate(id: u32, name: &str, antenna_cell: bool) -> GateFacts {
    GateFacts { name: name.into(), pin_name: format!("  {name} (x)"), id, is_valid: true, gate_area: 1.0, diff_area: 0.0, is_antenna_cell: antenna_cell }
}

fn rule(par: f64, car: f64) -> LayerAntenna {
    LayerAntenna { rule: Some(AntennaRule { area_factor: 1.0, side_area_factor: 1.0, par, car, ..AntennaRule::default() }), thickness_dbu: 0 }
}

fn record(par: f64, car: f64, iterms: Vec<usize>) -> NodeInfo {
    NodeInfo { par, car, diff_par: par, diff_car: car, area: par, iterm_gate_area: 1.0, iterms, ..NodeInfo::default() }
}

/// A PAR violation is reported with its excess ratio and the diodes it needs: the diffusion area
/// grows by one diode per gate of the group until the partial ratio passes.
#[test]
fn par_violation_counts_diodes_until_it_passes() {
    let layers = vec![LayerAntenna::default(), LayerAntenna::default(), rule(100.0, 0.0)];
    let gates = vec![gate(7, "u/A", false)];
    let mut info: GateInfo = BTreeMap::from([(7, BTreeMap::from([(2, record(250.0, 0.0, vec![0]))]))]);
    // No diode: counted, not repaired.
    let (pins, v) = check_gates(&mut info.clone(), &gates, &layers, &tech(), None, 0.0);
    assert_eq!(pins, 1);
    assert_eq!((v[0].routing_level, v[0].diode_count_per_gate, v[0].excess_ratio), (2, 0, 2.5));
    // A diode whose diffusion switches the check to the (absent) diffusion curve: one is enough.
    let (_, v) = check_gates(&mut info, &gates, &layers, &tech(), Some(0.5), 0.0);
    assert_eq!(v[0].diode_count_per_gate, 1);
}

/// ⛔ A CUMULATIVE violation adds a separate one-diode violation for every gate of the group that
/// is not itself a diode, with excess 1.0 — and with no partial violation, only that one.
#[test]
fn car_violation_protects_every_gate_but_the_diodes() {
    let layers = vec![LayerAntenna::default(), LayerAntenna::default(), rule(0.0, 100.0)];
    let gates = vec![gate(3, "u/A", false), gate(4, "d/DIODE", true), gate(5, "v/B", false)];
    let mut info: GateInfo = BTreeMap::from([(3, BTreeMap::from([(2, record(0.0, 150.0, vec![0, 1, 2]))]))]);
    let (_, v) = check_gates(&mut info, &gates, &layers, &tech(), None, 0.0);
    assert_eq!(v.len(), 1);
    assert_eq!((v[0].gates.clone(), v[0].diode_count_per_gate, v[0].excess_ratio), (vec![0, 2], 1, 1.0));
}

/// A ratio margin tightens the partial limit (as a float percentage), not the cumulative one.
#[test]
fn margin_tightens_the_partial_limit_only() {
    let layers = vec![LayerAntenna::default(), LayerAntenna::default(), rule(100.0, 100.0)];
    let gates = vec![gate(1, "u/A", false)];
    let mut info: GateInfo = BTreeMap::from([(1, BTreeMap::from([(2, record(95.0, 95.0, vec![0]))]))]);
    assert_eq!(check_gates(&mut info.clone(), &gates, &layers, &tech(), None, 0.0).0, 0);
    let (pins, v) = check_gates(&mut info, &gates, &layers, &tech(), None, 10.0);
    assert_eq!(pins, 1);
    assert_eq!(v.len(), 1); // the partial one; 95 < 100 cumulative
}

/// A violation on a CUT layer carries routing level 0 — that is what the repair reads.
#[test]
fn cut_layer_violation_is_level_zero() {
    let layers = vec![LayerAntenna::default(), rule(1.0, 0.0), LayerAntenna::default()];
    let gates = vec![gate(1, "u/A", false)];
    let mut info: GateInfo = BTreeMap::from([(1, BTreeMap::from([(1, record(2.0, 0.0, vec![0]))]))]);
    let (_, v) = check_gates(&mut info, &gates, &layers, &tech(), None, 0.0);
    assert_eq!(v[0].routing_level, 0);
}

/// ⛔ A gate violating on two layers gets its diodes once: the upper layer's count is reduced by
/// what the lower one already asked for.
#[test]
fn diodes_added_below_are_not_asked_for_again() {
    let layers = vec![LayerAntenna::default(), rule(100.0, 0.0), rule(100.0, 0.0)];
    let gates = vec![gate(1, "u/A", false)];
    let mut info: GateInfo = BTreeMap::from([(1, BTreeMap::from([(1, record(250.0, 0.0, vec![0])), (2, record(250.0, 0.0, vec![0]))]))]);
    let (_, v) = check_gates(&mut info, &gates, &layers, &tech(), Some(0.5), 0.0);
    assert_eq!(v.iter().map(|x| x.diode_count_per_gate).collect::<Vec<_>>(), vec![1, 0]);
}
