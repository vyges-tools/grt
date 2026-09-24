// SPDX-License-Identifier: Apache-2.0
//! The timer global routing reads its net slacks from, run in-process at each read: the netlist
//! as the database holds it, the parasitic networks of the routes as they stand, the libraries
//! and the constraints — timed by `vyges-sta`.
//!
//! Rules:
//! - the netlist is every instance (database order) with its master, every top port with its
//!   direction, and every signal net's pins — instance pins in the net's order, then its ports.
//!   That order is the timing graph's vertex creation order, which orders CRPR tags;
//! - a net with no network (a local net, never routed) is timed lumped at its pin capacitance;
//! - constraint values arrive in user units as the command line holds them (`double`); the user
//!   time unit is the first library's.

use std::collections::{BTreeMap, HashMap};

use vyges_opendb::Db;
use vyges_sta::graph::{Graph, NetParasitics};
use vyges_sta::liberty::Library;
use vyges_sta::netlist::{Conn, Net, Netlist, PortDir};
use vyges_sta::parasitics::Network;
use vyges_sta::sdc::{sta_to_user, user_to_sta, Clock, PortDelay, Sdc};
use vyges_sta::search::Search;

/// Which top ports a port delay applies to.
#[derive(Debug, Clone, PartialEq)]
pub enum PortSet {
    Named(Vec<String>),
    /// Every input but the clock's own source ports (a delay there is not allowed).
    InputsExceptClockSources,
    Outputs,
}

/// A value for a constraint, in user units: a literal, or the clock's period read back
/// (`[get_property $clk period]`) times a factor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UserValue {
    Literal(f64),
    PeriodTimes(f64),
}

/// `create_clock -name N -period P [get_ports …]`, `set_propagated_clock`, and port delays
/// relative to the clock's rise edge.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Constraints {
    /// (name, period in user units, source ports, `-waveform {rise fall}` in user units — default
    /// `{0, period/2}`).
    pub clock: Option<(String, f64, Vec<String>, Option<[f64; 2]>)>,
    pub propagated: bool,
    pub input_delays: Vec<(UserValue, PortSet)>,
    pub output_delays: Vec<(UserValue, PortSet)>,
}

/// The timer's inputs that do not change during a run.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Timing {
    pub libs: Vec<Library>,
    pub constraints: Constraints,
}

/// The netlist as the database holds it.
pub fn netlist(db: &Db) -> Netlist {
    let mut nl = Netlist::default();
    let mut inst_index = HashMap::new();
    for inst in db.inst_names() {
        inst_index.insert(inst.clone(), nl.insts.len());
        let master = db.inst_master(&inst);
        nl.insts.push((inst, master));
    }
    let mut port_index = HashMap::new();
    for bterm in db.bterm_names() {
        port_index.insert(bterm.clone(), nl.ports.len());
        let dir = match db.bterm_get_io_type(&bterm).as_str() {
            "INPUT" => PortDir::Input,
            "OUTPUT" => PortDir::Output,
            _ => PortDir::Inout,
        };
        nl.ports.push((bterm, dir));
    }
    for net in db.net_names() {
        if db.net_is_special(&net) {
            continue;
        }
        let mut pins = Vec::new();
        for iterm in db.net_iterms(&net) {
            let (inst, term) = iterm.rsplit_once('/').unwrap_or((&iterm, ""));
            if let Some(&i) = inst_index.get(inst) {
                pins.push(Conn::Inst(i, term.to_string()));
            }
        }
        for bterm in db.net_bterms(&net) {
            if let Some(&p) = port_index.get(&bterm) {
                pins.push(Conn::Port(p));
            }
        }
        if !pins.is_empty() {
            nl.nets.push(Net { name: net, pins });
        }
    }
    nl
}

/// The constraints in seconds, the ports resolved against the netlist. `None` without a clock.
pub fn sdc(c: &Constraints, nl: &Netlist, scale: f32) -> Result<Option<Sdc>, String> {
    let Some((name, period, sources, waveform)) = &c.clock else { return Ok(None) };
    let [source] = sources.as_slice() else {
        return Err(format!("clock {name} on {} ports: one source port is modelled", sources.len()));
    };
    let mut clock = Clock::new(name, user_to_sta(*period, scale), source, c.propagated);
    if let Some([rise, fall]) = waveform {
        clock.waveform = [user_to_sta(*rise, scale), user_to_sta(*fall, scale)];
    }
    let clock = clock;
    let value = |v: UserValue| match v {
        UserValue::Literal(x) => user_to_sta(x, scale),
        UserValue::PeriodTimes(f) => user_to_sta(sta_to_user(clock.period, scale) * f, scale),
    };
    let resolve = |set: &PortSet| -> Vec<String> {
        match set {
            PortSet::Named(v) => v.clone(),
            PortSet::InputsExceptClockSources => nl.ports.iter().filter(|(n, d)| *d == PortDir::Input && !sources.contains(n)).map(|(n, _)| n.clone()).collect(),
            PortSet::Outputs => nl.ports.iter().filter(|(_, d)| *d == PortDir::Output).map(|(n, _)| n.clone()).collect(),
        }
    };
    let delays = |list: &[(UserValue, PortSet)]| -> Vec<PortDelay> { list.iter().flat_map(|(v, set)| resolve(set).into_iter().map(move |p| PortDelay::uniform(&p, value(*v)))).collect() };
    Ok(Some(Sdc { input_delays: delays(&c.input_delays), output_delays: delays(&c.output_delays), clock }))
}

/// A parasitic network from the estimator, as the timer reads it: nodes in the builder's order,
/// a pin node named by its pin, a route point `<net>:<id + 1>`.
pub fn timer_network(net: &str, n: &crate::parasitics::Network) -> NetParasitics {
    let name = |id: &crate::parasitics::NodeId| match id {
        crate::parasitics::NodeId::Pin(p) => p.clone(),
        crate::parasitics::NodeId::Point(i) => format!("{net}:{}", i + 1),
    };
    let node_names: Vec<String> = n.nodes.iter().map(|(id, _)| name(id)).collect();
    let index: HashMap<&str, usize> = node_names.iter().enumerate().map(|(i, s)| (s.as_str(), i)).collect();
    let resistors = n.resistors.iter().map(|(a, b, r)| (index[name(a).as_str()], index[name(b).as_str()], *r)).collect();
    NetParasitics { network: Network { node_caps: n.nodes.iter().map(|(_, c)| *c).collect(), resistors }, node_names }
}

/// `getNetSlack` for every net of the netlist: the worst max slack over its load pins.
pub fn net_slacks(timing: &Timing, nl: &Netlist, parasitics: &BTreeMap<String, crate::parasitics::Network>) -> Result<HashMap<String, f32>, String> {
    let scale = timing.libs.first().ok_or("timing needs a library")?.time_scale;
    let Some(sdc) = sdc(&timing.constraints, nl, scale)? else {
        return Ok(nl.nets.iter().map(|n| (n.name.clone(), 1e30)).collect());
    };
    let par: HashMap<String, NetParasitics> = parasitics.iter().map(|(net, n)| (net.clone(), timer_network(net, n))).collect();
    let mut g = Graph::build(&timing.libs, nl)?;
    g.find_delays(&par, None)?;
    let mut s = Search::in_graph_order(&g, &sdc);
    s.find_arrivals()?;
    s.find_requireds()?;
    let index: HashMap<&str, usize> = g.vertices.iter().enumerate().map(|(i, v)| (v.name.as_str(), i)).collect();
    Ok(nl
        .nets
        .iter()
        .map(|n| {
            let loads: Vec<usize> = n.pins.iter().filter_map(|c| index.get(nl.pin_name(c).as_str()).copied()).filter(|&v| !g.vertices[v].is_driver).collect();
            (n.name.clone(), s.net_slack(&loads))
        })
        .collect())
}
