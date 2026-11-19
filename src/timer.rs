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
//!   time unit is the first library's;
//! - every read is `vyges_sta::scenes` over the scene set ([`Scenes`]): `getNetSlack` is
//!   `Sta::slack(net, max)`, the worst max slack over the net's loads and EVERY scene, folded scene
//!   by scene, rise then fall, loads in pin order. One scene is the timer of a single library set.

use std::collections::{BTreeMap, HashMap};

use vyges_opendb::Db;
use vyges_sta::graph::{NetParasitics, NetlistRef, SdcEnv};
use vyges_sta::search::Search;
use vyges_sta::incr::IncTimer;
use vyges_sta::liberty::Library;
use vyges_sta::netlist::{Conn, Net, Netlist, PortDir};
use vyges_sta::parasitics::Network;
use vyges_sta::sdc::{sta_to_user, user_to_sta, Clock, PortDelay, Sdc};
use vyges_sta::scenes::{SceneConstraints, SceneTimers};

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
    /// The constraints already in the timer's form, when the caller has them (the resizer replays
    /// a design's whole SDC): read in place of `constraints`.
    pub sdc: Option<Sdc>,
}

/// The scene set the timer reads: `define_corners` with each corner's libraries, and the mode's
/// constraints a `read_sdc` bound to the netlist. Default: one scene, [`Timing::libs`], timed under
/// [`Timing`]'s own constraints.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Scenes {
    /// Each scene's libraries in read order (`read_liberty -corner c`: that scene's; without
    /// `-corner`: every scene's), the scenes in `define_corners` order. Empty: one scene.
    pub libs: Vec<Vec<Library>>,
    /// The scene of the first library read: the LINK libraries', whose cells the graph's edges are
    /// made from (`makeInstanceEdges` on the network's cell); not always scene 0.
    pub link: usize,
    /// `read_sdc`: the mode's one `Sdc` and each scene's environment, read against the netlist when
    /// the command ran (`vyges_sta::scenes::scene_constraints`). Read in place of [`Timing`]'s.
    pub constraints: Option<SceneConstraints>,
}

impl Scenes {
    /// Each scene's libraries; one scene, `timing.libs`, without corners.
    pub fn slices<'a>(&'a self, timing: &'a Timing) -> Vec<&'a [Library]> {
        if self.libs.is_empty() {
            vec![&timing.libs[..]]
        } else {
            self.libs.iter().map(|l| &l[..]).collect()
        }
    }
}

impl Scenes {
    /// Each scene's libraries put in the order of the link scene's libraries they correspond to,
    /// ONLY when they are a permutation of whole library files: every one corresponds to exactly
    /// one link library (the same cells, each with the same pins), and no cell name is in two of
    /// the scene's libraries. Otherwise the scene is left as read (the timer then builds its graph
    /// on its own) and the reason is returned.
    ///
    /// Rule (`LibertyLibrary::makeSceneMap`): a scene finds its cell for a link cell by NAME among
    /// its libraries, so with unique names the order its libraries were read in is not observable;
    /// with a name in two libraries it is (one definition wins by read order). The scene graphs
    /// share the link graph's vertices, which name a cell's library by its POSITION
    /// (`vyges_sta::scenes::scene_views`), so a scene read in another order is re-ordered here.
    /// Witness: CoralNPU's ASAP7 `bc` corner reads SIMPLE before SEQ, `wc` (the link) SEQ first.
    /// The same rule as the resizer's `Libs::align_scene_order`.
    pub fn align_order(&mut self) -> Vec<String> {
        let mut why = Vec::new();
        let link = self.link;
        if link >= self.libs.len() {
            return why;
        }
        let same = |a: &Library, b: &Library| {
            a.cells.len() == b.cells.len() && a.cells.iter().all(|(n, c)| b.cells.get(n).is_some_and(|d| c.ports.len() == d.ports.len() && c.ports.iter().all(|p| d.port(&p.name).is_some())))
        };
        let n = self.libs[link].len();
        for k in 0..self.libs.len() {
            if k == link {
                continue;
            }
            if self.libs[k].len() != n {
                why.push(format!("scene {k}: {} libraries, the link {n}", self.libs[k].len()));
                continue;
            }
            let at: Option<Vec<usize>> = self.libs[k].iter().map(|l| self.libs[link].iter().position(|x| same(l, x))).collect();
            let Some(at) = at else {
                why.push(format!("scene {k}: a library matching no link library (by its cells and their pins)"));
                continue;
            };
            let mut sorted = at.clone();
            sorted.sort_unstable();
            if sorted.iter().enumerate().any(|(i, &p)| i != p) {
                why.push(format!("scene {k}: its libraries are not a permutation of the link's"));
                continue;
            }
            if at.iter().enumerate().all(|(i, &p)| i == p) {
                continue;
            }
            let mut names = std::collections::HashSet::new();
            if let Some(d) = self.libs[k].iter().flat_map(|l| l.cells.keys()).find(|c| !names.insert(c.as_str())) {
                why.push(format!("scene {k}: {d} is in two of its libraries (read order observable)"));
                continue;
            }
            let mut libs: Vec<(usize, Library)> = at.into_iter().zip(std::mem::take(&mut self.libs[k])).collect();
            libs.sort_by_key(|(p, _)| *p);
            self.libs[k] = libs.into_iter().map(|(_, l)| l).collect();
        }
        why
    }
}

/// `dbNetwork::findConstantNets`: every net of signal type GROUND (logic zero) or POWER (logic
/// one), in block order, that the timing netlist holds ([`netlist`] leaves a SPECIAL supply net
/// out: it connects no timing pin). The reference's simulation holds every pin of such a net, so a
/// signal terminal on a special supply net is refused: it would be held there and is unconnected
/// here.
pub fn constant_nets(db: &Db) -> Result<Vec<(String, bool)>, String> {
    let mut out = Vec::new();
    for net in db.net_names() {
        let value = match db.net_get_sig_type(&net).as_str() {
            "GROUND" => false,
            "POWER" => true,
            _ => continue,
        };
        if db.net_is_special(&net) {
            for iterm in db.net_iterms(&net) {
                let (inst, term) = iterm.rsplit_once('/').unwrap_or((&iterm, ""));
                if !matches!(db.iterm_get_sig_type(inst, term).as_str(), "POWER" | "GROUND") {
                    return Err(format!("{iterm}: a signal terminal on the special supply net {net} (held constant by the reference) is not modelled"));
                }
            }
            continue;
        }
        out.push((net, value));
    }
    Ok(out)
}

/// The netlist as the logic simulation and the constraint reader take it (`load::Design`): every
/// instance whose cell a scene's libraries hold, each with only the pins its liberty cell lists;
/// nets and ports as they are, names unchanged. Rule (`Sim::evalInstance` reads
/// `network_->libertyCell(inst)` and does nothing without one): a physical-only instance (a tap or
/// filler cell: no liberty cell) holds and propagates no value, and a pin the cell does not list
/// (a supply pin) carries none. What they read is keyed by name, so it applies to the timer's own
/// netlist unchanged.
pub fn linked_view(nl: &Netlist, scenes: &[&[Library]]) -> Netlist {
    let cell = |name: &str| scenes.iter().find_map(|libs| libs.iter().find_map(|l| l.cells.get(name)));
    let mut at = vec![None; nl.insts.len()];
    let mut insts = Vec::new();
    for (i, x) in nl.insts.iter().enumerate() {
        if cell(&x.1).is_some() {
            at[i] = Some(insts.len());
            insts.push(x.clone());
        }
    }
    let nets = nl
        .nets
        .iter()
        .map(|n| {
            let pins = n
                .pins
                .iter()
                .filter_map(|c| match c {
                    Conn::Inst(i, port) => {
                        let k = at[*i]?;
                        cell(&nl.insts[*i].1)?.port(port)?;
                        Some(Conn::Inst(k, port.clone()))
                    }
                    Conn::Port(p) => Some(Conn::Port(*p)),
                })
                .collect();
            Net { name: n.name.clone(), pins }.into()
        })
        .collect();
    Netlist { insts, ports: nl.ports.clone(), nets }
}

/// The timer's state kept across reads, as the reference's one `dbSta` is: one incremental timer
/// per scene (the first leads, the others follow). The graphs are built at each read and import it.
#[derive(Default)]
pub struct HeldTimers {
    incs: Vec<IncTimer>,
}

impl HeldTimers {
    /// `Sta::delaysInvalidFrom(pin)` in every scene: a pin of a net estimated again.
    pub fn delays_invalid_from(&mut self, pin: &str) {
        for inc in &mut self.incs {
            inc.delays_invalid_from(pin);
        }
    }
}

/// The netlist as the database holds it.
pub fn netlist(db: &Db) -> Netlist {
    let mut nl = Netlist::default();
    let mut inst_index = HashMap::new();
    for inst in db.inst_names() {
        inst_index.insert(inst.clone(), nl.insts.len());
        let master = db.inst_master(&inst);
        nl.insts.push(vyges_sta::netlist::inst(inst, master));
    }
    let mut port_index = HashMap::new();
    for bterm in db.bterm_names() {
        // A POWER or GROUND terminal is no timing port (`dbNetwork::dbToSta(sig_type, io_type)`
        // gives it direction power / ground; the graph makes no vertex for it). Nor is one on a
        // special POWER or GROUND net, whatever its own type (`isPGSupply`: the top pin iterator
        // skips it, so it is no port at all).
        let supply = |t: String| matches!(t.as_str(), "POWER" | "GROUND");
        let net = db.bterm_get_net(&bterm);
        if supply(db.bterm_get_sig_type(&bterm)) || (!net.is_empty() && db.net_is_special(&net) && supply(db.net_get_sig_type(&net))) {
            continue;
        }
        port_index.insert(bterm.clone(), nl.ports.len());
        let dir = match db.bterm_get_io_type(&bterm).as_str() {
            "INPUT" => PortDir::Input,
            "OUTPUT" => PortDir::Output,
            _ => PortDir::Inout,
        };
        nl.ports.push((bterm, dir));
    }
    for net in db.net_names() {
        // A special POWER / GROUND net connects no timing pin (`isPGSupply`); a special SIGNAL
        // net (a pad's connection to its terminal, as a pad ring writes it) is a net like any.
        if db.net_is_special(&net) && matches!(db.net_get_sig_type(&net).as_str(), "POWER" | "GROUND") {
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
            nl.nets.push(Net { name: net, pins }.into());
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
    Ok(Some(Sdc { input_delays: delays(&c.input_delays), output_delays: delays(&c.output_delays), clocks: vec![clock], path_delays: Vec::new() }))
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
    NetParasitics { network: Network { node_caps: n.nodes.iter().map(|(_, c)| *c).collect(), resistors }, node_names, ..Default::default() }
}

/// A placement estimate's network, as the timer reads it: pin nodes by pin name, Steiner points
/// `<net>:<id>`, resistors in the order they were made.
pub fn placement_network(net: &str, g: &vyges_est::network::Parasitic) -> NetParasitics {
    use vyges_est::network::Node;
    let mut node_names = Vec::new();
    let mut node_caps = Vec::new();
    let (mut sub, mut pin) = (HashMap::new(), HashMap::new());
    for (&id, &cap) in &g.sub_nodes {
        sub.insert(id, node_names.len());
        node_names.push(format!("{net}:{id}"));
        node_caps.push(cap);
    }
    for (&key, p) in &g.pin_nodes {
        pin.insert(key, node_names.len());
        node_names.push(p.name.clone());
        node_caps.push(p.cap);
    }
    let at = |n: &Node| match n {
        Node::Sub(id) => sub[id],
        Node::Pin(k) => pin[k],
    };
    let resistors = g.resistors.iter().map(|(a, b, r)| (at(a), at(b), *r)).collect();
    NetParasitics { network: Network { node_caps, resistors }, node_names, ..Default::default() }
}

/// `getNetSlack` for every net of the netlist on route estimates: the worst max slack over its
/// load pins and every scene. Each scene reads the same networks: the estimator's per-corner wire
/// RC is one table here (a `set_layer_rc -corner` is not modelled).
pub fn net_slacks(timing: &Timing, scenes: &Scenes, nl: &Netlist, parasitics: &BTreeMap<String, crate::parasitics::Network>) -> Result<HashMap<String, f32>, String> {
    let par: HashMap<String, NetParasitics> = parasitics.iter().map(|(net, n)| (net.clone(), timer_network(net, n))).collect();
    net_slacks_on(timing, scenes, nl, &par)
}

/// The same, on networks already in the timer's form.
pub fn net_slacks_on(timing: &Timing, scenes: &Scenes, nl: &Netlist, par: &HashMap<String, NetParasitics>) -> Result<HashMap<String, f32>, String> {
    net_slacks_timed(timing, scenes, nl, par, None)
}

/// [`net_slacks`] through a timer kept across reads (`held`), as the reference's one `dbSta` is.
/// A fresh timer's first read is a full timing, the same as [`net_slacks`]. Each later read
/// updates only from what the caller invalidated since (`held.delays_invalid_from` on a pin whose
/// net was estimated again): a delay, slew or arrival that changed by no more than fuzzy equality
/// does not re-time its fanout, so a value it does not reach stays as it was.
pub fn net_slacks_incremental(timing: &Timing, scenes: &Scenes, nl: &Netlist, parasitics: &BTreeMap<String, crate::parasitics::Network>, held: &mut HeldTimers) -> Result<HashMap<String, f32>, String> {
    let par: HashMap<String, NetParasitics> = parasitics.iter().map(|(net, n)| (net.clone(), timer_network(net, n))).collect();
    net_slacks_timed(timing, scenes, nl, &par, Some(held))
}

/// The constraints of a read: `read_sdc`'s, else [`Timing`]'s own in every scene (no SDC
/// environment). `None`: no clock, nothing is timed.
fn read_constraints(timing: &Timing, scenes: &Scenes, nl: &Netlist) -> Result<Option<SceneConstraints>, String> {
    let scale = timing.libs.first().ok_or("timing needs a library")?.time_scale;
    if let Some(c) = &scenes.constraints {
        return Ok(Some(c.clone()));
    }
    let sdc = match &timing.sdc {
        Some(s) => s.clone(),
        None => match sdc(&timing.constraints, nl, scale)? {
            Some(s) => s,
            None => return Ok(None),
        },
    };
    Ok(Some(SceneConstraints { sdc, envs: vec![SdcEnv::default(); scenes.libs.len().max(1)], notes: Vec::new() }))
}

/// Every net's worst load slack over the scenes, timed in full or (with `held`) incrementally:
/// `vyges_sta::scenes::net_slacks` on the scene set's timers, the graphs built for this read.
fn net_slacks_timed(timing: &Timing, scenes: &Scenes, nl: &Netlist, par: &HashMap<String, NetParasitics>, held: Option<&mut HeldTimers>) -> Result<HashMap<String, f32>, String> {
    let Some(c) = read_constraints(timing, scenes, nl)? else {
        return Ok(nl.nets.iter().map(|n| (n.name.clone(), 1e30)).collect());
    };
    let libs = scenes.slices(timing);
    let mut t = SceneTimers::for_design(&libs, scenes.link, NetlistRef::Borrowed(nl), None, c)?;
    let Some(held) = held else {
        return full_scene_slacks(&mut t, par);
    };
    // `Sta::slack` → `findRequireds` → `findArrivals` → `findDelays`, each from the state the
    // last read left (`vyges_sta::incr`, the resizer's timer).
    let held = match Some(held) {
        Some(h) if !h.incs.is_empty() => {
            if h.incs.len() != t.incs.len() {
                return Err(format!("{} held timers for {} scenes", h.incs.len(), t.incs.len()));
            }
            t.incs = std::mem::take(&mut h.incs);
            Some(h)
        }
        h => h,
    };
    let pars: Vec<&HashMap<String, NetParasitics>> = vec![par; libs.len()];
    let slacks = vyges_sta::scenes::net_slacks(&pars, &mut t)?;
    if let Some(h) = held {
        h.incs = std::mem::take(&mut t.incs);
    }
    Ok(slacks)
}

/// A read with no timer kept across reads (FastRoute's: every net's parasitics re-estimated, so
/// the reference's next slack query recomputes every delay, arrival and required): each scene
/// timed in full, no incremental state kept. On a first read the per-scene incremental timer does
/// exactly this (a full `find_delays` / `find_arrivals` / `find_requireds` per scene), so the
/// slacks are its slacks; what it adds, a name-keyed copy of every vertex's paths per scene for
/// the next read, is what a read like this never uses. Rule (`Sta::slack(net, max)`): over the
/// net's LOAD pins in pin order, each load's slack the fuzzily least over every scene's max paths,
/// scene by scene, each in its path order; the net's the fuzzily least over its loads in that
/// order. Fuzzy least is order-sensitive, so the order is the reference's, not a per-scene minimum
/// taken first.
fn full_scene_slacks(t: &mut SceneTimers<'_>, par: &HashMap<String, NetParasitics>) -> Result<HashMap<String, f32>, String> {
    for g in t.graphs.iter_mut() {
        g.find_delays(par, None)?;
    }
    let mut searches: Vec<Search<'_, '_>> = t.graphs.iter().map(|g| Search::in_graph_order(g, &t.sdc)).collect();
    for s in searches.iter_mut() {
        s.find_arrivals()?;
        s.find_requireds()?;
    }
    let g = &t.graphs[0];
    let index = g.pin_index();
    let mut name = String::new();
    Ok(g.netlist
        .nets
        .iter()
        .map(|n| {
            let mut slack = vyges_sta::search::INF_SLACK;
            for c in &n.pins {
                name.clear();
                let Some(&v) = index.get(g.netlist.pin_name_into(c, &mut name)) else { continue };
                if g.vertices[v].is_driver {
                    continue;
                }
                let mut load = vyges_sta::search::INF_SLACK;
                for s in &searches {
                    for p in &s.paths[v] {
                        if p.tag.mm == vyges_sta::liberty::MAX {
                            let x = p.required - p.arrival;
                            if vyges_sta::fuzzy::less(x, load) {
                                load = x;
                            }
                        }
                    }
                }
                if vyges_sta::fuzzy::less(load, slack) {
                    slack = load;
                }
            }
            (n.name.clone(), slack)
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `xor2` (A->Y then B->Y, or B->Y first: `ab`) and `buf`, every arc `d` ns.
    fn lib(name: &str, ab: bool, d: f32) -> Library {
        let timing = |p: &str, x: f32| {
            format!(
                r#"timing () {{ related_pin : "{p}"; timing_sense : non_unate;
            cell_rise (scalar) {{ values ("{x}"); }} rise_transition (scalar) {{ values ("0.1"); }}
            cell_fall (scalar) {{ values ("{x}"); }} fall_transition (scalar) {{ values ("0.1"); }} }}"#
            )
        };
        let (first, second) = if ab { (timing("A", d), timing("B", d * 2.0)) } else { (timing("B", d * 2.0), timing("A", d)) };
        let text = format!(
            r#"library ({name}) {{
  time_unit : "1ns"; capacitive_load_unit (1,pf);
  cell (xor2) {{
    pin (A) {{ direction : input; capacitance : 0.001; }}
    pin (B) {{ direction : input; capacitance : 0.001; }}
    pin (Y) {{ direction : output; function : "A^B"; {first} {second} }}
  }}
}}"#
        );
        Library::read(&vyges_sta::liberty_parse::parse(&text).unwrap()).unwrap()
    }

    /// Ports a, b into x1 (xor2); x1/Y to port y; a clock port clk on no net.
    fn netlist() -> Netlist {
        let pin = |p: &str| Conn::Inst(0, p.into());
        Netlist {
            insts: vec![vyges_sta::netlist::inst("x1", "xor2")],
            ports: vec![("a".into(), PortDir::Input), ("b".into(), PortDir::Input), ("y".into(), PortDir::Output), ("clk".into(), PortDir::Input)],
            nets: vec![
                Net { name: "a".into(), pins: vec![pin("A"), Conn::Port(0)] }.into(),
                Net { name: "b".into(), pins: vec![pin("B"), Conn::Port(1)] }.into(),
                Net { name: "y".into(), pins: vec![pin("Y"), Conn::Port(2)] }.into(),
            ],
        }
    }

    /// A 2 ns clock on clk, 0.1 ns input delays on a and b and output delay on y; `libs` read in order.
    fn timing(libs: Vec<Library>) -> Timing {
        let constraints = Constraints {
            clock: Some(("clk".into(), 2.0, vec!["clk".into()], None)),
            propagated: false,
            input_delays: vec![(UserValue::Literal(0.1), PortSet::Named(vec!["a".into(), "b".into()]))],
            output_delays: vec![(UserValue::Literal(0.1), PortSet::Named(vec!["y".into()]))],
        };
        Timing { libs, constraints, sdc: None }
    }

    fn scenes(libs: Vec<Vec<Library>>) -> Scenes {
        Scenes { libs, link: 0, constraints: None }
    }

    /// Net y's wire as an RC network of `cap` farads at each end.
    fn wire(cap: f32) -> HashMap<String, NetParasitics> {
        let net = NetParasitics { node_names: vec!["x1/Y".into(), "y".into()], network: Network { node_caps: vec![cap, cap], resistors: vec![(0, 1, 100.0)] }, ..Default::default() };
        [("y".to_string(), net)].into()
    }

    /// Rule (`getNetSlack` = `Sta::slack(net, max)`: the least over the net's loads, each the least
    /// over ONE path array holding every scene's paths): with `define_corners` the router reads the
    /// worst slack over the scenes, not the first scene's.
    #[test]
    fn a_net_slack_is_the_worst_over_every_scene() {
        let (slow, fast) = (|| lib("ss", true, 1.5), || lib("ff", false, 0.5));
        let nl = netlist();
        let all = net_slacks_on(&timing(vec![slow(), fast()]), &scenes(vec![vec![slow()], vec![fast()]]), &nl, &HashMap::new()).unwrap();
        let each: Vec<HashMap<String, f32>> = [slow(), fast()].into_iter().map(|l| net_slacks_on(&timing(vec![l]), &Scenes::default(), &nl, &HashMap::new()).unwrap()).collect();
        for (net, s) in &all {
            assert_eq!(*s, each[0][net].min(each[1][net]), "{net}");
        }
        assert!(all["y"] < 1.0, "net y is timed: {all:?}");
        assert_ne!(all["y"], each[1]["y"], "the slow scene decides, not the first-read fast one");
        // the same scenes with the fast one first: the fold does not depend on the scene order
        let swapped = net_slacks_on(&timing(vec![fast(), slow()]), &scenes(vec![vec![fast()], vec![slow()]]), &nl, &HashMap::new()).unwrap();
        assert_eq!(swapped, all);
    }

    /// Rule (`delaysInvalidFromFanin` then `Sta::slack`): a read through held timers updates from
    /// what was invalidated since, in every scene, and equals a full timing of the same networks.
    #[test]
    fn a_held_read_after_invalidation_equals_a_full_one_in_every_scene() {
        let (slow, fast) = (|| lib("ss", true, 1.5), || lib("ff", false, 0.5));
        let (nl, t, sc) = (netlist(), timing(vec![slow(), fast()]), scenes(vec![vec![slow()], vec![fast()]]));
        let par = wire(0.5e-12);
        let full = net_slacks_on(&t, &sc, &nl, &par).unwrap();
        let mut held = HeldTimers::default();
        net_slacks_timed(&t, &sc, &nl, &HashMap::new(), Some(&mut held)).unwrap();
        assert_eq!(held.incs.len(), 2, "one timer per scene, held");
        let mut stale = HeldTimers::default();
        net_slacks_timed(&t, &sc, &nl, &HashMap::new(), Some(&mut stale)).unwrap();
        for pin in ["x1/Y", "y"] {
            held.delays_invalid_from(pin);
        }
        assert_eq!(net_slacks_timed(&t, &sc, &nl, &par, Some(&mut held)).unwrap(), full);
        let not_invalidated = net_slacks_timed(&t, &sc, &nl, &par, Some(&mut stale)).unwrap();
        assert_ne!(not_invalidated["y"], full["y"], "without the invalidation the read is stale (the test can fail)");
    }

    /// Rule (an unheld read times every scene in full, as the per-scene timer's first read does):
    /// it equals that first read in every scene, slack for slack and bit for bit, here with a load
    /// whose two scenes' slacks are distinct and with the wire parasitics on.
    #[test]
    fn an_unheld_read_equals_the_scene_timers_first_read() {
        let (slow, fast) = (|| lib("ss", true, 1.5), || lib("ff", false, 0.5));
        let (nl, t, sc) = (netlist(), timing(vec![slow(), fast()]), scenes(vec![vec![slow()], vec![fast()]]));
        let par = wire(0.5e-12);
        let unheld = net_slacks_on(&t, &sc, &nl, &par).unwrap();
        let first = net_slacks_timed(&t, &sc, &nl, &par, Some(&mut HeldTimers::default())).unwrap();
        assert_eq!(unheld.len(), first.len());
        for (net, s) in &unheld {
            assert_eq!(s.to_bits(), first[net].to_bits(), "{net}");
        }
    }

    /// Rule (`Sim::evalInstance` does nothing for an instance with no liberty cell): the view the
    /// constraint reader takes keeps only linked instances and their liberty pins, indices remapped,
    /// names, nets and ports unchanged.
    #[test]
    fn the_linked_view_drops_physical_only_instances_and_supply_pins() {
        let mut nl = netlist();
        nl.insts.insert(0, vyges_sta::netlist::inst("tap0", "TAPCELL"));
        for n in &mut nl.nets {
            let pins = std::sync::Arc::make_mut(n).pins.iter().map(|c| match c {
                Conn::Inst(i, p) => Conn::Inst(i + 1, p.clone()),
                c => c.clone(),
            });
            std::sync::Arc::make_mut(n).pins = pins.collect();
        }
        nl.nets.push(Net { name: "VDD".into(), pins: vec![Conn::Inst(0, "VDD".into()), Conn::Inst(1, "VDD".into())] }.into());
        let ss = [lib("ss", true, 1.5)];
        let v = linked_view(&nl, &[&ss]);
        assert_eq!(v.insts.iter().map(|x| x.0.as_str()).collect::<Vec<_>>(), ["x1"]);
        assert_eq!(v.nets.iter().map(|n| n.name.as_str()).collect::<Vec<_>>(), ["a", "b", "y", "VDD"]);
        assert_eq!(v.nets[2].pins, vec![Conn::Inst(0, "Y".into()), Conn::Port(2)]);
        assert!(v.nets[3].pins.is_empty(), "no liberty pin on the supply net");
        assert_eq!(linked_view(&netlist(), &[&ss]), netlist(), "a linked netlist is its own view");
    }

    /// Rule (`makeSceneMap` finds a scene cell by NAME): a scene that read the link's library files
    /// in another order is put in the link order; one that is no permutation of them, or names a
    /// cell twice, is left as read with the reason.
    #[test]
    fn a_scene_read_in_another_order_is_put_in_the_link_order() {
        let named = |cells: &[&str]| {
            let body: String = cells.iter().map(|c| format!("cell ({c}) {{ pin (A) {{ direction : input; capacitance : 0.001; }} }}\n")).collect();
            Library::read(&vyges_sta::liberty_parse::parse(&format!("library (l) {{ time_unit : \"1ns\"; {body} }}")).unwrap()).unwrap()
        };
        let (a, b, c) = (|| named(&["A1", "A2"]), || named(&["S1"]), || named(&["F1"]));
        let mut sc = Scenes { libs: vec![vec![a(), b(), c()], vec![b(), a(), c()], vec![c(), named(&["X"]), a()]], link: 0, constraints: None };
        assert_eq!(sc.align_order().len(), 1, "scene 2 refused, scene 1 aligned");
        let order = |s: &Scenes, k: usize| s.libs[k].iter().map(|l| l.cells.keys().min().unwrap().clone()).collect::<Vec<_>>();
        assert_eq!(order(&sc, 1), order(&sc, 0), "in the link order");
        assert_eq!(order(&sc, 2), ["F1", "X", "A1"], "no permutation of the link: as read");
        let mut dup = Scenes { libs: vec![vec![a(), named(&["A1"])], vec![named(&["A1"]), a()]], link: 0, constraints: None };
        assert!(dup.align_order().iter().any(|w| w.contains("A1 is in two")));
        assert_eq!(dup.libs[1][0].cells.len(), 1, "a duplicate name: as read");
        // the link need not be scene 0
        let mut l1 = Scenes { libs: vec![vec![b(), a()], vec![a(), b()]], link: 1, constraints: None };
        assert!(l1.align_order().is_empty());
        assert_eq!(order(&l1, 0), order(&l1, 1));
    }
}
