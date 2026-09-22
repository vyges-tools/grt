// SPDX-License-Identifier: Apache-2.0
//! What global routing asks of a Liberty library: per cell, whether it is a pad and which of its
//! ports are REGISTER CLOCKS — the two facts `isClkTerm` reads to tell a leaf clock net from one
//! above the leaves.
//!
//! The timer marks a port a register clock (`LibertyPort::isRegClk`) when it is the FROM port of an
//! arc whose role is register clock-to-Q or latch enable-to-Q. Those roles come from one builder
//! rule (`makeRegLatchArcs`), applied to every `rising_edge` / `falling_edge` timing group:
//!
//! 1. for each port the TO pin's `function` names, find the sequential that port is an output of;
//! 2. the first such sequential whose clock expression names the FROM port → clock-to-Q (a register)
//!    or enable-to-Q (a latch) — a register clock either way;
//! 3. a latch whose data names it → D-to-Q; a sequential whose clear or preset names it → set/clear —
//!    NOT a register clock;
//! 4. no function, or no sequential decides → inferred clock-to-Q: a register clock.
//!
//! ⛔ Step 1's ports are a `std::set<LibertyPort*>` — POINTER order. When a function names outputs
//! of two sequentials that decide differently the reference's answer is allocation order, so that
//! is refused rather than guessed.
//!
//! ⚠️ Refused, not modelled: bus and bundle pins, `ff_bank` / `latch_bank` (a sequential per bit).

use std::collections::{BTreeMap, BTreeSet};

/// A Liberty group: `kind (args) { attrs; groups }`. Complex attributes (`values (…);`) are dropped.
#[derive(Debug, Default)]
struct Group {
    kind: String,
    args: Vec<String>,
    attrs: Vec<(String, String)>,
    groups: Vec<Group>,
}

impl Group {
    fn attr(&self, key: &str) -> Option<&str> {
        self.attrs.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }
    fn children<'a>(&'a self, kind: &'a str) -> impl Iterator<Item = &'a Group> + 'a {
        self.groups.iter().filter(move |g| g.kind == kind)
    }
}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Word(String),
    Punct(u8),
}

fn lex(text: &str) -> Vec<Tok> {
    let b = text.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c.is_ascii_whitespace() {
            i += 1;
        } else if c == b'\\' {
            i += 1; // a line continuation
        } else if c == b'/' && b.get(i + 1) == Some(&b'*') {
            i = text[i + 2..].find("*/").map_or(b.len(), |k| i + 2 + k + 2);
        } else if c == b'"' {
            let end = text[i + 1..].find('"').map_or(b.len(), |k| i + 1 + k);
            out.push(Tok::Word(text[i + 1..end].replace("\\\n", "").replace('\\', "")));
            i = end + 1;
        } else if b"(){}:;,".contains(&c) {
            out.push(Tok::Punct(c));
            i += 1;
        } else {
            let start = i;
            while i < b.len() && !b[i].is_ascii_whitespace() && !b"(){}:;,\"\\".contains(&b[i]) {
                i += 1;
            }
            out.push(Tok::Word(text[start..i].to_string()));
        }
    }
    out
}

fn parse_body(t: &[Tok], mut i: usize, g: &mut Group) -> Result<usize, String> {
    while i < t.len() {
        let name = match &t[i] {
            Tok::Punct(b'}') => return Ok(i + 1),
            Tok::Punct(b';') => {
                i += 1;
                continue;
            }
            Tok::Word(w) => w.clone(),
            Tok::Punct(p) => return Err(format!("liberty: unexpected '{}'", *p as char)),
        };
        i += 1;
        match t.get(i) {
            Some(Tok::Punct(b':')) => {
                i += 1;
                let mut v = Vec::new();
                while let Some(Tok::Word(w)) = t.get(i) {
                    v.push(w.clone());
                    i += 1;
                }
                g.attrs.push((name, v.join(" ")));
            }
            Some(Tok::Punct(b'(')) => {
                i += 1;
                let mut args = Vec::new();
                let mut depth = 1;
                while i < t.len() && depth > 0 {
                    match &t[i] {
                        Tok::Punct(b'(') => depth += 1,
                        Tok::Punct(b')') => depth -= 1,
                        Tok::Word(w) if depth == 1 => args.push(w.clone()),
                        _ => {}
                    }
                    i += 1;
                }
                if t.get(i) == Some(&Tok::Punct(b'{')) {
                    let mut child = Group { kind: name, args, ..Group::default() };
                    i = parse_body(t, i + 1, &mut child)?;
                    g.groups.push(child);
                }
            }
            _ => return Err(format!("liberty: '{name}' is neither an attribute nor a group")),
        }
    }
    Ok(i)
}

/// The identifiers an expression names (`"!(A & B_N)"` → `A`, `B_N`).
fn idents(expr: &str) -> BTreeSet<String> {
    expr.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_' || c == '[' || c == ']'))
        .filter(|w| !w.is_empty() && !w.chars().next().is_some_and(|c| c.is_ascii_digit()))
        .map(str::to_string)
        .collect()
}

/// One `ff` / `latch` group.
#[derive(Debug)]
struct Seq {
    is_register: bool,
    clock: BTreeSet<String>,
    data: BTreeSet<String>,
    clear_preset: BTreeSet<String>,
}

/// A cell's two facts.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CellClock {
    /// `is_pad` or `pad_cell` true.
    pub is_pad: bool,
    /// Every port, and whether it is a register clock.
    pub ports: BTreeMap<String, bool>,
}

/// The register-clock rule over one arc (see the module). `None` when pointer order would decide.
fn arc_is_reg_clk(function: Option<&str>, from: &str, seq_of: &BTreeMap<String, usize>, seqs: &[Seq]) -> Option<bool> {
    let Some(f) = function else { return Some(true) };
    let mut verdicts = BTreeMap::new();
    for port in idents(f) {
        let Some(&k) = seq_of.get(&port) else { continue };
        let s = &seqs[k];
        let v = if s.clock.contains(from) {
            Some(true)
        } else if (!s.is_register && s.data.contains(from)) || s.clear_preset.contains(from) {
            Some(false)
        } else {
            None
        };
        if let Some(v) = v {
            verdicts.insert(k, v);
        }
    }
    let mut vs = verdicts.values();
    match vs.next() {
        None => Some(true),
        Some(&v) if vs.all(|&w| w == v) => Some(v),
        Some(_) => None,
    }
}

fn read_cell(cell: &Group) -> Result<CellClock, String> {
    let name = cell.args.first().cloned().unwrap_or_default();
    for kind in ["bus", "bundle", "ff_bank", "latch_bank"] {
        if cell.children(kind).next().is_some() {
            return Err(format!("liberty cell {name}: a {kind} group is not modelled"));
        }
    }
    let truthy = |k: &str| cell.attr(k).is_some_and(|v| v.eq_ignore_ascii_case("true"));
    let mut seqs = Vec::new();
    let mut seq_of = BTreeMap::new();
    // makeSequentials: ff groups, then latch groups; a later output mapping overwrites.
    for (kind, is_register, clk, data) in [("ff", true, "clocked_on", "next_state"), ("latch", false, "enable", "data_in")] {
        for g in cell.children(kind) {
            let set = |k: &str| g.attr(k).map(idents).unwrap_or_default();
            let mut clear_preset = set("clear");
            clear_preset.extend(set("preset"));
            seqs.push(Seq { is_register, clock: set(clk), data: set(data), clear_preset });
            for out in &g.args {
                seq_of.insert(out.clone(), seqs.len() - 1);
            }
        }
    }
    let mut ports = BTreeMap::new();
    for pin in cell.children("pin") {
        for p in &pin.args {
            ports.entry(p.clone()).or_insert(false);
        }
    }
    for pin in cell.children("pin") {
        for timing in pin.children("timing") {
            if !matches!(timing.attr("timing_type"), Some("rising_edge" | "falling_edge")) {
                continue;
            }
            for from in timing.attr("related_pin").unwrap_or("").split_whitespace() {
                let v = arc_is_reg_clk(pin.attr("function"), from, &seq_of, &seqs)
                    .ok_or_else(|| format!("liberty cell {name}: pin {from} — two sequentials decide differently (pointer order)"))?;
                if v {
                    ports.insert(from.to_string(), true);
                }
            }
        }
    }
    Ok(CellClock { is_pad: truthy("is_pad") || truthy("pad_cell"), ports })
}

/// The cells of every library read, by name. ⛔ A cell in two libraries resolves to the FIRST
/// library read (the network's cell lookup walks the libraries in read order).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LibertyClocks {
    pub cells: BTreeMap<String, CellClock>,
}

impl LibertyClocks {
    /// Read one library's text and add its cells.
    pub fn read(&mut self, text: &str) -> Result<(), String> {
        let toks = lex(text);
        let mut top = Group::default();
        parse_body(&toks, 0, &mut top)?;
        for lib in top.children("library") {
            for cell in lib.children("cell") {
                let name = cell.args.first().cloned().unwrap_or_default();
                if !self.cells.contains_key(&name) {
                    let c = read_cell(cell)?;
                    self.cells.insert(name, c);
                }
            }
        }
        Ok(())
    }

    /// `isClkTerm`'s inputs for one instance terminal: whether it has a liberty port, whether that
    /// port is a register clock, and whether its cell is a pad.
    pub fn iterm_facts(&self, master: &str, mterm: &str) -> crate::init::ITermClockFacts {
        let cell = self.cells.get(master);
        let port = cell.and_then(|c| c.ports.get(mterm));
        crate::init::ITermClockFacts { has_liberty_port: port.is_some(), is_reg_clk: port.copied().unwrap_or(false), cell_is_pad: cell.is_some_and(|c| c.is_pad) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(body: &str) -> CellClock {
        let mut l = LibertyClocks::default();
        l.read(&format!("library (t) {{ cell (c) {{ {body} }} }}")).expect("parse");
        l.cells["c"].clone()
    }

    // makeRegLatchArcs: the edge arc's FROM port is named by the ff's clocked_on → clock-to-Q.
    #[test]
    fn a_flop_clock_is_a_register_clock() {
        let c = cell(r#"ff ("IQ","IQ_N") { clocked_on : "CLK"; next_state : "D"; clear : "!RESET_B"; }
            pin (CLK) { direction : input; clock : true; }
            pin (D) { direction : input; }
            pin (RESET_B) { direction : input; }
            pin (Q) { direction : output; function : "IQ";
              timing () { related_pin : "CLK"; timing_type : rising_edge; cell_rise (t) { values ("1, 2"); } }
              timing () { related_pin : "RESET_B"; timing_type : clear; } }"#);
        assert_eq!(c.ports, BTreeMap::from([("CLK".into(), true), ("D".into(), false), ("Q".into(), false), ("RESET_B".into(), false)]));
    }

    // A latch's enable arc is enable-to-Q — also a register clock (`isRegClk` covers both roles).
    #[test]
    fn a_latch_enable_is_a_register_clock_and_its_data_is_not() {
        let c = cell(r#"latch ("IQ","IQ_N") { enable : "!GATE_N"; data_in : "D"; }
            pin (GATE_N) { direction : input; } pin (D) { direction : input; }
            pin (Q) { function : "IQ";
              timing () { related_pin : "GATE_N"; timing_type : falling_edge; }
              timing () { related_pin : "D"; timing_type : rising_edge; } }"#);
        assert_eq!((c.ports["GATE_N"], c.ports["D"]), (true, false));
    }

    // An edge arc whose FROM port is the sequential's clear is set/clear, not clock-to-Q.
    #[test]
    fn an_edge_arc_from_a_clear_pin_is_not_a_register_clock() {
        let c = cell(r#"ff ("IQ","IQ_N") { clocked_on : "CLK"; next_state : "D"; clear : "!R"; }
            pin (R) { direction : input; }
            pin (Q) { function : "IQ"; timing () { related_pin : "R"; timing_type : rising_edge; } }"#);
        assert!(!c.ports["R"]);
    }

    // No function, or a function naming no sequential (a statetable cell): inferred clock-to-Q.
    #[test]
    fn an_edge_arc_with_no_sequential_is_inferred_clock_to_q() {
        let c = cell(r#"statetable ("CLK GATE","M0") { table : "L L : - : L"; }
            pin (GCLK) { function : "(CLK & M0)"; timing () { related_pin : "CLK"; timing_type : rising_edge; } }
            pin (CLK) { direction : input; } pin (GATE) { direction : input; }"#);
        assert_eq!((c.ports["CLK"], c.ports["GATE"]), (true, false));
    }

    // Only rising_edge / falling_edge arcs make roles here: a combinational or check arc does not.
    #[test]
    fn combinational_and_check_arcs_make_no_register_clock() {
        let c = cell(r#"pin (A) { direction : input; timing () { related_pin : "CLK"; timing_type : setup_rising; } }
            pin (Y) { function : "!A"; timing () { related_pin : "A"; timing_sense : negative_unate; } }
            pin (CLK) { clock : true; }"#);
        assert!(c.ports.values().all(|&v| !v), "clock : true alone does not make a register clock");
    }

    // A test_cell's pins are a separate cell to the timer.
    #[test]
    fn a_test_cell_is_not_read_as_the_cell() {
        let c = cell(r#"pin (Q) { function : "Y"; }
            test_cell () { pin (TCK) { timing () { related_pin : "TCK"; timing_type : rising_edge; } } }"#);
        assert!(!c.ports.contains_key("TCK"));
    }

    #[test]
    fn two_sequentials_that_disagree_are_refused() {
        let mut l = LibertyClocks::default();
        let e = l.read(r#"library (t) { cell (c) {
            ff ("IQ","IQ_N") { clocked_on : "CLK"; next_state : "D"; }
            latch ("L","L_N") { enable : "E"; data_in : "CLK"; }
            pin (Q) { function : "IQ & L"; timing () { related_pin : "CLK"; timing_type : rising_edge; } } } }"#);
        assert!(e.unwrap_err().contains("pointer order"));
    }

    #[test]
    fn a_bus_is_refused() {
        let mut l = LibertyClocks::default();
        assert!(l.read("library (t) { cell (c) { bus (D) { bus_type : b; } } }").unwrap_err().contains("bus"));
    }

    // isClkTerm: a terminal with no liberty port is never a clock terminal; a pad cell's is.
    #[test]
    fn iterm_facts_follow_the_lookup() {
        let mut l = LibertyClocks::default();
        l.read(r#"library (t) { cell (p) { pad_cell : true; pin (PAD) { } } }"#).expect("parse");
        let f = l.iterm_facts("p", "PAD");
        assert!(f.has_liberty_port && f.cell_is_pad && !f.is_reg_clk);
        assert!(!l.iterm_facts("p", "NOPE").has_liberty_port);
        assert!(!l.iterm_facts("absent", "A").has_liberty_port);
    }
}
