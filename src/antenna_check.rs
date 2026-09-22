// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 2 — `ant::AntennaChecker` over the wire the builder encoded, in the
//! reference's order: [`build_layer_maps`] (`wiresToPolygonSetMap`, `avoidPinIntersection`, the
//! nodes, their links through the vias) …
//!
//! The checker's geometry is Boost.Polygon's, and only the REGION of each layer decides it — see
//! [`crate::polygon90`] for why the polygons, their order and their hole slits still come out
//! exactly as the reference's.

use std::collections::{BTreeMap, BTreeSet};

use crate::polygon90::{get_polygons, Region, R};
use crate::repair_antennas::TechLayers;
use crate::wire_codec::Shape;

/// One polygon of one layer: a `GraphNode`.
#[derive(Debug, Clone)]
pub struct GraphNode {
    pub id: usize,
    pub is_via: bool,
    /// As `polygon_90_data` lists it.
    pub pol: Vec<(i32, i32)>,
    /// Indices into the layer BELOW's nodes (for a routing layer: its cut layer's).
    pub low_adj: Vec<usize>,
    /// `saveGates`' result: the gate pins this node's metal reaches, by name (`PinTypeCmp`).
    pub gates: BTreeSet<String>,
    /// The polygon as a region, for the overlap tests.
    pub region: Region,
}

/// `LayerToGraphNodes`: per tech layer (by index — the `PtrMap`'s order is the layer id's, which is
/// tech order), its nodes in polygon order.
pub type LayerNodes = BTreeMap<usize, Vec<GraphNode>>;

/// `buildLayerMaps`, as far as the nodes and their links (`saveGates` is the next step).
///
/// Per layer the region is every decoded segment box and every via box on it, less every
/// ROUTING-layer pin box of the net's instance terminals (`avoidPinIntersection`) — a pin cuts the
/// metal over it, so charge reaching a gate is counted on the pieces either side.
///
/// ⚠️ `pins` are the net's instance terminals' routing boxes, placed, by tech layer.
pub fn build_layer_maps(shapes: &[Shape], pins: &[(usize, R)], tech: &TechLayers) -> Result<LayerNodes, String> {
    let mut add: BTreeMap<usize, Vec<R>> = BTreeMap::new();
    let mut sub: BTreeMap<usize, Vec<R>> = BTreeMap::new();
    for sh in shapes {
        match sh {
            Shape::Segment { level, rect } => {
                let t = tech.find_routing_layer(*level).ok_or_else(|| format!("no routing level {level}"))?;
                add.entry(t).or_default().push((rect.x_min, rect.y_min, rect.x_max, rect.y_max));
            }
            Shape::Via { boxes, .. } => {
                for (t, b) in boxes {
                    add.entry(*t).or_default().push((b.x_min, b.y_min, b.x_max, b.y_max));
                }
            }
        }
    }
    for &(t, r) in pins {
        sub.entry(t).or_default().push(r);
        add.entry(t).or_default();
    }
    // Nodes: layers in tech order, each layer's polygons in formation order, ids running on.
    let mut nodes: LayerNodes = BTreeMap::new();
    let mut regions: BTreeMap<usize, Vec<Region>> = BTreeMap::new();
    let mut id = 0;
    for (&t, rects) in &add {
        let region = Region::new(rects, sub.get(&t).map(Vec::as_slice).unwrap_or(&[]));
        let is_via = tech.0[t].routing_level == 0;
        let mut list = Vec::new();
        let mut regs = Vec::new();
        for pol in get_polygons(&region) {
            let r = polygon_region(&pol);
            regs.push(r.clone());
            list.push(GraphNode { id, is_via, pol, low_adj: Vec::new(), gates: BTreeSet::new(), region: r });
            id += 1;
        }
        nodes.insert(t, list);
        regions.insert(t, regs);
    }
    // Links through each cut layer: every upper node touching a via lists it; the via lists every
    // lower node touching it.
    let cut_layers: Vec<usize> = add.keys().copied().filter(|&t| tech.0[t].routing_level == 0).collect();
    for t in cut_layers {
        let (lower, upper) = (tech.0[t].lower, tech.0[t].upper);
        let vias = regions.get(&t).cloned().unwrap_or_default();
        for (via_index, via) in vias.iter().enumerate() {
            let lower_index = lower.map(|l| find_nodes_with_intersection(regions.get(&l).map(Vec::as_slice).unwrap_or(&[]), via)).unwrap_or_default();
            let upper_index = upper.map(|u| find_nodes_with_intersection(regions.get(&u).map(Vec::as_slice).unwrap_or(&[]), via)).unwrap_or_default();
            if let Some(u) = upper {
                for up in upper_index {
                    nodes.get_mut(&u).expect("a node found there")[up].low_adj.push(via_index);
                }
            }
            for low in lower_index {
                nodes.get_mut(&t).expect("the via layer")[via_index].low_adj.push(low);
            }
        }
    }
    Ok(nodes)
}

/// `findNodesWithIntersection`: the nodes whose polygon overlaps `pol` grown by 1 (a square
/// Minkowski sum, as Boost's `+= 1` is) in POSITIVE area — so metal one unit away counts, and
/// metal touching only at a corner of the grown shape does not.
pub fn find_nodes_with_intersection(nodes: &[Region], pol: &Region) -> Vec<usize> {
    let grown: Vec<R> = pol.rects().into_iter().map(|r| (r.0 - 1, r.1 - 1, r.2 + 1, r.3 + 1)).collect();
    nodes
        .iter()
        .enumerate()
        .filter(|(_, n)| n.rects().iter().any(|a| grown.iter().any(|b| a.2 > b.0 && a.0 < b.2 && a.3 > b.1 && a.1 < b.3)))
        .map(|(i, _)| i)
        .collect()
}

/// The region a polygon's points enclose (even-odd over its vertical edges — a hole's slit, two
/// coincident opposite edges, encloses nothing).
pub fn polygon_region(pol: &[(i32, i32)]) -> Region {
    let mut xs: Vec<i32> = pol.iter().map(|p| p.0).collect();
    let mut ys: Vec<i32> = pol.iter().map(|p| p.1).collect();
    xs.sort_unstable();
    xs.dedup();
    ys.sort_unstable();
    ys.dedup();
    let nx = xs.len().saturating_sub(1);
    let ny = ys.len().saturating_sub(1);
    let mut cells = vec![vec![false; ny]; nx];
    let n = pol.len();
    for k in 0..n {
        let (a, b) = (pol[k], pol[(k + 1) % n]);
        if a.0 != b.0 || a.1 == b.1 {
            continue;
        }
        let (y0, y1) = (a.1.min(b.1), a.1.max(b.1));
        let i0 = xs.binary_search(&a.0).expect("a vertex x");
        let (j0, j1) = (ys.binary_search(&y0).expect("a vertex y"), ys.binary_search(&y1).expect("a vertex y"));
        for col in &mut cells[i0..] {
            for c in &mut col[j0..j1] {
                *c = !*c;
            }
        }
    }
    Region { xs, ys, cells }
}

/// An instance terminal of the net as `saveGates` reads it.
#[derive(Debug, Clone)]
pub struct PinFacts {
    /// `"  <inst>/<mterm> (<master>)"` — the `PinType` name, which orders and identifies pins.
    pub name: String,
    /// Its ROUTING-layer boxes, placed, by tech layer, in MPin → geometry order.
    pub boxes: Vec<(usize, R)>,
}

/// `saveGates`: which gate pins each node's metal reaches, AS THE LAYERS ARE BUILT.
///
/// Each pin box finds the nodes it touches on its own layer and on the layers directly above and
/// below it (the cut layers). Then a union-find walks the tech layers upward from routing level 1:
/// at each layer it first joins every pin whose layer is at or below the routing layer beneath
/// (the pin's nodes are one piece of metal through it), then joins this layer's nodes to the nodes
/// they sit on — and only THEN stamps each node of this layer with the pins in its set.
///
/// ⛔ The stamping is per layer at that stage: a node gets the gates connected to it by the metal
/// below and at its own layer, not by metal added higher up. That is the manufacturing sequence the
/// antenna ratio models.
pub fn save_gates(nodes: &mut LayerNodes, pins: &[PinFacts], tech: &TechLayers) {
    let node_count: usize = nodes.values().map(Vec::len).sum();
    let mut pin_nbrs: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    let mut pin_polys: Vec<(i32, String)> = Vec::new();
    let empty: Vec<GraphNode> = Vec::new();
    for pin in pins {
        for &(t, r) in &pin.boxes {
            let pol = Region::new(&[r], &[]);
            pin_polys.push((tech.0[t].routing_level, pin.name.clone()));
            for layer in [Some(t), tech.0[t].upper, tech.0[t].lower].into_iter().flatten() {
                let list = nodes.get(&layer).unwrap_or(&empty);
                let regions: Vec<Region> = list.iter().map(|n| n.region.clone()).collect();
                for i in find_nodes_with_intersection(&regions, &pol) {
                    pin_nbrs.entry(pin.name.clone()).or_default().push(list[i].id);
                }
            }
        }
    }
    // Highest level first, popped from the back: lowest first. (`std::sort`; ties are pins of one
    // level, whose order no union depends on.)
    pin_polys.sort_by(|a, b| b.0.cmp(&a.0));
    let mut dsu = Dsu::new(node_count);
    let mut iter = tech.find_routing_layer(1);
    while let Some(t) = iter {
        if let Some(lower) = tech.0[t].lower {
            if tech.0[lower].routing_level != 0 {
                let layer_level = tech.0[lower].routing_level;
                while pin_polys.last().is_some_and(|p| layer_level >= p.0) {
                    let (_, pin) = pin_polys.pop().expect("checked");
                    let mut last: Option<usize> = None;
                    for &nbr in pin_nbrs.get(&pin).map(Vec::as_slice).unwrap_or(&[]) {
                        if let Some(l) = last {
                            if dsu.find(l) != dsu.find(nbr) {
                                dsu.union(l, nbr);
                            }
                        }
                        last = Some(nbr);
                    }
                }
            }
            let links: Vec<(usize, usize)> = nodes
                .get(&t)
                .unwrap_or(&empty)
                .iter()
                .flat_map(|n| n.low_adj.iter().map(move |&low| (n.id, low)))
                .map(|(u, low)| (u, nodes[&lower][low].id))
                .collect();
            for (u, v) in links {
                if dsu.find(u) != dsu.find(v) {
                    dsu.union(u, v);
                }
            }
        }
        if let Some(list) = nodes.get_mut(&t) {
            for n in list {
                for (gate, nbrs) in &pin_nbrs {
                    if nbrs.iter().any(|&nbr| dsu.find(n.id) == dsu.find(nbr)) {
                        n.gates.insert(gate.clone());
                    }
                }
            }
        }
        iter = tech.0[t].upper;
    }
}

/// A disjoint-set forest; only its partition is ever read.
struct Dsu(Vec<usize>);

impl Dsu {
    fn new(n: usize) -> Self {
        Dsu((0..n).collect())
    }
    fn find(&mut self, mut x: usize) -> usize {
        while self.0[x] != x {
            self.0[x] = self.0[self.0[x]];
            x = self.0[x];
        }
        x
    }
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        self.0[ra] = rb;
    }
}

// ---- areas and ratios ----

/// A layer's default antenna rule, as LEF states it (`dbTechLayerAntennaRule`).
#[derive(Debug, Clone, Default)]
pub struct AntennaRule {
    pub area_factor: f64,
    pub area_factor_diff_use_only: bool,
    pub side_area_factor: f64,
    pub side_area_factor_diff_use_only: bool,
    pub area_minus_diff_factor: f64,
    pub gate_plus_diff_factor: f64,
    pub gate_plus_diff_pwl: Vec<(f64, f64)>,
    pub area_diff_reduce: Vec<(f64, f64)>,
    pub par: f64,
    pub psr: f64,
    pub car: f64,
    pub csr: f64,
    pub diff_par: Vec<(f64, f64)>,
    pub diff_psr: Vec<(f64, f64)>,
    pub diff_car: Vec<(f64, f64)>,
    pub diff_csr: Vec<(f64, f64)>,
}

/// What the checker reads of each tech layer.
#[derive(Debug, Clone, Default)]
pub struct LayerAntenna {
    /// `hasDefaultAntennaRule` → `getDefaultAntennaRule`.
    pub rule: Option<AntennaRule>,
    /// `getThickness`, in database units (0 when not stated).
    pub thickness_dbu: u32,
}

/// `AntennaModel` — the factors `initAntennaRules` derives per layer. With no rule every factor
/// is 1 and the diffusion terms 0.
#[derive(Debug, Clone)]
struct AntennaModel {
    metal_factor: f64,
    diff_metal_factor: f64,
    cut_factor: f64,
    diff_cut_factor: f64,
    side_metal_factor: f64,
    diff_side_metal_factor: f64,
    minus_diff_factor: f64,
    plus_diff_factor: f64,
    gate_plus_diff: Vec<(f64, f64)>,
}

impl AntennaModel {
    /// `initAntennaRules`: a DIFF-USE-ONLY factor applies only to the diffusion-dependent terms.
    fn new(rule: Option<&AntennaRule>) -> Self {
        let mut m = AntennaModel {
            metal_factor: 1.0,
            diff_metal_factor: 1.0,
            cut_factor: 1.0,
            diff_cut_factor: 1.0,
            side_metal_factor: 1.0,
            diff_side_metal_factor: 1.0,
            minus_diff_factor: 0.0,
            plus_diff_factor: 0.0,
            gate_plus_diff: Vec::new(),
        };
        if let Some(r) = rule {
            if r.area_factor_diff_use_only {
                m.diff_metal_factor = r.area_factor;
                m.diff_cut_factor = r.area_factor;
            } else {
                m.metal_factor = r.area_factor;
                m.diff_metal_factor = r.area_factor;
                m.cut_factor = r.area_factor;
                m.diff_cut_factor = r.area_factor;
            }
            if r.side_area_factor_diff_use_only {
                m.diff_side_metal_factor = r.side_area_factor;
            } else {
                m.side_metal_factor = r.side_area_factor;
                m.diff_side_metal_factor = r.side_area_factor;
            }
            m.minus_diff_factor = r.area_minus_diff_factor;
            m.plus_diff_factor = r.gate_plus_diff_factor;
            m.gate_plus_diff = r.gate_plus_diff_pwl.clone();
        }
        m
    }
}

/// An instance terminal as the area calculation reads it.
#[derive(Debug, Clone)]
pub struct GateFacts {
    /// `dbITerm::getName()` — `inst/pin`.
    pub name: String,
    /// The `PinType` name `saveGates` stamps nodes with.
    pub pin_name: String,
    /// The terminal's database id: the checker's maps of terminals are `PtrMap`s, ordered by it.
    pub id: u32,
    /// `isValidGate`: an INPUT with gate area.
    pub is_valid: bool,
    /// `gateArea` / `diffArea`: the MAX over the model's / the terminal's entries.
    pub gate_area: f64,
    pub diff_area: f64,
    /// Its master is a CORE ANTENNACELL (a diode): left out of a CAR repair.
    pub is_antenna_cell: bool,
}

/// `NodeInfo`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NodeInfo {
    pub par: f64,
    pub psr: f64,
    pub diff_par: f64,
    pub diff_psr: f64,
    pub area: f64,
    pub side_area: f64,
    pub iterm_gate_area: f64,
    pub iterm_diff_area: f64,
    pub car: f64,
    pub csr: f64,
    pub diff_car: f64,
    pub diff_csr: f64,
    pub excess_ratio_par: f64,
    pub excess_ratio_psr: f64,
    /// Terminal indices (into the caller's gate list), valid gates only.
    pub iterms: Vec<usize>,
}

impl NodeInfo {
    /// `operator+=`: the per-layer ratios and areas accumulate; the terminal areas do not.
    fn add(&mut self, a: &NodeInfo) {
        self.par += a.par;
        self.psr += a.psr;
        self.diff_par += a.diff_par;
        self.diff_psr += a.diff_psr;
        self.area += a.area;
        self.side_area += a.side_area;
    }
}

/// `GateToLayerToNodeInfo`: by terminal id, then by tech layer.
pub type GateInfo = BTreeMap<u32, BTreeMap<usize, NodeInfo>>;

/// `calculateAreas`: per layer, the metal each valid gate is exposed to.
///
/// Nodes that share a terminal are one piece of metal (the pin bridges them) and are grouped; a
/// group counts when any node reaches a valid gate. A group's area is its nodes' areas summed IN
/// NODE ORDER (floating point — the order is behaviour), and its side area each routing node's
/// perimeter times the layer's thickness. Each terminal then adds its gate and diffusion area once
/// to every counting group it touches, terminals in id order; every valid gate gets its group's
/// record.
pub fn calculate_areas(nodes: &LayerNodes, gates: &[GateFacts], layers: &[LayerAntenna], tech: &TechLayers, dbu_per_micron: f64) -> GateInfo {
    let by_pin: BTreeMap<&str, usize> = gates.iter().enumerate().map(|(i, g)| (g.pin_name.as_str(), i)).collect();
    let mut gate_info = GateInfo::new();
    for (&t, list) in nodes {
        let routing = tech.0[t].routing_level != 0;
        let wire_thickness = if routing { f64::from(layers[t].thickness_dbu) / dbu_per_micron } else { 0.0 };
        let n = list.len();
        let mut has_gate = vec![false; n];
        let mut nodes_by_iterm: BTreeMap<u32, (usize, Vec<usize>)> = BTreeMap::new();
        for (i, node) in list.iter().enumerate() {
            for pin in &node.gates {
                let g = by_pin[pin.as_str()];
                nodes_by_iterm.entry(gates[g].id).or_insert((g, Vec::new())).1.push(i);
                if gates[g].is_valid {
                    has_gate[i] = true;
                }
            }
        }
        let mut groups = Dsu::new(n);
        for (_, ids) in nodes_by_iterm.values() {
            for &id in ids {
                let (a, b) = (groups.find(ids[0]), groups.find(id));
                if a != b {
                    groups.union(a, b);
                }
            }
        }
        let mut group_has_gate = vec![false; n];
        for i in 0..n {
            if has_gate[i] {
                let g = groups.find(i);
                group_has_gate[g] = true;
            }
        }
        let mut info_by_group = vec![NodeInfo::default(); n];
        for (i, node) in list.iter().enumerate() {
            let g = groups.find(i);
            if !group_has_gate[g] {
                continue;
            }
            let info = &mut info_by_group[g];
            let area = crate::polygon90::polygon_area(&node.pol);
            info.area += area as f64 / (dbu_per_micron * dbu_per_micron);
            if routing {
                info.side_area += (crate::polygon90::polygon_perimeter(&node.pol) as f64 * wire_thickness) / dbu_per_micron;
            }
        }
        for (g, ids) in nodes_by_iterm.values() {
            let mut iterm_groups = BTreeSet::new();
            for &id in ids {
                let grp = groups.find(id);
                if group_has_gate[grp] {
                    iterm_groups.insert(grp);
                }
            }
            for grp in iterm_groups {
                let info = &mut info_by_group[grp];
                if gates[*g].is_valid {
                    info.iterms.push(*g);
                }
                info.iterm_gate_area += gates[*g].gate_area;
                info.iterm_diff_area += gates[*g].diff_area;
            }
        }
        for (&id, (g, ids)) in &nodes_by_iterm {
            if gates[*g].is_valid {
                let grp = groups.find(ids[0]);
                gate_info.entry(id).or_default().insert(t, info_by_group[grp].clone());
            }
        }
    }
    gate_info
}

/// `getPwlFactor`: a piecewise-linear table read at `ref_value` — the first ratio for a one-point
/// table, linear within a segment, EXTRAPOLATED along the last segment's slope past the end; the
/// default for an empty table.
///
/// ⚠️ The first iteration compares point 0 with itself: its slope is 0/0 (NaN) and its range test
/// `x0 <= v < x0` is always false, so it only sets up the walk.
pub fn get_pwl_factor(pwl: &[(f64, f64)], ref_value: f64, default_value: f64) -> f64 {
    if pwl.is_empty() {
        return default_value;
    }
    if pwl.len() == 1 {
        return pwl[0].1;
    }
    let (mut i1, mut r1) = pwl[0];
    let mut slope = 1.0;
    for &(i2, r2) in pwl {
        slope = (r2 - r1) / (i2 - i1);
        if ref_value >= i1 && ref_value < i2 {
            return r1 + (ref_value - i1) * slope;
        }
        i1 = i2;
        r1 = r2;
    }
    r1 + (ref_value - i1) * slope
}

fn area_diff_reduce(layer: &LayerAntenna, diff: f64) -> f64 {
    layer.rule.as_ref().map_or(1.0, |r| get_pwl_factor(&r.area_diff_reduce, diff, 1.0))
}

fn plus_diff_protect(m: &AntennaModel, diff: f64) -> f64 {
    if m.gate_plus_diff.is_empty() {
        m.plus_diff_factor * diff
    } else {
        get_pwl_factor(&m.gate_plus_diff, diff, 0.0)
    }
}

/// `calculateWirePar`: PAR / PSR, and their diffusion-aware forms.
fn calculate_wire_par(layer: &LayerAntenna, m: &AntennaModel, info: &mut NodeInfo) {
    let reduce = area_diff_reduce(layer, info.iterm_diff_area);
    if info.iterm_diff_area != 0.0 {
        info.par = (m.diff_metal_factor * info.area) / info.iterm_gate_area;
        info.psr = (m.diff_side_metal_factor * info.side_area) / info.iterm_gate_area;
        let protect = plus_diff_protect(m, info.iterm_diff_area);
        info.diff_par = (m.diff_metal_factor * info.area * reduce - m.minus_diff_factor * info.iterm_diff_area) / (info.iterm_gate_area + protect);
        info.diff_psr = (m.diff_side_metal_factor * info.side_area * reduce - m.minus_diff_factor * info.iterm_diff_area) / (info.iterm_gate_area + protect);
    } else {
        info.par = (m.metal_factor * info.area) / info.iterm_gate_area;
        info.psr = (m.side_metal_factor * info.side_area) / info.iterm_gate_area;
        info.diff_par = (m.metal_factor * info.area * reduce) / info.iterm_gate_area;
        info.diff_psr = (m.side_metal_factor * info.side_area * reduce) / info.iterm_gate_area;
    }
}

/// `calculateViaPar`: PAR and its diffusion-aware form (a cut has no side area).
fn calculate_via_par(layer: &LayerAntenna, m: &AntennaModel, info: &mut NodeInfo) {
    let reduce = area_diff_reduce(layer, info.iterm_diff_area);
    if info.iterm_diff_area != 0.0 {
        info.par = (m.diff_cut_factor * info.area) / info.iterm_gate_area;
        let protect = plus_diff_protect(m, info.iterm_diff_area);
        info.diff_par = (m.diff_cut_factor * info.area * reduce - m.minus_diff_factor * info.iterm_diff_area) / (info.iterm_gate_area + protect);
    } else {
        info.par = (m.cut_factor * info.area) / info.iterm_gate_area;
        info.diff_par = (m.cut_factor * info.area * reduce) / info.iterm_gate_area;
    }
}

/// `calculatePAR`, over every gate and layer.
pub fn calculate_par(gate_info: &mut GateInfo, layers: &[LayerAntenna], tech: &TechLayers) {
    for per_layer in gate_info.values_mut() {
        for (&t, info) in per_layer.iter_mut() {
            let m = AntennaModel::new(layers[t].rule.as_ref());
            if tech.0[t].routing_level == 0 {
                calculate_via_par(&layers[t], &m, info);
            } else {
                calculate_wire_par(&layers[t], &m, info);
            }
        }
    }
}

/// `calculateCAR`: walking the tech layers up from routing level 1, each layer's cumulative ratios
/// are the sums of the partial ones at and below it — cut layers and routing layers kept apart.
pub fn calculate_car(gate_info: &mut GateInfo, tech: &TechLayers) {
    for per_layer in gate_info.values_mut() {
        let (mut sum_wire, mut sum_via) = (NodeInfo::default(), NodeInfo::default());
        let mut iter = tech.find_routing_layer(1);
        while let Some(t) = iter {
            if let Some(info) = per_layer.get_mut(&t) {
                let sum = if tech.0[t].routing_level == 0 { &mut sum_via } else { &mut sum_wire };
                sum.add(info);
                info.car += sum.par;
                info.csr += sum.psr;
                info.diff_car += sum.diff_par;
                info.diff_csr += sum.diff_psr;
            }
            iter = tech.0[t].upper;
        }
    }
}

/// `printf("%.17g")` — the trace's number format.
pub fn fmt_g17(v: f64) -> String {
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    if !v.is_finite() {
        return if v.is_nan() { "nan".into() } else if v > 0.0 { "inf".into() } else { "-inf".into() };
    }
    let e = format!("{v:.16e}");
    let (mant, exp) = e.split_once('e').expect("an exponent");
    let exp: i32 = exp.parse().expect("an exponent");
    let strip = |s: &str| -> String {
        if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            s.to_string()
        }
    };
    if exp < -4 || exp >= 17 {
        format!("{}e{}{:02}", strip(mant), if exp < 0 { '-' } else { '+' }, exp.abs())
    } else {
        let decimals = (16 - exp).max(0) as usize;
        strip(&format!("{v:.decimals$}"))
    }
}

// ---- the verdict ----

/// `ant::Violation`: a layer (its ROUTING LEVEL — 0 for a cut layer), the gates to protect, how
/// many diodes each needs, and by how much the worst partial ratio is exceeded.
#[derive(Debug, Clone, PartialEq)]
pub struct Violation {
    pub routing_level: i32,
    /// Indices into the caller's gate list.
    pub gates: Vec<usize>,
    pub diode_count_per_gate: i32,
    pub excess_ratio: f64,
}

/// `kMaxDiodeCountPerGate`.
const MAX_DIODE_COUNT_PER_GATE: i32 = 10;

/// `checkPAR` (or `checkPSR` with `side`): against the diffusion-dependent limit when the gate has
/// diffusion or the plain limit is absent, else against the plain limit. The margin tightens both;
/// a zero limit checks nothing. Raises the record's excess ratio.
fn check_partial(rule: &AntennaRule, info: &mut NodeInfo, ratio_margin: f32, side: bool) -> bool {
    let (plain, curve) = if side { (rule.psr, &rule.diff_psr) } else { (rule.par, &rule.diff_par) };
    let margin = 1.0 - f64::from(ratio_margin) / 100.0;
    let ratio = plain * margin;
    let pwl = get_pwl_factor(curve, info.iterm_diff_area, 0.0) * margin;
    let (value, diff_value) = if side { (info.psr, info.diff_psr) } else { (info.par, info.diff_par) };
    let excess = if side { &mut info.excess_ratio_psr } else { &mut info.excess_ratio_par };
    if info.iterm_diff_area != 0.0 || ratio == 0.0 {
        if pwl != 0.0 {
            *excess = excess.max(diff_value / pwl);
            return diff_value > pwl;
        }
    } else if ratio != 0.0 {
        *excess = excess.max(value / ratio);
        return value > ratio;
    }
    false
}

/// `checkCAR` (or `checkCSR` with `side`): the same choice of limit, no margin, no excess ratio.
fn check_cumulative(rule: &AntennaRule, info: &NodeInfo, side: bool) -> bool {
    let (plain, curve) = if side { (rule.csr, &rule.diff_csr) } else { (rule.car, &rule.diff_car) };
    let pwl = get_pwl_factor(curve, info.iterm_diff_area, 0.0);
    let (value, diff_value) = if side { (info.csr, info.diff_csr) } else { (info.car, info.diff_car) };
    if info.iterm_diff_area != 0.0 || plain == 0.0 {
        if pwl != 0.0 {
            return diff_value > pwl;
        }
    } else if plain != 0.0 {
        return value > plain;
    }
    false
}

/// `checkGates`: which gates violate on which layers, and the violations repair acts on.
///
/// ⛔ `checkRatioViolations` is `checkPAR || checkCAR` — CAR is not evaluated once PAR fails — and
/// then, on a routing layer, PSR and CSR both. It runs on the gate's OWN record, so the excess
/// ratios it raises are what the second pass copies.
///
/// ⛔ Per violating (gate, layer), once per layer for a gate: the PAR/PSR violation is reported
/// with the diodes it needs — the diffusion area grows by one diode per gate of the group until it
/// passes (at most 11 tries), less the diodes this gate already got on a lower layer — and then
/// CAR / CSR are checked on that SAME grown record: a cumulative violation adds one diode for every
/// gate of the group that is not itself a diode.
pub fn check_gates(
    gate_info: &mut GateInfo,
    gates: &[GateFacts],
    layers: &[LayerAntenna],
    tech: &TechLayers,
    diode_diff_area: Option<f64>,
    ratio_margin: f32,
) -> (i32, Vec<Violation>) {
    let by_id: BTreeMap<u32, usize> = gates.iter().enumerate().map(|(i, g)| (g.id, i)).collect();
    let mut pin_violation_count = 0;
    let mut gates_with_violations: BTreeMap<u32, BTreeSet<usize>> = BTreeMap::new();
    for (&gate, per_layer) in gate_info.iter_mut() {
        let mut pin_has_violation = false;
        for (&t, info) in per_layer.iter_mut() {
            let Some(rule) = &layers[t].rule else { continue };
            let mut v = check_partial(rule, info, ratio_margin, false) || check_cumulative(rule, info, false);
            if tech.0[t].routing_level != 0 {
                let psr = check_partial(rule, info, ratio_margin, true);
                let csr = check_cumulative(rule, info, true);
                v = v || psr || csr;
            }
            if v {
                pin_has_violation = true;
                gates_with_violations.entry(gate).or_default().insert(t);
            }
        }
        if pin_has_violation {
            pin_violation_count += 1;
        }
    }
    let mut violations = Vec::new();
    if pin_violation_count == 0 {
        return (pin_violation_count, violations);
    }
    let mut num_diodes_added: BTreeMap<u32, i32> = BTreeMap::new();
    let mut pin_added: BTreeMap<usize, BTreeSet<u32>> = BTreeMap::new();
    for (&gate, per_layer) in &gates_with_violations {
        for &t in per_layer {
            if pin_added.get(&t).is_some_and(|s| s.contains(&gate)) {
                continue;
            }
            let rule = layers[t].rule.as_ref().expect("a violation has a rule");
            let routing = tech.0[t].routing_level != 0;
            let mut info = gate_info[&gate][&t].clone();
            let group = info.iterms.clone();
            let mut diode_count = 0;
            let mut par_v = check_partial(rule, &mut info, ratio_margin, false);
            let mut psr_v = routing && check_partial(rule, &mut info, ratio_margin, true);
            let violated = par_v || psr_v;
            let excess = if violated { info.excess_ratio_par.max(info.excess_ratio_psr) } else { 1.0 };
            if let Some(diode) = diode_diff_area {
                while par_v || psr_v {
                    info.iterm_diff_area += diode * group.len() as f64;
                    diode_count += 1;
                    let m = AntennaModel::new(Some(rule));
                    if routing {
                        calculate_wire_par(&layers[t], &m, &mut info);
                    } else {
                        calculate_via_par(&layers[t], &m, &mut info);
                    }
                    par_v = check_partial(rule, &mut info, ratio_margin, false);
                    if routing {
                        psr_v = check_partial(rule, &mut info, ratio_margin, true);
                    }
                    if diode_count > MAX_DIODE_COUNT_PER_GATE {
                        break;
                    }
                }
            }
            pin_added.entry(t).or_default().insert(gate);
            let already = num_diodes_added.get(&gate).copied().unwrap_or(0);
            diode_count = (diode_count - already).max(0);
            *num_diodes_added.entry(gate).or_insert(0) += diode_count;
            if violated {
                violations.push(Violation { routing_level: tech.0[t].routing_level, gates: vec![by_id[&gate]], diode_count_per_gate: diode_count, excess_ratio: excess });
            }
            let car_v = check_cumulative(rule, &info, false);
            let csr_v = check_cumulative(rule, &info, true);
            if car_v || csr_v {
                let for_diodes = group.iter().copied().filter(|&g| !gates[g].is_antenna_cell).collect();
                violations.push(Violation { routing_level: tech.0[t].routing_level, gates: for_diodes, diode_count_per_gate: 1, excess_ratio: 1.0 });
            }
        }
    }
    (pin_violation_count, violations)
}
