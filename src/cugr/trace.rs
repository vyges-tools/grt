// SPDX-License-Identifier: Apache-2.0
//! The model as `|`-separated records, one per line, for comparison with a reference run.
//!
//! Every container is written in the order the reference iterates it, so a comparison can tell a
//! different value from a different order. Numbers are Rust's shortest round-trip form; a reader
//! should compare them as numbers, not as text.
//!
//! | record | what |
//! | --- | --- |
//! | `init\|min\|max\|clock_nets` | once per `init` |
//! | `layer\|idx\|name\|dir=\|w=\|…` | each `MetalLayer` (`eol=-`: stored upstream, never read) |
//! | `via\|i\|name\|lo=dx,dy\|up=dx,dy` | the via chosen per layer pair |
//! | `design\|…`, `gridlines\|dim\|n\|…`, `obs\|i\|layer\|lx\|ly\|hx\|hy` | `Design::read` |
//! | `viadem\|i\|lower=\|upper=\|wrongway=` | via demand lengths |
//! | `dnet\|idx\|name\|pins=\|layers=lo,hi`, `dpin\|net\|pin\|B/I\|name\|l:lx,ly,hx,hy;…` | the netlist |
//! | `unit\|wire=\|via=\|short=…` | unit costs |
//! | `grid\|…`, `centers\|dim\|…`, `tracks\|layer\|…`, `cap\|layer\|x\|c(y=0),…`, `origres\|…` | `GridGraph` |
//! | `grnet\|idx\|name\|pins=\|driver=\|bbox=lx,ly,hx,hy\|hp=`, `pap\|net\|pin\|l:x:y;…` | each `GRNet` |

use std::fmt::Write;

use super::grid_graph::{Commit, GridGraph};
use super::grnet::GrNet;
use super::layers::MetalLayer;
use super::pattern_route::NetRoute;
use super::Cugr;

pub fn list<T: std::fmt::Display>(v: impl IntoIterator<Item = T>) -> String {
    let mut s = String::new();
    for x in v {
        let _ = write!(s, "{x},");
    }
    s
}

fn layer_line(l: &MetalLayer) -> String {
    let ps: String = l.parallel_spacing.iter().map(|row| format!("{};", list(row))).collect();
    format!(
        "VYGC|layer|{}|{}|dir={}|w={}|minw={}|sp={}|pitch={}|first={}|last={}|ntracks={}|minarea={}|minlen={}|pw={}|pl={}|ps={}|defsp={}|eol=-|adj={}|R={}|viaR={}",
        l.index,
        l.name,
        l.direction,
        l.width,
        l.min_width,
        l.spacing,
        l.pitch,
        l.first_track_loc,
        l.last_track_loc,
        l.num_tracks,
        l.area,
        l.min_length,
        list(&l.parallel_width),
        list(&l.parallel_length),
        ps,
        l.default_spacing,
        l.adjustment,
        l.resistance,
        l.via_resistance
    )
}

/// The stage-0 records for one `init`.
pub fn model(c: &Cugr, min_routing_layer: i32, max_routing_layer: i32, clock_nets: usize) -> Vec<String> {
    let (d, g) = (&c.design, &c.grid);
    let mut out = vec![format!("VYGC|init|{min_routing_layer}|{max_routing_layer}|{clock_nets}")];
    out.extend(d.layers.iter().map(layer_line));
    for (i, v) in d.pair_vias.iter().enumerate() {
        out.push(format!("VYGC|via|{i}|{}", v.as_deref().unwrap_or("-")));
    }
    out.push(format!(
        "VYGC|design|dbu={}|die={},{},{},{}|gcell={}|layers={}|nets={}|obstacles={}|special={}",
        d.dbu_per_micron,
        d.die.lx(),
        d.die.ly(),
        d.die.hx(),
        d.die.hy(),
        d.gridline_spacing,
        d.layers.len(),
        d.nets.len(),
        d.obstacles.len(),
        d.num_special_nets
    ));
    for (dim, lines) in d.gridlines.iter().enumerate() {
        out.push(format!("VYGC|gridlines|{dim}|{}|{}", lines.len(), list(lines)));
    }
    for (i, o) in d.obstacles.iter().enumerate() {
        out.push(format!("VYGC|obs|{i}|{}|{}|{}|{}|{}", o.layer, o.b.lx(), o.b.ly(), o.b.hx(), o.b.hy()));
    }
    for i in 0..d.layers.len() {
        out.push(format!("VYGC|viadem|{i}|lower={}|upper={}|wrongway={}", d.via_demand_length_lower[i], d.via_demand_length_upper[i], d.wrong_way_demand_length[i]));
    }
    for n in &d.nets {
        out.push(format!("VYGC|dnet|{}|{}|pins={}|layers={},{}", n.index, n.name, n.pins.len(), n.layer_range.min_layer, n.layer_range.max_layer));
        for p in &n.pins {
            let shapes: String = p.shapes.iter().map(|s| format!("{}:{},{},{},{};", s.layer, s.b.lx(), s.b.ly(), s.b.hx(), s.b.hy())).collect();
            out.push(format!("VYGC|dpin|{}|{}|{}|{}|{}", n.index, p.index, if p.is_port { "B" } else { "I" }, p.name, shapes));
        }
    }
    out.push(format!("VYGC|unit|wire={}|via={}|short={}", d.unit_length_wire_cost, d.unit_via_cost, list(&d.unit_length_short_costs)));
    out.push(format!("VYGC|grid|x={}|y={}|layers={}|m2pitch={}|minlayer={}", g.x_size, g.y_size, g.num_layers, g.m2_pitch, g.min_routing_layer));
    for (dim, centers) in g.grid_centers.iter().enumerate() {
        out.push(format!("VYGC|centers|{dim}|{}", list(centers)));
    }
    for (l, tracks) in g.grid_tracks.iter().enumerate() {
        out.push(format!("VYGC|tracks|{l}|{}", list(tracks)));
    }
    for (l, columns) in g.graph_edges.iter().enumerate() {
        for (x, column) in columns.iter().enumerate() {
            out.push(format!("VYGC|cap|{l}|{x}|{}", list(column.iter().map(|e| e.capacity))));
        }
    }
    out.push(format!("VYGC|origres|{}", list(&g.original_resources_per_layer)));
    for n in &c.nets {
        let b = &n.bounding_box;
        out.push(format!("VYGC|grnet|{}|{}|pins={}|driver={}|bbox={},{},{},{}|hp={}", n.index, n.name, n.num_pins(), n.driver_pin_index, b.lx(), b.ly(), b.hx(), b.hy(), b.hp()));
        if n.ndr_costs.iter().any(|&c| c > 1.0) {
            out.push(format!("VYGC|ndr|{}|{}", n.index, list(&n.ndr_costs)));
        }
        for (p, points) in n.pin_access_points.iter().enumerate() {
            let s: String = points.iter().map(|g| format!("{}:{}:{};", g.layer, g.p.x, g.p.y)).collect();
            out.push(format!("VYGC|pap|{}|{p}|{s}", n.index));
        }
    }
    out
}

/// `order|stage|pos|idx|name|slack|hp`: the nets in routing order.
pub fn order(out: &mut Vec<String>, c: &Cugr, order: &[usize], stage: i32) {
    for (pos, &k) in order.iter().enumerate() {
        let n = &c.nets[k];
        out.push(format!("VYGC|order|{stage}|{pos}|{k}|{}|{}|{}", n.name, n.slack, n.bounding_box.hp()));
    }
}

/// `cgv|dir|x|0101…`: stage 3's congestion view.
pub fn congestion_view(out: &mut Vec<String>, view: &super::grid_graph::View<bool>) {
    for (d, columns) in view.iter().enumerate() {
        for (x, column) in columns.iter().enumerate() {
            out.push(format!("VYGC|cgv|{d}|{x}|{}", column.iter().map(|&b| if b { '1' } else { '0' }).collect::<String>()));
        }
    }
}

/// `wcv|stage|dir|x|c(y=0),…`: the wire-cost view as the maze stage takes it.
pub fn wire_cost_view(out: &mut Vec<String>, view: &super::grid_graph::View<f64>, stage: i32) {
    for (d, columns) in view.iter().enumerate() {
        for (x, column) in columns.iter().enumerate() {
            out.push(format!("VYGC|wcv|{stage}|{d}|{x}|{}", list(column)));
        }
    }
}

/// `cm|stage|l|x|y|delta` for commits made outside a net's own route (rip-ups, demotions).
pub fn commits(out: &mut Vec<String>, commits: &[Commit], stage: i32) {
    for c in commits {
        out.push(format!("VYGC|cm|{stage}|{}|{}|{}|{}", c.layer, c.p.x, c.p.y, c.delta));
    }
}

/// `sparse|net|off=x,y|xs=…|ys=…|pins=…`, `mazepath|net|k|v:cost;…`, `mazetree|net|…`.
pub fn maze(out: &mut Vec<String>, n: &GrNet, r: &NetRoute, grid: &super::maze_route::SparseGrid) {
    let Some((g, arena, found)) = &r.maze else { return };
    let k = n.index;
    let pins: String = g.pseudo_pins.iter().map(|(p, l)| format!("{},{},{},{};", p.x, p.y, l.low, l.high)).collect();
    out.push(format!("VYGC|sparse|{k}|off={},{}|xs={}|ys={}|pins={pins}", grid.offset.x, grid.offset.y, list(&g.xs), list(&g.ys)));
    for (i, &f) in found.iter().enumerate() {
        let mut path = String::new();
        let mut t = Some(f);
        while let Some(id) = t {
            path.push_str(&format!("{}:{};", arena[id].vertex, arena[id].cost));
            t = arena[id].prev;
        }
        out.push(format!("VYGC|mazepath|{k}|{i}|{path}"));
    }
    // One cell: `getSteinerTree` returns before the reference records its tree.
    if g.pseudo_pins.len() == 1 {
        return;
    }
    let st: String = r
        .steiner
        .preorder()
        .into_iter()
        .map(|i| {
            let s = &r.steiner.nodes[i];
            format!("{},{},{},{},{};", s.p.x, s.p.y, s.fixed.low, s.fixed.high, s.children.len())
        })
        .collect();
    out.push(format!("VYGC|mazetree|{k}|{st}"));
}

/// One net's stage records, in the reference's order: per pin the (empty) detailed-router access
/// points and the shape choice, the preferred and selected cells, the Steiner input and tree, the
/// costed DAG, the chosen tree, and every demand committed.
pub fn net_route(out: &mut Vec<String>, n: &GrNet, r: &NetRoute, commits: &[Commit], stage: i32) {
    let k = n.index;
    for pin in 0..n.num_pins() {
        out.push(format!("VYGC|odbap|{k}|{pin}|0|"));
    }
    for &(pin, best, acc, dist, center) in &n.shape_ap_choices {
        out.push(format!("VYGC|shapeap|{k}|{pin}|{best}|{acc}|{dist}|center={},{}", center.x, center.y));
    }
    if r.chose_access_points {
        for (pin, (p, layers)) in &n.preferred_aps {
            out.push(format!("VYGC|pref|{k}|{pin}|{}|{}|{}|{}", p.x, p.y, layers.low, layers.high));
        }
        for (&(x, y), layers) in &r.selected {
            out.push(format!("VYGC|sel|{k}|{x}|{y}|{}|{}", layers.low, layers.high));
        }
    }
    if let Some((xs, ys, driver)) = &r.steiner.input {
        let input: String = xs.iter().zip(ys).map(|(x, y)| format!("{x},{y};")).collect();
        out.push(format!("VYGC|sttin|{k}|{driver}|{input}"));
        let branches: String = r.steiner.branches.iter().enumerate().map(|(i, (x, y, b))| format!("{i}:{x},{y},{b};")).collect();
        out.push(format!("VYGC|stt|{k}|{branches}"));
        let st: String = r
            .steiner
            .preorder()
            .into_iter()
            .map(|i| {
                let s = &r.steiner.nodes[i];
                format!("{},{},{},{},{};", s.p.x, s.p.y, s.fixed.low, s.fixed.high, s.children.len())
            })
            .collect();
        out.push(format!("VYGC|steiner|{k}|{}|{st}", r.steiner.root_branch));
    }
    let mut seen = vec![false; r.dag.nodes.len()];
    let mut stack = vec![r.dag.root];
    while let Some(i) = stack.pop() {
        if seen[i] {
            continue;
        }
        seen[i] = true;
        let d = &r.dag.nodes[i];
        let paths: String = d.paths.iter().map(|cp| format!("{};", list(cp))).collect();
        out.push(format!("VYGC|dag|{k}|{i}|{}|{}|opt={}|fix={},{}|paths={paths}|cost={}", d.p.x, d.p.y, d.optional, d.fixed.low, d.fixed.high, list(&d.costs)));
        for cp in d.paths.iter().rev() {
            for &p in cp.iter().rev() {
                stack.push(p);
            }
        }
    }
    if let Some(t) = &n.routing_tree {
        let s: String = t.preorder().into_iter().map(|i| format!("{}:{}:{}:{};", t.nodes[i].layer, t.nodes[i].p.x, t.nodes[i].p.y, t.nodes[i].children.len())).collect();
        out.push(format!("VYGC|tree|{stage}|{k}|{s}"));
    }
    for c in commits {
        out.push(format!("VYGC|cm|{stage}|{}|{}|{}|{}", c.layer, c.p.x, c.p.y, c.delta));
    }
}

/// `dem|stage|l|x|d(y=0),…`: every edge's demand after the stage.
pub fn demand(out: &mut Vec<String>, g: &GridGraph, stage: i32) {
    for (l, columns) in g.graph_edges.iter().enumerate() {
        for (x, column) in columns.iter().enumerate() {
            out.push(format!("VYGC|dem|{stage}|{l}|{x}|{}", list(column.iter().map(|e| e.demand))));
        }
    }
}
