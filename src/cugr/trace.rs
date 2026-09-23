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

use super::layers::MetalLayer;
use super::Cugr;

fn list<T: std::fmt::Display>(v: impl IntoIterator<Item = T>) -> String {
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
        for (p, points) in n.pin_access_points.iter().enumerate() {
            let s: String = points.iter().map(|g| format!("{}:{}:{};", g.layer, g.p.x, g.p.y)).collect();
            out.push(format!("VYGC|pap|{}|{p}|{s}", n.index));
        }
    }
    out
}
