// SPDX-License-Identifier: Apache-2.0
//! Antenna repair, stage 3 — jumper insertion (`RepairAntennas::jumperInsertion` and the functions
//! it calls), in the reference's order.
//!
//! For each net with violations (by net id), the segments of its route up to the highest layer a
//! jumper may fix become a graph of boxes per TECH layer — routing layers and the cut layers
//! between them. From each violating gate's pin a DFS walks that graph; on the violation layer it
//! scans a segment for a gap in the router's headroom two layers up, and the chosen positions
//! split the segment around a jumper: two via stacks and a wire on `layer + 2`.
//!
//! ⚠️ The router is behind [`JumperRouter`]: FastRoute's is [`FastRouteJumpers`], over the final
//! 3D edges of the run. CUGR's is not modelled.
//!
//! ⬜ Not here: FastRoute's 2D usage and tree-edge relayering that `updateJumperedRoute` also
//! performs (`updateEdge2DAnd3DUsage`'s 2D half, `updateRouteGridsLayer`). Nothing in the jumper
//! pass reads them back; incremental re-routing after diode insertion would.

use std::collections::{BTreeMap, BTreeSet};

use crate::finalize::Graph3d;
use crate::{global_routing_to_box, GSegment, Grid, Rect};

/// One tech layer as the jumper graph sees it. `upper` / `lower` are `getUpperLayer` /
/// `getLowerLayer` as indices into [`TechLayers`] — the tech's own stacking, cut layers included.
#[derive(Debug, Clone)]
pub struct TechLayer {
    pub name: String,
    /// `getRoutingLevel()`: 0 for every non-routing layer.
    pub routing_level: i32,
    pub is_routing: bool,
    pub upper: Option<usize>,
    pub lower: Option<usize>,
}

#[derive(Debug, Clone)]
pub struct TechLayers(pub Vec<TechLayer>);

impl TechLayers {
    /// `dbTech::findRoutingLayer`.
    pub fn find_routing_layer(&self, level: i32) -> Option<usize> {
        self.0.iter().position(|l| l.is_routing && l.routing_level == level)
    }
    fn level(&self, i: usize) -> i32 {
        self.0[i].routing_level
    }
}

/// A violating gate: what `findSegments` and `getSegmentsConnectedToPin` read of it.
#[derive(Debug, Clone)]
pub struct GatePin {
    /// `dbITerm::getName()`, for the trace.
    pub name: String,
    /// `getInstRect`: the instance's bounding box for a standard cell; for a block, the terminal's
    /// LAST pin box on ANY layer (the loop overwrites, it does not merge, and filters no layer).
    pub inst_rect: Rect,
    /// Every box of the terminal on a ROUTING layer, placed, in MPin → geometry order.
    pub pin_boxes: Vec<(usize, Rect)>,
}

/// `ant::Violation`, as far as jumpers read it.
#[derive(Debug, Clone)]
pub struct AntViolation {
    pub routing_level: i32,
    pub gates: Vec<GatePin>,
}

/// One net's violations. ⛔ The reference keeps them in a `PtrMap`, ordered by NET ID; the caller
/// must hand them over in that order.
#[derive(Debug, Clone)]
pub struct NetViolations {
    pub net: String,
    pub violations: Vec<AntViolation>,
}

/// The grid as the jumper pass addresses it.
#[derive(Debug, Clone, Copy)]
pub struct JumperGrid {
    /// For `globalRoutingToBox`.
    pub grid: Grid,
    pub x_grids: i32,
    pub y_grids: i32,
}

impl JumperGrid {
    /// `Grid::getPositionOnGrid`: the centre of the cell a point falls in, the last cell absorbing
    /// a point on the far edge.
    pub fn position_on_grid(&self, (x, y): (i32, i32)) -> (i32, i32) {
        let t = self.grid.tile_size;
        let (x0, y0) = (self.grid.area.x_min, self.grid.area.y_min);
        let mut gx = (x - x0) / t;
        let mut gy = (y - y0) / t;
        if gx >= self.x_grids {
            gx -= 1;
        }
        if gy >= self.y_grids {
            gy -= 1;
        }
        (gx * t + t / 2 + x0, gy * t + t / 2 + y0)
    }

    /// `GlobalRouter::dbuToTile`: the cell index, CLAMPED to the grid.
    pub fn dbu_to_tile(&self, coord: i32, is_x: bool) -> i32 {
        let (origin, grids) = if is_x { (self.grid.area.x_min, self.x_grids) } else { (self.grid.area.y_min, self.y_grids) };
        ((coord - origin) / self.grid.tile_size).clamp(0, grids - 1)
    }
}

/// What the jumper pass asks of the router.
pub trait JumperRouter {
    fn has_available_resources(&mut self, is_horizontal: bool, x: i32, y: i32, layer_level: i32, net: &str) -> bool;
    fn has_jumper_resources(&mut self, init: (i32, i32), fin: (i32, i32), layer_level: i32, net: &str) -> bool;
    /// `route` is the net's route as the jumper just left it (`routes_[net]`) — CUGR re-adopts it.
    fn update_jumpered_route(&mut self, route: &[GSegment], init: (i32, i32), fin: (i32, i32), layer_level: i32, new_layer_level: i32, net: &str) -> bool;
    /// `route` is the net's route rolled back.
    fn restore_net_demand(&mut self, route: &[GSegment], net: &str);
    /// For the trace only: `(cap, usage, cost)` of the edge `has_available_resources` reads.
    fn headroom(&self, is_horizontal: bool, x: i32, y: i32, layer_level: i32, net: &str) -> (i32, i32, i32);
}

/// FastRoute's side of [`JumperRouter`]: the run's final 3D edges, charged per net.
pub struct FastRouteJumpers<'a> {
    pub g3: &'a mut Graph3d,
    pub grid: JumperGrid,
    /// `getLayerEdgeCost(k)` per net, by 0-based layer.
    pub layer_edge_cost: &'a BTreeMap<String, Vec<i8>>,
    /// The router's per-net state by FastRoute id, and each net's id: a jumper rewrites the net's
    /// 3D tree (`updateRouteGridsLayer`), which a later rip-up of the net walks.
    pub trees: &'a mut [crate::brk_rsmt::NetState],
    pub ids: &'a BTreeMap<String, usize>,
}

impl FastRouteJumpers<'_> {
    fn cost(&self, net: &str, layer_level: i32) -> i32 {
        i32::from(self.layer_edge_cost.get(net).map_or(1, |v| v[(layer_level - 1) as usize]))
    }

    /// `updateResources` → `updateEdge2DAnd3DUsage`, the 3D half.
    ///
    /// ⚠️ The span is taken min→max in TILES and walked `x0..x1` exclusive: a span inside one tile
    /// charges nothing. Horizontal is tested first (`y1 == y2`), so a single-tile span is
    /// "horizontal" and empty. Usage is a `uint16_t` charged `+= used * int8_t` — a wrap.
    fn update_resources(&mut self, init: (i32, i32), fin: (i32, i32), layer_level: i32, used: i32, net: &str) {
        let x0 = self.grid.dbu_to_tile(init.0.min(fin.0), true);
        let y0 = self.grid.dbu_to_tile(init.1.min(fin.1), false);
        let x1 = self.grid.dbu_to_tile(fin.0.max(init.0), true);
        let y1 = self.grid.dbu_to_tile(fin.1.max(init.1), false);
        let k = (layer_level - 1) as usize;
        let delta = (used * self.cost(net, layer_level)) as u16;
        let xg = self.g3.x_grid;
        if y0 == y1 {
            for x in x0..x1 {
                let u = &mut self.g3.h_usage[k][y0 as usize * xg + x as usize];
                *u = u.wrapping_add(delta);
            }
        } else if x0 == x1 {
            for y in y0..y1 {
                let u = &mut self.g3.v_usage[k][y as usize * xg + x0 as usize];
                *u = u.wrapping_add(delta);
            }
        }
    }
}

impl JumperRouter for FastRouteJumpers<'_> {
    /// `hasAvailableResources`: `cap - usage >= getDbNetLayerEdgeCost` on the edge LEAVING the
    /// point's tile — east for a horizontal segment, north for a vertical one.
    fn has_available_resources(&mut self, is_horizontal: bool, x: i32, y: i32, layer_level: i32, net: &str) -> bool {
        let (cap, usage, cost) = self.headroom(is_horizontal, x, y, layer_level, net);
        cap - usage >= cost
    }
    /// FastRoute charges only the wire edge, already checked during the scan.
    fn has_jumper_resources(&mut self, _: (i32, i32), _: (i32, i32), _: i32, _: &str) -> bool {
        true
    }
    /// `updateJumperedRoute`: move the span's usage from the segment's layer to the jumper's, then
    /// the net's tree onto it (`updateRouteGridsLayer`, on the span's ends in tiles, 0-based
    /// layers); always accepted.
    ///
    /// ⚠️ Only the 3D half of the usage move is modelled: the 2D half is `-edgeCost` then
    /// `+edgeCost` on the same edges and cancels, and the used-grid entries the `+` adds are
    /// discarded by the next run's `clearUsed`.
    fn update_jumpered_route(&mut self, _: &[GSegment], init: (i32, i32), fin: (i32, i32), layer_level: i32, new_layer_level: i32, net: &str) -> bool {
        self.update_resources(init, fin, layer_level, -1, net);
        self.update_resources(init, fin, new_layer_level, 1, net);
        let (x1, y1) = (self.grid.dbu_to_tile(init.0, true), self.grid.dbu_to_tile(init.1, false));
        let (x2, y2) = (self.grid.dbu_to_tile(fin.0, true), self.grid.dbu_to_tile(fin.1, false));
        if let Some(t) = self.ids.get(net).and_then(|&id| self.trees[id].tree3d.as_mut()) {
            update_route_grids_layer(t, (x1, y1), (x2, y2), (layer_level - 1) as i16, (new_layer_level - 1) as i16);
        }
        true
    }
    fn restore_net_demand(&mut self, _: &[GSegment], _: &str) {}
    fn headroom(&self, is_horizontal: bool, x: i32, y: i32, layer_level: i32, net: &str) -> (i32, i32, i32) {
        let gx = self.grid.dbu_to_tile(x, true) as usize;
        let gy = self.grid.dbu_to_tile(y, false) as usize;
        let k = (layer_level - 1) as usize;
        let i = gy * self.g3.x_grid + gx;
        let (cap, usage) = if is_horizontal { (self.g3.h_cap[k][i], self.g3.h_usage[k][i]) } else { (self.g3.v_cap[k][i], self.g3.v_usage[k][i]) };
        (i32::from(cap), i32::from(usage), self.cost(net, layer_level))
    }
}

/// `updateRouteGridsLayer`: every grid point of the net's tree inside the tile box `lo..=hi` on
/// `layer` moves to `new_layer`. Where a promoted run meets an unpromoted point, a copy of the
/// boundary point on the OLD layer is kept beside it — before the run's first point (unless it is
/// the route's first) and after its last (unless it is the route's last) — so a later rip-up sees a
/// via there, not a same-layer step it never charged. Only edges with `len > 0 || routelen > 0`;
/// an edge with nothing promoted is left exactly as it was.
///
/// ⚠️ `lo`/`hi` are the span's init and final tiles AS GIVEN, not sorted: a span given final-first
/// matches no point.
pub fn update_route_grids_layer(tree: &mut crate::maze3d::Tree3D, lo: (i32, i32), hi: (i32, i32), layer: i16, new_layer: i16) {
    use crate::full3d::Point3D;
    let inside = |p: &Point3D| lo.0 <= i32::from(p.x) && i32::from(p.x) <= hi.0 && lo.1 <= i32::from(p.y) && i32::from(p.y) <= hi.1 && p.layer == layer;
    for e in tree.edges.iter_mut() {
        if e.len <= 0 && e.routelen <= 0 {
            continue;
        }
        let n = e.routelen as usize;
        let g = &e.grids;
        let mut out: Vec<Point3D> = Vec::with_capacity(g.len() + 4);
        let mut modified = false;
        for i in 0..=n {
            if !inside(&g[i]) {
                out.push(g[i]);
                continue;
            }
            modified = true;
            let prev_outside = i == 0 || !inside(&g[i - 1]);
            let next_outside = i == n || !inside(&g[i + 1]);
            if prev_outside && i > 0 {
                out.push(Point3D { layer, ..g[i] });
            }
            out.push(Point3D { layer: new_layer, ..g[i] });
            if next_outside && i < n {
                out.push(Point3D { layer, ..g[i] });
            }
        }
        if modified {
            e.routelen = out.len() as i32 - 1;
            e.grids = out;
        }
    }
}

/// `SegmentNode`: one box of the graph. `seg_id` is the route index, or -1 for the extra nodes a
/// via puts on its two routing layers.
#[derive(Debug, Clone)]
pub struct SegmentNode {
    pub node_id: usize,
    pub seg_id: i32,
    pub rect: Rect,
    /// `(tech layer, index within that layer's nodes)`, in discovery order.
    pub adjs: Vec<(usize, usize)>,
}

/// `LayerToSegmentNodeVector`. The reference's is an `unordered_map` keyed by layer pointer; its
/// iteration order reaches nothing a result depends on (see [`set_adjacent_segments`]), so it is
/// held ordered here.
pub type SegmentGraph = BTreeMap<usize, Vec<SegmentNode>>;

/// The pass's constants, set by `jumperInsertion` from the tile size.
#[derive(Debug, Clone, Copy)]
pub struct JumperSizes {
    pub tile_size: i32,
    pub jumper_size: i32,
    pub smaller_seg_size: i32,
}

impl JumperSizes {
    pub fn new(tile_size: i32) -> Self {
        JumperSizes { tile_size, jumper_size: 2 * tile_size, smaller_seg_size: 5 * tile_size }
    }
}

/// What a whole pass did.
#[derive(Debug, Clone, Default)]
pub struct JumperResult {
    /// `modified_nets`: the nets that got at least one jumper, in the order they got it.
    pub modified_nets: Vec<String>,
    pub total_jumpers: usize,
    /// GRT-302's second number.
    pub net_with_jumpers: usize,
}

/// Everything the pass reads besides the routes and the router.
pub struct JumperInputs<'a> {
    pub tech: &'a TechLayers,
    pub grid: JumperGrid,
    pub max_routing_layer: i32,
}

/// `RepairAntennas::jumperInsertion`.
///
/// `trace`, when given, collects the pass's call sequence in the reference instrument's `VYGJ|`
/// shape (`grt-jumper-trace.py`), for a line-by-line comparison.
pub fn jumper_insertion(
    violations: &[NetViolations],
    routes: &mut BTreeMap<String, Vec<GSegment>>,
    inp: &JumperInputs<'_>,
    router: &mut dyn JumperRouter,
    mut trace: Option<&mut Vec<String>>,
) -> Result<JumperResult, String> {
    let sizes = JumperSizes::new(inp.grid.grid.tile_size);
    let mut res = JumperResult::default();
    for nv in violations {
        let (violation_id_to_repair, max_layer_to_repair) = get_violations(&nv.violations, inp.tech, inp.max_routing_layer);
        if let Some(t) = trace.as_deref_mut() {
            let ids: String = violation_id_to_repair.iter().map(|v| format!("{v},")).collect();
            t.push(format!("net|{}|ids={ids}|max={max_layer_to_repair}", nv.net));
            for (i, sg) in routes.get(&nv.net).map(Vec::as_slice).unwrap_or(&[]).iter().enumerate() {
                t.push(format!("route|{}|{i}|{},{},{}|{},{},{}", nv.net, sg.init_x, sg.init_y, sg.init_layer, sg.final_x, sg.final_y, sg.final_layer));
            }
        }
        let mut segments_to_repair: BTreeMap<i32, BTreeSet<i32>> = BTreeMap::new();
        if !violation_id_to_repair.is_empty() {
            let route = routes.entry(nv.net.clone()).or_default();
            let (graph, num_nodes) = build_segment_graph(route, max_layer_to_repair, inp.tech, &inp.grid.grid);
            if let Some(t) = trace.as_deref_mut() {
                dump_graph(&graph, inp.tech, t);
            }
            for &violation_id in &violation_id_to_repair {
                let layer_level = nv.violations[violation_id].routing_level;
                for gate in &nv.violations[violation_id].gates {
                    let connected = get_segments_connected_to_pin(gate, &graph)?;
                    find_segments(route, gate, &connected, &graph, num_nodes, layer_level, &mut segments_to_repair, &nv.net, &sizes, inp, router, trace.as_deref_mut());
                }
            }
        }
        if !segments_to_repair.is_empty() {
            let route = routes.entry(nv.net.clone()).or_default();
            let jumper_by_net = add_jumper_on_segments(&segments_to_repair, route, &nv.net, &sizes, router, trace.as_deref_mut());
            if let Some(t) = trace.as_deref_mut() {
                t.push(format!("done|{}|{jumper_by_net}", nv.net));
            }
            if jumper_by_net > 0 {
                res.net_with_jumpers += 1;
                res.total_jumpers += jumper_by_net;
                res.modified_nets.push(nv.net.clone());
            }
        }
    }
    Ok(res)
}

/// `getViolations`: the violations a jumper can reach — those with a routing layer TWO above, at
/// or below the max routing layer — and the highest of their layers.
pub fn get_violations(violations: &[AntViolation], tech: &TechLayers, max_routing_layer: i32) -> (Vec<usize>, i32) {
    let mut ids = Vec::new();
    let mut max_layer_to_repair = -1;
    for (violation_id, v) in violations.iter().enumerate() {
        let upper = tech.find_routing_layer(v.routing_level + 2);
        let violation_is_routing = tech.find_routing_layer(v.routing_level).is_some();
        if upper.is_some_and(|u| tech.level(u) <= max_routing_layer) && violation_is_routing {
            ids.push(violation_id);
            max_layer_to_repair = max_layer_to_repair.max(v.routing_level);
        }
    }
    (ids, max_layer_to_repair)
}

/// `buildSegmentGraph`: the nodes, then their adjacency. Returns the graph and the node count.
pub fn build_segment_graph(route: &[GSegment], max_layer: i32, tech: &TechLayers, grid: &Grid) -> (SegmentGraph, usize) {
    let mut graph = SegmentGraph::new();
    let seg_count = get_segments_per_layer(route, max_layer, tech, grid, &mut graph);
    set_adjacent_segments(&mut graph, tech);
    (graph, seg_count)
}

/// `getSegmentsPerLayer`: every segment whose LOWER layer is at or below `max_layer` becomes a
/// node, boxed as its guide would be (`globalRoutingToBox`, no origin offset).
///
/// ⛔ A via becomes THREE nodes: one on the CUT layer above its lower layer, carrying its route
/// index, and one on each of its two routing layers with `seg_id = -1` — those join stacked vias
/// and wires on the same layer, and are never jumper candidates.
pub fn get_segments_per_layer(route: &[GSegment], max_layer: i32, tech: &TechLayers, grid: &Grid, graph: &mut SegmentGraph) -> usize {
    let mut added = 0usize;
    for (seg_id, seg) in route.iter().enumerate() {
        let seg_min_layer = seg.final_layer.min(seg.init_layer);
        if seg_min_layer <= max_layer {
            let tech_layer = tech.find_routing_layer(seg_min_layer).expect("routing layer of a route segment");
            let rect = global_routing_to_box(seg, grid);
            let mut node = |layer: usize, seg_id: i32| {
                graph.entry(layer).or_default().push(SegmentNode { node_id: added, seg_id, rect, adjs: Vec::new() });
                added += 1;
            };
            if seg.is_via() {
                node(tech.0[tech_layer].upper.expect("a layer above a via's lower layer"), seg_id as i32);
                node(tech.find_routing_layer(seg.init_layer).expect("via init layer"), -1);
                node(tech.find_routing_layer(seg.final_layer).expect("via final layer"), -1);
            } else {
                node(tech_layer, seg_id as i32);
            }
        }
    }
    added
}

/// `setAdjacentSegments`: each node's neighbours — same layer first, then the layer below, then
/// the layer above — by STRICT box overlap (`Rect::overlaps`: touching edges do not connect).
///
/// ⚠️ A node overlaps ITSELF, so it lists itself among its same-layer neighbours.
///
/// 🔑 The layer order of the outer loop is the reference's `unordered_map` order and cannot be
/// reproduced; it does not need to be. Each node's list is built from the nodes' RECTS alone, and
/// the lower and upper lists are copies whose rects never change, so every node gets the same list
/// in any layer order.
pub fn set_adjacent_segments(graph: &mut SegmentGraph, tech: &TechLayers) {
    let layers: Vec<usize> = graph.keys().copied().collect();
    for layer in layers {
        let neighbour = |l: Option<usize>| l.and_then(|l| graph.get(&l).map(|v| (l, v.iter().map(|n| n.rect).collect::<Vec<Rect>>())));
        let lower = neighbour(tech.0[layer].lower);
        let upper = neighbour(tech.0[layer].upper);
        let current: Vec<Rect> = graph[&layer].iter().map(|n| n.rect).collect();
        for node in graph.get_mut(&layer).expect("layer present") {
            for (index, r) in current.iter().enumerate() {
                if overlaps(&node.rect, r) {
                    node.adjs.push((layer, index));
                }
            }
            for (l, rects) in [&lower, &upper].into_iter().flatten() {
                if rects.is_empty() {
                    continue;
                }
                for (index, r) in rects.iter().enumerate() {
                    if overlaps(&node.rect, r) {
                        node.adjs.push((*l, index));
                    }
                }
            }
        }
    }
}

/// `odb::Rect::overlaps(Rect)`: strict — the interiors must intersect.
pub fn overlaps(a: &Rect, b: &Rect) -> bool {
    b.x_max > a.x_min && b.x_min < a.x_max && b.y_max > a.y_min && b.y_min < a.y_max
}

/// `SegmentNodeIds` for one gate: per tech layer, the INDICES (within that layer's nodes) of the
/// nodes its pin boxes overlap — in the order the reference iterates them.
pub type ConnectedIds = Vec<(usize, Vec<usize>)>;

/// `getSegmentsConnectedToPin`, and the ORDER the reference then iterates its result in.
///
/// ⛔ The result is an `unordered_map<dbTechLayer*, unordered_set<int>>` and its iteration order
/// SEEDS the DFS stack. The inner set's order is libc++'s (the reference is built with hermetic
/// LLVM) and is reproduced by [`LibcxxIntSet`]. The outer map is keyed by a POINTER, whose hash is
/// a memory address: with more than one layer present the order is not reproducible, so that case
/// is refused rather than guessed.
pub fn get_segments_connected_to_pin(gate: &GatePin, graph: &SegmentGraph) -> Result<ConnectedIds, String> {
    let mut by_layer: Vec<(usize, LibcxxIntSet)> = Vec::new();
    for &(layer, pin_rect) in &gate.pin_boxes {
        let Some(nodes) = graph.get(&layer) else { continue };
        for (seg_id, n) in nodes.iter().enumerate() {
            if overlaps(&n.rect, &pin_rect) {
                let pos = match by_layer.iter().position(|(l, _)| *l == layer) {
                    Some(p) => p,
                    None => {
                        by_layer.push((layer, LibcxxIntSet::default()));
                        by_layer.len() - 1
                    }
                };
                by_layer[pos].1.insert(seg_id);
            }
        }
    }
    if by_layer.len() > 1 {
        return Err(format!("gate {}: pin reaches the route on {} layers — the seed order is a pointer hash", gate.name, by_layer.len()));
    }
    Ok(by_layer.into_iter().map(|(l, s)| (l, s.iter())).collect())
}

/// libc++'s `std::unordered_set<int>` — enough of it to reproduce its ITERATION ORDER.
///
/// `std::hash<int>` is the identity. The bucket of `h` is `h & (n - 1)` for a power-of-two bucket
/// count and `h % n` otherwise (`h` when `h < n`). Before an insert that would exceed load factor
/// 1.0 (or into no buckets) the table rehashes to `max(2n + !is_hash_power2(n), size + 1)`,
/// rounded: 1 → 2, otherwise a non-power-of-two up to the next prime. A node going into an empty
/// bucket is linked at the FRONT of the whole list; into a non-empty bucket, at the front of that
/// bucket's run. A rehash walks the list in order and splices a node whose bucket is already
/// started to the front of that bucket's run.
#[derive(Debug, Clone, Default)]
pub struct LibcxxIntSet {
    list: Vec<usize>,
    bucket_count: usize,
}

impl LibcxxIntSet {
    fn constrain(h: usize, n: usize) -> usize {
        if (n & (n - 1)) == 0 {
            h & (n - 1)
        } else if h < n {
            h
        } else {
            h % n
        }
    }
    fn is_hash_power2(n: usize) -> bool {
        n > 2 && (n & (n - 1)) == 0
    }
    fn next_prime(n: usize) -> usize {
        (n..).find(|&p| p >= 2 && (2..).take_while(|d| d * d <= p).all(|d| p % d != 0)).expect("a prime")
    }
    /// `__rehash_unique(n)`, growing only (an insert never shrinks).
    fn rehash(&mut self, n: usize) {
        let n = if n == 1 {
            2
        } else if (n & (n - 1)) != 0 {
            Self::next_prime(n)
        } else {
            n
        };
        if n > self.bucket_count {
            self.do_rehash(n);
        }
    }
    fn do_rehash(&mut self, nbc: usize) {
        self.bucket_count = nbc;
        // In list order: a node in the current run stays; one starting a new bucket stays and
        // records the node before it; one whose bucket is already started is spliced in right
        // after that recorded node — to the FRONT of its bucket's run.
        let old = std::mem::take(&mut self.list);
        let mut out: Vec<usize> = Vec::with_capacity(old.len());
        // bucket -> position in `out` of the node BEFORE the bucket's run (None = list head)
        let mut before: BTreeMap<usize, Option<usize>> = BTreeMap::new();
        let mut phash = usize::MAX;
        for h in old {
            let c = Self::constrain(h, nbc);
            if out.is_empty() {
                before.insert(c, None);
                out.push(h);
                phash = c;
            } else if c == phash {
                out.push(h);
            } else if let Some(&b) = before.get(&c) {
                let at = b.map_or(0, |p| p + 1);
                out.insert(at, h);
                // Positions after the insertion point shift by one.
                for v in before.values_mut() {
                    if let Some(p) = v {
                        if *p >= at {
                            *p += 1;
                        }
                    }
                }
            } else {
                before.insert(c, Some(out.len() - 1));
                out.push(h);
                phash = c;
            }
        }
        self.list = out;
    }
    pub fn insert(&mut self, h: usize) {
        if self.list.contains(&h) {
            return;
        }
        let size = self.list.len();
        if size + 1 > self.bucket_count || self.bucket_count == 0 {
            let n = 2 * self.bucket_count + usize::from(!Self::is_hash_power2(self.bucket_count));
            self.rehash(n.max(size + 1));
        }
        let c = Self::constrain(h, self.bucket_count);
        // The bucket's run starts at its first node in list order.
        match self.list.iter().position(|&x| Self::constrain(x, self.bucket_count) == c) {
            Some(p) => self.list.insert(p, h),
            None => self.list.insert(0, h),
        }
    }
    pub fn iter(&self) -> Vec<usize> {
        self.list.clone()
    }
    pub fn bucket_count(&self) -> usize {
        self.bucket_count
    }
}

/// `findSegmentPos`: the segment's midpoint, truncating.
fn find_segment_pos(seg: &GSegment) -> (i32, i32) {
    ((seg.init_x + seg.final_x) / 2, (seg.init_y + seg.final_y) / 2)
}

fn seg_length(seg: &GSegment) -> i32 {
    (seg.init_x - seg.final_x).abs() + (seg.init_y - seg.final_y).abs()
}

/// `findSegments`: a DFS from the gate's pin over the graph, collecting a jumper position for
/// each wire on the violation layer it reaches.
///
/// ⛔ The stack holds node COPIES and is seeded in the connected set's iteration order (LIFO, so
/// the LAST seeded is explored first). Every seed's parent is the gate's grid position.
///
/// ⛔ A node on the violation layer with a route index is a candidate: its parent position is
/// snapped to the grid, and when a position is found the walk does NOT continue past it.
/// Otherwise the walk continues to every neighbour — but only from a node at or below the
/// violation layer, or on a cut layer (level 0) — and a neighbour's parent is overwritten with
/// this node's midpoint when this node has a route index, even if that neighbour is already on
/// the stack or visited.
#[allow(clippy::too_many_arguments)]
pub fn find_segments(
    route: &[GSegment],
    gate: &GatePin,
    segment_ids: &ConnectedIds,
    graph: &SegmentGraph,
    num_nodes: usize,
    violation_layer: i32,
    segments_to_repair: &mut BTreeMap<i32, BTreeSet<i32>>,
    net: &str,
    sizes: &JumperSizes,
    inp: &JumperInputs<'_>,
    router: &mut dyn JumperRouter,
    mut trace: Option<&mut Vec<String>>,
) {
    let mut stack: Vec<(usize, SegmentNode)> = Vec::new();
    let mut visited = vec![false; num_nodes];
    let mut parent_pos = vec![(0, 0); num_nodes];
    let r = &gate.inst_rect;
    let gate_pos = inp.grid.position_on_grid(((r.x_min + r.x_max) / 2, (r.y_min + r.y_max) / 2));
    if let Some(t) = trace.as_deref_mut() {
        t.push(format!("gate|{}|viol={violation_layer}|gate_pos={},{}", gate.name, gate_pos.0, gate_pos.1));
        for (layer, ids) in segment_ids {
            let ids: String = ids.iter().map(|i| format!("{i},")).collect();
            t.push(format!("conn|{}|{ids}", inp.tech.0[*layer].name));
        }
    }
    for (layer, ids) in segment_ids {
        for &i in ids {
            let node = graph[layer][i].clone();
            parent_pos[node.node_id] = gate_pos;
            stack.push((*layer, node));
        }
    }
    while let Some((cur_layer, cur)) = stack.pop() {
        if let Some(t) = trace.as_deref_mut() {
            let p = parent_pos[cur.node_id];
            t.push(format!("pop|{}|{}|{}|visited={}|parent={},{}", cur.node_id, inp.tech.0[cur_layer].name, cur.seg_id, i32::from(visited[cur.node_id]), p.0, p.1));
        }
        if visited[cur.node_id] {
            continue;
        }
        visited[cur.node_id] = true;
        let layer_level = inp.tech.level(cur_layer);
        if layer_level == violation_layer && cur.seg_id != -1 {
            parent_pos[cur.node_id] = inp.grid.position_on_grid(parent_pos[cur.node_id]);
            if let Some(pos) = find_pos_to_jumper(route, graph, &cur, parent_pos[cur.node_id], net, sizes, inp, router, trace.as_deref_mut()) {
                segments_to_repair.entry(cur.seg_id).or_default().insert(pos);
                continue;
            }
        }
        for &(adj_layer, adj_index) in &cur.adjs {
            if layer_level == 0 || layer_level <= violation_layer {
                let adj = graph[&adj_layer][adj_index].clone();
                if cur.seg_id != -1 {
                    parent_pos[adj.node_id] = find_segment_pos(&route[cur.seg_id as usize]);
                }
                if let Some(t) = trace.as_deref_mut() {
                    let p = parent_pos[adj.node_id];
                    t.push(format!("push|{}|{}|parent={},{}|set={}", adj.node_id, inp.tech.0[adj_layer].name, p.0, p.1, i32::from(cur.seg_id != -1)));
                }
                stack.push((adj_layer, adj));
            }
        }
    }
}

/// `getViaPosition`: where the segment's vias land, along its axis — the route index of every
/// neighbour on ANOTHER layer (a cut-layer node, which carries its via's index), read at `init_x`
/// for a horizontal segment and `init_y` for a vertical one.
fn get_via_position(graph: &SegmentGraph, route: &[GSegment], seg_node: &SegmentNode, tech: &TechLayers) -> BTreeSet<i32> {
    let seg = &route[seg_node.seg_id as usize];
    let layer_level = seg.init_layer;
    let is_horizontal = seg.init_x != seg.final_x;
    let mut via_pos = BTreeSet::new();
    for &(adj_layer, adj_index) in &seg_node.adjs {
        if tech.level(adj_layer) != layer_level {
            let adj_seg = &route[graph[&adj_layer][adj_index].seg_id as usize];
            via_pos.insert(if is_horizontal { adj_seg.init_x } else { adj_seg.init_y });
        }
    }
    via_pos
}

/// `findPosToJumper`: scan the segment one tile at a time; each tile that holds a via or lacks
/// headroom two layers up closes a free window, and each window long enough gives one candidate.
#[allow(clippy::too_many_arguments)]
fn find_pos_to_jumper(
    route: &[GSegment],
    graph: &SegmentGraph,
    seg_node: &SegmentNode,
    parent_pos: (i32, i32),
    net: &str,
    sizes: &JumperSizes,
    inp: &JumperInputs<'_>,
    router: &mut dyn JumperRouter,
    mut trace: Option<&mut Vec<String>>,
) -> Option<i32> {
    let seg = &route[seg_node.seg_id as usize];
    if seg_length(seg) < sizes.smaller_seg_size {
        return None;
    }
    let (seg_init_x, seg_init_y, seg_final_x, seg_final_y) = (seg.init_x, seg.init_y, seg.final_x, seg.final_y);
    let step = sizes.tile_size;
    let (mut pos_x, mut pos_y) = (seg_init_x, seg_init_y);
    let (mut last_block_x, mut last_block_y) = (pos_x, pos_y);
    let layer_level = seg.init_layer;
    let is_horizontal = seg.init_x != seg.final_x;
    let via_pos = get_via_position(graph, route, seg_node, inp.tech);
    let mut candidate_positions = Vec::new();
    let mut free_windows = Vec::new();
    while pos_x <= seg_final_x && pos_y <= seg_final_y {
        let has_available_resources = router.has_available_resources(is_horizontal, pos_x, pos_y, layer_level + 2, net);
        let is_via = (is_horizontal && via_pos.contains(&pos_x)) || (!is_horizontal && via_pos.contains(&pos_y));
        if let Some(t) = trace.as_deref_mut() {
            let (cap, usage, cost) = router.headroom(is_horizontal, pos_x, pos_y, layer_level + 2, net);
            let (gx, gy) = (inp.grid.dbu_to_tile(pos_x, true), inp.grid.dbu_to_tile(pos_y, false));
            t.push(format!(
                "scan|{}|{pos_x},{pos_y}|l={}|g={gx},{gy}|k={}|cap={cap}|usage={usage}|cost={cost}|avail={}|via={}",
                seg_node.seg_id,
                layer_level + 2,
                layer_level + 1,
                i32::from(has_available_resources),
                i32::from(is_via)
            ));
        }
        if is_via || !has_available_resources {
            find_jumper_candidate_positions((last_block_x, last_block_y), (pos_x, pos_y), parent_pos, is_horizontal, sizes, &mut candidate_positions, &mut free_windows);
            (last_block_x, last_block_y) = (pos_x, pos_y);
        }
        if is_horizontal {
            pos_x += step;
        } else {
            pos_y += step;
        }
    }
    if last_block_x != seg_final_x || last_block_y != seg_final_y {
        find_jumper_candidate_positions((last_block_x, last_block_y), (seg_final_x, seg_final_y), parent_pos, is_horizontal, sizes, &mut candidate_positions, &mut free_windows);
    }
    if let Some(t) = trace.as_deref_mut() {
        let c: String = candidate_positions.iter().map(|p| format!("{p},")).collect();
        let w: String = free_windows.iter().map(|(lo, hi)| format!("{lo}:{hi},")).collect();
        t.push(format!("cand|{}|{c}|win={w}|parent={},{}", seg_node.seg_id, parent_pos.0, parent_pos.1));
    }
    let pos = select_jumper_position(&mut candidate_positions, &free_windows, is_horizontal, parent_pos, (seg_init_x, seg_init_y), layer_level, net, sizes, router);
    if let Some(t) = trace.as_deref_mut() {
        t.push(format!("sel|{}|{pos}", seg_node.seg_id));
    }
    (pos != -1).then_some(pos)
}

/// `findJumperCandidatePositions`: a window from the last block to this one is usable when it
/// holds the jumper plus one tile of clearance at each end.
///
/// ⚠️ `free_via_size > 0` and the size test are both there; a zero-length window fails the first.
fn find_jumper_candidate_positions(
    init: (i32, i32),
    fin: (i32, i32),
    parent_pos: (i32, i32),
    is_horizontal: bool,
    sizes: &JumperSizes,
    candidate_positions: &mut Vec<i32>,
    free_windows: &mut Vec<(i32, i32)>,
) {
    let jumper_dist = 1;
    let free_via_size = (fin.0 - init.0).abs() + (fin.1 - init.1).abs();
    if free_via_size > 0 && free_via_size >= sizes.jumper_size + 2 * jumper_dist * sizes.tile_size {
        candidate_positions.push(if is_horizontal {
            get_jumper_position(init.0, fin.0, parent_pos.0, sizes)
        } else {
            get_jumper_position(init.1, fin.1, parent_pos.1, sizes)
        });
        let lo = if is_horizontal { init.0 } else { init.1 } + jumper_dist * sizes.tile_size;
        let hi = if is_horizontal { fin.0 } else { fin.1 } - sizes.jumper_size - jumper_dist * sizes.tile_size;
        free_windows.push((lo, hi));
    }
}

/// `getJumperPosition`: as close to the parent as the window allows, one tile in from its ends —
/// leaving the SHORTER piece connected to the parent side.
///
/// ⛔ The midpoint test is strict: at exactly equal distances the jumper goes on the FAR side of
/// the target (`target + tile`).
pub fn get_jumper_position(init_pos: i32, final_pos: i32, target_pos: i32, sizes: &JumperSizes) -> i32 {
    if target_pos <= init_pos {
        init_pos + sizes.tile_size
    } else if target_pos >= final_pos {
        final_pos - sizes.tile_size - sizes.jumper_size
    } else if (target_pos - init_pos) > (final_pos - target_pos) {
        target_pos - sizes.tile_size - sizes.jumper_size
    } else {
        target_pos + sizes.tile_size
    }
}

/// `selectJumperPosition`: candidates STABLE-sorted by `|pos + tile - target|`, the first that fits
/// wins; if none does, every tile-aligned position of every window not already tried, sorted the
/// same way.
#[allow(clippy::too_many_arguments)]
pub fn select_jumper_position(
    candidate_positions: &mut [i32],
    free_windows: &[(i32, i32)],
    is_horizontal: bool,
    parent_pos: (i32, i32),
    seg_init: (i32, i32),
    layer_level: i32,
    net: &str,
    sizes: &JumperSizes,
    router: &mut dyn JumperRouter,
) -> i32 {
    let target = if is_horizontal { parent_pos.0 } else { parent_pos.1 };
    let mut select = |positions: &mut [i32]| -> i32 {
        positions.sort_by_key(|&p| (p + sizes.tile_size - target).abs());
        for &pos in positions.iter() {
            if jumper_fits(pos, pos + sizes.jumper_size, seg_init, is_horizontal, layer_level, net, router) {
                return pos;
            }
        }
        -1
    };
    let mut jumper_position = select(candidate_positions);
    if jumper_position == -1 {
        let mut fallback = Vec::new();
        for &(lo, hi) in free_windows {
            let mut pos = lo;
            while pos <= hi {
                if !candidate_positions.contains(&pos) {
                    fallback.push(pos);
                }
                pos += sizes.tile_size;
            }
        }
        jumper_position = select(&mut fallback);
    }
    jumper_position
}

/// `jumperFits`: the whole jumper's headroom, two layers up.
fn jumper_fits(init_pos: i32, final_pos: i32, seg_init: (i32, i32), is_horizontal: bool, layer_level: i32, net: &str, router: &mut dyn JumperRouter) -> bool {
    let (init, fin) = if is_horizontal { ((init_pos, seg_init.1), (final_pos, seg_init.1)) } else { ((seg_init.0, init_pos), (seg_init.0, final_pos)) };
    router.has_jumper_resources(init, fin, layer_level + 2, net)
}

/// `addJumperOnSegments`: per segment in index order, its positions in ascending order.
///
/// ⛔ The length test `break`s the segment's loop — and the segment SHRINKS as jumpers split it,
/// so a later position can be dropped by an earlier jumper. A position within one jumper length of
/// the last one INSERTED is skipped (a rejected one does not count).
pub fn add_jumper_on_segments(
    segments_to_repair: &BTreeMap<i32, BTreeSet<i32>>,
    route: &mut Vec<GSegment>,
    net: &str,
    sizes: &JumperSizes,
    router: &mut dyn JumperRouter,
    mut trace: Option<&mut Vec<String>>,
) -> usize {
    let mut jumper_by_net = 0;
    for (&seg, positions) in segments_to_repair {
        let mut last_pos_aux = -1;
        for &pos in positions {
            if seg_length(&route[seg as usize]) < sizes.smaller_seg_size {
                break;
            }
            if last_pos_aux != -1 && (last_pos_aux - pos).abs() <= sizes.jumper_size {
                continue;
            }
            let layer = route[seg as usize].init_layer;
            let ok = add_jumper_to_route(route, seg as usize, pos, pos + sizes.jumper_size, layer, net, router);
            if let Some(t) = trace.as_deref_mut() {
                t.push(format!("try|{seg}|{pos}|ok={}", i32::from(ok)));
            }
            if ok {
                jumper_by_net += 1;
                last_pos_aux = pos;
            }
        }
    }
    jumper_by_net
}

/// `addJumperToRoute`: fit test, then the jumper's vias and wire appended, then the segment split —
/// the part BEFORE the jumper appended as a new segment, the original shortened to start AFTER it.
///
/// ⛔ Append order is behaviour, since the guides are written in route order: via up at the start,
/// via up at the end, the second via pair, the jumper wire, and only then the split-off segment.
#[allow(clippy::too_many_arguments)]
fn add_jumper_to_route(
    route: &mut Vec<GSegment>,
    seg_id: usize,
    jumper_init_pos: i32,
    jumper_final_pos: i32,
    layer_level: i32,
    net: &str,
    router: &mut dyn JumperRouter,
) -> bool {
    let route_size_before = route.len();
    let (seg_init_x, seg_init_y) = (route[seg_id].init_x, route[seg_id].init_y);
    let is_horizontal = seg_init_x != route[seg_id].final_x;
    let (ji, jf) = if is_horizontal {
        ((jumper_init_pos, seg_init_y), (jumper_final_pos, seg_init_y))
    } else {
        ((seg_init_x, jumper_init_pos), (seg_init_x, jumper_final_pos))
    };
    if !jumper_fits(jumper_init_pos, jumper_final_pos, (seg_init_x, seg_init_y), is_horizontal, layer_level, net, router) {
        return false;
    }
    add_jumper_and_vias(route, ji, jf, layer_level);
    route.push(GSegment::new(seg_init_x, seg_init_y, layer_level, ji.0, ji.1, layer_level));
    route[seg_id].init_x = jf.0;
    route[seg_id].init_y = jf.1;
    if !router.update_jumpered_route(route, ji, jf, layer_level, layer_level + 2, net) {
        route.truncate(route_size_before);
        route[seg_id].init_x = seg_init_x;
        route[seg_id].init_y = seg_init_y;
        router.restore_net_demand(route, net);
        return false;
    }
    true
}

/// `addJumperAndVias`: for `layer` and `layer + 1`, a via at the start then one at the end; then
/// the jumper wire on `layer + 2`, flagged a jumper.
pub fn add_jumper_and_vias(route: &mut Vec<GSegment>, init: (i32, i32), fin: (i32, i32), layer_level: i32) {
    for layer in layer_level..layer_level + 2 {
        route.push(GSegment::new(init.0, init.1, layer, init.0, init.1, layer + 1));
        route.push(GSegment::new(fin.0, fin.1, layer, fin.0, fin.1, layer + 1));
    }
    let mut jumper = GSegment::new(init.0, init.1, layer_level + 2, fin.0, fin.1, layer_level + 2);
    jumper.is_jumper = true;
    route.push(jumper);
}

/// The graph in the trace's shape: every node, then its adjacency.
fn dump_graph(graph: &SegmentGraph, tech: &TechLayers, t: &mut Vec<String>) {
    for (&layer, nodes) in graph {
        for (index, n) in nodes.iter().enumerate() {
            let r = n.rect;
            t.push(format!("node|{}|{}|{}|{}|{},{},{},{}", tech.0[layer].name, tech.0[layer].routing_level, n.node_id, n.seg_id, r.x_min, r.y_min, r.x_max, r.y_max));
            let adjs: String = n.adjs.iter().map(|(l, i)| format!("{}:{i};", tech.0[*l].name)).collect();
            t.push(format!("adj|{}|{index}|{}|{adjs}", tech.0[layer].name, n.node_id));
        }
    }
}

// ---- stage 4a: diode placement (`insertDiode`, `setDiodeLoc`) ----

/// A row as the diode placement reads it: its box and orientation.
#[derive(Debug, Clone)]
pub struct DiodeRow {
    pub bbox: Rect,
    pub orient: String,
}

/// What the diode placement reads of the block, fixed for one repair.
#[derive(Debug, Clone)]
pub struct DiodeFloor {
    /// In `getRows()` order.
    pub rows: Vec<DiodeRow>,
    pub core: Rect,
    /// The first non-PAD row's site width.
    pub site_width: i32,
    /// `opendp_->padLeft` / `padRight` of a diode, in sites.
    pub pad_left: i32,
    pub pad_right: i32,
    /// The diode master's size (its bbox at R0).
    pub diode_width: i32,
    pub diode_height: i32,
}

/// The gate a diode protects: `getInstancePlacementData`.
#[derive(Debug, Clone)]
pub struct DiodeGate {
    /// `getInstRect`: the instance box (a block's pin box).
    pub rect: Rect,
    pub orient: String,
    /// `isBlock() || isPad()`: the diode takes the ROW's orientation.
    pub block_or_pad: bool,
    /// The master itself is a block (the FIRM test reads only this).
    pub is_block: bool,
}

/// Where a diode ended up.
#[derive(Debug, Clone, PartialEq)]
pub struct DiodePlacement {
    pub x: i32,
    pub y: i32,
    pub orient: String,
    /// `setDiodeLoc` found a spot clear of the fixed cells, AND it lies in a row.
    pub legal: bool,
    pub in_row: bool,
    /// FIRM when legal, inside the core and not beside a block; PLACED for dpl to move.
    pub status: &'static str,
    /// Every attempt: `(x, y, orient, clear)`.
    pub tries: Vec<(i32, i32, String, bool)>,
}

/// `getRowOrient`: the orientation of the LAST row whose box strictly contains the point; `R0`
/// (the default `dbOrientType`) when none does.
pub fn row_orient(rows: &[DiodeRow], p: (i32, i32)) -> String {
    let mut orient = "R0".to_string();
    for r in rows {
        let b = &r.bbox;
        if p.0 > b.x_min && p.0 < b.x_max && p.1 > b.y_min && p.1 < b.y_max {
            orient = r.orient.clone();
        }
    }
    orient
}

/// A box's width and height at an orientation: a quarter turn swaps them.
fn oriented(w: i32, h: i32, orient: &str) -> (i32, i32) {
    if orient.contains("90") {
        (h, w)
    } else {
        (w, h)
    }
}

/// `checkDiodeLoc`: the diode's box, widened by BOTH paddings on EACH side and shrunk by one unit
/// all round, must touch no fixed box (`bgi::intersects` — edges count), and the diode must lie
/// inside the core (edges included).
pub fn check_diode_loc(bbox: &Rect, floor: &DiodeFloor, fixed: &[Rect]) -> bool {
    let pad = (floor.pad_left + floor.pad_right) * floor.site_width;
    let q = (bbox.x_min - pad + 1, bbox.y_min + 1, bbox.x_max + pad - 1, bbox.y_max - 1);
    let clear = !fixed.iter().any(|f| f.x_min <= q.2 && q.0 <= f.x_max && f.y_min <= q.3 && q.1 <= f.y_max);
    let c = &floor.core;
    clear && bbox.x_min >= c.x_min && bbox.y_min >= c.y_min && bbox.x_max <= c.x_max && bbox.y_max <= c.y_max
}

/// `diodeInRow`: some row's box contains the diode and is exactly as tall.
pub fn diode_in_row(bbox: &Rect, rows: &[DiodeRow]) -> bool {
    rows.iter().any(|r| {
        let b = &r.bbox;
        bbox.x_min >= b.x_min && bbox.y_min >= b.y_min && bbox.x_max <= b.x_max && bbox.y_max <= b.y_max && (bbox.y_max - bbox.y_min) == (b.y_max - b.y_min)
    })
}

/// `setDiodeLoc` and the rest of `insertDiode`: up to 50 tries, alternating sides — beside the
/// gate (left first, then right, each a site further out every time) for a horizontal violation
/// layer, above and below it (below first, starting ON the gate, each a gate height further) for a
/// vertical one.
///
/// ⛔ Every try takes the GATE's orientation — and a block or pad gate, or a vertical violation
/// layer, the orientation of the row under the diode's centre instead.
///
/// ⚠️ The diode's size is read once, before any orientation is set.
pub fn place_diode(gate: &DiodeGate, place_vertically: bool, floor: &DiodeFloor, fixed: &[Rect]) -> DiodePlacement {
    const MAX_LEGALIZE_ITR: usize = 50;
    let (inst_x, inst_y) = (gate.rect.x_min, gate.rect.y_min);
    let (inst_w, inst_h) = (gate.rect.x_max - gate.rect.x_min, gate.rect.y_max - gate.rect.y_min);
    let (dw, dh) = (floor.diode_width, floor.diode_height);
    let (mut place_at_left, mut place_at_top) = (true, false);
    let (mut left_offset, mut right_offset, mut top_offset, mut bottom_offset) = (0, 0, 0, 0);
    let (mut h_off, mut v_off) = (0, 0);
    let mut tries = Vec::new();
    let mut legal = false;
    let (mut x, mut y, mut orient) = (0, 0, String::new());
    while !legal && tries.len() < MAX_LEGALIZE_ITR {
        if place_vertically {
            if place_at_top {
                v_off = top_offset * inst_h;
                top_offset += 1;
                place_at_top = false;
            } else {
                v_off = -(bottom_offset * inst_h);
                bottom_offset += 1;
                place_at_top = true;
            }
        } else if place_at_left {
            h_off = -(dw + left_offset * floor.site_width);
            left_offset += 1;
            place_at_left = false;
        } else {
            h_off = inst_w + right_offset * floor.site_width;
            right_offset += 1;
            place_at_left = true;
        }
        orient = gate.orient.clone();
        if gate.block_or_pad || place_vertically {
            let centre = (inst_x + h_off + dw / 2, inst_y + v_off + dh / 2);
            orient = row_orient(&floor.rows, centre);
        }
        (x, y) = (inst_x + h_off, inst_y + v_off);
        let (w, h) = oriented(dw, dh, &orient);
        legal = check_diode_loc(&Rect { x_min: x, y_min: y, x_max: x + w, y_max: y + h }, floor, fixed);
        tries.push((x, y, orient.clone(), legal));
    }
    let (w, h) = oriented(dw, dh, &orient);
    let bbox = Rect { x_min: x, y_min: y, x_max: x + w, y_max: y + h };
    let in_row = diode_in_row(&bbox, &floor.rows);
    let legal = legal && in_row;
    let c = &floor.core;
    let in_core = bbox.x_min >= c.x_min && bbox.y_min >= c.y_min && bbox.x_max <= c.x_max && bbox.y_max <= c.y_max;
    let status = if in_core && !gate.is_block && legal { "FIRM" } else { "PLACED" };
    DiodePlacement { x, y, orient, legal, in_row, status, tries }
}
