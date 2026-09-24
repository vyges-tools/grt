// SPDX-License-Identifier: Apache-2.0
//! Stage 1, pattern routing (`PatternRoute`): per net, the access points → a Steiner tree over
//! them → a routing DAG of straight and L-shaped paths → a dynamic program over layers for the
//! cheapest → that choice as a routing tree.
//!
//! [`pattern_route_net`] is the stage's per-net sequence and nothing else; each step is its own
//! function, named after the reference's, in the reference's order.

use std::collections::BTreeMap;

use super::design::Design;
use super::geo::{Interval, Point};
use super::grid_graph::{GrTree, GridGraph};
use super::grnet::GrNet;
use super::layers::{H, V};
use super::Constants;

/// The Steiner tree builder as the stage calls it: `(xs, ys, driver index, alpha)`, returning each
/// branch as `(x, y, n)` (`stt::Tree::branch`).
pub type SteinerBuilder<'a> = &'a dyn Fn(&[i32], &[i32], usize, f32) -> Vec<(i32, i32, usize)>;

/// A cell and the union of the layers its pins need (`AccessPointMap`). ⚠️ The reference's map is
/// a hash map; it is iterated only to feed an (x, y) sort over unique keys and looked up otherwise,
/// so its order cannot leak — a map ordered by (x, y) is that sort's result directly.
pub type AccessPointMap = BTreeMap<(i32, i32), Interval>;

/// What went wrong where the reference would log an error or index out of range.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PatternError {
    /// GRT-0283: a pin with no shape-derived cell.
    NoAccessPoint { net: String, pin: usize },
    /// A layer with no finite cost: the reference would read a best path of `(-1, -1)`.
    NoFinitePath { net: String },
    /// Two degree-1 Steiner points at one location point at each other: the reference recurses
    /// without end.
    DegenerateSteinerTree { net: String },
}

/// Log lines the stage emits (`GRT-0274`).
pub type Log = Vec<String>;

// ---------------------------------------------------------------- access points ----

/// `selectShapeAccessPoint(net, pin)`: of the cells the pin's shapes touch, the most accessible —
/// then the closest to the net's bbox centre.
///
/// Upstream rule: a cell's accessibility, on a routable layer, counts the edge at it and the edge
/// before it along the layer's direction (where one exists) that have at least one track; a cell
/// below the min routing layer is accessible (1). Strictly better accessibility, or equal and
/// strictly nearer, wins — the first such cell in the pin's order. Every cell of the chosen
/// location, on any layer, widens the pin's layer interval.
fn select_shape_access_point(net: &mut GrNet, pin: usize, grid: &GridGraph, map: &mut AccessPointMap, log: &mut Log) -> Result<(), PatternError> {
    let center = Point::new(net.bounding_box.cx(), net.bounding_box.cy());
    let points = &net.pin_access_points[pin];
    let (mut best_acc, mut best_dist, mut best_index) = (0, i32::MAX, None);
    for (index, g) in points.iter().enumerate() {
        let mut accessibility = 0;
        if g.layer as usize >= grid.min_routing_layer {
            let d = grid.layer_directions[g.layer as usize];
            accessibility += i32::from(grid.edge(g.layer as usize, g.p.x as usize, g.p.y as usize).capacity >= 1.0);
            if g.p.get(d) > 0 {
                let mut lower = g.p;
                lower.set(d, g.p.get(d) - 1);
                accessibility += i32::from(grid.edge(g.layer as usize, lower.x as usize, lower.y as usize).capacity >= 1.0);
            }
        } else {
            accessibility = 1;
        }
        let distance = (center.x - g.p.x).abs() + (center.y - g.p.y).abs();
        if accessibility > best_acc || (accessibility == best_acc && distance < best_dist) {
            (best_acc, best_dist, best_index) = (accessibility, distance, Some(index));
        }
    }
    if best_acc == 0 {
        log.push(format!("[WARNING GRT-0274] Pin {pin} of net {} is hard to access.", net.name));
    }
    let best = best_index.ok_or_else(|| PatternError::NoAccessPoint { net: net.name.clone(), pin })?;
    let selected = points[best].p;
    let mut fixed = Interval::default();
    for g in points {
        if g.p == selected {
            fixed.update(g.layer);
        }
    }
    let layers = map.entry((selected.x, selected.y)).or_default();
    *layers = layers.union_with(&fixed);
    net.preferred_aps.insert(pin, (selected, fixed));
    net.shape_ap_choices.push((pin, best as i32, best_acc, best_dist, center));
    Ok(())
}

/// `selectAccessPoints(net)`. Only the shape path is modelled: the caller refuses a design whose
/// database carries detailed-router access points (`findODBAccessPoints`' path).
pub fn select_access_points(net: &mut GrNet, grid: &GridGraph, log: &mut Log) -> Result<AccessPointMap, PatternError> {
    let mut map = AccessPointMap::new();
    net.shape_ap_choices.clear();
    for pin in 0..net.num_pins() {
        select_shape_access_point(net, pin, grid, &mut map, log)?;
    }
    Ok(map)
}

// ---------------------------------------------------------------- Steiner tree ----

/// A node of the Steiner tree (`SteinerTreeNode`).
#[derive(Debug, Clone, PartialEq)]
pub struct SteinerNode {
    pub p: Point,
    pub fixed: Interval,
    pub children: Vec<usize>,
}

/// The Steiner tree, as an arena: `root` is its root.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SteinerTree {
    pub nodes: Vec<SteinerNode>,
    pub root: usize,
    /// `(xs, ys, driver index)` as the builder was called, and its branches (for the trace).
    pub input: Option<(Vec<i32>, Vec<i32>, usize)>,
    pub branches: Vec<(i32, i32, usize)>,
    /// The branch index the tree was rooted at.
    pub root_branch: usize,
}

impl SteinerTree {
    pub fn preorder(&self) -> Vec<usize> {
        let mut out = Vec::new();
        let mut stack = vec![self.root];
        while let Some(n) = stack.pop() {
            out.push(n);
            stack.extend(self.nodes[n].children.iter().rev());
        }
        out
    }
}

/// `constructSteinerTree`.
///
/// Upstream rule: one cell is the whole tree. Otherwise the cells go to the builder in (x, y)
/// order with the driver's cell as the driver index (0 when the driver has none). The branches
/// become an adjacency list (both directions, branch order); the root is the first branch of
/// degree 1 — following a degree-1 branch through co-located neighbours. A branch at its
/// parent's location is merged into the parent; each node takes the fixed layers of its cell.
pub fn construct_steiner_tree(net: &GrNet, map: &AccessPointMap, alpha: f32, stt: SteinerBuilder<'_>) -> Result<SteinerTree, PatternError> {
    if map.len() == 1 {
        let (&(x, y), &layers) = map.iter().next().expect("one entry");
        return Ok(SteinerTree { nodes: vec![SteinerNode { p: Point::new(x, y), fixed: layers, children: Vec::new() }], ..Default::default() });
    }
    let sorted: Vec<(i32, i32)> = map.keys().copied().collect();
    let (xs, ys): (Vec<i32>, Vec<i32>) = sorted.iter().copied().unzip();
    let driver_index = net
        .driver_access_point()
        .and_then(|d| sorted.iter().position(|&(x, y)| x == d.x && y == d.y))
        .unwrap_or(0);
    let branches = stt(&xs, &ys, driver_index, alpha);
    let points: Vec<Point> = branches.iter().map(|&(x, y, _)| Point::new(x, y)).collect();
    let mut adjacent = vec![Vec::new(); branches.len()];
    for (i, &(_, _, n)) in branches.iter().enumerate() {
        if i == n {
            continue;
        }
        adjacent[i].push(n);
        adjacent[n].push(i);
    }
    let has_degree1 = |start: usize| -> Result<bool, PatternError> {
        let mut index = start;
        for _ in 0..=branches.len() {
            if adjacent[index].len() != 1 {
                return Ok(false);
            }
            let next = adjacent[index][0];
            if points[index] != points[next] {
                return Ok(true);
            }
            index = next;
        }
        Err(PatternError::DegenerateSteinerTree { net: net.name.clone() })
    };
    let mut root = 0;
    for i in 0..points.len() {
        if has_degree1(i)? {
            root = i;
            break;
        }
    }
    let mut tree = SteinerTree { input: Some((xs, ys, driver_index)), branches: branches.clone(), root_branch: root, ..Default::default() };
    fn construct(tree: &mut SteinerTree, parent: Option<usize>, prev: Option<usize>, cur: usize, points: &[Point], adjacent: &[Vec<usize>], map: &AccessPointMap) -> usize {
        if let Some(p) = parent {
            if tree.nodes[p].p == points[cur] {
                for &next in &adjacent[cur] {
                    if Some(next) != prev {
                        construct(tree, Some(p), Some(cur), next, points, adjacent, map);
                    }
                }
                return p;
            }
        }
        tree.nodes.push(SteinerNode { p: points[cur], fixed: Interval::default(), children: Vec::new() });
        let me = tree.nodes.len() - 1;
        for &next in &adjacent[cur] {
            if Some(next) != prev {
                construct(tree, Some(me), Some(cur), next, points, adjacent, map);
            }
        }
        if let Some(&layers) = map.get(&(points[cur].x, points[cur].y)) {
            tree.nodes[me].fixed = layers;
        }
        if let Some(p) = parent {
            tree.nodes[p].children.push(me);
        }
        me
    }
    tree.root = construct(&mut tree, None, None, root, &points, &adjacent, map);
    Ok(tree)
}

// ---------------------------------------------------------------- routing DAG ----

/// A DAG node (`PatternRoutingNode`).
#[derive(Debug, Clone, PartialEq)]
pub struct DagNode {
    pub p: Point,
    pub index: usize,
    pub fixed: Interval,
    pub optional: bool,
    pub children: Vec<usize>,
    /// Per child, the alternative paths to it (1 straight, or 2 L-shaped through optional mids).
    pub paths: Vec<Vec<usize>>,
    pub costs: Vec<f64>,
    /// Per layer, per child: `(path index, layer)`.
    pub best_paths: Vec<Vec<(i32, i32)>>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Dag {
    pub nodes: Vec<DagNode>,
    pub root: usize,
}

impl Dag {
    fn add(&mut self, p: Point, fixed: Interval, optional: bool) -> usize {
        let index = self.nodes.len();
        self.nodes.push(DagNode { p, index, fixed, optional, children: Vec::new(), paths: Vec::new(), costs: Vec::new(), best_paths: Vec::new() });
        index
    }

    /// `constructPaths(start, end)`: a straight path when aligned, else two L-shapes — through
    /// `(end.x, start.y)` first, then `(start.x, end.y)` — as a NEW child's paths.
    fn construct_paths(&mut self, start: usize, end: usize) {
        self.construct_paths_at(start, end, None);
    }

    /// `constructPaths(start, end, child_index)`: with an index, the paths are ADDED to that
    /// child's alternatives (a detour); without one, a new child's are made.
    fn construct_paths_at(&mut self, start: usize, end: usize, child_index: Option<usize>) {
        let child_index = child_index.unwrap_or_else(|| {
            self.nodes[start].paths.push(Vec::new());
            self.nodes[start].paths.len() - 1
        });
        let (s, e) = (self.nodes[start].p, self.nodes[end].p);
        if s.x == e.x || s.y == e.y {
            self.nodes[start].paths[child_index].push(end);
        } else {
            for path_index in 0..2 {
                let mid = if path_index == 1 { Point::new(s.x, e.y) } else { Point::new(e.x, s.y) };
                let m = self.add(mid, Interval::default(), true);
                self.nodes[m].paths = vec![vec![end]];
                self.nodes[start].paths[child_index].push(m);
            }
        }
    }
}

/// `constructRoutingDAG`: the Steiner tree, node by node (a node, then its subtrees, then the paths
/// from its parent to it — the order the indices are handed out in).
pub fn construct_routing_dag(steiner: &SteinerTree) -> Dag {
    fn construct(dag: &mut Dag, dst: Option<usize>, st: &SteinerTree, s: usize) -> usize {
        let current = dag.add(st.nodes[s].p, st.nodes[s].fixed, false);
        for &child in &st.nodes[s].children {
            construct(dag, Some(current), st, child);
        }
        if let Some(d) = dst {
            dag.nodes[d].children.push(current);
            dag.construct_paths(d, current);
        }
        current
    }
    let mut dag = Dag::default();
    dag.root = construct(&mut dag, None, steiner, steiner.root);
    dag
}

// ---------------------------------------------------------------- detours (stage 3) ----

/// A node of a detour scaffold: a DAG node (none for the virtual root) and its scaffold children.
struct Scaffold {
    node: Option<usize>,
    children: Vec<usize>,
}

/// `constructDetours(congestion_view)`: for each run of the DAG along one direction that crosses
/// a congested edge (a SCAFFOLD), copies of the run shifted across that direction, as alternative
/// paths the DP may choose.
///
/// Upstream rules, all transcribed as written:
/// - scaffolds are grown bottom-up in the DAG's path order; a child's pending scaffold is handed to
///   a new top scaffold inside the loop over each child's PATHS (so once per path list);
/// - the shift range grows while the stems' total length stays within `trunk × max_detour_ratio`
///   (truncated to an int) — upward the test reads the stems at `high - 1`, not `high + 1`;
/// - the step widens until fewer than `target_detour_count` shifts remain on either side;
/// - a pinned node is copied (its pins stay) and joined to its shifted twin.
pub fn construct_detours(dag: &mut Dag, view: &super::grid_graph::View<bool>, sizes: [usize; 2], c: &Constants, log: &mut Log) {
    use super::grid_graph::view_check;
    let n0 = dag.nodes.len();
    let mut sc: Vec<Scaffold> = Vec::new();
    let mut scaffolds: [Vec<usize>; 2] = [Vec::new(), Vec::new()];
    let mut scaffold_nodes: [Vec<Option<usize>>; 2] = [vec![None; n0], vec![None; n0]];
    let mut visited = vec![false; n0];
    fn new_scaffold(sc: &mut Vec<Scaffold>, node: Option<usize>) -> usize {
        sc.push(Scaffold { node, children: Vec::new() });
        sc.len() - 1
    }
    #[allow(clippy::too_many_arguments)]
    fn build(dag: &Dag, node: usize, view: &super::grid_graph::View<bool>, visited: &mut [bool], sc: &mut Vec<Scaffold>, sn: &mut [Vec<Option<usize>>; 2], scaffolds: &mut [Vec<usize>; 2]) {
        if visited[node] {
            return;
        }
        visited[node] = true;
        let dir_of = |a: usize, b: usize| if dag.nodes[a].p.y == dag.nodes[b].p.y { H } else { V };
        if dag.nodes[node].optional {
            let path = dag.nodes[node].paths[0][0];
            build(dag, path, view, visited, sc, sn, scaffolds);
            let d = dir_of(node, path);
            if sn[d][path].is_none() && view_check(view, dag.nodes[node].p, dag.nodes[path].p) {
                sn[d][path] = Some(new_scaffold(sc, Some(path)));
            }
            return;
        }
        for child_paths in dag.nodes[node].paths.clone() {
            for &path in &child_paths {
                build(dag, path, view, visited, sc, sn, scaffolds);
                let d = dir_of(node, path);
                if dag.nodes[path].optional {
                    if sn[d][node].is_none() && view_check(view, dag.nodes[node].p, dag.nodes[path].p) {
                        sn[d][node] = Some(new_scaffold(sc, Some(node)));
                    }
                } else if view_check(view, dag.nodes[node].p, dag.nodes[path].p) {
                    if sn[d][node].is_none() {
                        sn[d][node] = Some(new_scaffold(sc, Some(node)));
                    }
                    let parent = sn[d][node].expect("set above");
                    match sn[d][path] {
                        None => {
                            let s = new_scaffold(sc, Some(path));
                            sc[parent].children.push(s);
                        }
                        Some(s) => {
                            sc[parent].children.push(s);
                            sn[d][path] = None;
                        }
                    }
                }
            }
            for &child in &dag.nodes[node].children {
                for d in 0..2 {
                    if let Some(s) = sn[d][child] {
                        let top = new_scaffold(sc, Some(node));
                        sc[top].children.push(s);
                        scaffolds[d].push(top);
                        sn[d][child] = None;
                    }
                }
            }
        }
    }
    build(dag, dag.root, view, &mut visited, &mut sc, &mut scaffold_nodes, &mut scaffolds);
    for d in 0..2 {
        if let Some(s) = scaffold_nodes[d][dag.root] {
            let top = new_scaffold(&mut sc, None);
            sc[top].children.push(s);
            scaffolds[d].push(top);
        }
    }

    fn trunk_and_stems(dag: &Dag, sc: &[Scaffold], s: usize, trunk: &mut Interval, stems: &mut Vec<i32>, d: usize, starting: bool) {
        if starting {
            if let Some(n) = sc[s].node {
                stems.push(dag.nodes[n].p.get(1 - d));
                trunk.update(dag.nodes[n].p.get(d));
            }
            for &child in &sc[s].children {
                trunk_and_stems(dag, sc, child, trunk, stems, d, false);
            }
            return;
        }
        let n = sc[s].node.expect("a scaffold below the top has a node");
        trunk.update(dag.nodes[n].p.get(d));
        if dag.nodes[n].fixed.is_valid() {
            stems.push(dag.nodes[n].p.get(1 - d));
        }
        for &tree_child in &dag.nodes[n].children {
            match sc[s].children.iter().find(|&&c| sc[c].node == Some(tree_child)) {
                Some(&c) => trunk_and_stems(dag, sc, c, trunk, stems, d, false),
                None => {
                    stems.push(dag.nodes[tree_child].p.get(1 - d));
                    trunk.update(dag.nodes[tree_child].p.get(d));
                }
            }
        }
    }
    let stem_length = |stems: &[i32], pos: i32| stems.iter().map(|s| (s - pos).abs()).sum::<i32>();

    fn build_detour(dag: &mut Dag, sc: &[Scaffold], s: usize, d: usize, shift: i32) -> usize {
        let tree_node = sc[s].node.expect("a scaffold below the top has a node");
        let (p, fixed) = (dag.nodes[tree_node].p, dag.nodes[tree_node].fixed);
        let shifted = if fixed.is_valid() {
            let dup = dag.add(p, fixed, false);
            let shifted = dag.add(p, Interval::default(), false);
            dag.nodes[shifted].p.set(1 - d, p.get(1 - d) + shift);
            dag.construct_paths(shifted, dup);
            shifted
        } else {
            let shifted = dag.add(p, Interval::default(), false);
            dag.nodes[shifted].p.set(1 - d, p.get(1 - d) + shift);
            shifted
        };
        for tree_child in dag.nodes[tree_node].children.clone() {
            match sc[s].children.iter().find(|&&c| sc[c].node == Some(tree_child)) {
                Some(&c) => {
                    let child = build_detour(dag, sc, c, d, shift);
                    dag.construct_paths(shifted, child);
                }
                None => dag.construct_paths(shifted, tree_child),
            }
        }
        shifted
    }

    for d in 0..2 {
        for &top in &scaffolds[d] {
            let mut trunk = Interval::default();
            let mut stems = Vec::new();
            trunk_and_stems(dag, &sc, top, &mut trunk, &mut stems, d, true);
            stems.sort();
            let first = sc[top].children[0];
            let trunk_pos = dag.nodes[sc[first].node.expect("scaffold child has a node")].p.get(1 - d);
            let original = stem_length(&stems, trunk_pos);
            let mut shift = Interval::point(trunk_pos);
            let max_increase = (f64::from(trunk.range()) * c.max_detour_ratio) as i32;
            let size = sizes[1 - d] as i32;
            while shift.low - 1 >= 0 && stem_length(&stems, shift.low - 1) - original <= max_increase {
                shift.low -= 1;
            }
            while shift.high + 1 < size && stem_length(&stems, shift.high - 1) - original <= max_increase {
                shift.high += 1;
            }
            let step = detour_step(trunk_pos, shift.low, shift.high, c.target_detour_count);
            shift = Interval::new(trunk_pos - (trunk_pos - shift.low) / step * step, trunk_pos + (shift.high - trunk_pos) / step * step);
            let mut pos = shift.low;
            while pos <= shift.high {
                let amount = pos - trunk_pos;
                pos += step;
                if amount == 0 {
                    continue;
                }
                if let Some(node) = sc[top].node {
                    let child_node = sc[first].node.expect("scaffold child has a node");
                    let at = dag.nodes[child_node].p.get(1 - d) + amount;
                    if at < 0 || at >= size {
                        continue;
                    }
                    for child_index in 0..dag.nodes[node].children.len() {
                        if dag.nodes[node].children[child_index] == child_node {
                            let shifted = build_detour(dag, &sc, first, d, amount);
                            dag.construct_paths_at(node, shifted, Some(child_index));
                        }
                    }
                } else {
                    let tree_node = sc[first].node.expect("scaffold child has a node");
                    if dag.nodes[tree_node].children.len() == 1 {
                        let p = dag.nodes[tree_node].p;
                        let at = p.get(1 - d) + amount;
                        if at < 0 || at >= size {
                            continue;
                        }
                        let shifted = dag.add(p, Interval::default(), false);
                        dag.nodes[shifted].p.set(1 - d, at);
                        dag.construct_paths_at(tree_node, shifted, Some(0));
                        for tree_child in dag.nodes[tree_node].children.clone() {
                            match sc[first].children.iter().find(|&&c| sc[c].node == Some(tree_child)) {
                                Some(&c) => {
                                    let child = build_detour(dag, &sc, c, d, amount);
                                    dag.construct_paths(shifted, child);
                                }
                                None => dag.construct_paths(shifted, tree_child),
                            }
                        }
                    } else {
                        log.push("[WARNING GRT-0277] The root doesn't have exactly one child.".into());
                    }
                }
            }
        }
    }
}

/// `constructDetours`' step: widened while the shifts left on the two sides at the NEXT step
/// still number the target or more (`>=`).
fn detour_step(trunk_pos: i32, low: i32, high: i32, target: i32) -> i32 {
    let mut step = 1;
    while (trunk_pos - low) / (step + 1) + (high - trunk_pos) / (step + 1) >= target {
        step += 1;
    }
    step
}

// ---------------------------------------------------------------- the DP ----

/// Everything a cost reads.
pub struct CostContext<'a> {
    pub grid: &'a GridGraph,
    pub design: &'a Design,
    pub constants: &'a Constants,
    pub cost_multiplier: f64,
}

/// `calculateRoutingCosts(node)`, memoised on the node's costs.
///
/// Upstream rule, per node: for each child, per layer of the child path's direction from the min
/// routing layer, the cheapest path (child's cost on that layer + the wire; out of the net's layer
/// range costs max), first minimum kept. Via costs accumulate upward from layer 0. The node's own
/// layers: its pins' interval, top clamped to the last layer and bottom raised to the net's min
/// layer — or the net's whole range for a Steiner point. For every low layer up to the interval's
/// bottom, walking up from it: each child's cheapest so far; once at or above the interval's top,
/// the via stack from low plus those children is a candidate for this layer. Then each layer
/// takes the layer above's cost where cheaper, top down.
fn calculate_routing_costs(dag: &mut Dag, node: usize, net: &GrNet, cx: &CostContext<'_>) {
    if !dag.nodes[node].costs.is_empty() {
        return;
    }
    let num_layers = cx.grid.num_layers;
    let num_children = dag.nodes[node].paths.len();
    let mut child_costs: Vec<Vec<(f64, i32)>> = vec![Vec::new(); num_children];
    for child_index in 0..num_children {
        child_costs[child_index] = vec![(f64::MAX, -1); num_layers];
        let child_paths = dag.nodes[node].paths[child_index].clone();
        for (path_index, &path) in child_paths.iter().enumerate() {
            calculate_routing_costs(dag, path, net, cx);
            let (np, pp) = (dag.nodes[node].p, dag.nodes[path].p);
            let direction = if np.x == pp.x { V } else { H };
            for layer in cx.grid.min_routing_layer..num_layers {
                if cx.grid.layer_directions[layer] != direction {
                    continue;
                }
                let mut cost = f64::MAX;
                if net.is_inside_layer_range(layer as i32) {
                    let wire = cx.grid.wire_cost(layer, np, pp, net.ndr_cost(layer), cx.constants, cx.cost_multiplier).expect("a path is aligned by construction");
                    cost = dag.nodes[path].costs[layer] + wire;
                }
                if cost < child_costs[child_index][layer].0 {
                    child_costs[child_index][layer] = (cost, path_index as i32);
                }
            }
        }
    }
    let n = &mut dag.nodes[node];
    n.costs = vec![f64::MAX; num_layers];
    n.best_paths = vec![Vec::new(); num_layers];
    if num_children > 0 {
        for layer in 1..num_layers {
            n.best_paths[layer] = vec![(-1, -1); num_children];
        }
    }
    let p = n.p;
    let mut via_costs = vec![0.0; num_layers];
    for layer in 1..num_layers {
        via_costs[layer] = via_costs[layer - 1] + cx.grid.via_cost(cx.design, layer - 1, p, &net.ndr_costs, cx.constants, cx.cost_multiplier);
    }
    let range = net.layer_range;
    let fixed = if n.fixed.is_valid() {
        Interval::new(n.fixed.low.min(num_layers as i32 - 1), n.fixed.high.max(range.min_layer))
    } else {
        Interval::new(range.min_layer, range.max_layer)
    };
    for low in 0..=fixed.low.max(-1) {
        let low = low as usize;
        let mut min_child_costs = vec![f64::MAX; num_children];
        let mut best_paths = vec![(-1, -1); num_children];
        for layer in low..num_layers {
            for c in 0..num_children {
                if child_costs[c][layer].0 < min_child_costs[c] {
                    min_child_costs[c] = child_costs[c][layer].0;
                    best_paths[c] = (child_costs[c][layer].1, layer as i32);
                }
            }
            if layer as i32 >= fixed.high {
                let mut cost = via_costs[layer] - via_costs[low];
                for &child_cost in &min_child_costs {
                    cost += child_cost;
                }
                if cost < n.costs[layer] {
                    n.costs[layer] = cost;
                    n.best_paths[layer] = best_paths.clone();
                }
            }
        }
        for layer in (low..num_layers.saturating_sub(1)).rev() {
            if n.costs[layer + 1] < n.costs[layer] {
                n.costs[layer] = n.costs[layer + 1];
                n.best_paths[layer] = n.best_paths[layer + 1].clone();
            }
        }
    }
}

/// `getRoutingTree(node, parent_layer)`: the DP's choice as a routing tree.
///
/// Upstream rule: the root takes the cheapest layer (first minimum). At each node the children
/// are grouped by the layer their best path is on: same-layer children hang off the node; lower
/// layers off a descending via chain, higher off an ascending one, a chain node per layer that has
/// children. The chain is then extended down and up to the node's pin interval.
fn get_routing_tree(dag: &Dag, node: usize, parent_layer: i32, tree: &mut GrTree, net: &GrNet, num_layers: usize) -> Result<usize, PatternError> {
    let mut parent_layer = parent_layer;
    if parent_layer == -1 {
        let mut min_cost = f64::MAX;
        for (layer, &c) in dag.nodes[dag.root].costs.iter().enumerate() {
            if c < min_cost {
                min_cost = c;
                parent_layer = layer as i32;
            }
        }
        if parent_layer < 0 {
            return Err(PatternError::NoFinitePath { net: net.name.clone() });
        }
    }
    let n = &dag.nodes[node];
    let routing_node = tree.add(parent_layer, n.p);
    let (mut lowest, mut highest) = (routing_node, routing_node);
    if !n.paths.is_empty() {
        let mut paths_on_layer: Vec<Vec<usize>> = vec![Vec::new(); num_layers];
        for child_index in 0..n.paths.len() {
            let (path_index, layer) = n.best_paths[parent_layer as usize].get(child_index).copied().unwrap_or((-1, -1));
            if path_index < 0 || layer < 0 {
                return Err(PatternError::NoFinitePath { net: net.name.clone() });
            }
            paths_on_layer[layer as usize].push(n.paths[child_index][path_index as usize]);
        }
        for &path in &paths_on_layer[parent_layer as usize] {
            let c = get_routing_tree(dag, path, parent_layer, tree, net, num_layers)?;
            tree.nodes[routing_node].children.push(c);
        }
        for layer in (0..parent_layer as usize).rev() {
            if !paths_on_layer[layer].is_empty() {
                let chain = tree.add(layer as i32, n.p);
                tree.nodes[lowest].children.push(chain);
                lowest = chain;
                for &path in &paths_on_layer[layer] {
                    let c = get_routing_tree(dag, path, layer as i32, tree, net, num_layers)?;
                    tree.nodes[lowest].children.push(c);
                }
            }
        }
        for layer in (parent_layer as usize + 1)..num_layers {
            if !paths_on_layer[layer].is_empty() {
                let chain = tree.add(layer as i32, n.p);
                tree.nodes[highest].children.push(chain);
                highest = chain;
                for &path in &paths_on_layer[layer] {
                    let c = get_routing_tree(dag, path, layer as i32, tree, net, num_layers)?;
                    tree.nodes[highest].children.push(c);
                }
            }
        }
    }
    if tree.nodes[lowest].layer > n.fixed.low {
        let c = tree.add(n.fixed.low, n.p);
        tree.nodes[lowest].children.push(c);
    }
    if tree.nodes[highest].layer < n.fixed.high {
        let c = tree.add(n.fixed.high, n.p);
        tree.nodes[highest].children.push(c);
    }
    Ok(routing_node)
}

/// `run()`: the DP from the root, then the tree it chose.
pub fn run(dag: &mut Dag, net: &GrNet, cx: &CostContext<'_>) -> Result<GrTree, PatternError> {
    let root = dag.root;
    calculate_routing_costs(dag, root, net, cx);
    let mut tree = GrTree::default();
    get_routing_tree(dag, root, -1, &mut tree, net, cx.grid.num_layers)?;
    // The arena's root is the first node added; keep it at index 0.
    debug_assert!(!tree.nodes.is_empty());
    Ok(tree)
}

/// What one net's pattern route leaves, for the trace.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NetRoute {
    pub selected: AccessPointMap,
    pub steiner: SteinerTree,
    pub dag: Dag,
    /// The access points were chosen by this route (`constructSteinerTree`), not by the maze.
    pub chose_access_points: bool,
    /// Stage 4: the sparsified graph, the search's states and the paths found.
    pub maze: Option<(super::maze_route::SparseGraph, Vec<super::maze_route::Solution>, Vec<usize>)>,
}

/// Stages 1 and 3 for one net: `constructSteinerTree` (with `selectAccessPoints`) →
/// `constructRoutingDAG` → with a congestion view, `constructDetours` → `run`. The tree is set on
/// the net; committing its usage is the caller's.
pub fn pattern_route_net(
    net: &mut GrNet,
    alpha: f32,
    stt: SteinerBuilder<'_>,
    cx: &CostContext<'_>,
    detours: Option<&super::grid_graph::View<bool>>,
    log: &mut Log,
) -> Result<NetRoute, PatternError> {
    let selected = select_access_points(net, cx.grid, log)?;
    let steiner = construct_steiner_tree(net, &selected, alpha, stt)?;
    let mut dag = construct_routing_dag(&steiner);
    if let Some(view) = detours {
        construct_detours(&mut dag, view, [cx.grid.x_size, cx.grid.y_size], cx.constants, log);
    }
    let tree = run(&mut dag, net, cx)?;
    net.routing_tree = Some(tree);
    net.adopted = false; // `setRoutingTree`
    Ok(NetRoute { selected, steiner, dag, chose_access_points: true, maze: None })
}

/// Stage 4's second half for one net: `setSteinerTree` (the maze's) → `constructRoutingDAG` →
/// `run`.
pub fn pattern_route_tree(net: &mut GrNet, steiner: SteinerTree, cx: &CostContext<'_>) -> Result<NetRoute, PatternError> {
    let mut dag = construct_routing_dag(&steiner);
    let tree = run(&mut dag, net, cx)?;
    net.routing_tree = Some(tree);
    net.adopted = false; // `setRoutingTree`
    Ok(NetRoute { selected: AccessPointMap::new(), steiner, dag, chose_access_points: false, maze: None })
}

#[cfg(test)]
pub(crate) mod tests_support {
    pub(crate) use super::tests::net;
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::cugr::design::{DesignFacts, TechLayerFacts};
    use crate::cugr::grid_graph::GridGraph;
    use crate::cugr::grnet::GrPoint;
    use crate::cugr::layers::MetalLayerFacts;
    use crate::cugr::{init, Cugr};

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

    /// A 6×6-gcell die (gridlines every 1000 up to 6001), m1 H / m2 V / m3 H, no nets.
    fn router() -> Cugr {
        let f = DesignFacts {
            dbu_per_micron: 1000,
            die: (0, 0, 6001, 6001),
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

    pub(crate) fn net(grid: &GridGraph, pins: &[&[(i32, i32, i32)]]) -> GrNet {
        let mut n = GrNet {
            index: 0,
            name: "n".into(),
            pin_access_points: pins.iter().map(|p| p.iter().map(|&(l, x, y)| GrPoint { layer: l, p: Point::new(x, y) }).collect()).collect(),
            bounding_box: Default::default(),
            driver_pin_index: -1,
            layer_range: crate::cugr::design::LayerRange { min_layer: 1, max_layer: 2 },
            slack: 0.0,
            preferred_aps: Default::default(),
            ndr_costs: vec![1.0; grid.num_layers],
            routing_tree: None,
            shape_ap_choices: Vec::new(),
            soft_ndr: false,
            adopted: false,
        };
        for p in &n.pin_access_points.clone() {
            for g in p {
                n.bounding_box.update(g.p);
            }
        }
        n
    }

    // Upstream rule (GridGraph `selectShapeAccessPoint`): strictly more accessible, or as
    // accessible and strictly nearer the bbox centre, wins; a tie keeps the FIRST. The chosen
    // location's layer interval spans every cell of the pin at that location.
    #[test]
    fn shape_access_point_prefers_access_then_distance_then_order() {
        let c = router();
        // Pin 0 on m1 (below the min routing layer: accessibility 1) at (0,0) and (5,5); pin 1 at
        // (4,4) on m1 and m2. Centre (2,2): (0,0) is 4 away, (5,5) 6 away.
        let mut n = net(&c.grid, &[&[(0, 0, 0), (0, 5, 5)], &[(0, 4, 4), (1, 4, 4)]]);
        let mut log = Vec::new();
        let map = select_access_points(&mut n, &c.grid, &mut log).unwrap();
        assert_eq!(n.preferred_aps[&0].0, Point::new(0, 0));
        // pin 1: the m2 cell counts two usable edges (at y=4 and below it) — more accessible than m1's 1
        assert_eq!(n.preferred_aps[&1].0, Point::new(4, 4));
        assert_eq!(n.preferred_aps[&1].1, Interval::new(0, 1), "both layers at the chosen cell");
        assert_eq!(map.len(), 2);
        // equal accessibility and distance: the first cell in the pin's order
        let mut n = net(&c.grid, &[&[(0, 1, 2), (0, 3, 2)], &[(0, 2, 0), (0, 2, 4)]]);
        select_access_points(&mut n, &c.grid, &mut log).unwrap();
        assert_eq!(n.preferred_aps[&0].0, Point::new(1, 2));
        assert_eq!(n.preferred_aps[&1].0, Point::new(2, 0));
    }

    // Upstream rule (GridGraph `selectShapeAccessPoint`): an edge counts toward accessibility only
    // with at least ONE whole track (`capacity >= 1`). A cell at the bbox centre whose two edges
    // hold half a track each (0) loses to a far cell with a whole one (1). No script in the suite
    // has a fractional edge at a chosen pin cell: this case is the only witness.
    #[test]
    fn a_fractional_edge_is_not_accessible() {
        let mut c = router();
        c.grid.graph_edges[1][2][2].capacity = 0.5;
        c.grid.graph_edges[1][2][1].capacity = 0.5;
        let mut n = net(&c.grid, &[&[(1, 2, 2), (1, 0, 0)], &[(1, 4, 4)]]);
        select_access_points(&mut n, &c.grid, &mut Vec::new()).unwrap();
        assert_eq!(n.preferred_aps[&0].0, Point::new(0, 0));
    }

    // Upstream rule (PatternRoute `calculateRoutingCosts`, the top-down fill): a layer takes the
    // layer above's cost and paths only when STRICTLY cheaper. Constructed tie (m1 H, m2 V, m3 H,
    // m4 V; unit via 4): a node pinned to m2 with a co-located child costing 13 on m2 and 5 on m4.
    // From low = m2, m2's own candidate is 0 + 13 = 13 via the m2 path, and m4's is 8 + 5 = 13 via
    // the m4 path — equal. m2 must keep ITS path. No script in the suite ties exactly.
    #[test]
    fn the_top_down_fill_keeps_a_layer_on_an_exact_tie() {
        let f = DesignFacts {
            dbu_per_micron: 1000,
            die: (0, 0, 6001, 6001),
            gcell_tile_size: 1000,
            layers: vec![layer("m1", 1, true), layer("m2", 2, false), layer("m3", 3, true), layer("m4", 4, false)],
            vias: Vec::new(),
            nets: Vec::new(),
            instance_shapes: Vec::new(),
            design_obstructions: Vec::new(),
            min_layer_for_clock: 0,
            max_layer_for_clock: 0,
        };
        let c = init(&f, &[], 2, 4).unwrap();
        let mut n = net(&c.grid, &[&[(1, 2, 2)]]);
        n.layer_range.max_layer = 3;
        let mut dag = Dag::default();
        let node = dag.add(Point::new(2, 2), Interval::point(1), false);
        let child = dag.add(Point::new(2, 2), Interval::default(), false);
        dag.nodes[child].costs = vec![f64::MAX, 13.0, f64::MAX, 5.0];
        dag.nodes[node].children.push(child);
        dag.nodes[node].paths = vec![vec![child]];
        let cx = CostContext { grid: &c.grid, design: &c.design, constants: &c.constants, cost_multiplier: 1.0 };
        calculate_routing_costs(&mut dag, node, &n, &cx);
        assert_eq!(dag.nodes[node].costs[1], 13.0);
        assert_eq!(dag.nodes[node].best_paths[1], vec![(0, 1)], "the tie keeps m2's own path");
    }

    // Upstream rule (PatternRoute `constructDetours`): the step widens while the next step still
    // leaves the target count or MORE shifts. Trunk 20 in 0..40, target 20: step 1 → 20/2 + 20/2
    // = 20, not fewer, so step 2 (6 + 6). No stage-3 range in the suite lands on the boundary.
    #[test]
    fn detour_step_widens_on_reaching_the_target() {
        assert_eq!(detour_step(20, 0, 40, 20), 2);
        assert_eq!(detour_step(20, 0, 39, 20), 1, "19 shifts left: stays at 1");
    }

    // Upstream rule (PatternRoute `constructPaths`): an unaligned pair gets TWO L-shapes, the mid
    // at (end.x, start.y) first, then (start.x, end.y); an aligned pair a single straight path.
    #[test]
    fn l_shape_mids_in_order() {
        let mut dag = Dag::default();
        let s = dag.add(Point::new(1, 1), Interval::default(), false);
        let e = dag.add(Point::new(4, 3), Interval::default(), false);
        let a = dag.add(Point::new(1, 5), Interval::default(), false);
        dag.construct_paths(s, e);
        dag.construct_paths(s, a);
        let mids = &dag.nodes[s].paths[0];
        assert_eq!((dag.nodes[mids[0]].p, dag.nodes[mids[1]].p), (Point::new(4, 1), Point::new(1, 3)));
        assert!(dag.nodes[mids[0]].optional);
        assert_eq!(dag.nodes[s].paths[1], vec![a], "aligned: straight");
    }

    // Upstream rule (PatternRoute `constructSteinerTree`): the root is the first branch of degree
    // 1; a branch at its parent's location is merged into the parent, its subtree hung there.
    #[test]
    fn steiner_root_and_colocated_merge() {
        let c = router();
        let n = net(&c.grid, &[&[(1, 0, 0)], &[(1, 3, 0)], &[(1, 3, 3)]]);
        let mut map = AccessPointMap::new();
        for &(x, y) in &[(0, 0), (3, 0), (3, 3)] {
            map.insert((x, y), Interval::point(1));
        }
        // branches: 0 (0,0) -> 3; 1 (3,0) -> 3; 2 (3,3) -> 3; 3 (3,0) Steiner -> itself
        let stt = |_: &[i32], _: &[i32], _: usize, _: f32| vec![(0, 0, 3), (3, 0, 3), (3, 3, 3), (3, 0, 3)];
        let t = construct_steiner_tree(&n, &map, 0.3, &stt).unwrap();
        assert_eq!(t.root_branch, 0);
        assert_eq!(t.nodes.len(), 3, "branch 3 is co-located with branch 1 and merged");
        let order: Vec<Point> = t.preorder().iter().map(|&i| t.nodes[i].p).collect();
        assert_eq!(order, vec![Point::new(0, 0), Point::new(3, 0), Point::new(3, 3)]);
        assert_eq!(t.nodes[t.root].fixed, Interval::point(1));
    }

    // Upstream rule (CUGR `sortNetIndices`): a STABLE sort by (slack, bbox half perimeter).
    #[test]
    fn neutral_order_is_stable_by_slack_then_hp() {
        let mut c = router();
        for (k, (slack, hp)) in [(0.0f32, 3), (0.0, 1), (-1.0, 9), (0.0, 1)].into_iter().enumerate() {
            let mut n = net(&c.grid, &[&[(1, 0, 0)], &[(1, hp, 0)]]);
            n.index = k;
            n.slack = slack;
            c.nets.push(n);
        }
        let mut order: Vec<usize> = (0..4).collect();
        c.sort_net_indices(&mut order).unwrap();
        assert_eq!(order, vec![2, 1, 3, 0]);
    }

    // Upstream rule (PatternRoute `calculateRoutingCosts` / `getRoutingTree`): the wire runs on the
    // only horizontal layer at or above the min (m3, index 2); the top-down fill gives layers 0-2
    // the SAME root cost, and the root takes the FIRST minimum — layer 0, the pin's own. So the
    // tree starts on m1 at the first pin, climbs a via chain to m3, runs to the second pin and
    // drops to its fixed layer.
    #[test]
    fn two_pin_route_climbs_to_the_routable_layer_and_back_down() {
        let mut c = router();
        let mut n = net(&c.grid, &[&[(0, 1, 2)], &[(0, 4, 2)]]);
        n.layer_range.max_layer = 2;
        c.nets.push(n);
        let stt = |x: &[i32], y: &[i32], _: usize, _: f32| vec![(x[0], y[0], 1), (x[1], y[1], 1)];
        let mut log = Vec::new();
        c.pattern_route(&mut vec![0], &[0.3], &stt, &mut log, None).unwrap();
        let t = c.nets[0].routing_tree.as_ref().unwrap();
        let s: Vec<(i32, i32, i32)> = t.preorder().iter().map(|&i| (t.nodes[i].layer, t.nodes[i].p.x, t.nodes[i].p.y)).collect();
        assert_eq!(s, vec![(0, 1, 2), (2, 1, 2), (2, 4, 2), (0, 4, 2)]);
        // committed: 3 m3 edges of one track each, plus the via flank deposits
        assert!((0..3).all(|x| c.grid.edge(2, 1 + x, 2).demand >= 1.0));
    }
}
