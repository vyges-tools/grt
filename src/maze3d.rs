// SPDX-License-Identifier: Apache-2.0
//! R18 — the three-dimensional maze pass, `mazeRouteMSMDOrder3D`.
//!
//! Runs twice after layer assignment (long edge window, then short) and only when the planar
//! overflow is zero. Measured before building: that gate passes on 67 of 76 runs per cost mode,
//! and the pass changes the routes of thousands of nets — it decides the final routes on most
//! designs.
//!
//! This module is built in pieces, each against its own capture. **R18a is the driver's control
//! sequence** and **R18b the 3D rip-up** (below it): which nets are walked, which are skipped, which edges fall in
//! the window, and how the retry passes repeat. The per-edge work is handed to a
//! [`Maze3DEdgeWork`], so the sequence is testable now and each later piece slots in behind it:
//!
//! | piece | the per-edge work, in the reference's order |
//! | --- | --- |
//! | R18b | `newRipup3DType3` — rip the edge up (declines only a zero-length edge) |
//! | R18c | the 3D heap primitives |
//! | R18d | `setupHeap3D` — seed both subtrees |
//! | R18e | the six-direction search |
//! | R18f | the backtrace to the crossing |
//! | R18g | the tree surgery (`updateRouteType13D/23D`), which yields the retry list |
//! | R18h | the commit, and `recoverEdge` when no legal path was found |

/// The final nets percentage the resistance-aware prelude sets, unless the user fixed one.
pub const FINAL_RES_AWARE_NETS_PERCENTAGE: f32 = 100.0;
/// The detour penalty the resistance-aware prelude sets for an incremental run…
pub const LOW_DETOUR_PENALTY: i32 = 5;
/// …and for the first run.
pub const HIGH_DETOUR_PENALTY: i32 = 15;

/// One entry of the net order the pass walks (`tree_order_pv_`), as it stands AFTER the
/// resistance-aware prelude has re-ordered it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrderedNet {
    pub net_id: usize,
    pub slack: f32,
    pub res_aware: bool,
}

/// One call's parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Maze3DCall {
    /// The window is `ripup_lb < len < ripup_ub`, exclusive at both ends.
    pub ripup_lb: i32,
    pub ripup_ub: i32,
    pub resistance_aware: bool,
    pub incremental: bool,
}

/// What the resistance-aware prelude sets before the walk (the reference also re-runs
/// `updateSlacks` and `netpinOrderInc` there — both transcribed in R16).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prelude {
    pub nets_percentage: Option<f32>,
    pub detour_penalty: Option<i32>,
}

/// ⚠️ Nothing is set outside resistance-aware mode. Inside it, the nets percentage is raised to
/// the final value only when the user did not fix one — 4 captured calls keep a fixed 30%.
pub fn prelude(call: &Maze3DCall, fixed_nets_percentage: bool) -> Prelude {
    if !call.resistance_aware {
        return Prelude { nets_percentage: None, detour_penalty: None };
    }
    Prelude {
        nets_percentage: (!fixed_nets_percentage).then_some(FINAL_RES_AWARE_NETS_PERCENTAGE),
        detour_penalty: Some(if call.incremental { LOW_DETOUR_PENALTY } else { HIGH_DETOUR_PENALTY }),
    }
}

/// How many nets of the order are walked.
///
/// ⛔ Outside resistance-aware mode only the first **90%**, computed as `size * 0.9` in double
/// and TRUNCATED into an `int` — 495 nets walk 445, not 446. The reference's comment: the rest is
/// left to the incremental optimisations.
pub fn end_index(order_len: usize, resistance_aware: bool) -> usize {
    if resistance_aware {
        order_len
    } else {
        (order_len as f64 * 0.9) as i32 as usize
    }
}

/// ⚠️ Positive-slack nets are skipped only in a resistance-aware, NON-incremental call — and
/// `>= 0`, so zero slack is skipped too.
pub fn skips_for_slack(call: &Maze3DCall, slack: f32) -> bool {
    call.resistance_aware && !call.incremental && slack >= 0.0
}

/// ⛔ Exclusive at BOTH ends: `len >= ub || len <= lb` is outside.
pub fn edge_in_window(len: i32, call: &Maze3DCall) -> bool {
    !(len >= call.ripup_ub || len <= call.ripup_lb)
}

/// Retry passes are allowed only in an incremental, resistance-aware call.
pub fn max_reroute_iter(call: &Maze3DCall) -> i32 {
    if call.incremental && call.resistance_aware {
        5
    } else {
        0
    }
}

/// What routing one edge produced, as the driver needs it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EdgeResult {
    /// Edges whose content a node shift overwrote — the retry candidates.
    pub retry: Vec<usize>,
    /// No legal path was found and the original route was kept.
    pub recovered: bool,
}

/// The per-edge work behind the driver — pieces R18b–h.
pub trait Maze3DEdgeWork {
    fn num_edges(&self, net: usize) -> usize;
    /// ⚠️ Read at the check, not up front: routing an earlier edge of the same net can change a
    /// later edge's length through the tree surgery.
    fn edge_len(&mut self, net: usize, edge: usize) -> i32;
    /// Called once per walked net; in a resistance-aware call it carries the net's flag, which
    /// the reference stores as the router-wide `resistance_aware_` before routing any edge.
    fn begin_net(&mut self, _net: usize, _res_aware: Option<bool>) {}
    fn route_edge(&mut self, net: usize, edge: usize) -> EdgeResult;
}

/// One step of the walk, in order — the driver's own trace.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Maze3DEvent {
    Net { net: usize, skipped: bool },
    Edge { edge: usize, len: i32, in_window: bool },
    Pass { iter: i32, stop: bool, retry: Vec<usize> },
}

/// The walk — the reference's driver, and nothing else.
///
/// Returns every step and the number of nets with a recovered edge (the reference warns GRT-183
/// with it).
pub fn maze_route_msmd_order_3d(
    call: &Maze3DCall,
    order: &[OrderedNet],
    work: &mut impl Maze3DEdgeWork,
) -> (Vec<Maze3DEvent>, usize) {
    let mut events = Vec::new();
    let max_iter = max_reroute_iter(call);
    let mut recovered_nets = 0;

    for entry in &order[..end_index(order.len(), call.resistance_aware)] {
        let net = entry.net_id;
        if skips_for_slack(call, entry.slack) {
            events.push(Maze3DEvent::Net { net, skipped: true });
            continue;
        }
        work.begin_net(net, call.resistance_aware.then_some(entry.res_aware));
        events.push(Maze3DEvent::Net { net, skipped: false });

        // The first pass visits every edge in index order; later passes only the retry list.
        let mut to_process: Vec<usize> = (0..work.num_edges(net)).collect();
        let mut iter = 0;
        let mut recovered = false;
        loop {
            let mut next: Vec<usize> = Vec::new();
            for &edge in &to_process {
                let len = work.edge_len(net, edge);
                let in_window = edge_in_window(len, call);
                events.push(Maze3DEvent::Edge { edge, len, in_window });
                if !in_window {
                    continue;
                }
                let r = work.route_edge(net, edge);
                recovered |= r.recovered;
                next.extend(r.retry);
            }
            // ⚠️ Sorted and de-duplicated before the stop test, as the reference does.
            next.sort_unstable();
            next.dedup();
            let stop = next.is_empty() || iter >= max_iter;
            events.push(Maze3DEvent::Pass { iter, stop, retry: next.clone() });
            if stop {
                break;
            }
            to_process = next;
            iter += 1;
        }
        if recovered {
            recovered_nets += 1;
        }
    }
    (events, recovered_nets)
}

// ─── R18b — `newRipup3DType3` ───────────────────────────────────────────────────────────────

use crate::full3d::Point3D;
use crate::softndr::UsageGrid;
use crate::spiral::MAX_CONNECTIONS;

/// The reference's `BIG_INT`, the "no edge" id and the Steiner node's starting bottom layer.
const BIG_INT: i32 = 1_000_000_000;

/// A tree node's connection bookkeeping, as the 3D rip-up reads and rewrites it.
///
/// ⛔ Fixed arrays with a count, as the reference holds them (`eID[10]`, `heights[10]`,
/// `conCNT`), not a growable list: removing shifts later entries down and decrements the count,
/// leaving a stale copy in the slot past it — and if the edge is not found at all, the count still
/// drops, discarding whatever sat in the last slot. A `Vec` would express neither.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeConnections {
    pub e_id: [i32; MAX_CONNECTIONS],
    pub heights: [i16; MAX_CONNECTIONS],
    pub con_cnt: i16,
    pub bot_layer: i16,
    pub top_layer: i16,
    pub l_id: i32,
    pub h_id: i32,
}

/// Remove one edge from a node and recompute its layer range and extreme edges — the reference's
/// `removeEdgeFromNode` lambda.
///
/// ⛔ A PIN starts from its pin layer with no edge ids; a STEINER node starts from `[BIG_INT, 0]`.
/// The comparisons are strict, so:
/// - an edge exactly at a pin's layer never claims `l_id` / `h_id` (1,496 captured);
/// - a Steiner node whose remaining edges are all on layer 0 keeps `h_id = BIG_INT` (53 captured);
/// - on a tie the FIRST remaining edge in list order keeps the id (~8,600 captured each way).
///
/// ⚠️ A Steiner node left with nothing becomes `bot = -1, top = 0` — NOT the `(num_layers, -1)`
/// "no layers" pair other stages test for. Never captured: Steiner nodes are never emptied.
pub fn remove_edge_from_node(node: &mut NodeConnections, edge_id: i32, pin_layer: Option<i32>) {
    let (mut bl, mut hl) = match pin_layer {
        Some(l) => (l, l),
        None => (BIG_INT, 0),
    };
    let (mut bid, mut hid) = (BIG_INT, BIG_INT);
    let n = node.con_cnt as usize;
    let mut consider = |h: i16, id: i32| {
        let h = i32::from(h);
        if bl > h {
            bl = h;
            bid = id;
        }
        if hl < h {
            hl = h;
            hid = id;
        }
    };
    for i in 0..n {
        if node.e_id[i] == edge_id {
            // Shift the rest down, considering each as it moves.
            for k in i + 1..n {
                node.e_id[k - 1] = node.e_id[k];
                node.heights[k - 1] = node.heights[k];
                consider(node.heights[k], node.e_id[k]);
            }
            break;
        }
        consider(node.heights[i], node.e_id[i]);
    }
    node.con_cnt -= 1;
    // ⚠️ The reference clamps the sentinel to its struct default so it does not truncate into 16
    // bits; the top is narrowed as is.
    node.bot_layer = if bl == BIG_INT { -1 } else { bl as i16 };
    node.l_id = bid;
    node.top_layer = hl as i16;
    node.h_id = hid;
}

/// "Maze ripup wrong" — a planar step that moves in both x and y. The reference aborts (GRT-122).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MazeRipupWrong {
    pub step: usize,
}

/// Rip one edge up in three dimensions — `newRipup3DType3`.
///
/// Returns `Ok(false)` without touching anything for a zero-length edge (the reference's "not
/// ripup for degraded edge"). ⚠️ Unreachable from R18's driver: its window already excludes
/// `len <= 0` — 0 refusals in 73,179 calls.
///
/// Otherwise removes the edge from both ALIAS endpoints, then gives back every PLANAR step's
/// usage (a step that changes layer is a via and gives back nothing):
/// - 2D committed usage by the net's edge cost — through the grid, which in the reference is
///   NDR-aware; measured inert: every applied delta equals the edge cost, NDR nets included;
/// - 3D usage on the step's layer by that layer's edge cost (3, 5 and 7 all captured).
///
/// Both are indexed at the step's LOWER endpoint.
#[allow(clippy::too_many_arguments)]
pub fn new_ripup_3d_type3(
    edge_id: usize,
    len: i32,
    (n1a, n2a): (usize, usize),
    grids: &[Point3D],
    routelen: i32,
    nodes: &mut [NodeConnections],
    pin_layer: &dyn Fn(usize) -> Option<i32>,
    edge_cost: i8,
    layer_edge_cost: &dyn Fn(i16) -> i8,
    grid: &mut dyn UsageGrid,
) -> Result<bool, MazeRipupWrong> {
    if len == 0 {
        return Ok(false);
    }
    remove_edge_from_node(&mut nodes[n1a], edge_id as i32, pin_layer(n1a));
    remove_edge_from_node(&mut nodes[n2a], edge_id as i32, pin_layer(n2a));

    for i in 0..routelen.max(0) as usize {
        let (a, b) = (grids[i], grids[i + 1]);
        if a.layer != b.layer {
            continue;
        }
        let lc = i32::from(layer_edge_cost(a.layer));
        if a.x == b.x {
            let ymin = a.y.min(b.y);
            grid.add_usage_v_2d(a.x, ymin, -i32::from(edge_cost));
            grid.add_usage_v_3d(a.layer, a.x, ymin, -lc);
        } else if a.y == b.y {
            let xmin = a.x.min(b.x);
            grid.add_usage_h_2d(xmin, a.y, -i32::from(edge_cost));
            grid.add_usage_h_3d(a.layer, xmin, a.y, -lc);
        } else {
            return Err(MazeRipupWrong { step: i });
        }
    }
    Ok(true)
}

// ─── R18d — `setupHeap3D` + `addNeighborPoints` ─────────────────────────────────────────────

/// A tree node as the 3D heap setup reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeedNode {
    pub x: i16,
    pub y: i16,
    /// Adjacent nodes and the tree edge reaching each, in the reference's stored order.
    pub neighbours: Vec<(usize, usize)>,
    pub stack_alias: usize,
    /// The layer range — read on the node's ALIAS, and read as the rip-up just left it.
    pub bot_layer: i16,
    pub top_layer: i16,
}

/// A tree edge as the 3D heap setup reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeedEdge {
    pub n1: usize,
    pub n2: usize,
    pub routelen: i32,
    pub maze_route: bool,
    pub grids: Vec<Point3D>,
}

/// A seeded cell: layer, x, y.
pub type Cell3 = (i16, i16, i16);

/// The two frontiers in push order, and every `corr_edge` write in order.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Heaps3D {
    pub src: Vec<Cell3>,
    pub dest: Vec<Cell3>,
    /// ⚠️ A cell written twice keeps the LAST write. The start node's own cells are never
    /// written — whatever an earlier edge left there stays.
    pub corr_edge: Vec<(Cell3, usize)>,
}

/// Seed both 3D frontiers — the reference's `setupHeap3D`.
///
/// ⛔ **Every seed has distance 0, so push ORDER is the whole tie-break** of the search that
/// follows. A cell can be pushed twice (a node that is also an interior route point); it is.
///
/// Differences from the 2D [`crate::setup_heap`], found by diffing the two reference functions:
///
/// | | 2D | 3D |
/// | --- | --- | --- |
/// | two-pin seeds | the pin's cell | ⛔ the pin's ACCESS LAYER only |
/// | a node | one cell | ⛔ every layer of its ALIAS's range, ascending |
/// | route interior | the cell | the cell at its own layer |
/// | a counted edge that is not a maze route | fatal | ⛔ its interior is skipped, silently |
///
/// Two further differences are inert by construction: the 2D walks share one visited set and the
/// 3D walks each start fresh (the subtrees are disjoint), and 2D marks the region for a two-pin net
/// too (which never reads it).
pub fn setup_heap_3d(
    num_terminals: usize,
    nodes: &[SeedNode],
    edges: &[SeedEdge],
    edge_id: usize,
    access_layers: (i16, i16),
    (region_x1, region_x2, region_y1, region_y2): (i32, i32, i32, i32),
) -> Heaps3D {
    let mut h = Heaps3D::default();
    let (n1, n2) = (edges[edge_id].n1, edges[edge_id].n2);
    if num_terminals == 2 {
        // ⚠️ The pin's access layer, looked up through the ALIAS node's pin — not the node's
        // layer range, and without any region test. On the corpus the two coincide: after the
        // rip-up a two-pin net's leaf pin has no edges left, so its range is exactly
        // `[access, access]` (16,374 of 16,374) — only a constructed case separates them.
        h.src.push((access_layers.0, nodes[n1].y, nodes[n1].x));
        h.dest.push((access_layers.1, nodes[n2].y, nodes[n2].x));
        return h;
    }
    let in_region = |x: i16, y: i16| {
        let (x, y) = (i32::from(x), i32::from(y));
        x >= region_x1 && x <= region_x2 && y >= region_y1 && y <= region_y2
    };
    add_neighbor_points(nodes, edges, n1, n2, &in_region, &mut h.src, &mut h.corr_edge);
    add_neighbor_points(nodes, edges, n2, n1, &in_region, &mut h.dest, &mut h.corr_edge);
    h
}

/// One subtree's seeds — the reference's `addNeighborPoints`.
///
/// ⛔ The START node is seeded at every layer of its alias's range with NO region test and no
/// `corr_edge` write. (A region test there would be equivalent: the driver's region always
/// contains both endpoints of the edge.) ⛔ The walk marks a node visited when it is DEQUEUED, never crosses
/// `stop_at`, and counts an edge by `routelen > 0` — not by its length.
fn add_neighbor_points(
    nodes: &[SeedNode],
    edges: &[SeedEdge],
    start: usize,
    stop_at: usize,
    in_region: &dyn Fn(i16, i16) -> bool,
    seeds: &mut Vec<Cell3>,
    corr: &mut Vec<(Cell3, usize)>,
) {
    let mut visited = vec![false; nodes.len()];
    let (sx, sy) = (nodes[start].x, nodes[start].y);
    let alias = &nodes[nodes[start].stack_alias];
    for l in alias.bot_layer..=alias.top_layer {
        seeds.push((l, sy, sx));
        visited[start] = true;
    }
    let mut queue = std::collections::VecDeque::from([start]);
    while let Some(cur) = queue.pop_front() {
        visited[cur] = true;
        for &(nbr, edge) in &nodes[cur].neighbours {
            if nbr == stop_at || visited[nbr] {
                continue;
            }
            let e = &edges[edge];
            if e.routelen > 0 {
                let (nx, ny) = (nodes[nbr].x, nodes[nbr].y);
                if in_region(nx, ny) {
                    let a = &nodes[nodes[nbr].stack_alias];
                    for l in a.bot_layer..=a.top_layer {
                        seeds.push((l, ny, nx));
                        corr.push(((l, ny, nx), edge));
                    }
                }
                if e.maze_route {
                    // Interior points only; the endpoints are the tree nodes themselves.
                    for p in &e.grids[1..e.routelen as usize] {
                        if in_region(p.x, p.y) {
                            seeds.push((p.layer, p.y, p.x));
                            corr.push(((p.layer, p.y, p.x), edge));
                        }
                    }
                }
            }
            queue.push_back(nbr);
        }
    }
}

// ─── R18e — the search ──────────────────────────────────────────────────────────────────────

/// How a cell was reached — the reference's `Direction`, in its declared order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir3 {
    North,
    East,
    South,
    West,
    Origin,
    Up,
    Down,
}

/// What one search reads, besides its seeds.
///
/// ⚠️ The move PRICES are inputs. In plain mode a wire is `1.0` and a via is `via_cost_` (1); in
/// resistance-aware mode both come from the technology's resistance tables — a separate function
/// (`getWireCost` / `getViaCost`), priced per layer and per layer pair, not a rule of the search.
pub struct Search3DInputs<'a> {
    pub num_layers: i16,
    /// `(x1, x2, y1, y2)`, inclusive.
    pub region: (i32, i32, i32, i32),
    /// Per layer: is its preferred direction horizontal?
    pub horizontal: &'a [bool],
    pub min_layer: i32,
    pub max_layer: i32,
    /// Does the planar 3D edge leaving `(x, y)` on `layer` — toward +x on a horizontal layer, +y
    /// on a vertical one — admit the net (`usage + layer_edge_cost <= cap`)?
    pub admits: &'a dyn Fn(i16, i32, i32) -> bool,
    pub wire_cost: &'a dyn Fn(i16) -> f32,
    pub via_cost: &'a dyn Fn(i16, i16) -> f32,
    pub original_len: i32,
    pub resistance_aware: bool,
    pub detour_penalty: i32,
}

/// One reached cell's final state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CellState {
    pub dist: i32,
    pub path_len: i32,
    /// `None` for a seed — the reference never writes a seed's parent.
    pub parent: Option<Cell3>,
    pub dir: Dir3,
}

/// A search's outcome.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Search3D {
    /// Every popped cell, in order.
    pub pops: Vec<Cell3>,
    /// The destination cell the search stopped on, or `None` when the heap ran dry (the reference
    /// then recovers the original route — 0 of 73,179 captured searches).
    pub crossing: Option<Cell3>,
    /// Every reached cell (distance below `BIG_INT`) with its final state.
    pub reached: Vec<(Cell3, CellState)>,
}

/// "Unable to update: position not found in 3D heap" — a relaxation improved a cell that had
/// already left the heap. The reference aborts (GRT-601..606).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotInHeap {
    pub cell: Cell3,
}

/// The 3D search — the `while` loop of `mazeRouteMSMDOrder3D`.
///
/// ⛔ **One-directional.** Only the source heap is searched; destination cells are merely MARKED,
/// and the search stops when the next cell to pop is one of them.
///
/// ⛔ Per pop, in this order: the two in-plane moves of the layer's PREFERRED direction only
/// (left then right on a horizontal layer, bottom then top on a vertical one), then down, then up.
/// A move is never taken back along the direction the cell was reached from.
///
/// ⛔ **The arithmetic is the reference's, not an idealisation**: the candidate distance is
/// `float` — `d + cost + penalty` — compared as `float` against the stored `int`, and stored
/// TRUNCATED. Resistance-aware wire prices reach ~8e8, beyond `float`'s exact-integer range, so
/// the conversion itself rounds.
pub fn maze_search_3d(
    inp: &Search3DInputs,
    src: &[Cell3],
    dest: &[Cell3],
) -> Result<Search3D, NotInHeap> {
    let (x1, x2, y1, y2) = inp.region;
    let (w, h) = ((x2 - x1 + 1) as usize, (y2 - y1 + 1) as usize);
    let idx = |(l, y, x): Cell3| (l as usize * h + (i32::from(y) - y1) as usize) * w + (i32::from(x) - x1) as usize;
    let cell = |i: usize| -> Cell3 {
        let (l, r) = (i / (w * h), i % (w * h));
        (l as i16, (r / w) as i32 as i16 + y1 as i16, (r % w) as i16 + x1 as i16)
    };
    let n = inp.num_layers as usize * w * h;
    let mut dist = vec![BIG_INT; n];
    let mut path_len = vec![BIG_INT; n];
    let mut parent: Vec<Option<Cell3>> = vec![None; n];
    let mut dir = vec![Dir3::Origin; n];
    let mut is_dest = vec![false; n];

    let mut heap: Vec<usize> = Vec::with_capacity(src.len());
    for &c in src {
        let i = idx(c);
        dist[i] = 0;
        path_len[i] = 0;
        dir[i] = Dir3::Origin;
        heap.push(i);
    }
    for &c in dest {
        is_dest[idx(c)] = true;
    }

    let mut pops = Vec::new();
    let mut crossing = None;
    let mut cur = heap[0];
    loop {
        if is_dest[cur] {
            crossing = Some(cell(cur));
            break;
        }
        pops.push(cell(cur));
        crate::maze::remove_min(&mut heap, &dist);
        let (l, y, x) = cell(cur);
        let (xi, yi) = (i32::from(x), i32::from(y));

        let planar = |to: Cell3, edge_x: i32, edge_y: i32, d: Dir3| -> Option<(Cell3, f32, i32, Dir3)> {
            let new_len = path_len[cur] + 1;
            let penalty = if new_len > inp.original_len && inp.resistance_aware {
                inp.detour_penalty as f32
            } else {
                0.0
            };
            let tmp = dist[cur] as f32 + (inp.wire_cost)(l) + penalty;
            let open = (inp.admits)(l, edge_x, edge_y)
                && inp.min_layer <= i32::from(l)
                && i32::from(l) <= inp.max_layer;
            open.then_some((to, tmp, new_len, d))
        };

        let from = dir[cur];
        let mut moves: Vec<(Cell3, f32, i32, Dir3)> = Vec::new();
        if inp.horizontal[l as usize] {
            if xi > x1 && from != Dir3::East {
                moves.extend(planar((l, y, x - 1), xi - 1, yi, Dir3::West));
            }
            if xi < x2 && from != Dir3::West {
                moves.extend(planar((l, y, x + 1), xi, yi, Dir3::East));
            }
        } else {
            if yi > y1 && from != Dir3::South {
                moves.extend(planar((l, y - 1, x), xi, yi - 1, Dir3::North));
            }
            if yi < y2 && from != Dir3::North {
                moves.extend(planar((l, y + 1, x), xi, yi, Dir3::South));
            }
        }
        // ⚠️ Vias: no admission test, no layer-range test, no detour penalty, and the path length
        // does not grow.
        if l > 0 && from != Dir3::Up {
            moves.push(((l - 1, y, x), dist[cur] as f32 + (inp.via_cost)(l, l - 1), path_len[cur], Dir3::Down));
        }
        if l < inp.num_layers - 1 && from != Dir3::Down {
            moves.push(((l + 1, y, x), dist[cur] as f32 + (inp.via_cost)(l, l + 1), path_len[cur], Dir3::Up));
        }
        // The relaxation, shared by all six moves: the candidate is compared, then stored.
        //
        // 🔑 All of a pop's moves are priced BEFORE any is relaxed. Equivalent to the reference's
        // interleaving: a relaxation writes only the NEIGHBOUR, never the popped cell whose
        // distance, length and direction the prices read.
        let mut relax = |nb: Cell3, tmp: f32, new_len: i32, d: Dir3,
                         heap: &mut Vec<usize>, dist: &mut Vec<i32>|
         -> Result<(), NotInHeap> {
            let j = idx(nb);
            let fresh = dist[j] >= BIG_INT;
            if fresh || (dist[j] as f32) > tmp {
                dist[j] = tmp as i32;
                path_len[j] = new_len;
                parent[j] = Some((l, y, x));
                dir[j] = d;
                if fresh {
                    heap.push(j);
                    let last = heap.len() - 1;
                    crate::maze::update_heap(heap, last, dist);
                } else {
                    // ⚠️ The FIRST occurrence — a seed can sit in the heap twice.
                    let pos = heap.iter().position(|&e| e == j).ok_or(NotInHeap { cell: nb })?;
                    crate::maze::update_heap(heap, pos, dist);
                }
            }
            Ok(())
        };

        for (nb, tmp, new_len, d) in moves {
            relax(nb, tmp, new_len, d, &mut heap, &mut dist)?;
        }

        if heap.is_empty() {
            break;
        }
        cur = heap[0];
    }

    let reached = (0..n)
        .filter(|&i| dist[i] < BIG_INT)
        .map(|i| (cell(i), CellState { dist: dist[i], path_len: path_len[i], parent: parent[i], dir: dir[i] }))
        .collect();
    Ok(Search3D { pops, crossing, reached })
}

// ─── R18f — the backtrace ───────────────────────────────────────────────────────────────────

/// What the backtrace leaves for the tree surgery.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Backtrace3D {
    /// From the subtree-1 end to the crossing: every parent walked, reversed, then the crossing.
    pub grids: Vec<Point3D>,
    /// The index of the LAST point still at the path's start position — the top of the via
    /// stack there. ⚠️ Counted as "points at the start position" minus one, so a path whose
    /// first step is planar has `head_room == 0`.
    pub head_room: usize,
    /// The first point's layer, and the layer at `head_room`.
    pub orig_layer: i16,
    pub last_layer: i16,
}

/// Why no path came back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Recovery {
    /// The search ran dry. The reference recovers the edge AND counts the net for GRT-183.
    Underflow,
    /// The crossing's own distance is 0 — a destination cell that is also a seed. ⚠️ The
    /// reference recovers the edge but does NOT count the net for GRT-183.
    ZeroDistance,
}

/// Walk back from the crossing — the backtrace in `mazeRouteMSMDOrder3D`.
///
/// Differences from the 2D [`crate::backtrace`], found by diffing the two reference routines: ONE
/// parent grid carrying the layer, where 2D keeps two chosen by the arrival direction; no
/// hyper-edge reflections; and a `head_room` scan the 2D code has no counterpart for.
///
/// ⛔ The walk stops at the first cell whose DISTANCE is 0 — not at the first seed. They agree only
/// because every move costs at least 1, so a cell a relaxation reached never stores 0.
pub fn backtrace_3d(
    crossing: Option<Cell3>,
    state: &dyn Fn(Cell3) -> CellState,
) -> Result<Backtrace3D, Recovery> {
    let cross = crossing.ok_or(Recovery::Underflow)?;
    if state(cross).dist == 0 {
        return Err(Recovery::ZeroDistance);
    }
    let mut cur = cross;
    let mut walked: Vec<Point3D> = Vec::new();
    while state(cur).dist != 0 {
        cur = state(cur).parent.expect("a cell with a non-zero distance was relaxed, so has a parent");
        walked.push(Point3D { x: cur.2, y: cur.1, layer: cur.0 });
    }
    let mut grids: Vec<Point3D> = walked.into_iter().rev().collect();
    grids.push(Point3D { x: cross.2, y: cross.1, layer: cross.0 });

    let (e1x, e1y) = (grids[0].x, grids[0].y);
    let mut head_room = 0;
    while head_room < grids.len() && grids[head_room].x == e1x && grids[head_room].y == e1y {
        head_room += 1;
    }
    // Always at least one point at the start, so this never underflows.
    head_room -= 1;
    Ok(Backtrace3D {
        orig_layer: grids[0].layer,
        last_layer: grids[head_room].layer,
        grids,
        head_room,
    })
}

// ─── R18g1 — `copyGrids3D` + `updateRouteType13D` ───────────────────────────────────────────

use crate::full3d::RouteType;

/// A tree edge as the 3D tree surgery reads and rewrites it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SurgeryEdge3D {
    pub n1: usize,
    pub n2: usize,
    pub route_type: RouteType,
    pub routelen: i32,
    pub len: i32,
    pub grids: Vec<Point3D>,
}

/// A tree node as the 3D tree surgery reads it — and, for the moved node, writes it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SurgeryNode3D {
    pub x: i16,
    pub y: i16,
    pub bot_layer: i16,
}

/// One edge's points, read out starting from `from` — the reference's `copyGrids3D`.
///
/// ⛔ A STEPLESS edge (`routelen <= 0`, whatever its type) yields one point: `from`'s own cell at
/// `from`'s BOTTOM layer. (The 2D `copyGrids` gates on the route type instead, and has no layer.)
pub fn copy_grids_3d(
    nodes: &[SurgeryNode3D],
    from: usize,
    edges: &[SurgeryEdge3D],
    edge_id: usize,
) -> Vec<Point3D> {
    let e = &edges[edge_id];
    if e.routelen <= 0 {
        let n = nodes[from];
        return vec![Point3D { x: n.x, y: n.y, layer: n.bot_layer }];
    }
    let taken = &e.grids[..=e.routelen as usize];
    if e.n1 == from {
        taken.to_vec()
    } else {
        taken.iter().rev().copied().collect()
    }
}

/// Why a type-1 shift aborts the run in the reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShiftError {
    /// GRT-187: the edge the node moves along has only one point.
    SinglePointEdge,
    /// GRT-171 / GRT-172: the new position is not on that edge.
    NotOnEdge,
    /// The type-2 merge would write past the end of the vector it sized — undefined behaviour in
    /// the reference (see [`update_route_type2_3d`]). Never captured.
    WriteBeyondEnd,
}

/// The node moved onto one of its own edges — `updateRouteType13D`.
///
/// Differences from the 2D [`crate::update_route_type1`], found by diffing the two:
/// - ⛔ the near half ends at the FIRST point at the new position, the far half starts at the LAST
///   — in 3D a via stack repeats the same x, y;
/// - ⛔ where the two lists join, a stack of VIAS fills the gap between the first list's last layer
///   and the second list's first layer;
/// - the node is moved HERE, not by the caller;
/// - a one-point first list is fatal.
///
/// Each rewritten edge is oriented by x, as in 2D: the endpoint with the smaller column first.
#[allow(clippy::too_many_arguments)]
pub fn update_route_type1_3d(
    nodes: &mut [SurgeryNode3D],
    n1: usize,
    a1: usize,
    a2: usize,
    (e1x, e1y): (i16, i16),
    edges: &mut [SurgeryEdge3D],
    edge_n1a1: usize,
    edge_n1a2: usize,
) -> Result<(), ShiftError> {
    // Both copies are taken before anything is written: the edges read are the edges rewritten.
    let g1 = copy_grids_3d(nodes, a1, edges, edge_n1a1);
    let g2 = copy_grids_3d(nodes, n1, edges, edge_n1a2);
    if g1.len() == 1 {
        return Err(ShiftError::SinglePointEdge);
    }
    let at_e1 = |p: &Point3D| p.x == e1x && p.y == e1y;
    let pos1 = g1.iter().position(at_e1).ok_or(ShiftError::NotOnEdge)?;
    let pos2 = g1.iter().rposition(at_e1).unwrap_or(0);

    let (a1x, a1y) = (nodes[a1].x, nodes[a1].y);
    let (a2x, a2y) = (nodes[a2].x, nodes[a2].y);

    // The near half: A1 as far as the FIRST point at E1.
    let head = &g1[..=pos1];
    let e = &mut edges[edge_n1a1];
    if a1x <= e1x {
        e.grids = head.to_vec();
        (e.n1, e.n2) = (a1, n1);
    } else {
        e.grids = head.iter().rev().copied().collect();
        (e.n1, e.n2) = (n1, a1);
    }
    e.len = i32::from((a1x - e1x).abs() + (a1y - e1y).abs());
    e.route_type = RouteType::MazeRoute;
    e.routelen = pos1 as i32;

    // The far half: from the LAST point at E1, a via fill, then the other edge past its first point.
    let last1 = g1[g1.len() - 1].layer;
    let (fx, fy, first2) = (g2[0].x, g2[0].y, g2[0].layer);
    let mut out: Vec<Point3D> = Vec::new();
    if e1x <= a2x {
        out.extend_from_slice(&g1[pos2..]);
        if g2.len() > 1 {
            if last1 > first2 {
                let mut l = last1 - 1;
                while l >= first2 {
                    out.push(Point3D { x: fx, y: fy, layer: l });
                    l -= 1;
                }
            } else if last1 < first2 {
                for l in last1 + 1..=first2 {
                    out.push(Point3D { x: fx, y: fy, layer: l });
                }
            }
        }
        out.extend_from_slice(&g2[1..]);
        (edges[edge_n1a2].n1, edges[edge_n1a2].n2) = (n1, a2);
    } else {
        out.extend(g2[1..].iter().rev().copied());
        if g2.len() > 1 {
            if last1 > first2 {
                for l in first2..last1 {
                    out.push(Point3D { x: fx, y: fy, layer: l });
                }
            } else if last1 < first2 {
                let mut l = first2;
                while l > last1 {
                    out.push(Point3D { x: fx, y: fy, layer: l });
                    l -= 1;
                }
            }
        }
        out.extend(g1[pos2..].iter().rev().copied());
        (edges[edge_n1a2].n1, edges[edge_n1a2].n2) = (a2, n1);
    }
    let e = &mut edges[edge_n1a2];
    e.route_type = RouteType::MazeRoute;
    e.routelen = out.len() as i32 - 1;
    e.grids = out;
    e.len = i32::from((a2x - e1x).abs() + (a2y - e1y).abs());

    nodes[n1].x = e1x;
    nodes[n1].y = e1y;
    Ok(())
}

// ─── R18g2 — `updateRouteType23D` ───────────────────────────────────────────────────────────

/// The node moved onto a DIFFERENT edge — `updateRouteType23D`.
///
/// Its own two edges merge into the landed-on edge's slot as the new (A1, A2); the landed-on edge
/// (C1, C2) splits at the new position into the node's two slots — near half to the FIRST point at
/// E1, far half from the LAST.
///
/// ⚠️ Unlike type 1: no orientation by x, no endpoint writes (the caller rewires five nodes), and
/// NO route type is set on any of the three slots.
///
/// ⛔ Faithful to the reference's VECTOR handling, because the slot's size is observable:
/// - a slot's old points are cleared only if it held a maze route (and, for the split halves, only
///   with steps); otherwise `resize` keeps them;
/// - a merge of a single point sets `routelen = 0` and resizes NOTHING.
///
/// ⛔ **Possible upstream defect, never captured (0 of 1,879 calls)**: the merge's descending via
/// fill is SIZED from the second list's first layer but LOOPS down to its SECOND point's layer.
/// When they differ, fewer points are written (the tail keeps `resize`'s defaults) or more (the
/// reference writes past the end — undefined). Reproduced for the first; an error for the second.
#[allow(clippy::too_many_arguments)]
pub fn update_route_type2_3d(
    nodes: &[SurgeryNode3D],
    n1: usize,
    (a1, a2): (usize, usize),
    (c1, c2): (usize, usize),
    (e1x, e1y): (i16, i16),
    edges: &mut [SurgeryEdge3D],
    (edge_n1a1, edge_n1a2, edge_c1c2): (usize, usize, usize),
) -> Result<(), ShiftError> {
    let g1 = copy_grids_3d(nodes, a1, edges, edge_n1a1);
    let g2 = copy_grids_3d(nodes, n1, edges, edge_n1a2);
    let g3 = copy_grids_3d(nodes, c1, edges, edge_c1c2);
    let dist = |a: usize, x: i16, y: i16| i32::from((nodes[a].x - x).abs() + (nodes[a].y - y).abs());
    let blank = Point3D { x: 0, y: 0, layer: 0 };

    // (A1, n1) + (n1, A2) -> the new (A1, A2), in the landed-on edge's slot.
    let slot = &mut edges[edge_c1c2];
    if slot.route_type == RouteType::MazeRoute {
        slot.grids.clear();
    }
    let mut len = g1.len() + g2.len() - 1;
    let a1a2 = dist(a1, nodes[a2].x, nodes[a2].y);
    if len == 1 {
        slot.routelen = 0;
        slot.len = a1a2;
    } else {
        let mut extra = 0;
        if g1.len() > 1 && g2.len() > 1 {
            extra = (g1[g1.len() - 1].layer - g2[0].layer).unsigned_abs() as usize;
            len += extra;
        }
        slot.grids.resize(len, blank);
        slot.routelen = len as i32 - 1;
        slot.len = a1a2;
        let mut cnt = 0usize;
        let mut write = |slot: &mut SurgeryEdge3D, p: Point3D| -> Result<(), ShiftError> {
            let at = slot.grids.get_mut(cnt).ok_or(ShiftError::WriteBeyondEnd)?;
            *at = p;
            cnt += 1;
            Ok(())
        };
        let mut start = 0;
        if g1.len() > 1 {
            start = 1;
            for &p in &g1 {
                write(slot, p)?;
            }
        }
        if extra > 0 {
            let last1 = g1[g1.len() - 1].layer;
            let (fx, fy) = (g2[0].x, g2[0].y);
            if last1 < g2[0].layer {
                for l in last1 + 1..=g2[0].layer {
                    write(slot, Point3D { x: fx, y: fy, layer: l })?;
                }
            } else {
                // ⛔ The reference's bound: the SECOND point's layer, not the first's.
                let mut l = last1 - 1;
                while l >= g2[1].layer {
                    write(slot, Point3D { x: fx, y: fy, layer: l })?;
                    l -= 1;
                }
            }
        }
        for &p in &g2[start..] {
            write(slot, p)?;
        }
    }

    // (C1, C2) -> (C1, n1) and (n1, C2), in the node's two slots.
    let at_e1 = |p: &Point3D| p.x == e1x && p.y == e1y;
    let pos2 = g3.iter().rposition(at_e1).ok_or(ShiftError::NotOnEdge)?;
    let pos1 = g3.iter().position(at_e1).expect("a last match implies a first");
    let split = |slot: &mut SurgeryEdge3D, part: &[Point3D], len: i32| {
        if slot.route_type == RouteType::MazeRoute && slot.routelen > 0 {
            slot.grids.clear();
        }
        slot.grids.resize(part.len(), blank);
        slot.grids.copy_from_slice(part);
        slot.routelen = part.len() as i32 - 1;
        slot.len = len;
    };
    split(&mut edges[edge_n1a1], &g3[..=pos1], dist(c1, e1x, e1y));
    split(&mut edges[edge_n1a2], &g3[pos2..], dist(c2, e1x, e1y));
    Ok(())
}

// ─── R18g3 — the surgery's wiring ───────────────────────────────────────────────────────────

/// A tree node carrying every field the 3D tree surgery reads or writes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Node3D {
    pub x: i16,
    pub y: i16,
    pub stack_alias: usize,
    pub assigned: bool,
    pub status: i16,
    /// `eID` / `heights` / `conCNT` and the layer range and extreme edges.
    pub conn: NodeConnections,
    /// `(nbr, edge)` pairs, `nbr_count` of them (at most 3).
    pub nbr: Vec<(usize, usize)>,
}

/// A tree edge carrying every field the 3D tree surgery reads or writes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Edge3D {
    pub n1: usize,
    pub n2: usize,
    pub n1a: usize,
    pub n2a: usize,
    pub len: i32,
    pub route_type: RouteType,
    pub routelen: i32,
    pub grids: Vec<Point3D>,
}

/// One net's tree, as the surgery sees it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Tree3D {
    pub num_terminals: usize,
    pub num_layers: i16,
    /// Each terminal's pin layer (`getPinL()[node_to_pin_idx[t]]`).
    pub pin_layers: Vec<i16>,
    pub nodes: Vec<Node3D>,
    pub edges: Vec<Edge3D>,
}

/// Split the edge next to a moved PIN — the reference's `splitEdge`, the same C++ function the 2D
/// pass calls ([`crate::split_edge`] is its 2D-typed transcription), here on the 3D tree.
///
/// `n2` is the pin that moved, `n1` the edge's other end. A new node takes `n2`'s place on the
/// edge (same position, `n2`'s alias), and a new zero-length edge joins it to `n2`.
///
/// ⚠️ The new edge's single grid point is `{n2x, n2y}` — its LAYER is the struct default, 0.
/// ⚠️ The new node's other fields are the struct defaults (`botL = topL = -1`, `hID = lID = -1`);
/// the caller's `setTreeNodesVariables` overwrites them straight after.
pub fn split_edge_3d(tree: &mut Tree3D, n1: usize, n2: usize, edge_n1n2: usize) -> usize {
    let (n2x, n2y) = (tree.nodes[n2].x, tree.nodes[n2].y);
    let new_node = tree.nodes.len();
    let new_edge = tree.edges.len();
    let alias = tree.nodes[n2].stack_alias;
    let (nbr, edge_n2_nbr) = if tree.nodes[n2].nbr[0].0 == n1 { tree.nodes[n2].nbr[1] } else { tree.nodes[n2].nbr[0] };

    let old = tree.nodes[n2].nbr.clone();
    tree.nodes[n2].nbr = old
        .iter()
        .filter(|&&(v, _)| v != n1)
        .map(|&(v, e)| if v == nbr { (new_node, new_edge) } else { (v, e) })
        .collect();

    for e in [edge_n2_nbr, edge_n1n2] {
        let edge = &mut tree.edges[e];
        if edge.n1 == n2 {
            edge.n1 = new_node;
            edge.n1a = alias;
        } else {
            edge.n2 = new_node;
            edge.n2a = alias;
        }
    }
    for who in [nbr, n1] {
        for slot in tree.nodes[who].nbr.iter_mut() {
            if slot.0 == n2 {
                slot.0 = new_node;
            }
        }
    }
    tree.edges.push(Edge3D {
        n1: new_node,
        n1a: alias,
        n2,
        n2a: tree.nodes[n2].stack_alias,
        len: 0,
        route_type: RouteType::MazeRoute,
        routelen: 0,
        grids: vec![Point3D { x: n2x, y: n2y, layer: 0 }],
    });
    tree.nodes.push(Node3D {
        x: n2x,
        y: n2y,
        stack_alias: alias,
        assigned: false,
        status: 0,
        conn: NodeConnections {
            e_id: [0; MAX_CONNECTIONS],
            heights: [0; MAX_CONNECTIONS],
            con_cnt: 0,
            bot_layer: -1,
            top_layer: -1,
            l_id: -1,
            h_id: -1,
        },
        nbr: vec![(nbr, edge_n2_nbr), (n2, new_edge), (n1, edge_n1n2)],
    });
    new_node
}

/// Recompute every node's layer bookkeeping — the reference's `setTreeNodesVariables`.
///
/// Reset (terminals to their pin layer, status 1; everything else open), alias each non-terminal
/// that sits on an earlier node's position to the FIRST node inserted there, then register every
/// POSITIVE-length edge at both alias ends by its first / last layer with strict extremes.
///
/// ⚠️ `eID` / `heights` past the new count keep their old values — the reference never clears them.
pub fn set_tree_nodes_variables(tree: &mut Tree3D) {
    let mut first_at: std::collections::HashMap<(i16, i16), usize> = std::collections::HashMap::new();
    for d in 0..tree.nodes.len() {
        let n = &mut tree.nodes[d];
        n.conn.top_layer = -1;
        n.conn.bot_layer = tree.num_layers;
        n.assigned = false;
        n.stack_alias = d;
        n.conn.con_cnt = 0;
        n.conn.h_id = BIG_INT;
        n.conn.l_id = BIG_INT;
        n.status = 0;
        let key = (n.x, n.y);
        if d < tree.num_terminals {
            n.conn.bot_layer = tree.pin_layers[d];
            n.conn.top_layer = tree.pin_layers[d];
            n.assigned = true;
            n.status = 1;
            first_at.entry(key).or_insert(d);
        } else if let Some(&first) = first_at.get(&key) {
            n.stack_alias = first;
        } else {
            first_at.insert(key, d);
        }
    }
    for k in 0..tree.edges.len() {
        if tree.edges[k].len <= 0 {
            continue;
        }
        let (n1a, n2a) = (tree.nodes[tree.edges[k].n1].stack_alias, tree.nodes[tree.edges[k].n2].stack_alias);
        let e = &mut tree.edges[k];
        e.n1a = n1a;
        e.n2a = n2a;
        let (l1, l2) = (e.grids[0].layer, e.grids[e.routelen as usize].layer);
        for (node, layer) in [(n1a, l1), (n2a, l2)] {
            let c = &mut tree.nodes[node].conn;
            let at = c.con_cnt as usize;
            c.heights[at] = layer;
            c.e_id[at] = k as i32;
            c.con_cnt += 1;
            if layer > c.top_layer {
                c.h_id = k as i32;
                c.top_layer = layer;
            }
            if layer < c.bot_layer {
                c.l_id = k as i32;
                c.bot_layer = layer;
            }
            tree.nodes[node].assigned = true;
        }
    }
}

/// Register the re-routed edge at an end that did not shift — `newUpdateNodeLayers`.
///
/// ⚠️ Top is tested before bottom, both strict, on the node as it stands.
pub fn new_update_node_layers(node: &mut NodeConnections, edge_id: usize, last_layer: i16) {
    let at = node.con_cnt as usize;
    node.heights[at] = last_layer;
    node.e_id[at] = edge_id as i32;
    node.con_cnt += 1;
    if node.top_layer < last_layer {
        node.top_layer = last_layer;
        node.h_id = edge_id as i32;
    }
    if node.bot_layer > last_layer {
        node.bot_layer = last_layer;
        node.l_id = edge_id as i32;
    }
}

/// What one edge's surgery reports back to the driver.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SurgeryOutcome {
    pub n1_shift: bool,
    pub n2_shift: bool,
    /// Edges whose points a node shift overwrote, in the order they were queued.
    pub retry: Vec<usize>,
    /// Every usage request, in order: (horizontal?, layer, x, y). Each is +edge cost on the 2D
    /// grid (NDR-aware in the reference) and +layer edge cost on the 3D grid.
    pub usage: Vec<(bool, i16, i16, i16)>,
}

/// Apply a node shift through the g1/g2 helpers on this tree's own types.
fn with_surgery_view<R>(tree: &mut Tree3D, f: impl FnOnce(&mut [SurgeryNode3D], &mut [SurgeryEdge3D]) -> R) -> R {
    let mut sn: Vec<SurgeryNode3D> = tree.nodes.iter().map(|n| SurgeryNode3D { x: n.x, y: n.y, bot_layer: n.conn.bot_layer }).collect();
    let mut se: Vec<SurgeryEdge3D> = tree.edges.iter().map(|e| SurgeryEdge3D {
        n1: e.n1, n2: e.n2, route_type: e.route_type, routelen: e.routelen, len: e.len, grids: e.grids.clone(),
    }).collect();
    let r = f(&mut sn, &mut se);
    for (n, s) in tree.nodes.iter_mut().zip(&sn) {
        (n.x, n.y) = (s.x, s.y);
    }
    for (e, s) in tree.edges.iter_mut().zip(se) {
        (e.n1, e.n2, e.route_type, e.routelen, e.len, e.grids) = (s.n1, s.n2, s.route_type, s.routelen, s.len, s.grids);
    }
    r
}

/// The two neighbours of `n` other than `other`, with their edges — the reference's A1/A2 (B1/B2)
/// choice: skip whichever of the first two slots holds `other`, else take the first two.
fn the_other_two(node: &Node3D, other: usize) -> ((usize, usize), (usize, usize)) {
    if node.nbr[0].0 == other {
        (node.nbr[1], node.nbr[2])
    } else if node.nbr[1].0 == other {
        (node.nbr[0], node.nbr[2])
    } else {
        (node.nbr[0], node.nbr[1])
    }
}

/// One end's half of the surgery: split at a moved pin, then shift a moved Steiner node (type 1 or
/// type 2, with type 2's five-node rewiring), or register the edge at an end that stayed put.
#[allow(clippy::too_many_arguments)]
fn surgery_end(
    tree: &mut Tree3D,
    me: &mut usize,
    other: usize,
    alias: usize,
    orig_pos: (i16, i16),
    e: (i16, i16),
    corr: usize,
    edge_n1n2: usize,
    last_layer: i16,
    out: &mut SurgeryOutcome,
) -> Result<bool, ShiftError> {
    let moved = e != orig_pos;
    if *me < tree.num_terminals && moved {
        *me = split_edge_3d(tree, other, *me, edge_n1n2);
        set_tree_nodes_variables(tree);
    }
    if !(*me >= tree.num_terminals && moved) {
        new_update_node_layers(&mut tree.nodes[alias].conn, edge_n1n2, last_layer);
        return Ok(false);
    }
    let n = *me;
    let (endpt1, endpt2) = (tree.edges[corr].n1, tree.edges[corr].n2);
    let ((mut a1, mut e_a1), (mut a2, mut e_a2)) = the_other_two(&tree.nodes[n], other);
    if endpt1 == n || endpt2 == n {
        if endpt1 == a2 || endpt2 == a2 {
            std::mem::swap(&mut a1, &mut a2);
            std::mem::swap(&mut e_a1, &mut e_a2);
        }
        with_surgery_view(tree, |sn, se| update_route_type1_3d(sn, n, a1, a2, e, se, e_a1, e_a2))?;
        out.retry.extend([e_a1, e_a2]);
        tree.nodes[n].assigned = true;
    } else {
        let (c1, c2, e_c1c2) = (endpt1, endpt2, corr);
        with_surgery_view(tree, |sn, se| update_route_type2_3d(sn, n, (a1, a2), (c1, c2), e, se, (e_a1, e_a2, e_c1c2)))?;
        out.retry.extend([e_a1, e_a2, e_c1c2]);
        tree.nodes[n].x = e.0;
        tree.nodes[n].y = e.1;
        tree.nodes[n].assigned = true;
        let (e_nc1, e_nc2, e_a1a2) = (e_a1, e_a2, e_c1c2);
        (tree.edges[e_nc1].n1, tree.edges[e_nc1].n2) = (c1, n);
        (tree.edges[e_nc2].n1, tree.edges[e_nc2].n2) = (n, c2);
        (tree.edges[e_a1a2].n1, tree.edges[e_a1a2].n2) = (a1, a2);
        tree.nodes[n].nbr[0] = (other, edge_n1n2);
        tree.nodes[n].nbr[1] = (c1, e_nc1);
        tree.nodes[n].nbr[2] = (c2, e_nc2);
        // Each of the four neighbours swaps ONE entry — the first that matches.
        for (who, was, now, edge) in [(a1, n, a2, e_a1a2), (a2, n, a1, e_a1a2), (c1, c2, n, e_nc1), (c2, c1, n, e_nc2)] {
            if let Some(slot) = tree.nodes[who].nbr.iter_mut().take(3).find(|s| s.0 == was) {
                *slot = (now, edge);
            }
        }
    }
    Ok(true)
}

/// The tree surgery after one edge's re-route — the driver's section from the backtrace to the end
/// of the edge's iteration.
///
/// ⛔ The reference's ORDER, which decides results: end 1 (split, shift or register) → the tail
/// room → end 2 (which reads end 1's possibly-new node id) → the route written from `head_room` to
/// `tail_room` → usage for every PLANAR step in that span → `setTreeNodesVariables` only if either
/// end shifted.
///
/// `corr(layer, x, y)` is the `corr_edge_3D` value the search's seeding left at a cell; the ends'
/// original positions are the ones read before the rip-up.
#[allow(clippy::too_many_arguments)]
pub fn tree_surgery_3d(
    tree: &mut Tree3D,
    edge_id: usize,
    bt: &Backtrace3D,
    orig_positions: ((i16, i16), (i16, i16)),
    aliases: (usize, usize),
    corr: &dyn Fn(i16, i16, i16) -> usize,
) -> Result<SurgeryOutcome, ShiftError> {
    let g = &bt.grids;
    let cnt = g.len();
    let e1 = (g[0].x, g[0].y);
    let e2 = (g[cnt - 1].x, g[cnt - 1].y);
    let (mut n1, mut n2) = (tree.edges[edge_id].n1, tree.edges[edge_id].n2);
    let mut out = SurgeryOutcome::default();

    let lazy1 = corr(bt.orig_layer, e1.1, e1.0);
    out.n1_shift = surgery_end(tree, &mut n1, n2, aliases.0, orig_positions.0, e1, lazy1, edge_id, bt.last_layer, &mut out)?;

    // The tail room: the FIRST point of the via stack at the path's end.
    let orig_layer2 = g[cnt - 1].layer;
    let mut tail = cnt - 1;
    while tail > 0 && g[tail].x == e2.0 && g[tail].y == e2.1 {
        tail -= 1;
    }
    if tail < cnt - 1 {
        tail += 1;
    }
    let last_layer2 = g[tail].layer;
    let lazy2 = corr(orig_layer2, e2.1, e2.0);
    out.n2_shift = surgery_end(tree, &mut n2, n1, aliases.1, orig_positions.1, e2, lazy2, edge_id, last_layer2, &mut out)?;

    // The re-routed edge itself, trimmed to [head_room, tail].
    let newcnt = tail as i32 - bt.head_room as i32 + 1;
    let e = &mut tree.edges[edge_id];
    if e.route_type == RouteType::MazeRoute {
        e.grids.clear();
    }
    if newcnt > 0 {
        e.grids.resize(newcnt as usize, Point3D { x: 0, y: 0, layer: 0 });
    }
    e.route_type = RouteType::MazeRoute;
    e.routelen = newcnt - 1;
    e.len = i32::from((e1.0 - e2.0).abs() + (e1.1 - e2.1).abs());
    for i in 0..newcnt.max(0) as usize {
        e.grids[i] = g[bt.head_room + i];
    }
    // ⚠️ Anything planar that is not vertical is charged as HORIZONTAL — no diagonal test here.
    for i in bt.head_room..tail {
        if g[i].layer == g[i + 1].layer {
            if g[i].x == g[i + 1].x {
                out.usage.push((false, g[i].layer, g[i].x, g[i].y.min(g[i + 1].y)));
            } else {
                out.usage.push((true, g[i].layer, g[i].x.min(g[i + 1].x), g[i].y));
            }
        }
    }
    if out.n1_shift || out.n2_shift {
        set_tree_nodes_variables(tree);
    }
    Ok(out)
}

// ─── R18h — `recoverEdge` ───────────────────────────────────────────────────────────────────

/// "Trying to recover a 0-length edge" — the reference aborts (GRT-206).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecoverZeroLength;

/// Put back an edge the search could not re-route — `recoverEdge`, the inverse of
/// [`new_ripup_3d_type3`].
///
/// Re-registers the edge at both ALIAS ends (its first layer at `n1a`, its last at `n2a`, strict
/// extremes, `assigned` set) and re-requests usage for every PLANAR step, at the lower endpoint:
/// (horizontal?, layer, x, y), each +edge cost in 2D (NDR-aware in the reference) and +layer edge
/// cost in 3D. The route itself is untouched — the rip-up never changed it.
///
/// ⚠️ Two differences from its inverse: a step that moves in both x and y is SKIPPED here, where the
/// rip-up aborts on it; and a zero-length edge is fatal here, where the rip-up merely declines it.
///
/// Reached from two places in the driver: the search running dry (counted for GRT-183, never
/// captured) and a zero-distance crossing (NOT counted — 2 of 73,179 searches, both on
/// `overlapping_edges`).
pub fn recover_edge(tree: &mut Tree3D, edge_id: usize) -> Result<Vec<(bool, i16, i16, i16)>, RecoverZeroLength> {
    let e = &tree.edges[edge_id];
    if e.len == 0 {
        return Err(RecoverZeroLength);
    }
    let (n1a, n2a) = (e.n1a, e.n2a);
    let (first, last) = (e.grids[0].layer, e.grids[e.routelen as usize].layer);
    for (node, layer) in [(n1a, first), (n2a, last)] {
        let c = &mut tree.nodes[node].conn;
        let at = c.con_cnt as usize;
        c.heights[at] = layer;
        c.e_id[at] = edge_id as i32;
        c.con_cnt += 1;
        if layer > c.top_layer {
            c.h_id = edge_id as i32;
            c.top_layer = layer;
        }
        if layer < c.bot_layer {
            c.l_id = edge_id as i32;
            c.bot_layer = layer;
        }
        tree.nodes[node].assigned = true;
    }
    let g = &tree.edges[edge_id].grids;
    let mut usage = Vec::new();
    for i in 0..tree.edges[edge_id].routelen.max(0) as usize {
        let (a, b) = (g[i], g[i + 1]);
        if a.layer != b.layer {
            continue;
        }
        if a.x == b.x {
            usage.push((false, a.layer, a.x, a.y.min(b.y)));
        } else if a.y == b.y {
            usage.push((true, a.layer, a.x.min(b.x), a.y));
        }
    }
    Ok(usage)
}
