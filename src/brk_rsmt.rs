// SPDX-License-Identifier: Apache-2.0
//! Stages R5 and R7 — `gen_brk_RSMT`: build each net's Steiner tree and break it into segments.
//!
//! The router calls this twice. R5 is `gen_brk_RSMT(false, false, false, false, noADJ)`: build a
//! tree and emit its segments, nothing else. R7 is `gen_brk_RSMT(true, true, true, false, noADJ)`:
//! rip up the net's L-routes, build a congestion-aware tree, shift its edges off congestion, copy
//! it into the router's own tree, emit segments, and re-route every tree edge as an L.
//!
//! ⚠️ **`newType` is `false` at both call sites**, so the tree-edge rip-up (`newRipup`) branch of
//! the reference's body never runs; it is not transcribed. The old-segment rip-up is.
//!
//! ⚠️ **The tree comes from one of two builders, chosen per net by its routing alpha.** The
//! default alpha is 0.3, so almost every net takes the Steiner-tree engine's tree, which arrives
//! here injected. Only alpha ≤ 0 reaches the router's own flute path below — in the corpus, one
//! design (`set_routing_alpha 0.0`).
//!
//! Stages are in the reference's order and carry its names: [`flute_congest`], [`net_congestion`],
//! [`htree_suite`], [`coeff_adj`], [`flute_normal`], [`copy_st_tree`], [`gen_brk_rsmt`]; then the
//! helpers from elsewhere in the reference: [`edge_shift`], [`edge_shift_new`] (utility.cpp),
//! [`ripup_seg_l`] (RipUp.cpp), [`newroute_l`] (route.cpp).

use crate::estimate::{capacity_lower_bound, Usage2d, EstimateGrid, LShape};
use crate::graph2d::Graph2d;
use crate::ndr_cost::NdrCostNet;
use crate::lroute::{route_edge, EdgeRoute, TreeEdge, TreeNode};
use crate::ripup_route::{new_ripup, RoutedShape};
use crate::rsmt::{segments_from_tree, Branch, Segment, COEFF_V_DEFAULT, COEFF_V_NO_ADJUSTMENTS, ROUTER_FLUTE_ACCURACY};

/// The reference's `BIG_INT`.
const BIG_INT: i32 = 1e9 as i32;

/// A Steiner tree as the reference's `stt::Tree` holds it.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RsmtTree {
    /// The number of terminals, not the number of branches.
    pub deg: usize,
    /// ⚠️ On the flute path this is in the SCALED coordinates flute was called with — the
    /// branches are scaled back, the length is not.
    pub length: i64,
    pub branch: Vec<Branch>,
}

/// The pre-sorted FLUTE the Steiner-tree engine exports (`Flute::flutes(xs, ys, s, acc)`):
/// x-sorted coordinates, y-sorted coordinates, and `s`, the x-rank of each y-sorted pin.
pub type Flutes<'a> = &'a dyn Fn(&[i32], &[i32], &[usize], i32) -> RsmtTree;

/// The sorted pins [`flute_normal`] keeps per net (`gxs_`, `gys_`, `gs_`), which
/// [`flute_congest`] reads back.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SortedPins {
    pub xs: Vec<i32>,
    pub ys: Vec<i32>,
    pub s: Vec<usize>,
}

/// A segment in the router's list, with the bend its L-route took.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoutedSegment {
    pub seg: Segment,
    /// The reference's `Segment::xFirst`.
    pub x_first: bool,
}

/// One layer of the router's 3D capacities, indexed `[y][x]` like `h_edges_3D_[l][y][x]`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CapLayer {
    pub h: Vec<i32>,
    pub v: Vec<i32>,
}

/// The router's 3D edge capacities, which `getEdgeCapacity` sums over a net's layer range.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Caps3D {
    pub x_grid: usize,
    pub layers: Vec<CapLayer>,
}

impl Caps3D {
    /// `getEdgeCapacity(net, x, y, dir)` — the edge's capacity summed over layers
    /// `min_layer..=max_layer`, both ends inclusive.
    pub fn edge_capacity(&self, min_layer: usize, max_layer: usize, x: i32, y: i32, horizontal: bool) -> i32 {
        let i = y as usize * self.x_grid + x as usize;
        (min_layer..=max_layer)
            .map(|l| if horizontal { self.layers[l].h[i] } else { self.layers[l].v[i] })
            .sum()
    }
}

/// What the router knows about a net that these stages read.
#[derive(Debug, Clone, Copy)]
pub struct RsmtNet<'a> {
    pub pins_x: &'a [i32],
    pub pins_y: &'a [i32],
    /// The net's routing alpha, as the Steiner-tree builder reports it (`getAlpha`).
    pub alpha: f32,
    pub edge_cost: i8,
    pub min_layer: usize,
    pub max_layer: usize,
    /// `getLayerEdgeCost(l)` for `min_layer..=max_layer` — what the NDR-aware charge reads.
    pub layer_edge_cost: &'a [i8],
}

impl RsmtNet<'_> {
    /// The net as the NDR-aware charge sees it; `id` is its identity on an edge.
    pub fn ndr_net(&self, id: usize) -> NdrCostNet {
        NdrCostNet {
            id,
            edge_cost: self.edge_cost,
            min_layer: self.min_layer,
            max_layer: self.max_layer,
            layer_edge_cost: Some(self.layer_edge_cost.to_vec()),
            soft_ndr: false,
        }
    }
}

/// The grid state the congestion-driven call reads and writes.
pub struct BrkGrid<'a> {
    /// The 2D graph: estimated usage (read by the tree builders, written by the rip-up and
    /// re-route through the NDR-aware charge), used grids, 2D capacities.
    pub g: &'a mut Graph2d,
    /// The edge reduction (`red`) — `getEstUsageRed*` is `est_usage + red`.
    pub red_h: &'a dyn Fn(usize, usize) -> u16,
    pub red_v: &'a dyn Fn(usize, usize) -> u16,
    pub caps: &'a Caps3D,
    /// `h_capacity_` / `v_capacity_`.
    pub h_capacity: i32,
    pub v_capacity: i32,
    /// `via_cost_` — the router sets it to 0 before R5, so the via bias contributes nothing here.
    pub via_cost: f64,
}

/// The reference's `gen_brk_RSMT` flags, minus `newType` (false at both call sites).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BrkFlags {
    pub congestion_driven: bool,
    pub re_route: bool,
    pub gen_tree: bool,
    pub no_adj: bool,
}

/// Which builder produced a net's tree.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TreeKind {
    /// The Steiner-tree engine (alpha > 0).
    Stt,
    /// [`flute_normal`].
    Normal,
    /// [`flute_congest`].
    Congest,
}

/// A tree edge's route type — `RouteType`, in the reference's order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RouteKind {
    #[default]
    NoRoute = 0,
    LRoute = 1,
    ZRoute = 2,
    MazeRoute = 3,
}

/// A tree edge's route (`TreeEdge::route`): the type, the fields the L and Z shapes read, and the
/// walked-out maze route. ⚠️ Defaults are the reference's: `NoRoute`, `xFirst` false, `HVH` false,
/// `Zpoint -1`, no grids, `routelen` and `last_routelen` 0.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TreeRoute {
    pub kind: RouteKind,
    pub x_first: bool,
    pub hvh: bool,
    /// ⛔ `int16_t`.
    pub z_point: i16,
    /// The route's grid points, `routelen + 1` of them (valid for `MazeRoute`).
    pub grids: Vec<(i32, i32)>,
    /// The number of steps in `grids` (valid for `MazeRoute`).
    pub routelen: i32,
    /// `routelen` as `SaveLastRouteLen` last recorded it.
    pub last_routelen: i32,
    /// Each grid point's layer (`GPoint3D::layer`), parallel to `grids` — empty until layer
    /// assignment (R16) writes it.
    pub layers: Vec<i16>,
}

impl Default for TreeRoute {
    fn default() -> Self {
        TreeRoute { kind: RouteKind::NoRoute, x_first: false, hvh: false, z_point: -1, grids: Vec::new(), routelen: 0, last_routelen: 0, layers: Vec::new() }
    }
}

impl TreeRoute {
    /// The route as `newRipup` undoes it.
    pub fn shape(&self) -> RoutedShape {
        match self.kind {
            RouteKind::NoRoute => RoutedShape::None,
            RouteKind::LRoute => RoutedShape::L { x_first: self.x_first },
            RouteKind::ZRoute => RoutedShape::Z { hvh: self.hvh, z_point: self.z_point as i32 },
            RouteKind::MazeRoute => RoutedShape::Maze { grids: self.grids.clone(), routelen: self.routelen as usize },
        }
    }
}

/// The router's copy of a net's tree (`sttrees_[net]`), as [`copy_st_tree`] writes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StTree {
    pub num_terminals: usize,
    pub nodes: Vec<TreeNode>,
    /// Neighbours and the edge to each, `nbr_count[i]` of them valid.
    pub nbr: Vec<[usize; 3]>,
    pub edge: Vec<[usize; 3]>,
    pub nbr_count: Vec<usize>,
    pub edges: Vec<TreeEdge>,
    /// `node_to_pin_idx` for the terminals; `-1` where no pin sits at the node.
    pub node_to_pin_idx: Vec<i32>,
    /// Per edge, the route (`TreeEdge::route`) — every edge starts at the default.
    pub routes: Vec<TreeRoute>,
    /// The walk's node state (`topL`, `botL`, `assigned`, `stackAlias`, `hID`, `lID`, the alias's
    /// edge list) — empty until spiral routing (R9) resets it. `x`/`y`/`status` stay canonical in
    /// [`nodes`](Self::nodes); the walk copies its statuses back.
    pub walk: Vec<crate::spiral::SpiralNode>,
    /// Per edge, the endpoints' alias nodes (`n1a`, `n2a`) and `assigned`, as spiral routing
    /// registers them — empty until R9.
    pub edge_reg: Vec<crate::spiral::EdgeReg>,
}

/// Per-net router state that outlives one call.
#[derive(Debug, Clone)]
pub struct NetState {
    /// `seglist_[net]`. ⛔ **R7 appends to it without clearing**: the old segments are ripped up
    /// (their usage removed) but stay in the list, and the new tree's segments follow them.
    pub seglist: Vec<RoutedSegment>,
    pub sorted: Option<SortedPins>,
    pub tree: Option<StTree>,
    /// `FrNet::slack_` — ⛔ MUTATED by the maze phase: the congestion ordering stamps de-prioritised
    /// nets with `f32::MAX`. Starts at the sentinel `ceil(lowest float)`.
    pub slack: f32,
    /// `FrNet::is_critical_` — set by the rip-up gate's critical arm, never cleared.
    pub critical: bool,
    /// The net's layer range as `assignEdge` leaves it (`setMinLayer`/`setMaxLayer`), or `None`
    /// while it is still the net's own. ⛔ The widening PERSISTS: later edges, and the 3D passes,
    /// read the widened range.
    pub layer_range: Option<(usize, usize)>,
    /// The net's tree in three dimensions, as layer assignment (R16) leaves it — nodes with their
    /// connection arrays (`eID`, `heights`), edges with layered grids. ⛔ From R16 on this is the
    /// canonical tree: the 3D passes rewrite it, and [`tree`](Self::tree) stays as R16 left it.
    pub tree3d: Option<crate::maze3d::Tree3D>,
    /// `FrNet::is_res_aware_` — set by `updateSlacks` (a survivor clock or NDR net, or a marked
    /// candidate), never cleared within a run.
    pub res_aware: bool,
    /// `FrNet::resistance_` — written only for a net that survived `updateSlacks`' skip rules; ⚠️ a
    /// net skipped later keeps the value an earlier call wrote.
    pub resistance: f32,
    /// `FrNet::net_length_` — written by every `updateSlacks` call, for every net.
    pub net_length: i32,
    /// `FrNet::is_soft_ndr_` — set by the congestion loop's soft-NDR demotion, never cleared.
    pub soft_ndr: bool,
}

impl Default for NetState {
    fn default() -> Self {
        NetState {
            seglist: Vec::new(),
            sorted: None,
            tree: None,
            slack: crate::ripup::SLACK_SENTINEL,
            critical: false,
            layer_range: None,
            tree3d: None,
            res_aware: false,
            resistance: 0.0,
            net_length: 0,
            soft_ndr: false,
        }
    }
}

/// What one net went through, returned so a divergence can be pinned to its stage.
#[derive(Debug, Clone, PartialEq)]
pub struct NetRecord {
    pub net: usize,
    pub kind: TreeKind,
    /// The coefficient passed to the flute path, when it ran.
    pub coeff_v: Option<f32>,
    /// `HTreeSuite`, when it was evaluated.
    pub htree: Option<bool>,
    /// `netCongestion`, when it was evaluated.
    pub congested: Option<bool>,
    /// `edgeShiftNew`'s return, when it ran.
    pub shifts: Option<i32>,
    /// The tree after any edge shifting, before it is copied.
    pub tree: RsmtTree,
    /// The router's copy as `copyStTree` left it — before the re-route marks its node statuses.
    pub copied: Option<StTree>,
}

/// The totals the reference reports under its `rsmt` debug level.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BrkSummary {
    pub wirelength: i64,
    pub wirelength1: i64,
    pub total_num_seg: usize,
    pub num_shift: i32,
    pub nets: Vec<NetRecord>,
}

/// `copyStTree` refuses a tree it cannot hold.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CopyTreeError {
    /// GRT-188 "Invalid number of node neighbors." — a node with more than three.
    InvalidNeighbors,
    /// GRT-189 "Failure in copy tree." — the edges do not number `nodes - 1`.
    EdgeCount { edges: usize, nodes: usize },
}

/// `mapxy` — binary search `nxs` for `nx` and return the matching `xs`, or `-1`.
///
/// ⚠️ `nxs` need not be strictly increasing (a zero-length step repeats a value); the search
/// returns whichever match it lands on first, as the reference's does.
pub fn mapxy(nx: i32, xs: &[i32], nxs: &[i32], d: usize) -> i32 {
    let (mut min, mut max) = (0i32, d as i32 - 1);
    while min <= max {
        let mid = (min + max) / 2;
        if nx == nxs[mid as usize] {
            return xs[mid as usize];
        }
        if nx < nxs[mid as usize] {
            max = mid - 1;
        } else {
            min = mid + 1;
        }
    }
    -1
}

/// The degree-2 and degree-3 trees both flute paths build without calling flute.
///
/// ⚠️ Degree 3's Steiner point is the MEDIAN x and median y, and its length the bounding box's
/// half-perimeter. Every branch points at the Steiner point, which points at itself.
fn small_tree(x: &[i32], y: &[i32]) -> RsmtTree {
    if x.len() == 2 {
        return RsmtTree {
            deg: 2,
            length: ((x[0] - x[1]).abs() + (y[0] - y[1]).abs()) as i64,
            branch: vec![Branch { x: x[0], y: y[0], n: 1 }, Branch { x: x[1], y: y[1], n: 1 }],
        };
    }
    let mut xs = [x[0], x[1], x[2]];
    xs.sort();
    let mut ys = [y[0], y[1], y[2]];
    ys.sort();
    RsmtTree {
        deg: 3,
        length: ((xs[2] - xs[0]).abs() + (ys[2] - ys[0]).abs()) as i64,
        branch: vec![
            Branch { x: x[0], y: y[0], n: 3 },
            Branch { x: x[1], y: y[1], n: 3 },
            Branch { x: x[2], y: y[2], n: 3 },
            Branch { x: xs[1], y: ys[1], n: 3 },
        ],
    }
}

/// `fluteCongest` — flute over coordinates stretched by each gap's congestion, mapped back.
///
/// ⚠️ **Reads the SORTED pins [`flute_normal`] stored for the net**, not the pins it is given;
/// only degree 2 and 3 use the pins directly. ⛔ Never reached in the corpus — `netCongestion` is
/// false for every net there — so it is pinned by constructed cases only.
///
/// ⛔ Every step keeps the reference's types: the usage sums are `int += double` (truncated per
/// add), the scale factor is `float`, and `x_seg *= factor` truncates back to `int`.
pub fn flute_congest(
    x: &[i32],
    y: &[i32],
    sorted: &SortedPins,
    acc: i32,
    coeff_v: f32,
    grid: &BrkGrid<'_>,
    flutes: Flutes<'_>,
) -> RsmtTree {
    let coeff_h: f32 = 1.0;
    let d = x.len();
    if d <= 3 {
        return small_tree(x, y);
    }
    let (xs, ys, s) = (&sorted.xs[..d], &sorted.ys[..d], &sorted.s[..d]);
    let mut x_seg: Vec<i32> = (0..d - 1).map(|i| (xs[i + 1] - xs[i]) * 100).collect();
    let mut y_seg: Vec<i32> = (0..d - 1).map(|i| (ys[i + 1] - ys[i]) * 100).collect();
    let height = ys[d - 1] - ys[0] + 1;
    let width = xs[d - 1] - xs[0] + 1;
    let est_red_h = |x: i32, y: i32| grid.g.est.h(x as usize, y as usize) + (grid.red_h)(x as usize, y as usize) as f64;
    let est_red_v = |x: i32, y: i32| grid.g.est.v(x as usize, y as usize) + (grid.red_v)(x as usize, y as usize) as f64;

    for i in 0..d - 1 {
        let mut usage_h: i32 = 0;
        for k in ys[0]..=ys[d - 1] {
            for j in xs[i]..xs[i + 1] {
                usage_h = (usage_h as f64 + est_red_h(j, k)) as i32;
            }
        }
        if x_seg[i] != 0 && usage_h != 0 {
            let f = coeff_h * usage_h as f32 / ((xs[i + 1] - xs[i]) * height * grid.h_capacity) as f32;
            x_seg[i] = (x_seg[i] as f32 * f) as i32;
            x_seg[i] = x_seg[i].max(1);
        }
        let mut usage_v: i32 = 0;
        for j in ys[i]..ys[i + 1] {
            for k in xs[0]..=xs[d - 1] {
                usage_v = (usage_v as f64 + est_red_v(k, j)) as i32;
            }
        }
        if y_seg[i] != 0 && usage_v != 0 {
            let f = coeff_v * usage_v as f32 / ((ys[i + 1] - ys[i]) * width * grid.v_capacity) as f32;
            y_seg[i] = (y_seg[i] as f32 * f) as i32;
            y_seg[i] = y_seg[i].max(1);
        }
    }

    let mut nxs = vec![xs[0]; d];
    let mut nys = vec![ys[0]; d];
    for i in 0..d - 1 {
        nxs[i + 1] = nxs[i] + x_seg[i];
        nys[i + 1] = nys[i] + y_seg[i];
    }

    let mut t = flutes(&nxs, &nys, s, acc);
    for b in &mut t.branch {
        b.x = mapxy(b.x, xs, &nxs, d);
        b.y = mapxy(b.y, ys, &nys, d);
    }
    t
}

/// `netCongestion` — whether any edge under the net's CURRENT segments is at or over the net's
/// capacity.
///
/// ⚠️ Called after the rip-up, so the net's own usage is already gone; it walks each segment's
/// L the way it was routed (`xFirst`). The first full edge ends the search.
pub fn net_congestion(net: &RsmtNet<'_>, seglist: &[RoutedSegment], grid: &BrkGrid<'_>) -> bool {
    let cap = |x: i32, y: i32, h: bool| grid.caps.edge_capacity(net.min_layer, net.max_layer, x, y, h);
    for rs in seglist {
        let s = rs.seg;
        let (ymin, ymax) = (s.y1.min(s.y2), s.y1.max(s.y2));
        if rs.x_first {
            for i in s.x1..s.x2 {
                if grid.g.est.h(i as usize, s.y1 as usize) >= cap(i, s.y1, true) as f64 {
                    return true;
                }
            }
            for i in ymin..ymax {
                if grid.g.est.v(s.x2 as usize, i as usize) >= cap(s.x2, i, false) as f64 {
                    return true;
                }
            }
        } else {
            for i in ymin..ymax {
                if grid.g.est.v(s.x1 as usize, i as usize) >= cap(s.x1, i, false) as f64 {
                    return true;
                }
            }
            for i in s.x1..s.x2 {
                if grid.g.est.h(i as usize, s.y2 as usize) >= cap(i, s.y2, true) as f64 {
                    return true;
                }
            }
        }
    }
    false
}

/// The pins' bounding box as the reference scans it: the maxima start at 0, the minima at
/// `BIG_INT`.
fn pin_bbox(net: &RsmtNet<'_>) -> (i32, i32, i32, i32) {
    let (mut xmin, mut xmax, mut ymin, mut ymax) = (BIG_INT, 0, BIG_INT, 0);
    for (&x, &y) in net.pins_x.iter().zip(net.pins_y) {
        xmin = xmin.min(x);
        xmax = xmax.max(x);
        ymin = ymin.min(y);
        ymax = ymax.max(y);
    }
    (xmin, xmax, ymin, ymax)
}

/// `HTreeSuite` — a net five times wider than tall. Strict: exactly five times is not.
pub fn htree_suite(net: &RsmtNet<'_>) -> bool {
    let (xmin, xmax, ymin, ymax) = pin_bbox(net);
    5 * (ymax - ymin) < (xmax - xmin)
}

/// `coeffADJ` — the vertical coefficient for the congestion-driven flute: how much scarcer
/// vertical capacity is than horizontal, relative to usage, over the net's bounding box.
///
/// ⛔ The types are the reference's and each one matters: the capacities are `int`, the usages
/// `float` accumulated from `double` (rounded to `float` per add), the ratio `float`, and the
/// floor `max<double>(coef, 1.2)` compares in double and narrows back — so a floored result is
/// `1.2f`.
///
/// ⚠️ A degenerate box (one row or one column) scans its single line and then ignores it: the
/// coefficient is 1, and the floor lifts it to 1.2. The zero-denominator guard also gives 1.2.
// The degenerate arms sum a capacity they then discard, as the reference's do.
#[allow(unused_assignments)]
pub fn coeff_adj(net: &RsmtNet<'_>, grid: &BrkGrid<'_>) -> f32 {
    let (xmin, xmax, ymin, ymax) = pin_bbox(net);
    let cap = |x: i32, y: i32, h: bool| grid.caps.edge_capacity(net.min_layer, net.max_layer, x, y, h);
    let (mut hcap, mut vcap) = (0i32, 0i32);
    let (mut husage, mut vusage) = (0f32, 0f32);
    let coef: f32;
    if xmin == xmax {
        for j in ymin..ymax {
            vcap += cap(xmin, j, false);
            vusage = (vusage as f64 + grid.g.est.v(xmin as usize, j as usize)) as f32;
        }
        coef = 1.0;
    } else if ymin == ymax {
        for i in xmin..xmax {
            hcap += cap(i, ymin, true);
            husage = (husage as f64 + grid.g.est.h(i as usize, ymin as usize)) as f32;
        }
        coef = 1.0;
    } else {
        for j in ymin..=ymax {
            for i in xmin..xmax {
                hcap += cap(i, j, true);
                husage = (husage as f64 + grid.g.est.h(i as usize, j as usize)) as f32;
            }
        }
        for j in ymin..ymax {
            for i in xmin..=xmax {
                vcap += cap(i, j, false);
                vusage = (vusage as f64 + grid.g.est.v(i as usize, j as usize)) as f32;
            }
        }
        coef = if husage * vcap as f32 > 0.0 {
            (hcap as f32 * vusage) / (husage * vcap as f32)
        } else {
            1.2
        };
    }
    (coef as f64).max(1.2) as f32
}

/// `fluteNormal` — sort the pins, scale y by the coefficient, call flute, scale back.
///
/// ⛔ **The two sorts are DIFFERENT selection sorts**, and a stable sort reproduces neither on
/// ties. The x sort SWAPS the minimum into place (first minimum wins, strict `>`); the y sort
/// REPLACES — the minimum's slot is overwritten with the element at `i`, which is never written
/// back — and the last element is taken as it lies. `s[i]` is the x-rank of the i-th y-sorted
/// pin. From degree 1000 up both become `stable_sort`.
///
/// ⛔ **The scale is `int(100 * coeffV)` computed in `float`** (136 for 1.36, 120 for 1.2), and the
/// scale-back is integer division, truncating. The length is NOT scaled back.
///
/// Returns the sorted pins too (`gxs_`, `gys_`, `gs_`) — degree > 3 only — for
/// [`flute_congest`].
pub fn flute_normal(x: &[i32], y: &[i32], acc: i32, coeff_v: f32, flutes: Flutes<'_>) -> (RsmtTree, Option<SortedPins>) {
    let d = x.len();
    if d <= 3 {
        return (small_tree(x, y), None);
    }
    // `pt[i]` with its x-rank `o`; `ptp` is the permutation the sorts move.
    let mut o = vec![0usize; d];
    let mut ptp: Vec<usize> = (0..d).collect();

    if d < 1000 {
        for i in 0..d - 1 {
            let (mut minval, mut minidx) = (x[ptp[i]], i);
            for j in i + 1..d {
                if minval > x[ptp[j]] {
                    minval = x[ptp[j]];
                    minidx = j;
                }
            }
            ptp.swap(i, minidx);
        }
    } else {
        ptp.sort_by_key(|&p| x[p]);
    }
    let xs: Vec<i32> = ptp.iter().map(|&p| x[p]).collect();
    for (i, &p) in ptp.iter().enumerate() {
        o[p] = i;
    }

    let (mut ys, mut s) = (vec![0i32; d], vec![0usize; d]);
    if d < 1000 {
        for i in 0..d - 1 {
            let (mut minval, mut minidx) = (y[ptp[i]], i);
            for j in i + 1..d {
                if minval > y[ptp[j]] {
                    minval = y[ptp[j]];
                    minidx = j;
                }
            }
            ys[i] = y[ptp[minidx]];
            s[i] = o[ptp[minidx]];
            ptp[minidx] = ptp[i];
        }
        ys[d - 1] = y[ptp[d - 1]];
        s[d - 1] = o[ptp[d - 1]];
    } else {
        ptp.sort_by_key(|&p| y[p]);
        for i in 0..d {
            ys[i] = y[ptp[i]];
            s[i] = o[ptp[i]];
        }
    }

    let y_scale = (100.0f32 * coeff_v) as i32;
    let tmp_xs: Vec<i32> = xs.iter().map(|&v| v * 100).collect();
    let tmp_ys: Vec<i32> = ys.iter().map(|&v| v * y_scale).collect();
    let mut t = flutes(&tmp_xs, &tmp_ys, &s, acc);
    for b in &mut t.branch {
        b.x /= 100;
        b.y /= y_scale;
    }
    (t, Some(SortedPins { xs, ys, s }))
}

/// `copyStTree` — copy a tree into the router's node/edge form.
///
/// ⛔ Terminals get status **2**, Steiner nodes 0 — the status [`newroute_l`]'s via bias reads.
/// ⚠️ Each edge runs from its lower-x end (`x1 < x2`, else swapped: a vertical edge has the
/// PARENT first). Neighbour lists fill in branch order.
///
/// ⚠️ `node_to_pin_idx` finds the k-th pin at a terminal's position, where k counts terminals
/// seen so far at that position — so stacked pins map one-to-one in order.
pub fn copy_st_tree(rsmt: &RsmtTree, net: &RsmtNet<'_>) -> Result<StTree, CopyTreeError> {
    let d = rsmt.deg;
    let numnodes = rsmt.branch.len();
    let size_v = 2 * net.pins_x.len();
    let mut nbrcnt = vec![0usize; size_v.max(numnodes)];
    let mut nodes = vec![TreeNode { x: 0, y: 0, status: 0 }; numnodes];
    let mut nbr = vec![[0usize; 3]; numnodes];
    let mut edge = vec![[0usize; 3]; numnodes];
    let mut edges = Vec::with_capacity(numnodes.saturating_sub(1));

    for i in 0..numnodes {
        let (x1, y1, n) = (rsmt.branch[i].x, rsmt.branch[i].y, rsmt.branch[i].n);
        let (x2, y2) = (rsmt.branch[n].x, rsmt.branch[n].y);
        // ⛔ `int16_t`, as the reference's node holds them.
        nodes[i] = TreeNode { x: x1 as i16, y: y1 as i16, status: if i < d { 2 } else { 0 } };
        if n != i {
            // ⚠️ The reference writes the fourth neighbour out of bounds and then errors; this
            // errors first.
            if nbrcnt[i] >= 3 || nbrcnt[n] >= 3 {
                return Err(CopyTreeError::InvalidNeighbors);
            }
            let len = (x1 - x2).abs() + (y1 - y2).abs();
            let (n1, n2) = if x1 < x2 { (i, n) } else { (n, i) };
            let e = edges.len();
            edges.push(TreeEdge { n1, n2, len });
            nbr[i][nbrcnt[i]] = n;
            edge[i][nbrcnt[i]] = e;
            nbr[n][nbrcnt[n]] = i;
            edge[n][nbrcnt[n]] = e;
            nbrcnt[i] += 1;
            nbrcnt[n] += 1;
        }
    }

    let mut pos_count: std::collections::HashMap<(i16, i16), i32> = std::collections::HashMap::new();
    let node_to_pin_idx = (0..d)
        .map(|i| {
            let c = pos_count.entry((nodes[i].x, nodes[i].y)).or_insert(0);
            *c += 1;
            pin_idx_from_position(net, nodes[i].x as i32, nodes[i].y as i32, *c)
        })
        .collect();

    if edges.len() != numnodes - 1 {
        return Err(CopyTreeError::EdgeCount { edges: edges.len(), nodes: numnodes });
    }
    nbrcnt.truncate(numnodes);
    Ok(StTree { num_terminals: d, nodes, nbr, edge, nbr_count: nbrcnt, edges, node_to_pin_idx, routes: vec![TreeRoute::default(); numnodes - 1], walk: Vec::new(), edge_reg: Vec::new() })
}

/// `FrNet::getPinIdxFromPosition` — the index of the `count`-th pin at `(x, y)`, or `-1`.
pub fn pin_idx_from_position(net: &RsmtNet<'_>, x: i32, y: i32, count: i32) -> i32 {
    let mut cnt = 1;
    for (idx, (&px, &py)) in net.pins_x.iter().zip(net.pins_y).enumerate() {
        if x == px && y == py {
            if cnt == count {
                return idx as i32;
            }
            cnt += 1;
        }
    }
    -1
}

/// `gen_brk_RSMT` — the call sequence, and nothing else. Nets are visited in `net_ids` order.
///
/// Per net: rip up its old segments (R7), build its tree, copy it (R7), append its segments,
/// re-route its tree edges (R7).
#[allow(clippy::too_many_arguments)]
pub fn gen_brk_rsmt(
    flags: BrkFlags,
    net_ids: &[usize],
    nets: &[RsmtNet<'_>],
    state: &mut [NetState],
    grid: &mut BrkGrid<'_>,
    stt_tree: &dyn Fn(usize) -> RsmtTree,
    flutes: Flutes<'_>,
) -> Result<BrkSummary, CopyTreeError> {
    let mut sum = BrkSummary::default();
    for &id in net_ids {
        let net = &nets[id];
        let nn = net.ndr_net(id);
        if flags.re_route {
            ripup_net_segments(&state[id].seglist, &mut grid.g.for_net(&nn));
        }
        let mut rec = build_tree(flags, id, net, &mut state[id], grid, stt_tree, flutes);
        if flags.gen_tree {
            let copied = copy_st_tree(&rec.tree, net)?;
            rec.copied = Some(copied.clone());
            state[id].tree = Some(copied);
        }
        if flags.congestion_driven {
            sum.wirelength1 += state[id].tree.as_ref().map_or(0, |t| t.edges.iter().map(|e| e.len as i64).sum());
        }
        sum.wirelength += append_segments(&rec.tree, net.edge_cost, &mut state[id].seglist);
        sum.total_num_seg += state[id].seglist.len();
        if flags.re_route {
            let tree = state[id].tree.as_mut().expect("R7 copies the tree before re-routing it");
            newroute_l(tree, &nn, grid, false, true);
        }
        sum.num_shift += rec.shifts.unwrap_or(0);
        sum.nets.push(rec);
    }
    Ok(sum)
}

/// The rip-up loop: `ripupSegL` over every segment in the net's list.
fn ripup_net_segments<G: Usage2d + ?Sized>(seglist: &[RoutedSegment], est: &mut G) {
    for s in seglist {
        ripup_seg_l(est, s);
    }
}

/// The tree-building block of `gen_brk_RSMT`'s loop body: which builder, and with what
/// coefficient.
///
/// ⚠️ `noADJ || HTreeSuite(net)` short-circuits, so the H-tree test is not evaluated when
/// adjustments are off. ⚠️ Edge shifting needs degree > 3 by PIN count.
fn build_tree(
    flags: BrkFlags,
    id: usize,
    net: &RsmtNet<'_>,
    st: &mut NetState,
    grid: &BrkGrid<'_>,
    stt_tree: &dyn Fn(usize) -> RsmtTree,
    flutes: Flutes<'_>,
) -> NetRecord {
    let d = net.pins_x.len();
    let mut rec = NetRecord { net: id, kind: TreeKind::Stt, coeff_v: None, htree: None, congested: None, shifts: None, tree: RsmtTree::default(), copied: None };
    if net.alpha > 0.0 {
        rec.tree = stt_tree(id);
        return rec;
    }
    let mut coeff_v = COEFF_V_DEFAULT;
    if flags.congestion_driven {
        coeff_v = if flags.no_adj { COEFF_V_NO_ADJUSTMENTS } else { coeff_adj(net, grid) };
        let cong = net_congestion(net, &st.seglist, grid);
        rec.congested = Some(cong);
        let mut t = if cong {
            rec.kind = TreeKind::Congest;
            let sorted = st.sorted.as_ref().expect("fluteCongest reads the pins fluteNormal sorted");
            flute_congest(net.pins_x, net.pins_y, sorted, ROUTER_FLUTE_ACCURACY, coeff_v, grid, flutes)
        } else {
            rec.kind = TreeKind::Normal;
            let (t, sorted) = flute_normal(net.pins_x, net.pins_y, ROUTER_FLUTE_ACCURACY, coeff_v, flutes);
            if sorted.is_some() {
                st.sorted = sorted;
            }
            t
        };
        if d > 3 {
            rec.shifts = Some(edge_shift_new(&mut t, d, &grid.g.est));
        }
        rec.tree = t;
    } else {
        let h = !flags.no_adj && htree_suite(net);
        if !flags.no_adj {
            rec.htree = Some(h);
        }
        if flags.no_adj || h {
            coeff_v = COEFF_V_NO_ADJUSTMENTS;
        }
        rec.kind = TreeKind::Normal;
        let (t, sorted) = flute_normal(net.pins_x, net.pins_y, ROUTER_FLUTE_ACCURACY, coeff_v, flutes);
        if sorted.is_some() {
            st.sorted = sorted;
        }
        rec.tree = t;
    }
    rec.coeff_v = Some(coeff_v);
    rec
}

/// The segment-append loop: one x-ordered segment per non-degenerate branch, APPENDED to the
/// list. Returns the branches' total Manhattan length.
fn append_segments(t: &RsmtTree, edge_cost: i8, seglist: &mut Vec<RoutedSegment>) -> i64 {
    let ns = segments_from_tree(&t.branch, edge_cost);
    // ⚠️ A new segment's bend is unset (`xFirst` false) until the next L-routing pass sets it.
    seglist.extend(ns.segments.into_iter().map(|seg| RoutedSegment { seg, x_first: false }));
    ns.wirelength
}

/// `edgeShift` — slide horizontal and vertical Steiner-to-Steiner edges to the row or column of
/// least usage, one best move at a time, while a move still helps.
///
/// ⛔ **The neighbour table is read three-wide whatever the count**: `nbr` is zero-initialised,
/// so an unfilled slot names node 0 and its coordinate joins the shift range. Transcribed as a
/// flat `2 * pins × 3` table, as the reference's `multi_array` lays it out.
///
/// ⛔ The costs are `int += double` — truncated at EVERY add, not once at the end — and a tie on
/// cost keeps the lowest position; a tie on benefit keeps the first pair.
pub fn edge_shift(t: &mut RsmtTree, num_pins: usize, est: &EstimateGrid) -> i32 {
    let size_v = 2 * num_pins;
    let mut nbr = vec![0usize; size_v * 3];
    let mut nbr_cnt = vec![0usize; size_v];
    let deg = t.deg;
    let bc = t.branch.len();
    let eh = |x: i32, y: i32| est.h(x as usize, y as usize);
    let ev = |x: i32, y: i32| est.v(x as usize, y as usize);
    let add = |acc: &mut i32, v: f64| *acc = (*acc as f64 + v) as i32;

    let root = (deg..bc).find(|&i| t.branch[i].n == i).unwrap_or(0);
    for i in 0..deg {
        let n = t.branch[i].n;
        assert!(n >= deg && n < bc, "GRT-149 Invalid access to nbrCnt vector");
        nbr[n * 3 + nbr_cnt[n]] = i;
        nbr_cnt[n] += 1;
    }
    for i in deg..bc {
        if i != root {
            let n = t.branch[i].n;
            nbr[i * 3 + nbr_cnt[i]] = n;
            nbr_cnt[i] += 1;
            nbr[n * 3 + nbr_cnt[n]] = i;
            nbr_cnt[n] += 1;
        }
    }

    // One neighbour's L cost with the shifted end at `(sx, sy)`: the cheaper of its two bends.
    let side_cost = |(sx1, bx1): (i32, i32), (sy1, by1): (i32, i32)| -> i32 {
        let (mut c1, mut c2) = (0i32, 0i32);
        for m in sx1..bx1 {
            add(&mut c1, eh(m, sy1));
            add(&mut c2, eh(m, by1));
        }
        for m in sy1..by1 {
            add(&mut c1, ev(bx1, m));
            add(&mut c2, ev(sx1, m));
        }
        c1.min(c2)
    };
    let order = |a: i32, b: i32| if a < b { (a, b) } else { (b, a) };

    let mut num_shift = 0;
    let mut best_benefit = BIG_INT;
    while best_benefit > 0 {
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for i in deg..bc {
            let n = t.branch[i].n;
            let (bi, bn) = (t.branch[i], t.branch[n]);
            if bi.x == bn.x {
                if bi.y < bn.y {
                    pairs.push((i, n));
                } else if bi.y > bn.y {
                    pairs.push((n, i));
                }
            } else if bi.y == bn.y {
                if bi.x < bn.x {
                    pairs.push((i, n));
                } else if bi.x > bn.x {
                    pairs.push((n, i));
                }
            }
        }

        let mut best_pair: Option<usize> = None;
        best_benefit = -1;
        let mut best_pos = 0;
        for (pi, &(n1, n2)) in pairs.iter().enumerate() {
            let horizontal = t.branch[n1].y == t.branch[n2].y;
            // The shift range: the band both ends' neighbours span, along the other axis.
            let coord = |b: &Branch| if horizontal { b.y } else { b.x };
            let span = |n: usize| {
                let (mut lo, mut hi) = (coord(&t.branch[n]), coord(&t.branch[n]));
                for j in 0..3 {
                    let c = coord(&t.branch[nbr[n * 3 + j]]);
                    if c > hi {
                        hi = c;
                    } else if c < lo {
                        lo = c;
                    }
                }
                (lo, hi)
            };
            let ((lo1, hi1), (lo2, hi2)) = (span(n1), span(n2));
            let (lo, hi) = (lo1.max(lo2), hi1.min(hi2));
            if lo >= hi {
                continue;
            }
            let mut cost = vec![0i32; (hi - lo + 1) as usize];
            for j in lo..=hi {
                let mut c = 0i32;
                if horizontal {
                    for k in t.branch[n1].x..t.branch[n2].x {
                        add(&mut c, eh(k, j));
                    }
                } else {
                    for k in t.branch[n1].y..t.branch[n2].y {
                        add(&mut c, ev(j, k));
                    }
                }
                for (end, other) in [(n1, n2), (n2, n1)] {
                    for l in 0..nbr_cnt[end] {
                        let n3 = nbr[end * 3 + l];
                        if n3 == other {
                            continue;
                        }
                        let (xr, yr) = if horizontal {
                            (order(t.branch[end].x, t.branch[n3].x), if j < t.branch[n3].y { (j, t.branch[n3].y) } else { (t.branch[n3].y, j) })
                        } else {
                            (if j < t.branch[n3].x { (j, t.branch[n3].x) } else { (t.branch[n3].x, j) }, order(t.branch[end].y, t.branch[n3].y))
                        };
                        c += side_cost(xr, yr);
                    }
                }
                cost[(j - lo) as usize] = c;
            }
            let cur = coord(&t.branch[n1]);
            let (mut best_cost, mut pos) = (BIG_INT, cur);
            for j in lo..=hi {
                if cost[(j - lo) as usize] < best_cost {
                    best_cost = cost[(j - lo) as usize];
                    pos = j;
                }
            }
            if pos != cur {
                let benefit = cost[(cur - lo) as usize] - best_cost;
                if benefit > best_benefit {
                    best_benefit = benefit;
                    best_pair = Some(pi);
                    best_pos = pos;
                }
            }
        }

        if best_benefit > 0 {
            let (n1, n2) = pairs[best_pair.expect("a benefit names its pair")];
            if t.branch[n1].y == t.branch[n2].y {
                t.branch[n1].y = best_pos;
                t.branch[n2].y = best_pos;
            } else {
                t.branch[n1].x = best_pos;
                t.branch[n2].x = best_pos;
            }
            num_shift += 1;
        }
    }
    num_shift
}

/// `edgeShiftNew` — [`edge_shift`], then up to three rounds of re-pairing coincident Steiner
/// nodes and shifting again.
///
/// ⚠️ A round with no coincident pair ends the loop. A round whose first pair is the one tried
/// last time takes the second pair instead, or does nothing when there is none — but still counts
/// as a round.
pub fn edge_shift_new(t: &mut RsmtTree, num_pins: usize, est: &EstimateGrid) -> i32 {
    let mut num_shift = edge_shift(t, num_pins, est);
    let deg = t.deg;
    let (mut cur1, mut cur2): (i64, i64) = (-1, -1);
    let mut iter = 0;
    while iter < 3 {
        iter += 1;
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for i in deg..t.branch.len() {
            let n = t.branch[i].n;
            if n != i && n != t.branch[n].n && t.branch[i].x == t.branch[n].x && t.branch[i].y == t.branch[n].y {
                pairs.push((i, n));
            }
        }
        if pairs.is_empty() {
            iter = 3;
            continue;
        }
        let is_pair = if pairs[0].0 as i64 != cur1 || pairs[0].1 as i64 != cur2 {
            (cur1, cur2) = (pairs[0].0 as i64, pairs[0].1 as i64);
            true
        } else if pairs.len() > 1 {
            (cur1, cur2) = (pairs[1].0 as i64, pairs[1].1 as i64);
            true
        } else {
            false
        };
        if !is_pair {
            continue;
        }
        let (p1, p2) = (cur1 as usize, cur2 as usize);
        let (mut n1h, mut n1v, mut n2h, mut n2v): (Option<usize>, Option<usize>, Option<usize>, Option<usize>) = (None, None, None, None);
        for j in 0..t.branch.len() {
            let n = t.branch[j].n;
            let bj = t.branch[j];
            if n == p1 {
                let b = t.branch[p1];
                if bj.x == b.x && bj.y != b.y {
                    n1v = Some(j);
                } else if bj.y == b.y && bj.x != b.x {
                    n1h = Some(j);
                }
            } else if n == p2 {
                let b = t.branch[p2];
                if bj.x == b.x && bj.y != b.y {
                    n2v = Some(j);
                } else if bj.y == b.y && bj.x != b.x {
                    n2h = Some(j);
                }
            }
        }
        let n = t.branch[p2].n;
        let (bn, b2) = (t.branch[n], t.branch[p2]);
        if bn.x == b2.x && bn.y != b2.y {
            n2v = Some(n);
        } else if bn.y == b2.y && bn.x != b2.x {
            n2h = Some(n);
        }

        let swap = |t: &mut RsmtTree, a: usize, b: usize| {
            if b == t.branch[p2].n {
                t.branch[a].n = p2;
                t.branch[p1].n = b;
                t.branch[p2].n = p1;
            } else {
                t.branch[a].n = p2;
                t.branch[b].n = p1;
            }
        };
        if let (Some(a), Some(b)) = (n1h, n2h) {
            swap(t, a, b);
            num_shift += edge_shift(t, num_pins, est);
        } else if let (Some(a), Some(b)) = (n1v, n2v) {
            swap(t, a, b);
            num_shift += edge_shift(t, num_pins, est);
        }
    }
    num_shift
}

/// `ripupSegL` — take a segment's L-route usage back off the grid, the way it bent.
pub fn ripup_seg_l<G: Usage2d + ?Sized>(est: &mut G, s: &RoutedSegment) {
    let g = s.seg;
    let cost = -(g.edge_cost as f64);
    let (ymin, ymax) = (g.y1.min(g.y2), g.y1.max(g.y2));
    if s.x_first {
        est.update_h(g.x1, g.x2, g.y1, cost);
        est.update_v(g.x2, ymin, ymax, cost);
    } else {
        est.update_v(g.x1, ymin, ymax, cost);
        est.update_h(g.x1, g.x2, g.y2, cost);
    }
}

/// `newrouteL(net, ripuptype, viaGuided)` — route every tree edge as an L, first ripping up its
/// previous route when `ripup` (the reference's `ripuptype > RouteType::NoRoute`). The per-edge
/// decision is [`route_edge`]'s.
///
/// ⚠️ An edge of positive length becomes an `LRoute` (H/V edges too, with `xFirst` true for H and
/// false for V); a zero-length edge becomes `NoRoute` and keeps its other fields.
pub fn newroute_l(tree: &mut StTree, net: &NdrCostNet, grid: &mut BrkGrid<'_>, ripup: bool, via_guided: bool) {
    let (v_lb, h_lb) = (capacity_lower_bound(grid.v_capacity), capacity_lower_bound(grid.h_capacity));
    for i in 0..tree.edges.len() {
        let e = tree.edges[i];
        if e.len <= 0 {
            tree.routes[i].kind = RouteKind::NoRoute;
            continue;
        }
        let mut g = grid.g.for_net(net);
        if ripup {
            let (a, b) = (&tree.nodes[e.n1], &tree.nodes[e.n2]);
            new_ripup(&mut g, (a.x as i32, a.y as i32), (b.x as i32, b.y as i32), &tree.routes[i].shape(), net.edge_cost);
        }
        let r = route_edge(&mut g, &mut tree.nodes, &e, net.edge_cost, grid.via_cost, via_guided, v_lb, h_lb, grid.red_v, grid.red_h);
        let rt = &mut tree.routes[i];
        rt.kind = RouteKind::LRoute;
        rt.x_first = match r {
            EdgeRoute::Vertical | EdgeRoute::L(LShape::YFirst) => false,
            EdgeRoute::Horizontal | EdgeRoute::L(LShape::XFirst) => true,
            EdgeRoute::None => unreachable!("a positive-length edge is routed"),
        };
    }
}
