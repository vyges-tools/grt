// SPDX-License-Identifier: Apache-2.0
//! Global routing: the guide writer.
//!
//! A global router decides, per net, which grid cells the detailed router may use. That answer is
//! emitted as **route guides** — one rectangle per segment, on a layer — and guides are what the
//! next stage consumes. This crate turns a routed net's segments into those guide records.
//!
//! Reimplemented from behaviour published by the OpenROAD project's global router, not
//! transliterated from it. Where the two disagree the presumption is that this engine is wrong.
//!
//! # Shape
//!
//! [`save_guides`] is a thin sequencer and nothing else: it reads in the same order the published
//! stage runs, so a divergence can be pointed at one line rather than bisected. The per-segment
//! rules live in [`guides_for_segment`], the geometry in [`global_routing_to_box`].

pub mod estimate;
pub mod lroute;
pub mod spiral;
pub mod maze;
pub mod layerdp;
pub mod layertable;
pub mod netpinorder;
pub mod full3d;
pub mod slacks;
pub mod softndr;
pub mod checks3d;
pub mod fillvia;
pub mod routes;
pub mod overflow2d;
pub mod maze3d;
pub mod maze3d_pass;
pub mod pricing;
pub mod tracks;
pub mod adjust;
pub mod pins;
pub mod netlist;
pub mod driver;
pub mod findrouting;
pub mod finish;
pub mod congestion;
pub use congestion::{boost_uniform_int, congestion_markers, update_db_congestion, utl_shuffle, CongestedCell, CongestionEdge, CongestionMarker, CrossingNet, DbCongestionLayer, Mt19937};
pub use finish::{cell_suggestion, compute_net_wirelength, compute_suggested_adjustment, compute_wirelength, congestion_verdict, report_congestion, report_routed_nets, suggest_adjustment, CongestedGrid, CongestionLayer};
pub use findrouting::{add_guides_for_local_net, add_remaining_guides, connect_pad_pins, connect_top_level_pins, merge_segments, GridPin, NoRouteGuides, RemainingNet};
pub use driver::{compute_max_routing_layer, get_min_max_layer, has_routable_nets, report_resources, ResourceLayer, GRT_7};
pub use netlist::{add_resources_for_pin_access, compute_net_degree, compute_track_consumption, find_fastroute_pins, get_net_layer_range, has_stacked_vias, initial_net_order_is_kept, makes_fastroute_net, net_max_routing_layer, pin_access_edges, report_net_degree, AccessPinFacts, NdrConsumptionTooLarge, NdrLayerRule, NetlistGrid, RouterPinFacts};
pub use pins::{check_pin_placement, InvalidPinPlacement, compute_pin_position_on_grid, determine_edge, find_nets, find_on_grid_positions, find_pin, is_pin_reachable, make_bterm_pin, make_iterm_pin, pin_overlaps_with_single_track, position_near_inst_edge, rect_middle, AccessPoint, MasterClass, NetCandidate, NetPin, PinEdge, PinError, PinGrid, TermBox};
pub use adjust::{adjust_tile_set, apply_obstruction_adjustment, compute_region_adjustments, compute_tile_reduce, compute_tile_reduce_interval, compute_user_global_adjustments, compute_user_layer_adjustments, init_blocked_intervals, save_resources_before_adjustments, EdgeState, IntervalSet, RegionOutsideDie, RouterEdges};
pub use tracks::{calc_layer_pitches, get_average_track_spacing, get_default_vias, get_via_dims, init_routing_tracks, PitchLayer, RoutingTracks, SpacingLookup, TechVia, TrackError, TrackGrid, TrackLayer, TrackPattern, V54Rule};
pub use pricing::{get_maze_route_cost_3d, get_via_cost, get_via_resistance, get_wire_cost, get_wire_resistance, MoveCost, TechLayers, WireNet};
pub use maze3d_pass::{counts_for_grt183, maze_route_3d_pass, route_one_edge_3d, PassGrid, PassNet, PassParams};
pub use maze3d::{recover_edge, RecoverZeroLength, new_update_node_layers, set_tree_nodes_variables, split_edge_3d, tree_surgery_3d, Edge3D as SurgEdge3D, Node3D, SurgeryOutcome, Tree3D, copy_grids_3d, update_route_type1_3d, update_route_type2_3d, ShiftError, SurgeryEdge3D, SurgeryNode3D, backtrace_3d, Backtrace3D, Recovery, maze_search_3d, CellState, Dir3, NotInHeap, Search3D, Search3DInputs, setup_heap_3d, Cell3, Heaps3D, SeedEdge, SeedNode, new_ripup_3d_type3, remove_edge_from_node, MazeRipupWrong, NodeConnections, edge_in_window, end_index, max_reroute_iter, maze_route_msmd_order_3d, prelude, skips_for_slack, EdgeResult, Maze3DCall, Maze3DEdgeWork, Maze3DEvent, OrderedNet, Prelude, FINAL_RES_AWARE_NETS_PERCENTAGE, HIGH_DETOUR_PENALTY, LOW_DETOUR_PENALTY};
pub use overflow2d::{get_overflow_2d, get_overflow_2d_maze, history_threshold, Overflow2DScan, UsedCell};
pub use routes::{get_net_route, get_routes, grid_to_dbu, report_run_metrics, GridOrigin, NetForRoutes, RouteEdge, RunReport};
pub use fillvia::{fill_via, get_via_stack_range, EdgeFill, EndClaim, NoPreviousRouting, ViaCounts, ViaEdge, ViaNet, ViaNode, ViaPin, NO_EDGE};
pub mod mazecost;
pub use layerdp::{assign_edge_layers, selection_column_witness, LayerDpInputs, LayerEnd};
pub use layertable::{build_layer_grid, LayerDir, LayerRange, TableInputs, BARRED};
pub use netpinorder::{netpin_order_inc, NetForOrder, OrderNetPin, MIN_X_INITIAL};
pub use full3d::{convert_edge_to_full_3d, convert_to_full_3d_type2, Edge3D, Point3D, RouteType};
pub use checks3d::{check_route_3d, ensure_pin_coverage, get_overflow_3d, three_d_via, Cell3D, Overflow3D, RouteDefect, RoutedEdge, RoutedNode, ViaStackEdge};
pub use softndr::{apply_soft_ndr, compute_congested_ndr_nets, congested_ndr_nets, congested_ndr_nets_by_fraction, sort_congested_ndr_nets, CongestedNdr, Overflow2D, disable_ndr_for_congested_nets, set_soft_ndr, update_net_3d_usage, update_planar_net_usage, CongestionView, NdrEdge, NdrNet, UsageGrid};
pub use slacks::{res_aware_score, update_slacks, NetSlackInput, NetSlackOutput, SlackParams, SlackUpdate, WorstMetrics, SHORT_NET_THRESHOLD};
pub use maze::{backtrace, charge_route, remove_loops, maze_route_pass, AfterEdge, route_one_edge, EdgeContext, EdgeOutcome, rewire_after_type2, split_edge, copy_grids, maze_search, maze_edge_is_long_enough, maze_edge_region, heapify, update_route_type1, update_route_type2, SurgeryEdge, netedge_order_dec, relax_adjacent, remove_min, setup_heap, update_heap, Heaps, MazeEdge, MazeNode, MazeSearch, OrderNetEdge, RelaxInputs, BIG_INT};
pub mod mazeconv;
pub use mazecost::{cost_table, get_cost, CostParams};
pub mod monotonic;
pub use monotonic::{monotonic_box, monotonic_cost_table, route_monotonic, walk_monotonic_route, MonotonicRoute};
pub mod zroute;
pub use estimate::{check_2d_edges_usage, save_last_route_len, UsageViolation};
pub use mazeconv::{convert_to_mazeroute, MazeRoute, SymbolicRoute};
pub use zroute::{newroute_z, newroute_z_edge, route_z_edge_after_ripup, ZChoice, HCOST};
pub use spiral::{
    propagate_alias_status, register_edges, reset_and_alias, spiral_route, chooses_y_first, traversal_order, EdgeReg, ResetParams, LAYER_RESET, WALK_RESET, record_layer_extremes, reset_for_layer_extremes, layer_extremes, EdgeLayers, LayerExtremes,
    SpiralNode, MAX_CONNECTIONS,
};
pub use lroute::{
    is_known_status, mark_h, mark_v, route_edge, via_bias, EdgeRoute, TreeEdge, TreeNode,
};
pub use estimate::{
    capacity_lower_bound, choose_l_shape, commit_l_shape, congestion_cost, estimate_all,
    estimate_one_seg, needs_l_route, EstimateGrid, LShape, CAPACITY_LOWER_BOUND_FRACTION,
};
pub mod ripup;
pub mod ripup_route;
pub use ripup_route::{new_ripup, new_ripup_check, new_ripup_congested_l, new_ripup_net, CriticalCheck, RipupReason, RoutedShape};
pub use ripup::{
    cost_and_enlarge_step, logistic_coefficient, order_by_congestion, order_for_ripup,
    step_threshold_m, OrderTree, Schedule, DEPRIORITISE_PERCENT, SLACK_SENTINEL,
};
pub mod rsmt;
pub mod brk_rsmt;
pub mod ndr_cost;
pub mod graph2d;
pub use graph2d::{Graph2d, NetUsage};
pub mod route_l;
pub mod maze_phase;
pub use maze_phase::{convert_to_mazeroute_all, init_for_congestion_loop, lv_rounds, route_monotonic_all, route_monotonic_edge, LvRound};
pub use route_l::{newroute_l_all, newroute_z_all, newroute_z_net, route_l_all, route_seg_l_first_time, spiral_route_all};
pub use ndr_cost::{NdrCap, NdrCostNet, NdrLedger, OVERFLOW_COST_MULTIPLIER};
pub use brk_rsmt::{coeff_adj, copy_st_tree, edge_shift, edge_shift_new, flute_congest, flute_normal, gen_brk_rsmt, htree_suite, mapxy, net_congestion, newroute_l, pin_idx_from_position, ripup_seg_l, BrkFlags, BrkGrid, BrkSummary, CapLayer, Caps3D, CopyTreeError, Flutes, NetRecord, NetState, RoutedSegment, RouteKind, RsmtNet, RsmtTree, SortedPins, StTree, TreeKind, TreeRoute};
pub use rsmt::{
    segments_from_tree, Branch, NetSegments, Segment, COEFF_V_DEFAULT, COEFF_V_NO_ADJUSTMENTS,
    ROUTER_FLUTE_ACCURACY,
};
pub mod capacity;
pub use capacity::{
    capacity_edge_order, check_adjacent_layers_direction, compute_gcell_capacity,
    init_routing_layers, Direction, LayerError, RoutingLayer, INFINITE_CAPACITY,
    mirror_grid_to_fast_route, set_capacities, init_edges_capacity_per_layer, CapacityLayer, EdgeCapacities, FastRouteGrid,
};
pub mod init;
pub use init::{
    init_fast_route, init_grid, is_clk_term, is_local, is_non_leaf_clock, is_routable, order_nets,
    AbsentStage, CoreGrid, DiscoveredNet, ITermClockFacts, SetupReport, config_fast_route, report_layer_settings, FastRouteConfig, SetupOptions,
};

#[cfg(feature = "odb")]
pub mod apply;
#[cfg(feature = "odb")]
pub use apply::apply_guides;

/// An inclusive rectangle in database units.
///
/// ⚠️ Normalised on construction — `(x1, x2)` is stored as `(min, max)` — because the database
/// rectangle this becomes normalises too, so corner order must not be able to change a result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rect {
    pub x_min: i32,
    pub y_min: i32,
    pub x_max: i32,
    pub y_max: i32,
}

impl Rect {
    pub fn new(x1: i32, y1: i32, x2: i32, y2: i32) -> Self {
        Rect {
            x_min: x1.min(x2),
            y_min: y1.min(y2),
            x_max: x1.max(x2),
            y_max: y1.max(y2),
        }
    }
    /// Translate by a delta — the grid origin offset every guide carries.
    pub fn move_delta(self, dx: i32, dy: i32) -> Self {
        Rect {
            x_min: self.x_min + dx,
            y_min: self.y_min + dy,
            x_max: self.x_max + dx,
            y_max: self.y_max + dy,
        }
    }
}

/// One routed segment: a straight run on a layer, or a via between two layers.
///
/// ⛔ **Equality and hashing cover the six coordinates and NOT `is_jumper`** — the reference's
/// `operator==` and `GSegmentHash` both leave the flag out, and `getRoutes`' dedup set is built on
/// them. A derived `PartialEq` would compare the flag too.
#[derive(Debug, Clone, Copy)]
pub struct GSegment {
    pub init_x: i32,
    pub init_y: i32,
    pub init_layer: i32,
    pub final_x: i32,
    pub final_y: i32,
    pub final_layer: i32,
    pub is_jumper: bool,
}

impl PartialEq for GSegment {
    fn eq(&self, o: &Self) -> bool {
        self.init_layer == o.init_layer
            && self.final_layer == o.final_layer
            && self.init_x == o.init_x
            && self.init_y == o.init_y
            && self.final_x == o.final_x
            && self.final_y == o.final_y
    }
}

impl Eq for GSegment {}

impl std::hash::Hash for GSegment {
    fn hash<H: std::hash::Hasher>(&self, h: &mut H) {
        (self.init_x, self.init_y, self.init_layer, self.final_x, self.final_y, self.final_layer)
            .hash(h);
    }
}

impl GSegment {
    /// The reference's constructor.
    ///
    /// ⛔ x and y are each sorted on their own; the layers are NOT. A step from (5, 1) to (2, 3)
    /// becomes (2, 1)–(5, 3): neither endpoint of the original. So a planar step walked backwards
    /// equals the forward one, while a via walked downwards differs from the same via upwards.
    pub fn new(x0: i32, y0: i32, l0: i32, x1: i32, y1: i32, l1: i32) -> GSegment {
        GSegment {
            init_x: x0.min(x1),
            init_y: y0.min(y1),
            init_layer: l0,
            final_x: x0.max(x1),
            final_y: y0.max(y1),
            final_layer: l1,
            is_jumper: false,
        }
    }

    /// ⛔ **A via is defined by POSITION, not by layer.** A segment is a via when it does not move
    /// in x or y — `init_x == final_x && init_y == final_y`.
    ///
    /// The obvious reading, "the layers differ", is a different predicate and a wrong one: it
    /// would classify nothing extra here but would silently diverge on any segment that both
    /// moves and changes layer, and it is not what the published rule says.
    pub fn is_via(&self) -> bool {
        self.init_x == self.final_x && self.init_y == self.final_y
    }
    pub fn is_jumper(&self) -> bool {
        self.is_jumper
    }
}

/// The routing grid, as much of it as guide geometry needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Grid {
    /// Edge length of one grid cell, in database units.
    pub tile_size: i32,
    /// The grid's extent. Only the upper corner is read: a guide that lands within one tile of
    /// the boundary is snapped out to it.
    pub area: Rect,
}

/// A pin, reduced to what the covering-pin test reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pin {
    pub connection_layer: i32,
    pub on_grid_x: i32,
    pub on_grid_y: i32,
}

/// One net's routing, and the facts about it the guide rules consult.
#[derive(Debug, Clone)]
pub struct NetRoute {
    pub name: String,
    pub segments: Vec<GSegment>,
    pub pins: Vec<Pin>,
    /// A net entirely inside one grid cell. Local nets take the two-guide via form.
    pub is_local: bool,
}

/// A guide record: one rectangle, on a layer, with the layer a via rises to.
///
/// ⚠️ `via_layer` is **not** optional. A wire guide names its own layer twice; only a via guide
/// names two different layers, and that is the only way a reader tells the two apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Guide {
    pub layer: i32,
    pub via_layer: i32,
    pub box_: Rect,
    pub is_congested: bool,
    pub is_jumper: bool,
    /// Set on the first guide that touches a route point where one of the net's pins sits.
    ///
    /// ⚠️ Downstream antenna checking reads this to bind guides to pins. It is **not** recorded in
    /// the guide file, so a comparison of guide geometry cannot see it either way.
    pub is_connected_to_term: bool,
}

/// What the run as a whole contributes to every guide it writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SaveOptions {
    /// ⚠️ Computed **once per run**, not per net, and stamped on every guide. A design that is
    /// congested anywhere marks all of them.
    pub guide_is_congested: bool,
    /// The grid origin, added to every guide box.
    pub origin_x: i32,
    pub origin_y: i32,
    /// Below this layer a diagonal wire segment is an error.
    pub min_routing_layer: i32,
}

/// What went wrong, named after the condition rather than the call site.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GuideError {
    /// A via between layers that are not adjacent.
    NonAdjacentLayers { net: String, from: i32, to: i32 },
    /// A diagonal wire segment below the minimum routing layer.
    BlockedMetal { net: String, layer: i32 },
}

impl std::fmt::Display for GuideError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GuideError::NonAdjacentLayers { net, from, to } => write!(
                f,
                "connection between non-adjacent layers {from} and {to} in net {net}"
            ),
            GuideError::BlockedMetal { net, layer } => {
                write!(f, "routing with guides in blocked metal (layer {layer}) for net {net}")
            }
        }
    }
}

impl std::error::Error for GuideError {}

/// The guide rectangle for one segment, before the grid origin is applied.
///
/// The segment's endpoints are grid-cell centres; a guide covers the cells it spans, so the box
/// grows by half a tile in each direction.
///
/// ⚠️ **The endpoints are min/max'd first.** A segment may be stored in either direction, and the
/// half-tile is added to the *lower* corner and *upper* corner rather than to init and final.
///
/// ⚠️ **The snap is a truncating integer division, and that is the rule.** A guide whose upper
/// edge is less than one whole tile from the grid's upper edge is extended to it —
/// `(area_max - ur) / tile_size < 1` — so the gap is closed rather than left as a sliver the
/// detailed router cannot use. Written with a float divide it would snap on a different set of
/// boxes.
pub fn global_routing_to_box(seg: &GSegment, grid: &Grid) -> Rect {
    let (init_x, final_x) = (seg.init_x.min(seg.final_x), seg.init_x.max(seg.final_x));
    let (init_y, final_y) = (seg.init_y.min(seg.final_y), seg.init_y.max(seg.final_y));

    let half = grid.tile_size / 2;
    let ll_x = init_x - half;
    let ll_y = init_y - half;
    let mut ur_x = final_x + half;
    let mut ur_y = final_y + half;

    if (grid.area.x_max - ur_x) / grid.tile_size < 1 {
        ur_x = grid.area.x_max;
    }
    if (grid.area.y_max - ur_y) / grid.tile_size < 1 {
        ur_y = grid.area.y_max;
    }

    Rect::new(ll_x, ll_y, ur_x, ur_y)
}

/// Whether a via segment lands exactly on one of the net's pins.
///
/// ⚠️ Compares against the segment's **final** point and its **top** layer, not its init point.
pub fn is_covering_pin(pins: &[Pin], seg: &GSegment) -> bool {
    let seg_top_layer = seg.init_layer.max(seg.final_layer);
    pins.iter().any(|p| {
        p.connection_layer == seg_top_layer
            && p.on_grid_x == seg.final_x
            && p.on_grid_y == seg.final_y
    })
}

/// The guides one segment produces — **zero, one, or two of them**.
///
/// The three-way split is the substance of the stage:
///
/// | segment | guides |
/// | --- | --- |
/// | via on a local net, or covering a pin | **two**, `(l1,l2)` and `(l2,l1)`, over the same box |
/// | any other via | **one**, `layer = min`, `via_layer = max` |
/// | wire (`init_layer == final_layer`) | **one**, `(layer, layer)` |
///
/// ⚠️ A segment that is neither a via nor same-layer produces **nothing**. That is the published
/// behaviour — the `else if` chain has no final `else` — and it is deliberate here rather than an
/// oversight: such a segment is a diagonal layer change, which the router does not emit.
pub fn guides_for_segment(
    net: &NetRoute,
    seg: &GSegment,
    grid: &Grid,
    opts: &SaveOptions,
) -> Result<Vec<Guide>, GuideError> {
    let box_ = global_routing_to_box(seg, grid).move_delta(opts.origin_x, opts.origin_y);

    if seg.is_via() {
        if (seg.final_layer - seg.init_layer).abs() > 1 {
            return Err(GuideError::NonAdjacentLayers {
                net: net.name.clone(),
                from: seg.init_layer,
                to: seg.final_layer,
            });
        }

        if net.is_local || is_covering_pin(&net.pins, seg) {
            // Both directions of the layer pair, over the same box.
            return Ok(vec![
                Guide {
                    layer: seg.init_layer,
                    via_layer: seg.final_layer,
                    box_,
                    is_congested: opts.guide_is_congested,
                    is_jumper: false,
                    is_connected_to_term: false,
                },
                Guide {
                    layer: seg.final_layer,
                    via_layer: seg.init_layer,
                    box_,
                    is_congested: opts.guide_is_congested,
                    is_jumper: false,
                    is_connected_to_term: false,
                },
            ]);
        }

        return Ok(vec![Guide {
            layer: seg.init_layer.min(seg.final_layer),
            via_layer: seg.init_layer.max(seg.final_layer),
            box_,
            is_congested: opts.guide_is_congested,
            is_jumper: false,
            is_connected_to_term: false,
        }]);
    }

    if seg.init_layer == seg.final_layer {
        if seg.init_layer < opts.min_routing_layer
            && seg.init_x != seg.final_x
            && seg.init_y != seg.final_y
        {
            return Err(GuideError::BlockedMetal {
                net: net.name.clone(),
                layer: seg.init_layer,
            });
        }
        return Ok(vec![Guide {
            layer: seg.init_layer,
            via_layer: seg.init_layer,
            box_,
            is_congested: opts.guide_is_congested,
            is_jumper: seg.is_jumper(),
            is_connected_to_term: false,
        }]);
    }

    Ok(Vec::new())
}

/// A point on the routing grid, on a layer — the key pins and segment ends are matched by.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct RoutePt {
    pub x: i32,
    pub y: i32,
    pub layer: i32,
}

/// Mark the guides of one segment that land on a pin of this net.
///
/// ⛔ **First guide to touch a route point wins, and the state persists across segments.** Once a
/// point is claimed it is never claimed again, so this cannot be computed per segment in
/// isolation — it depends on the order segments are walked, which is why the claim set is
/// threaded through the whole net rather than rebuilt.
///
/// ⚠️ **Which guide gets which end matters.** The segment's init point marks the FIRST guide and
/// its final point the SECOND. For the two-guide via form those are different guides; for every
/// other form the same guide is offered both ends, so either end can claim it.
pub fn mark_connected_to_terms(
    claimed: &mut std::collections::BTreeMap<RoutePt, bool>,
    seg: &GSegment,
    guides: &mut [Guide],
) {
    if guides.is_empty() {
        return;
    }
    let init_pt = RoutePt { x: seg.init_x, y: seg.init_y, layer: seg.init_layer };
    let final_pt = RoutePt { x: seg.final_x, y: seg.final_y, layer: seg.final_layer };
    // index 1 only exists for the two-guide via form; otherwise the same guide is offered twice.
    let final_idx = if guides.len() > 1 { 1 } else { 0 };

    for (pt, idx) in [(init_pt, 0usize), (final_pt, final_idx)] {
        if claimed.get(&pt) == Some(&false) {
            claimed.insert(pt, true);
            guides[idx].is_connected_to_term = true;
        }
    }
}

/// The route points this net's pins sit on, none of them claimed yet.
pub fn find_route_pt_pins(pins: &[Pin]) -> std::collections::BTreeMap<RoutePt, bool> {
    pins.iter()
        .map(|p| (RoutePt { x: p.on_grid_x, y: p.on_grid_y, layer: p.connection_layer }, false))
        .collect()
}

/// One net's guides, in segment order.
#[derive(Debug, Clone, PartialEq)]
pub struct NetGuides {
    pub net: String,
    pub guides: Vec<Guide>,
    /// How many of them are jumpers — reported per run, not per guide.
    pub jumper_count: usize,
}

/// The stage: every net's guides, in the order the nets were given.
///
/// ⛔ **A net with no segments contributes nothing and must not clear anything.** The published
/// stage skips it before touching the database, so a net whose route is empty keeps whatever
/// guides it already had.
///
/// ⚠️ **Guide order within a net is segment order**, and it is load-bearing: the guide file is
/// compared as an ordered list. A database whose guide set prepends must be reversed after
/// writing to get back to this order.
pub fn save_guides(
    nets: &[NetRoute],
    grid: &Grid,
    opts: &SaveOptions,
) -> Result<Vec<NetGuides>, GuideError> {
    let mut out = Vec::with_capacity(nets.len());
    for net in nets {
        if net.segments.is_empty() {
            continue;
        }
        let mut guides = Vec::new();
        let mut jumper_count = 0;
        // ⛔ Per NET, and mutated as the segments are walked: see `mark_connected_to_terms`.
        let mut claimed = find_route_pt_pins(&net.pins);
        for seg in &net.segments {
            let mut seg_guides = guides_for_segment(net, seg, grid, opts)?;
            mark_connected_to_terms(&mut claimed, seg, &mut seg_guides);
            for guide in seg_guides {
                if guide.is_jumper {
                    jumper_count += 1;
                }
                guides.push(guide);
            }
        }
        out.push(NetGuides {
            net: net.name.clone(),
            guides,
            jumper_count,
        });
    }
    Ok(out)
}
