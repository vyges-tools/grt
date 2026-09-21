// SPDX-License-Identifier: Apache-2.0
//! R20 — `getRoutes` and `reportRunMetrics`.
//!
//! Golden `getroutes.json`, both cost modes: per net the edges read and the segments emitted in
//! order; per call the router's net order; per report the via count and usage it was handed and,
//! when verbose, the GRT-0111 / GRT-0112 lines the SAME run printed — the reference's own output.

use serde_json::Value;
use vyges_grt::{
    get_net_route, get_routes, grid_to_dbu, report_run_metrics, GSegment, GridOrigin,
    NetForRoutes, Point3D, RouteEdge,
};

fn golden() -> Value {
    read(&format!("{}/examples/grt_gate/getroutes.json", env!("CARGO_MANIFEST_DIR")))
}

fn read(path: &str) -> Value {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).expect("golden parses")
}

fn int(v: &Value) -> i64 {
    v.as_i64().expect("integer")
}

fn origin(v: &Value) -> GridOrigin {
    GridOrigin {
        tile_size: int(&v[0]) as i32,
        x_corner: int(&v[1]) as i32,
        y_corner: int(&v[2]) as i32,
    }
}

fn edges(r: &Value) -> Vec<RouteEdge> {
    r["edges"].as_array().expect("edges").iter().map(|e| RouteEdge {
        len: int(&e["len"]) as i32,
        routelen: int(&e["routelen"]) as i32,
        grids: e["grids"].as_array().expect("grids").iter().map(|p| Point3D {
            x: int(&p[0]) as i16,
            y: int(&p[1]) as i16,
            layer: int(&p[2]) as i16,
        }).collect(),
    }).collect()
}

fn seg(v: &Value) -> [i32; 6] {
    let a: Vec<i32> = v.as_array().expect("seg").iter().map(|x| int(x) as i32).collect();
    [a[0], a[1], a[2], a[3], a[4], a[5]]
}

fn six(s: &GSegment) -> [i32; 6] {
    [s.init_x, s.init_y, s.init_layer, s.final_x, s.final_y, s.final_layer]
}

// ─── Against the reference ──────────────────────────────────────────────────────────────────

/// Replay each net; returns the number of segments compared.
fn replay(g: &Value) -> usize {
    let records = g["records"].as_array().expect("records");
    assert!(records.len() >= 100, "too few nets: {}", records.len());
    let mut compared = 0;
    for r in records {
        let who = format!("{} net {}", r["design"].as_str().expect("design"), r["net_id"]);
        let got: Vec<[i32; 6]> = get_net_route(&edges(r), origin(&r["origin"])).iter().map(six).collect();
        let want: Vec<[i32; 6]> = r["segments"].as_array().expect("segs").iter().map(seg).collect();
        assert_eq!(got, want, "{who}: segments");
        compared += want.len();
    }
    compared
}

/// Every captured net's segments, in emission order.
#[test]
fn get_routes_matches_the_reference() {
    let compared = replay(&golden());
    assert!(compared >= 3_000, "too few segments: {compared}");
}

/// The whole uncapped corpus.
///
/// GRT_GETROUTES_FULL=/path/to/getroutes-all.json cargo test --test routes -- --ignored
#[test]
#[ignore = "needs the uncapped dump; the committed sample is what CI runs"]
fn get_routes_matches_the_reference_exhaustively() {
    let path = std::env::var("GRT_GETROUTES_FULL").expect("set GRT_GETROUTES_FULL");
    eprintln!("exhaustive: {} segments", replay(&read(&path)));
}

/// ⛔ The returned map is ordered by DATABASE ID, not by the router's net order — and the corpus
/// has calls where the two differ, so the rule is witnessed, not just stated.
#[test]
fn the_route_map_is_ordered_by_database_id_not_router_order() {
    let g = golden();
    let calls = g["calls"].as_array().expect("calls");
    let mut differing = 0;
    for c in calls {
        let order: Vec<u32> = c["order"].as_array().expect("order").iter().map(|v| int(v) as u32).collect();
        let nets: Vec<NetForRoutes> =
            order.iter().map(|&db_id| NetForRoutes { db_id, edges: vec![] }).collect();
        let keys: Vec<u32> = get_routes(&nets, origin(&c["origin"])).into_keys().collect();
        let mut want = order.clone();
        want.sort_unstable();
        want.dedup();
        assert_eq!(keys, want);
        differing += usize::from(order != want);
    }
    assert!(differing >= 10, "too few calls where router order differs from id order: {differing}");
    // ⚠️ The empty-netlist early return is a call too, and returns nothing.
    assert!(calls.iter().any(|c| int(&c["nets"]) == 0), "no empty call captured");
}

/// GRT-0111 / GRT-0112 exactly as the reference printed them, from what it was handed.
#[test]
fn the_run_report_matches_the_references_own_log_lines() {
    let g = golden();
    let (mut matched, mut quiet) = (0, 0);
    for m in g["metrics"].as_array().expect("metrics") {
        let verbose = int(&m["verbose"]) == 1;
        let got = report_run_metrics(int(&m["vias"]) as i32, int(&m["final"]) as i32, verbose);
        assert_eq!(got.vias_metric, int(&m["vias"]) as i32);
        if !verbose {
            assert!(got.verbose_lines.is_empty());
            continue;
        }
        match m.get("want_lines") {
            // The test captured its own log with `tee -quiet`; nothing to compare.
            Some(Value::Null) | None => quiet += 1,
            Some(w) => {
                let want: Vec<String> =
                    w.as_array().expect("lines").iter().map(|l| l.as_str().expect("s").to_string()).collect();
                assert_eq!(got.verbose_lines, want, "{}", m["design"]);
                matched += 1;
            }
        }
    }
    assert!(matched >= 50, "too few verbose reports compared: {matched}");
    assert!(quiet <= 10, "too many reports with their output captured away: {quiet}");
}

// ─── Constructed cases ──────────────────────────────────────────────────────────────────────

fn p(x: i16, y: i16, layer: i16) -> Point3D {
    Point3D { x, y, layer }
}

const O: GridOrigin = GridOrigin { tile_size: 10, x_corner: 100, y_corner: 200 };

fn edge(len: i32, grids: Vec<Point3D>) -> RouteEdge {
    RouteEdge { len, routelen: grids.len() as i32 - 1, grids }
}

/// ⚠️ The cell centre is computed in double and TRUNCATED: an odd tile loses its half unit.
/// No captured design has an odd tile.
#[test]
fn a_cell_centre_truncates_on_an_odd_tile() {
    assert_eq!(grid_to_dbu(2, 7, 0), 17); // 7 * 2.5 = 17.5
    assert_eq!(grid_to_dbu(0, 7, 3), 6); // 3.5 + 3 = 6.5
    assert_eq!(grid_to_dbu(3, 10, 100), 135);
}

/// ⛔ Layers are counted from one, and x and y are sorted independently.
#[test]
fn segments_are_in_dbu_from_layer_one_with_x_and_y_sorted() {
    let route = get_net_route(&[edge(1, vec![p(1, 0, 2), p(0, 0, 2)])], O);
    assert_eq!(route.iter().map(six).collect::<Vec<_>>(), vec![[105, 205, 3, 115, 205, 3]]);
}

/// ⛔ A planar step walked back is the SAME segment and is dropped — across edges of one net.
#[test]
fn a_planar_step_walked_back_is_dropped_across_edges() {
    let route = get_net_route(
        &[edge(1, vec![p(0, 0, 1), p(1, 0, 1)]), edge(1, vec![p(1, 0, 1), p(0, 0, 1)])],
        O,
    );
    assert_eq!(route.len(), 1);
}

/// ⛔ A via walked down is a DIFFERENT segment from the same via walked up (layers are not
/// sorted), and is dropped only by the explicit reversed-via test.
#[test]
fn a_via_walked_the_other_way_is_dropped_by_the_reversed_test() {
    let up = GSegment::new(5, 5, 1, 5, 5, 2);
    let down = GSegment::new(5, 5, 2, 5, 5, 1);
    assert_ne!(up, down);
    let route = get_net_route(
        &[edge(0, vec![p(0, 0, 0), p(0, 0, 1)]), edge(0, vec![p(0, 0, 1), p(0, 0, 0)])],
        O,
    );
    assert_eq!(route.len(), 1);
    assert_eq!(six(&route[0]), [105, 205, 1, 105, 205, 2]);
}

/// ⛔ The gate is `len > 0 || routelen > 0`: a zero-length edge with steps (a pin-coverage stack)
/// is emitted; a positive-length edge without steps emits nothing.
#[test]
fn a_zero_length_edge_with_steps_is_emitted() {
    let stack = RouteEdge { len: 0, routelen: 1, grids: vec![p(0, 0, 0), p(0, 0, 1)] };
    assert_eq!(get_net_route(&[stack], O).len(), 1);
    let bare = RouteEdge { len: 3, routelen: 0, grids: vec![p(0, 0, 0)] };
    assert!(get_net_route(&[bare], O).is_empty());
    let neither = RouteEdge { len: 0, routelen: 0, grids: vec![p(0, 0, 0), p(0, 0, 1)] };
    assert!(get_net_route(&[neither], O).is_empty());
}

/// ⚠️ `routelen` bounds the walk, not the array: points past it are never read.
#[test]
fn the_walk_stops_at_routelen_not_the_array_end() {
    let e = RouteEdge { len: 1, routelen: 1, grids: vec![p(0, 0, 1), p(1, 0, 1), p(9, 9, 1)] };
    assert_eq!(get_net_route(&[e], O).len(), 1);
}

/// ⚠️ Two nets sharing a database id APPEND into one entry, each with its own dedup set — so a
/// segment both nets route appears twice.
#[test]
fn two_nets_with_one_database_id_append_with_separate_dedup() {
    let e = || vec![edge(1, vec![p(0, 0, 1), p(1, 0, 1)])];
    let routes = get_routes(
        &[NetForRoutes { db_id: 7, edges: e() }, NetForRoutes { db_id: 7, edges: e() }],
        O,
    );
    assert_eq!(routes[&7].len(), 2);
}

/// ⛔ Equality ignores the jumper flag, as the reference's does.
#[test]
fn segment_equality_ignores_the_jumper_flag() {
    let a = GSegment::new(0, 0, 1, 10, 0, 1);
    let b = GSegment { is_jumper: true, ..a };
    assert_eq!(a, b);
}

/// ⛔ "Final usage 3D" adds three per via.
#[test]
fn final_usage_adds_three_per_via() {
    let r = report_run_metrics(10, 100, true);
    assert_eq!(r.verbose_lines[1], "[INFO GRT-0112] Final usage 3D: 130");
    assert!(report_run_metrics(10, 100, false).verbose_lines.is_empty());
}
