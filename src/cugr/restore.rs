// SPDX-License-Identifier: Apache-2.0
//! Adopting a route back into CUGR (`CUGR::restoreNetRoute`): after antenna repair rewrites a
//! net's route (a jumper), the router's tree and demand are rebuilt from the route's segments.

use std::collections::{BTreeMap, BTreeSet, HashSet, VecDeque};

use super::geo::{BoxT, Interval, Point};
use super::grid_graph::{Commit, GrTree, GridGraph};
use super::grnet::GrPoint;
use super::Cugr;
use crate::GSegment;

/// `GridGraph::hashCell`: `(layer × x_size + x) × y_size + y`.
fn hash_cell(g: &GridGraph, layer: i32, p: Point) -> u64 {
    (layer as u64 * g.x_size as u64 + p.x as u64) * g.y_size as u64 + p.y as u64
}

/// `buildTreeFromRoute(route)`: the gcells and unit edges the segments cover, then a spanning tree
/// by BFS.
///
/// Upstream rules: cells and adjacency live in ORDERED maps keyed by `hashCell`, so the BFS starts
/// at the smallest key and visits neighbours in key order. A via's layers are clamped to the top
/// layer; any other layer out of range, a wire changing layer, or a wire spanning cells in both
/// axes rejects the route (`None`). A wire may run either way across its layer (a wrong-way span).
/// The tree must reach every cell.
pub fn build_tree_from_route(g: &GridGraph, route: &[GSegment]) -> Option<GrTree> {
    let mut nodes: BTreeMap<u64, GrPoint> = BTreeMap::new();
    let mut adjacency: BTreeMap<u64, BTreeSet<u64>> = BTreeMap::new();
    let add_node = |nodes: &mut BTreeMap<u64, GrPoint>, gp: GrPoint| {
        let key = hash_cell(g, gp.layer, gp.p);
        nodes.entry(key).or_insert(gp);
        key
    };
    let add_edge = |nodes: &mut BTreeMap<u64, GrPoint>, adjacency: &mut BTreeMap<u64, BTreeSet<u64>>, a: GrPoint, b: GrPoint| {
        let (ka, kb) = (add_node(nodes, a), add_node(nodes, b));
        adjacency.entry(ka).or_default().insert(kb);
        adjacency.entry(kb).or_default().insert(ka);
    };
    let num_layers = g.num_layers as i32;
    for s in route {
        let (mut init, mut fin) = (s.init_layer - 1, s.final_layer - 1);
        if s.is_via() {
            init = init.min(num_layers - 1);
            fin = fin.min(num_layers - 1);
        }
        if init < 0 || init >= num_layers || fin < 0 || fin >= num_layers {
            return None;
        }
        let cells = g.range_search_cells(&BoxT::new(s.init_x, s.init_y, s.final_x, s.final_y)).ok()?;
        if s.is_via() {
            let (x, y) = (cells.x.low, cells.y.low);
            let (lo, hi) = (init.min(fin), init.max(fin));
            add_node(&mut nodes, GrPoint { layer: lo, p: Point::new(x, y) });
            for l in lo..hi {
                add_edge(&mut nodes, &mut adjacency, GrPoint { layer: l, p: Point::new(x, y) }, GrPoint { layer: l + 1, p: Point::new(x, y) });
            }
        } else {
            if init != fin {
                return None;
            }
            if cells.x.low != cells.x.high && cells.y.low != cells.y.high {
                return None;
            }
            if cells.x.low != cells.x.high {
                let y = cells.y.low;
                add_node(&mut nodes, GrPoint { layer: init, p: Point::new(cells.x.low, y) });
                for x in cells.x.low..cells.x.high {
                    add_edge(&mut nodes, &mut adjacency, GrPoint { layer: init, p: Point::new(x, y) }, GrPoint { layer: init, p: Point::new(x + 1, y) });
                }
            } else {
                let x = cells.x.low;
                add_node(&mut nodes, GrPoint { layer: init, p: Point::new(x, cells.y.low) });
                for y in cells.y.low..cells.y.high {
                    add_edge(&mut nodes, &mut adjacency, GrPoint { layer: init, p: Point::new(x, y) }, GrPoint { layer: init, p: Point::new(x, y + 1) });
                }
            }
        }
    }
    let (&root_key, &root) = nodes.iter().next()?;
    let mut tree = GrTree::default();
    let mut built: BTreeMap<u64, usize> = BTreeMap::new();
    built.insert(root_key, tree.add(root.layer, root.p));
    let mut queue = VecDeque::from([root_key]);
    while let Some(key) = queue.pop_front() {
        for &nb in adjacency.get(&key).into_iter().flatten() {
            if built.contains_key(&nb) {
                continue;
            }
            let gp = nodes[&nb];
            let id = tree.add(gp.layer, gp.p);
            tree.nodes[built[&key]].children.push(id);
            built.insert(nb, id);
            queue.push_back(nb);
        }
    }
    (built.len() == nodes.len()).then_some(tree)
}

/// Why `restore_net_route` answered as it did (for the trace).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Restore {
    Ok(GrTree),
    Fail(String),
}

impl Cugr {
    /// `restoreNetRoute(net, route)`: adopt `route` as the net's tree.
    ///
    /// Upstream rules: an empty route or one `build_tree_from_route` rejects fails with nothing
    /// changed. Otherwise the net's old demand is released (with ITS adopted flag) and the net
    /// rebuilt as new — no preferred access points, slack 0, its rule's factors (kept at 1 if it
    /// was soft-demoted). Each pin, in order, binds to its FIRST cell the tree occupies, on that
    /// cell's layer alone; a pin with none fails the restore — with the old demand already gone
    /// and no tree. Success: the tree set, marked ADOPTED, its demand committed.
    ///
    /// `k` is the net's index; the jumper pass only restores nets the router already holds, and
    /// the database is unchanged by a jumper, so the rebuilt net is the held one reset.
    pub fn restore_net_route(&mut self, k: usize, route: &[GSegment], commits: &mut Vec<Commit>) -> Result<Restore, super::StageError> {
        if route.is_empty() {
            return Ok(Restore::Fail("empty".into()));
        }
        let Some(tree) = build_tree_from_route(&self.grid, route) else {
            return Ok(Restore::Fail("tree".into()));
        };
        let occupied: HashSet<u64> = tree.nodes.iter().map(|n| hash_cell(&self.grid, n.layer, n.p)).collect();
        if self.nets[k].routing_tree.is_some() {
            self.commit_net(k, true, commits)?;
        }
        let net = &mut self.nets[k];
        net.slack = 0.0;
        net.preferred_aps.clear();
        net.routing_tree = None;
        net.adopted = false;
        for pin in 0..net.num_pins() {
            let hit = net.pin_access_points[pin].iter().find(|c| occupied.contains(&hash_cell(&self.grid, c.layer, c.p))).copied();
            match hit {
                Some(c) => {
                    net.preferred_aps.insert(pin, (c.p, Interval::point(c.layer)));
                }
                None => return Ok(Restore::Fail(format!("pin {pin}"))),
            }
        }
        net.routing_tree = Some(tree.clone());
        net.adopted = true;
        self.commit_net(k, false, commits)?;
        Ok(Restore::Ok(tree))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cugr::design::{DesignFacts, TechLayerFacts};
    use crate::cugr::grnet::GrNet;
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

    /// 6×6 gcells of 1000 (gridlines to 6001); m1 H, m2 V, m3 H.
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

    fn seg(a: (i32, i32, i32), b: (i32, i32, i32)) -> GSegment {
        GSegment::new(a.0, a.1, a.2, b.0, b.1, b.2)
    }

    // Upstream rule (CUGR `buildTreeFromRoute`): cells and adjacency in ORDERED maps keyed by
    // `hashCell` = (layer × xs + x) × ys + y; BFS from the SMALLEST key, neighbours in key order.
    // A via above the top layer is clamped; a wire spanning cells in both axes rejects the route.
    #[test]
    fn tree_from_route_bfs_from_the_smallest_cell() {
        let c = router();
        // m3 wire (3,1)-(1,1), a via m2↔m3 at (1,1), and a via m3→(4) clamped at (3,1).
        let route = [seg((1500, 1500, 3), (3500, 1500, 3)), seg((1500, 1500, 2), (1500, 1500, 3)), seg((3500, 1500, 3), (3500, 1500, 4))];
        let t = build_tree_from_route(&c.grid, &route).unwrap();
        let order: Vec<(i32, i32, i32)> = t.preorder().iter().map(|&i| (t.nodes[i].layer, t.nodes[i].p.x, t.nodes[i].p.y)).collect();
        // smallest key: layer 1 (m2) at (1,1); then m3 (1,1), (2,1), (3,1)
        assert_eq!(order, vec![(1, 1, 1), (2, 1, 1), (2, 2, 1), (2, 3, 1)]);
        assert!(build_tree_from_route(&c.grid, &[seg((500, 500, 3), (2500, 2500, 3))]).is_none(), "diagonal");
        assert!(build_tree_from_route(&c.grid, &[seg((500, 500, 2), (2500, 500, 3))]).is_none(), "a wire changing layer");
    }

    fn two_pin_net(c: &Cugr) -> GrNet {
        let mut n = crate::cugr::pattern_route::tests_support::net(&c.grid, &[&[(0, 1, 1), (1, 1, 1)], &[(0, 3, 1)]]);
        n.layer_range.max_layer = 2;
        n
    }

    // Upstream rule (CUGR `restoreNetRoute`): each pin binds to its FIRST cell the tree occupies,
    // on that layer alone; the tree is ADOPTED. A pin the tree misses fails the restore AFTER the
    // old demand is released: the net is left with no tree.
    #[test]
    fn restore_binds_first_covered_cell_and_fails_late() {
        let mut c = router();
        c.nets.push(two_pin_net(&c));
        let route = [seg((1500, 1500, 1), (1500, 1500, 3)), seg((1500, 1500, 3), (3500, 1500, 3)), seg((3500, 1500, 1), (3500, 1500, 3))];
        let mut commits = Vec::new();
        assert!(matches!(c.restore_net_route(0, &route, &mut commits).unwrap(), Restore::Ok(_)));
        let n = &c.nets[0];
        assert_eq!(n.preferred_aps[&0], (Point::new(1, 1), Interval::point(0)), "the first candidate, (0,1,1), is on the tree");
        assert!(n.adopted);
        assert!(!commits.is_empty());
        // a route missing pin 1's cell: released, then failed — no tree left
        let short = [seg((1500, 1500, 1), (1500, 1500, 3)), seg((1500, 1500, 3), (2500, 1500, 3))];
        assert_eq!(c.restore_net_route(0, &short, &mut commits).unwrap(), Restore::Fail("pin 1".into()));
        assert!(c.nets[0].routing_tree.is_none());
        assert!(c.grid.graph_edges.iter().flatten().flatten().all(|e| e.demand.abs() < 1e-12), "the old demand was released");
    }

    // Upstream rule (GridGraph `commitTree`, adopted): a WRONG-WAY span in an adopted tree is
    // legal — per cell crossed, the layer's wrong-way demand length over that cell's flank edges
    // (`commitWrongWayWire`); in a native tree it is an error. No CUGR jumper script re-adopts a
    // wrong-way span.
    #[test]
    fn an_adopted_wrong_way_span_spreads_over_the_flanks() {
        let c = router();
        let mut g = c.grid.clone();
        let mut t = GrTree::default();
        let a = t.add(2, Point::new(1, 1));
        let b = t.add(2, Point::new(1, 3));
        t.nodes[a].children.push(b);
        let mut commits = Vec::new();
        assert!(g.commit_tree(&c.design, &t, false, &[], false, &mut commits).is_err(), "native: an error");
        g.commit_tree(&c.design, &t, false, &[], true, &mut commits).unwrap();
        // m3 is horizontal: the span crosses cells y=1,2, each spreading over its x-flanks.
        assert_eq!(commits.len(), 4);
        assert!(commits.iter().all(|m| m.layer == 2 && m.delta > 0.0));
    }
}
