// SPDX-License-Identifier: Apache-2.0
//! Reading the design database into CUGR's [`DesignFacts`].
//!
//! Nothing here decides anything: each function is a database walk in the order the reference's
//! `Design::read` makes it, and every filter it applies lives in [`super::design`]. A wrong fact can
//! then be told from a wrong rule by comparing the facts alone.

use std::collections::{BTreeMap, BTreeSet};

use vyges_opendb::Db;

use super::design::{DesignFacts, NetFacts, PinFacts, Shape, ShapeLayer, TechLayerFacts, TechViaFacts};
use super::layers::MetalLayerFacts;
use crate::read::transform_rect;
use crate::Rect as DbRect;

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// A layer by its number, as a shape's layer: routing or not, and its routing level.
fn shape_layer(db: &Db, number: i64) -> Option<ShapeLayer> {
    if number < 0 {
        return None;
    }
    let name = db.layer_name_by_number(number);
    Some(ShapeLayer { is_routing: db.layer_get_type(&name).map(|t| t == "ROUTING").unwrap_or(false), routing_level: db.layer_get_routing_level(&name) })
}

fn transformed(orient: &str, origin: (i32, i32), (n, x0, y0, x1, y1): (i64, i32, i32, i32, i32), db: &Db) -> Shape {
    let r = transform_rect(orient, origin, DbRect { x_min: x0, y_min: y0, x_max: x1, y_max: y1 });
    Shape { layer: shape_layer(db, n), rect: (r.x_min, r.y_min, r.x_max, r.y_max) }
}

/// One master's pin boxes in `getMTerms()` / `getMPins()` / `getGeometry()` order, per terminal,
/// and its obstructions — in master coordinates, boxes with no layer omitted.
struct MasterGeometry {
    pins: BTreeMap<String, Vec<(i64, i32, i32, i32, i32)>>,
    /// The terminal names in the master's order.
    order: Vec<String>,
    obstructions: Vec<(i64, i32, i32, i32, i32)>,
}

fn master_geometry(db: &Db, master: &str) -> Res<MasterGeometry> {
    let mut pins = BTreeMap::new();
    let mut order = Vec::new();
    for (term, _) in db.master_mterms(master)? {
        let mut boxes = Vec::new();
        for p in 0..db.num_mpins(master, &term) {
            boxes.extend(db.mpin_boxes(master, &term, p)?);
        }
        order.push(term.clone());
        pins.insert(term, boxes);
    }
    Ok(MasterGeometry { pins, order, obstructions: db.master_obstruction_boxes(master)? })
}

/// The track grid's `getAverageTrackSpacing` for a layer (`findTrackGrid`: the first grid on it).
fn average_track_spacing(db: &Db, layer: &str) -> Option<(i32, i32, i32)> {
    (0..db.num_block_get_track_grids())
        .find(|&i| db.trackgrid_get_tech_layer(i) == layer)
        .map(|i| (db.trackgrid_get_average_track_spacing_track_step(i), db.trackgrid_get_average_track_spacing_track_init(i), db.trackgrid_get_average_track_spacing_num_tracks(i)))
}

/// `dbTech::getLayers()`, each with what `MetalLayer` reads where the block has a track grid.
fn read_layers(db: &Db) -> Res<Vec<TechLayerFacts>> {
    db.layers_with_direction()?
        .into_iter()
        .map(|(name, dir)| {
            let is_routing = db.layer_get_type(&name)? == "ROUTING";
            let upper_layer = db.layer_get_upper_layer(&name);
            let metal = match (is_routing, average_track_spacing(db, &name)) {
                (true, Some(tracks)) => {
                    let v55 = db.layer_v55_spacing_table(&name)?;
                    Some(MetalLayerFacts {
                        name: name.clone(),
                        routing_level: db.layer_get_routing_level(&name),
                        horizontal: dir == "HORIZONTAL",
                        width: db.layer_get_width(&name) as i32,
                        min_width: db.layer_get_min_width(&name) as i32,
                        spacing: db.layer_get_spacing(&name),
                        resistance: db.layer_get_resistance(&name),
                        via_resistance: if upper_layer.is_empty() { 0.0 } else { db.layer_get_resistance(&upper_layer) },
                        tracks,
                        area: db.layer_get_area(&name)?,
                        v55_widths_and_lengths: v55.widths_and_lengths,
                        v55_table: v55.table,
                        adjustment: db.layer_get_layer_adjustment(&name),
                    })
                }
                _ => None,
            };
            Ok(TechLayerFacts { routing_level: db.layer_get_routing_level(&name), name, is_routing, upper_layer, metal })
        })
        .collect()
}

/// `dbTech::getVias()`.
fn read_vias(db: &Db) -> Res<Vec<TechViaFacts>> {
    db.tech_get_vias()
        .into_iter()
        .map(|name| {
            let boxes = db.tech_via_boxes(&name)?.into_iter().map(|(n, x0, y0, x1, y1)| (db.layer_name_by_number(n), (x0, y0, x1, y1))).collect();
            Ok(TechViaFacts {
                bottom: db.techvia_get_bottom_layer(&name),
                top: db.techvia_get_top_layer(&name),
                boxes,
                or_default: db.techvia_has_string_property(&name, "OR_DEFAULT"),
                is_default: db.techvia_is_default(&name),
                name,
            })
        })
        .collect()
}

/// `dbBlock::getNets()` with each net's terminals, special wires and first driver terminal.
fn read_nets(db: &Db, clock_nets: &BTreeSet<String>, masters: &mut BTreeMap<String, MasterGeometry>) -> Res<(Vec<NetFacts>, Vec<String>)> {
    let mut nets = Vec::new();
    let mut drivers = Vec::new();
    for name in db.net_names() {
        let sig = db.net_sigtype(&name);
        let mut pins = Vec::new();
        for bterm in db.net_bterms(&name) {
            let has_location = db.bterm_first_pin_location(&bterm).is_some();
            let mut shapes = Vec::new();
            if has_location {
                for p in 0..db.num_bterm_get_b_pins(&bterm) {
                    for (n, x0, y0, x1, y1) in db.bpin_layer_boxes(&bterm, p)? {
                        shapes.push(Shape { layer: shape_layer(db, n), rect: (x0, y0, x1, y1) });
                    }
                }
            }
            // findODBAccessPoints: every block pin's access points, APPENDED in pin order.
            let per_pin = (0..db.num_bterm_get_b_pins(&bterm)).map(|p| db.bpin_access_points(&bterm, p)).collect::<Result<Vec<_>, _>>()?;
            let access_points = super::design::port_access_points(per_pin);
            pins.push(PinFacts { name: bterm, is_port: true, has_location, shapes, access_points });
        }
        for iterm in db.net_iterms(&name) {
            let (inst, term) = iterm.rsplit_once('/').ok_or("an instance terminal without a slash")?;
            let master = db.inst_get_master(inst);
            if !masters.contains_key(&master) {
                masters.insert(master.clone(), master_geometry(db, &master)?);
            }
            let (orient, origin) = (db.inst_get_orient(inst), (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst)));
            let shapes = masters[&master].pins.get(term).map(|v| v.iter().map(|&b| transformed(&orient, origin, b, db)).collect()).unwrap_or_default();
            // findODBAccessPoints: the preferred access points, offset by the instance's location
            // (orientation R0). ⛔ A non-core instance without them reads EVERY access point,
            // pointer-ordered by master pin — refused when there are any.
            let (ix, iy) = db.inst_location(inst);
            let access_points: Vec<(i32, i32, i32)> = db.iterm_pref_access_points(inst, term)?.into_iter().map(|(x, y, l)| (x + ix, y + iy, l)).collect();
            if access_points.is_empty() && !db.master_is_core(&master) && db.iterm_access_point_count(inst, term)? > 0 {
                return Err(format!("cugr: {iterm}: a non-core terminal's access points (every master pin's, pointer-ordered) are not modelled").into());
            }
            pins.push(PinFacts { name: iterm.clone(), is_port: false, has_location: true, shapes, access_points });
        }
        let swire_boxes = db.net_swire_expanded_boxes(&name)?.into_iter().map(|(n, via, x0, y0, x1, y1)| (Shape { layer: shape_layer(db, n), rect: (x0, y0, x1, y1) }, via)).collect();
        drivers.push(db.net_first_driver_term(&name)?);
        nets.push(NetFacts {
            is_special: db.net_is_special(&name),
            is_supply: sig == "POWER" || sig == "GROUND",
            has_swires: db.num_net_get_s_wires(&name) > 0,
            connected_by_abutment: db.net_is_connected_by_abutment(&name),
            is_clock: clock_nets.contains(&name),
            pins,
            wire_count: db.net_get_wire_count_wire_cnt(&name),
            swire_boxes,
            name,
        });
    }
    Ok((nets, drivers))
}

/// `readInstanceObstructions`' walk: per instance, its terminals' pin boxes in `getITerms()` order
/// (the master's terminal order), then the master's obstructions, all transformed.
fn read_instance_shapes(db: &Db, masters: &mut BTreeMap<String, MasterGeometry>) -> Res<Vec<Shape>> {
    let mut out = Vec::new();
    for inst in db.inst_names() {
        let master = db.inst_get_master(&inst);
        if !masters.contains_key(&master) {
            masters.insert(master.clone(), master_geometry(db, &master)?);
        }
        let g = &masters[&master];
        let (orient, origin) = (db.inst_get_orient(&inst), (db.inst_get_origin_x(&inst), db.inst_get_origin_y(&inst)));
        for term in &g.order {
            out.extend(g.pins[term].iter().map(|&b| transformed(&orient, origin, b, db)));
        }
        out.extend(g.obstructions.iter().map(|&b| transformed(&orient, origin, b, db)));
    }
    Ok(out)
}

/// Every fact `Design` reads, plus each net's first driver terminal (`net_first_driver_term`), by
/// position. Read AFTER the router's setup has written the max routing layer and the global
/// adjustment back to the database, as the reference reads them.
pub fn read_design_facts(db: &Db, clock_nets: &BTreeSet<String>) -> Res<(DesignFacts, Vec<String>)> {
    let mut masters = BTreeMap::new();
    let (nets, drivers) = read_nets(db, clock_nets, &mut masters)?;
    let facts = DesignFacts {
        dbu_per_micron: db.dbu_per_micron(),
        die: (db.block_get_die_area_x_min(), db.block_get_die_area_y_min(), db.block_get_die_area_x_max(), db.block_get_die_area_y_max()),
        gcell_tile_size: db.block_get_g_cell_tile_size(),
        layers: read_layers(db)?,
        vias: read_vias(db)?,
        nets,
        instance_shapes: read_instance_shapes(db, &mut masters)?,
        design_obstructions: db.obstruction_boxes()?.into_iter().map(|(n, x0, y0, x1, y1)| Shape { layer: shape_layer(db, n), rect: (x0, y0, x1, y1) }).collect(),
        min_layer_for_clock: db.block_get_min_layer_for_clock(),
        max_layer_for_clock: db.block_get_max_layer_for_clock(),
    };
    Ok((facts, drivers))
}
