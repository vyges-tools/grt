// SPDX-License-Identifier: Apache-2.0
//! Reading the design database into the setup's typed facts.
//!
//! The sibling of [`crate::apply`]: that module writes what the rules decided, this one reads what
//! the rules are given. Nothing here decides anything — every function is a database walk that
//! fills one of the structs a setup stage already takes, in the order the reference reads it, so a
//! wrong fact can be told from a wrong rule by comparing the facts alone.
//!
//! ⚠️ The database is addressed by NAME, and the reference by pointer; where the two could differ
//! (a routing layer found by its level, a via by its bottom layer) the lookup below says which the
//! reference makes.

use vyges_opendb::Db;

use crate::capacity::{Direction, RoutingLayer};
use crate::tracks::{PitchLayer, SpacingLookup, TechVia, TrackGrid, TrackLayer, TrackPattern, V54Rule};
use crate::Rect;

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// What the technology stages (I4 routing layers, I6 tracks and pitches, I7 grid) read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TechFacts {
    /// `dbTech::findRoutingLayer(l)` for `l` in `1..=getRoutingLayerCount()`, in level order.
    pub routing_layers: Vec<RoutingLayer>,
    /// The same layers as I6 reads them; `index` is I4's index (see [`crate::init_routing_layers`]).
    pub pitch_layers: Vec<PitchLayer>,
    /// `dbTech::getVias()`, in the technology's order.
    pub vias: Vec<TechVia>,
    /// Each routing layer's track grid (`dbBlock::findTrackGrid`), `None` where it has none.
    pub tracks: Vec<TrackLayer>,
    pub routing_layer_count: i32,
    pub dbu_per_micron: i32,
    /// `dbBlock::getDieArea()` — the grid is laid over the DIE, not the core.
    pub die: Rect,
    /// `dbBlock::get{Min,Max}RoutingLayer` and `get{Min,Max}LayerForClock`, as set on the block.
    pub block_min_routing_layer: i32,
    pub block_max_routing_layer: i32,
    pub min_layer_for_clock: i32,
    pub max_layer_for_clock: i32,
}

fn direction(d: &str) -> Option<Direction> {
    match d {
        "HORIZONTAL" => Some(Direction::Horizontal),
        "VERTICAL" => Some(Direction::Vertical),
        _ => None,
    }
}

/// The routing layers in level order: `(name, routing level)`.
///
/// ⚠️ `findRoutingLayer(l)` is by LEVEL; the technology's own layer order interleaves cut and
/// masterslice layers, so the routing layers are picked out and sorted by level.
fn routing_layers_by_level(db: &Db) -> Res<Vec<(String, String)>> {
    let mut layers: Vec<(i32, String, String)> = db
        .layers_with_direction()?
        .into_iter()
        .map(|(name, dir)| (db.layer_get_routing_level(&name), name, dir))
        .filter(|(level, _, _)| *level > 0)
        .collect();
    layers.sort_by_key(|(level, _, _)| *level);
    Ok(layers.into_iter().map(|(_, n, d)| (n, d)).collect())
}

/// A layer's track grid, or `None` where the block has none for it.
fn track_grid(db: &Db, layer: &str) -> Option<TrackGrid> {
    let (x, y) = db.track_patterns(layer).ok()?;
    if x.is_empty() && y.is_empty() {
        return None;
    }
    let pat = |v: Vec<(i32, i32, i32)>| v.into_iter().map(|(origin, count, step)| TrackPattern { origin, count, step }).collect();
    Some(TrackGrid { x: pat(x), y: pat(y) })
}

/// `dbTech::getVias()`, each as `getDefaultVias` and `getViaDims` read it.
fn tech_vias(db: &Db) -> Res<Vec<TechVia>> {
    let level_of_number = |n: i64| db.layer_get_routing_level(&db.layer_name_by_number(n));
    db.tech_get_vias()
        .into_iter()
        .map(|name| {
            let bottom = db.techvia_get_bottom_layer(&name);
            let bottom = (!bottom.is_empty()).then(|| db.layer_get_routing_level(&bottom));
            // Boxes on ROUTING layers only (a cut box has level 0), `dbBox::getDX/getDY`.
            let boxes = db
                .tech_via_boxes(&name)?
                .into_iter()
                .filter_map(|(n, x0, y0, x1, y1)| {
                    let level = level_of_number(n);
                    (level > 0).then_some((level, x1 - x0, y1 - y0))
                })
                .collect();
            let or_default = db.techvia_has_string_property(&name, "OR_DEFAULT");
            Ok(TechVia { name, bottom, or_default, boxes })
        })
        .collect()
}

/// Read every fact the technology stages take.
pub fn read_tech(db: &Db) -> Res<TechFacts> {
    let by_level = routing_layers_by_level(db)?;
    let routing_layers: Vec<RoutingLayer> = by_level
        .iter()
        .map(|(name, dir)| RoutingLayer {
            name: name.clone(),
            routing_level: db.layer_get_routing_level(name),
            direction: direction(dir),
            has_track_grid: track_grid(db, name).is_some(),
            is_backside: db.layer_is_backside(name),
        })
        .collect();
    let pitch_layers = by_level
        .iter()
        .enumerate()
        .map(|(i, (name, _))| {
            Ok(PitchLayer {
                index: i as i32 + 1,
                name: name.clone(),
                is_routing: db.layer_get_type(name)? == "ROUTING",
                routing_level: db.layer_get_routing_level(name),
                width: db.layer_get_width(name) as i32,
                has_two_widths: db.layer_has_two_widths_spacing_rules(name),
                has_v55: db.layer_has_v55_spacing_rules(name),
                v54: db.layer_v54_spacing_rules(name)?.into_iter().map(|(spacing, range)| V54Rule { spacing, range }).collect(),
            })
        })
        .collect::<Res<Vec<_>>>()?;
    let tracks = by_level
        .iter()
        .enumerate()
        .map(|(i, (name, dir))| TrackLayer { index: i as i32 + 1, name: name.clone(), direction: direction(dir), grid: track_grid(db, name) })
        .collect();
    Ok(TechFacts {
        routing_layers,
        pitch_layers,
        vias: tech_vias(db)?,
        tracks,
        routing_layer_count: db.tech_get_routing_layer_count(),
        dbu_per_micron: db.dbu_per_micron(),
        die: Rect {
            x_min: db.block_get_die_area_x_min(),
            y_min: db.block_get_die_area_y_min(),
            x_max: db.block_get_die_area_x_max(),
            y_max: db.block_get_die_area_y_max(),
        },
        block_min_routing_layer: db.block_get_min_routing_layer(),
        block_max_routing_layer: db.block_get_max_routing_layer(),
        min_layer_for_clock: db.block_get_min_layer_for_clock(),
        max_layer_for_clock: db.block_get_max_layer_for_clock(),
    })
}

/// `dbBlock::getGCellTileSize()` — odb's own rule (the M2–M4 pitch × 15).
///
/// ⛔ NOT a technology fact to read up front: it reads the block's MAX ROUTING LAYER, which
/// `getMinMaxLayer` computes and writes back first (`setMaxRoutingLayer`). Read it after that
/// write, where `initCoreGrid` does — before it, on a design with no `set_routing_layers`, odb
/// raises ODB-1219 (routing layer #-1).
pub fn read_tile_size(db: &Db) -> i32 {
    db.block_get_g_cell_tile_size()
}

/// odb's spacing-table lookups, as `calcLayerPitches` makes them.
pub struct DbSpacing<'a>(pub &'a Db);

impl SpacingLookup for DbSpacing<'_> {
    fn tw(&self, layer: &PitchLayer, width1: i32, width2: i32, prl: i32) -> i32 {
        self.0.layer_find_tw_spacing(&layer.name, width1, width2, prl).unwrap_or(0)
    }
    fn v55(&self, layer: &PitchLayer, width: i32, prl: i32) -> i32 {
        self.0.layer_find_v55_spacing(&layer.name, width, prl).unwrap_or(0)
    }
}

/// `dbTransform::apply(Rect&)` for an instance: each corner ORIENTED about the master's origin,
/// then moved by the instance's origin (`dbInst::getTransform` — the ORIGIN, not the location),
/// and the rectangle re-normalised.
///
/// The orientations as odb applies them to a point: `R90` rotates `(x, y)` to `(-y, x)`, `R180` to
/// `(-x, -y)`, `R270` to `(y, -x)`; `MY` negates x, `MX` negates y; `MYR90` / `MXR90` mirror FIRST
/// and then rotate by 90.
pub fn transform_rect(orient: &str, origin: (i32, i32), r: Rect) -> Rect {
    let apply = |(x, y): (i32, i32)| -> (i32, i32) {
        let (x, y) = match orient {
            "R90" => (-y, x),
            "R180" => (-x, -y),
            "R270" => (y, -x),
            "MY" => (-x, y),
            "MYR90" => (-y, -x),
            "MX" => (x, -y),
            "MXR90" => (y, x),
            _ => (x, y),
        };
        (x + origin.0, y + origin.1)
    };
    let (a, b) = (apply((r.x_min, r.y_min)), apply((r.x_max, r.y_max)));
    Rect { x_min: a.0.min(b.0), y_min: a.1.min(b.1), x_max: a.0.max(b.0), y_max: a.1.max(b.1) }
}

/// One master's shapes as `findInstancesObstructions` walks them, in master coordinates:
/// `getObstructions()` then, per `getMTerms()` / `getMPins()`, the pin geometry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MasterShapes {
    pub is_block: bool,
    pub is_pad: bool,
    /// `(routing level, rect)` — level 0 for a non-routing layer.
    pub obstructions: Vec<(i32, Rect)>,
    /// Per pin box: `(terminal, is supply, layer is ROUTING, routing level, rect)`.
    pub pins: Vec<(String, bool, bool, i32, Rect)>,
}

/// Read one master's shapes.
pub fn read_master_shapes(db: &Db, master: &str) -> Res<MasterShapes> {
    let level = |n: i64| db.layer_get_routing_level(&db.layer_name_by_number(n));
    let is_routing = |n: i64| db.layer_get_type(&db.layer_name_by_number(n)).map(|t| t == "ROUTING").unwrap_or(false);
    let rect = |x0, y0, x1, y1| Rect { x_min: x0, y_min: y0, x_max: x1, y_max: y1 };
    let obstructions = db.master_obstruction_boxes(master)?.into_iter().map(|(n, x0, y0, x1, y1)| (level(n), rect(x0, y0, x1, y1))).collect();
    let mut pins = Vec::new();
    for (term, sig) in db.master_mterms(master)? {
        let supply = sig == "POWER" || sig == "GROUND";
        for p in 0..db.num_mpins(master, &term) {
            for (n, x0, y0, x1, y1) in db.mpin_boxes(master, &term, p)? {
                pins.push((term.clone(), supply, is_routing(n), level(n), rect(x0, y0, x1, y1)));
            }
        }
    }
    Ok(MasterShapes { is_block: db.master_is_block(master), is_pad: db.master_is_pad(master), obstructions, pins })
}

/// One database net as net discovery reads it (`findNets`, `addNet`, the has-wires flag).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetFacts {
    pub name: String,
    /// `dbNet::getSigType()` — `SIGNAL`, `CLOCK`, `POWER`, `GROUND`, …
    pub sig_type: String,
    pub is_special: bool,
    pub term_count: i32,
    pub has_special_wires: bool,
    pub connected_by_abutment: bool,
    /// `getWire() != nullptr`.
    pub has_wire: bool,
    /// `getITerms()` as `(instance, terminal)`, and `getBTerms()`, in the net's order.
    pub iterms: Vec<(String, String)>,
    pub bterms: Vec<String>,
}

impl NetFacts {
    pub fn is_supply(&self) -> bool {
        self.sig_type == "POWER" || self.sig_type == "GROUND"
    }
}

/// `dbBlock::getNets()`, in the block's order.
pub fn read_nets(db: &Db) -> Vec<NetFacts> {
    db.net_names()
        .into_iter()
        .map(|name| NetFacts {
            sig_type: db.net_sigtype(&name),
            is_special: db.net_is_special(&name),
            term_count: db.net_get_term_count(&name) as i32,
            has_special_wires: db.num_net_get_s_wires(&name) > 0,
            connected_by_abutment: db.net_is_connected_by_abutment(&name),
            has_wire: db.net_has_wire(&name),
            iterms: db.net_iterms(&name).into_iter().filter_map(|it| it.rsplit_once('/').map(|(i, t)| (i.to_string(), t.to_string()))).collect(),
            bterms: db.net_bterms(&name),
            name,
        })
        .collect()
}

/// `dbPlacementStatus::isPlaced()`: PLACED, FIRM, LOCKED, FIXED or COVER.
pub fn is_placed(status: &str) -> bool {
    matches!(status, "PLACED" | "FIRM" | "LOCKED" | "FIXED" | "COVER")
}

/// One block terminal's pins as `makeBtermPins` reads them: whether every pin is placed, and each
/// pin's boxes as `(routing level, is ROUTING, rect)`.
pub fn read_bterm(db: &Db, bterm: &str) -> Res<(bool, Vec<(i32, bool, Rect)>)> {
    let mut placed = true;
    let mut boxes = Vec::new();
    for p in 0..db.num_bterm_get_b_pins(bterm) {
        placed &= is_placed(&db.bpin_get_placement_status(bterm, p));
        for (n, x0, y0, x1, y1) in db.bpin_layer_boxes(bterm, p)? {
            let name = db.layer_name_by_number(n);
            boxes.push((db.layer_get_routing_level(&name), db.layer_get_type(&name).map(|t| t == "ROUTING").unwrap_or(false), Rect { x_min: x0, y_min: y0, x_max: x1, y_max: y1 }));
        }
    }
    Ok((placed, boxes))
}
