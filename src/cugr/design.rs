// SPDX-License-Identifier: Apache-2.0
//! CUGR's view of the design (`Design`): the routing layers, the nets and their pin shapes, every
//! obstacle, the gcell gridlines, the via-demand lengths and the unit costs.
//!
//! [`Design::new`] is the constructor's call sequence and nothing else — `read()` then
//! `setUnitCosts()` — and [`Design::read`] is `read()`'s: each stage is its own function, named
//! after the reference's, in the reference's order. The database walks that fill [`DesignFacts`]
//! decide nothing; every filter the reference applies while reading is here.

use super::geo::{BoxT, Interval};
use super::layers::{MetalLayer, MetalLayerFacts, H};
use super::Constants;

/// A DBU rectangle as the database gives it: `(x_min, y_min, x_max, y_max)`.
pub type Rect = (i32, i32, i32, i32);

/// A shape's layer as the database gives it: `None` for a shape with no tech layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShapeLayer {
    pub is_routing: bool,
    /// `getRoutingLevel()`: 0 for a non-routing layer.
    pub routing_level: i32,
}

/// One shape: its layer (if any) and its rectangle, already in design coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Shape {
    pub layer: Option<ShapeLayer>,
    pub rect: Rect,
}

/// One tech layer in `dbTech::getLayers()` order.
#[derive(Debug, Clone, PartialEq)]
pub struct TechLayerFacts {
    pub name: String,
    pub is_routing: bool,
    pub routing_level: i32,
    /// `getUpperLayer()`'s name (the cut above a routing layer), empty when there is none.
    pub upper_layer: String,
    /// The layer as `MetalLayer` reads it; `None` when the block has no track grid for it.
    pub metal: Option<MetalLayerFacts>,
}

/// One tech via in `dbTech::getVias()` order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TechViaFacts {
    pub name: String,
    pub bottom: String,
    pub top: String,
    /// `getBoxes()`: each box's layer NAME (boxes with no layer omitted) and rectangle.
    pub boxes: Vec<(String, Rect)>,
    /// Carries the `OR_DEFAULT` string property.
    pub or_default: bool,
    /// `isDefault()` (LEF `DEFAULT`).
    pub is_default: bool,
}

/// One net terminal as `makeNetPins` reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinFacts {
    /// `dbBTerm::getName()` or `dbITerm::getName()` (`inst/term`).
    pub name: String,
    pub is_port: bool,
    /// A block terminal: whether `getFirstPinLocation` succeeds. Always true for an instance's.
    pub has_location: bool,
    /// Every box: a block terminal's `getBPins()`/`getBoxes()`, an instance terminal's
    /// `getMPins()`/`getGeometry()` transformed by the instance.
    pub shapes: Vec<Shape>,
}

/// One block net in `dbBlock::getNets()` order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetFacts {
    pub name: String,
    pub is_special: bool,
    pub is_supply: bool,
    /// `!getSWires().empty()`.
    pub has_swires: bool,
    pub connected_by_abutment: bool,
    /// In GlobalRouter's clock-net set (`findClockNets`).
    pub is_clock: bool,
    /// Block terminals first, then instance terminals, each in the net's order.
    pub pins: Vec<PinFacts>,
    /// `getWireCount`'s wire count (special-net obstacles only).
    pub wire_count: u32,
    /// Special-wire boxes with each via expanded (`getViaBoxes`), in order: `(shape, from_via)`.
    pub swire_boxes: Vec<(Shape, bool)>,
}

/// Everything `Design` reads.
#[derive(Debug, Clone, PartialEq)]
pub struct DesignFacts {
    pub dbu_per_micron: i32,
    pub die: Rect,
    /// `dbBlock::getGCellTileSize()`.
    pub gcell_tile_size: i32,
    pub layers: Vec<TechLayerFacts>,
    pub vias: Vec<TechViaFacts>,
    pub nets: Vec<NetFacts>,
    /// Per instance in `getInsts()` order: its terminals' pin boxes (in `getITerms()` order), then
    /// its master's obstructions — all transformed by the instance.
    pub instance_shapes: Vec<Shape>,
    /// `dbBlock::getObstructions()`' boxes.
    pub design_obstructions: Vec<Shape>,
    /// `getMinLayerForClock` / `getMaxLayerForClock`.
    pub min_layer_for_clock: i32,
    pub max_layer_for_clock: i32,
}

/// A shape on a layer (`BoxOnLayer`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxOnLayer {
    pub layer: i32,
    pub b: BoxT,
}

fn box_of(r: Rect) -> BoxT {
    BoxT::new(r.0, r.1, r.2, r.3)
}

/// `CUGRPin`.
#[derive(Debug, Clone, PartialEq)]
pub struct CugrPin {
    pub index: usize,
    pub name: String,
    pub is_port: bool,
    pub shapes: Vec<BoxOnLayer>,
}

/// `LayerRange`, 0-based.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LayerRange {
    pub min_layer: i32,
    pub max_layer: i32,
}

/// `CUGRNet`.
#[derive(Debug, Clone, PartialEq)]
pub struct CugrNet {
    pub index: usize,
    pub name: String,
    pub pins: Vec<CugrPin>,
    pub layer_range: LayerRange,
    /// The position in [`DesignFacts::nets`] it came from.
    pub facts_index: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Design {
    pub dbu_per_micron: i32,
    pub die: BoxT,
    pub layers: Vec<MetalLayer>,
    pub gridline_spacing: i32,
    pub gridlines: [Vec<i32>; 2],
    pub nets: Vec<CugrNet>,
    pub obstacles: Vec<BoxOnLayer>,
    pub num_special_nets: usize,
    /// The via chosen per layer pair, `None` where none connects it (for reporting).
    pub pair_vias: Vec<Option<String>>,
    pub via_demand_length_lower: Vec<f64>,
    pub via_demand_length_upper: Vec<f64>,
    pub wrong_way_demand_length: Vec<f64>,
    pub unit_length_wire_cost: f64,
    pub unit_via_cost: f64,
    pub unit_length_short_costs: Vec<f64>,
}

/// A design the reference would read out of range; refused rather than guessed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DesignError {
    /// A special-net wire box on a non-routing layer: the reference files it under layer index -1.
    SpecialWireOffRoutingLayer { net: String },
    /// Fewer than two routing layers: the unit costs read layer 1's pitch.
    TooFewLayers,
    /// The die is narrower than one gcell in a dimension: `computeGrid` reads an empty vector.
    DieSmallerThanGcell,
}

impl Design {
    /// The constructor: `read()`, then `setUnitCosts()`.
    pub fn new(f: &DesignFacts, c: &Constants, min_routing_layer: i32, max_routing_layer: i32) -> Result<Design, DesignError> {
        let mut d = Design::read(f, c, min_routing_layer, max_routing_layer)?;
        d.set_unit_costs(c)?;
        Ok(d)
    }

    /// `Design::read()`.
    fn read(f: &DesignFacts, c: &Constants, min_routing_layer: i32, max_routing_layer: i32) -> Result<Design, DesignError> {
        let mut d = Design {
            dbu_per_micron: f.dbu_per_micron,
            die: box_of(f.die),
            layers: Vec::new(),
            gridline_spacing: 0,
            gridlines: [Vec::new(), Vec::new()],
            nets: Vec::new(),
            obstacles: Vec::new(),
            num_special_nets: 0,
            pair_vias: Vec::new(),
            via_demand_length_lower: Vec::new(),
            via_demand_length_upper: Vec::new(),
            wrong_way_demand_length: Vec::new(),
            unit_length_wire_cost: 0.0,
            unit_via_cost: 0.0,
            unit_length_short_costs: Vec::new(),
        };
        d.read_layers(f, max_routing_layer);
        d.read_netlist(f, min_routing_layer, max_routing_layer);
        d.read_instance_obstructions(f, max_routing_layer);
        d.num_special_nets = d.read_special_net_obstructions(f, max_routing_layer)?;
        d.read_design_obstructions(f, max_routing_layer);
        d.compute_grid()?;
        d.compute_via_demand_lengths(f, c);
        Ok(d)
    }

    /// `readLayers`: every ROUTING layer at or below the max routing layer that has a track grid,
    /// in the technology's order.
    ///
    /// ⚠️ The vector is indexed by routing level − 1 everywhere downstream, so a routing layer
    /// with no track grid below the max would shift every layer above it. The reference does the
    /// same; it is transcribed, not repaired.
    fn read_layers(&mut self, f: &DesignFacts, max_routing_layer: i32) {
        for l in &f.layers {
            if l.is_routing && l.routing_level <= max_routing_layer {
                if let Some(m) = &l.metal {
                    self.layers.push(MetalLayer::new(m));
                }
            }
        }
        self.gridline_spacing = f.gcell_tile_size;
    }

    /// `clampPinLayerIdx`: a pin shape above the top layer is filed on the top layer.
    fn clamp_pin_layer_idx(&self, layer_idx: i32) -> i32 {
        layer_idx.min(self.layers.len() as i32 - 1)
    }

    /// `makeNetPins`: block terminals, then instance terminals, numbered together.
    ///
    /// Upstream rule: a block terminal whose `getFirstPinLocation` fails still becomes a pin — with
    /// no shapes. Only ROUTING-layer boxes are kept, each on its clamped layer.
    fn make_net_pins(&self, n: &NetFacts) -> Vec<CugrPin> {
        n.pins
            .iter()
            .enumerate()
            .map(|(index, p)| {
                let shapes = if p.has_location {
                    p.shapes
                        .iter()
                        .filter_map(|s| {
                            let l = s.layer?;
                            l.is_routing.then(|| BoxOnLayer { layer: self.clamp_pin_layer_idx(l.routing_level - 1), b: box_of(s.rect) })
                        })
                        .collect()
                } else {
                    Vec::new()
                };
                CugrPin { index, name: p.name.clone(), is_port: p.is_port, shapes }
            })
            .collect()
    }

    /// `readNetlist`.
    ///
    /// Upstream rule: a special, supply, special-wired or abutment-connected net is not routed; nor
    /// is one with fewer than two terminals. A clock net takes the CLOCK layer range only when
    /// both clock bounds are set (> 0).
    fn read_netlist(&mut self, f: &DesignFacts, min_routing_layer: i32, max_routing_layer: i32) {
        for (k, n) in f.nets.iter().enumerate() {
            if n.is_special || n.is_supply || n.has_swires || n.connected_by_abutment {
                continue;
            }
            let pins = self.make_net_pins(n);
            if pins.len() < 2 {
                continue;
            }
            let mut layer_range = LayerRange { min_layer: min_routing_layer - 1, max_layer: max_routing_layer - 1 };
            if n.is_clock && f.min_layer_for_clock > 0 && f.max_layer_for_clock > 0 {
                layer_range = LayerRange { min_layer: f.min_layer_for_clock - 1, max_layer: f.max_layer_for_clock - 1 };
            }
            let index = self.nets.len();
            self.nets.push(CugrNet { index, name: n.name.clone(), pins, layer_range, facts_index: k });
        }
    }

    /// The obstacle filter `readInstanceObstructions` and `readDesignObstructions` share: a layer,
    /// ROUTING, at or below the max routing layer. The layer index is NOT clamped.
    fn push_routing_obstacle(&mut self, s: &Shape, max_routing_layer: i32) {
        if let Some(l) = s.layer {
            if l.is_routing && l.routing_level <= max_routing_layer {
                self.obstacles.push(BoxOnLayer { layer: l.routing_level - 1, b: box_of(s.rect) });
            }
        }
    }

    /// `readInstanceObstructions`: every instance's pin shapes (ALL of its terminals, signal and
    /// supply alike), then its master's obstructions.
    fn read_instance_obstructions(&mut self, f: &DesignFacts, max_routing_layer: i32) {
        for s in &f.instance_shapes {
            self.push_routing_obstacle(s, max_routing_layer);
        }
    }

    /// `readSpecialNetObstructions`; returns the number of special nets counted.
    ///
    /// Upstream rule: a net that is special OR supply, with a non-zero wire count. A via is
    /// expanded into its boxes and each kept on a routing layer at or below the max (a cut box has
    /// level 0 and is dropped); a wire box is kept at or below the max with NO routing-layer test.
    fn read_special_net_obstructions(&mut self, f: &DesignFacts, max_routing_layer: i32) -> Result<usize, DesignError> {
        let mut num_special_nets = 0;
        for n in &f.nets {
            if !n.is_special && !n.is_supply {
                continue;
            }
            if n.wire_count == 0 {
                continue;
            }
            for (s, from_via) in &n.swire_boxes {
                let level = s.layer.map_or(0, |l| l.routing_level);
                if *from_via {
                    if level == 0 || level > max_routing_layer {
                        continue;
                    }
                } else if level > max_routing_layer {
                    continue;
                } else if level == 0 {
                    return Err(DesignError::SpecialWireOffRoutingLayer { net: n.name.clone() });
                }
                self.obstacles.push(BoxOnLayer { layer: level - 1, b: box_of(s.rect) });
            }
            num_special_nets += 1;
        }
        Ok(num_special_nets)
    }

    /// `readDesignObstructions`.
    fn read_design_obstructions(&mut self, f: &DesignFacts, max_routing_layer: i32) {
        for s in &f.design_obstructions {
            self.push_routing_obstacle(s, max_routing_layer);
        }
    }

    /// `computeGrid`: gridlines every gcell from the die's low edge while one more still fits
    /// strictly inside, then the die's high edge unless it is already the last.
    fn compute_grid(&mut self) -> Result<(), DesignError> {
        let spacing = self.gridline_spacing;
        for dimension in 0..2 {
            let (low, high) = (self.die.get(dimension).low, self.die.get(dimension).high);
            let lines = &mut self.gridlines[dimension];
            let mut i = low;
            while i + spacing < high {
                lines.push(i);
                i += spacing;
            }
            match lines.last() {
                None => return Err(DesignError::DieSmallerThanGcell),
                Some(&last) if last != high => lines.push(high),
                Some(_) => {}
            }
        }
        Ok(())
    }

    /// `setUnitCosts`.
    ///
    /// Upstream rule: everything is normalised by layer 1's pitch (`m2_pitch`); the short cost per
    /// unit length is the area cost times the layer's width. `m2_pitch * m2_pitch` is an int.
    fn set_unit_costs(&mut self, c: &Constants) -> Result<(), DesignError> {
        let m2_pitch = self.layers.get(1).ok_or(DesignError::TooFewLayers)?.pitch;
        self.unit_length_wire_cost = c.weight_wire_length / f64::from(m2_pitch);
        self.unit_via_cost = c.weight_via_number;
        let unit_area_short_cost = c.weight_short_area / f64::from(m2_pitch * m2_pitch);
        self.unit_length_short_costs = self.layers.iter().map(|l| unit_area_short_cost * f64::from(l.width)).collect();
        Ok(())
    }

    /// `chooseViaForPair`: the via connecting exactly this lower/upper pair, ranked by
    /// `(not OR_DEFAULT, cut count, not LEF DEFAULT, enclosure area)`; the first minimum in the
    /// technology's order wins. Cuts are boxes on the lower layer's upper (cut) layer; the
    /// enclosure area sums the boxes on the two metal layers.
    fn choose_via_for_pair<'a>(f: &'a DesignFacts, lower: &str, upper: &str, cut: &str) -> Option<&'a TechViaFacts> {
        let mut best: Option<(&TechViaFacts, (bool, i32, bool, i64))> = None;
        for via in &f.vias {
            if via.bottom != lower || via.top != upper {
                continue;
            }
            let (mut cuts, mut enc_area) = (0, 0i64);
            for (layer, r) in &via.boxes {
                if layer == cut {
                    cuts += 1;
                } else if layer == lower || layer == upper {
                    enc_area += i64::from(r.2 - r.0) * i64::from(r.3 - r.1);
                }
            }
            let key = (!via.or_default, cuts, !via.is_default, enc_area);
            if best.as_ref().map_or(true, |(_, b)| key < *b) {
                best = Some((via, key));
            }
        }
        best.map(|(v, _)| v)
    }

    /// `viaDemandLength(layer, dx, dy)`: the pad's extent along the tracks times the whole tracks
    /// it blocks across them, `ceil((perp + 2·spacing) / pitch)` in double (1 when the pitch is 0).
    /// A layer with no plain spacing uses its parallel-run default.
    pub fn via_demand_length(layer: &MetalLayer, dx: i32, dy: i32) -> f64 {
        let spacing = if layer.spacing > 0 { layer.spacing } else { layer.default_spacing };
        let (along, perp) = if layer.direction == H { (dx, dy) } else { (dy, dx) };
        let tracks_blocked = if layer.pitch > 0 { (f64::from(perp + 2 * spacing) / f64::from(layer.pitch)).ceil() } else { 1.0 };
        f64::from(along) * tracks_blocked
    }

    /// `computeViaDemandLengths`.
    ///
    /// Upstream rule: a wrong-way wire is a pad `width` along the tracks spanning one gcell across
    /// them. Per layer pair, the chosen via's boxes on each metal layer are UNIONED into one
    /// rectangle and that rectangle's demand length used — unless the union is empty or
    /// degenerate, or no via connects the pair, when the fallback `min_length × via_multiplier`
    /// stands. The top layer's entries stay 0.
    fn compute_via_demand_lengths(&mut self, f: &DesignFacts, c: &Constants) {
        let n = self.layers.len();
        self.via_demand_length_lower = vec![0.0; n];
        self.via_demand_length_upper = vec![0.0; n];
        self.wrong_way_demand_length = self
            .layers
            .iter()
            .map(|l| {
                if l.direction == H {
                    Design::via_demand_length(l, l.width, self.gridline_spacing)
                } else {
                    Design::via_demand_length(l, self.gridline_spacing, l.width)
                }
            })
            .collect();
        self.pair_vias = Vec::new();
        for i in 0..n.saturating_sub(1) {
            let (lower, upper) = (&self.layers[i], &self.layers[i + 1]);
            let cut = f.layers.iter().find(|t| t.name == lower.name).map(|t| t.upper_layer.as_str()).unwrap_or("");
            let via = Design::choose_via_for_pair(f, &lower.name, &upper.name, cut);
            let mut num_lower = f64::from(lower.min_length) * c.via_multiplier;
            let mut num_upper = f64::from(upper.min_length) * c.via_multiplier;
            if let Some(v) = via {
                // `Rect::mergeInit` then `merge`: an inverted box grows by min/max.
                let merge = |layer: &str| {
                    let mut m = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
                    for (l, r) in &v.boxes {
                        if l == layer {
                            m = (m.0.min(r.0), m.1.min(r.1), m.2.max(r.2), m.3.max(r.3));
                        }
                    }
                    m
                };
                let usable = |m: (i32, i32, i32, i32)| !(m.0 > m.2 || m.1 > m.3) && m.2 - m.0 > 0 && m.3 - m.1 > 0;
                let (lo, up) = (merge(&lower.name), merge(&upper.name));
                if usable(lo) {
                    num_lower = Design::via_demand_length(lower, lo.2 - lo.0, lo.3 - lo.1);
                }
                if usable(up) {
                    num_upper = Design::via_demand_length(upper, up.2 - up.0, up.3 - up.1);
                }
            }
            self.via_demand_length_lower[i] = num_lower;
            self.via_demand_length_upper[i] = num_upper;
            self.pair_vias.push(via.map(|v| v.name.clone()));
        }
    }

    /// `getAllObstacles(skip_m1)`: the obstacles per layer, layer 0's dropped when asked.
    pub fn all_obstacles(&self, skip_m1: bool) -> Vec<Vec<BoxT>> {
        let mut all = vec![Vec::new(); self.layers.len()];
        for o in &self.obstacles {
            if o.layer > 0 || !skip_m1 {
                all[o.layer as usize].push(o.b);
            }
        }
        all
    }

    pub fn num_layers(&self) -> usize {
        self.layers.len()
    }

    /// An interval of the die in one dimension (for the gridline search).
    pub fn die_interval(&self, dimension: usize) -> Interval {
        self.die.get(dimension)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layer(name: &str, level: i32, horizontal: bool, width: i32, pitch: i32, area: i64) -> TechLayerFacts {
        TechLayerFacts {
            name: name.into(),
            is_routing: true,
            routing_level: level,
            upper_layer: format!("{name}_cut"),
            metal: Some(MetalLayerFacts {
                name: name.into(),
                routing_level: level,
                horizontal,
                width,
                min_width: width,
                spacing: width,
                resistance: 0.0,
                via_resistance: 0.0,
                tracks: (pitch, pitch / 2, 100),
                area,
                v55_widths_and_lengths: None,
                v55_table: None,
                adjustment: 0.0,
            }),
        }
    }

    fn facts() -> DesignFacts {
        DesignFacts {
            dbu_per_micron: 2000,
            die: (0, 0, 20000, 11000),
            gcell_tile_size: 5000,
            layers: vec![layer("m1", 1, true, 140, 280, 0), layer("m2", 2, false, 140, 380, 0), layer("m3", 3, true, 140, 280, 0)],
            vias: Vec::new(),
            nets: Vec::new(),
            instance_shapes: Vec::new(),
            design_obstructions: Vec::new(),
            min_layer_for_clock: 0,
            max_layer_for_clock: 0,
        }
    }

    fn via(name: &str, boxes: &[(&str, Rect)], or_default: bool, is_default: bool) -> TechViaFacts {
        TechViaFacts {
            name: name.into(),
            bottom: "m1".into(),
            top: "m2".into(),
            boxes: boxes.iter().map(|(l, r)| (l.to_string(), *r)).collect(),
            or_default,
            is_default,
        }
    }

    // Upstream rule (Design.cpp `computeGrid`): a gridline only while `i + spacing < high` — STRICT —
    // then the die's high edge. The last gcell absorbs the remainder, up to a whole extra gcell:
    // 20000 / 5000 gives 0, 5000, 10000, 20000 (a 10000-wide last column, not two of 5000); 11000
    // gives 0, 5000, 11000.
    #[test]
    fn last_gcell_absorbs_the_remainder() {
        let d = Design::new(&facts(), &Constants::default(), 2, 3).unwrap();
        assert_eq!(d.gridlines[0], vec![0, 5000, 10000, 20000]);
        assert_eq!(d.gridlines[1], vec![0, 5000, 11000]);
    }

    // Upstream rule (Design.cpp `setUnitCosts`): normalised by LAYER 1's pitch, the short cost
    // per unit length is weight_short_area / pitch² × the layer's width.
    #[test]
    fn unit_costs_read_layer_one_pitch() {
        let d = Design::new(&facts(), &Constants::default(), 2, 3).unwrap();
        assert_eq!(d.unit_length_wire_cost, 0.5 / 380.0);
        assert_eq!(d.unit_length_short_costs[0], 500.0 / f64::from(380 * 380) * 140.0);
    }

    // Upstream rule (Design.cpp `chooseViaForPair`): OR_DEFAULT first, then fewest cuts, then LEF
    // DEFAULT, then smallest enclosure; the FIRST minimum in technology order wins a tie.
    #[test]
    fn via_choice_ranking() {
        let mut f = facts();
        let one_cut = [("m1", (0, 0, 10, 10)), ("m1_cut", (0, 0, 5, 5)), ("m2", (0, 0, 10, 10))];
        let two_cuts = [("m1", (0, 0, 10, 10)), ("m1_cut", (0, 0, 5, 5)), ("m1_cut", (6, 6, 9, 9)), ("m2", (0, 0, 10, 10))];
        let small = [("m1", (0, 0, 4, 4)), ("m1_cut", (0, 0, 2, 2)), ("m2", (0, 0, 4, 4))];
        f.vias = vec![via("a", &one_cut, false, true), via("b", &two_cuts, true, false)];
        assert_eq!(Design::choose_via_for_pair(&f, "m1", "m2", "m1_cut").unwrap().name, "b", "OR_DEFAULT beats cut count");
        f.vias = vec![via("a", &two_cuts, false, true), via("b", &one_cut, false, false)];
        assert_eq!(Design::choose_via_for_pair(&f, "m1", "m2", "m1_cut").unwrap().name, "b", "fewer cuts beats DEFAULT");
        f.vias = vec![via("a", &one_cut, false, false), via("b", &small, false, true)];
        assert_eq!(Design::choose_via_for_pair(&f, "m1", "m2", "m1_cut").unwrap().name, "b", "DEFAULT beats area");
        f.vias = vec![via("a", &one_cut, false, false), via("b", &one_cut, false, false)];
        assert_eq!(Design::choose_via_for_pair(&f, "m1", "m2", "m1_cut").unwrap().name, "a", "a full tie keeps the first");
    }

    // Upstream rule (Design.cpp `viaDemandLength`): extent along the tracks × ceil((across +
    // 2·spacing) / pitch). A 140×280 pad on a vertical m2 (pitch 380, spacing 140): along = 280,
    // tracks = ceil((140 + 280) / 380) = 2.
    #[test]
    fn via_demand_length_blocks_whole_tracks() {
        let d = Design::new(&facts(), &Constants::default(), 2, 3).unwrap();
        assert_eq!(Design::via_demand_length(&d.layers[1], 140, 280), 560.0);
        // horizontal m1 (pitch 280): along = 140, ceil((280 + 280) / 280) = 2
        assert_eq!(Design::via_demand_length(&d.layers[0], 140, 280), 280.0);
    }

    // Upstream rule (Design.cpp `viaDemandLength`): a layer whose plain spacing is 0 (a tech with
    // only a parallel-run table) uses the table's default — its width row at length 0. Vertical,
    // pitch 380, default spacing 200: a 140×280 pad blocks ceil((140 + 400) / 380) = 2 tracks,
    // where spacing 0 would give ceil(140 / 380) = 1. No layer in the suite has spacing 0.
    #[test]
    fn via_demand_spacing_falls_back_to_the_parallel_run_default() {
        let mut f = facts();
        let m = f.layers[1].metal.as_mut().unwrap();
        m.spacing = 0;
        m.v55_widths_and_lengths = Some((vec![0], vec![0]));
        m.v55_table = Some(vec![vec![200]]);
        let d = Design::new(&f, &Constants::default(), 2, 3).unwrap();
        assert_eq!(d.layers[1].default_spacing, 200);
        assert_eq!(Design::via_demand_length(&d.layers[1], 140, 280), 560.0);
    }

    // Upstream rule (Design.cpp `computeViaDemandLengths`): the via's boxes on one metal layer are
    // UNIONED; with no via for the pair the fallback is min_length × via_multiplier.
    #[test]
    fn via_demand_unions_boxes_and_falls_back() {
        let mut f = facts();
        f.layers[0].metal.as_mut().unwrap().area = 140 * 400; // min_length 400 - 140 = 260
        let d = Design::new(&f, &Constants::default(), 2, 3).unwrap();
        assert_eq!(d.pair_vias[0], None);
        assert_eq!(d.via_demand_length_lower[0], 260.0 * 2.0);
        f.vias = vec![via("v", &[("m1", (0, 0, 100, 140)), ("m1", (50, 0, 300, 140)), ("m2", (0, 0, 140, 140))], false, false)];
        let d = Design::new(&f, &Constants::default(), 2, 3).unwrap();
        // union 300×140 on horizontal m1: along 300, ceil((140 + 280) / 280) = 2
        assert_eq!(d.via_demand_length_lower[0], 600.0);
        assert_eq!(d.via_demand_length_lower[2], 0.0, "the top layer has no pair");
    }

    fn pin(name: &str, is_port: bool, has_location: bool, shapes: &[(Option<ShapeLayer>, Rect)]) -> PinFacts {
        PinFacts { name: name.into(), is_port, has_location, shapes: shapes.iter().map(|&(layer, rect)| Shape { layer, rect }).collect() }
    }

    fn net(name: &str, pins: Vec<PinFacts>) -> NetFacts {
        NetFacts { name: name.into(), is_special: false, is_supply: false, has_swires: false, connected_by_abutment: false, is_clock: false, pins, wire_count: 0, swire_boxes: Vec::new() }
    }

    const M1: Option<ShapeLayer> = Some(ShapeLayer { is_routing: true, routing_level: 1 });
    const M9: Option<ShapeLayer> = Some(ShapeLayer { is_routing: true, routing_level: 9 });
    const CUT: Option<ShapeLayer> = Some(ShapeLayer { is_routing: false, routing_level: 0 });

    // Upstream rule (Design.cpp `readNetlist` / `makeNetPins`): an unplaced block terminal is a pin
    // with NO shapes and still counts toward the two-pin minimum; a pin above the top layer is
    // clamped onto it; non-routing boxes are dropped.
    #[test]
    fn net_pins_clamp_and_keep_unplaced_terminals() {
        let mut f = facts();
        f.nets = vec![
            net("a", vec![pin("p", true, false, &[(M1, (0, 0, 1, 1))]), pin("u1/A", false, true, &[(M9, (0, 0, 1, 1)), (CUT, (0, 0, 1, 1))])]),
            net("single", vec![pin("u2/A", false, true, &[(M1, (0, 0, 1, 1))])]),
        ];
        let d = Design::new(&f, &Constants::default(), 2, 3).unwrap();
        assert_eq!(d.nets.len(), 1, "a one-terminal net is not routed");
        let n = &d.nets[0];
        assert!(n.pins[0].shapes.is_empty(), "no pin location: no shapes");
        assert_eq!(n.pins[1].shapes.len(), 1, "the cut box is dropped");
        assert_eq!(n.pins[1].shapes[0].layer, 2, "level 9 clamps to the top layer, index 2");
        assert_eq!(n.layer_range, LayerRange { min_layer: 1, max_layer: 2 });
    }

    // Upstream rule (Design.cpp `readNetlist`): a clock net takes the clock range only when BOTH
    // clock bounds are set.
    #[test]
    fn clock_range_needs_both_bounds() {
        let mut f = facts();
        let two = || vec![pin("u1/A", false, true, &[(M1, (0, 0, 1, 1))]), pin("u2/A", false, true, &[(M1, (0, 0, 1, 1))])];
        let mut clk = net("clk", two());
        clk.is_clock = true;
        f.nets = vec![clk];
        f.min_layer_for_clock = 3;
        assert_eq!(Design::new(&f, &Constants::default(), 2, 3).unwrap().nets[0].layer_range.min_layer, 1);
        f.max_layer_for_clock = 3;
        assert_eq!(Design::new(&f, &Constants::default(), 2, 3).unwrap().nets[0].layer_range, LayerRange { min_layer: 2, max_layer: 2 });
    }

    // Upstream rule (Design.cpp `readSpecialNetObstructions`): special OR supply, non-zero wire
    // count; via boxes on a cut (level 0) or above the max are dropped, wire boxes only above the
    // max. The net counts even when every box is dropped.
    #[test]
    fn special_net_obstacles() {
        let mut f = facts();
        let mut vdd = net("VDD", Vec::new());
        vdd.is_supply = true;
        vdd.wire_count = 2;
        vdd.swire_boxes = vec![
            (Shape { layer: M1, rect: (0, 0, 10, 10) }, true),
            (Shape { layer: CUT, rect: (0, 0, 5, 5) }, true),
            (Shape { layer: M9, rect: (0, 0, 10, 10) }, false),
            (Shape { layer: M1, rect: (0, 0, 99, 10) }, false),
        ];
        let mut unwired = net("VSS", Vec::new());
        unwired.is_supply = true;
        unwired.swire_boxes = vec![(Shape { layer: M1, rect: (0, 0, 1, 1) }, false)];
        f.nets = vec![vdd, unwired];
        let d = Design::new(&f, &Constants::default(), 2, 3).unwrap();
        assert_eq!(d.num_special_nets, 1, "a zero wire count skips the net");
        assert_eq!(d.obstacles.len(), 2);
        assert_eq!(d.obstacles[1].b, BoxT::new(0, 0, 99, 10));
    }
}
