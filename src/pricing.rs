// SPDX-License-Identifier: Apache-2.0
//! Resistance-aware move pricing — `getWireResistance`, `getWireCost`, `getViaResistance`,
//! `getViaCost` and `getMazeRouteCost3D`, one function each, in the reference's order.
//!
//! Pure functions of the technology's resistance tables and the net. Three callers read them: the
//! 3D maze search (a length-one move per step), layer assignment (`assignEdge`, one tile per
//! column; via costs between any two layers), and the net resistance estimate
//! (`getNetResistanceOnLayer`, whole segments).
//!
//! ⛔ The PRECISION is the reference's, value by value: every `float` local is an `f32` here and
//! every `double` an `f64`, and mixed arithmetic widens exactly where C++ widens. The wire and via
//! costs look alike but are NOT: the wire divides an `f32` by an `f64` default (in `f64`); the via
//! narrows its default to `f32` first and divides in `f32`.

/// `static const int BIG_INT = 1e9` — the out-of-range resistance and the cost cap.
pub const BIG_INT: i32 = 1_000_000_000;

/// The technology tables the pricing reads (`preProcessTechLayers`: routing layer `l` and the cut
/// layer above it).
#[derive(Debug, Clone, PartialEq)]
pub struct TechLayers {
    /// `dbTech::getDbUnitsPerMicron`.
    pub dbu_per_micron: i32,
    /// Per routing layer: default width, dbu.
    pub width: Vec<i32>,
    /// Per routing layer: `getResistance()` (ohms per square).
    pub resistance: Vec<f64>,
    /// Per routing layer: the cut layer above's `getResistance()` (ohms per cut); `None` where
    /// there is no layer above.
    pub via_resistance: Vec<Option<f64>>,
}

/// The net's side of a wire price.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WireNet {
    /// The NDR's layer-rule width on the layer being priced, when the net has an NDR.
    pub ndr_width: Option<i32>,
    pub min_layer: i32,
    pub max_layer: i32,
}

/// `dbBlock::dbuToMicrons`: `dbu / (double) dbu_per_micron`.
fn dbu_to_microns(t: &TechLayers, dbu: i32) -> f64 {
    dbu as f64 / t.dbu_per_micron as f64
}

/// `getWireResistance`: ohms of `length` dbu of wire on `layer`, at the NDR width when the net has
/// one. A layer outside the net's range costs `BIG_INT` — checked AFTER the arithmetic, which is
/// discarded.
///
/// `float layer_width = dbuToMicrons(width)` narrows; `res / layer_width` is `double / float` →
/// `double`, narrowed on store; `res_per_micron * dbuToMicrons(length)` is `float * double` →
/// `double`, narrowed on store.
pub fn get_wire_resistance(t: &TechLayers, layer: i32, length: i32, net: WireNet) -> f32 {
    let width = net.ndr_width.unwrap_or(t.width[layer as usize]);
    let resistance = t.resistance[layer as usize];
    let layer_width = dbu_to_microns(t, width) as f32;
    let res_ohm_per_micron = (resistance / layer_width as f64) as f32;
    let final_resistance = (res_ohm_per_micron as f64 * dbu_to_microns(t, length)) as f32;
    if layer < net.min_layer || layer > net.max_layer {
        return BIG_INT as f32;
    }
    final_resistance
}

/// `getWireCost`: the wire's resistance in units of layer 0's sheet resistance, rounded UP, capped
/// at `BIG_INT`. Zero when the net is not being priced for resistance, or layer 0 has none.
///
/// `ceil(float / double)` — the division is in `double`.
pub fn get_wire_cost(t: &TechLayers, resistance_aware: bool, layer: i32, length: i32, net: WireNet) -> i32 {
    if !resistance_aware {
        return 0;
    }
    let default_resistance = t.resistance[0];
    if default_resistance <= 0.0 {
        return 0;
    }
    let final_resistance = get_wire_resistance(t, layer, length, net);
    let cost = (final_resistance as f64 / default_resistance).ceil();
    cost.min(BIG_INT as f64) as i32
}

/// `getViaResistance`: the stacked cuts between the two layers, summed into a `float`
/// (`float += double` narrows at every addition).
pub fn get_via_resistance(t: &TechLayers, from_layer: i32, to_layer: i32) -> f32 {
    let mut total_via_resistance = 0.0f32;
    for i in from_layer.min(to_layer)..from_layer.max(to_layer) {
        let resistance = t.via_resistance[i as usize].expect("a cut layer above every routing layer but the top");
        total_via_resistance = (total_via_resistance as f64 + resistance) as f32;
    }
    total_via_resistance
}

/// `getViaCost`: the stack's resistance in units of the FIRST cut layer's, rounded UP, capped.
///
/// ⛔ Unlike the wire, the default is narrowed to `float` (`const float default_res`) and the
/// division and `ceil` are both in `float`.
pub fn get_via_cost(t: &TechLayers, resistance_aware: bool, from_layer: i32, to_layer: i32) -> i32 {
    if !resistance_aware {
        return 0;
    }
    if from_layer == to_layer {
        return 0;
    }
    let default_res = t.via_resistance[0].expect("a cut layer above layer 0") as f32;
    if default_res <= 0.0 {
        return 0;
    }
    let total_via_resistance = get_via_resistance(t, from_layer, to_layer);
    let cost = (total_via_resistance / default_res).ceil() as f64;
    cost.min(BIG_INT as f64) as i32
}

/// One `getMazeRouteCost3D` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveCost {
    pub from_layer: i32,
    pub to_layer: i32,
    /// `|to_x - from_x|`, `|to_y - from_y|` — all the coordinates are read for.
    pub dx: i32,
    pub dy: i32,
    pub is_via: bool,
}

/// `getMazeRouteCost3D`: a via costs `via_cost_` plus its resistance cost; a wire costs `1.0` plus
/// the wire cost of its length in dbu.
///
/// `float length = |dx| + |dy|`; `length * tile_size_` is `float * int` → `float`, TRUNCATED to
/// the `int` parameter. The result is `float + int` → `float`, which ROUNDS a large cost (an
/// out-of-range layer's ~8e8 lands on a multiple of 64).
pub fn get_maze_route_cost_3d(
    t: &TechLayers,
    resistance_aware: bool,
    via_cost: i32,
    tile_size: i32,
    mv: MoveCost,
    net: WireNet,
) -> f32 {
    if mv.is_via {
        let base_cost = via_cost as f32;
        return base_cost + get_via_cost(t, resistance_aware, mv.from_layer, mv.to_layer) as f32;
    }
    let base_cost = 1.0f32;
    let length = (mv.dx + mv.dy) as f32;
    let wire_resistance_cost =
        get_wire_cost(t, resistance_aware, mv.from_layer, (length * tile_size as f32) as i32, net) as f32;
    base_cost + wire_resistance_cost
}

/// `set_layer_rc -layer L -resistance r` as it reaches the database (`set_dblayer_wire_rc`): the
/// user-unit resistance per length converted to ohm/m through the timer's units, then to ohms per
/// square at the layer's width — the Tcl's double arithmetic, in its order:
/// `res = (r * r_scale) / (1.0 * d_scale)`, `wire_width = width / dbu`, `wire_width * 1e-6 * res`.
///
/// ⛔ The unit scales are `float` (`Unit::scale_`): `d_scale` 1e-6 enters as `9.99999997e-7`.
/// ⛔ No `-resistance` is 0.0, and it is still WRITTEN.
pub fn set_dblayer_wire_r(res_ui: f64, r_scale: f32, d_scale: f32, width_dbu: i32, dbu_per_micron: i32) -> f64 {
    let res = (res_ui * f64::from(r_scale)) / (1.0 * f64::from(d_scale));
    let wire_width = f64::from(width_dbu) / f64::from(dbu_per_micron);
    wire_width * 1e-6 * res
}

/// `set_layer_rc -via V -resistance r` (`set_dbvia_wire_r`): ohms per cut, `r * r_scale`.
pub fn set_dbvia_wire_r(res_ui: f64, r_scale: f32) -> f64 {
    res_ui * f64::from(r_scale)
}

#[cfg(test)]
mod layer_rc_tests {
    use super::*;

    // Against the reference's own tech-layer resistances after `source sky130hs.rc` and
    // `source asap7/setRC.tcl` (grt-slack-trace.py, `VYGS|layerR` / `VYGS|viaR`), kohm libraries.
    #[test]
    fn set_layer_rc_writes_what_the_reference_writes() {
        let (kohm, um) = (1e3f32, 1e-6f32);
        let cases = [
            (7.176e-02, 170, 1000, 0x402865fd8be34bec_u64), // sky130 li1
            (8.929e-04, 140, 1000, 0x3fc0003255946443),     // sky130 met1
            (1.567e-04, 300, 1000, 0x3fa811b1da307fc0),     // sky130 met3
            (7.04175E-02, 18, 1000, 0x3ff447bdcfdef1e3),    // asap7 M1
            (1.18619E-02, 32, 1000, 0x3fd84b0d45938ee4),    // asap7 M6
        ];
        for (r, w, dbu, bits) in cases {
            assert_eq!(set_dblayer_wire_r(r, kohm, um, w, dbu).to_bits(), bits, "{r} at width {w}");
        }
        assert_eq!(set_dbvia_wire_r(4.5E-3, kohm).to_bits(), 0x4012000000000000); // sky130 via
        assert_eq!(set_dbvia_wire_r(9.249146E-3, kohm).to_bits(), 0x40227f901083dbc2); // sky130 mcon
        assert_eq!(set_dbvia_wire_r(1.72E-02, kohm).to_bits(), 0x4031333333333333); // asap7 V1
    }

    // ⛔ The trap: with the scales taken as doubles the wire resistances move in their low bits.
    #[test]
    fn double_unit_scales_would_miss_the_reference() {
        let res: f64 = (7.176e-02 * 1e3) / (1.0 * 1e-6);
        let as_double: f64 = 170.0 / 1000.0 * 1e-6 * res;
        assert_ne!(as_double.to_bits(), set_dblayer_wire_r(7.176e-02, 1e3, 1e-6, 170, 1000).to_bits());
    }
}
