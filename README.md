# vyges-grt

Global routing over a placed design.

A global router decides, per net, which cells of a coarse routing grid the detailed router may
use. That answer is emitted as **route guides** — one rectangle per segment, on a layer — and the
guides are what the next stage consumes. This crate turns a routed net's segments into those
guide records.

## Status

⬜ **Early.** The guide writer is implemented and tested; the router that produces the segments is
not. The crate is useful today as the guide-geometry half of a routing flow, and as a reference
for exactly how guide rectangles are derived from grid coordinates.

✅ **Validated end to end.** Over a whole design — 563 nets, 3,770 segments — every one of the
3,848 guides matches the output of a published global router, per net and in order. The golden was
verified byte-identical to the one shipping with that router's own test suite before it was
committed. See `examples/grt_gate/`.

The gate is control-verified: widening the guide box, removing the two-guide via form, or
reversing guide order within a net each make it fail, with the diagnostic naming which.

## What it does

```rust
use vyges_grt::*;

let grid = Grid { tile_size: 100, area: Rect::new(0, 0, 1000, 1000) };
let opts = SaveOptions {
    guide_is_congested: false,
    origin_x: 0, origin_y: 0,
    min_routing_layer: 2,
};

let net = NetRoute {
    name: "clk".into(),
    segments: vec![GSegment {
        init_x: 300, init_y: 300, init_layer: 2,
        final_x: 500, final_y: 300, final_layer: 2,
        is_jumper: false,
    }],
    pins: vec![],
    is_local: false,
};

let guides = save_guides(&[net], &grid, &opts).unwrap();
assert_eq!(guides[0].guides[0].box_, Rect::new(250, 250, 550, 350));
```

## The rules it implements, and why they are not obvious

- **A via is defined by position, not by layer.** A segment is a via when it does not move in x or
  y. The natural reading — "its layers differ" — is a different predicate.
- **A via writes one guide, or two.** On a net local to one grid cell, or where the via lands on
  one of the net's own pins, the layer pair is written in *both* directions over the same box.
  Everywhere else it is written once, low layer then high. The fork changes the guide *count*.
- **`via_layer` is not optional.** A wire guide names its own layer twice; only a via guide names
  two different layers, and that is the only way a reader tells them apart.
- **A guide box grows by half a tile in each direction**, after the endpoints are ordered — so a
  segment stored either way round gives the same box.
- **A box within one tile of the grid edge snaps out to it**, under a truncating integer divide.
  The gap is closed rather than left as a sliver the detailed router cannot use.
- **Guide order within a net is segment order**, and guide files are compared as ordered lists.

Each of these is pinned by a test that states the rule.

## Relationship to OpenROAD

This engine reimplements behaviour published by the [OpenROAD](https://github.com/The-OpenROAD-Project/OpenROAD)
project's global router. It is a reimplementation from published behaviour, not a transliteration,
and carries no OpenROAD source. Where the two disagree, the presumption is that this engine is
wrong: OpenROAD has carried a great many tapeouts and this has not.

## Licence

Apache-2.0. See `LICENSE` and `NOTICE`.
