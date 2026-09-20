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

✅ **Validated end to end, over five designs.** On the main one — 563 nets, 3,770 segments —
every one of the 3,848 guides matches the output of a published global router, per net and in
order. Four smaller designs add 19 nets and 329 guides, each chosen for a path the first does not
walk; **two rules that were previously undecidable became decidable** because of one 15-net
addition. The golden was
verified byte-identical to the one shipping with that router's own test suite before it was
committed. See `examples/grt_gate/`.

A second golden carries the `is_connected_to_term` flag, which the guide file does not record —
**all 1,414 marked guides of the 3,848 match**.

The gate is control-verified: widening the guide box, removing the two-guide via form, reversing
guide order within a net, swapping which guide a via's two ends mark, and sharing the pin-claim
set between nets each make it fail, with the diagnostic naming which.

⬜ **One rule no corpus can show**, and the reference's call sites say why rather than leaving it
a mystery: a pinless net is local. Locality is asked in three places, and on the two this crate
implements a pinless net is filtered out *before* the question is put — only the incremental path
asks without a pin guard. So the gap closes when that path is implemented, not when a bigger
design is found. The rule is pinned by a synthetic case either way.

Two sibling gaps — the pin-claim tie-break and locality ignoring the layer — were in the same
state and are now **closed**, each verified by the mutation that previously slipped through.
Mutation testing found all three; the passing gate found none of them.

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

## Setting up a run

`init_fast_route` derives the routing grid from the die area and the cell size, and reports by
name every stage of the published setup order it does not yet run — a stage that is simply not
called produces no diff to chase, so the gaps are visible from the outside.

`is_local` classifies a net, and is checked against the reference on all 563 nets of the design.

`position_on_grid` snaps a coordinate to the centre of the cell containing it — the arithmetic
every pin position funnels through. Checked against the reference by the fixed-point property: its
1,536 pin positions and 7,540 segment endpoints are all cell centres, so snapping one must return
it unchanged.

`order_for_ripup` reproduces the order nets are ripped up and rerouted — **all 321 nets and all
139 de-prioritisations match the reference**. ⛔ Its congestion comparison discriminates almost
nothing: 309 of 320 adjacent pairs tie, and a single tie group of 200 nets — 62% of the design —
is ordered entirely by the sorts being *stable*. An unstable sort is a different function here.

`segments_from_tree` turns a net's Steiner tree into the segments the router consumes — the seam
between two engines. **All 3,733 segments across 1,980 reference invocations match**, on *both*
tree sources: the Steiner engine (the default) and the coefficient-weighted flute the router owns.

`compute_gcell_capacity` counts the routing tracks crossing one grid cell — the per-edge capacity
the router works from. **Every one of 1,076,274 reference calls was replayed and matched**, cell
rectangles and capacities both; a stratified 810-row sample ships as the committed gate.

`order_nets` reproduces the order nets are handed to the router — **non-leaf clock nets first,
then everything else, each group sorted by name**. Checked against the reference on a design where
that is *distinguishable* from a plain name sort, because on many designs it is not.

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
- **The rip-up order is two STABLE sorts with a position-dependent step between them**, so they
  cannot be fused: the middle step reads each net's position in the list the first sort produced.
- **A tree's segments are ordered by X alone**, not lexicographically — so a *vertical* segment
  always comes out with its endpoints swapped, and nothing else does.
- **Which builder produces a net's tree is chosen by its routing alpha**, and the default is 0.3:
  greater than zero, so the ordinary path is the Steiner engine. Alpha zero switches the design to
  a coefficient-weighted flute the router owns, with a different accuracy (2, not 3).
- **Capacity's two bounds are not symmetric.** The lower is a ceiling; the upper floors over
  `max_bound - track_init - 1`, so a track sitting exactly on a cell's upper edge belongs to the
  *next* cell. Dropping that `- 1` over-counts on every boundary a track lands on.
- **Adjacent routing layers must prefer different directions** — but the check is half-open, so
  the topmost pair is never examined, and a backside layer beside a frontside one is skipped.
- **Routing layers are indexed by a running counter**, not by their routing level; layers at
  level 0 are skipped and do not consume an index.
- **A clock net is not a clock-typed net.** One that reaches any clock terminal is a *leaf* and
  routes with the ordinary nets; only one that reaches none of them goes to the front. A pad
  terminal counts as a clock terminal even without a register.
- **A guide landing on one of the net's own pins is flagged**, and the *first* guide to reach that
  point claims it — the one piece of state that carries across segments.

Each of these is pinned by a test that states the rule.

## Relationship to OpenROAD

This engine reimplements behaviour published by the [OpenROAD](https://github.com/The-OpenROAD-Project/OpenROAD)
project's global router. It is a reimplementation from published behaviour, not a transliteration,
and carries no OpenROAD source. Where the two disagree, the presumption is that this engine is
wrong: OpenROAD has carried a great many tapeouts and this has not.

## Licence

Apache-2.0. See `LICENSE` and `NOTICE`.
