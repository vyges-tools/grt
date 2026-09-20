# `grt_gate` — the end-to-end corpus and its golden

Two files, and they come from the same run:

| file | what it is |
| --- | --- |
| `corpus.json` | the inputs the guide writer consumes: the routing grid, the run-wide options, the routing-level-to-layer-name map, and per net its segments, its pins and whether it is local |
| `grt_gate.ok` | the guide file a published global router writes for that design |

`grt_gate.ok` was verified **byte-identical** to the guide golden that ships with that router's own
test suite before it was committed here, so it is that project's output rather than ours captured
and blessed.

## Why the corpus carries more than the segments

Two of the inputs appear in **no output file**: whether a net fits inside a single grid cell, and
where each of its pins sits. Both select between the one-guide and two-guide via forms, so a
replay without them cannot tell a missing rule from a missing input.

## What it reaches

| | |
| --- | --- |
| nets | 563 |
| segments | 3,770 |
| pins | 1,536 |
| **local nets** | **68** — the two-guide via form, which changes the guide *count* |
| guides in the golden | 3,848 (the file has 5,537 lines; the rest are net names and brackets) |

The design is `gcd` on a 45nm open cell library, both of which ship with the router this is
compared against.

`connected.ok` is a **second golden**, one line per net with one character per guide, carrying the
`is_connected_to_term` flag that the guide file does not record. 1,414 of the 3,848 guides are
marked.

## `cases/` — four more designs, each for a path the main one does not walk

| case | nets | why it is here |
| --- | --- | --- |
| ⭐ `pin_access1` | 15 | **closes two coverage gaps** the 563-net design could not: a pin route point touched twice, and a net with two pins at one point on different layers. Also the access-point path in pin derivation, on every pin |
| `pin_track_not_aligned` | 1 (2 pins) | the **only** case found that reaches the instance-edge path, and it also reaches the pad/macro fallback |
| `macro_obs_not_aligned` | 1 | pad/macro fallback |
| `modeling_instance_obs` | 2 | pad/macro fallback |

Each carries its own `corpus.json`, `guideok` and `connected.ok`, all captured from the same run
and each verified identical to the golden that ships with the reference's own suite.

🔑 **19 nets against the main design's 563, and they are what made two undecidable rules
decidable.** Coverage is not size — `overlapping_edges` evaluates 24,328 pins and adds nothing.

## ⬜ The third gap is CHARACTERISED, not open

A net with **no pins** is local. No design here has such a net — checked in the DEFs as well as
the corpora, and the count is zero in both — so no corpus can distinguish that rule from its
opposite.

Adding designs would not have helped, and the reference's own call sites say why. Locality is
asked in three places:

| call site | guard | reachable with zero pins |
| --- | --- | --- |
| the guide writer's via fork | only nets that have a route, and a net is routed only when it has more than one pin | no |
| building the router's netlist | the same "more than one pin" gate | no |
| ⭐ the incremental route collection | walks every net with **no pin guard** | **yes** |

⟹ On every path this crate implements, a pinless net is filtered out *before* locality is asked.
The branch is not dead — on the incremental path the test is `route.empty() && !isLocal()`, and a
pinless net answering **true** is exactly what keeps it out of the incremental set; answering
false would insert an empty route for a net with nothing to route.

**So the gap closes when the incremental path is implemented, not when a bigger design is found.**

## `net_order.ok` — a second corpus, chosen on exactly that criterion

The order nets are handed to the router is *non-leaf clock nets first, then the rest, each group
sorted by name*. On many designs the clock nets sort first anyway, so that is indistinguishable
from sorting the whole list — `gcd` has no clock nets at all, and `clock_route`'s two are named
`clk` and `clknet_0_clk`, which sort first regardless.

`net_order.ok` is 348 nets from a design where the full list is **not** name-sorted, so the two
rules give different answers and the test can fail. The nets are fed to `order_nets` **reversed**,
to show the answer depends on the rules rather than on input order.

## `spiral.json` — the traversal that feeds layer assignment

7,088 net states from five designs, each carrying the inputs (coordinates, pin layers, edge list)
and every output the stage produces (aliases, statuses, per-node edge registration, per-edge alias
fields, and the order edges are routed in).

**Five designs, because most of them cannot reach the alias branch.** Swept across all 147
traceable cases: only `gcd_flute` (266 aliased nodes) and `overlapping_edges` (43) ever collapse a
coincident Steiner node at any scale. Every design whose trees come from the PD builder gives 0 or
1. A corpus without a FLUTE case validates the alias resolution against nothing.

**Each net appears many times.** The stage runs once per rip-up iteration and the tree differs
each time, so the repeats are real coverage and are kept as separate cases; only states identical
in every field are dropped. A first extractor attached every iteration's visit order to the
*first* record for each net id, which produced a corpus where most cases had nothing to check and
still passed.

### Two rules no design witnesses

Both were read out of the reference and then found to survive a deliberate mutation across all
7,088 states, so each is pinned by a constructed case instead:

| rule | why no design reaches it |
| --- | --- |
| a Steiner node aliases to the **first** recorded node at its coordinate | first and last differ only where one coordinate holds **two or more terminals**, since a matched Steiner node is never itself recorded. Zero such coordinates in the corpus. |
| the traversal is seeded from the **terminals only** | seeding from every node gives the same answer everywhere here, because these designs index Steiner nodes in discovery order. Separating it needs a Steiner node indexed *below* another on its only path to a terminal. |

⚠️ The second is **not** an equivalence — the constructed tree gives `e0 e1 e2 e3` breadth-first
against `e0 e1 e3 e2` in node order. The one mutation that does survive on purpose is testing the
opposite endpoint when expanding, which is provably the same function; the argument is at the site.

## `spiralroute.json` — the per-edge routing the walk drives

16,229 distinct calls from four designs; 3,487 take the bent arm and **1,619 of those are exact
ties**, so the tie-break is nearly half the decisions rather than a rare corner.

The body is the earlier L re-route's, and the diff against it is three things: marks land on the
**alias** as well as the node, `hID`/`lID` are incremented on the **alias** nodes and crossed
against the shape, and the via bias has no `viaGuided` guard.

⛔ **That third difference is unobservable.** The stage has exactly one call site, between the
assignment that zeroes `via_cost_` and the one that raises it in the 3D phase. All 16,229 captured
calls carry `via_cost_ = 0`, which the test asserts rather than assumes — so the bias contributes
nothing and making it conditional is a mutation nothing can kill. It is transcribed anyway and
recorded as unwitnessed.

### How the bent arm is replayed

Reproducing the captured costs would mean rebuilding the whole demand grid at that instant.
Instead the two halves are checked separately and neither re-implements the other:

- the **comparison** is replayed against the costs the reference actually computed, as raw IEEE
  bits;
- the **state transition** is driven through the real function with the blockage rigged so the
  captured arm is the one taken.

⚠️ `hID` and `lID` are **cumulative over a net's edges** — reset once in the stage's first pass,
not per call — so the replay seeds them from captured before-values. Starting them at zero checks
only a delta and disagrees with the reference from the second edge of every net onward.

⚠️ The degenerate-edge guard is **unreachable from the only caller** and is pinned by a
constructed case for that reason.

## `zroute.json` — the two-bend re-route

285 decisions from four designs, **89 horizontal-first and 196 vertical-first**, each carrying the
whole grid patch its cost loops read. Median patch is 33 cells and the largest 1,681, so the cost
computation is replayed **end to end** rather than checked against a captured intermediate.

⛔ **Every via term in this stage is dead.** `via_cost_` is an `int`, and it is 0 whenever the
stage runs — both family base costs and both endpoint penalties contribute nothing. The corpus
asserts the zero rather than relying on it, and swapping the two endpoint-status arms is a
mutation nothing can kill.

⚠️ **The unreduced-usage asymmetry is not dead**, and it is easy to assume it is. The reference
asks for plain rather than reduced usage on exactly one branch. Blockage differs from zero on
24,658 of 49,012 captured cells and 135 of the 285 decisions take that branch, so levelling the
asymmetry out fails the gate.

### An `else if` that can never run

The dispatch ends with a branch guarded by `len > threshold` hanging off an `if` on exactly that
condition. It is unreachable, and measuring says so: **0 hits across all 147 traceable designs**.

### The horizontal tie-break is inert — provably, not just here

The horizontal family breaks ties on a second cost, and that cost only ever receives two **uniform
fills**. Nothing varies it per candidate, so it is constant across the candidates and the
tie-break clause can never prefer one column over another. Deleting the clause is a mutation
nothing can kill, and that is a property of the code rather than a gap in the corpus.

⚠️ The tie-break *between the two families* is a different matter and does decide real cases — an
exact tie always goes to horizontal-first.

### Capturing this needed a fix first

`via_cost_` is an `int`. Printing it with `%g` makes the float arguments after it read from the
wrong registers, so the first capture reported the same denormal for one bound on every design
while the other varied correctly — which is what gave it away. All three scalars were wrong, not
just the one that looked wrong.
