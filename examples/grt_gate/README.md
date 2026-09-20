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

## `mazeconv.json` — symbolic routes walked out into grid points

3,903 records from four designs, checked **point by point** rather than by length. Every one of
the eight (shape, turn, y-ordering) combinations is present and the test asserts each has at
least fifty records — every shape branches on the y-ordering, so a corpus missing one arm would
validate it against nothing.

⚠️ Capped at 800 records per bucket to keep the fixture small, **but never at the cost of an
unusual record**: anything with a buffer/point mismatch, a length out of step with its geometry, a
non-positive entry length or reversed x is kept regardless of the cap. A flat cap would silently
drop exactly the records worth having.

### The sizing invariant, asserted rather than assumed

The reference sizes the point buffer from the length the edge carried **on entry** and then fills
it from the geometry. Those are different quantities. Every shape writes exactly `manhattan + 1`
points, so they agree only while the stored length is already the Manhattan distance — which is
what makes the `resize` safe. The corpus asserts it holds on all 3,903 records.

Two consequences, both confirmed as mutations nothing can kill:

- the explicit zero-length write for a degenerate edge is **redundant** — all 301 such records
  have coincident endpoints, so the recomputed distance is already zero;
- `routelen` taken from the entry length is **indistinguishable** from the recomputed length.

Both are kept, because they are what the reference does and what would hold if an earlier stage
ever left the stored length out of step.

### The two calls that follow

Folding the estimate into the committed demand **adds** and leaves the estimate in place — pinned
because moving it instead looks identical on a first fold and diverges on the second, and the
router folds more than once. The runaway-usage check fires **strictly above** a whole multiple of
the capacity, and each direction is judged against its own capacity; it is constructed, because
it is an error path no shipped design reaches.

## `monotonic.json` — the cheapest pair of bends through a searched midpoint

Validated in two halves, because they cost very different amounts to capture:

- **the walk** — 1,429 routed edges, replayed point by point from the reference's own midpoint and
  orientations;
- **the search** — 130 of those also carry the usage patch over the whole box, the cost table in
  force, and the demand the walk went on to charge, so the choice itself is replayed.

The boxes are too large to carry for every edge — median 476 cells, 3.1 million in total — so the
second set is sampled in the instrument and bucketed here. Buckets are the branches taken: the two
orientation flags, whether the midpoint landed on an endpoint, and the direction of travel on each
axis. Twelve are populated and the test asserts a floor in each.

### The demand the walk charges is checked, not just the points it emits

⛔ **A backward run charges the edge BELOW the cell it stands on.** Nothing about a point list
shows that: swapping the index reproduces every point of every routed edge and still corrupts the
grid that every later stage reads. Both directions survived the first mutation round for exactly
that reason, so the after-walk demand is now captured as a sparse delta and compared directly.

### `via_cost` is zero here too

Third stage running. The two orientation pairs that pay for a via compete unpenalised, and the
corpus asserts the zero rather than relying on it.

### Everything float is carried as raw IEEE bits

⚠️ Including the **inputs** — the cost table and the usage patches — not only the compared cost.
A prefix sum over ~30 table lookups will show a one-unit difference in the last place long before
anything else does.

## `ripup_gates.json` — the two gates that decide whether a route is replaced

583 one-bend decisions and 742 walked-route decisions from five designs. Each carries what the
gate read, its verdict, **and the state afterwards** — a gate that says yes also gives back demand
and, in the one-bend case, undoes node marks.

⛔ **The two gates read different grids and compare differently.** The one-bend gate reads the
**estimate** and asks `usage > capacity`, against a per-edge capacity summed over the net's layer
range. The walked gate reads **committed** demand and asks `usage >= capacity - threshold`.
Feeding either rule to the other passes nothing.

⚠️ **The horizontal mark is given back only to a Steiner node**, while the vertical mark is given
back unconditionally — so a terminal keeps a mark set by a route that has since gone.

### The critical-net arm is live, and entirely unwitnessed in its details

142 of the captured decisions are torn up for a **detour** rather than for congestion, so the arm
itself is exercised. But every one of its four conditions survived a deliberate mutation, and
measuring says why — the corpus has **zero** cases that could separate any of them:

| condition | distinguishing cases in the corpus |
| --- | --- |
| the detour ratio is ≥ 2, not ≥ 1 | 0 |
| the sentinel slack is excluded | 0 — no net carries it |
| a zero previous length disables the check | 0 |
| congestion is decided before the detour | 0 |

All four are pinned by constructed cases. ⚠️ A one-step route can never be a detour however short
its predecessor, which is what made the first attempt at those cases fail.

### The Z undo has no witness either

Neither gate reaches it — the one-bend gate refuses anything that is not a single bend, and the
walked gate only sees routes already expanded into points. It is pinned by the property that
matters: **routing a Z and then ripping it up returns the grid exactly to its previous state.**
The charge and the undo are written in different modules from different reference functions, so
they agree only if both are right.

## `mazecost.json` — the maze router's edge-cost tables

18 distinct builds from three designs, spanning every slope, capacity pair and curve shape the
congestion loop produced, with **every entry** as raw IEEE bits.

⛔ **Carrying every entry rather than a sample was the point.** The curve is a logistic, and `exp`
is the one place two correct implementations may legitimately differ in the last place. Across
all 167,960 entries originally captured, this crate reproduces the reference **bit for bit** — so
these curves need no tolerance anywhere in the pipeline.

### Two tables, not one

⚠️ Each direction is priced against **its own** capacity. The monotonic stage earlier in the
pipeline prices vertical edges from the *horizontal* table; this stage does not, and the obvious
mistake is to carry that over. The corpus makes it visible because the two capacities differ on
every captured design.

⚠️ The span is **forty** times capacity, where the monotonic table spans ten and the runaway-usage
check allows a hundred — three different multiples in three places, none derived from the others.

### A boundary with nothing to catch

The ramp past capacity contributes exactly zero at capacity, so the two pieces meet continuously.
Both `index > capacity` and `index > capacity - 1` reproduce every captured entry — the first
because the term it skips is zero, the second because it is the same condition written
differently. Neither is a corpus gap.

## `netedgeorder.json` — the order a net's own edges are routed in

1,476 nets from four designs, sorted by **routed length, longest first**, **stably**.

⛔ **Stability is the specification, and the corpus proves it can be tested.** 673 of the captured
cases have a tie group *and* required the sort to move something — only those can distinguish a
stable sort from an unstable one. A corpus full of ties where nothing moves would say nothing
about stability, so that count is asserted rather than assumed, and `sort_unstable_by` fails the
gate.

⚠️ The length sorted on is the **routed** length — how many steps the current path takes — not the
distance between the endpoints. A detour therefore raises an edge's priority, and a zero-length
route sorts last rather than being skipped.

## `setupheap.json` — seeding both search frontiers

1,059 setups from four designs, each carrying the net's whole tree — every node with its
neighbours and the edge reaching each, and every edge with its current route — plus the search
region and both frontiers **in push order**.

The edge being re-routed splits its net's tree in two. Everything already routed on one side
becomes a source, everything on the other a destination, which is what makes the search that
follows multi-source and multi-destination rather than point to point.

⛔ **Push order is the behaviour under test.** Every seed is given a distance of zero, so the heap
is entirely ties and pop order is decided by insertion alone. Comparing the frontiers as sets
would pass an implementation that searches in a different order.

⚠️ **309 of the setups have part of the subtree outside the search region**, so the region test is
decided by the corpus rather than assumed — and that count is asserted, because a corpus whose
regions always contain the whole subtree would say nothing about it.

### Two mutations that cannot be killed, both for structural reasons

| mutation | why nothing can kill it |
| --- | --- |
| a two-pin net runs the full traversal | every two-pin net has **exactly two nodes and one edge**, so the traversal has nowhere to go. The shortcut is an optimisation, not a different rule. |
| `visited` marked on enqueue rather than dequeue | every captured net satisfies `edges == nodes - 1` — a proper tree — so each node has one parent and nothing is enqueued twice. |

Both are transcribed as the reference writes them. The second is worth keeping as written: the
reference **relies** on that invariant rather than enforcing it.

## `relax.json` — one step of the search

6,752 relaxations from three designs across **69 branch buckets**: both flags, all four
directions, first arrival against improvement against refusal, and with and without carried-over
usage.

Everything the step writes is compared — the new distance, **both** parent pairs, the flag saying
which pair holds the parent, and the two hyper flags. A step that computes the right distance and
records the wrong parent routes correctly and then backtraces wrongly.

### Three inputs that had to be captured, not guessed

Each was found by a failing test, and each is a value the step reads that nothing else records:

| value | why it cannot be inferred |
| --- | --- |
| the distance of the cell **behind** the current one | the detour test reads a third distance, distinct from the current and the adjacent |
| the cost parameters in force | they change every congestion iteration |
| the hyper flags **before** the step | the first capture read them after the block that sets them, so the trace showed zero transitions |

⟹ The last is the same mistake this corpus already has a rule about: capture the before-state
**before** the mutation, not merely before the write you happen to be looking at.

### The via guard is redundant, and only the call sequence shows it

Read alone, the step says a turn is free at a source. Read from its caller, that case never
arrives: the caller derives the flag as `pre != cur` and initialises `pre` **to `cur`** exactly
when the distance is zero. Measured: of 1,855 relaxations starting from a source, **none**
requests a via, and removing the guard is a mutation nothing can kill.

### Other findings

- ⛔ **The detour test truncates to an integer** before comparing against a double, so it is
  coarser than it looks. Un-truncating it fails the gate.
- ⛔ **The usage blends two rounds**: this one's plus `L` times the previous one's. The
  carried-over component is non-zero on about a tenth of the calls, so both the blend and its
  weight are decided by the corpus.
- ⚠️ **An equal-cost path is refused, not accepted.** Only one captured case offers an exactly
  equal cost and the distance is unchanged either way — what differs is the parent. Pinned by a
  constructed case.
- ⚠️ `removeMin` is **not** the idiomatic swap-pop-sift: the tail is copied to the root and the
  heap repaired while the stale element is still present, so it takes part in comparisons.
