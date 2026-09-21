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

## `backtrace.json` — walking the parents back from the meeting point

1,760 walks from four designs, 22,348 steps, **744 of them jumps**.

The search stops the moment it pops a cell that already belongs to the destination subtree, so the
meeting point is already on that subtree and a single walk back to a source cell is the whole new
route. There is no second traversal.

### The capture had to be placed at the read, not near it

Each step records the horizontal jump flag **at the cell entered**, the vertical jump flag at the
cell as it stood **after** the horizontal test, and the column where that second read happened.

⛔ The first capture read the vertical flag *after* the whole test block, so whenever a vertical
jump fired it sampled the flag at the post-jump cell rather than the one tested. The trace then
described a step that moved with no flag set — impossible — and the replay diverged on 66 walks.
**Sample a value where it is read, not merely before the next write.**

### Two mutations that cannot be killed, and why they differ

| mutation | verdict |
| --- | --- |
| the two jump tests swapped in order | ⭐ **structural equivalence** — after a parent step the cell differs from the previous one in exactly one axis, after a jump in none, so at most one test can fire. 0 of 744 captured jumps move both axes. |
| the jump shortened so the position does not advance | **not a survivor at all** — it spins. The reflection is what makes the walk terminate; there is no bound on the loop. |

⚠️ The first-step guard **is** a real gap — not one of the 1,760 walks has a jump flag set at its
meeting point, so the corpus cannot tell the guard from its absence. Pinned by a constructed case,
which matters because the previous position it guards is uninitialised on that first pass.

## `surgery.json` — splicing the new path back into the tree

1,500 surgeries from four designs, in two shapes: the node landed on one of **its own** edges, or
on a **different** one.

⛔ **In the second shape the three edge slots are recycled, not created.** The node's two edges
become the single edge joining its former neighbours, and the edge it landed on becomes the node's
two new edges — all in the same three slots, roles swapped. The merged edge is written into the
slot of the edge about to be split, so the split only works because every point list is copied out
first. Reordering those two steps fails the gate.

⚠️ **The second shape assigns no endpoints**, unlike the first: the caller rewrites all three
edges' endpoints and the adjacency of five nodes afterwards. The test compares endpoints only
where the function itself sets them.

### What the corpus decides, and what it cannot

All four of the first shape's orientation combinations are present and asserted — each rewritten
edge is stored with the smaller column first, the same convention the segment emission uses, which
is why the endpoints are assigned in both arms rather than once.

⛔ **No captured input edge is un-routed.** The single-point branch of the copy — which hands back
the requested endpoint's own coordinates rather than the stored points — is pinned by a
constructed case, because an empty list there would silently shorten every join below it.

## `region.json` — how far the search may stray

2,410 regions from four designs across **14 branch buckets**: the allowance capped by the caller's
limit or by the edge's own route length, the net critical or not, and the region clamped at either
grid edge or neither. Both intermediates are carried as well as the result, so a mismatch
attributes to the allowance or to the clamping rather than to "the region".

⛔ **The allowance is capped by the edge's CURRENT route length**, not by the distance between its
endpoints — an edge already routed the long way round gets a wider search than a short one between
the same points. Both caps bind somewhere in the corpus, which is asserted.

### A branch that is reached 302 times and never does anything

A critical net narrows its own region — but only from the **seventh** iteration, because the
shrink is `(iter / 7) * 5`. Measured: critical nets appear **only at iterations 3, 4 and 5**, so
the shrink is **zero on all 302 of them**.

⟹ The branch is live and its effect never is. Those are different claims and the corpus supports
only the first, so what it does when it bites is pinned by a constructed case: applied inwards on
every side, capped at half the allowance so the region cannot invert, and clamped to the grid
afterwards.

⚠️ The two divisions step on **different cadences** — the allowance every sixth iteration, the
shrink every seventh. Iterations 6 and 7 share an allowance step, which is what lets a test
attribute a difference between them to the shrink alone.

## `mazesearch.json` — the search loop, end to end

331 complete searches from four designs, **27,672 expansions**. Each drives the heap setup's
output, the relaxation, the heap primitives and the stopping rule together, and is checked on the
meeting point it arrives at **and** the detour flags it leaves behind.

⛔ **This is the strongest statement available about the search.** A meeting point that matches
after a hundred expansions depends on every pop order, every parent recorded and every cost looked
up along the way. A single relaxation matching says far less.

### Why the detour flags had to be captured too

The guard deciding whether a detour is even *considered* changes no distance and no meeting point
— only which cells are marked, and those decide the backtrace later. Both mutations of that guard
survived a meeting-point-only comparison. With the flags captured, both die.

⟹ **A guard whose only effect is on state a later stage reads needs that state in the corpus**,
not just the result the current stage returns.

### Two guards that are mutually redundant

The loop leaves the predecessor equal to the cell when the distance is zero, so the via flag it
passes is already false at a source; the relaxation then refuses the via again on the same
condition. Removing **either** is a mutation nothing can kill — removing **both** would charge a
via for turning at a source. Both are transcribed, and neither survivor is a corpus gap.

⚠️ The cell index is decomposed with the distance grid's **allocated** row stride, not the
design's grid width — a thousand here, against a design barely thirty wide.

## `splitedge.json` and `rewire.json` — rebuilding the tree around a moved node

**100 splits** and **509 rewirings** from four designs. Both compare the **whole tree** before and
after — every node's neighbours and every edge's endpoints — not just the part that changed. A
rebuild that re-points four of five nodes still leaves a tree, and still a wrong one.

### The pin stand-in

A pin cannot be relocated, so a duplicate node is created at the same coordinates and joined to it
by a **zero-length edge**; the duplicate takes over the pin's connections and moves instead.

⛔ The stand-in **inherits the pin's alias** rather than taking its own, so the two share
connection state exactly as coincident nodes do elsewhere in this engine.

⚠️ The pin's neighbour list **shrinks** — the caller is dropped outright and another entry is
rebuilt onto the stand-in. All four shapes are present: the pin holding two or three neighbours,
with the caller first in its list or not.

### The rewiring, and where order matters

⛔ **301 of the 509 rewirings have the four surrounding nodes NOT all distinct** — a neighbour of
the moved node is also an endpoint of the edge it landed on. Each patch replaces the **first**
matching entry, so where two of them touch the same node, the order they run in decides the
answer. Reordering them fails the gate; that count is asserted, because a corpus of only distinct
cases would say nothing about it.

🔑 Replacing **every** match rather than the first is a mutation nothing can kill, and that one is
structural: a node's neighbours are distinct, so there is never more than one match. Measured
across 11,084 captured neighbour lists, not one holds a duplicate. Transcribed as written, because
the reference relies on that rather than enforcing it.

## `e2e.json` — the per-edge call sequence, end to end

360 re-routed edges from four designs, 1,582 path points, 279 on multi-pin nets. Each record
carries the net's whole tree, every threshold and knob, and the usage patch over the search
region — everything the sequence reads — and is checked on the path it produced.

⛔ **This validates the THREADING between stages, not the stages.** Each is already gated
separately. What it catches is a region computed correctly and handed to the wrong seeding, or a
search whose result is backtraced against the wrong state — failures a per-stage corpus cannot see
precisely because every stage is individually right.

⚠️ The sequence reports the region it searched and the frontiers it seeded, not just the path.
Without that, handing the seeding a region one cell too small survived the comparison: the path
usually comes out the same anyway.

### What this corpus cannot decide

Three mutations survive here and die elsewhere, because every edge in it **passed both gates** and
was routed **monotonically**:

| mutation | where it dies |
| --- | --- |
| the region capped by the edge's span rather than its route length | the region corpus — **185** of its edges have a route longer than their span; this one has **none** |
| the length gate skipped | beside the region, where the boundary is constructed |
| the rip-up gate ignored | against its own captured verdicts, which include refusals |

A test asserts that this corpus still has no edge routed the long way round, so the note goes stale
loudly rather than quietly.

⚠️ One more survivor is a property of the transcription, not the reference: the search grid's row
stride. The reference's matters because it decomposes a flat cell index that crosses function
boundaries; no flat index escapes this crate, so any sufficient width round-trips.

## `netloop.json` — the per-net loop and its retry

40 passes from four designs, **19,800 net visits**, each carrying the nets entered **in order** —
so a retry shows as a repeat at the same index. A count of visits could not tell "processed every
net once" from "skipped one and retried another".

### The recovery is unreachable on every shipped design

A surgery that cannot place a contact point clears the net's tree, rebuilds it from the pins, and
reprocesses the **same** net. Swept across **all 148 traceable designs: 713,915 net visits, zero
retries.**

⛔ It is transcribed anyway and pinned by constructed cases — that the failing net is entered again
**in place** rather than skipped, and that nothing bounds the repetition. The absence is asserted
too, so a recapture that ever reaches it fails loudly instead of quietly invalidating these notes.

⚠️ The reference steps its net index **back** before breaking out of the edge loop, so the loop's
own increment returns to the same net. Writing that as "move on to the next net" is the natural
mistake, and nothing in any corpus would catch it.

## `removeloops.json` — cutting loops out of routed paths

750 edges kept from 17,542 scanned. A maze route can revisit a cell — the search is over a grid,
not a tree — so this stage finds a repeated point, removes everything between the two visits,
gives back exactly the demand that stretch was charged, and **restarts the scan**.

⛔ **No captured path contains a loop.** Swept across **all 148 traceable designs: 79,813 edges
scanned, zero loops.** The corpus therefore decides only that the scan leaves a loop-free path
untouched and charges nothing back — worth having, and not the same as validating the removal.

The removal, the give-back and the restart are pinned by constructed cases, and the absence is
asserted so a recapture that finds a loop fails loudly.

### Why the restart is not merely conservative

Removing a stretch shifts everything after it **down**, so a duplicate pair that sat beyond the
index can land entirely before it — where a continuing scan would never look again. The case
showing this had to be built deliberately: the obvious two-loop path is cleaned correctly either
way, because its second loop happens to stay above the index.

🔑 **And because it restarts, at most one earlier point can ever match.** When the scan reaches an
index, no two earlier points are equal — if they were it would have stopped at the second of them.
So taking the last match rather than the first is a mutation nothing can kill: a property of the
algorithm, not a gap in the corpus.

## `layerassign.json` — layer assignment's reset and edge registration

1,324 nets from four designs, 424 of which alias a node (699 aliased nodes in all).

⛔ **These two passes are the outward walk's, bar two values.** A literal diff of the two reference
functions shows every other line identical: layer assignment starts the per-node counters at the
reference's infinity rather than zero, and marks a terminal **1** rather than **2**.

They share one implementation here, parameterised on those two values, rather than a copy that can
drift. **This corpus is what holds that claim** — if either reference function ever diverges
further, the shared code stops matching one of the two goldens.

⚠️ A degenerate edge's alias fields are left untouched by the reference, so the captured values
are whatever was already there. The test asserts they are *unwritten*, not that they hold a
particular value.

## `layerextremes.json` — which edges reach each node's highest and lowest layers

900 nets from four designs, **4,317 nodes keeping the sentinel**.

This is the pass that explains the infinity constant. The two per-node fields hold the **edge id**
at the node's extreme layers, not a count, and the sentinel means "no edge rises above (or drops
below) this node's own layer".

⛔ **A terminal starts at its pin layer**, so an edge that stays on that layer sets neither field
and the sentinel survives. A Steiner node starts wide open and is always set by its first edge.

⛔ **The comparisons are strict, so the first edge to reach a layer keeps it.** A later edge
arriving on the same layer is still recorded in the node's edge list but does not displace it —
which makes the result depend on edge order, not only on layers.

⚠️ Both ends of every edge are recorded, on the **alias** nodes, and each end takes the layer where
the edge *meets it* — the first grid point for one end, the last for the other. A via partway
along is invisible here.

## `assignedge.json` — the layer-assignment dynamic program

1,001 edge assignments from **35 designs, captured in two cost modes**, 468 of which change layer
somewhere. Both directions: 750 forward, 251 backward.

⛔ **The second mode is the point.** The reference's wire and via cost functions return zero
outright unless the router runs resistance-aware, which is off by default. A corpus taken from
default runs multiplies every cost weight in the program by zero, and a reimplementation that
drops the wire cost entirely scores exactly the same on it. A `global_route -resistance_aware`
sweep contributed 716 of these cases — 594 with a live wire cost, 237 with a live via cost — and
killed that mutation outright. ⚠️ **More designs would not have found it; a second mode did.**

⛔ **Three cost tiers, and the ordering between them is the policy.** A usable step costs one plus
the layer's wire cost. A layer whose orientation does not match the step, or which lies outside
the net's permitted range, costs *twice* the infinity constant; anything else short of resources
costs it once. So when nothing is usable the program still picks a layer, preferring one that
merely lacks resources over one that cannot carry the step at all.

⚠️ **The availability table is captured rather than the 3D usage it is derived from.** That splits
the stage: this golden decides the program, and the table's own derivation is a separate piece
with its own inputs.

⛔ **The last column of that table is never written by the reference** — all 6,943 captured cells
are zero — yet the backward direction reads its orientation from exactly there on the first step.
Recorded as finding 5 in the programme's upstream notes and pinned by a test here.

⚠️ Five deliberate mutations survive this corpus, all measured rather than assumed: three cost
weights that shift costs without moving any `argmin`, the narrowing of the running minimum to an
`int` (the worst cost reached is within **7%** of overflowing), and the "still unset" tie-break
clause (reached 19 times, changes the selected layer 3 times, changes no assignment — the
read-back follows the same link either way). Each has a test stating the measurement, so a corpus
that later does reach one fails the gate instead of silently invalidating the note.

## `layertable.json` — the availability table's derivation

1,040 edges from 69 designs: the per-step free resource on every layer, the layer directions, both
edge-cost thresholds, the 2D-overflow flag, and the net's layer range **before and after**.

⛔ **The range is captured twice because this pass can widen it.** When no layer in the net's range
can carry a step, the reference reaches outside the range and writes the widened bound back onto
the net — so later steps of the same edge, and the dynamic program afterwards, see the wider range.
A golden recording the range once could not tell a pass that widens from one that does not.

⛔ **That path fires on none of these 1,040 edges, and the absence is measured.** It needs no
in-range layer to fit *and* two-dimensional routing to have left no overflow. Across 29,264 steps
the first condition is met **494 times and the second never coincides** — every exhausted step
belongs to a run that still had 2D overflow. The two are naturally opposed. The path's rules are
therefore pinned by constructed cases, and a tripwire asserts the absence so the day a design does
reach it the gate fails rather than the note going stale.

⚠️ **Two different thresholds for "enough".** The in-range scan compares a layer's free resource
against that *layer's* edge cost; the reach-outside scan compares the best found so far against the
*net's* edge cost. Mutating either into the other is caught.

⚠️ The per-step resource is captured rather than the 3D capacity and usage arrays behind it: that
read is a database walk indexed by the step's position, not a rule. Splitting there keeps this
golden about the rules.

## `netpinorder.json` — the net ordering layer assignment walks in

115 calls, **33,359 nets**, from 71 designs in **both cost modes** — 48 default, 67
resistance-aware.

⛔ **Two of the six sort keys are identically zero without resistance-aware.** The clock key is `0`
outright in a default run, and the score key is `0` for every net that is not flagged — which is
all of them. A default-only corpus therefore decides four keys out of six. Same trap as
`assignedge.json`, and the reason both sweeps now run by default.

⛔ **Not the congestion ordering, despite the family resemblance.** That one accumulates per-edge
overflow into a field called `xmin`, then mutates slack and re-sorts; this one takes a real
minimum x, sorts once, and mutates nothing. Diffing the two reference functions was what
established they share nothing — the name was the only similarity.

⚠️ **The minimum runs in a narrow type and is stored in a wide one.** `int16_t` for the reduction,
`int` in the record, so a net with no edges keeps **32,767** rather than the larger type's maximum.
No captured net is edgeless, so that value is pinned by a constructed case with the absence
asserted.

⚠️ **The minimum is over each edge's FIRST node only** — the second endpoint never enters it, so
this is not the net's leftmost point.

⚠️ The per-net score is captured rather than derived: the reference divides four net properties by
four run-wide worst-case values, which is state this pass does not own.

11 of 13 mutations die. The two survivors are measured: the division's precision is unobservable
because no net's total length reaches 2²⁴, where a `float` stops being exact (asserted as a
threshold); and the sort's stability cannot matter because the net index is the last key, so no
two records ever compare equal (asserted as the precondition).

## `full3d.json` — expanding a routed edge into a full three-dimensional path

836 edges from 69 designs: **633 that gain points, 143 converted without changing, and 60 the pass
skips**, captured on both sides so a skipped edge can be shown to come out byte-identical.

⛔ **The inserted points take the NEXT point's coordinates.** A step that changes layer becomes a
move in the plane and then a stack of vias at the destination — never a stack at the origin
followed by a move. Both directions start at the *current* layer and stop before the next one, so
the first inserted point repeats the origin's layer at the destination's position.

⚠️ **`routelen` is the authority, not the number of points.** The walk reads that many steps plus
one final point; anything beyond is dropped.

⛔ **Three behaviours no design witnesses**, each pinned by a constructed case with the limitation
asserted so a later capture that reaches one fails loudly:

| unwitnessed | why the corpus cannot decide it |
|---|---|
| the route-type stamp | every captured edge already enters as a maze route |
| the length gate's boundary | all 60 skipped edges hold one point and no steps, so converting them would be a no-op either way |
| points past the step count | no captured edge carries any |

13 of 13 mutations die — including every loop bound, both coordinate choices, the final push, the
step-count recomputation and the stamp.

## `updateslacks.json` — choosing which nets are treated as resistance-aware

87 calls, **11,564 nets, 993 surviving the filters**, from 41 designs.

⛔ **There is no default-mode half of this corpus, because there is nothing to capture.** The pass
returns before touching anything unless the router is resistance-aware *and* a liberty library is
loaded — so in a default run no net's slack, length or resistance is written at all.

⛔ **The score divides by worst-case values that are still being accumulated.** Each net is scored
inside the same loop that updates the worst metrics, so it divides by the worst seen *so far*,
including itself — not by the final values. The golden captures the worst metrics **as they stood
at each net**, so hoisting the accumulation into its own pass is caught per net rather than only
if it happens to move the marked set. 100+ of the 993 scores differ from what the final metrics
would give, and the test asserts that, so the gate cannot go vacuous.

⚠️ **Slack and length are written for EVERY net, including the ones the pass then skips.** Only
resistance, the worst metrics and candidacy are confined to the survivors.

⛔ **Two boundaries no design reaches**, both measured and pinned by constructed cases:

| unwitnessed | measurement |
|---|---|
| the incremental exemption from the positive-slack filter | 48 incremental nets reach the condition; **all 48** are already short or unconstrained, so it never decides |
| slack of exactly zero | **0 of 11,564** nets have it, so `> 0` and `>= 0` cannot be told apart |

15 of 15 mutations die.

## `softndr.json` — the soft-NDR demotion, captured where it actually runs

8 demotions and **328 planar writes** from 2 designs.

⛔ **Captured through a different caller than the one it belongs to.** R17's own gate never fires
(finding 8), so `applySoftNDR` and the planar usage update were first transcribed from the source
and pinned by constructed cases only. They are **also** called from the congestion loop's
escalation path, which does fire — so running the same probe there gives the reference's own
answer for the shared helpers. ⚠️ It caught nothing, which is the result worth having: the
constructed cases were right, and now that is measured rather than assumed.

⚠️ **The write ORDER is the gate, not the totals.** The reference refunds the entire route at the
old edge cost and only then charges it at the new one, so the sequence is `n` negative writes
followed by `n` positive ones over the same positions. Summing the deltas would score a single
signed pass as correct.

⛔ **What this corpus cannot decide**, asserted rather than implied: every captured demotion
starts from the same edge cost (9), no net is demoted twice, and **the layered bracket is never
exercised** — the caller that fires does not use it. The recorder panics if our code writes
layered usage on this path, so reaching further than the reference does is a failure, not a pass.
R17's congestion scan and layered bracket stay pinned by constructed cases; see finding 8.
