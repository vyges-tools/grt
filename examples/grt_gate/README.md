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

## ⬜ The one gap that remains

A net with **no pins** is local. No design here has one, so replacing that rule with "a pinless
net is not local" still produces identical output everywhere; only a synthetic case covers it.
`ONE_gap_remains_of_the_three_and_the_suite_says_which` asserts the count, so adding such a design
makes the suite say so.

## `net_order.ok` — a second corpus, chosen on exactly that criterion

The order nets are handed to the router is *non-leaf clock nets first, then the rest, each group
sorted by name*. On many designs the clock nets sort first anyway, so that is indistinguishable
from sorting the whole list — `gcd` has no clock nets at all, and `clock_route`'s two are named
`clk` and `clknet_0_clk`, which sort first regardless.

`net_order.ok` is 348 nets from a design where the full list is **not** name-sorted, so the two
rules give different answers and the test can fail. The nets are fed to `order_nets` **reversed**,
to show the answer depends on the rules rather than on input order.
