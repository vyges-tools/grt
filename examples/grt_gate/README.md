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

## ⬜ What this corpus cannot discriminate

Three rules can each be replaced by a plausible wrong one and this design produces **identical
output**. Mutation testing found all three; the passing gate found none of them.

| rule | a wrong version that also passes | why it passes here |
| --- | --- | --- |
| the **first** guide to reach a pin's route point claims it | mark every guide that touches one | no net touches one of its own pin points twice |
| locality compares **position only** | compare the layer as well | no net has two pins at one point on different layers |
| a net with **no pins** is local | say it is not | no net has zero pins |

Synthetic cases in `tests/guides.rs` and `tests/init.rs` cover all three, and
`the_corpus_CANNOT_DISCRIMINATE_these_three_rules_and_says_so` asserts the three counts are zero —
so a richer design announces that a gap has closed rather than leaving it to be rediscovered.

⟹ This design is small and well behaved. A second corpus should be chosen for what it makes
*different*, not for being bigger.

## `net_order.ok` — a second corpus, chosen on exactly that criterion

The order nets are handed to the router is *non-leaf clock nets first, then the rest, each group
sorted by name*. On many designs the clock nets sort first anyway, so that is indistinguishable
from sorting the whole list — `gcd` has no clock nets at all, and `clock_route`'s two are named
`clk` and `clknet_0_clk`, which sort first regardless.

`net_order.ok` is 348 nets from a design where the full list is **not** name-sorted, so the two
rules give different answers and the test can fail. The nets are fed to `order_nets` **reversed**,
to show the answer depends on the rules rather than on input order.
