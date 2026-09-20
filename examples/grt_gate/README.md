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

## ⬜ What this corpus does NOT reach

The flag is claimed by the **first** guide to reach a route point where a pin sits; later guides
over the same point are left unmarked. **No net in this design touches one of its own pin points
twice**, so replacing that rule with "mark every guide that touches a pin point" produces
identical output here. The whole-design gate cannot tell the two apart — only the synthetic case
in `tests/guides.rs` does.

`the_corpus_does_NOT_witness_the_first_claim_tie_break_and_says_so` asserts that count is zero, so
a richer corpus will announce that the gap has closed rather than leaving it to be rediscovered.

## Regenerating

The corpus and the golden must be captured from the **same run**, or they describe different
designs. `tests/end_to_end.rs` reads both and compares every guide, per net and in order.
