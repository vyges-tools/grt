# vyges-grt — global routing

A global router decides, for each net, which cells of a coarse routing grid the detailed router
may use. The answer is a set of **route guides**, one rectangle per segment on a layer, and the
guides are what detailed routing (`vyges-drt detailed_route`) consumes.

`vyges-grt` routes a placed design and writes those guides. Two routers sit behind the
`global_route` step: the default negotiation-based router, with pattern and maze routing, 3D layer
assignment and rip-up and reroute, and a second, CUGR-style router selected with `use_cugr`. Both
handle non-default rules, incremental routing and antenna repair, and timing-driven routing is
timed by the engine's own timer.

## Run it

The input is a JSON **job**: the design, and the steps a routing script would run, in order.

```json
{
  "lefs": ["tech.lef", "cells.lef"],
  "def": "placed.def",
  "steps": [
    { "cmd": "set_routing_layers", "signal": ["met1", "met5"] },
    { "cmd": "global_adjustment", "value": 0.5 },
    { "cmd": "global_route" },
    { "cmd": "write_guides", "path": "route.guide" }
  ]
}
```

```sh
vyges-grt route job.json               # the report on stdout
vyges-grt route job.json -o report.json
```

A design can come from a database (`"db": "placed.odb"`) instead of LEF and DEF. Order matters:
a per-layer adjustment is judged against the maximum routing layer at the time it is given, and a
second `global_route` runs on the design the first one left. The full step list is in the
[CLI reference](./reference/vyges-grt.md).

The report is JSON:

```json
{"status":"routed","global_route":[{"nets":2,"total_overflow":0,"congested":false,"clock_nets":[]}],
 "files_written":["route.guide"],"guides_written":["route.guide"],"log":[]}
```

## Status and exit codes

| status | exit | meaning |
| --- | --- | --- |
| `routed` | 0 | every step ran; the report lists each `global_route` and every file written |
| `vacuous` | 2 | every step ran but none produced anything: no `global_route` and no file written. **Not a pass** |
| `error` | 2 | usage, unreadable input, a failed write, or routing that ends congested without `allow_congestion` |
| `refused` | 3 | a step needs something this engine does not model; `reason` names it |

Diagnostics go to stderr as `vyges-events` records (`GRT-DONE`, `GRT-REFUSED`, `GRT-ERROR`, and
the design database's own messages), so stdout carries nothing but the report.

## Limits

- **One input is an oracle.** `repair_antennas` takes the antenna violations as a reference
  checker reports them (its `violations` file); the violation check itself is not modelled, and
  diode insertion is refused. `timer_slacks`, when given, also answers the timer from a captured
  file instead of computing the slacks.
- **Refused rather than approximated:** reading pin access points during antenna checking; a
  second antenna-repair iteration; incremental routing with no CUGR route in the session; design
  edits inside an incremental bracket that are not modelled (each named); RC settings before a
  liberty library; a clock period with no liberty library to time it.
- The Steiner trees come from `vyges-stt` and the placement parasitics from `vyges-est`, each
  correlated separately.
