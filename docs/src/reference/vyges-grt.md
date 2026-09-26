# vyges-grt — CLI reference

_Generated from `vyges-grt --help` — this page is the tool's own output, verbatim._

```text
vyges-grt — global routing: route guides from a placed design

USAGE:
  vyges-grt route <job.json> [-o REPORT.json]
  vyges-grt --describe
  vyges-grt --help
  vyges-grt --version

JOB (JSON):
  { "lefs": [..], "liberty": [..], "def": ".." | "db": "..", "timer_slacks": "..",
    "steps": [ STEP, .. ] }
  timer_slacks: slacks captured from a reference timer, `<call> <net> <f32 hex bits>` per line.
  With liberty and no clock every net is unconstrained; with a clock and a period the slacks are timed at each
  read (captured timer_slacks, when given, answer instead); with a clock and neither, a pass that reads slacks is refused.
  STEP is one of
    { "cmd": "set_routing_layers", "signal": [lo, hi], "clock": [lo, hi] }
    { "cmd": "layer_adjustment", "layers": [lo, hi], "value": f }      (one layer: lo == hi)
    { "cmd": "global_adjustment", "value": f }                         (the `*` form)
    { "cmd": "region_adjustment", "rect_um": [x0, y0, x1, y1], "layer": l, "value": f }
    { "cmd": "routing_alpha", "alpha": f, "nets": [..] | "min_fanout": n | "min_hpwl": um | "clock_nets": true }
    { "cmd": "nets_to_route", "patterns": [..] }                        (Tcl globs)
    { "cmd": "global_route", "verbose": b, "allow_congestion": b, "grid_origin": [x, y],
      "skip_large_fanout": n, "congestion_iterations": n, "critical_nets_percentage": f,
      "resistance_aware": b, "res_aware_nets_percentage": f, "use_cugr": b }
      (use_cugr: CUGR — its model and every stage written to the file $VYGC_OUT names)
    { "cmd": "create_clock", "ports": [..], "name": s, "period": f, "waveform": [r, f] }  (user units: the slacks are timed)
    { "cmd": "set_input_delay" | "set_output_delay", "value": f | "period_times": f,
      "ports": [..] | "inputs_except_clocks" | "outputs" }      (relative to the clock's rise edge)
    { "cmd": "set_layer_rc", "layer" | "via": name, "resistance": f } (user units)
    { "cmd": "propagated_clock" }
    { "cmd": "estimate_parasitics", "placement": b }            (what a timer read with no estimate of its own sees)
    { "cmd": "write_parasitics", "path": ".." }       (the networks the slacks are read from)
    { "cmd": "write_spef", "path": "..", "source": "partial" | "routed" }
    { "cmd": "write_guides", "path": ".." }
    { "cmd": "write_db", "path": ".." }            (the database as global_route leaves it: guides, gcell grid, what detailed routing reads)
    { "cmd": "antenna_wires", "path": ".." }      (the wires antenna checking synthesises from the guides)
    { "cmd": "repair_antennas", "violations": "..", "jumper_only": b, "diode_only": b, "iterations": n,
      "allow_congestion": b, "trace": ".." }
      violations: the reference checker's, captured (`VYGA|viol|…` lines) — an ORACLE; the
      violation check itself is not modelled. Diode insertion is refused.

OPTIONS:
  -o FILE      write the JSON report to FILE instead of stdout
  --json       accepted; the report is JSON either way
  --describe   print a machine-readable JSON description of the command

EXIT STATUS:
  0  routed    every step ran; the report lists each global_route and every file written
  2  vacuous   every step ran but none produced anything (no global_route, no file written).
               NOT a pass
  2  error     usage, unreadable input, or a failed write
  3  refused   a step needs a feature not modelled yet (named in `reason`)
```
