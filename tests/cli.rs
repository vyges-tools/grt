// SPDX-License-Identifier: Apache-2.0
//! The `vyges-grt` command's contract, on a tiny design (`tests/data/tiny.{lef,def}`, shared with
//! vyges-drt: two buffers, a port, two nets): exit status, the status word, the files named, and
//! the descriptor.
//!
//! ⚠️ The routed output below is a REGRESSION pin (this engine's own answer on this design), not a
//! correlation claim; correlation against the reference router runs outside this repository.
#![cfg(feature = "cli-core")]

use std::path::PathBuf;
use std::process::{Command, Output};

fn bin() -> Command {
    Command::new(env!("CARGO_BIN_EXE_vyges-grt"))
}

fn data(f: &str) -> String {
    format!("{}/tests/data/{f}", env!("CARGO_MANIFEST_DIR"))
}

fn tmp(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("vyges-grt-{}-{name}", std::process::id()))
}

/// Run `route` on a job written to a temp file; returns the output and the parsed report.
fn route(name: &str, job: serde_json::Value, extra: &[&str]) -> (Output, serde_json::Value) {
    let path = tmp(name);
    std::fs::write(&path, job.to_string()).unwrap();
    let o = bin().arg("route").arg(&path).args(extra).output().unwrap();
    let report = serde_json::from_slice(&o.stdout).unwrap_or(serde_json::Value::Null);
    (o, report)
}

fn descriptor() -> serde_json::Value {
    serde_json::from_slice(&bin().arg("--describe").output().unwrap().stdout).expect("--describe must be valid JSON")
}

#[test]
fn help_describe_and_version_exit_zero() {
    for a in ["--help", "--describe", "--version"] {
        assert!(bin().arg(a).output().unwrap().status.success(), "{a}");
    }
    let d = descriptor();
    assert_eq!(d["schema"], "vyges-tool-descriptor/1.1");
    assert_eq!(d["name"], "grt");
    assert!(["discovered", "structured", "workflow-validated"].contains(&d["maturity"].as_str().unwrap()));
}

/// ⛔ The pin the binary LINKS, not a typed one: the placeholder must not survive into the output.
#[test]
fn the_descriptor_reports_the_pin_this_binary_was_built_against() {
    let d = descriptor();
    assert_eq!(d["openroad_pin"], vyges_opendb::OPENROAD_PIN);
    assert_eq!(d["openroad_pin"].as_str().unwrap().len(), 40, "a full commit SHA");
}

/// The assertion is `field` + `pass_when` with ONE predicate — the form the registry accepts —
/// on the two pass words (`routed`, and `up_to_date`: every routable net was already wired), and
/// the artifact field is one the report actually carries. `vacuous` is not among them.
#[test]
fn the_assertion_passes_on_routed_or_up_to_date() {
    let d = descriptor();
    assert_eq!(d["assertion"]["field"], "status");
    assert_eq!(d["assertion"]["pass_when"], serde_json::json!({ "in": ["routed", "up_to_date"] }));
    assert_eq!(d["artifacts"][0]["field"], "guides_written");
}

#[test]
fn a_bad_invocation_exits_two() {
    assert_eq!(bin().output().unwrap().status.code(), Some(2));
    assert_eq!(bin().args(["route"]).output().unwrap().status.code(), Some(2));
    assert_eq!(bin().args(["route", "missing.json"]).output().unwrap().status.code(), Some(2));
    assert_eq!(bin().args(["route", "x.json", "-o"]).output().unwrap().status.code(), Some(2));
}

/// ⛔ Every step ran and none produced anything: VACUOUS, exit 2 — never `routed`.
#[test]
fn a_job_that_produces_nothing_is_vacuous() {
    let (o, r) = route("vacuous.json", serde_json::json!({ "steps": [] }), &[]);
    assert_eq!(o.status.code(), Some(2));
    assert_eq!(r["status"], "vacuous", "{r}");
}

/// Refused is exit 3 — the suite's word for "not modelled", distinct from a usage error (2).
#[test]
fn an_unmodelled_step_is_refused_with_exit_three() {
    let (o, r) = route("refused.json", serde_json::json!({ "steps": [{ "cmd": "no_such_step" }] }), &[]);
    assert_eq!(o.status.code(), Some(3));
    assert_eq!(r["status"], "refused", "{r}");
    assert!(r["reason"].as_str().unwrap().contains("no_such_step"), "{r}");
}

/// Routed: exit 0, both nets routed, and the guide file the job wrote is named in the report —
/// the descriptor's artifact points at that field. `--json` is accepted and changes nothing.
#[test]
fn the_tiny_design_is_routed_and_its_guides_named() {
    let guides = tmp("guides.txt");
    let job = serde_json::json!({
        "lefs": [data("tiny.lef")],
        "def": data("tiny.def"),
        "steps": [{ "cmd": "global_route" }, { "cmd": "write_guides", "path": guides.to_str().unwrap() }]
    });
    let (o, r) = route("routed.json", job, &["--json"]);
    assert_eq!(o.status.code(), Some(0), "{}", String::from_utf8_lossy(&o.stderr));
    assert_eq!(r["status"], "routed", "{r}");
    assert_eq!(r["global_route"][0]["nets"], 2, "{r}");
    assert_eq!(r["guides_written"], serde_json::json!([guides.to_str().unwrap()]), "{r}");
    assert!(std::fs::metadata(&guides).unwrap().len() > 0);
}

/// `-o FILE` writes the report there and nothing on stdout.
#[test]
fn the_report_goes_to_the_file_o_names() {
    let out = tmp("report.json");
    let (o, _) = route("o.json", serde_json::json!({ "steps": [] }), &["-o", out.to_str().unwrap()]);
    assert_eq!(o.status.code(), Some(2));
    assert!(o.stdout.is_empty());
    let r: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&out).unwrap()).unwrap();
    assert_eq!(r["status"], "vacuous");
}

/// ⚠️ The book's CLI reference is `--help` verbatim; regenerate it when USAGE changes.
#[test]
fn the_book_reference_is_the_help_verbatim() {
    let help = String::from_utf8(bin().arg("--help").output().unwrap().stdout).unwrap();
    let page = include_str!("../docs/src/reference/vyges-grt.md");
    assert!(page.contains(&format!("```text\n{help}```")), "docs/src/reference/vyges-grt.md is stale");
}

/// A DEF whose cells no LEF defines fails with a reason a user can act on: the file, the generic
/// error, its cause (the cell and instance) and what to do — not libodb's bare `ODB-0421`.
#[test]
fn a_def_with_undefined_cells_names_the_file_the_cause_and_the_fix() {
    let lef = std::fs::read_to_string(data("tiny.lef")).unwrap();
    let tech = tmp("tech.lef");
    std::fs::write(&tech, format!("{}\nEND LIBRARY\n", &lef[..lef.find("\nMACRO").unwrap()])).unwrap();
    let (o, r) = route("undefined.json", serde_json::json!({
        "lefs": [tech.to_str().unwrap()], "def": data("tiny.def"), "steps": [{ "cmd": "global_route" }]
    }), &[]);
    assert_eq!(o.status.code(), Some(2));
    let reason = r["reason"].as_str().unwrap();
    assert!(reason.starts_with(&format!("{}: ODB-0421: DEF parser returns an error!", data("tiny.def"))), "{reason}");
    assert!(reason.contains("caused by ODB-0092: unknown library cell referenced (BUF) for instance (u1)"), "{reason}");
    assert!(reason.contains("a LEF defining that cell was not read"), "{reason}");
}

/// `write_db` writes the database as global routing leaves it — what detailed routing reads: each
/// routed net's guides, the gcell grid (one pattern per axis), and `grt_use_cugr` false after the
/// default router. ⚠️ A regression pin on this design; the gate against the reference's database is
/// grt-db-gate.py, outside this repository.
#[test]
fn write_db_leaves_guides_and_the_gcell_grid_in_the_database() {
    let db_out = tmp("routed.odb");
    let (o, r) = route("write_db.json", serde_json::json!({
        "lefs": [data("tiny.lef")], "def": data("tiny.def"),
        "steps": [{ "cmd": "global_route" }, { "cmd": "write_db", "path": db_out.to_str().unwrap() }]
    }), &[]);
    assert_eq!(o.status.code(), Some(0), "{r}");
    assert_eq!(r["files_written"], serde_json::json!([db_out.to_str().unwrap()]), "{r}");
    let db = vyges_opendb::Db::open(&db_out).unwrap();
    let routed: Vec<String> = db.net_names().into_iter().filter(|n| !db.net_guides(n).unwrap().is_empty()).collect();
    assert_eq!(routed.len(), 2, "both nets carry guides: {routed:?}");
    assert!(db.gcell_grid_pattern_x(0).is_ok() && db.gcell_grid_pattern_y(0).is_ok(), "a gcell grid is written");
    assert!(db.gcell_grid_pattern_x(1).is_err() && db.gcell_grid_pattern_y(1).is_err(), "one pattern per axis");
    assert_eq!(db.block_bool_property("grt_use_cugr").unwrap(), Some(false));
}

/// tiny.lef plus `DIODE0`: a CORE ANTENNACELL whose one signal terminal has NO diffusion area.
fn lef_with_dead_diode() -> String {
    let lef = std::fs::read_to_string(data("tiny.lef")).unwrap();
    let cut = lef.rfind("END LIBRARY").unwrap();
    let diode = "MACRO DIODE0\n  CLASS CORE ANTENNACELL ;\n  ORIGIN 0 0 ;\n  SIZE 0.5 BY 1.2 ;\n  SITE core ;\n  PIN DIODE\n    DIRECTION INPUT ;\n    USE SIGNAL ;\n    PORT\n      LAYER M1 ;\n        RECT 0.1 0.25 0.4 0.45 ;\n    END\n  END DIODE\nEND DIODE0\n\n";
    // ⛔ one file PER CALL: the tests run in parallel, and a shared path was read by one test while
    // another rewrote it (an empty LEF: "undefined site (core)") — flaky on a loaded machine.
    static SEQ: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let path = tmp(&format!("dead-diode-{}.lef", SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed)));
    std::fs::write(&path, format!("{}{diode}{}", &lef[..cut], &lef[cut..])).unwrap();
    path.to_str().unwrap().to_string()
}

fn repair_with(name: &str, repair: serde_json::Value) -> (Output, serde_json::Value) {
    route(name, serde_json::json!({
        "lefs": [lef_with_dead_diode()], "def": data("tiny.def"),
        "steps": [{ "cmd": "global_route" }, repair]
    }), &[])
}

/// `repair_antennas [diode_cell]` resolves the cell in its Tcl, before the C++ runs: `findMaster`
/// (GRT-69 when absent), then the master's terminals that are neither POWER nor GROUND — exactly
/// one, or GRT-73. Upstream `GlobalRouter.tcl`, `proc repair_antennas`.
#[test]
fn a_named_diode_cell_must_exist_and_have_one_signal_terminal() {
    let (o, r) = repair_with("diode-missing.json", serde_json::json!({ "cmd": "repair_antennas", "diode_cell": "NOPE" }));
    assert_eq!(o.status.code(), Some(2), "{r}");
    assert_eq!(r["reason"], "[ERROR GRT-0069] Diode cell NOPE not found.", "{r}");
    let (o, r) = repair_with("diode-two.json", serde_json::json!({ "cmd": "repair_antennas", "diode_cell": "BUF" }));
    assert_eq!(o.status.code(), Some(2), "{r}");
    assert_eq!(r["reason"], "[ERROR GRT-0073] Diode cell has more than one non power/ground port.", "{r}");
}

/// The named cell is used AS GIVEN: `findDiodeMTerm` (which skips a terminal with no diffusion
/// area) is only the fallback when no cell is named. So naming `DIODE0` reaches the C++'s
/// `diffArea(diode_mterm) == 0.0` error (GRT-244), while naming nothing finds no usable diode and
/// warns (GRT-246) and does nothing. Upstream `GlobalRouter::repairAntennas`.
#[test]
fn a_named_diode_bypasses_the_search_and_its_diffusion_area_is_checked() {
    let (o, r) = repair_with("diode-dead.json", serde_json::json!({ "cmd": "repair_antennas", "diode_cell": "DIODE0" }));
    assert_eq!(o.status.code(), Some(2), "{r}");
    assert_eq!(r["reason"], "[ERROR GRT-0244] Diode DIODE0/DIODE ANTENNADIFFAREA is zero.", "{r}");
    let (o, r) = repair_with("diode-none.json", serde_json::json!({ "cmd": "repair_antennas" }));
    assert_eq!(o.status.code(), Some(0), "{r}");
    assert!(r["log"].as_array().unwrap().iter().any(|l| l == "[WARNING GRT-0246] No diode with LEF class CORE ANTENNACELL found."), "{r}");
    // The same line is an event on stderr, under its code and at its level.
    let events: Vec<serde_json::Value> = String::from_utf8_lossy(&o.stderr).lines().filter_map(|l| serde_json::from_str(l).ok()).collect();
    assert!(events.iter().any(|e| e["code"] == "GRT-0246" && e["severity"] == "warn" && e["raw_msg"] == "No diode with LEF class CORE ANTENNACELL found."), "{events:?}");
}

/// tiny.lef plus `DIODE1`: a CORE ANTENNACELL whose signal terminal HAS diffusion area.
fn lef_with_live_diode() -> String {
    let lef = std::fs::read_to_string(data("tiny.lef")).unwrap();
    let cut = lef.rfind("END LIBRARY").unwrap();
    let diode = "MACRO DIODE1\n  CLASS CORE ANTENNACELL ;\n  ORIGIN 0 0 ;\n  SIZE 0.5 BY 1.2 ;\n  SITE core ;\n  PIN DIODE\n    DIRECTION INPUT ;\n    USE SIGNAL ;\n    ANTENNADIFFAREA 0.1 ;\n    PORT\n      LAYER M1 ;\n        RECT 0.1 0.25 0.4 0.45 ;\n    END\n  END DIODE\nEND DIODE1\n\n";
    static SEQ: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let path = tmp(&format!("live-diode-{}.lef", SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed)));
    std::fs::write(&path, format!("{}{diode}{}", &lef[..cut], &lef[cut..])).unwrap();
    path.to_str().unwrap().to_string()
}

/// The checker's answers, one block per check in call order (`true`: net n1 violates at u2/A).
fn violation_blocks(name: &str, blocks: &[bool]) -> String {
    let text: String = blocks.iter().map(|&v| if v { "VYGA|viol|n1|level=1|ratio=9|diodes=1|gates=u2/A\n--\n" } else { "--\n" }).collect();
    let path = tmp(name);
    std::fs::write(&path, text).unwrap();
    path.to_str().unwrap().to_string()
}

/// The repair's own lines, codes only, in order: `[INFO GRT-0012] …` → `GRT-0012`.
fn repair_codes(r: &serde_json::Value) -> Vec<String> {
    const REPAIR: [&str; 6] = ["GRT-0012", "GRT-0015", "GRT-0027", "GRT-0243", "GRT-0246", "GRT-0302"];
    let code = |l: &str| l.strip_prefix('[')?.split(']').next()?.split(' ').nth(1).map(str::to_string);
    r["log"].as_array().unwrap().iter().filter_map(|l| code(l.as_str()?)).filter(|c| REPAIR.contains(&c.as_str())).collect()
}

/// `-jumper_only` past the first iteration: with no diodes `nets_to_repair` is never narrowed, so
/// each iteration while violations remain checks every net, runs the jumper pass (every net is new:
/// the violations were cleared at the end of the last iteration) and checks again — and the loop
/// ends when a check finds nothing. Upstream `GlobalRouter::repairAntennas`, `while (violations &&
/// itr < iterations)`; witnessed on every fresh ascon/picorv32 job with `-jumper_only` (10-23).
#[test]
fn jumper_only_runs_the_jumper_pass_every_iteration_while_violations_remain() {
    let pass = ["GRT-0012", "GRT-0302", "GRT-0012"];
    for (blocks, passes) in [(&[true; 6][..], 3), (&[true, true, true, false][..], 2)] {
        let v = violation_blocks(&format!("jo-{passes}.viol"), blocks);
        let (o, r) = route(&format!("jo-{passes}.json"), serde_json::json!({
            "lefs": [lef_with_live_diode()], "def": data("tiny.def"),
            "steps": [{ "cmd": "global_route" }, { "cmd": "repair_antennas", "diode_cell": "DIODE1", "iterations": 3, "jumper_only": true, "violations": v }]
        }), &[]);
        assert_eq!(o.status.code(), Some(0), "{r}");
        assert_eq!(repair_codes(&r), pass.repeat(passes), "{r}");
    }
}

/// Under CUGR a later iteration runs the jumper pass too: after the diodes and the incremental
/// reroute, the next iteration checks the dirty nets, and violations there are new (cleared at the
/// end of the last iteration) — so CUGR is asked for jumpers again, then the nets checked again.
/// Upstream `GlobalRouter::repairAntennas`; witnessed on every fresh ascon/picorv32 job routed
/// with `-use_cugr` (10-23: 13 then 2 jumpers on ascon, 26 then 7 on picorv32).
#[test]
fn a_later_cugr_iteration_runs_the_jumper_pass_again() {
    // tiny.def on a die a CUGR gcell fits in (15 M2 pitches = 3000 > its 2400 height).
    let def = std::fs::read_to_string(data("tiny.def")).unwrap()
        .replace("DIEAREA ( 0 0 ) ( 4000 2400 )", "DIEAREA ( 0 0 ) ( 8000 8000 )")
        .replace("DO 12 STEP 200", "DO 40 STEP 200").replace("DO 20 STEP 200 LAYER", "DO 40 STEP 200 LAYER");
    let big = tmp("cugr-later.def");
    std::fs::write(&big, def).unwrap();
    let v = violation_blocks("cugr-later.viol", &[true, true, true, false]);
    let (o, r) = route("cugr-later.json", serde_json::json!({
        "lefs": [lef_with_live_diode()], "def": big.to_str().unwrap(),
        "steps": [{ "cmd": "global_route", "use_cugr": true }, { "cmd": "repair_antennas", "diode_cell": "DIODE1", "iterations": 2, "violations": v }]
    }), &[]);
    assert_eq!(o.status.code(), Some(0), "{r}");
    assert_eq!(repair_codes(&r), ["GRT-0012", "GRT-0302", "GRT-0012", "GRT-0015", "GRT-0012", "GRT-0302", "GRT-0012"], "{r}");
}

/// Debug flags change what is printed, never a result: with the stage times and every grt debug
/// group on, the tiny design's guides and report are the same as with all off; the stage lines
/// name the router's stages after the reference's functions, and with the flags off none is
/// printed. The old `VYGES_GRT_STAGE_TIMES` still turns the stage times on.
#[test]
fn debug_flags_print_stages_and_change_no_result() {
    let run = |tag: &str, env: &[(&str, &str)]| {
        let guides = tmp(&format!("flags-{tag}.guide"));
        let job = serde_json::json!({
            "lefs": [data("tiny.lef")],
            "def": data("tiny.def"),
            "steps": [{ "cmd": "global_route" }, { "cmd": "write_guides", "path": guides.to_str().unwrap() }]
        });
        let path = tmp(&format!("flags-{tag}.json"));
        std::fs::write(&path, job.to_string()).unwrap();
        let mut c = bin();
        for v in ["VYGES_STAGE_TIMES", "VYGES_DEBUG", "VYGES_GRT_STAGE_TIMES", "VYGES_GRT_INCR_TRACE", "VYGES_GRT_TIMER_STATS"] {
            c.env_remove(v);
        }
        let o = c.envs(env.iter().copied()).arg("route").arg(&path).env("VYGES_LOG_FORMAT", "json").output().unwrap();
        assert_eq!(o.status.code(), Some(0), "{}", String::from_utf8_lossy(&o.stderr));
        let mut report: serde_json::Value = serde_json::from_slice(&o.stdout).unwrap();
        report["guides_written"] = serde_json::Value::Null;
        report["files_written"] = serde_json::Value::Null;
        (report, std::fs::read(&guides).unwrap(), String::from_utf8_lossy(&o.stderr).into_owned())
    };
    let (r_off, g_off, e_off) = run("off", &[]);
    let (r_on, g_on, e_on) = run("on", &[("VYGES_STAGE_TIMES", "1"), ("VYGES_DEBUG", "grt:*=1")]);
    let (r_old, g_old, e_old) = run("old", &[("VYGES_GRT_STAGE_TIMES", "1")]);
    assert_eq!((&r_on, &g_on), (&r_off, &g_off), "a flag changed a result");
    assert_eq!((&r_old, &g_old), (&r_off, &g_off), "a flag changed a result");
    assert!(!e_off.contains("VYGES-STAGE"), "{e_off}");
    for name in ["\"global_route\"", "\"initFastRoute\"", "\"FastRouteCore::run\"", "\"gen_brk_RSMT\"", "\"layerAssignment\"", "\"saveGuides\""] {
        assert!(e_on.contains(name) && e_old.contains(name), "no stage {name} in:\n{e_on}");
    }
    assert!(e_old.contains("VYGES_GRT_STAGE_TIMES is deprecated"), "{e_old}");
}

/// The CUGR tracer behind `grt:vy_cugr_trace`: with the group on it writes `cugr.vygc` under
/// `VYGES_DEBUG_DIR/grt/`, with the old `VYGC_OUT` the file it names (the same lines); either way
/// the guides and the report equal a run with neither, which writes no trace.
#[test]
fn the_cugr_trace_group_writes_the_trace_and_changes_no_result() {
    // tiny.def on a die a CUGR gcell fits in, as the jumper-pass test makes it
    let def = std::fs::read_to_string(data("tiny.def")).unwrap()
        .replace("DIEAREA ( 0 0 ) ( 4000 2400 )", "DIEAREA ( 0 0 ) ( 8000 8000 )")
        .replace("DO 12 STEP 200", "DO 40 STEP 200").replace("DO 20 STEP 200 LAYER", "DO 40 STEP 200 LAYER");
    let big = tmp("cugrtrace.def");
    std::fs::write(&big, def).unwrap();
    let run = |tag: &str, env: &[(&str, &str)]| {
        let guides = tmp(&format!("cugrtrace-{tag}.guide"));
        let job = serde_json::json!({
            "lefs": [data("tiny.lef")],
            "def": big.to_str().unwrap(),
            "steps": [{ "cmd": "global_route", "use_cugr": true }, { "cmd": "write_guides", "path": guides.to_str().unwrap() }]
        });
        let path = tmp(&format!("cugrtrace-{tag}.json"));
        std::fs::write(&path, job.to_string()).unwrap();
        let mut c = bin();
        for v in ["VYGES_STAGE_TIMES", "VYGES_DEBUG", "VYGES_DEBUG_DIR", "VYGC_OUT"] {
            c.env_remove(v);
        }
        let o = c.envs(env.iter().copied()).arg("route").arg(&path).output().unwrap();
        assert_eq!(o.status.code(), Some(0), "{}", String::from_utf8_lossy(&o.stderr));
        let mut report: serde_json::Value = serde_json::from_slice(&o.stdout).unwrap();
        report["guides_written"] = serde_json::Value::Null;
        report["files_written"] = serde_json::Value::Null;
        (report, std::fs::read(&guides).unwrap())
    };
    let dir = tmp("cugrtrace-dbg");
    let old = tmp("cugrtrace-old.vygc");
    let off = run("off", &[]);
    let on = run("on", &[("VYGES_DEBUG", "grt:vy_cugr_trace=1"), ("VYGES_DEBUG_DIR", dir.to_str().unwrap())]);
    let by_old = run("old", &[("VYGC_OUT", old.to_str().unwrap())]);
    assert_eq!(on, off, "the group changed a result");
    assert_eq!(by_old, off, "the old variable changed a result");
    let traced = std::fs::read_to_string(dir.join("grt").join("cugr.vygc")).expect("the group wrote the trace");
    assert!(!traced.is_empty());
    assert_eq!(std::fs::read_to_string(&old).expect("VYGC_OUT still names the file"), traced);
}

/// The router's parasitics are estimated only when a step reads them: the reference's
/// `global_route` estimates nothing after the run (`estimate_parasitics -global_routing` is its
/// own command). With a `write_parasitics` step the routed estimate runs and the file has the
/// nets; without one it does not run (no stage line), and the guides are the same.
#[test]
fn the_parasitics_are_estimated_only_when_a_step_reads_them() {
    let run = |tag: &str, write: bool| {
        let guides = tmp(&format!("lazy-{tag}.guide"));
        let pars = tmp(&format!("lazy-{tag}.par"));
        let mut steps = vec![serde_json::json!({ "cmd": "global_route" }), serde_json::json!({ "cmd": "write_guides", "path": guides.to_str().unwrap() })];
        if write {
            steps.push(serde_json::json!({ "cmd": "write_parasitics", "path": pars.to_str().unwrap(), "source": "routed" }));
        }
        let job = serde_json::json!({ "lefs": [data("tiny.lef")], "def": data("tiny.def"), "steps": steps });
        let path = tmp(&format!("lazy-{tag}.json"));
        std::fs::write(&path, job.to_string()).unwrap();
        let mut c = bin();
        c.env_remove("VYGES_DEBUG").env_remove("VYGES_GRT_STAGE_TIMES");
        let o = c.env("VYGES_STAGE_TIMES", "1").env("VYGES_LOG_FORMAT", "json").arg("route").arg(&path).output().unwrap();
        assert_eq!(o.status.code(), Some(0), "{}", String::from_utf8_lossy(&o.stderr));
        let par = std::fs::read_to_string(&pars).unwrap_or_default();
        (std::fs::read(&guides).unwrap(), String::from_utf8_lossy(&o.stderr).into_owned(), par)
    };
    let (g_with, e_with, par) = run("with", true);
    let (g_without, e_without, _) = run("without", false);
    assert_eq!(g_with, g_without);
    let stage = "\"estimateAllGlobalRouteParasitics (routed)\"";
    assert!(e_with.contains(stage), "{e_with}");
    assert!(!e_without.contains(stage), "{e_without}");
    assert!(par.lines().any(|l| l.contains("|res|") || l.contains("|node|")), "no networks written:\n{par}");
    assert!(vyges_grt::global_route::RouteOptions::new().keep_parasitics, "library callers keep them by default");
}
