// SPDX-License-Identifier: Apache-2.0
//! The `vyges-grt` command's contract, on a tiny design (`tests/data/tiny.{lef,def}`, shared with
//! vyges-drt: two buffers, a port, two nets): exit status, the status word, the files named, and
//! the descriptor.
//!
//! ⚠️ The routed output below is a REGRESSION pin (this engine's own answer on this design), not a
//! correlation claim; correlation against the reference router runs outside this repository.
#![cfg(feature = "cli")]

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
/// on the pass word, and the artifact field is one the report actually carries.
#[test]
fn the_assertion_passes_only_on_routed() {
    let d = descriptor();
    assert_eq!(d["assertion"]["field"], "status");
    assert_eq!(d["assertion"]["pass_when"]["eq"], "routed");
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
