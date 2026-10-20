// SPDX-License-Identifier: Apache-2.0
//! The timer's netlist as the database holds it ([`vyges_grt::timer::netlist`]).
#![cfg(feature = "odb")]

use vyges_opendb::Db;
use vyges_sta::netlist::PortDir;

fn data(f: &str) -> String {
    format!("{}/tests/data/{f}", env!("CARGO_MANIFEST_DIR"))
}

// Rule (the reference's `dbNetwork::dbToSta(sig_type, io_type)`): a block terminal typed POWER
// or GROUND has direction power / ground, whatever its io type, and the timing graph makes no
// vertex for it (`isPowerGround`); every other terminal keeps its io type, INOUT as bidirect.
// A power grid's pins are exactly such terminals: INOUT, USE POWER / GROUND.
#[test]
fn supply_terminals_are_not_timing_ports() {
    let def = std::fs::read_to_string(data("tiny.def")).expect("tiny.def");
    let def = def
        .replace(
            "PINS 1 ;",
            "PINS 4 ;\n\
             - VPWR + NET VPWR + SPECIAL + DIRECTION INOUT + USE POWER\n  + LAYER M2 ( -35 0 ) ( 35 140 )\n  + PLACED ( 300 2200 ) N ;\n\
             - VGND + NET VGND + SPECIAL + DIRECTION INOUT + USE GROUND\n  + LAYER M2 ( -35 0 ) ( 35 140 )\n  + PLACED ( 500 2200 ) N ;\n\
             - bio + NET bio + DIRECTION INOUT + USE SIGNAL\n  + LAYER M2 ( -35 0 ) ( 35 140 )\n  + PLACED ( 1900 2200 ) N ;",
        )
        .replace(
            "NETS 2 ;",
            "SPECIALNETS 2 ;\n- VPWR ( PIN VPWR ) + USE POWER ;\n- VGND ( PIN VGND ) + USE GROUND ;\nEND SPECIALNETS\nNETS 3 ;\n- bio ( PIN bio ) ;",
        );
    let dir = std::env::temp_dir().join(format!("grt-timer-netlist-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let path = dir.join("supply.def");
    std::fs::write(&path, def).expect("write def");
    let mut db = Db::new();
    db.read_lef(data("tiny.lef")).expect("tiny.lef");
    db.read_def(path.to_str().expect("utf-8 path"), "default").expect("supply.def");
    let _ = std::fs::remove_dir_all(&dir);
    assert_eq!(db.bterm_names().len(), 4, "the supply terminals are in the database");

    let nl = vyges_grt::timer::netlist(&db);
    let ports: Vec<(&str, PortDir)> = nl.ports.iter().map(|(n, d)| (n.as_str(), *d)).collect();
    assert_eq!(ports, [("bio", PortDir::Inout), ("in", PortDir::Input)]);
}
