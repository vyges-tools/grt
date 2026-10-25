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
// Rule (`isPGSupply(dbBTerm*)`): a terminal on a SPECIAL POWER / GROUND net is skipped by the top
// instance's pin iterator whatever its own type, so it is no port (a USE SIGNAL `vdd` pin on the
// special `vdd` net, as a backside-power design writes it). A special SIGNAL net is a net like any
// (a pad ring writes the pad-to-terminal connection so): its pins connect.
#[test]
fn supply_terminals_are_not_timing_ports() {
    let def = std::fs::read_to_string(data("tiny.def")).expect("tiny.def");
    let def = def
        .replace(
            "PINS 1 ;",
            "PINS 6 ;\n\
             - sio + NET sio + SPECIAL + DIRECTION INPUT + USE SIGNAL\n  + LAYER M2 ( -35 0 ) ( 35 140 )\n  + PLACED ( 900 2200 ) N ;\n\
             - vdd + NET vdd + SPECIAL + DIRECTION INPUT + USE SIGNAL\n  + LAYER M2 ( -35 0 ) ( 35 140 )\n  + PLACED ( 700 2200 ) N ;\n\
             - VPWR + NET VPWR + SPECIAL + DIRECTION INOUT + USE POWER\n  + LAYER M2 ( -35 0 ) ( 35 140 )\n  + PLACED ( 300 2200 ) N ;\n\
             - VGND + NET VGND + SPECIAL + DIRECTION INOUT + USE GROUND\n  + LAYER M2 ( -35 0 ) ( 35 140 )\n  + PLACED ( 500 2200 ) N ;\n\
             - bio + NET bio + DIRECTION INOUT + USE SIGNAL\n  + LAYER M2 ( -35 0 ) ( 35 140 )\n  + PLACED ( 1900 2200 ) N ;",
        )
        .replace(
            "NETS 2 ;",
            "SPECIALNETS 4 ;\n- sio ( PIN sio ) + USE SIGNAL ;\n- vdd ( PIN vdd ) + USE POWER ;\n- VPWR ( PIN VPWR ) + USE POWER ;\n- VGND ( PIN VGND ) + USE GROUND ;\nEND SPECIALNETS\nNETS 3 ;\n- bio ( PIN bio ) ;",
        );
    let dir = std::env::temp_dir().join(format!("grt-timer-netlist-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let path = dir.join("supply.def");
    std::fs::write(&path, def).expect("write def");
    let mut db = Db::new();
    db.read_lef(data("tiny.lef")).expect("tiny.lef");
    db.read_def(path.to_str().expect("utf-8 path"), "default").expect("supply.def");
    let _ = std::fs::remove_dir_all(&dir);
    assert_eq!(db.bterm_names().len(), 6, "the supply terminals are in the database");

    assert_eq!(db.bterm_get_sig_type("vdd"), "SIGNAL");
    let nl = vyges_grt::timer::netlist(&db);
    let ports: Vec<(&str, PortDir)> = nl.ports.iter().map(|(n, d)| (n.as_str(), *d)).collect();
    assert_eq!(ports, [("sio", PortDir::Input), ("bio", PortDir::Inout), ("in", PortDir::Input)]);
    assert!(nl.nets.iter().any(|n| n.name == "sio"), "a special signal net is a net");
    assert!(!nl.nets.iter().any(|n| ["vdd", "VPWR", "VGND"].contains(&n.name.as_str())), "a special supply net is not");
}
