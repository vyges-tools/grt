// SPDX-License-Identifier: Apache-2.0
//! grt's debug flags (the engines' debug-flag standard): its debug groups, the old variables
//! mapped onto them, and the stage guard the sequencers hold.
//!
//! - `VYGES_DEBUG=grt:<group>=<level>` turns a group on (see [`DEBUG_GROUPS`]).
//! - `VYGES_STAGE_TIMES=1` prints one `VYGES-STAGE` line per stage (wall, CPU, RSS, process
//!   peak); the stages are named after the reference's functions.
//! - A flag only changes what is printed, never a result.

use vyges_events::debug::{debug_enabled, DebugGroup};
use vyges_events::Severity;

/// The tool name every grt stage and debug line carries.
pub const TOOL: &str = "grt";

/// Each slack read's timer, per scene: graph sizes and path storage (count, capacity, bytes).
pub const TIMER_STATS: &str = "vy_timer_stats";
/// Each dirty net's pins as the incremental router compares them, one line per net.
pub const INCR_PINS: &str = "vy_incr_pins";
/// Turns `VYGES_STAGE_TIMES` on; the target of the old `VYGES_GRT_STAGE_TIMES`.
pub const STAGE_TIMES: &str = "vy_stage_times";
/// CUGR's model, each stage and each restore, as the CUGR instruments compare them; the target of
/// the old `VYGC_OUT` (which still names the file).
pub const CUGR_TRACE: &str = "vy_cugr_trace";

/// grt's debug groups. None mirrors a reference group: the reference's GRT `incr` (level 2) lists
/// the dirty nets' names only; ours prints their pins as compared.
pub const DEBUG_GROUPS: &[DebugGroup] = &[
    DebugGroup { group: TIMER_STATS, max_level: 1, what: "per slack read and scene: graph sizes, path count, capacity and bytes", reference: None },
    DebugGroup { group: INCR_PINS, max_level: 1, what: "incremental routing: each dirty net's pins as compared (file: VYGES_GRT_INCR_TRACE, else VYGES_DEBUG_DIR/grt/incr-pins.txt)", reference: None },
    DebugGroup { group: STAGE_TIMES, max_level: 1, what: "turns VYGES_STAGE_TIMES on (alias target of VYGES_GRT_STAGE_TIMES)", reference: None },
    DebugGroup { group: CUGR_TRACE, max_level: 1, what: "CUGR: its model, every stage and each restore, appended (file: VYGC_OUT, else VYGES_DEBUG_DIR/grt/cugr.vygc)", reference: None },
];

/// The old variables, mapped onto groups. Call once at the start of `main`, before any `Logger`
/// is built and before the first stage; also applies the stage-times group to `VYGES_STAGE_TIMES`.
pub fn register_aliases() {
    vyges_events::debug::register_alias("VYGES_GRT_STAGE_TIMES", TOOL, STAGE_TIMES, 1);
    vyges_events::debug::register_alias("VYGES_GRT_INCR_TRACE", TOOL, INCR_PINS, 1);
    vyges_events::debug::register_alias("VYGES_GRT_TIMER_STATS", TOOL, TIMER_STATS, 1);
    vyges_events::debug::register_alias("VYGC_OUT", TOOL, CUGR_TRACE, 1);
    if debug_enabled(TOOL, STAGE_TIMES, 1) && std::env::var_os("VYGES_STAGE_TIMES").is_none() {
        // Single-threaded here (start of main), and before the stage switch is first read.
        std::env::set_var("VYGES_STAGE_TIMES", "1");
    }
    vyges_events::debug::warn_unknown_debug_groups(TOOL, DEBUG_GROUPS);
}

/// A stage named after the reference function it reproduces; reports when dropped.
pub fn stage(name: &str) -> vyges_events::stage::Stage {
    vyges_events::stage::stage_in(TOOL, name)
}

/// Whether a grt group is on. Check once, outside a hot loop.
pub fn on(group: &str) -> bool {
    debug_enabled(TOOL, group, 1)
}

/// Where the CUGR trace goes: the old variable's path (`VYGC_OUT`, as before), else with
/// `vy_cugr_trace` on the debug directory's `cugr.vygc`; `None` when neither. Read it once per write.
pub fn cugr_out() -> Option<String> {
    cugr_out_from(std::env::var("VYGC_OUT").ok(), on(CUGR_TRACE), || vyges_events::debug_dir(TOOL))
}

fn cugr_out_from(old: Option<String>, on: bool, dir: impl FnOnce() -> Option<std::path::PathBuf>) -> Option<String> {
    old.or_else(|| if on { dir().map(|d| d.join("cugr.vygc").to_string_lossy().into_owned()) } else { None })
}

/// One debug line of a group that is on (the caller checked [`on`]).
pub fn line(group: &str, text: String) {
    vyges_events::log(TOOL, Severity::Info, format!("[{group}] {text}"));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_groups_are_ours_and_named_by_the_standard() {
        assert!(DEBUG_GROUPS.iter().all(|g| g.group.starts_with("vy_") && g.reference.is_none()));
        let spec = vyges_events::debug::parse_debug_spec("grt:vy_timer_stats=1,grt:incr=2");
        let w = vyges_events::debug::check_debug_groups(TOOL, DEBUG_GROUPS, &spec.specs);
        assert_eq!(w.len(), 1, "{w:?}");
        assert!(w[0].contains("grt:incr") && w[0].contains(TIMER_STATS), "{}", w[0]);
    }

    /// `VYGC_OUT` keeps naming the file; the group alone writes into the debug directory; off and
    /// unset, nothing is written (and the directory is not asked for).
    #[test]
    fn the_cugr_trace_goes_to_the_old_path_else_the_debug_directory() {
        let dir = || Some(std::path::PathBuf::from("/d/grt"));
        assert_eq!(cugr_out_from(Some("/x.vygc".into()), true, dir), Some("/x.vygc".into()));
        assert_eq!(cugr_out_from(Some("/x.vygc".into()), false, dir), Some("/x.vygc".into()));
        assert_eq!(cugr_out_from(None, true, dir), Some("/d/grt/cugr.vygc".into()));
        assert_eq!(cugr_out_from(None, true, || None), None);
        assert_eq!(cugr_out_from(None, false, || panic!("not asked when off")), None);
        let (spec, _) = vyges_events::debug::alias_spec("VYGC_OUT", Some("/x.vygc"), TOOL, CUGR_TRACE, 1).expect("a path turns it on");
        assert_eq!(spec.group, CUGR_TRACE);
    }

    #[test]
    fn an_old_variable_maps_onto_its_group() {
        let (spec, note) = vyges_events::debug::alias_spec("VYGES_GRT_TIMER_STATS", Some("1"), TOOL, TIMER_STATS, 1).expect("set");
        assert_eq!((spec.tool.as_str(), spec.group.as_str(), spec.level), (TOOL, TIMER_STATS, 1));
        assert!(note.contains("VYGES_DEBUG=grt:vy_timer_stats=1"), "{note}");
        assert!(vyges_events::debug::alias_spec("VYGES_GRT_TIMER_STATS", None, TOOL, TIMER_STATS, 1).is_none());
        assert!(vyges_events::debug::alias_spec("VYGES_GRT_TIMER_STATS", Some("0"), TOOL, TIMER_STATS, 1).is_none());
    }
}
