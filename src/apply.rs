// SPDX-License-Identifier: Apache-2.0
//! Writing guides to the design database.
//!
//! The rules that decide *what* the guides are live in the crate root and need no database. This
//! module is the thin part: it writes what they decided, in the order they decided it.

use crate::NetGuides;
use vyges_opendb::Db;

/// Write every net's guides to the database.
///
/// Per net, in order: clear the net's existing guides, write the new ones in segment order, then
/// **put the set back into that order**.
///
/// ⛔ **The reversal is not optional and is the easiest thing here to leave out.** The database's
/// guide set prepends, so a freshly written net reads back newest-first. The published stage ends
/// by undoing exactly that, and guide files are compared as ordered lists — so an applier that
/// skips it writes the right guides and still fails every ordered comparison, on every net with
/// more than one segment.
///
/// ⚠️ **A net with no guides is skipped, not cleared.** Reaching here with an empty list means the
/// planner produced nothing for that net, which is not the same as asking for its guides to be
/// removed.
///
/// Returns the number of guides written.
pub fn apply_guides(db: &mut Db, nets: &[NetGuides]) -> Result<usize, Box<dyn std::error::Error>> {
    let mut written = 0;
    for net in nets {
        if net.guides.is_empty() {
            continue;
        }
        db.net_clear_guides(&net.net)?;
        for g in &net.guides {
            db.add_guide(
                &net.net,
                &layer_name(db, g.layer)?,
                &layer_name(db, g.via_layer)?,
                g.box_.x_min,
                g.box_.y_min,
                g.box_.x_max,
                g.box_.y_max,
                g.is_congested,
            )?;
            written += 1;
        }
        // ⛔ see the note above: without this the order is reversed.
        db.reverse_guides(&net.net)?;
        // The flags `saveGuides` sets on a guide (`setIsJumper`, `setIsConnectedToTerm`), by
        // position in the now-ordered list. The detailed router reads neither; the database keeps
        // them, and a later antenna check binds guides to pins through the second.
        for (k, g) in net.guides.iter().enumerate() {
            if g.is_jumper {
                db.guide_set_is_jumper(&net.net, k, true)?;
            }
            if g.is_connected_to_term {
                db.guide_set_is_connected_to_term(&net.net, k, true)?;
            }
        }
    }
    Ok(written)
}

/// Routing level -> the database's name for that layer.
///
/// ⚠️ The engine carries layers as routing LEVELS because that is what routing works in; the
/// database addresses them by name. The translation belongs here, at the boundary, rather than
/// leaking names into the geometry rules.
fn layer_name(db: &Db, level: i32) -> Result<String, Box<dyn std::error::Error>> {
    db.layers_with_direction()?
        .into_iter()
        .map(|(name, _dir)| name)
        .find(|n| db.layer_get_routing_level(n) == level)
        .ok_or_else(|| format!("no routing layer at level {level}").into())
}
