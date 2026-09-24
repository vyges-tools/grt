// SPDX-License-Identifier: Apache-2.0
//! GlobalRouter's database callbacks (`GRouteDbCbk`) inside an incremental bracket
//! (`startIncremental` … `endIncremental`): which nets a netlist edit marks dirty.
//!
//! A terminal disconnected (`inDbITermPreDisconnect`, and each terminal `dbInst::destroy` and
//! `dbNet::destroy` disconnect) or connected (`inDbITermPostConnect`) calls `addDirtyNet` on its
//! net when the net is set and not special. `addDirtyNet` marks only a net the global router holds
//! (`db_net_map_`). A net created is added (`addNet`, if routable); a net destroyed is removed
//! (`removeNet`), which takes it out of the dirty set.

use std::collections::BTreeSet;

/// `dirty_nets_` over a bracket, with the callbacks' trace (`VYGC|cb|…`) as the reference prints it.
#[derive(Debug, Default)]
pub struct DirtyNets {
    dirty: BTreeSet<String>,
    /// The records since the last [`DirtyNets::take_trace`].
    trace: Vec<String>,
}

impl DirtyNets {
    /// A terminal of `net` (dis)connected: `addDirtyNet`, when the net is set and not special.
    ///
    /// Upstream rule: `addDirtyNet` marks only a net in `db_net_map_` — one the global router holds.
    pub fn mark(&mut self, net: &str, special: bool, held: bool) {
        if net.is_empty() || special {
            return;
        }
        self.trace.push(format!("VYGC|cb|dirty|{net}|held={}", i32::from(held)));
        if held {
            self.dirty.insert(net.to_string());
        }
    }

    /// `addNet` (`inDbNetCreate`): `made` when the net is routable and the router now holds it.
    pub fn added(&mut self, net: &str, made: bool) {
        self.trace.push(format!("VYGC|cb|add|{net}|made={}", i32::from(made)));
    }

    /// `removeNet` (`inDbNetDestroy`).
    ///
    /// Upstream rule: the destroyed net leaves the dirty set — `dirty_nets_` holds the dbNet itself,
    /// so a later net of the same name is a different net, dirty only if marked.
    pub fn removed(&mut self, net: &str) {
        self.trace.push(format!("VYGC|cb|remove|{net}"));
        self.dirty.remove(net);
    }

    /// `dirty_nets_` in its order (a PtrSet: the block's).
    pub fn in_block_order(&self, order: &[String]) -> Vec<String> {
        order.iter().filter(|n| self.dirty.contains(*n)).cloned().collect()
    }

    pub fn take_trace(&mut self) -> Vec<String> {
        std::mem::take(&mut self.trace)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Upstream rule (`addDirtyNet`): a net the global router does not hold is not marked, though
    // the callback fired. Every net the corpus marks is held.
    #[test]
    fn an_unheld_net_is_not_marked() {
        let mut d = DirtyNets::default();
        d.mark("a", false, false);
        d.mark("b", false, true);
        d.mark("s", true, true);
        d.mark("", false, true);
        let order = ["a", "b", "s"].map(String::from);
        assert_eq!(d.in_block_order(&order), vec!["b".to_string()]);
        assert_eq!(d.take_trace(), vec!["VYGC|cb|dirty|a|held=0".to_string(), "VYGC|cb|dirty|b|held=1".to_string()]);
    }

    // Upstream rule (`removeNet`): a destroyed net leaves the dirty set, so a net later created
    // under its name is not dirty unless marked. The corpus never re-creates one.
    #[test]
    fn a_destroyed_net_leaves_the_dirty_set() {
        let mut d = DirtyNets::default();
        d.mark("n", false, true);
        d.removed("n");
        d.added("n", true);
        assert!(d.in_block_order(&["n".to_string()]).is_empty());
    }
}
