// SPDX-License-Identifier: Apache-2.0
//! `initCUGR`'s call sequence, apart from the database so its order is tested in every build
//! (the database side, `route::InitCugrDb`, needs the `odb` feature).

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// `initCUGR`'s stages, one method each, named after the reference's functions. Behind a trait so
/// their ORDER — which is the rule: every report lands in the log where the stage runs — is tested
/// without a database ([`init_cugr_sequence`]).
pub trait InitCugrStages {
    /// `initRoutingGrid`: routing layers, `reportLayerSettings` (GRT-0020..0023), tracks (GRT-0088),
    /// core grid.
    fn init_routing_grid(&mut self) -> Res<()>;
    /// `computeUserGlobalAdjustments`: the global adjustment written INTO each tech layer in range
    /// that has none of its own; `MetalLayer` reads it back from there, and it persists.
    fn compute_user_global_adjustments(&mut self) -> Res<()>;
    /// `reportMacrosAndBlockages` (GRT-0003/0004, verbose), counted from the database.
    fn report_macros_and_blockages(&mut self) -> Res<()>;
    /// `initNets(true)` → `findNets(true)`: `initClockNets` (the timer's clock nets re-typed CLOCK;
    /// GRT-0019), then net discovery (GRT-0280/0281).
    fn init_nets(&mut self) -> Res<()>;
    /// `findClockNets`: the admitted nets that are non-leaf clocks.
    fn find_clock_nets(&mut self);
    /// `reportNetDegree` (GRT-0001/0002, verbose) over the admitted nets.
    fn report_net_degree(&mut self);
    /// The inline block after it: no liberty library and a critical-nets percentage → GRT-0309,
    /// and the percentage is set to 0.
    fn check_timing_available(&mut self);
    /// `cugr_->init`.
    fn cugr_init(&mut self) -> Res<()>;
}

/// `initCUGR` (4155ac3): the reference's call sequence and nothing else.
///
/// Not modelled: `initNets`' `checkPinPlacement` (its warnings: a pin off its layer's tracks),
/// `setCongestionIterations` / `setVerbose` (options `CUGR::init` is given directly).
pub fn init_cugr_sequence<S: InitCugrStages + ?Sized>(s: &mut S) -> Res<()> {
    s.init_routing_grid()?;
    s.compute_user_global_adjustments()?;
    s.report_macros_and_blockages()?;
    s.init_nets()?;
    s.find_clock_nets();
    s.report_net_degree();
    s.check_timing_available();
    s.cugr_init()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Records each `initCUGR` stage as it is called; `fail_at` makes that stage return an error.
    struct StageSpy {
        calls: Vec<&'static str>,
        fail_at: Option<&'static str>,
    }

    impl StageSpy {
        fn call(&mut self, name: &'static str) -> Res<()> {
            self.calls.push(name);
            if self.fail_at == Some(name) {
                return Err(name.into());
            }
            Ok(())
        }
    }

    impl InitCugrStages for StageSpy {
        fn init_routing_grid(&mut self) -> Res<()> {
            self.call("initRoutingGrid")
        }
        fn compute_user_global_adjustments(&mut self) -> Res<()> {
            self.call("computeUserGlobalAdjustments")
        }
        fn report_macros_and_blockages(&mut self) -> Res<()> {
            self.call("reportMacrosAndBlockages")
        }
        fn init_nets(&mut self) -> Res<()> {
            self.call("initNets")
        }
        fn find_clock_nets(&mut self) {
            self.calls.push("findClockNets");
        }
        fn report_net_degree(&mut self) {
            self.calls.push("reportNetDegree");
        }
        fn check_timing_available(&mut self) {
            self.calls.push("GRT-0309 check");
        }
        fn cugr_init(&mut self) -> Res<()> {
            self.call("cugr_->init")
        }
    }

    /// `initCUGR` at 4155ac3 calls, in this order: initRoutingGrid, computeUserGlobalAdjustments,
    /// reportMacrosAndBlockages, initNets(true), findClockNets, reportNetDegree, the GRT-0309
    /// timing check, `cugr_->init`. The order IS the rule: each stage's report lands in the log
    /// where the stage runs, so GRT-0020..0023/0088 precede GRT-0003/0004, which precede GRT-0019
    /// and GRT-0280/0281, then GRT-0001/0002, then GRT-0309. A stage that fails stops the rest.
    #[test]
    fn init_cugr_calls_its_stages_in_the_reference_order() {
        let mut s = StageSpy { calls: Vec::new(), fail_at: None };
        init_cugr_sequence(&mut s).expect("no stage fails");
        assert_eq!(
            s.calls,
            ["initRoutingGrid", "computeUserGlobalAdjustments", "reportMacrosAndBlockages", "initNets", "findClockNets", "reportNetDegree", "GRT-0309 check", "cugr_->init"]
        );
        let mut s = StageSpy { calls: Vec::new(), fail_at: Some("reportMacrosAndBlockages") };
        assert!(init_cugr_sequence(&mut s).is_err());
        assert_eq!(s.calls, ["initRoutingGrid", "computeUserGlobalAdjustments", "reportMacrosAndBlockages"]);
    }
}
