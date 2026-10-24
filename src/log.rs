// SPDX-License-Identifier: Apache-2.0
//! The router's message lines. Each stage hands its messages to a [`Sink`] as [`Msg`] values; the
//! binary's sink is the suite's logger (`vyges_events::logger::Logger`), which writes each line in
//! the reference's format and keeps its print limits and counts. A `Vec<Msg>` is a sink too: it
//! holds a stage's messages until the caller puts them where the reference prints them, and it is
//! what a test reads. The library stays dependency-free.

/// A message's level, as the line names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Info,
    Warning,
    Error,
}

impl Level {
    pub fn name(self) -> &'static str {
        match self {
            Level::Info => "INFO",
            Level::Warning => "WARNING",
            Level::Error => "ERROR",
        }
    }
}

/// One message: a coded line, or report text with no prefix.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Msg {
    Coded { level: Level, tool: &'static str, id: u16, text: String },
    Report(String),
}

impl Msg {
    /// The line as the reference's logger writes it: `[INFO GRT-0001] text`, or the report text.
    pub fn line(&self) -> String {
        match self {
            Msg::Coded { level, tool, id, text } => format!("[{} {tool}-{id:04}] {text}", level.name()),
            Msg::Report(text) => text.clone(),
        }
    }
}

/// Where a stage's messages go.
pub trait Sink {
    fn put(&mut self, m: Msg);
}

impl Sink for Vec<Msg> {
    fn put(&mut self, m: Msg) {
        self.push(m);
    }
}

impl dyn Sink + '_ {
    pub fn info(&mut self, tool: &'static str, id: u16, text: impl Into<String>) {
        self.put(Msg::Coded { level: Level::Info, tool, id, text: text.into() });
    }

    pub fn warn(&mut self, tool: &'static str, id: u16, text: impl Into<String>) {
        self.put(Msg::Coded { level: Level::Warning, tool, id, text: text.into() });
    }

    /// An error line. The stage that logs it decides whether the run stops.
    pub fn error(&mut self, tool: &'static str, id: u16, text: impl Into<String>) {
        self.put(Msg::Coded { level: Level::Error, tool, id, text: text.into() });
    }

    pub fn report(&mut self, text: impl Into<String>) {
        self.put(Msg::Report(text.into()));
    }

    /// Put messages a stage held, in order.
    pub fn replay(&mut self, msgs: impl IntoIterator<Item = Msg>) {
        for m in msgs {
            self.put(m);
        }
    }
}

/// The lines of held messages, as they print.
pub fn lines(msgs: &[Msg]) -> Vec<String> {
    msgs.iter().map(Msg::line).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_coded_line_carries_level_tool_and_four_digit_id_and_a_report_is_bare() {
        let mut held: Vec<Msg> = Vec::new();
        let s: &mut dyn Sink = &mut held;
        s.info("GRT", 1, "Minimum degree: 2");
        s.warn("ODB", 1000, "Layer x not found");
        s.error("GRT", 116, "congested");
        s.report("");
        assert_eq!(lines(&held), ["[INFO GRT-0001] Minimum degree: 2", "[WARNING ODB-1000] Layer x not found", "[ERROR GRT-0116] congested", ""]);
    }
}
