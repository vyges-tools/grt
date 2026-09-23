// SPDX-License-Identifier: Apache-2.0
//! One routing layer as CUGR models it (`MetalLayer`): tracks, the min-length and spacing rules its
//! capacity and via-demand models read, and the user's capacity adjustment.

use super::geo::Interval;

/// `MetalLayer::H` / `MetalLayer::V` — the index a point or box is read along.
pub const H: usize = 0;
pub const V: usize = 1;

/// What the layer is built from, as the database gives it.
#[derive(Debug, Clone, PartialEq)]
pub struct MetalLayerFacts {
    pub name: String,
    /// `getRoutingLevel()`; the layer's index is this minus one.
    pub routing_level: i32,
    pub horizontal: bool,
    pub width: i32,
    pub min_width: i32,
    pub spacing: i32,
    pub resistance: f64,
    /// The resistance of the cut layer just above (`getUpperLayer()`), 0 when there is none.
    pub via_resistance: f64,
    /// `dbTrackGrid::getAverageTrackSpacing`: `(pitch, first track, number of tracks)`.
    pub tracks: (i32, i32, i32),
    /// `getArea()` — the raw AREA field, 0 when unset (not the LEF58-aware minimum).
    pub area: i64,
    /// `getV55SpacingWidthsAndLengths`, `None` when it returns false.
    pub v55_widths_and_lengths: Option<(Vec<u32>, Vec<u32>)>,
    /// `getV55SpacingTable`, rows by width, `None` when it returns false.
    pub v55_table: Option<Vec<Vec<u32>>>,
    /// `getLayerAdjustment()`, read AFTER the global adjustment is written into it.
    pub adjustment: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MetalLayer {
    pub name: String,
    pub index: i32,
    /// [`H`] or [`V`].
    pub direction: usize,
    pub width: i32,
    pub min_width: i32,
    pub spacing: i32,
    pub resistance: f64,
    pub via_resistance: f64,
    pub pitch: i32,
    pub first_track_loc: i32,
    pub last_track_loc: i32,
    pub num_tracks: i32,
    /// `getArea()` as read (kept for reporting; only `min_length` is used).
    pub area: i64,
    pub min_length: i32,
    pub parallel_width: Vec<i32>,
    pub parallel_length: Vec<i32>,
    pub parallel_spacing: Vec<Vec<i32>>,
    pub default_spacing: i32,
    pub adjustment: f32,
}

impl MetalLayer {
    /// The `MetalLayer` constructor.
    pub fn new(f: &MetalLayerFacts) -> MetalLayer {
        let (pitch, first, num) = f.tracks;
        // Upstream rule: min length is `max(int(area / width) - width, 0)` — the int64 quotient
        // narrowed to int before the subtraction.
        let min_length = ((f.area / i64::from(f.width)) as i32 - f.width).max(0);
        let mut l = MetalLayer {
            name: f.name.clone(),
            index: f.routing_level - 1,
            direction: if f.horizontal { H } else { V },
            width: f.width,
            min_width: f.min_width,
            spacing: f.spacing,
            resistance: f.resistance,
            via_resistance: f.via_resistance,
            pitch,
            first_track_loc: first,
            last_track_loc: first + pitch * (num - 1),
            num_tracks: num,
            area: f.area,
            min_length,
            parallel_width: vec![0],
            parallel_length: vec![0],
            parallel_spacing: vec![vec![0]],
            default_spacing: 0,
            adjustment: f.adjustment,
        };
        l.load_parallel_run_spacing(f);
        l.default_spacing = l.parallel_spacing_for(l.width, 0);
        l
    }

    /// The V55 table as the constructor loads it, over the defaults `{0}`, `{0}`, `{{0}}`.
    ///
    /// Upstream rule: nothing is loaded unless the TABLE is non-empty; lengths and widths then
    /// replace their defaults only when present, and each spacing row is resized to
    /// `max(1, #lengths)` with zero fill before the table's entries are copied in.
    fn load_parallel_run_spacing(&mut self, f: &MetalLayerFacts) {
        let table = f.v55_table.clone().unwrap_or_default();
        if table.is_empty() {
            return;
        }
        let (widths, lengths) = f.v55_widths_and_lengths.clone().unwrap_or_default();
        let num_length = lengths.len();
        if num_length > 0 {
            self.parallel_length = lengths.iter().map(|&v| v as i32).collect();
        }
        let num_width = widths.len();
        if num_width > 0 {
            self.parallel_width = widths.iter().map(|&v| v as i32).collect();
            self.parallel_spacing.resize(num_width, Vec::new());
            for w in 0..num_width {
                self.parallel_spacing[w].resize(num_length.max(1), 0);
                for l in 0..num_length {
                    self.parallel_spacing[w][l] = table[w][l] as i32;
                }
            }
        }
    }

    /// `getParallelSpacing(width, length)`.
    ///
    /// Upstream rule: walk DOWN from the last row while the row's width is `>=` the query, so the
    /// row taken is the last whose width is strictly below it (row 0 at worst); a length of 0 reads
    /// column 0, otherwise the column is found the same way. ⚠️ Not odb's `findV55Spacing`.
    pub fn parallel_spacing_for(&self, width: i32, length: i32) -> i32 {
        let mut w = self.parallel_width.len() as i32 - 1;
        while w > 0 && self.parallel_width[w as usize] >= width {
            w -= 1;
        }
        if length == 0 {
            return self.parallel_spacing[w as usize][0];
        }
        let mut l = self.parallel_length.len() as i32 - 1;
        while l > 0 && self.parallel_length[l as usize] >= length {
            l -= 1;
        }
        self.parallel_spacing[w as usize][l as usize]
    }

    pub fn track_location(&self, track_index: i32) -> i32 {
        self.first_track_loc + track_index * self.pitch
    }

    /// `rangeSearchTracks(loc_range, include_bound)`: the track indices inside a DBU interval.
    ///
    /// Upstream rule: clamp to the first/last track, then `ceil` / `floor` of the offset over the
    /// pitch in DOUBLE; without bounds, a track exactly on either end is dropped.
    pub fn range_search_tracks(&self, loc: Interval, include_bound: bool) -> Interval {
        let lo = loc.low.max(self.first_track_loc);
        let hi = loc.high.min(self.last_track_loc);
        let pitch = f64::from(self.pitch);
        let mut r = Interval::new(
            (f64::from(lo - self.first_track_loc) / pitch).ceil() as i32,
            (f64::from(hi - self.first_track_loc) / pitch).floor() as i32,
        );
        if !r.is_valid() {
            return r;
        }
        if !include_bound {
            if self.track_location(r.low) == loc.low {
                r.low += 1;
            }
            if self.track_location(r.high) == loc.high {
                r.high -= 1;
            }
        }
        r
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn facts() -> MetalLayerFacts {
        MetalLayerFacts {
            name: "m2".into(),
            routing_level: 2,
            horizontal: false,
            width: 140,
            min_width: 140,
            spacing: 140,
            resistance: 0.0,
            via_resistance: 0.0,
            tracks: (380, 190, 527),
            area: 0,
            v55_widths_and_lengths: Some((vec![0, 180, 540], vec![0, 600, 1800])),
            v55_table: Some(vec![vec![140, 140, 140], vec![140, 180, 180], vec![140, 180, 540]]),
            adjustment: 0.0,
        }
    }

    // Upstream rule (Layers.cpp `getParallelSpacing`): walk down while width[w] >= query — the row
    // is the last with width STRICTLY below the query. A width equal to a row's bound steps past
    // it: a 180-wide shape reads row 0, a 181-wide one row 1.
    #[test]
    fn parallel_spacing_row_is_strictly_below_the_query() {
        let l = MetalLayer::new(&facts());
        assert_eq!(l.parallel_spacing_for(180, 1000), 140, "180 is not above row 1's 180");
        assert_eq!(l.parallel_spacing_for(181, 1000), 180, "181 reads row 1");
        assert_eq!(l.parallel_spacing_for(541, 1801), 540, "past both last bounds: last cell");
        assert_eq!(l.parallel_spacing_for(541, 1800), 180, "length 1800 is not above 1800");
        assert_eq!(l.default_spacing, 140, "default spacing is the layer width, length 0");
    }

    // Upstream rule (Layers.cpp ctor): with no V55 TABLE nothing loads — even when widths and
    // lengths exist — and the defaults {0}, {0}, {{0}} answer 0 everywhere.
    #[test]
    fn no_table_keeps_the_defaults() {
        let mut f = facts();
        f.v55_table = None;
        let l = MetalLayer::new(&f);
        assert_eq!((l.parallel_width.clone(), l.parallel_length.clone()), (vec![0], vec![0]));
        assert_eq!(l.parallel_spacing_for(1000, 1000), 0);
    }

    // Upstream rule (Layers.cpp ctor): `max(int(area / width) - width, 0)`.
    #[test]
    fn min_length_from_area() {
        let mut f = facts();
        f.area = 83_000; // 83000 / 140 = 592 -> 592 - 140
        assert_eq!(MetalLayer::new(&f).min_length, 452);
        f.area = 10_000; // 71 - 140 < 0
        assert_eq!(MetalLayer::new(&f).min_length, 0);
    }

    // Upstream rule (Layers.cpp `rangeSearchTracks`): ceil/floor of the offset over the pitch;
    // without bounds a track exactly on an end is dropped.
    #[test]
    fn range_search_tracks_bounds() {
        let l = MetalLayer::new(&facts());
        // tracks at 190, 570, 950, …
        assert_eq!(l.range_search_tracks(Interval::new(190, 950), true), Interval::new(0, 2));
        assert_eq!(l.range_search_tracks(Interval::new(190, 950), false), Interval::new(1, 1));
        assert_eq!(l.range_search_tracks(Interval::new(191, 949), true), Interval::new(1, 1));
        assert!(!l.range_search_tracks(Interval::new(200, 500), true).is_valid());
    }
}
