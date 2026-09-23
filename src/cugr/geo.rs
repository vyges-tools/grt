// SPDX-License-Identifier: Apache-2.0
//! CUGR's geometry primitives: a point, a closed integer interval, a box of two intervals.
//!
//! Their edge semantics decide results, so they are the reference's and not the obvious ones:
//!
//! - An interval is born INVALID, `(i32::MAX, i32::MIN)`, and `update` grows it from there — a
//!   bounding box starts empty rather than at the origin.
//! - `range()` is 0 for an invalid interval, not negative.
//! - `center()` truncates toward zero, as C++ integer division does (Rust's `/` agrees).
//! - Two invalid intervals compare EQUAL whatever their bounds.
//! - `intersect_with` does not normalise: an empty intersection is an invalid interval.

/// A gcell or DBU point. The default point is `(i32::MAX, i32::MAX)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Default for Point {
    fn default() -> Self {
        Point { x: i32::MAX, y: i32::MAX }
    }
}

impl Point {
    pub const fn new(x: i32, y: i32) -> Self {
        Point { x, y }
    }
    /// `operator[]`: dimension 0 is x, 1 is y.
    pub fn get(self, dimension: usize) -> i32 {
        if dimension == 0 {
            self.x
        } else {
            self.y
        }
    }
    pub fn set(&mut self, dimension: usize, value: i32) {
        if dimension == 0 {
            self.x = value;
        } else {
            self.y = value;
        }
    }
}

/// A closed interval `[low, high]`. The default interval is invalid: `(i32::MAX, i32::MIN)`.
#[derive(Debug, Clone, Copy)]
pub struct Interval {
    pub low: i32,
    pub high: i32,
}

impl Default for Interval {
    fn default() -> Self {
        Interval { low: i32::MAX, high: i32::MIN }
    }
}

impl PartialEq for Interval {
    /// Two invalid intervals are equal whatever their bounds.
    fn eq(&self, rhs: &Self) -> bool {
        (!self.is_valid() && !rhs.is_valid()) || (self.low == rhs.low && self.high == rhs.high)
    }
}

impl Interval {
    pub const fn new(low: i32, high: i32) -> Self {
        Interval { low, high }
    }
    /// `IntervalT(int val)`: the one-point interval.
    pub const fn point(value: i32) -> Self {
        Interval { low: value, high: value }
    }
    pub fn is_valid(&self) -> bool {
        self.low <= self.high
    }
    /// `high - low`, and 0 when invalid.
    pub fn range(&self) -> i32 {
        if self.is_valid() {
            self.high - self.low
        } else {
            0
        }
    }
    /// `(high + low) / 2`, truncating toward zero. Wrapping, as the reference's `int` sum is only
    /// ever taken of a valid interval in practice.
    pub fn center(&self) -> i32 {
        self.high.wrapping_add(self.low) / 2
    }
    pub fn update(&mut self, value: i32) {
        self.low = self.low.min(value);
        self.high = self.high.max(value);
    }
    /// An invalid side gives the other side back unchanged.
    pub fn union_with(&self, rhs: &Interval) -> Interval {
        if !self.is_valid() {
            return *rhs;
        }
        if !rhs.is_valid() {
            return *self;
        }
        Interval::new(self.low.min(rhs.low), self.high.max(rhs.high))
    }
    /// Not normalised: disjoint intervals give an invalid one.
    pub fn intersect_with(&self, rhs: &Interval) -> Interval {
        Interval::new(self.low.max(rhs.low), self.high.min(rhs.high))
    }
    pub fn contains(&self, value: i32) -> bool {
        value >= self.low && value <= self.high
    }
}

/// A box: an x interval and a y interval. The default box is invalid in both.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BoxT {
    pub x: Interval,
    pub y: Interval,
}

impl BoxT {
    pub const fn new(lx: i32, ly: i32, hx: i32, hy: i32) -> Self {
        BoxT { x: Interval::new(lx, hx), y: Interval::new(ly, hy) }
    }
    pub const fn from_intervals(x: Interval, y: Interval) -> Self {
        BoxT { x, y }
    }
    /// `operator[]`: dimension 0 is the x interval, 1 the y interval.
    pub fn get(&self, dimension: usize) -> Interval {
        if dimension == 0 {
            self.x
        } else {
            self.y
        }
    }
    pub fn lx(&self) -> i32 {
        self.x.low
    }
    pub fn ly(&self) -> i32 {
        self.y.low
    }
    pub fn hx(&self) -> i32 {
        self.x.high
    }
    pub fn hy(&self) -> i32 {
        self.y.high
    }
    pub fn cx(&self) -> i32 {
        self.x.center()
    }
    pub fn cy(&self) -> i32 {
        self.y.center()
    }
    pub fn is_valid(&self) -> bool {
        self.x.is_valid() && self.y.is_valid()
    }
    /// Half perimeter: `width + height`, each 0 when invalid.
    pub fn hp(&self) -> i32 {
        self.x.range() + self.y.range()
    }
    pub fn update(&mut self, p: Point) {
        self.x.update(p.x);
        self.y.update(p.y);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Upstream rule (geo.h `IntervalT()`): an interval is born invalid, (INT_MAX, INT_MIN), and a
    // bounding box grows from EMPTY — the first update makes it the point itself.
    #[test]
    fn a_bounding_box_grows_from_empty() {
        let mut b = BoxT::default();
        assert!(!b.is_valid());
        assert_eq!(b.hp(), 0, "an invalid box has half perimeter 0, not a wrapped negative");
        b.update(Point::new(3, 7));
        assert_eq!((b.lx(), b.ly(), b.hx(), b.hy()), (3, 7, 3, 7));
        b.update(Point::new(1, 9));
        assert_eq!(b.hp(), 2 + 2);
    }

    // Upstream rule (geo.h `IntervalT::center`): `(high + low) / 2` in int — truncation toward
    // zero, so a negative odd sum rounds UP, not down.
    #[test]
    fn center_truncates_toward_zero() {
        assert_eq!(Interval::new(-3, 0).center(), -1);
        assert_eq!(Interval::new(1, 4).center(), 2);
    }

    // Upstream rule (geo.h `IntervalT::operator==`): two invalid intervals are equal whatever
    // their bounds; `unionWith` returns the other side when one is invalid.
    #[test]
    fn invalid_intervals_are_equal_and_absorbed_by_union() {
        assert_eq!(Interval::new(5, 2), Interval::default());
        assert_ne!(Interval::new(2, 5), Interval::new(2, 6));
        assert_eq!(Interval::default().union_with(&Interval::point(4)), Interval::point(4));
        assert!(!Interval::new(0, 1).intersect_with(&Interval::new(3, 4)).is_valid());
    }
}
