// SPDX-License-Identifier: Apache-2.0
//! Boost.Polygon's rectilinear polygon formation (`get_polygons`, holes fractured), against a probe
//! compiled with the reference's own Boost headers (`boost-polygon90-probe.cpp`): 408 cases of
//! rectangles added and subtracted — holes, nested holes, corner-touching pieces, a ring — each
//! polygon's points IN ORDER, then its area and perimeter (which counts a hole's slit twice).

use vyges_grt::polygon90::{get_polygons, polygon_area, polygon_perimeter, Region, R};

fn rects(s: &str) -> Vec<R> {
    s.split(';')
        .filter(|t| !t.is_empty())
        .map(|t| {
            let v: Vec<i32> = t.split(',').map(|n| n.parse().unwrap()).collect();
            (v[0], v[1], v[2], v[3])
        })
        .collect()
}

#[test]
fn formation_matches_boost() {
    assert_eq!(replay(include_str!("data/boost-polygon90-fracture.txt")), (408, 1025));
}

/// Every `CASE add|sub` of a probe corpus, compared polygon by polygon: `(cases, polygons)`.
fn replay(text: &str) -> (usize, usize) {
    let (mut cases, mut polys) = (0, 0);
    let mut lines = text.lines();
    while let Some(head) = lines.next() {
        let spec = head.strip_prefix("CASE ").expect("a case header");
        let (add, sub) = spec.split_once('|').unwrap();
        let mut want = Vec::new();
        for l in lines.by_ref() {
            if l == "END" {
                break;
            }
            let (pts, rest) = l.split_once('|').unwrap();
            let pts: Vec<(i32, i32)> = pts
                .split(';')
                .filter(|t| !t.is_empty())
                .map(|t| {
                    let (x, y) = t.split_once(',').unwrap();
                    (x.parse().unwrap(), y.parse().unwrap())
                })
                .collect();
            let f: Vec<i64> = rest.split('|').map(|kv| kv.split_once('=').unwrap().1.parse().unwrap()).collect();
            want.push((pts, f[0], f[1]));
        }
        let got = get_polygons(&Region::new(&rects(add), &rects(sub)));
        let got: Vec<(Vec<(i32, i32)>, i64, i64)> = got.iter().map(|p| (p.clone(), polygon_area(p), polygon_perimeter(p))).collect();
        assert_eq!(got, want, "case {spec}");
        cases += 1;
        polys += want.len();
    }
    (cases, polys)
}

/// `GRT_POLYGON90_STRESS=/path/to/stress.txt cargo test --release --test polygon90 -- --ignored`
///
/// The same comparison over a generated corpus in the committed file's format (dense small grids,
/// one outline with many holes, tie-heavy coordinates), each case answered by the probe. Too large
/// to commit; the cases that kill a mutation the committed corpus let through are copied into it.
#[test]
#[ignore = "needs a generated corpus; the committed one is what CI runs"]
fn formation_matches_boost_on_a_stress_corpus() {
    let path = std::env::var("GRT_POLYGON90_STRESS").expect("set GRT_POLYGON90_STRESS");
    let text = std::fs::read_to_string(&path).expect("a readable corpus");
    let (cases, polys) = replay(&text);
    eprintln!("stress: {cases} cases, {polys} polygons");
}
