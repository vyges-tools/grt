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
    let text = include_str!("data/boost-polygon90-fracture.txt");
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
    assert_eq!((cases, polys), (408, 1025));
}
