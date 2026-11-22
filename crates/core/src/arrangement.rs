//! The 2D planar arrangement that draw-on-face, the conversion weld and the mesh booleans share.
//!
//! Segments on one plane are snapped, split at their crossings, traced into cells, and each cell kept
//! or dropped by the operation asked for. `resolve` is the whole interface: one pipeline, three modes,
//! so the snap and the trace are written and tested once. Callers pass coordinates in a plane's own axes:
//! staying near the local origin is what keeps the weld and the area sums fine at millimetre scale.

use std::collections::{BTreeMap, BTreeSet};

use crate::geom::P2;
use crate::model::MAX_COORD;

/// `EPS_PLANE` of the mesh ladder, in mm: two points nearer than this are one point, and a point
/// nearer than this to a segment lies on it.
pub const EPS: f64 = 1e-2;

/// The most points one arrangement may make; a face beyond this is refused rather than run long.
pub const MAX_POINTS: usize = 20_000;

/// A straight segment on the plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Seg {
    pub a: P2,
    pub b: P2,
}

impl Seg {
    pub fn new(a: P2, b: P2) -> Seg {
        Seg { a, b }
    }

    fn dir(self) -> P2 {
        self.b - self.a
    }

    fn length(self) -> f64 {
        self.dir().length()
    }
}

/// What of the cells to keep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Cells inside `a`; `b` only cuts. Draw-on-face: each kept cell is a face.
    Split,
    /// Cells inside `a` and outside `b`. The weld's subtraction on one plane.
    Difference,
    /// Cells inside either. A boolean's union on one plane.
    Union,
}

/// A region: an outer loop counter-clockwise, its holes clockwise.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    pub outer: Vec<P2>,
    pub holes: Vec<Vec<P2>>,
}

/// Solve `a` and `b` on one plane into regions, keeping what `mode` says.
///
/// The segments are snapped together at [`EPS`], split at every crossing and T-junction, and the
/// cells they make are kept by a test against the closed loops `a` and `b`. A dangling segment that
/// ends inside a cell is pruned, so it cuts nothing and adds no region; the caller that must refuse
/// it (M4's draw-on-face) checks the input's ends itself.
pub fn resolve(a: &[Seg], b: &[Seg], mode: Mode) -> Result<Vec<Region>, String> {
    let in_range = |p: P2| p.is_finite() && p.0.abs() <= MAX_COORD && p.1.abs() <= MAX_COORD;
    if a.iter().chain(b).any(|s| !in_range(s.a) || !in_range(s.b)) {
        return Err(format!("a segment of the arrangement must be finite within {MAX_COORD} mm"));
    }
    let (pts, edges) = graph(a, b)?;
    if edges.is_empty() {
        return Ok(Vec::new());
    }
    let (traced, left) = trace(&pts, &edges)?;
    let kept: Vec<bool> = traced.iter().map(|t| t.area > 0.0 && keep(&t.verts, &pts, a, b, mode)).collect();
    // Where each kept cell's region went, so a hole can be given to it.
    let mut at: BTreeMap<usize, usize> = BTreeMap::new();
    let mut regions = match mode {
        // Each kept cell is a region; a negative cycle held by it is a hole.
        Mode::Split => {
            let mut out = Vec::new();
            for (i, t) in traced.iter().enumerate() {
                if !kept.get(i).copied().unwrap_or(false) {
                    continue;
                }
                at.insert(i, out.len());
                out.push(Region { outer: t.verts.iter().map(|&k| pts[k]).collect(), holes: Vec::new() });
            }
            out
        }
        // The kept cells merge: an edge kept on one side only is their boundary.
        Mode::Difference | Mode::Union => boundary(&edges, &left, &kept, &pts)?,
    };
    if mode == Mode::Split {
        for (i, t) in traced.iter().enumerate() {
            if t.area >= 0.0 || kept.get(i).copied().unwrap_or(false) {
                continue;
            }
            let Some(holder) = held_by(&traced, &kept, i, &pts).and_then(|h| at.get(&h).copied()) else {
                continue;
            };
            if let Some(region) = regions.get_mut(holder) {
                region.holes.push(t.verts.iter().map(|&k| pts[k]).collect());
            }
        }
    }
    Ok(regions)
}

/// Whether `mode` keeps a cell whose interior point is `p`.
fn keep(cell: &[usize], pts: &[P2], a: &[Seg], b: &[Seg], mode: Mode) -> bool {
    let Some(p) = cell_point(cell, pts) else { return false };
    let (in_a, in_b) = (inside(p, a), inside(p, b));
    match mode {
        Mode::Split => in_a,
        Mode::Difference => in_a && !in_b,
        Mode::Union => in_a || in_b,
    }
}

/// A point inside the cell: the midpoint of its longest edge, just to the left, where a
/// counter-clockwise boundary keeps the cell. A hole cannot capture it the way it captures a vertex
/// average, and an L-shaped cell's average lies outside the cell. Unlike an ear's centroid it needs no
/// convex corner; it assumes a cell wider than the 4 EPS offset, which is below the mesh's minimum edge.
fn cell_point(cycle: &[usize], pts: &[P2]) -> Option<P2> {
    let n = cycle.len();
    if n < 3 {
        return None;
    }
    let mut best: Option<(f64, P2, P2)> = None;
    for i in 0..n {
        let (a, b) = (pts[cycle[i]], pts[cycle[(i + 1) % n]]);
        let d = b.dist(a);
        if best.is_none_or(|(l, _, _)| d > l) {
            best = Some((d, a, b));
        }
    }
    let (_, a, b) = best?;
    Some(left_of(a, b))
}

// --- the pipeline -------------------------------------------------------------------------------

/// The welded points and the undirected edges between them, keyed by point index.
type Graph = (Vec<P2>, BTreeSet<(usize, usize)>);

/// The snapped points and the undirected edges between them, welded and pruned.
fn graph(a: &[Seg], b: &[Seg]) -> Result<Graph, String> {
    let input: Vec<Seg> = a.iter().chain(b).copied().filter(|s| s.length() >= EPS).collect();
    // Each segment adds at most two points. Refuse before `point`'s linear weld makes the build
    // quadratic in the number of segments a hostile caller can send.
    if input.len() > MAX_POINTS / 2 {
        return Err(format!("an arrangement of more than {MAX_POINTS} points is refused"));
    }
    let mut pts: Vec<P2> = Vec::new();
    // A set, not a Vec with `contains`: the cap is checked after this pass, so a hostile input of many
    // segments must not pay a quadratic scan before it is refused.
    let mut ends: BTreeSet<(usize, usize)> = BTreeSet::new();
    for s in &input {
        let (ia, ib) = (point(&mut pts, s.a), point(&mut pts, s.b));
        if ia != ib {
            ends.insert((ia.min(ib), ia.max(ib)));
        }
    }
    if pts.len() > MAX_POINTS {
        return Err(format!("an arrangement of more than {MAX_POINTS} points is refused"));
    }
    // Every proper crossing, added once so both segments split at the same point.
    for (i, s) in input.iter().enumerate() {
        for t in input.iter().skip(i + 1) {
            let (d1, d2) = (s.dir(), t.dir());
            let den = d1.cross(d2);
            if den.abs() <= EPS * d1.length() * d2.length() {
                continue; // parallel: a shared end or a collinear overlap is already a point
            }
            let q = t.a - s.a;
            let (ta, tb) = (q.cross(d2) / den, q.cross(d1) / den);
            if ta > 0.0 && ta < 1.0 && tb > 0.0 && tb < 1.0 {
                point(&mut pts, s.a + d1 * ta);
            }
        }
    }
    if pts.len() > MAX_POINTS {
        return Err(format!("an arrangement of more than {MAX_POINTS} points is refused"));
    }
    // Split each segment at every point that lies on it, then weld the edges.
    let mut edges: BTreeSet<(usize, usize)> = BTreeSet::new();
    for (ia, ib) in ends {
        let (pa, pb) = (pts[ia], pts[ib]);
        let d = pb - pa;
        let len2 = d.dot(d);
        let mut cuts: Vec<(f64, usize)> = pts
            .iter()
            .enumerate()
            .filter_map(|(k, q)| {
                if k == ia || k == ib {
                    return None;
                }
                let t = (*q - pa).dot(d) / len2;
                if t <= 1e-9 || t >= 1.0 - 1e-9 {
                    return None;
                }
                (pa + d * t).dist(*q).le(&EPS).then_some((t, k))
            })
            .collect();
        cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut prev = ia;
        for (_, k) in cuts {
            if k != prev {
                edges.insert((prev.min(k), prev.max(k)));
                prev = k;
            }
        }
        if prev != ib {
            edges.insert((prev.min(ib), prev.max(ib)));
        }
    }
    prune(&mut edges);
    Ok((pts, edges))
}

/// The index of `p`, welding it to a point already within EPS.
fn point(pts: &mut Vec<P2>, p: P2) -> usize {
    if let Some(i) = pts.iter().position(|q| q.dist(p) <= EPS) {
        return i;
    }
    pts.push(p);
    pts.len() - 1
}

/// Drop every degree-1 vertex and its edge, repeatedly: a segment that ends in the middle of a cell
/// cuts nothing, and a run of them goes with it.
fn prune(edges: &mut BTreeSet<(usize, usize)>) {
    loop {
        let mut deg: BTreeMap<usize, usize> = BTreeMap::new();
        for (u, v) in edges.iter() {
            *deg.entry(*u).or_default() += 1;
            *deg.entry(*v).or_default() += 1;
        }
        let dead: BTreeSet<usize> = deg.iter().filter(|(_, d)| **d < 2).map(|(k, _)| *k).collect();
        if dead.is_empty() {
            return;
        }
        edges.retain(|(u, v)| !dead.contains(u) && !dead.contains(v));
        if edges.is_empty() {
            return;
        }
    }
}

/// One traced cycle: its vertices and twice its signed area, positive counter-clockwise.
struct Traced {
    verts: Vec<usize>,
    area: f64,
}

/// The cycles of the arrangement, and the face on the left of every directed edge (`None` for the
/// unbounded face).
fn trace(pts: &[P2], edges: &BTreeSet<(usize, usize)>) -> Result<(Vec<Traced>, Face), String> {
    let mut adj: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (u, v) in edges.iter() {
        adj.entry(*u).or_default().push(*v);
        adj.entry(*v).or_default().push(*u);
    }
    let mut seen: BTreeSet<(usize, usize)> = BTreeSet::new();
    let mut traced: Vec<Traced> = Vec::new();
    for (u, v) in edges.iter() {
        for (s, t) in [(*u, *v), (*v, *u)] {
            if seen.contains(&(s, t)) {
                continue;
            }
            let mut verts = vec![s];
            let (mut a, mut b) = (s, t);
            let mut guard = 0;
            loop {
                if !seen.insert((a, b)) {
                    return Err("the arrangement's edges do not close a face".into());
                }
                verts.push(b);
                if b == s {
                    break;
                }
                let Some(next) = turn(&adj, pts, b, a) else {
                    return Err(format!("the arrangement has a dead end at vertex {b}"));
                };
                (a, b) = (b, next);
                guard += 1;
                if guard > 2 * edges.len() + 2 {
                    return Err("the arrangement's trace does not close".into());
                }
            }
            if verts.last() == verts.first() {
                verts.pop();
            }
            if verts.len() < 3 {
                return Err("the arrangement traced a face of fewer than three sides".into());
            }
            traced.push(Traced { area: area(&verts, pts), verts });
        }
    }
    let mut left: Face = BTreeMap::new();
    for (i, t) in traced.iter().enumerate() {
        let holder = if t.area > 0.0 { Some(i) } else { held_by(&traced, &kept_of(&traced), i, pts) };
        for pair in dirs(&t.verts) {
            left.insert(pair, holder);
        }
    }
    Ok((traced, left))
}

/// The face on the left of each directed edge of a cycle.
fn dirs(verts: &[usize]) -> Vec<(usize, usize)> {
    (0..verts.len()).map(|i| (verts[i], verts[(i + 1) % verts.len()])).collect()
}

/// A cycle's face if it is a cell, else the cell that holds it as a hole.
fn held_by(traced: &[Traced], kept: &[bool], i: usize, pts: &[P2]) -> Option<usize> {
    let neg = &traced.get(i)?.verts;
    let (a, b) = (*neg.first()?, *neg.get(1)?);
    // Just left of the cycle's first edge: the side its face is on, so a positive cell that merely
    // shares the ring (the hole's own cell) cannot hold it.
    let p = left_of(pts[a], pts[b]);
    let mut best: Option<usize> = None;
    let mut size = f64::INFINITY;
    for (j, t) in traced.iter().enumerate() {
        if t.area <= 0.0 || !kept.get(j).copied().unwrap_or(false) {
            continue;
        }
        let ring: Vec<P2> = t.verts.iter().map(|&k| pts[k]).collect();
        let s = ring_area(&ring).abs();
        if s < size && inside_ring(p, &ring) {
            size = s;
            best = Some(j);
        }
    }
    best
}

fn kept_of(traced: &[Traced]) -> Vec<bool> {
    traced.iter().map(|t| t.area > 0.0).collect()
}

/// A point just left of a directed edge, where the face on its left lies.
fn left_of(a: P2, b: P2) -> P2 {
    let d = (b - a).unit();
    P2((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0) + d.perp() * (EPS * 4.0)
}

/// The next vertex after arriving at `v` from `u`: the first edge clockwise from the way back, so the
/// face stays on the left.
fn turn(adj: &BTreeMap<usize, Vec<usize>>, pts: &[P2], v: usize, u: usize) -> Option<usize> {
    let start = angle(pts[u] - pts[v]);
    let mut best: Option<(f64, usize)> = None;
    for &w in adj.get(&v)? {
        if w == u {
            continue;
        }
        let d = angle(pts[w] - pts[v]);
        let mut cw = (start - d).rem_euclid(std::f64::consts::TAU);
        if cw <= 0.0 {
            cw += std::f64::consts::TAU;
        }
        if best.is_none_or(|(b, k)| cw < b || (cw == b && w < k)) {
            best = Some((cw, w));
        }
    }
    best.map(|(_, w)| w)
}

fn angle(d: P2) -> f64 {
    d.1.atan2(d.0)
}

/// Twice the signed area of a cycle: positive counter-clockwise.
fn area(verts: &[usize], pts: &[P2]) -> f64 {
    let mut sum = 0.0;
    for i in 0..verts.len() {
        sum += pts[verts[i]].cross(pts[verts[(i + 1) % verts.len()]]);
    }
    sum
}

fn inside_ring(p: P2, ring: &[P2]) -> bool {
    if ring.len() < 3 {
        return false;
    }
    let mut hit = false;
    for i in 0..ring.len() {
        let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
        if (a.1 > p.1) != (b.1 > p.1) {
            let x = a.0 + (p.1 - a.1) * (b.0 - a.0) / (b.1 - a.1);
            if p.0 < x {
                hit = !hit;
            }
        }
    }
    hit
}

fn inside(p: P2, segs: &[Seg]) -> bool {
    let mut hit = false;
    for s in segs {
        let (a, b) = (s.a, s.b);
        if (a.1 > p.1) != (b.1 > p.1) {
            let x = a.0 + (p.1 - a.1) * (b.0 - a.0) / (b.1 - a.1);
            if p.0 < x {
                hit = !hit;
            }
        }
    }
    hit
}

// --- output -------------------------------------------------------------------------------------

type Face = BTreeMap<(usize, usize), Option<usize>>;

/// The boundary of the kept cells: an edge kept on one side only, traced into outer loops.
fn boundary(edges: &BTreeSet<(usize, usize)>, left: &Face, kept: &[bool], pts: &[P2]) -> Result<Vec<Region>, String> {
    let face_kept = |f: Option<usize>| f.and_then(|i| kept.get(i).copied()).unwrap_or(false);
    let mut dirs: BTreeSet<(usize, usize)> = BTreeSet::new();
    for (u, v) in edges.iter() {
        let (a, b) = (face_kept(left.get(&(*u, *v)).copied().flatten()), face_kept(left.get(&(*v, *u)).copied().flatten()));
        if a == b {
            continue;
        }
        if a {
            dirs.insert((*u, *v));
        } else {
            dirs.insert((*v, *u));
        }
    }
    let mut adj: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (u, v) in dirs.iter() {
        adj.entry(*u).or_default().push(*v);
    }
    let mut seen: BTreeSet<(usize, usize)> = BTreeSet::new();
    let mut out: Vec<Region> = Vec::new();
    let mut holes: Vec<Vec<P2>> = Vec::new();
    for (s, t) in dirs.iter() {
        if seen.contains(&(*s, *t)) {
            continue;
        }
        let (mut a, mut b) = (*s, *t);
        let mut ring = vec![pts[a]];
        let mut guard = 0;
        loop {
            if !seen.insert((a, b)) {
                return Err("the kept boundary's edges do not close a loop".into());
            }
            ring.push(pts[b]);
            if b == *s {
                break;
            }
            let Some(next) = turn(&adj, pts, b, a) else {
                return Err("the kept boundary has a dead end".into());
            };
            (a, b) = (b, next);
            guard += 1;
            if guard > dirs.len() + 1 {
                return Err("the kept boundary's trace does not close".into());
            }
        }
        if ring.last() == ring.first() {
            ring.pop();
        }
        if ring_area(&ring) > 0.0 {
            out.push(Region { outer: ring, holes: Vec::new() });
        } else {
            holes.push(ring);
        }
    }
    // A hole belongs to the smallest outer loop that holds it.
    for hole in holes {
        let Some(p) = hole.get(1).map(|b| left_of(hole[0], *b)) else { continue };
        let mut best: Option<usize> = None;
        let mut size = f64::INFINITY;
        for (i, r) in out.iter().enumerate() {
            let s = ring_area(&r.outer).abs();
            if s < size && inside_ring(p, &r.outer) {
                size = s;
                best = Some(i);
            }
        }
        if let Some(i) = best {
            out[i].holes.push(hole);
        }
    }
    Ok(out)
}

fn ring_area(ring: &[P2]) -> f64 {
    let mut sum = 0.0;
    for i in 0..ring.len() {
        sum += ring[i].cross(ring[(i + 1) % ring.len()]);
    }
    sum
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ring(lo: (f64, f64), hi: (f64, f64)) -> Vec<Seg> {
        let (x0, y0, x1, y1) = (lo.0, lo.1, hi.0, hi.1);
        vec![
            Seg::new(P2(x0, y0), P2(x1, y0)),
            Seg::new(P2(x1, y0), P2(x1, y1)),
            Seg::new(P2(x1, y1), P2(x0, y1)),
            Seg::new(P2(x0, y1), P2(x0, y0)),
        ]
    }

    fn size(r: &Region) -> f64 {
        ring_area(&r.outer) / 2.0 - r.holes.iter().map(|h| -ring_area(h) / 2.0).sum::<f64>()
    }

    fn rounded(v: f64) -> f64 {
        (v * 1000.0).round() / 1000.0
    }

    #[test]
    fn two_crossing_lines_make_four_cells() {
        let mut b = vec![Seg::new(P2(0.0, 1.0), P2(2.0, 1.0)), Seg::new(P2(1.0, 0.0), P2(1.0, 2.0))];
        b.extend(ring((0.0, 0.0), (2.0, 2.0)));
        // `a` is the square; the two lines only cut it.
        let a = ring((0.0, 0.0), (2.0, 2.0));
        let out = resolve(&a, &b, Mode::Split).unwrap();
        assert_eq!(out.len(), 4, "{out:?}");
        for r in &out {
            assert_eq!(rounded(size(r)), 1.0);
        }
    }

    #[test]
    fn a_sill_against_a_pier_subtracts_only_what_it_covers() {
        let face = ring((0.0, 0.0), (4.0, 3.0));
        let sill = ring((0.0, 0.0), (1.0, 1.2));
        let out = resolve(&face, &sill, Mode::Difference).unwrap();
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(rounded(size(&out[0])), 12.0 - 1.2);
    }

    #[test]
    fn a_box_on_a_podium_leaves_a_hole_in_its_roof() {
        let roof = ring((0.0, 0.0), (4.0, 4.0));
        let box_top = ring((1.0, 1.0), (3.0, 3.0));
        let out = resolve(&roof, &box_top, Mode::Difference).unwrap();
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(out[0].holes.len(), 1);
        assert_eq!(rounded(size(&out[0])), 16.0 - 4.0);
    }

    #[test]
    fn nested_loops_union_to_one_region() {
        let outer = ring((0.0, 0.0), (6.0, 6.0));
        let inner = ring((2.0, 2.0), (4.0, 4.0));
        let out = resolve(&outer, &inner, Mode::Union).unwrap();
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(rounded(size(&out[0])), 36.0);
    }

    /// The `prisms_holed` case the arrangement exists for: two coplanar tops share part of their boundary,
    /// so the overlap is collinear, not a crossing. Subtracting the right box must leave only the left part.
    #[test]
    fn a_collinear_overlap_subtracts_only_the_covered_part() {
        let a = ring((0.0, 0.0), (4.0, 4.0));
        let b = ring((2.0, 0.0), (6.0, 4.0));
        let out = resolve(&a, &b, Mode::Difference).unwrap();
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(rounded(size(&out[0])), 8.0);
    }

    /// A closed loop drawn inside a face: Split keeps both cells - the island and the face around it,
    /// with the drawn loop as that face's hole. Punch-through builds on this output.
    #[test]
    fn a_closed_loop_inside_a_face_makes_an_island_and_a_hole() {
        let a = ring((0.0, 0.0), (6.0, 6.0));
        let b = ring((2.0, 2.0), (4.0, 4.0));
        let out = resolve(&a, &b, Mode::Split).unwrap();
        assert_eq!(out.len(), 2, "{out:?}");
        let island = out.iter().find(|r| r.holes.is_empty()).unwrap();
        let around = out.iter().find(|r| r.holes.len() == 1).unwrap();
        assert_eq!(rounded(size(island)), 4.0);
        assert_eq!(rounded(size(around)), 32.0);
    }

    /// A chord ending on the same boundary segment twice: both ends are T-junctions, not crossings, and
    /// neither is an antenna. Draw-on-face needs both halves of the face to survive.
    #[test]
    fn a_chord_with_two_t_junctions_splits_the_face_in_two() {
        let a = ring((0.0, 0.0), (2.0, 2.0));
        let b = vec![Seg::new(P2(0.0, 1.0), P2(2.0, 1.0))];
        let out = resolve(&a, &b, Mode::Split).unwrap();
        assert_eq!(out.len(), 2, "{out:?}");
        let mut sizes: Vec<f64> = out.iter().map(|r| rounded(size(r))).collect();
        sizes.sort_by(|x, y| x.total_cmp(y));
        assert_eq!(sizes, vec![2.0, 2.0]);
    }

    /// A point with a non-finite coordinate is refused before any tracing, and the cap is checked.
    #[test]
    fn hostile_segments_are_refused() {
        let a = ring((0.0, 0.0), (2.0, 2.0));
        let bad = vec![Seg::new(P2(f64::NAN, 0.0), P2(1.0, 1.0))];
        assert!(resolve(&a, &bad, Mode::Split).is_err());
        let far = vec![Seg::new(P2(1e300, 0.0), P2(1e300, 1.0))];
        assert!(resolve(&a, &far, Mode::Split).is_err(), "finite but out of range must be refused too");
        let many = (0..=MAX_POINTS).map(|i| Seg::new(P2(i as f64, 0.0), P2(i as f64, 1.0))).collect::<Vec<_>>();
        assert!(resolve(&many, &[], Mode::Split).is_err());
    }

    #[test]
    fn a_dangling_line_is_pruned_and_cuts_nothing() {
        let a = ring((0.0, 0.0), (2.0, 2.0));
        let mut b = ring((0.0, 0.0), (2.0, 2.0));
        b.push(Seg::new(P2(0.0, 1.0), P2(1.0, 1.0)));
        let out = resolve(&a, &b, Mode::Split).unwrap();
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(rounded(size(&out[0])), 4.0);
    }
}
