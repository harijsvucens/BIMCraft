//! Direct modeling: the flat faces a form's triangles make, and pushing one along its normal.
//!
//! A face is grouped the way the 3D scene groups one ([`crate::view3d::facets`]): triangles that share a welded corner's
//! edge and lie in one plane are one face, however they are wound; triangles that turn into one another less than the
//! scene's smooth angle are one curved surface, which has no single plane to push. The weld is the scene's too, so a
//! corner a pick can select is the corner a push moves.

use std::collections::{HashMap, HashSet};

use crate::geom::P2;
use crate::model::MAX_COORD;
use crate::view3d::{P3, add, cross, dot, mul, sub, unit};

/// How far off a face's plane a point may lie and still be on it, and how far outside it: the weld's micron, with room
/// for a pick's rounding.
pub const ON_FACE: f64 = 1e-3;

/// A flat face of a mesh: the triangles it is made of, and the way it faces, unit out of the solid as its triangles wound
/// it.
#[derive(Debug)]
pub struct Face {
    pub tris: Vec<usize>,
    pub normal: P3,
}

/// The flat face `at` lies on, in the mesh's own axes: the face of the triangle under it nearest its plane, if one holds
/// it. A curved side has no plane of its own, and is refused with what to pick instead.
pub fn face_at(tris: &[[P3; 3]], at: P3) -> Result<Face, String> {
    if !at.iter().all(|v| v.is_finite() && v.abs() <= MAX_COORD) {
        return Err(format!("a point to push is [x, y, z] in mm within {MAX_COORD}"));
    }
    let (_, faces) = crate::view3d::facets(tris);
    let mut under: Option<(f64, usize, P3)> = None;
    for (i, t) in tris.iter().enumerate() {
        let n = normal(*t);
        let size = dot(n, n);
        if size <= 0.5 || size.is_nan() {
            continue;
        }
        let off = dot(n, sub(at, t[0])).abs();
        if off > ON_FACE || !holds(*t, n, at) {
            continue;
        }
        if under.is_none_or(|(was, ..)| off < was) {
            under = Some((off, i, n));
        }
    }
    let Some((_, i, n)) = under else { return Err("no flat face of the mesh holds that point".into()) };
    let Some(face) = faces.get(i).copied().flatten() else {
        return Err("that point is on a curved side, which has no plane of its own to push: pick a flat face".into());
    };
    let group: Vec<usize> = (0..tris.len()).filter(|j| faces.get(*j).copied().flatten() == Some(face)).collect();
    Ok(Face { tris: group, normal: n })
}

/// Why these triangles are no closed surface, if they are not: every edge of the welded mesh is walked once each way, by
/// one triangle in each direction. That is two triangles an edge, no fewer (a hole) and no more (a fold or two solids
/// meeting), facing the same way — which is what a shell IFC writes and a shaded face both take for granted.
pub fn sealed(tris: &[[P3; 3]]) -> Result<(), String> {
    let mut pairs: HashMap<([i64; 3], [i64; 3]), usize> = HashMap::new();
    let mut ways: HashMap<([i64; 3], [i64; 3]), usize> = HashMap::new();
    for k in crate::view3d::weld(tris) {
        for j in 0..3 {
            let (a, b) = (k[j], k[(j + 1) % 3]);
            if a == b {
                return Err("a triangle of it is folded on itself, two of its corners being one".into());
            }
            *pairs.entry((a.min(b), a.max(b))).or_default() += 1;
            *ways.entry((a, b)).or_default() += 1;
        }
    }
    if pairs.is_empty() {
        return Err("it has no triangles".into());
    }
    if let Some(n) = pairs.values().find(|n| **n != 2) {
        return Err(format!("an edge of it is shared by {n} triangles, not two: the triangles do not close a solid"));
    }
    if ways.values().any(|n| *n != 1) {
        return Err("an edge of it is walked the same way by both triangles beside it: one of them is turned round".into());
    }
    Ok(())
}

/// The volume the triangles enclose, mm³: only meaningful where they close a solid.
fn volume(tris: &[[P3; 3]]) -> f64 {
    tris.iter().map(|[a, b, c]| dot(*a, cross(*b, *c))).sum::<f64>() / 6.0
}

/// `tris` with `face` moved `distance` mm along its normal, out of the solid where that is positive: every corner welded
/// to one of the face's moves with it, so the faces round it stretch as a hand pulling a SketchUp face's does. A solid
/// that closes may not be pushed through itself, into an inside-out shell.
pub fn push(tris: &[[P3; 3]], face: &Face, distance: f64) -> Result<Vec<[P3; 3]>, String> {
    if !distance.is_finite() || distance.abs() > MAX_COORD {
        return Err(format!("a distance is a number of mm within {MAX_COORD}, not {distance}"));
    }
    if face.tris.iter().any(|i| *i >= tris.len()) {
        return Err("that face is no face of these triangles".into());
    }
    let keys = crate::view3d::weld(tris);
    let mut moving: HashSet<[i64; 3]> = HashSet::new();
    for i in &face.tris {
        if let Some(k) = keys.get(*i) {
            moving.extend(k.iter().copied());
        }
    }
    let step = mul(face.normal, distance);
    let out: Vec<[P3; 3]> =
        tris.iter().zip(&keys).map(|(t, k)| std::array::from_fn(|j| if moving.contains(&k[j]) { add(t[j], step) } else { t[j] })).collect();
    if out.iter().flatten().flatten().any(|v| !v.is_finite() || v.abs() > MAX_COORD) {
        return Err(format!("that would put a corner past {MAX_COORD} mm"));
    }
    if sealed(tris).is_ok() && volume(&out) <= 1.0 {
        return Err("that would turn the solid inside out: push it less far".into());
    }
    Ok(out)
}

/// The way triangle `t` faces, unit: no area, no way.
fn normal([a, b, c]: [P3; 3]) -> P3 {
    unit(cross(sub(b, a), sub(c, a)))
}

/// Whether `at`, in the plane of triangle `t`, lies inside it, an edge counting as inside: the triangle flattened along
/// the axis its normal leans on most, so its area does not collapse, and each side read as a distance in mm.
fn holds([a, b, c]: [P3; 3], n: P3, at: P3) -> bool {
    let axis = (0..3).max_by(|i, j| n[*i].abs().total_cmp(&n[*j].abs())).unwrap_or(2);
    let flat = |v: P3| match axis {
        0 => P2(v[1], v[2]),
        1 => P2(v[0], v[2]),
        _ => P2(v[0], v[1]),
    };
    let (a, b, c, p) = (flat(a), flat(b), flat(c), flat(at));
    // How far p stands to one side of the line uv: the cross product is twice the area, so dividing by the line's own
    // length gives a length.
    let side = |u: P2, v: P2| {
        let cross = (v.0 - u.0) * (p.1 - u.1) - (v.1 - u.1) * (p.0 - u.0);
        let len = (v - u).length();
        if len > 0.0 { cross / len } else { cross }
    };
    let (ab, bc, ca) = (side(a, b), side(b, c), side(c, a));
    (ab >= -ON_FACE && bc >= -ON_FACE && ca >= -ON_FACE) || (ab <= ON_FACE && bc <= ON_FACE && ca <= ON_FACE)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A box `side` mm each way with a corner on the origin, its triangles turned out: two a face.
    fn box_tris(side: f64) -> Vec<[P3; 3]> {
        let s = side;
        let p = [[0.0, 0.0, 0.0], [s, 0.0, 0.0], [s, s, 0.0], [0.0, s, 0.0], [0.0, 0.0, s], [s, 0.0, s], [s, s, s], [0.0, s, s]];
        let ix = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ];
        ix.iter().map(|[a, b, c]| [p[*a], p[*b], p[*c]]).collect()
    }

    fn volume(tris: &[[P3; 3]]) -> f64 {
        tris.iter().map(|[a, b, c]| dot(*a, cross(*b, *c))).sum::<f64>() / 6.0
    }

    #[test]
    fn a_boxs_top_is_one_face_of_four_corners_and_pushing_it_makes_the_sides_taller() {
        let tris = box_tris(1000.0);
        let face = face_at(&tris, [500.0, 300.0, 1000.0]).expect("the top holds the point");
        assert_eq!((face.tris.len(), face.normal), (2, [0.0, 0.0, 1.0]), "two triangles, facing up");
        let pushed = push(&tris, &face, 500.0).expect("it moves");
        assert_eq!(volume(&pushed), 1.5e9, "a cube a metre each way, half a metre taller");
        // The face is still one face, and its corners are all that moved.
        let again = face_at(&pushed, [500.0, 300.0, 1500.0]).expect("the top is there");
        assert_eq!(again.tris.len(), 2);
        assert_eq!(again.normal, [0.0, 0.0, 1.0]);
        assert!(pushed.iter().flatten().all(|p| p[2] == 0.0 || p[2] == 1500.0), "every corner is on the base or the new top");
        let top: std::collections::BTreeSet<[i64; 2]> =
            pushed.iter().flatten().filter(|p| p[2] == 1500.0).map(|p| [p[0] as i64, p[1] as i64]).collect();
        assert_eq!(top.len(), 4, "the top's four corners, wherever the triangles write them");
    }

    #[test]
    fn a_push_is_refused_where_no_face_holds_the_point_or_it_would_leave_the_model() {
        let tris = box_tris(1000.0);
        let e = face_at(&tris, [5000.0, 5000.0, 5000.0]).expect_err("nothing there");
        assert!(e.contains("no flat face"), "{e}");
        let top = face_at(&tris, [500.0, 300.0, 1000.0]).expect("the top");
        assert!(push(&tris, &top, f64::NAN).is_err(), "a distance that is not a number");
        assert!(push(&tris, &top, 1e300).is_err(), "a distance past the model");
        assert!(push(&tris, &top, MAX_COORD).is_err(), "a push that would leave the model");
        let e = push(&tris, &top, -2000.0).expect_err("the top would pass the bottom");
        assert!(e.contains("inside out"), "{e}");
    }

    #[test]
    fn a_mesh_is_sealed_only_where_every_edge_meets_two_triangles_facing_apart() {
        let tris = box_tris(1000.0);
        assert!(sealed(&tris).is_ok(), "a box closes");
        let mut open = tris.clone();
        open.pop();
        let e = sealed(&open).unwrap_err();
        assert!(e.contains("not two"), "a hole: {e}");
        // The same triangles with one turned round: every edge still meets two, and they face the same way.
        let mut turned = tris.clone();
        turned[2].swap(1, 2);
        let e = sealed(&turned).unwrap_err();
        assert!(e.contains("turned round"), "{e}");
        // A third triangle on an edge: a fold, which no push can keep a solid.
        let mut folded = tris.clone();
        folded.push(tris[0]);
        let e = sealed(&folded).unwrap_err();
        assert!(e.contains("not two"), "a fold: {e}");
    }

    #[test]
    fn a_curved_side_is_no_face_while_its_caps_are() {
        // A 24-sided prism: its sides turn 15 degrees into one another, under the scene's 20, so they are one curved
        // surface; each cap is a fan of 22 coplanar triangles and one flat face.
        let n = 24;
        let ring = |z: f64| -> Vec<P3> {
            (0..n)
                .map(|k| {
                    let a = k as f64 * std::f64::consts::TAU / n as f64;
                    [1000.0 * a.cos(), 1000.0 * a.sin(), z]
                })
                .collect()
        };
        let (lo, hi) = (ring(0.0), ring(1000.0));
        let mut tris = Vec::new();
        for k in 0..n {
            let (a, b) = (k, (k + 1) % n);
            tris.push([lo[a], lo[b], hi[b]]);
            tris.push([lo[a], hi[b], hi[a]]);
        }
        for k in 1..n - 1 {
            tris.push([lo[0], lo[k + 1], lo[k]]);
            tris.push([hi[0], hi[k], hi[k + 1]]);
        }
        let e = face_at(&tris, [1000.0, 0.0, 500.0]).expect_err("a side");
        assert!(e.contains("curved side"), "{e}");
        let top = face_at(&tris, [0.0, 0.0, 1000.0]).expect("its cap");
        assert_eq!(top.tris.len(), n - 2, "the whole cap is one face");
    }
}
