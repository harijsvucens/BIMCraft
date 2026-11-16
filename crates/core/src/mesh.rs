//! Direct modeling: the flat faces a form's triangles make, and pushing one along its normal.
//!
//! A face is grouped here ([`facets`]): triangles that share a welded corner's edge and lie in one plane are one face,
//! however they are wound; triangles that turn into one another less than [`SMOOTH`] degrees are one curved surface, which
//! has no single plane to push. The scene draws by the same grouping, so a face a pick can select is the face a push moves.

use std::collections::{HashMap, HashSet};

use crate::geom::P2;
use crate::model::MAX_COORD;
use crate::vec3::{P3, add, cross, dot, mul, sub, unit};

/// How far off a face's plane a point may lie and still be on it, and how far outside it: the weld's micron, with room
/// for a pick's rounding.
pub const ON_FACE: f64 = 1e-3;

/// A flat face of a mesh: the triangles it is made of, the way it faces, unit out of the solid as its triangles wound it, and
/// how much surface it has.
#[derive(Debug)]
pub struct Face {
    /// Which face of the mesh it is: the place of its first triangle, as [`facets`] numbers one, so `mesh_check` can list it
    /// and a command name it again.
    pub id: usize,
    pub tris: Vec<usize>,
    pub normal: P3,
    /// Its surface, mm².
    pub area: f64,
}

/// The flat face `at` lies on, in the mesh's own axes: the face of the triangle under it nearest its plane, if one holds
/// it. A curved side has no plane of its own, and is refused with what to pick instead.
pub fn face_at(tris: &[[P3; 3]], at: P3) -> Result<Face, String> {
    if !at.iter().all(|v| v.is_finite() && v.abs() <= MAX_COORD) {
        return Err(format!("a point to push is [x, y, z] in mm within {MAX_COORD}"));
    }
    let (_, faces) = facets(tris);
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
    Ok(Face { id: face, tris: group.clone(), normal: n, area: area(tris, &group) })
}

/// Every flat face of a mesh, by id: what `mesh_check` lists and what a push can name instead of a point.
pub fn faces(tris: &[[P3; 3]]) -> Vec<Face> {
    let (_, ids) = facets(tris);
    let mut seen: HashSet<usize> = HashSet::new();
    let mut out = Vec::new();
    for (i, id) in ids.iter().enumerate() {
        let Some(id) = *id else { continue };
        if !seen.insert(id) {
            continue;
        }
        let group: Vec<usize> = (0..tris.len()).filter(|j| ids.get(*j).copied().flatten() == Some(id)).collect();
        out.push(Face { id, normal: normal(tris[i]), area: area(tris, &group), tris: group });
    }
    out.sort_by_key(|f| f.id);
    out
}

/// The flat face `id` of a mesh: the one [`faces`] lists and `mesh_check` names.
pub fn face_of(tris: &[[P3; 3]], id: usize) -> Result<Face, String> {
    faces(tris).into_iter().find(|f| f.id == id).ok_or_else(|| format!("this mesh has no flat face {id}: `mesh_check` lists them"))
}

/// The surface `group`'s triangles cover, mm².
fn area(tris: &[[P3; 3]], group: &[usize]) -> f64 {
    group
        .iter()
        .map(|i| tris.get(*i).map_or(0.0, |[a, b, c]| dot(cross(sub(*b, *a), sub(*c, *a)), cross(sub(*b, *a), sub(*c, *a))).sqrt() / 2.0))
        .sum()
}

/// What a mesh's triangles make of themselves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A closed surface: every welded edge is walked by two triangles, one each way — a solid.
    Solid,
    /// An open surface, as a sheet is: some edge is walked by one triangle only.
    Sheet,
    /// An edge walked by this many triangles: no surface, let alone a solid (three triangles meeting on an edge, or more).
    Crowded(usize),
    /// Two triangles walk one edge the same way: one of them is turned round.
    Reversed,
}

impl Kind {
    /// A word or two for what this is, for a command's reply.
    pub fn word(self) -> String {
        match self {
            Kind::Solid => "solid".into(),
            Kind::Sheet => "open sheet".into(),
            Kind::Crowded(n) => format!("an edge shared by {n} triangles"),
            Kind::Reversed => "a triangle turned round".into(),
        }
    }
}

/// A mesh measured: what it makes, how much space it holds and how much surface it has.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measure {
    pub kind: Kind,
    /// The space it encloses, mm³, as its triangles are wound: positive for a solid turned out, and close to 0 for a sheet,
    /// which encloses nothing.
    pub volume: f64,
    /// Its surface, mm².
    pub area: f64,
    pub triangles: usize,
}

impl Measure {
    /// Whether these triangles close a solid.
    pub fn is_solid(&self) -> bool {
        self.kind == Kind::Solid
    }
}

/// Why these triangles are no closed surface, if they are not: every edge of the welded mesh is walked once each way, by
/// one triangle in each direction. That is two triangles an edge, no fewer (a hole) and no more (a fold or two solids
/// meeting), facing the same way — which is what a shell IFC writes and a shaded face both take for granted.
pub fn sealed(tris: &[[P3; 3]]) -> Result<(), String> {
    match measure(tris)?.kind {
        Kind::Solid => Ok(()),
        Kind::Sheet => Err("an edge of it is shared by 1 triangles, not two: the triangles do not close a solid".into()),
        Kind::Crowded(n) => Err(format!("an edge of it is shared by {n} triangles, not two: the triangles do not close a solid")),
        Kind::Reversed => Err("an edge of it is walked the same way by both triangles beside it: one of them is turned round".into()),
    }
}

/// `tris` measured: which of [`Kind`] they make, the space they enclose and the surface they have. The one place a mesh's
/// solidity and its numbers are worked out, so `mesh`, a push, `mesh_check` and `forms::closed` all read the same answer.
pub fn measure(tris: &[[P3; 3]]) -> Result<Measure, String> {
    if tris.iter().flatten().flatten().any(|x| !x.is_finite() || x.abs() > MAX_COORD) {
        return Err("it reaches past where the model may go".into());
    }
    let mut pairs: HashMap<Edge, usize> = HashMap::new();
    let mut ways: HashMap<(Corner, Corner), usize> = HashMap::new();
    for k in weld(tris) {
        for j in 0..3 {
            let (a, b) = (k[j], k[(j + 1) % 3]);
            if a == b {
                return Err("a triangle of it is folded on itself, two of its corners being one".into());
            }
            *pairs.entry(edge(a, b)).or_default() += 1;
            *ways.entry((a, b)).or_default() += 1;
        }
    }
    if pairs.is_empty() {
        return Err("it has no triangles".into());
    }
    let crowded = pairs.values().copied().max().unwrap_or_default();
    let reversed = ways.values().any(|n| *n > 1);
    let open = pairs.values().any(|n| *n == 1);
    let kind = if crowded > 2 {
        Kind::Crowded(crowded)
    } else if reversed {
        Kind::Reversed
    } else if open {
        Kind::Sheet
    } else {
        Kind::Solid
    };
    let area = tris.iter().map(|[a, b, c]| dot(cross(sub(*b, *a), sub(*c, *a)), cross(sub(*b, *a), sub(*c, *a))).sqrt() / 2.0).sum();
    Ok(Measure { kind, volume: volume(tris), area, triangles: tris.len() })
}

/// A mesh's triangles as the closed solid a form made by a command must be: measured once, held to closing and to a volume,
/// and turned out. A sheet is refused here — `mesh` makes solids — though a mesh that comes from a file may be one.
pub(crate) struct Solid {
    pub tris: Vec<[P3; 3]>,
    pub measure: Measure,
}

pub(crate) fn solid(mut tris: Vec<[P3; 3]>) -> Result<Solid, String> {
    if tris.len() > crate::family::MAX_TRIANGLES {
        return Err(format!("it would be more than {} triangles", crate::family::MAX_TRIANGLES));
    }
    let mut measure = measure(&tris)?;
    match measure.kind {
        Kind::Solid => {}
        Kind::Sheet => return Err("it is an open surface, not a solid: every edge must be shared by two triangles".into()),
        Kind::Crowded(n) => {
            return Err(format!("an edge of it is shared by {n} triangles, not two: the triangles do not close a solid"));
        }
        Kind::Reversed => {
            return Err("an edge of it is walked the same way by both triangles beside it: one of them is turned round".into());
        }
    }
    if !measure.volume.is_finite() || measure.volume.abs() < 1.0 {
        return Err("it has no volume".into());
    }
    if measure.volume < 0.0 {
        tris.iter_mut().for_each(|t| t.swap(1, 2));
        measure.volume = -measure.volume;
    }
    Ok(Solid { tris, measure })
}

/// The volume the triangles enclose, mm³: only meaningful where they close a solid.
fn volume(tris: &[[P3; 3]]) -> f64 {
    tris.iter().map(|[a, b, c]| dot(*a, cross(*b, *c))).sum::<f64>() / 6.0
}

/// A corner of the weld, as the scene's own: a micron's grid.
pub(crate) type Corner = [i64; 3];

/// An edge of the weld, either way round: both ends sorted, so a walk from one and a walk from the other name one edge.
type Edge = (Corner, Corner);

/// How far, in degrees, two triangles may turn at the edge they share and still be one surface: the facets a curve is cut
/// into turn less, a crease like a seat's rim more.
const SMOOTH: f64 = 20.0;

/// The curved surfaces and the flat faces a mesh's triangles make: those that share an edge and turn less than [`SMOOTH`]
/// degrees there are one. A surface with a bend in it is a curve, each triangle's as its first triangle's index; one with
/// none is a face, numbered the same way, so two faces in one plane that do not touch are two. None for a triangle with no
/// area, which has no plane to be one.
pub(crate) fn facets(tris: &[[P3; 3]]) -> (Vec<Option<usize>>, Vec<Option<usize>>) {
    /// The first triangle of `i`'s surface, as `up` links them, shortening the way as it goes.
    fn first(up: &mut [usize], mut i: usize) -> usize {
        while let Some(&p) = up.get(i)
            && p != i
        {
            let grand = up.get(p).copied().unwrap_or(p);
            if let Some(slot) = up.get_mut(i) {
                *slot = grand;
            }
            i = grand;
        }
        i
    }
    let limit = SMOOTH.to_radians().cos();
    let normals: Vec<P3> = tris.iter().map(|[a, b, c]| unit(cross(sub(*b, *a), sub(*c, *a)))).collect();
    let normal = |i: usize| normals.get(i).copied().unwrap_or_default();
    let mut up: Vec<usize> = (0..tris.len()).collect();
    let corners = weld(tris);
    // The first triangle on each side, whose corners a micron's grid names and [`welded`] joins to a twin in the cells
    // around it.
    // ponytail: a third triangle on a side meets only the first; pair them all if facet lines show.
    let mut edges: HashMap<Edge, usize> = HashMap::new();
    let mut bent = Vec::new();
    for (i, _) in tris.iter().enumerate() {
        let c = corners.get(i).copied().unwrap_or_default();
        for k in 0..3 {
            let (a, b) = (c[k], c[(k + 1) % 3]);
            let j = *edges.entry(edge(a, b)).or_insert(i);
            // A triangle with no area has no normal, so it turns 90° from every other.
            let turn = dot(normal(i), normal(j));
            // One plane, whichever way round each triangle is wound, is one face; turning little the same way is one
            // surface. A fold (normals near opposite, so `turn` near -1) is neither.
            let flat = turn.abs() >= 1.0 - 1e-9;
            if j != i && a != b && (turn > limit || flat) {
                let (ri, rj) = (first(&mut up, i), first(&mut up, j));
                if let Some(slot) = up.get_mut(ri.max(rj)) {
                    *slot = ri.min(rj);
                }
                if !flat {
                    bent.push(i);
                }
            }
        }
    }
    let curved: HashSet<usize> = bent.into_iter().map(|i| first(&mut up, i)).collect();
    let mut curves = Vec::with_capacity(tris.len());
    let mut faces = Vec::with_capacity(tris.len());
    for (i, n) in normals.iter().enumerate() {
        let root = first(&mut up, i);
        let (curve, face) = if curved.contains(&root) { (Some(root), None) } else { (None, (dot(*n, *n) > 0.5).then_some(root)) };
        curves.push(curve);
        faces.push(face);
    }
    (curves, faces)
}

/// The corner each triangle corner of a mesh welds to, by a micron's grid: the same point written twice, or twice within
/// the weld, gets one. The scene groups a mesh's faces by these corners, so what a pick selects is what a push moves.
pub(crate) fn weld(tris: &[[P3; 3]]) -> Vec<[Corner; 3]> {
    let mut canon: HashMap<Corner, Corner> = HashMap::new();
    tris.iter().map(|t| t.map(|p| welded(&mut canon, p.map(|x| (x * 1e3).round() as i64), p))).collect()
}

/// The corner a rounded `q` stands for: itself, or the first corner already met within the weld, which a cell's edge may
/// have put next door. `p` is `q` before it was rounded: a corner within a micron of another can land either side of a
/// cell's edge, and the two are one point.
fn welded(canon: &mut HashMap<Corner, Corner>, q: Corner, p: P3) -> Corner {
    if let Some(c) = canon.get(&q) {
        return *c;
    }
    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                if dx == 0 && dy == 0 && dz == 0 {
                    continue;
                }
                // Saturated: a corner of 1e300 (or an infinity) rounds to the key's end, where `+` would overflow.
                let next = [q[0].saturating_add(dx), q[1].saturating_add(dy), q[2].saturating_add(dz)];
                if let Some(&c) = canon.get(&next) {
                    // The weld, and the half micron the cell's own rounding may add.
                    let at = c.map(|v| v as f64 * 1e-3);
                    if sub(p, at).iter().all(|v| v.abs() <= 1.5e-3) {
                        canon.insert(q, c);
                        return c;
                    }
                }
            }
        }
    }
    canon.insert(q, q);
    q
}

/// An edge on a face's boundary: which edge it is, and the way round the face's triangles walk it.
struct OnRim {
    edge: Edge,
    from: Corner,
    to: Corner,
}

fn edge(a: Corner, b: Corner) -> Edge {
    (a.min(b), a.max(b))
}

/// A face's boundary, walked as its triangles walk it: each edge of it no other triangle of the face shares, from its first
/// corner to its second, which is the way the face's outside runs.
fn rim(keys: &[[Corner; 3]], group: &[usize]) -> Vec<OnRim> {
    let mut within: HashMap<Edge, usize> = HashMap::new();
    let tri = |i: usize| keys.get(i).copied().unwrap_or_default();
    for i in group {
        let t = tri(*i);
        for j in 0..3 {
            *within.entry(edge(t[j], t[(j + 1) % 3])).or_default() += 1;
        }
    }
    let mut out = Vec::new();
    for i in group {
        let t = tri(*i);
        for j in 0..3 {
            let (a, b) = (t[j], t[(j + 1) % 3]);
            if a != b && within.get(&edge(a, b)) == Some(&1) {
                out.push(OnRim { edge: edge(a, b), from: a, to: b });
            }
        }
    }
    out
}

/// Whether the face is a sheet: no other triangle of the mesh holds it, along a whole edge or at a corner standing part way
/// along one — a T-junction, as a tessellated surface brings in, which whole edges alone cannot see. A face that is anything
/// else is part of a surface: pushing it moves its corners and stretches what is beside it.
fn alone(tris: &[[P3; 3]], keys: &[[Corner; 3]], group: &[usize], rim: &[OnRim], face: &Face) -> bool {
    if group.len() == tris.len() {
        return true;
    }
    let inside: HashSet<usize> = group.iter().copied().collect();
    let boundary: HashSet<Edge> = rim.iter().map(|e| e.edge).collect();
    let mut place: HashMap<Corner, P3> = HashMap::new();
    for i in group {
        let (Some(t), Some(k)) = (tris.get(*i), keys.get(*i)) else { continue };
        for j in 0..3 {
            place.entry(k[j]).or_insert(t[j]);
        }
    }
    let start = group.first().and_then(|i| tris.get(*i)).map_or([0.0; 3], |t| t[0]);
    for (i, k) in keys.iter().enumerate().filter(|(i, _)| !inside.contains(i)) {
        if (0..3).any(|j| boundary.contains(&edge(k[j], k[(j + 1) % 3]))) {
            return false;
        }
        let t = tris.get(i).copied().unwrap_or_default();
        for p in t {
            // Only a corner in the face's own plane can stand on its boundary.
            if dot(face.normal, sub(p, start)).abs() > ON_FACE {
                continue;
            }
            if rim.iter().any(|e| stands_on(place.get(&e.from).copied(), place.get(&e.to).copied(), p)) {
                return false;
            }
        }
    }
    true
}

/// Whether corner `p` stands on the line from `a` to `b` between them, a weld away from either end: a corner there is a
/// T-junction, its triangle meeting the face along part of an edge and walking none of it whole.
fn stands_on(a: Option<P3>, b: Option<P3>, p: P3) -> bool {
    let (Some(a), Some(b)) = (a, b) else { return false };
    let ab = sub(b, a);
    let square = dot(ab, ab);
    if square <= ON_FACE * ON_FACE {
        return false;
    }
    let along = dot(sub(p, a), ab) / square.sqrt();
    if along <= ON_FACE || along >= square.sqrt() - ON_FACE {
        return false;
    }
    let foot = add(a, mul(ab, along / square.sqrt()));
    let off = sub(p, foot);
    dot(off, off).sqrt() <= ON_FACE
}

/// `tris` with `face` moved `distance` mm along its normal, out of the solid where that is positive: every corner welded
/// to one of the face's moves with it, so the faces round it stretch as a hand pulling a SketchUp face's does. A face with
/// nothing beside it — a sheet, as an imported mesh may hold — is no surface to stretch: it rises into a solid, keeping
/// itself as the near end. A solid that closes may not be pushed through itself, into an inside-out shell.
pub fn push(tris: &[[P3; 3]], face: &Face, distance: f64) -> Result<Vec<[P3; 3]>, String> {
    if !distance.is_finite() || distance.abs() > MAX_COORD {
        return Err(format!("a distance is a number of mm within {MAX_COORD}, not {distance}"));
    }
    if face.tris.iter().any(|i| *i >= tris.len()) {
        return Err("that face is no face of these triangles".into());
    }
    let keys = weld(tris);
    let rim = rim(&keys, &face.tris);
    if !rim.is_empty() && alone(tris, &keys, &face.tris, &rim, face) {
        return extrude(tris, &keys, face, &rim, distance).and_then(held);
    }
    let mut moving: HashSet<[i64; 3]> = HashSet::new();
    for i in &face.tris {
        if let Some(k) = keys.get(*i) {
            moving.extend(k.iter().copied());
        }
    }
    let step = mul(face.normal, distance);
    let out: Vec<[P3; 3]> =
        tris.iter().zip(&keys).map(|(t, k)| std::array::from_fn(|j| if moving.contains(&k[j]) { add(t[j], step) } else { t[j] })).collect();
    let out = held(out)?;
    if measure(tris).is_ok_and(|m| m.is_solid()) && volume(&out) <= 1.0 {
        return Err("that would turn the solid inside out: push it less far".into());
    }
    Ok(out)
}

/// `tris` with `face` raised `distance` mm into a solid and the face left as its near end, its copy standing at the far one,
/// with a wall of two triangles along each of `rim`'s edges: what a sheet becomes when a hand pulls it up, off either side.
fn extrude(tris: &[[P3; 3]], keys: &[[Corner; 3]], face: &Face, rim: &[OnRim], distance: f64) -> Result<Vec<[P3; 3]>, String> {
    let step = mul(face.normal, distance);
    // Where each corner the face welds to stands, as its triangles write it.
    let mut place: HashMap<Corner, P3> = HashMap::new();
    for i in &face.tris {
        let (Some(t), Some(k)) = (tris.get(*i), keys.get(*i)) else { continue };
        for j in 0..3 {
            place.entry(k[j]).or_insert(t[j]);
        }
    }
    let inside: HashSet<usize> = face.tris.iter().copied().collect();
    let mut out: Vec<[P3; 3]> = tris.iter().enumerate().filter(|(i, _)| !inside.contains(i)).map(|(_, t)| *t).collect();
    if out.len().saturating_add(2 * face.tris.len() + 2 * rim.len()) > crate::family::MAX_TRIANGLES {
        return Err(format!("that would make the mesh more than {} triangles, the most a form holds", crate::family::MAX_TRIANGLES));
    }
    let mut raised: Vec<[P3; 3]> = Vec::with_capacity(2 * face.tris.len() + 2 * rim.len());
    for i in &face.tris {
        let Some(t) = tris.get(*i) else { continue };
        // The face turned round is the near end, facing out of what rises; its copy is the far end.
        raised.push([t[2], t[1], t[0]]);
        raised.push(t.map(|p| add(p, step)));
    }
    for e in rim {
        let (Some(pa), Some(pb)) = (place.get(&e.from).copied(), place.get(&e.to).copied()) else {
            return Err("a corner of that face is not one its triangles name".into());
        };
        let (qa, qb) = (add(pa, step), add(pb, step));
        raised.push([pa, pb, qb]);
        raised.push([pa, qb, qa]);
    }
    sealed(&raised).map_err(|e| format!("that face is no surface a solid can rise from: {e}"))?;
    // Wound one way the walls face in: the solid is then the mirror of itself, so it turns out.
    if volume(&raised) < 0.0 {
        for t in &mut raised {
            t.swap(1, 2);
        }
    }
    out.extend(raised);
    Ok(out)
}

/// `out`, if every corner of it stands where the model holds one.
fn held(out: Vec<[P3; 3]>) -> Result<Vec<[P3; 3]>, String> {
    if out.iter().flatten().flatten().any(|v| !v.is_finite() || v.abs() > MAX_COORD) {
        return Err(format!("that would put a corner past {MAX_COORD} mm"));
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

    /// A square `side` mm a way at height `z`, its two triangles turned to face up.
    fn pane(side: f64, z: f64) -> Vec<[P3; 3]> {
        let p = [[0.0, 0.0, z], [side, 0.0, z], [side, side, z], [0.0, side, z]];
        vec![[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
    }

    #[test]
    fn a_sheet_pulled_up_becomes_a_solid_and_pulled_down_the_same_one_the_other_way_out() {
        let sheet = pane(1000.0, 0.0);
        let face = face_at(&sheet, [500.0, 500.0, 0.0]).expect("the sheet's one face");
        assert_eq!((face.tris.len(), face.normal), (2, [0.0, 0.0, 1.0]));
        let up = push(&sheet, &face, 500.0).expect("it rises");
        assert_eq!(up.len(), 12, "the face as the near end, its copy at the far one, and two walls a side");
        assert!(sealed(&up).is_ok(), "a solid where there was a sheet");
        assert_eq!(volume(&up), 5e8, "a metre square, half a metre high");
        // The sheet itself stayed, and now faces out of the solid, down.
        let near = face_at(&up, [500.0, 500.0, 0.0]).expect("the sheet is still its near end");
        assert_eq!(near.normal, [0.0, 0.0, -1.0]);
        // Pushed the other way it is the same solid, turned out: the sheet is its top.
        let face = face_at(&sheet, [500.0, 500.0, 0.0]).expect("the face");
        let down = push(&sheet, &face, -500.0).expect("it sinks");
        assert!(sealed(&down).is_ok(), "a solid too");
        assert_eq!(volume(&down), 5e8, "the same solid, hanging under the sheet");
        assert_eq!(down.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max), 0.0, "its top is the sheet");
    }

    #[test]
    fn a_sheet_pushed_leaves_the_other_sheets_of_the_mesh_where_they_are() {
        let mut two = pane(1000.0, 0.0);
        let above = pane(500.0, 2000.0);
        two.extend(above.iter().copied());
        let face = face_at(&two, [500.0, 500.0, 0.0]).expect("the sheet at the bottom");
        assert_eq!(face.tris, [0, 1], "the lower one alone, not the one above it");
        let up = push(&two, &face, 500.0).expect("it rises");
        assert_eq!(up.len(), 14, "the lower sheet's twelve and the other one's two");
        assert_eq!(&up[..2], &above[..], "the sheet above stands where it was, wound as it was");
    }

    #[test]
    fn a_sheet_with_another_triangle_on_part_of_its_edge_moves_instead_of_rising() {
        // A sheet A whose bottom edge has two triangles of its plane on parts of it: A alone is no sheet, whatever the exact
        // edges say, and pushing it moves its corners as any face of a surface does.
        let a = [[0.0, 0.0, 0.0], [4000.0, 0.0, 0.0], [2000.0, 2000.0, 0.0]];
        let tris = vec![
            a,
            [[0.0, 0.0, 0.0], [2000.0, 0.0, 0.0], [0.0, -1000.0, 0.0]],
            [[2000.0, 0.0, 0.0], [4000.0, 0.0, 0.0], [4000.0, -1000.0, 0.0]],
        ];
        let face = face_at(&tris, [2000.0, 500.0, 0.0]).expect("the sheet's face");
        assert_eq!(face.tris, [0], "the triangle alone");
        let pushed = push(&tris, &face, 500.0).expect("it moves");
        assert_eq!(pushed.len(), 3, "nothing rises: the face moves and the two triangles beside it stretch");
        assert_eq!(pushed.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max), 500.0, "the face went up");
    }

    #[test]
    fn a_face_no_solid_can_rise_from_is_refused() {
        // Three triangles over one edge, each walking it the same way: a face a push cannot wall in, refused with why.
        let spine: [[f64; 3]; 2] = [[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0]];
        let tris = vec![
            [spine[0], spine[1], [500.0, 500.0, 0.0]],
            [spine[0], spine[1], [500.0, 1000.0, 0.0]],
            [spine[0], spine[1], [500.0, 1500.0, 0.0]],
        ];
        let face = face_at(&tris, [500.0, 800.0, 0.0]).expect("the face they make");
        assert_eq!(face.tris.len(), 3, "one flat face of three triangles");
        let e = push(&tris, &face, 500.0).expect_err("no solid rises from it");
        assert!(e.contains("no surface a solid can rise from"), "{e}");
    }

    #[test]
    fn a_mesh_is_measured_once_as_a_solid_a_sheet_or_a_broken_surface() {
        let box_ = box_tris(1000.0);
        let m = measure(&box_).expect("measured");
        assert_eq!((m.kind, m.triangles), (Kind::Solid, 12));
        assert!((m.volume - 1e9).abs() < 1.0 && (m.area - 6e6).abs() < 1.0, "{m:?}");
        // A sheet encloses nothing; a box one triangle short is a sheet too; one turned round is no surface.
        assert_eq!(measure(&pane(1000.0, 0.0)).expect("a sheet").kind, Kind::Sheet);
        let mut hole = box_.clone();
        hole.pop();
        assert_eq!(measure(&hole).expect("open").kind, Kind::Sheet);
        let mut turned = box_.clone();
        turned[2].swap(1, 2);
        assert_eq!(measure(&turned).expect("turned").kind, Kind::Reversed);
        // Three triangles on one edge have no surface at all.
        let fan = [
            [[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [500.0, 500.0, 0.0]],
            [[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [500.0, 1000.0, 0.0]],
            [[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [500.0, 1500.0, 0.0]],
        ];
        assert_eq!(measure(&fan).expect("a fan").kind, Kind::Crowded(3));
        assert!(sealed(&fan).unwrap_err().contains("shared by 3 triangles"));
        // A form made by a command is a solid, turned out: an inside-out box comes back positive.
        let inside_out: Vec<[P3; 3]> = box_.iter().map(|[a, b, c]| [*a, *c, *b]).collect();
        let made = solid(inside_out).expect("turned out");
        assert!((made.measure.volume - 1e9).abs() < 1.0, "{:?}", made.measure);
        assert!(solid(pane(1000.0, 0.0)).err().unwrap_or_default().contains("open surface"));
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
