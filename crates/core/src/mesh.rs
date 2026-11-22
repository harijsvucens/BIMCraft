//! Direct modeling: the flat faces a form's triangles make, and pushing one along its normal.
//!
//! A face is grouped here ([`facets`]): triangles that share a welded corner's edge and lie in one plane are one face,
//! however they are wound; triangles that turn into one another less than [`SMOOTH`] degrees are one curved surface, which
//! has no single plane to push. The scene draws by the same grouping, so a face a pick can select is the face a push moves.

use std::collections::{BTreeMap, HashMap, HashSet};

use crate::geom::P2;
use crate::model::MAX_COORD;
use crate::vec3::{P3, add, cross, dot, mul, sub, unit};

pub mod identity;

/// How far off a face's plane a point may lie and still be on it, and how far outside it: the weld's micron, with room
/// for a pick's rounding.
pub const ON_FACE: f64 = 1e-3;

/// A flat face of a mesh: the triangles it is made of, the way it faces, unit out of the solid as its triangles wound it, and
/// how much surface it has.
#[derive(Debug, Clone)]
pub struct TopoFace {
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
pub fn face_at(tris: &[[P3; 3]], at: P3) -> Result<TopoFace, String> {
    let (ids, faces) = faces_and_ids(tris, None);
    face_from(tris, at, &ids, &faces)
}

/// Every flat face of a mesh, by id: what `mesh_check` lists and what a push can name instead of a point.
pub fn faces(tris: &[[P3; 3]]) -> Vec<TopoFace> {
    faces_and_ids(tris, None).1
}

/// The faces and their per-triangle labels, keyed from one weld. Grouping triangle indices once is
/// linear in the triangle count, where the old per-face filter was O(triangles × faces).
fn faces_and_ids(tris: &[[P3; 3]], welded: Option<&[[Corner; 3]]>) -> (Vec<Option<usize>>, Vec<TopoFace>) {
    let (_, ids) = facets_with(tris, welded);
    let mut by_id: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (i, id) in ids.iter().enumerate() {
        if let Some(id) = *id {
            by_id.entry(id).or_default().push(i);
        }
    }
    let faces = by_id
        .into_iter()
        .map(|(id, group)| TopoFace { id, normal: normal(tris[group[0]]), area: area(tris, &group), tris: group })
        .collect();
    (ids, faces)
}

/// The flat face `id` of a mesh: the one [`faces`] lists and `mesh_check` names.
pub fn face_of(tris: &[[P3; 3]], id: usize) -> Result<TopoFace, String> {
    faces(tris).into_iter().find(|f| f.id == id).ok_or_else(|| format!("this mesh has no flat face {id}: `mesh_check` lists them"))
}

/// One directed side of a triangle as `edges` groups it: its welded ends, the surface it belongs to, and where it stands.
type Side = ([i64; 3], [i64; 3], Option<(bool, usize)>, [P3; 2]);

/// The edges a drawing shows on these triangles: a side walked once (a boundary), and one whose two sides lie in
/// different surfaces — the surfaces [`facets`] makes, which the picture's own edges use, so what a preview shows is
/// what the picture draws. A bend the surfaces merge along draws no edge, whatever a normal's dot product would say.
pub fn edges(tris: &[[P3; 3]]) -> Vec<[P3; 2]> {
    let (curves, faces) = facets(tris);
    let surface = |i: usize| -> Option<(bool, usize)> {
        if let Some(f) = faces.get(i).copied().flatten() { Some((false, f)) } else { curves.get(i).copied().flatten().map(|c| (true, c)) }
    };
    let key = |p: P3| p.map(|x| (x * 1e3).round() as i64);
    let mut sides: Vec<Side> = Vec::with_capacity(tris.len() * 3);
    for (i, t) in tris.iter().enumerate() {
        let of = surface(i);
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let (qa, qb) = (key(a), key(b));
            if qa == qb {
                continue;
            }
            sides.push((qa.min(qb), qa.max(qb), of, [a, b]));
        }
    }
    sides.sort_unstable_by_key(|s| (s.0, s.1));
    let mut out = Vec::new();
    for on in sides.chunk_by(|a, b| (a.0, a.1) == (b.0, b.1)) {
        let one_surface = on.len() == 2 && on[0].2.is_some() && on[0].2 == on[1].2;
        if !one_surface && let Some(s) = on.first() {
            out.push(s.3);
        }
    }
    out
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
/// solidity and its numbers are worked out, so `mesh`, a push, `mesh_check` and a form's checks all read the same answer.
pub fn measure(tris: &[[P3; 3]]) -> Result<Measure, String> {
    measure_keys(tris, &weld(tris))
}

/// `measure` with the weld already done: a push's source is welded once and re-measured on every clamp step without welding
/// again. Corner topology comes from `keys`; only the area and the volume come from the (possibly moved) corners.
fn measure_keys(tris: &[[P3; 3]], keys: &[[Corner; 3]]) -> Result<Measure, String> {
    if tris.iter().flatten().flatten().any(|x| !x.is_finite() || x.abs() > MAX_COORD) {
        return Err("it reaches past where the model may go".into());
    }
    if keys.len() != tris.len() {
        return Err("a mesh's corners and its triangles disagree".into());
    }
    let mut pairs: HashMap<Edge, usize> = HashMap::new();
    let mut ways: HashMap<(Corner, Corner), usize> = HashMap::new();
    for (t, k) in tris.iter().zip(keys) {
        // Three distinct corners in one line enclose no surface. A split or an inset can leave such a sliver; refuse it
        // rather than let a zero-area face be counted as part of a closed solid. The threshold is only large enough for
        // float noise at this triangle's scale, so a thin but real triangle still passes. This is a stricter read too:
        // `Family::check` measures a mesh on open, so an imported degenerate triangle is reported then, with what to fix.
        let (ab, bc, ca) = (sub(t[1], t[0]), sub(t[2], t[1]), sub(t[0], t[2]));
        let twice = cross(ab, bc);
        let longest = dot(ab, ab).max(dot(bc, bc)).max(dot(ca, ca)).sqrt();
        if dot(twice, twice).sqrt() <= 1e-12 * longest.powi(2) {
            return Err("a triangle of it has three corners in a line, so it has no area".into());
        }
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

/// Whether the corners `keys` name are still distinct after a move. A push reuses the source's weld, so two corners that
/// land within the weld's reach of one another must be caught here; `measure` would otherwise fold them into one. A cell
/// grid of the rounded positions does it without re-welding the mesh on every clamp step.
fn corners_hold(tris: &[[P3; 3]], keys: &[[Corner; 3]]) -> bool {
    let mut place: HashMap<Corner, P3> = HashMap::new();
    for (t, k) in tris.iter().zip(keys) {
        for j in 0..3 {
            place.entry(k[j]).or_insert(t[j]);
        }
    }
    let mut cells: HashMap<Corner, Vec<Corner>> = HashMap::new();
    for (&corner, &p) in &place {
        cells.entry(p.map(|x| (x * 1e3).round() as i64)).or_default().push(corner);
    }
    for (&corner, &p) in &place {
        let q = p.map(|x| (x * 1e3).round() as i64);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let next = [q[0] + dx, q[1] + dy, q[2] + dz];
                    if let Some(other) = cells.get(&next) {
                        for &c in other {
                            if c != corner && sub(p, place[&c]).iter().all(|v| v.abs() <= 1.5e-3) {
                                return false;
                            }
                        }
                    }
                }
            }
        }
    }
    true
}

/// A mesh's triangles as the closed solid a form made by a command must be: measured once, held to closing and to a volume,
/// and turned out. A sheet is refused here — `mesh` makes solids — though a mesh that comes from a file may be one.
pub(crate) struct Solid {
    pub tris: Vec<[P3; 3]>,
    pub measure: Measure,
}

pub(crate) fn solid(mut tris: Vec<[P3; 3]>) -> Result<Solid, String> {
    if let Some(why) = crate::family::over_triangles("it", tris.len()) {
        return Err(why);
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
    facets_with(tris, None)
}

/// [`facets`] with a weld the caller already made: one prepared mesh can answer a face question and a
/// push without welding the same triangles again.
fn facets_with(tris: &[[P3; 3]], welded: Option<&[[Corner; 3]]>) -> (Vec<Option<usize>>, Vec<Option<usize>>) {
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
    let owned = welded.is_none().then(|| weld(tris));
    let corners: &[[Corner; 3]] = welded.or(owned.as_deref()).unwrap_or(&[]);
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
    // Face ids are keyed to the face's least welded corner, not to a triangle's place, so they
    // survive a gesture that rewrites the triangles. Curved surfaces keep their root numbering.
    (identity::stable_ids(corners, &curves), identity::stable_ids(corners, &faces))
}

#[cfg(test)]
std::thread_local! {
    /// How many welds this test thread has asked for: `push_clamped` must weld its source once, not once per clamp step.
    static WELD_CALLS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// The corner each triangle corner of a mesh welds to, by a micron's grid: the same point written twice, or twice within
/// the weld, gets one. The scene groups a mesh's faces by these corners, so what a pick selects is what a push moves.
pub(crate) fn weld(tris: &[[P3; 3]]) -> Vec<[Corner; 3]> {
    #[cfg(test)]
    WELD_CALLS.with(|n| n.set(n.get() + 1));
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

/// The boundary ring of each of a mesh's welded facets, what the kernel bridge sews its sheets from: the triangles
/// [`facets`] group into one plane, their welded edges walked once chained into the loop they close, in the facet's own
/// winding. A facet with a hole in it — two rings — is refused: the bridge sews whole sheets, and a hole is what the
/// mesh path keeps.
#[cfg(feature = "kernel-solidcore")]
pub(crate) fn facet_rings(tris: &[[P3; 3]]) -> Result<Vec<Vec<P3>>, String> {
    let (_, labels) = facets(tris);
    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut by_label: HashMap<usize, usize> = HashMap::new();
    for (i, label) in labels.iter().enumerate() {
        let Some(label) = label else { continue };
        let slot = *by_label.entry(*label).or_insert_with(|| {
            groups.push(Vec::new());
            groups.len() - 1
        });
        groups[slot].push(i);
    }
    let mut rings = Vec::new();
    for group in &groups {
        // Every directed side of the facet, grouped by unordered welded corner pair: a pair walked once is on the
        // boundary, a pair walked both ways is interior. The first walk's direction is kept, so the ring follows the
        // facet's winding.
        let mut pairs: HashMap<Edge, (u32, Corner, Corner, P3)> = HashMap::new();
        for &ti in group {
            let t = tris.get(ti).ok_or_else(|| format!("facet names triangle {ti}, past the mesh"))?;
            for j in 0..3 {
                let (a, b) = (t[j], t[(j + 1) % 3]);
                let (ka, kb) = (a.map(|x| (x * 1e3).round() as i64), b.map(|x| (x * 1e3).round() as i64));
                if ka == kb {
                    continue; // a sliver edge shorter than a micron
                }
                let e = pairs.entry(edge(ka, kb)).or_insert((0, ka, kb, a));
                e.0 += 1;
            }
        }
        // The boundary, directed as the facet's triangles walk it: an edge's start corner names the next step.
        let mut starts: HashMap<Corner, (Corner, P3)> = HashMap::new();
        for (_, (count, ka, kb, at)) in pairs {
            if count == 1 {
                starts.insert(ka, (kb, at));
            }
        }
        let Some(&first) = starts.keys().next() else {
            continue; // every edge cancelled: the facet closes in itself, impossible for a real face
        };
        let mut ring: Vec<P3> = Vec::new();
        let mut current = first;
        loop {
            let Some((end, at)) = starts.remove(&current) else {
                return Err("a facet's boundary does not close into one ring".into());
            };
            ring.push(at);
            if end == first && ring.len() >= 3 {
                break;
            }
            current = end;
        }
        if !starts.is_empty() {
            return Err("a facet's boundary does not close into one ring".into());
        }
        rings.push(ring);
    }
    Ok(rings)
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
fn alone(tris: &[[P3; 3]], keys: &[[Corner; 3]], group: &[usize], rim: &[OnRim], face: &TopoFace) -> bool {
    if group.len() == tris.len() {
        return true;
    }
    let inside: HashSet<usize> = group.iter().copied().collect();
    let boundary: HashSet<Edge> = rim.iter().map(|e| e.edge).collect();
    let others: Vec<usize> = (0..tris.len()).filter(|i| !inside.contains(i)).collect();
    // A rim edge with another triangle walking it whole makes the face part of a surface, and no corner need be asked: that is
    // every edge of a solid, so a push on one pays nothing for this question.
    if others.iter().any(|i| keys.get(*i).is_some_and(|k| (0..3).any(|j| boundary.contains(&edge(k[j], k[(j + 1) % 3]))))) {
        return false;
    }
    let mut place: HashMap<Corner, P3> = HashMap::new();
    for i in group {
        let (Some(t), Some(k)) = (tris.get(*i), keys.get(*i)) else { continue };
        for j in 0..3 {
            place.entry(k[j]).or_insert(t[j]);
        }
    }
    let start = group.first().and_then(|i| tris.get(*i)).map_or([0.0; 3], |t| t[0]);
    // A corner of another triangle standing part way along a rim edge is a T-junction, so the face is part of a surface.
    // Index the in-plane corners in a grid and walk each rim edge through it: linear in the mesh and the rim, with no
    // pair-count cut-off that could silently call a sheet a surface on a large import. One cell is at least a weld's
    // reach, so a corner within `ON_FACE` of an edge is bucketed next to the edge's own cell.
    let span = rim
        .iter()
        .filter_map(|e| place.get(&e.from).zip(place.get(&e.to)))
        .map(|(a, b)| sub(*b, *a).into_iter().map(f64::abs).fold(0.0, f64::max))
        .fold(0.0, f64::max);
    let cell_size = (span / 256.0).max(0.1);
    let cell = |p: P3| -> [i64; 3] { std::array::from_fn(|i| (p[i] / cell_size).floor() as i64) };
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
    let mut corners: Vec<P3> = Vec::new();
    for i in &others {
        for p in tris.get(*i).copied().unwrap_or_default() {
            // Only a corner in the face's own plane can stand on its boundary.
            if dot(face.normal, sub(p, start)).abs() > ON_FACE {
                continue;
            }
            let at = corners.len();
            corners.push(p);
            let c = cell(p);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        grid.entry([c[0] + dx, c[1] + dy, c[2] + dz]).or_default().push(at);
                    }
                }
            }
        }
    }
    let mut seen = vec![0u32; corners.len()];
    let mut stamp = 1u32;
    for e in rim {
        let (Some(a), Some(b)) = (place.get(&e.from).copied(), place.get(&e.to).copied()) else { continue };
        let along = sub(b, a);
        let steps = (dot(along, along).sqrt() / cell_size).ceil() as usize + 1;
        for s in 0..steps {
            let f = s as f64 / steps.saturating_sub(1).max(1) as f64;
            for k in grid.get(&cell(add(a, mul(along, f)))).into_iter().flatten() {
                if seen[*k] == stamp {
                    continue;
                }
                seen[*k] = stamp;
                if stands_on(Some(a), Some(b), corners[*k]) {
                    return false;
                }
            }
        }
        stamp = stamp.wrapping_add(1);
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

/// `tris` with the triangles of `face` taken away: a hole opens where it stood, so a solid becomes an open sheet. A mesh's
/// one and only face cannot be erased — a form needs a triangle. What opened is measured by the caller.
pub fn erase(tris: &[[P3; 3]], face: &TopoFace) -> Result<Vec<[P3; 3]>, String> {
    if face.tris.iter().any(|i| *i >= tris.len()) {
        return Err("that face is no face of these triangles".into());
    }
    let gone: HashSet<usize> = face.tris.iter().copied().collect();
    let out: Vec<[P3; 3]> = tris.iter().enumerate().filter(|(i, _)| !gone.contains(i)).map(|(_, t)| *t).collect();
    if out.is_empty() {
        return Err("that is the mesh's only face: erasing it leaves no mesh".into());
    }
    Ok(out)
}

/// How far `at` lies off the segment from `a` to `b`, if it lies within [`ON_FACE`] of it (an end counting as on it).
fn segment_offset(a: P3, b: P3, at: P3) -> Option<f64> {
    let ab = sub(b, a);
    let square = dot(ab, ab);
    if square <= ON_FACE * ON_FACE {
        return None;
    }
    let along = dot(sub(at, a), ab) / square;
    if along < -ON_FACE || along > 1.0 + ON_FACE {
        return None;
    }
    let foot = add(a, mul(ab, along.clamp(0.0, 1.0)));
    let off = sub(at, foot);
    let d = dot(off, off).sqrt();
    (d <= ON_FACE).then_some(d)
}

/// `tris` with the edge holding `at` moved so that `at` stands at `to`: both ends of that edge shift by the same step, and
/// every face they touch must stay flat. A ridge between two slopes moves; a quad pulled by one of its corners warps and
/// is refused. An edge inside one face (a triangulation diagonal) or on a curved surface has no two faces to keep flat
/// and is refused too.
pub fn move_edge(tris: &[[P3; 3]], at: P3, to: P3) -> Result<Vec<[P3; 3]>, String> {
    if !at.iter().chain(to.iter()).all(|v| v.is_finite() && v.abs() <= MAX_COORD) {
        return Err(format!("points to move an edge by are [x, y, z] in mm within {MAX_COORD}"));
    }
    let step = sub(to, at);
    if !step.iter().all(|v| v.is_finite() && v.abs() <= MAX_COORD) {
        return Err(format!("that would move an edge past {MAX_COORD} mm"));
    }
    if step.iter().all(|v| v.abs() <= 1e-9) {
        return Err("a move of nothing moves no edge".into());
    }
    let keys = weld(tris);
    let mut place: HashMap<Corner, P3> = HashMap::new();
    for (t, k) in tris.iter().zip(&keys) {
        for j in 0..3 {
            place.entry(k[j]).or_insert(t[j]);
        }
    }
    // The edge `at` lies along, with the triangles that walk it.
    let mut found: Vec<(Edge, Corner, Corner, Vec<usize>, f64)> = Vec::new();
    for (i, k) in keys.iter().enumerate() {
        for j in 0..3 {
            let (a, b) = (k[j], k[(j + 1) % 3]);
            if a == b {
                continue;
            }
            let (Some(pa), Some(pb)) = (place.get(&a).copied(), place.get(&b).copied()) else { continue };
            let Some(off) = segment_offset(pa, pb, at) else { continue };
            let e = edge(a, b);
            if let Some(slot) = found.iter_mut().find(|(f, ..)| *f == e) {
                slot.3.push(i);
                slot.4 = slot.4.min(off);
            } else {
                found.push((e, a, b, vec![i], off));
            }
        }
    }
    found.sort_by(|a, b| a.4.total_cmp(&b.4));
    let Some((edge, a, b, walks, off)) = found.first() else {
        return Err("no edge of the mesh holds that point: pick one along an edge, not on a face or past it".into());
    };
    if found.get(1).is_some_and(|other| other.4 <= *off + ON_FACE && other.0 != *edge) {
        return Err("that point is a corner where edges meet: pick one along an edge".into());
    }
    if walks.len() != 2 {
        return Err(format!("that edge is walked by {} triangles, not the two faces a move needs", walks.len()));
    }
    let (_, ids) = facets(tris);
    let sides: Vec<Option<usize>> = walks.iter().map(|i| ids.get(*i).copied().flatten()).collect();
    if sides.iter().any(Option::is_none) {
        return Err("that edge is on a curved surface, which has no flat face to keep: pick a straight face's edge".into());
    }
    if sides[0] == sides[1] {
        return Err("that edge is inside one face, a triangulation diagonal: pick one on its boundary".into());
    }
    let moving: HashSet<Corner> = [*a, *b].into_iter().collect();
    let out: Vec<[P3; 3]> =
        tris.iter().zip(&keys).map(|(t, k)| std::array::from_fn(|j| if moving.contains(&k[j]) { add(t[j], step) } else { t[j] })).collect();
    measure(&out)?;
    if measure(tris).is_ok_and(|m| m.is_solid()) && volume(&out) <= 1.0 {
        return Err("that would turn the solid inside out: move the edge less far".into());
    }
    // Every face the moved corners touch must still lie in one plane — grouped as they were before, so a warp that would
    // split a face in two cannot pass by becoming two planar groups.
    let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for (i, id) in ids.iter().enumerate() {
        if let Some(id) = id {
            groups.entry(*id).or_default().push(i);
        }
    }
    for group in groups.values() {
        let Some(first) = group.first().and_then(|i| out.get(*i)) else { continue };
        let n = normal(*first);
        if dot(n, n) <= 0.5 {
            return Err("that move would flatten a face of the mesh".into());
        }
        if group.iter().any(|i| out.get(*i).is_some_and(|t| t.iter().any(|p| dot(n, sub(*p, first[0])).abs() > ON_FACE))) {
            return Err("that move would warp a face of the mesh: move it another way, or triangulate the face first".into());
        }
    }
    Ok(out)
}

/// A mesh's weld, measurement and flat faces, made in one pass so a gesture never welds the same
/// triangles again — a face question, a push and a clamp's bisection all read this one value.
pub struct Prepared {
    keys: Vec<[Corner; 3]>,
    measure: Measure,
    faces: Vec<TopoFace>,
    ids: Vec<Option<usize>>,
}

/// Prepare a mesh once: weld it, measure it and group its faces from that same weld.
pub fn prepare(tris: &[[P3; 3]]) -> Result<Prepared, String> {
    let keys = weld(tris);
    let measure = measure_keys(tris, &keys)?;
    let (ids, faces) = faces_and_ids(tris, Some(&keys));
    Ok(Prepared { keys, measure, faces, ids })
}

impl Prepared {
    /// The flat faces of the prepared mesh, keyed as [`faces`] keys them.
    pub fn faces(&self) -> &[TopoFace] {
        &self.faces
    }

    /// The flat face `at` lies on, answered from this weld without touching the mesh again.
    pub fn face_at(&self, tris: &[[P3; 3]], at: P3) -> Result<TopoFace, String> {
        face_from(tris, at, &self.ids, &self.faces)
    }

    /// The flat face `id` names, answered from this weld.
    pub fn face_of(&self, id: usize) -> Result<TopoFace, String> {
        self.faces.iter().find(|f| f.id == id).cloned().ok_or_else(|| format!("this mesh has no flat face {id}: `mesh_check` lists them"))
    }

    /// What the prepared mesh measures: the weld's own answer, no re-measure needed.
    pub fn measure(&self) -> Measure {
        self.measure
    }
}

/// The one point→face question, shared by [`face_at`] and [`Prepared::face_at`].
fn face_from(tris: &[[P3; 3]], at: P3, ids: &[Option<usize>], faces: &[TopoFace]) -> Result<TopoFace, String> {
    if !at.iter().all(|v| v.is_finite() && v.abs() <= MAX_COORD) {
        return Err(format!("a point to push is [x, y, z] in mm within {MAX_COORD}"));
    }
    let mut under: Option<(f64, usize)> = None;
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
            under = Some((off, i));
        }
    }
    let Some((_, i)) = under else { return Err("no flat face of the mesh holds that point".into()) };
    let Some(id) = ids.get(i).copied().flatten() else {
        return Err("that point is on a curved side, which has no plane of its own to push: pick a flat face".into());
    };
    faces.iter().find(|f| f.id == id).cloned().ok_or_else(|| "no flat face holds that point".into())
}

/// The triangles a push makes and the corners they weld to: the clamp measures the result from these keys, without welding
/// the moved mesh again. A sheet that rises keeps the same corner topology in its kept part and names the raised corners.
struct Pushed {
    tris: Vec<[P3; 3]>,
    keys: Vec<[Corner; 3]>,
}

/// A new corner for the far end of an extruded face. Weld corners are rounded model coordinates within `MAX_COORD`; a
/// negative sentinel cannot be one, and the counter keeps every copy distinct from the others.
fn fresh_corner(c: Corner, fresh: &mut HashMap<Corner, Corner>, n: &mut i64) -> Corner {
    if let Some(v) = fresh.get(&c) {
        return *v;
    }
    let v = [i64::MIN + *n, 0, 0];
    *n += 1;
    fresh.insert(c, v);
    v
}

/// `tris` with `face` moved `distance` mm along its normal, out of the solid where that is positive: every corner welded
/// to one of the face's moves with it, so the faces round it stretch as a hand pulling a SketchUp face's does. A face with
/// nothing beside it — a sheet, as an imported mesh may hold — is no surface to stretch: it rises into a solid, keeping
/// itself as the near end. A solid that closes may not be pushed through itself, into an inside-out shell.
pub fn push(tris: &[[P3; 3]], face: &TopoFace, distance: f64) -> Result<Vec<[P3; 3]>, String> {
    let pre = prepare(tris)?;
    Ok(push_prepared(tris, face, distance, &pre)?.tris)
}

fn push_prepared(tris: &[[P3; 3]], face: &TopoFace, distance: f64, pre: &Prepared) -> Result<Pushed, String> {
    if !distance.is_finite() || distance.abs() > MAX_COORD {
        return Err(format!("a distance is a number of mm within {MAX_COORD}, not {distance}"));
    }
    if face.tris.iter().any(|i| *i >= tris.len()) {
        return Err("that face is no face of these triangles".into());
    }
    let keys = &pre.keys;
    let rim = rim(keys, &face.tris);
    if !rim.is_empty() && alone(tris, keys, &face.tris, &rim, face) {
        return extrude(tris, keys, face, &rim, distance);
    }
    let mut moving: HashSet<Corner> = HashSet::new();
    for i in &face.tris {
        if let Some(k) = keys.get(*i) {
            moving.extend(k.iter().copied());
        }
    }
    let step = mul(face.normal, distance);
    let out: Vec<[P3; 3]> =
        tris.iter().zip(keys).map(|(t, k)| std::array::from_fn(|j| if moving.contains(&k[j]) { add(t[j], step) } else { t[j] })).collect();
    let out = held(out)?;
    if let Some(why) = crate::family::over_triangles("that push", out.len()) {
        return Err(why);
    }
    if pre.measure.is_solid() && volume(&out) <= 1.0 {
        return Err("that would turn the solid inside out: push it less far".into());
    }
    Ok(Pushed { tris: out, keys: keys.clone() })
}

/// The largest push toward `by` the mesh allows, and the triangles it makes: [`push`] refuses a solid turned inside out
/// and [`measure`] a fold, so a bisection from zero finds the last distance that keeps both. The source is welded and
/// measured once, before the loop; each step reuses those corners and measures the moved ones from them. A preview draws
/// what this returns and a release commits the distance with it, so what is shown is what is made.
pub fn push_clamped(tris: &[[P3; 3]], face: &TopoFace, by: f64) -> (f64, Vec<[P3; 3]>) {
    let Ok(pre) = prepare(tris) else { return (0.0, Vec::new()) };
    push_clamped_prepared(&pre, tris, face, by)
}

/// [`push_clamped`] with the source already prepared: every clamp step reuses its one weld.
pub fn push_clamped_prepared(pre: &Prepared, tris: &[[P3; 3]], face: &TopoFace, by: f64) -> (f64, Vec<[P3; 3]>) {
    let valid = |d: f64| {
        let out = push_prepared(tris, face, d, pre).ok()?;
        // The source's corners are only authoritative while the moved ones stay apart: near an inside-out push the top
        // can land on the base and weld to it, which a stale key would hide. Reject such a step without re-welding.
        if !corners_hold(&out.tris, &out.keys) {
            return None;
        }
        measure_keys(&out.tris, &out.keys).ok()?;
        Some(out)
    };
    if let Some(out) = valid(by) {
        return (by, out.tris);
    }
    let (mut lo, mut hi) = (0.0, by);
    let mut made = Vec::new();
    for _ in 0..20 {
        let mid = (lo + hi) / 2.0;
        match valid(mid) {
            Some(out) => {
                lo = mid;
                made = out.tris;
            }
            None => hi = mid,
        }
    }
    (lo, made)
}

/// A closed loop on one flat face punched through to the exactly parallel face opposite it: each face becomes
/// a ring and the loop's sides close between them. The loop is in the mesh's own axes, and its corners must
/// lie on `face`. Refuses an open sheet, a loop outside the face, or a solid with no parallel opposite face.
pub fn punch_through(tris: &[[P3; 3]], face: &TopoFace, hole: &[P3]) -> Result<Vec<[P3; 3]>, String> {
    if hole.len() < 3 {
        return Err("a punch needs a closed loop of at least three points".into());
    }
    if hole.iter().flatten().any(|v| !v.is_finite() || v.abs() > crate::model::MAX_COORD) {
        return Err(format!("every loop point must be finite within {} mm", crate::model::MAX_COORD));
    }
    let face_tris: Vec<[P3; 3]> = face.tris.iter().filter_map(|i| tris.get(*i).copied()).collect();
    if face_tris.len() != face.tris.len() || face_tris.is_empty() {
        return Err("the face to punch is not in this mesh".into());
    }
    let normal = face.normal;
    if normal.iter().any(|v| !v.is_finite()) || dot(normal, normal) < 0.5 {
        return Err("only a flat face can be punched".into());
    }
    let origin = face_tris[0][0];
    let off = |p: P3| dot(sub(p, origin), normal);
    if hole.iter().any(|p| off(*p).abs() > ON_FACE * 4.0) {
        return Err("every corner of the loop must lie on the face being punched".into());
    }
    let boundary = edges(&face_tris);
    if boundary.is_empty() {
        return Err("the face has no boundary to punch through".into());
    }
    let u = unit(sub(boundary[0][1], boundary[0][0]));
    if u.iter().any(|v| !v.is_finite()) {
        return Err("the face has no direction to measure its loop in".into());
    }
    let v = cross(normal, u);
    let to2 = |q: P3| {
        let d = sub(q, origin);
        P2(dot(d, u), dot(d, v))
    };
    let to3 = |q: P2| add(origin, add(mul(u, q.0), mul(v, q.1)));
    let loop2: Vec<P2> = hole.iter().map(|q| to2(*q)).collect();
    let segs = |ring: &[P2]| -> Vec<crate::arrangement::Seg> {
        (0..ring.len()).map(|i| crate::arrangement::Seg::new(ring[i], ring[(i + 1) % ring.len()])).collect()
    };
    let loop_segs = segs(&loop2);
    // The nearest face facing the other way *across* the near plane is the one a punch reaches. A soffit
    // on the near side is not: lifting the ring there would leave the original geometry in place. Every
    // corner must be at the same depth, or one constant shift would crack a tilted far face.
    let plane = |f: &TopoFace| -> Option<f64> {
        let mut corners = f.tris.iter().filter_map(|i| tris.get(*i)).flatten().map(|p: &P3| off(*p));
        let first = corners.next()?;
        corners.all(|d| (d - first).abs() <= ON_FACE).then_some(first).filter(|d| *d < -ON_FACE * 4.0)
    };
    let (far, depth) = faces(tris)
        .into_iter()
        .filter(|f| f.id != face.id && dot(f.normal, normal) < -0.999)
        .filter_map(|f| plane(&f).map(|d| (f, d)))
        .max_by(|(_, a), (_, b)| a.total_cmp(b))
        .ok_or_else(|| "the face has no parallel opposite face to punch through".to_string())?;
    let ring_of = |f: &TopoFace| -> Result<Vec<crate::arrangement::Region>, String> {
        let ft: Vec<[P3; 3]> = f.tris.iter().filter_map(|i| tris.get(*i).copied()).collect();
        let side: Vec<crate::arrangement::Seg> = edges(&ft).iter().map(|[a, b]| crate::arrangement::Seg::new(to2(*a), to2(*b))).collect();
        crate::arrangement::resolve(&side, &loop_segs, crate::arrangement::Mode::Split)
    };
    // A region whose outer ring is the loop is the hole; every other region is part of the face left behind.
    let is_hole = |r: &crate::arrangement::Region| {
        r.outer.len() == loop2.len() && r.outer.iter().all(|p| loop2.iter().any(|q| p.dist(*q) <= crate::arrangement::EPS))
    };
    let mut out: Vec<[P3; 3]> =
        tris.iter().enumerate().filter(|(i, _)| !face.tris.contains(i) && !far.tris.contains(i)).map(|(_, t)| *t).collect();
    let mut count = 0usize;
    for f in [face, &far] {
        let regions = ring_of(f)?;
        if !regions.iter().any(is_hole) {
            return Err("the loop does not lie inside the parallel face either".into());
        }
        let kept: Vec<_> = regions.iter().filter(|r| !is_hole(r)).collect();
        if kept.is_empty() {
            return Err("the loop covers the whole face; a punch must leave a ring around it".into());
        }
        // The triangulation is counter-clockwise in the near face's axes, so it faces the near normal.
        // The far face faces the other way: its ring must be wound back the other way to face out of the solid.
        let flip = f.id == far.id;
        for r in kept {
            let t = crate::triangulation::constrained(&r.outer, &r.holes, &[], &[])
                .map_err(|e| format!("the punched ring cannot be triangulated: {e}"))?;
            for [a, b, c] in t.triangles {
                let (b, c) = if flip { (c, b) } else { (b, c) };
                let lift = |q: P2| {
                    let p = to3(q);
                    if flip { add(p, mul(normal, depth)) } else { p }
                };
                out.push([lift(t.points[a]), lift(t.points[b]), lift(t.points[c])]);
                count += 1;
            }
        }
    }
    // A counter-clockwise loop leaves the wall's outward normal toward the hole, into the loop's empty middle.
    let mut ring = loop2.clone();
    let twice = (0..ring.len()).map(|i| ring[i].cross(ring[(i + 1) % ring.len()])).sum::<f64>();
    if twice < 0.0 {
        ring.reverse();
    }
    for i in 0..ring.len() {
        let j = (i + 1) % ring.len();
        let (na, nb) = (to3(ring[i]), to3(ring[j]));
        let (fa, fb) = (
            [na[0] + normal[0] * depth, na[1] + normal[1] * depth, na[2] + normal[2] * depth],
            [nb[0] + normal[0] * depth, nb[1] + normal[1] * depth, nb[2] + normal[2] * depth],
        );
        out.push([na, nb, fb]);
        out.push([na, fb, fa]);
        count += 2;
    }
    if count == 0 {
        return Err("the punch made no faces".into());
    }
    Ok(out)
}

/// `tris` with `face` raised `distance` mm into a solid and the face left as its near end, its copy standing at the far one,
/// with a wall of two triangles along each of `rim`'s edges: what a sheet becomes when a hand pulls it up, off either side.
fn extrude(tris: &[[P3; 3]], keys: &[[Corner; 3]], face: &TopoFace, rim: &[OnRim], distance: f64) -> Result<Pushed, String> {
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
    let kept: Vec<usize> = (0..tris.len()).filter(|i| !inside.contains(i)).collect();
    let mut out: Vec<[P3; 3]> = kept.iter().filter_map(|i| tris.get(*i).copied()).collect();
    let mut out_keys: Vec<[Corner; 3]> = kept.iter().filter_map(|i| keys.get(*i).copied()).collect();
    if let Some(why) = crate::family::over_triangles("that erase", out.len().saturating_add(2 * face.tris.len() + 2 * rim.len())) {
        return Err(why);
    }
    let mut raised: Vec<[P3; 3]> = Vec::with_capacity(2 * face.tris.len() + 2 * rim.len());
    let mut raised_keys: Vec<[Corner; 3]> = Vec::with_capacity(2 * face.tris.len() + 2 * rim.len());
    let mut fresh: HashMap<Corner, Corner> = HashMap::new();
    let mut next = 1i64;
    for i in &face.tris {
        let (Some(t), Some(k)) = (tris.get(*i), keys.get(*i)) else { continue };
        // The face turned round is the near end, facing out of what rises; its copy is the far end.
        raised.push([t[2], t[1], t[0]]);
        raised_keys.push([k[2], k[1], k[0]]);
        raised.push(t.map(|p| add(p, step)));
        raised_keys.push(k.map(|c| fresh_corner(c, &mut fresh, &mut next)));
    }
    for e in rim {
        let (Some(pa), Some(pb)) = (place.get(&e.from).copied(), place.get(&e.to).copied()) else {
            return Err("a corner of that face is not one its triangles name".into());
        };
        let qa = fresh_corner(e.from, &mut fresh, &mut next);
        let qb = fresh_corner(e.to, &mut fresh, &mut next);
        raised.push([pa, pb, add(pb, step)]);
        raised_keys.push([e.from, e.to, qb]);
        raised.push([pa, add(pb, step), add(pa, step)]);
        raised_keys.push([e.from, qb, qa]);
    }
    let risen = measure_keys(&raised, &raised_keys).map_err(|e| format!("that face is no surface a solid can rise from: {e}"))?;
    if !risen.is_solid() {
        return Err(format!("that face is no surface a solid can rise from: {}", risen.kind.word()));
    }
    // Wound one way the walls face in: the solid is then the mirror of itself, so it turns out.
    if volume(&raised) < 0.0 {
        for t in &mut raised {
            t.swap(1, 2);
        }
        for k in &mut raised_keys {
            k.swap(1, 2);
        }
    }
    out.extend(raised);
    out_keys.extend(raised_keys);
    // A huge step can put a far corner on top of a near one. The raised keys were built for distinct copies, so catch
    // a collision here rather than let a re-measure of the saved mesh fold it later.
    if !corners_hold(&out, &out_keys) {
        return Err("that face would land on itself: push it less far".into());
    }
    Ok(Pushed { tris: out, keys: out_keys })
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
/// the axis its normal leans on most, so its area does not collapse, and each side read as a distance in mm. The one
/// rule for "is this point on this triangle's surface": the pick, the kernel bridge and the drag overlay all ask it.
pub fn holds([a, b, c]: [P3; 3], n: P3, at: P3) -> bool {
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

/// Whether `at` lies on triangle `t`: in its plane (within [`ON_FACE`]) and on its surface. `n` is unit.
/// The one plane-plus-surface question a pick, the kernel bridge and the drag overlay all ask.
pub fn holds_at(t: [P3; 3], n: P3, at: P3) -> bool {
    dot(n, sub(at, t[0])).abs() <= ON_FACE && holds(t, n, at)
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

    /// A loop drawn on one face, punched to the exactly parallel opposite face, opens a hole. The volume
    /// must lose exactly the prism, and the shell must stay closed: the near and far faces become rings
    /// and the loop's sides close between them.
    #[test]
    fn a_loop_on_a_boxs_top_opens_a_hole_through_its_bottom() {
        let tris = box_tris(1000.0);
        let face = face_at(&tris, [500.0, 500.0, 1000.0]).expect("the top holds the point");
        let hole = [[250.0, 250.0, 1000.0], [750.0, 250.0, 1000.0], [750.0, 750.0, 1000.0], [250.0, 750.0, 1000.0]];
        let punched = punch_through(&tris, &face, &hole).expect("the box has a parallel bottom");
        assert!(measure(&punched).expect("the result measures").is_solid(), "the shell stays closed");
        assert_eq!(volume(&punched), 1e9 - 0.25e9, "a 500 mm square hole through a 1000 mm cube");
        let top = face_at(&punched, [100.0, 100.0, 1000.0]).expect("the top outside the hole remains");
        assert_eq!(top.normal, [0.0, 0.0, 1.0]);
        assert!(face_at(&punched, [500.0, 500.0, 1000.0]).is_err(), "nothing stands in the hole's middle");
    }

    #[test]
    fn a_loop_covering_the_whole_face_is_refused() {
        let tris = box_tris(1000.0);
        let face = face_at(&tris, [500.0, 500.0, 1000.0]).expect("the top holds the point");
        let whole = [[0.0, 0.0, 1000.0], [1000.0, 0.0, 1000.0], [1000.0, 1000.0, 1000.0], [0.0, 1000.0, 1000.0]];
        let e = punch_through(&tris, &face, &whole).expect_err("there is no ring left to punch");
        assert!(e.contains("covers the whole face"), "{e}");
    }

    #[test]
    fn punch_through_refuses_a_loop_that_cannot_close_against_a_parallel_face() {
        let tris = box_tris(1000.0);
        let face = face_at(&tris, [500.0, 500.0, 1000.0]).expect("the top holds the point");
        let bad = [[250.0, 250.0, 1000.0], [750.0, 250.0, 1000.0], [f64::NAN, 750.0, 1000.0]];
        assert!(punch_through(&tris, &face, &bad).is_err(), "a non-finite loop is refused");
        let sheet = vec![tris[2], tris[3]];
        let sheet_face = face_at(&sheet, [500.0, 500.0, 1000.0]).expect("the sheet holds the point");
        let hole = [[250.0, 250.0, 1000.0], [750.0, 250.0, 1000.0], [750.0, 750.0, 1000.0], [250.0, 750.0, 1000.0]];
        assert!(punch_through(&sheet, &sheet_face, &hole).is_err(), "an open sheet has no opposite face");
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

    #[test]
    fn a_prisms_strips_are_one_surface_so_its_sides_draw_no_edges_and_its_rims_do() {
        // The 24-sided prism above: the picture draws its rims and nothing down a side, so a preview of it must not
        // either. The preview's old rule (a normal's dot short of 0.999, a 2.6-degree turn) drew a strip edge at every
        // 15-degree bend, which the committed picture never did.
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
        let drawn = edges(&tris);
        assert_eq!(drawn.len(), 2 * n, "each ring's rim, and nothing else");
        assert!(drawn.iter().all(|[a, b]| (a[2] - b[2]).abs() < 1e-9), "no edge runs down a side");
    }

    #[test]
    fn a_bend_the_surfaces_merge_along_draws_no_edge_between_the_two() {
        // Two triangles meeting at 5.7 degrees, under the 20 the surfaces merge at: one surface, so the edge they share
        // is not drawn. The preview's old rule drew it — 0.995 is short of its 0.999 — while the picture did not.
        let a = [[0.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [1000.0, 0.0, 0.0]];
        let b = [[0.0, 1000.0, 0.0], [0.0, 0.0, 0.0], [-995.1, 500.0, -99.6]];
        let drawn = edges(&[a, b]);
        assert_eq!(drawn.len(), 4, "each triangle's two outer sides");
        assert!(
            !drawn.iter().any(|[p, q]| p[0].abs() < 1e-9 && q[0].abs() < 1e-9 && p[2].abs() < 1e-9 && q[2].abs() < 1e-9),
            "the edge they share is not drawn"
        );
    }

    #[test]
    fn a_push_past_inside_out_is_clamped_to_the_last_distance_that_still_makes_the_solid() {
        let tris = box_tris(1000.0);
        let face = face_at(&tris, [500.0, 500.0, 1000.0]).expect("the top");
        assert!(push(&tris, &face, -1000.0).is_err());
        let (by, out) = push_clamped(&tris, &face, -1000.0);
        assert!(by > -1000.0 && by < 0.0, "the clamp was {by}");
        assert!(!out.is_empty(), "the triangles the clamp found, for a preview to draw");
    }

    #[test]
    fn a_clamped_push_that_fits_is_the_push_itself() {
        let sheet = pane(1000.0, 0.0);
        let face = face_at(&sheet, [500.0, 500.0, 0.0]).expect("the sheet's face");
        let (by, out) = push_clamped(&sheet, &face, 500.0);
        assert_eq!(by, 500.0, "nothing to clamp");
        assert_eq!(out.len(), 12, "the core push makes the solid a preview draws");
        assert_eq!(out.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max), 500.0);
    }

    #[test]
    fn three_distinct_corners_in_a_line_are_no_face() {
        let sliver = [[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [2000.0, 0.0, 0.0]];
        let e = measure(&[sliver]).unwrap_err();
        assert!(e.contains("no area"), "a zero-area sliver: {e}");
        assert!(sealed(&[sliver]).unwrap_err().contains("no area"), "a seal cannot close over it");
        // And a solid with one such triangle added is no longer a solid a push may trust.
        let mut box_ = box_tris(1000.0);
        box_.push(sliver);
        assert!(measure(&box_).unwrap_err().contains("no area"));
        // A thin but real triangle still passes.
        let thin = [[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [500.0, 0.001, 0.0]];
        assert!(measure(&[thin]).is_ok(), "a triangle one weld high has an area");
    }

    #[test]
    fn push_clamped_welds_its_source_once_not_once_per_bisection_step() {
        let tris = box_tris(1000.0);
        let face = face_at(&tris, [500.0, 500.0, 1000.0]).expect("the top");
        assert!(push(&tris, &face, -1000.0).is_err(), "this one needs the clamp");
        WELD_CALLS.with(|n| n.set(0));
        let (by, out) = push_clamped(&tris, &face, -1000.0);
        let calls = WELD_CALLS.with(|n| n.get());
        assert_eq!(calls, 1, "one source weld, not one per clamp step: {calls}");
        assert!(by > -1000.0 && by < 0.0 && !out.is_empty(), "the clamp still found a push: {by}");
        // Welding the source first changes nothing in the answer: the clamp's result is the same from canonical corners.
        let welded: Vec<[P3; 3]> =
            weld(&tris).into_iter().map(|k| k.map(|[x, y, z]| [x as f64 * 1e-3, y as f64 * 1e-3, z as f64 * 1e-3])).collect();
        assert_eq!(welded, tris, "a box is already on the weld grid");
        assert_eq!(push_clamped(&welded, &face, -1000.0), (by, out));
    }

    #[test]
    fn one_prepared_mesh_answers_the_face_question_and_the_push_from_a_single_weld() {
        let tris = box_tris(1000.0);
        WELD_CALLS.with(|n| n.set(0));
        let pre = prepare(&tris).unwrap();
        let top = pre.faces().iter().find(|f| f.normal[2] > 0.99).unwrap().clone();
        assert_eq!(pre.face_at(&tris, [500.0, 500.0, 1000.0]).unwrap().id, top.id, "the same face by point");
        assert_eq!(pre.face_of(top.id).unwrap().id, top.id, "and by id");
        let (by, out) = push_clamped_prepared(&pre, &tris, &top, -1000.0);
        assert!(by > -1000.0 && !out.is_empty(), "the push still answers: {by}");
        assert_eq!(WELD_CALLS.with(|n| n.get()), 1, "one weld for measure, faces, point, id and push");
    }

    /// A fan of `n` triangles has `n` rim edges; a second fan far away makes `n*n` pairs. At `n = 2100` that is 4.41 M,
    /// past the old 4 M cut-off, which silently called the face a surface and stretched it instead of extruding it.
    #[test]
    fn a_dense_split_face_still_extrudes_past_four_million_pair_candidates() {
        let fan = |n: usize, z: f64, east: f64| -> Vec<[P3; 3]> {
            let point = |k: usize| {
                let a = k as f64 * std::f64::consts::TAU / n as f64;
                [east + 1000.0 * a.cos(), 1000.0 * a.sin(), z]
            };
            (0..n).map(|k| [[east, 0.0, z], point(k), point((k + 1) % n)]).collect()
        };
        const N: usize = 2100;
        const { assert!(N * N > 4_000_000, "this is the case the old cut-off refused") };
        let mut tris = fan(N, 0.0, 0.0);
        let far = fan(N, 5000.0, 100_000.0);
        tris.extend(far.iter().copied());
        let centre = fan(N, 0.0, 0.0)[0][0];
        let inside = [
            (centre[0] + 1000.0 + 1000.0 * (std::f64::consts::TAU / N as f64).cos()) / 3.0,
            1000.0 * (std::f64::consts::TAU / N as f64).sin() / 3.0,
            0.0,
        ];
        let face = face_at(&tris, inside).expect("a face of the lower fan");
        assert_eq!(face.tris.len(), N, "the fan is one flat face");
        let pushed = push(&tris, &face, 500.0).expect("the face rises");
        assert_eq!(
            pushed.len(),
            far.len() + 4 * N,
            "the lower fan extruded into a solid, not stretched in place: {} triangles",
            pushed.len()
        );
        assert!(sealed(&pushed[N..]).is_ok(), "what rose from the fan closes a solid");
    }
}
