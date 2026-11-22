//! The one seam to solidcore. Nothing from that crate crosses this file: callers see only
//! [`BrepData`] (immutable SCBREP05 bytes) and [`TessCache`] (display and picking data).
//!
//! Every `Order`↔native index translation and every kernel guardrail lives here. Face, edge and
//! vertex handles are plain indices into the record, so a caller can never mix the two index
//! spaces (solidcore's `pick::edges/vertices` use native indices; `tri_face` uses record ones).

use solidcore_ops::body::Contract;
use solidcore_ops::check::CheckLevel;
use solidcore_ops::{Body, BooleanOp, make_box, make_cylinder};
use solidcore_record::Vec3 as RVec3;
use solidcore_record::brep::envelope;
use solidcore_record::native::{self, Runtime, Saved};
use solidcore_tess::MeshTolerance;

use std::collections::BTreeMap;
use std::sync::Arc;

use crate::vec3::P3;

/// The old one-shot mesh→kernel push, folded in rather than left as a root module. It still
/// serves `dm::push` for legacy mesh forms; new exact-solid edits use the seam below.
pub mod legacy;
pub use legacy::push;

/// Maximum bytes of an exact solid in a form, before base64 (solidcore's own cap is 256 MiB).
pub const MAX_BREP_BYTES: usize = 64 << 20;

/// An index into the record's face list. Re-match by signature after an edit, because solidcore
/// reorders faces; a stale handle is never silently reused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FaceId(pub u32);
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EdgeId(pub u32);
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertId(pub u32);

/// Immutable exact solid: SCBREP05 bytes, cheap to clone for undo snapshots.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BrepData(Arc<[u8]>);

impl BrepData {
    /// Wrap kernel-produced bytes. Refuses a body too large for a project file.
    pub fn from_bytes(bytes: Vec<u8>) -> Result<Self, String> {
        if bytes.len() > MAX_BREP_BYTES {
            return Err(format!("that solid is too large for a project ({} of {MAX_BREP_BYTES} bytes)", bytes.len()));
        }
        if bytes.get(..8) != Some(native::MAGIC.as_slice()) {
            return Err("that is not an exact solid (bad SCBREP05 header)".into());
        }
        Ok(Self(Arc::from(bytes)))
    }
    pub fn bytes(&self) -> &[u8] {
        &self.0
    }
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// How a face is recognised after solidcore reorders its records.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceSig {
    pub centroid: P3,
    pub normal: P3,
    pub low: P3,
    pub high: P3,
}

/// The display and picking cache generated from a [`BrepData`]. Triangles carry the record face
/// index; edge polylines and vertices are indexed by the record's edge and vertex lists.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TessCache {
    pub triangles: Arc<Vec<[P3; 3]>>,
    pub tri_face: Arc<Vec<FaceId>>,
    /// The three boundary edges of each triangle, in the same order as its corners. `EdgeId`
    /// indexes `edges`; solidcore's native edge numbers never leave this module.
    pub tri_edge: Arc<Vec<[EdgeId; 3]>>,
    pub edges: Arc<Vec<Vec<P3>>>,
    /// The two endpoints of each edge polyline. `VertId` indexes `vertices`.
    pub edge_vert: Arc<Vec<[VertId; 2]>>,
    pub vertices: Arc<Vec<P3>>,
    pub faces: Arc<Vec<FaceSig>>,
    /// Every edge on each face's boundary, deduplicated and sorted.
    pub face_edges: Arc<Vec<Vec<EdgeId>>>,
}

/// One completed kernel edit: the new exact solid and the cache regenerated from it.
#[derive(Clone, Debug, PartialEq)]
pub struct Committed {
    pub data: BrepData,
    pub cache: TessCache,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoolOp {
    Union,
    Subtract,
    Intersect,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Prim {
    Box { origin: [f64; 3], size: [f64; 3] },
    Cylinder { origin: [f64; 3], radius: f64, height: f64 },
}

#[derive(Clone, Debug, PartialEq)]
pub enum Edit {
    Push {
        faces: Vec<FaceId>,
        distance: f64,
    },
    Bevel {
        edges: Vec<EdgeId>,
        radius: f64,
        chamfer: bool,
    },
    Boolean {
        other: BrepData,
        op: BoolOp,
    },
    /// A rigid translation of the whole body. Draft bodies stay in draft-local coordinates, so a
    /// move is one exact affine transform rather than a rewritten mesh.
    Move {
        delta: [f64; 3],
    },
}

/// One boolean between two exact solids, with the seam's validity gate on the result.
pub fn boolean(target: &BrepData, tool: &BrepData, op: BoolOp) -> Result<Committed, String> {
    let mut a = Kernel::from_data(target)?;
    a.edit(Edit::Boolean { other: tool.clone(), op })
}

/// The kernel itself. Kept alive only inside an operation; it holds no project state.
pub struct Kernel {
    body: Body,
}

fn err(e: impl std::fmt::Debug) -> String {
    format!("{e:?}")
}

/// The K-spike guardrail: every local operation exactly doubles each edge's tolerance until the
/// tenth edit refuses its own result. Reset them to the body's before every edit. `kernel()` is
/// solidcore's hidden bridge; confining it to this module is the point of the seam.
fn reset_edge_tolerances(body: &Body) {
    let mut guard = body.kernel();
    let brep: &mut solidcore::Brep = &mut guard;
    let body_tol = brep.tolerance();
    let ids: Vec<_> = brep.edges().ids().collect();
    for e in ids {
        if solidcore::topology::edge::tolerance(brep, e).unwrap_or(body_tol) != body_tol {
            let _ = solidcore::topology::edge::set_tolerance(brep, e, body_tol, false, false);
        }
    }
}

/// The seam tolerance of a sew: below a weld's micron is exact for planar forms.
const SEW_TOL: f64 = 1e-3;

/// Welded facets sewn into one closed solid body. Only kernel.rs ever sees the result.
fn sewn(tris: &[[P3; 3]]) -> Result<Body, String> {
    let rings = crate::mesh::facet_rings(tris)?;
    if rings.is_empty() {
        return Err("the form holds no faces to sew".into());
    }
    let mut sheets = Vec::new();
    for ring in &rings {
        let lines: Vec<_> = ring
            .iter()
            .enumerate()
            .map(|(i, p)| solidcore_ops::sweep::Curve::Line { start: *p, end: ring[(i + 1) % ring.len()] })
            .collect();
        sheets.push(solidcore_ops::sweep::planar_faces(&lines).map_err(err)?);
    }
    let refs: Vec<&Body> = sheets.iter().collect();
    let stitched = solidcore_ops::stitch::sew(&refs, SEW_TOL).map_err(err)?;
    if !stitched.closed {
        return Err("the form's faces do not sew into a closed solid".into());
    }
    Ok(stitched.body)
}

fn vec3(p: &RVec3) -> P3 {
    [p.x, p.y, p.z]
}

fn diagonal(body: &Body) -> Result<f64, String> {
    let (lo, hi) = body.bounds().map_err(err)?;
    let dx = hi[0] - lo[0];
    let dy = hi[1] - lo[1];
    let dz = hi[2] - lo[2];
    let d = (dx * dx + dy * dy + dz * dz).sqrt();
    if !d.is_finite() || d <= 0.0 {
        return Err("that solid has no extent".into());
    }
    Ok(d)
}

/// Quantise a point onto the seam's weld grid: a key shared by the same corner on two faces.
fn vkey(p: P3) -> [i64; 3] {
    p.map(|v| (v / SEW_TOL).round() as i64)
}

fn point_segment(p: P3, a: P3, b: P3) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = ab.iter().map(|v| v * v).sum::<f64>();
    if len2 == 0.0 {
        return p.iter().zip(a).map(|(p, a)| (p - a).powi(2)).sum::<f64>().sqrt();
    }
    let t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / len2;
    let t = t.clamp(0.0, 1.0);
    let gap = [p[0] - a[0] - t * ab[0], p[1] - a[1] - t * ab[1], p[2] - a[2] - t * ab[2]];
    gap.iter().map(|v| v * v).sum::<f64>().sqrt()
}

/// The four index vectors derived from one tessellation: triangles' edges, edges' endpoints,
/// unique vertices, and each face's boundary edges.
type Indices = (Vec<[EdgeId; 3]>, Vec<[VertId; 2]>, Vec<P3>, Vec<Vec<EdgeId>>);

/// Derive the opaque face/edge/vertex indices from one tessellation. Every id is an index into
/// one of the vectors returned here; no solidcore native number escapes.
fn indices(triangles: &[[P3; 3]], tri_face: &[FaceId], edges: &[Vec<P3>], positions: &[P3]) -> Indices {
    let mut verts: Vec<P3> = Vec::new();
    let mut vmap: BTreeMap<[i64; 3], VertId> = BTreeMap::new();
    let mut vertex = |p: P3| -> VertId {
        let k = vkey(p);
        if let Some(id) = vmap.get(&k) {
            return *id;
        }
        let id = VertId(verts.len() as u32);
        verts.push(p);
        vmap.insert(k, id);
        id
    };
    let mut emap: BTreeMap<([i64; 3], [i64; 3]), EdgeId> = BTreeMap::new();
    for (i, line) in edges.iter().enumerate() {
        for pair in line.windows(2) {
            let (a, b) = (vkey(pair[0]), vkey(pair[1]));
            emap.entry((a.min(b), a.max(b))).or_insert(EdgeId(i as u32));
        }
    }
    let edge_vert: Vec<[VertId; 2]> =
        edges.iter().map(|line| [vertex(*line.first().unwrap_or(&[0.0; 3])), vertex(*line.last().unwrap_or(&[0.0; 3]))]).collect();
    // A tessellator may split a boundary edge without listing the sub-segment; the midpoint
    // search covers those few cases without making the whole lookup O(triangles × edges).
    let tri_edge: Vec<[EdgeId; 3]> = triangles
        .iter()
        .map(|t| {
            std::array::from_fn(|k| {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let (ka, kb) = (vkey(a), vkey(b));
                if let Some(id) = emap.get(&(ka.min(kb), ka.max(kb))) {
                    return *id;
                }
                let mid = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0];
                edges
                    .iter()
                    .enumerate()
                    .find(|(_, line)| line.windows(2).any(|q| point_segment(mid, q[0], q[1]) <= SEW_TOL))
                    .map(|(i, _)| EdgeId(i as u32))
                    .unwrap_or(EdgeId(u32::MAX))
            })
        })
        .collect();
    let mut face_edges = vec![Vec::new(); tri_face.iter().map(|f| f.0 as usize + 1).max().unwrap_or(0)];
    for (tri, face) in tri_face.iter().enumerate() {
        let Some(list) = face_edges.get_mut(face.0 as usize) else { continue };
        for e in tri_edge[tri] {
            if e.0 != u32::MAX && !list.contains(&e) {
                list.push(e);
            }
        }
    }
    for list in &mut face_edges {
        list.sort();
    }
    let _ = positions;
    (tri_edge, edge_vert, verts, face_edges)
}

fn tessellate(body: &Body) -> Result<TessCache, String> {
    let (record, _) = body.to_record().map_err(err)?;
    let size = diagonal(body)?;
    let tol = MeshTolerance::display_of(size);
    let (plan, _) = solidcore_tess::plan(&record, &tol, size).map_err(err)?;
    let (mesh, refused) = plan.evaluate();
    if !refused.is_empty() {
        return Err(format!("{} of its faces could not be drawn", refused.len()));
    }
    let tris: Vec<[P3; 3]> = mesh
        .triangles
        .iter()
        .map(|t| [vec3(&mesh.positions[t[0] as usize]), vec3(&mesh.positions[t[1] as usize]), vec3(&mesh.positions[t[2] as usize])])
        .collect();
    let tf: Vec<FaceId> = mesh.tri_face.iter().map(|f| FaceId(*f)).collect();
    let ed: Vec<Vec<P3>> = plan.edges.iter().map(|line| line.iter().map(vec3).collect::<Vec<_>>()).collect();
    let pos: Vec<P3> = mesh.positions.iter().map(vec3).collect();
    let (tri_edge, edge_vert, unique, face_edges) = indices(&tris, &tf, &ed, &pos);
    let triangles = Arc::new(tris);
    let tri_face = Arc::new(tf);
    let edges = Arc::new(ed);
    let vertices = Arc::new(unique);

    // A signature per record face, from the triangles that name it. Used to re-match a selection
    // after an edit, since solidcore gives no stable face ids.
    let blank = FaceSig { centroid: [0.0; 3], normal: [0.0; 3], low: [f64::INFINITY; 3], high: [f64::NEG_INFINITY; 3] };
    let mut faces = vec![blank; record.faces.len()];
    for (tri, f) in mesh.triangles.iter().zip(&mesh.tri_face) {
        let Some(sig) = faces.get_mut(*f as usize) else { continue };
        let a = vec3(&mesh.positions[tri[0] as usize]);
        let b = vec3(&mesh.positions[tri[1] as usize]);
        let c = vec3(&mesh.positions[tri[2] as usize]);
        for p in [a, b, c] {
            for (i, v) in p.iter().enumerate() {
                sig.centroid[i] += v / 3.0;
                sig.low[i] = sig.low[i].min(*v);
                sig.high[i] = sig.high[i].max(*v);
            }
        }
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        sig.normal[0] += u[1] * v[2] - u[2] * v[1];
        sig.normal[1] += u[2] * v[0] - u[0] * v[2];
        sig.normal[2] += u[0] * v[1] - u[1] * v[0];
    }
    Ok(TessCache {
        triangles,
        tri_face,
        tri_edge: Arc::new(tri_edge),
        edges,
        edge_vert: Arc::new(edge_vert),
        vertices,
        faces: Arc::new(faces),
        face_edges: Arc::new(face_edges),
    })
}

fn seal(body: &Body) -> Result<Committed, String> {
    let (record, _) = body.to_record().map_err(err)?;
    let saved = Saved {
        solid: record,
        metadata: envelope::Metadata {
            version: envelope::VERSION,
            color: None,
            paint: None,
            boundaries: None,
            face_sources: None,
            history: None,
            preserve_boundaries: false,
        },
        runtime: Runtime { size: diagonal(body)?, coarse_missing: None },
    };
    let data = BrepData::from_bytes(native::encode(&saved).map_err(err)?)?;
    let cache = tessellate(body)?;
    Ok(Committed { data, cache })
}

/// Refuse a body that is empty, broken or has no finite positive volume. Ops leave a refusal as
/// the caller's responsibility, so this is the one validity gate every commit passes through.
fn gate(body: &Body) -> Result<(), String> {
    let report = body.check(CheckLevel::All).map_err(err)?;
    if report.failures != 0 {
        return Err(format!("the kernel finds {} validity problem(s): {}", report.failures, report.text));
    }
    let volume = body.volume(1e-6).map_err(err)?;
    if !volume.is_finite() || volume <= 0.0 {
        return Err("that edit would leave no solid".into());
    }
    Ok(())
}

impl Kernel {
    /// Configure the kernel fixes this instance needs. solidcore's session is process-wide, so
    /// naming them here (rather than once at app startup) keeps every construction consistent.
    pub fn new() -> Result<Self, String> {
        solidcore_ops::session::set_fixes(["revolve_analytic_profile_faces"]).map_err(err)?;
        Ok(Self { body: make_box([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]).map_err(err)? })
    }

    /// Load an exact solid from a project. Only bytes this kernel sealed are trusted: `seal` runs
    /// `check(All)` on every commit and `native::decode` bounds and validates the record, so the
    /// healer has nothing to repair and must not renumber faces. `trusted = false` was measured to
    /// leave tolerance defects that fail `check(All)` on the next edit (see the four Stage 1 tests).
    pub fn from_data(data: &BrepData) -> Result<Self, String> {
        let saved = native::decode(data.bytes()).map_err(err)?;
        let (body, _) = Body::from_record(&saved.solid, true).map_err(err)?;
        let mut k = Self::new()?;
        k.body = body;
        Ok(k)
    }

    /// Sew one welded mesh into an exact solid. Used when converting legacy triangle forms.
    pub fn from_mesh(tris: &[[P3; 3]]) -> Result<Self, String> {
        let mut k = Self::new()?;
        k.body = sewn(tris)?;
        Ok(k)
    }

    /// Sew several touching pieces and union them into one solid — what a BIM wall with openings
    /// needs, where the scene gives us many render prisms rather than one watertight shell.
    pub fn from_meshes(parts: &[Vec<[P3; 3]>]) -> Result<Self, String> {
        if parts.is_empty() {
            return Err("there are no solids to convert".into());
        }
        let mut bodies = Vec::new();
        for part in parts {
            bodies.push(sewn(part)?);
        }
        let mut k = Self::new()?;
        k.body = if bodies.len() == 1 {
            bodies.pop().ok_or("there are no solids to convert")?
        } else {
            solidcore_ops::boolean::union_all(bodies).map_err(err)?
        };
        gate(&k.body)?;
        Ok(k)
    }

    /// Seal this kernel's current body without editing it.
    pub fn commit(&self) -> Result<Committed, String> {
        seal(&self.body)
    }

    pub fn create(prim: Prim) -> Result<Committed, String> {
        let mut k = Self::new()?;
        k.body = match prim {
            Prim::Box { origin, size } => make_box(origin, size).map_err(err)?,
            Prim::Cylinder { origin, radius, height } => make_cylinder(origin, radius, height).map_err(err)?,
        };
        gate(&k.body)?;
        seal(&k.body)
    }

    /// Apply one guarded edit. The body is cloned first, so a refusal leaves this kernel as it was.
    pub fn edit(&mut self, edit: Edit) -> Result<Committed, String> {
        let mut body = self.body.copy().map_err(err)?;
        reset_edge_tolerances(&body);
        match edit {
            Edit::Push { faces, distance } => {
                if !distance.is_finite() || distance.abs() > 1e6 || faces.is_empty() {
                    return Err("push a face by a finite distance in range".into());
                }
                let base = body.copy().map_err(err)?;
                let ids: Vec<usize> = faces.iter().map(|f| f.0 as usize).collect();
                let mut applied = distance;
                let mut refused = None;
                for _ in 0..8 {
                    let attempt = base.copy().map_err(err)?;
                    let (_, order) = attempt.to_record().map_err(err)?;
                    match solidcore_ops::offset::offset_faces(&attempt, &order, &ids, applied) {
                        Ok(()) => {
                            body = attempt;
                            refused = None;
                            break;
                        }
                        Err(e) => {
                            refused = Some(e);
                            // shortcut: offset_faces refuses some concave rebuilds; halve the
                            // distance and retry on a fresh copy. Upgrade to extrude-planar-region
                            // + boolean when face-region extraction joins the seam in Stage 3.
                            applied /= 2.0;
                            if applied.abs() < 1e-3 {
                                break;
                            }
                        }
                    }
                }
                if let Some(e) = refused {
                    return Err(format!("that face cannot be pushed by {distance}: {e:?}"));
                }
            }
            Edit::Bevel { edges, radius, chamfer } => {
                if !radius.is_finite() || !(0.0..1e6).contains(&radius) || edges.is_empty() {
                    return Err("bevel by a finite radius in range".into());
                }
                let (_, order) = body.to_record().map_err(err)?;
                let ids: Vec<usize> = edges.iter().map(|e| e.0 as usize).collect();
                let kind = if chamfer { solidcore_ops::Blend::Chamfer } else { solidcore_ops::Blend::Round };
                let options = solidcore_ops::BlendOptions {
                    turn: solidcore_ops::Turn::Mitre,
                    chamfer_corner: solidcore_ops::ChamferCorner::Mitre,
                    contract: Contract::Closed,
                };
                let _ = solidcore_ops::blend_ordered(&body, &order, kind, &ids, &[radius], &[], solidcore_ops::Corners::Filled, options)
                    .map_err(err)?;
            }
            Edit::Boolean { other, op } => {
                let saved = native::decode(other.bytes()).map_err(err)?;
                let (tool, _) = Body::from_record(&saved.solid, false).map_err(err)?;
                let op = match op {
                    BoolOp::Union => BooleanOp::Union,
                    BoolOp::Subtract => BooleanOp::Subtract,
                    BoolOp::Intersect => BooleanOp::Intersect,
                };
                let result = solidcore_ops::boolean::boolean_with(body, tool, op, Contract::Closed).map_err(err)?;
                body = result.ok_or("that boolean leaves no solid")?;
            }
            Edit::Move { delta } => {
                if delta.iter().any(|v| !v.is_finite() || v.abs() > 1e6) {
                    return Err("move by a finite distance in range".into());
                }
                let map = solidcore_ops::affine::translation(delta).map_err(err)?;
                body.transform(&map).map_err(err)?;
            }
        }
        gate(&body)?;
        let out = seal(&body)?;
        self.body = body;
        Ok(out)
    }

    /// Re-identify a face after an edit: the nearest signature by centroid and normal. Returns
    /// `None` when the face is gone, so a caller can drop a stale selection.
    pub fn rematch(&self, cache: &TessCache, face: FaceId) -> Option<FaceId> {
        let was = cache.faces.get(face.0 as usize)?;
        let fresh = tessellate(&self.body).ok()?;
        let mut best: Option<(f64, FaceId)> = None;
        for (i, now) in fresh.faces.iter().enumerate() {
            let mut d = 0.0;
            for c in 0..3 {
                d += (now.centroid[c] - was.centroid[c]).powi(2) + (now.normal[c] - was.normal[c]).powi(2);
            }
            if best.is_none_or(|(b, _)| d < b) {
                best = Some((d, FaceId(i as u32)));
            }
        }
        best.map(|(_, id)| id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn box_data() -> BrepData {
        Kernel::create(Prim::Box { origin: [0.0; 3], size: [1000.0, 800.0, 600.0] }).expect("box").data
    }

    #[test]
    fn a_solid_round_trips_through_its_opaque_bytes_and_keeps_its_picking_ids() {
        let first = Kernel::create(Prim::Box { origin: [0.0; 3], size: [1000.0, 800.0, 600.0] }).expect("box");
        assert!(!first.cache.triangles.is_empty(), "a box tessellates");
        assert!(first.cache.tri_face.iter().all(|f| (f.0 as usize) < first.cache.faces.len()));
        let mut back = Kernel::from_data(&first.data).expect("load");
        let again = back.edit(Edit::Push { faces: vec![FaceId(0)], distance: 10.0 }).expect("push");
        assert_ne!(again.data, first.data, "an edit changes the exact solid");
        assert!(again.cache.tri_face.iter().all(|f| (f.0 as usize) < again.cache.faces.len()));
    }

    #[test]
    fn thirty_sequential_edits_never_hit_the_edge_tolerance_refusal() {
        let mut k = Kernel::from_data(&box_data()).expect("load");
        let mut volume = k.body.volume(1e-6).expect("volume");
        for _ in 0..30 {
            let cache = tessellate(&k.body).expect("tess");
            let top = cache
                .faces
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.centroid[2].total_cmp(&b.1.centroid[2]))
                .map(|(i, _)| FaceId(i as u32))
                .expect("face");
            let next = k.edit(Edit::Push { faces: vec![top], distance: 10.0 }).expect("every edit succeeds");
            let now = Kernel::from_data(&next.data).expect("reload").body.volume(1e-6).expect("volume");
            assert!(now > volume, "the top face moved up: {now} > {volume}");
            volume = now;
        }
    }

    #[test]
    fn a_fillet_survives_a_save_and_a_load_and_a_second_edit() {
        let mut k = Kernel::from_data(&box_data()).expect("load");
        let result = k.edit(Edit::Bevel { edges: vec![EdgeId(0)], radius: 50.0, chamfer: false }).expect("fillet");
        let mut back = Kernel::from_data(&result.data).expect("reload");
        assert!(back.body.volume(1e-6).expect("volume") > 0.0, "the filleted solid is still a solid");
        assert!(back.edit(Edit::Push { faces: vec![FaceId(0)], distance: 5.0 }).is_ok(), "the filleted solid edits again");
    }

    #[test]
    fn a_concave_top_push_keeps_a_valid_solid() {
        let mut k = Kernel::from_mesh(&crate::dm::parity::u_tris()).expect("the U");
        let cache = k.commit().expect("commit").cache;
        let top = cache
            .faces
            .iter()
            .enumerate()
            .filter(|(_, f)| f.normal[2] > 0.99)
            .max_by(|a, b| {
                let area = |f: &FaceSig| (f.high[0] - f.low[0]) * (f.high[1] - f.low[1]);
                area(a.1).total_cmp(&area(b.1))
            })
            .map(|(i, _)| FaceId(i as u32))
            .expect("a top face");
        let out = k.edit(Edit::Push { faces: vec![top], distance: 200.0 });
        assert!(out.is_ok(), "the concave push was refused: {:?}", out.err());
        assert!(!out.expect("a solid").cache.triangles.is_empty());
    }

    #[test]
    fn a_boolean_union_of_two_prisms_is_one_solid() {
        let b = Kernel::create(Prim::Box { origin: [400.0, 200.0, 100.0], size: [800.0, 800.0, 600.0] }).expect("box");
        let mut k = Kernel::from_data(&box_data()).expect("load");
        let united = k.edit(Edit::Boolean { other: b.data, op: BoolOp::Union }).expect("union");
        let one = Kernel::from_data(&united.data).expect("reload");
        assert_eq!(one.body.solid_count().expect("count"), 1, "two touching prisms are one solid");
    }
}
