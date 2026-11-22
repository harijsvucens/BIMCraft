//! The solidcore kernel behind the Direct Model push, behind the `kernel-solidcore`
//! feature (fork-only experiment): a mesh form's welded facets are sewn into an exact
//! B-rep, the picked face is moved with the kernel's `offset_faces`, and the result is
//! tessellated back into triangles for the form's mesh. Nothing is stored in the
//! solidcore representation: each command sews fresh from the mesh, edits once, and
//! returns triangles, so the file format, the render path and the picking are
//! untouched. The face to move is the record face whose surface holds the middle of the
//! face the pick named — the same question `mesh::face_at` answers — so the kernel
//! moves the face the user picked, however the tessellator orders its records: two
//! disjoint faces in one plane both hold the plane, and only the pick point tells them
//! apart. Every refusal falls back to the caller (the command keeps the mesh path);
//! this module only ever answers with new triangles or an error.

use crate::mesh::TopoFace;
use crate::model::MAX_COORD;
use crate::vec3::P3;

use solidcore_ops::Body;
use solidcore_ops::sweep::Curve as OpsCurve;
use solidcore_record::brep::Brep as Record;
use solidcore_tess::MeshTolerance;

/// The seam tolerance of a sew: below a weld's micron is exact for planar forms.
const SEW_TOL: f64 = 1e-3;

/// The mesh's welded facets sewn into one closed solid body.
fn sewn(tris: &[[P3; 3]]) -> Result<Body, String> {
    let rings = crate::mesh::facet_rings(tris)?;
    if rings.is_empty() {
        return Err("the form holds no faces to sew".to_string());
    }
    let mut sheets: Vec<Body> = Vec::new();
    for ring in &rings {
        let lines: Vec<OpsCurve> =
            ring.iter().enumerate().map(|(i, p)| OpsCurve::Line { start: *p, end: ring[(i + 1) % ring.len()] }).collect();
        sheets.push(solidcore_ops::sweep::planar_faces(&lines).map_err(|e| format!("a facet is no planar sheet: {e}"))?);
    }
    let refs: Vec<&Body> = sheets.iter().collect();
    let sewn = solidcore_ops::stitch::sew(&refs, SEW_TOL).map_err(|e| format!("the form's faces do not sew into a solid: {e}"))?;
    if !sewn.closed {
        return Err("the form's faces do not sew into a closed solid".to_string());
    }
    Ok(sewn.body)
}

/// What the tessellator says per record face: its outward normal, its triangle area, one
/// point of its plane, and whether the pick's probe stands on its surface.
#[derive(Clone, Copy)]
struct Group {
    normal: [f64; 3],
    area: f64,
    at: [f64; 3],
    holds: bool,
}

/// What the tessellator says per record face of `record`: its outward normal, its
/// triangle area, one point of its plane, and whether `probe` — the middle of the face
/// the pick named — stands on its surface, by the mesh's own rule ([`crate::mesh::holds`]).
fn groups(record: &Record, diag: f64, probe: P3) -> Result<Vec<Group>, String> {
    let tolerance = MeshTolerance::display_of(diag);
    let (plan, _) = solidcore_tess::plan(record, &tolerance, diag).map_err(|e| format!("tess plan: {e}"))?;
    let (mesh, refused) = plan.evaluate();
    if !refused.is_empty() {
        return Err(format!("the tessellator refused {} faces", refused.len()));
    }
    let mut out: Vec<Group> = Vec::new();
    for (tri, f) in mesh.triangles.iter().zip(&mesh.tri_face) {
        let f = *f as usize;
        if f >= out.len() {
            out.resize(f + 1, Group { normal: [0.0; 3], area: 0.0, at: [0.0; 3], holds: false });
        }
        let a = mesh.positions[tri[0] as usize];
        let b = mesh.positions[tri[1] as usize];
        let c = mesh.positions[tri[2] as usize];
        let (pa, pb, pc) = ([a.x, a.y, a.z], [b.x, b.y, b.z], [c.x, c.y, c.z]);
        let u = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
        let v = [pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]];
        let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        let size = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let g = &mut out[f];
        if g.area == 0.0 {
            g.at = pa; // the group's first triangle: a point of its plane
        }
        if !g.holds && size > 1e-12 {
            // The same question the mesh's pick asks: does the plane hold the probe, and does its surface?
            let unit = [n[0] / size, n[1] / size, n[2] / size];
            g.holds = crate::mesh::holds_at([pa, pb, pc], unit, probe);
        }
        g.normal[0] += n[0];
        g.normal[1] += n[1];
        g.normal[2] += n[2];
        g.area += size * 0.5;
    }
    Ok(out
        .into_iter()
        .map(|mut g| {
            let len = (g.normal[0] * g.normal[0] + g.normal[1] * g.normal[1] + g.normal[2] * g.normal[2]).sqrt();
            g.normal = if len > 1e-12 { [g.normal[0] / len, g.normal[1] / len, g.normal[2] / len] } else { [0.0; 3] };
            g
        })
        .collect())
}

/// The mesh's bounding diagonal, for the display tolerance.
fn diagonal(tris: &[[P3; 3]]) -> f64 {
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for t in tris {
        for p in t {
            for j in 0..3 {
                lo[j] = lo[j].min(p[j]);
                hi[j] = hi[j].max(p[j]);
            }
        }
    }
    let d = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt().max(1.0)
}

/// The whole-body tessellation as triangles, what the form's mesh keeps.
fn triangles(record: &Record, diag: f64) -> Result<Vec<[P3; 3]>, String> {
    let tolerance = MeshTolerance::display_of(diag);
    let (plan, _) = solidcore_tess::plan(record, &tolerance, diag).map_err(|e| format!("tess plan: {e}"))?;
    let (mesh, refused) = plan.evaluate();
    if !refused.is_empty() {
        return Err(format!("the tessellator refused {} faces", refused.len()));
    }
    Ok(mesh
        .triangles
        .iter()
        .map(|t| {
            let a = mesh.positions[t[0] as usize];
            let b = mesh.positions[t[1] as usize];
            let c = mesh.positions[t[2] as usize];
            [[a.x, a.y, a.z], [b.x, b.y, b.z], [c.x, c.y, c.z]]
        })
        .collect())
}

/// `mesh::push` with the kernel: the form's facets are sewn into an exact B-rep, the
/// face the pick named is found in it by the point of the pick, moved `distance` mm
/// along its normal with the kernel's offset executive, and the result is tessellated
/// back into triangles. The distance follows the face's own outward normal, which is the
/// one the tessellator reports, so a reversed mesh moves the other way and stays reversed.
pub fn push(tris: &[[P3; 3]], face: &TopoFace, distance: f64) -> Result<Vec<[P3; 3]>, String> {
    if !distance.is_finite() || distance.abs() > MAX_COORD {
        return Err(format!("a distance is a number of mm within {MAX_COORD}, not {distance}"));
    }
    let diag = diagonal(tris);
    let body = sewn(tris)?;
    // One record serves both the face question and the move: its `Order` is what offset_faces asks for.
    let (record, order) = body.to_record().map_err(|e| format!("to_record: {e}"))?;
    // The one probe rule: the middle of the face's first triangle, strictly inside it.
    let probe = crate::mesh::identity::probe(tris, face)?;
    let g = groups(&record, diag, probe)?;
    let mut picked: Option<(usize, f64)> = None;
    for (i, grp) in g.iter().enumerate() {
        if grp.area <= 1.0 || !grp.holds {
            continue;
        }
        let dot = grp.normal[0] * face.normal[0] + grp.normal[1] * face.normal[1] + grp.normal[2] * face.normal[2];
        if dot.abs() < 0.99 {
            continue;
        }
        picked = Some((i, dot));
        break;
    }
    let (index, dot) = picked.ok_or("the kernel does not hold that face: the sew split it")?;
    solidcore_ops::offset::offset_faces(&body, &order, &[index], distance * dot)
        .map_err(|e| format!("the kernel refused the move: {e}"))?;
    let (after, _) = body.to_record().map_err(|e| format!("to_record: {e}"))?;
    triangles(&after, diag)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dm::parity::{box_tris, u_tris};

    #[test]
    fn a_box_s_facets_close_into_rings() {
        let rings = crate::mesh::facet_rings(&box_tris()).unwrap();
        assert_eq!(rings.len(), 6, "six facets");
        assert!(rings.iter().all(|r| r.len() == 4), "each ring is the box's four corners: {rings:?}");
    }

    #[test]
    fn a_box_pushes_through_the_kernel() {
        let tris = box_tris();
        let top = crate::mesh::faces(&tris).into_iter().find(|f| f.normal[2] > 0.99).unwrap();
        let out = push(&tris, &top, 500.0).unwrap();
        let m = crate::mesh::measure(&out).unwrap();
        assert!(m.is_solid());
        assert!((m.volume - 1.5e9).abs() < 1.0, "volume {} after a 500 pull of the 1e6-area top", m.volume);
    }

    #[test]
    fn the_kernel_picks_the_face_the_pick_lies_on() {
        let tris = u_tris();
        let body = sewn(&tris).unwrap();
        let (record, _) = body.to_record().unwrap();
        for (at, area) in [([250.0, 250.0, 500.0], 250_000.0), ([1150.0, 250.0, 500.0], 150_000.0)] {
            let face = crate::mesh::face_at(&tris, at).unwrap();
            let t = tris[face.tris[0]];
            let probe = std::array::from_fn(|j| (t[0][j] + t[1][j] + t[2][j]) / 3.0);
            let g = groups(&record, diagonal(&tris), probe).unwrap();
            let (_, grp) =
                g.iter().enumerate().find(|(_, grp)| grp.area > 1.0 && grp.holds).expect("the pick's face is in the kernel's body");
            assert!((grp.area - area).abs() < 1.0, "the face holding {at:?} is the {area}-mm² top, not {:.0}", grp.area);
        }
    }

    #[test]
    fn both_engines_move_the_picked_face_and_agree_on_its_volume() {
        let tris = u_tris();
        let base = crate::mesh::measure(&tris).unwrap();
        assert!(base.is_solid(), "the U is a solid, not {:?}", base.kind);
        assert!((base.volume - 2.25e8).abs() < 1.0, "the U's volume {}", base.volume);
        // Two disjoint tops in one plane: only the pick point tells them apart. The corpus answers
        // the engine question and pins the refusal policy — a kernel refusal is asserted, never
        // printed into an always-passing test.
        for (at, area, rise, want) in
            [([250.0, 250.0, 500.0], 250_000.0, 200.0, 2.75e8), ([1150.0, 250.0, 500.0], 150_000.0, 150.0, 2.475e8)]
        {
            let face = crate::mesh::face_at(&tris, at).unwrap();
            crate::dm::parity::agree(&format!("U at {at:?}"), &tris, &face, rise, want);
            // The picked top rose by exactly this, and nothing rose higher — on the mesh path, the
            // fallback the command uses whenever the kernel refuses this concave move.
            let out = crate::mesh::push(&tris, &face, rise).unwrap();
            let tops: Vec<(f64, f64)> =
                crate::mesh::faces(&out).iter().filter(|f| f.normal[2] > 0.99).map(|f| (out[f.tris[0]][0][2], f.area)).collect();
            assert!(
                tops.iter().any(|(z, a)| (*z - 500.0 - rise).abs() < 1.0 && (*a - area).abs() < 1.0),
                "the picked top rose as {} mm²: {tops:?}",
                area
            );
            assert!(!tops.iter().any(|(z, _)| *z > 501.0 + rise), "nothing rose higher: {tops:?}");
        }
    }
}
