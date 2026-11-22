//! Direct modeling's one push: engine choice, clamp, validation and wording in one place.
//!
//! The command, the drag's preview and the kernel bridge all ask [`push`], so what a preview
//! shows is what a release commits, whichever engine the build has. The engine seam — which
//! until here was a `cfg` arm inside `commands.rs` — lives here: `commands.rs` knows no engine
//! exists, and a later boolean engine adds a function beside [`push`], not another arm in the
//! command file (the file main's release merges touch hardest).

use crate::mesh::{self, Measure, TopoFace};
use crate::vec3::P3;

#[cfg(test)]
pub(crate) mod parity;

/// The one writer between an exact solid and a form.
pub mod brep;
/// One built element into one exact solid.
pub mod convert;
/// The multi-body studio session, held outside `Model` until Finish.
pub mod draft;
/// The opaque solidcore seam. No kernel type crosses out of it.
pub mod kernel;
/// Hit tests over the tessellated cache, in the seam's own id space.
pub mod pick;

/// The answer to one push: the distance actually applied, the new triangles, their measure,
/// the engine that answered, and the kernel's reason when it fell back to the mesh.
pub struct Push {
    pub by: f64,
    pub tris: Vec<[P3; 3]>,
    pub measure: Measure,
    pub backend: &'static str,
    pub fallback: Option<String>,
}

/// Push one flat face `distance` mm along its normal, clamped to the largest move the mesh
/// allows. The clamp is decided once by the mesh path (cheap, engine-independent); the chosen
/// engine then moves the face exactly that far, and a kernel refusal falls back to the clamped
/// mesh result. A preview draws this answer and a release commits it, so the two cannot drift.
pub fn push(tris: &[[P3; 3]], face: &TopoFace, distance: f64) -> Result<Push, String> {
    let prepared = mesh::prepare(tris)?;
    push_prepared(&prepared, tris, face, distance)
}

/// [`push`] with the source already prepared: a gesture can ask its face question and push from one
/// weld, instead of welding the same triangles again at every seam.
pub fn push_prepared(prepared: &mesh::Prepared, tris: &[[P3; 3]], face: &TopoFace, distance: f64) -> Result<Push, String> {
    // The clamp: `push_clamped` answers the largest valid distance and the mesh triangles at it.
    // Its result is the preview's ink and the fallback when the kernel will not move this face.
    let (by, mesh_tris) = mesh::push_clamped_prepared(prepared, tris, face, distance);
    if mesh_tris.is_empty() {
        return Err("that would turn the solid inside out: push it less far".to_string());
    }

    #[cfg(feature = "kernel-solidcore")]
    let (out, backend, fallback) = match crate::dm::kernel::push(tris, face, by) {
        Ok(candidate) => match mesh::measure(&candidate) {
            Ok(m) if m.is_solid() => (candidate, "kernel", None),
            _ => (mesh_tris, "mesh", Some("the kernel's result does not close a solid".to_string())),
        },
        Err(reason) => (mesh_tris, "mesh", Some(reason)),
    };
    #[cfg(not(feature = "kernel-solidcore"))]
    let (out, backend, fallback) = (mesh_tris, "mesh", None);

    let measure = mesh::measure(&out)?;
    if let Some(why) = crate::family::over_triangles("that push", out.len()) {
        return Err(why);
    }
    Ok(Push { by, tris: out, measure, backend, fallback })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn box_tris() -> Vec<[P3; 3]> {
        let p: Vec<P3> = vec![
            [0.0, 0.0, 0.0],
            [1000.0, 0.0, 0.0],
            [1000.0, 1000.0, 0.0],
            [0.0, 1000.0, 0.0],
            [0.0, 0.0, 1000.0],
            [1000.0, 0.0, 1000.0],
            [1000.0, 1000.0, 1000.0],
            [0.0, 1000.0, 1000.0],
        ];
        [
            [0usize, 2, 1],
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
        ]
        .iter()
        .map(|t| [p[t[0]], p[t[1]], p[t[2]]])
        .collect()
    }

    #[test]
    fn a_push_over_the_whole_solid_is_clamped_to_the_last_valid_distance() {
        let tris = box_tris();
        let top = mesh::faces(&tris).into_iter().find(|f| f.normal[2] > 0.99).unwrap();
        // A raw -2000 would turn the box inside out. The one seam clamps it to the last distance
        // that still closes, so the typed push and the drag preview answer the same question.
        let out = push(&tris, &top, -2000.0).unwrap();
        assert!(out.by < 0.0 && out.by > -1000.0, "the clamp kept a sliver: {}", out.by);
        assert!(out.measure.is_solid(), "the clamped push still closes");
    }

    #[test]
    fn a_refused_kernel_answer_falls_back_to_the_clamped_mesh() {
        // An open sheet sews to no kernel body, so the mesh path must stand in and the reason is kept.
        let tris: Vec<[P3; 3]> = vec![[[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0]]];
        let sheet = mesh::TopoFace { id: 0, tris: vec![0], normal: [0.0, 0.0, 1.0], area: 5e5 };
        let out = push(&tris, &sheet, 500.0).unwrap();
        assert!(out.tris.len() > 2 && out.measure.is_solid(), "the mesh path raised the sheet into a solid");
        assert_eq!(out.backend, "mesh");
        if cfg!(feature = "kernel-solidcore") {
            assert!(out.fallback.is_some(), "the kernel's refusal is named");
        }
    }
}
