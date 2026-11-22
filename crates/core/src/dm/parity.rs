//! The fork's review: one corpus of shapes, one driver over both push engines, one volume
//! tolerance and one refusal policy. With no maintainer review on the fork, this module *is* the
//! quality gate — a kernel change that answers differently on any listed shape is a red test, and
//! a refusal is asserted (never printed into an always-passing test).

use crate::mesh::{self, TopoFace};
use crate::vec3::P3;

/// The engine agreement tolerance, in mm³. Both engines move the picked face, so a solid's volume
/// after a push must match to this — anything larger is a divergence the corpus catches.
pub const VOLUME_TOL: f64 = 1.0;

/// A box a metre each way, its triangles turned out: eight points, twelve triangles.
pub fn box_tris() -> Vec<[P3; 3]> {
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

/// A U-shaped solid along x: a bottom slab x0..1300, two columns on it (x0..500 and x1000..1300),
/// all y0..500 — two disjoint tops in one plane, which only the pick point tells apart.
pub fn u_tris() -> Vec<[P3; 3]> {
    let p: Vec<P3> = vec![
        [0.0, 0.0, 0.0],
        [500.0, 0.0, 0.0],
        [1000.0, 0.0, 0.0],
        [1300.0, 0.0, 0.0],
        [0.0, 0.0, 100.0],
        [500.0, 0.0, 100.0],
        [1000.0, 0.0, 100.0],
        [1300.0, 0.0, 100.0],
        [0.0, 0.0, 500.0],
        [500.0, 0.0, 500.0],
        [1000.0, 0.0, 500.0],
        [1300.0, 0.0, 500.0],
        [0.0, 500.0, 0.0],
        [500.0, 500.0, 0.0],
        [1000.0, 500.0, 0.0],
        [1300.0, 500.0, 0.0],
        [0.0, 500.0, 100.0],
        [500.0, 500.0, 100.0],
        [1000.0, 500.0, 100.0],
        [1300.0, 500.0, 100.0],
        [0.0, 500.0, 500.0],
        [500.0, 500.0, 500.0],
        [1000.0, 500.0, 500.0],
        [1300.0, 500.0, 500.0],
    ];
    [
        [0usize, 13, 1],
        [0, 12, 13],
        [1, 14, 2],
        [1, 13, 14],
        [2, 15, 3],
        [2, 14, 15],
        [8, 9, 21],
        [8, 21, 20],
        [10, 11, 23],
        [10, 23, 22],
        [5, 6, 18],
        [5, 18, 17],
        [5, 17, 21],
        [5, 21, 9],
        [18, 6, 10],
        [18, 10, 22],
        [12, 0, 4],
        [12, 4, 16],
        [16, 4, 8],
        [16, 8, 20],
        [3, 15, 19],
        [3, 19, 7],
        [7, 19, 23],
        [7, 23, 11],
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [4, 5, 9],
        [4, 9, 8],
        [6, 7, 11],
        [6, 11, 10],
        [12, 17, 13],
        [12, 16, 17],
        [13, 18, 14],
        [13, 17, 18],
        [14, 19, 15],
        [14, 18, 19],
        [16, 21, 17],
        [16, 20, 21],
        [18, 23, 19],
        [18, 22, 23],
    ]
    .iter()
    .map(|t| [p[t[0]], p[t[1]], p[t[2]]])
    .collect()
}

/// The kernel engine's answer, when this build has it. The mesh path is always present.
pub fn kernel_push(tris: &[[P3; 3]], face: &TopoFace, by: f64) -> Result<Vec<[P3; 3]>, String> {
    #[cfg(feature = "kernel-solidcore")]
    {
        crate::dm::kernel::push(tris, face, by)
    }
    #[cfg(not(feature = "kernel-solidcore"))]
    {
        let _ = (tris, face, by);
        Err("this build has no kernel".into())
    }
}

/// The one refusal policy. Where both engines answer they must agree to [`VOLUME_TOL`]; where the
/// kernel refuses, the mesh must still answer (the command falls back), or both refuse together —
/// but never silently, and never the kernel alone answering where the mesh cannot.
pub fn agree(name: &str, tris: &[[P3; 3]], face: &TopoFace, by: f64, want: f64) {
    let mesh = mesh::push(tris, face, by);
    let kernel = kernel_push(tris, face, by);
    match (mesh, kernel) {
        (Ok(mo), Ok(ko)) => {
            let mv = mesh::measure(&mo).unwrap();
            let kv = mesh::measure(&ko).unwrap();
            assert!(mv.is_solid() && kv.is_solid(), "{name}: both engines must close a solid");
            assert!((mv.volume - want).abs() <= VOLUME_TOL, "{name}: mesh volume {}, want {want}", mv.volume);
            assert!((mv.volume - kv.volume).abs() <= VOLUME_TOL, "{name}: engines disagree, mesh {} kernel {}", mv.volume, kv.volume);
        }
        (Ok(mo), Err(why)) => {
            // The kernel's refusal is the command's fallback reason; the mesh is the answer it leaves.
            let mv = mesh::measure(&mo).unwrap();
            assert!(mv.is_solid() && (mv.volume - want).abs() <= VOLUME_TOL, "{name}: mesh fell back badly ({why}): {}", mv.volume);
            let _ = crate::dm::push(tris, face, by).expect("the one seam answers where the kernel refuses");
        }
        (Err(mesh_why), Err(kernel_why)) => {
            // Both raw engines refuse. The seam clamps to a valid solid or refuses; it can never
            // return a broken result, and that is what the command relies on here.
            if let Ok(out) = crate::dm::push(tris, face, by) {
                assert!(out.measure.is_solid(), "{name}: the seam's clamp must close: {mesh_why} / {kernel_why}");
            }
        }
        (Err(mesh_why), Ok(_)) => {
            panic!("{name}: the kernel answered where the mesh refuses — a divergence the corpus forbids: {mesh_why}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_box_pushes_through_both_engines_and_they_agree() {
        let tris = box_tris();
        let top = mesh::faces(&tris).into_iter().find(|f| f.normal[2] > 0.99).unwrap();
        agree("box +500", &tris, &top, 500.0, 1.5e9);
        agree("box -2000", &tris, &top, -2000.0, 0.0);
    }

    #[test]
    fn the_u_s_disjoint_tops_are_moved_by_the_pick_point_on_both_engines() {
        let tris = u_tris();
        assert!(mesh::measure(&tris).unwrap().is_solid(), "the U is a solid");
        for (at, rise, want) in [([250.0, 250.0, 500.0], 200.0, 2.75e8), ([1150.0, 250.0, 500.0], 150.0, 2.475e8)] {
            let face = mesh::face_at(&tris, at).unwrap();
            agree(&format!("U at {at:?}"), &tris, &face, rise, want);
        }
    }

    #[test]
    fn a_move_that_turns_a_solid_inside_out_is_refused_by_both_engines() {
        let tris = box_tris();
        let top = mesh::faces(&tris).into_iter().find(|f| f.normal[2] > 0.99).unwrap();
        // The raw `mesh::push` refuses an outright inversion; the seam clamps it, so the two
        // questions differ by design — this case pins the engines' own refusals, not the clamp.
        assert!(mesh::push(&tris, &top, -1e5).is_err());
        assert!(kernel_push(&tris, &top, -1e5).is_err());
    }
}
