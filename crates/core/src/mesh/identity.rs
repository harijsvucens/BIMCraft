//! Face identity: one owner for numbering, the point→face rule and the probe a kernel matches.
//!
//! A face used to be named by the index of its first triangle, so renumbering the triangles
//! renumbered the faces. Here an id is keyed to the face's least welded corner — geometry, not
//! triangle order — so a face keeps its identity across a gesture that rewrites the triangles, and
//! no `FORMAT` bump is needed because the id is re-derived, never stored. The probe a kernel uses
//! to find the record face is derived here too, so pick, hover, preview and commit name one face.

use super::{Corner, TopoFace};
use crate::vec3::P3;
use std::collections::{HashMap, HashSet};

/// Renumber facet groups by their geometry: each group's id is the rank of its sorted set of welded
/// corners among all groups', so it does not move when triangles are reordered. Two faces may share
/// a corner, so a single corner is not enough — the whole set is the key. `groups` holds the
/// union-find root per triangle (the old id); the returned vector holds the stable id per triangle.
pub(crate) fn stable_ids(corners: &[[Corner; 3]], groups: &[Option<usize>]) -> Vec<Option<usize>> {
    let mut sets: HashMap<usize, HashSet<Corner>> = HashMap::new();
    for (i, group) in groups.iter().enumerate() {
        let Some(root) = *group else { continue };
        let Some(c) = corners.get(i) else { continue };
        let set = sets.entry(root).or_default();
        set.extend(c.iter().copied());
    }
    let mut ordered: Vec<(Vec<Corner>, usize)> = sets
        .into_iter()
        .map(|(root, set)| {
            let mut set: Vec<Corner> = set.into_iter().collect();
            set.sort_unstable();
            (set, root)
        })
        .collect();
    ordered.sort();
    let rank: HashMap<usize, usize> = ordered.into_iter().enumerate().map(|(n, (_, root))| (root, n)).collect();
    groups.iter().map(|g| g.map(|root| rank[&root])).collect()
}

/// The point of `tris` that names `face` to the kernel: the middle of its first triangle, strictly
/// inside it, so it can stand on no other face. The one probe rule the pick and the kernel bridge use.
pub fn probe(tris: &[[P3; 3]], face: &TopoFace) -> Result<P3, String> {
    let i = face.tris.first().ok_or("that face is no face of these triangles")?;
    let t = tris.get(*i).ok_or("that face is no face of these triangles")?;
    Ok(std::array::from_fn(|j| (t[0][j] + t[1][j] + t[2][j]) / 3.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh;

    #[test]
    fn a_face_keeps_its_id_when_the_triangles_are_reordered() {
        let tris = crate::dm::parity::box_tris();
        let before = mesh::faces(&tris);
        let mut moved = tris.clone();
        moved.rotate_left(3);
        let after = mesh::faces(&moved);
        assert_eq!(before.len(), after.len(), "the same six faces");
        for face in &before {
            let same = after.iter().find(|f| f.normal == face.normal).expect("the face is still there");
            assert_eq!(face.id, same.id, "the {:?} face keeps its id across a reorder", face.normal);
        }
    }

    #[test]
    fn the_probe_is_strictly_inside_the_faces_first_triangle() {
        let tris = crate::dm::parity::box_tris();
        let top = mesh::faces(&tris).into_iter().find(|f| f.normal[2] > 0.99).unwrap();
        let at = probe(&tris, &top).unwrap();
        assert!(mesh::holds_at(tris[top.tris[0]], top.normal, at), "the probe names this face");
    }
}
