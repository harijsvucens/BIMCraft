//! Hit tests over [`TessCache`]. A caller gets the seam's own ids, never a solidcore number,
//! so a face re-matched after an edit and a face picked before it are the same kind of handle.

use super::kernel::{EdgeId, FaceId, TessCache, VertId};
use crate::vec3::P3;

/// What an ordered, multi-selection holds. `Body` is an index into `DirectDraft::bodies`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelItem {
    Vertex(VertId),
    Edge(EdgeId),
    Face(FaceId),
    Body(u64),
}

/// A hit and the world point on it, which an operation card's numeric fields start from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Picked {
    pub item: SelItem,
    pub at: P3,
}

fn dist2(a: P3, b: P3) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)
}

fn point_segment(p: P3, a: P3, b: P3) -> (f64, P3) {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = ab.iter().map(|v| v * v).sum::<f64>();
    let t = if len2 == 0.0 { 0.0 } else { ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / len2 }.clamp(0.0, 1.0);
    let at = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
    (dist2(p, at).sqrt(), at)
}

/// The nearest vertex within `tol`.
pub fn vertex(cache: &TessCache, at: P3, tol: f64) -> Option<Picked> {
    let (id, _, d) = cache.vertices.iter().enumerate().fold((None, at, f64::INFINITY), |best, (i, p)| {
        let d = dist2(at, *p).sqrt();
        if d < best.2 { (Some(i), *p, d) } else { best }
    });
    let id = id?;
    (d <= tol).then_some(Picked { item: SelItem::Vertex(VertId(id as u32)), at })
}

/// The nearest point on any edge polyline within `tol`.
pub fn edge(cache: &TessCache, at: P3, tol: f64) -> Option<Picked> {
    let mut best: Option<(f64, P3, EdgeId)> = None;
    for (i, line) in cache.edges.iter().enumerate() {
        for q in line.windows(2) {
            let (d, on) = point_segment(at, q[0], q[1]);
            if best.is_none_or(|(b, _, _)| d < b) {
                best = Some((d, on, EdgeId(i as u32)));
            }
        }
    }
    let (d, on, id) = best?;
    (d <= tol).then_some(Picked { item: SelItem::Edge(id), at: on })
}

/// The record face a tessellated triangle belongs to. The 3D view's picker already gives the
/// triangle index, so this is the one face question the UI asks on a click.
pub fn face_of_triangle(cache: &TessCache, tri: usize) -> Option<FaceId> {
    cache.tri_face.get(tri).copied()
}

/// The closest vertex, else edge, for a world point: vertex wins ties, as in the rest of the app.
pub fn at(cache: &TessCache, at: P3, tol: f64) -> Option<Picked> {
    vertex(cache, at, tol).or_else(|| edge(cache, at, tol))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dm::kernel::{Kernel, Prim};

    fn box_cache() -> TessCache {
        Kernel::create(Prim::Box { origin: [0.0; 3], size: [1000.0, 800.0, 600.0] }).expect("box").cache
    }

    #[test]
    fn a_box_indexes_its_eight_vertices_twelve_edges_and_six_faces() {
        let c = box_cache();
        assert_eq!(c.vertices.len(), 8, "{:?}", c.vertices);
        assert_eq!(c.edges.len(), 12);
        assert_eq!(c.faces.len(), 6);
        assert_eq!(c.tri_edge.len(), c.triangles.len());
        assert_eq!(c.edge_vert.len(), c.edges.len());
        assert_eq!(c.face_edges.len(), c.faces.len());
        assert!(c.face_edges.iter().all(|list| !list.is_empty() && list.windows(2).all(|w| w[0] < w[1])));
        assert!(c.edge_vert.iter().all(|pair| pair[0] != pair[1] && pair.iter().all(|v| (v.0 as usize) < c.vertices.len())));
    }

    #[test]
    fn a_corner_pick_returns_a_vertex_and_a_mid_edge_pick_an_edge() {
        let c = box_cache();
        let corner = vertex(&c, [0.0, 0.0, 0.0], 0.01).expect("the corner");
        assert!(matches!(corner.item, SelItem::Vertex(_)));
        assert!(at(&c, [0.0, 0.0, 0.0], 0.01).is_some_and(|p| p.item == corner.item));
        let mid = edge(&c, [500.0, 0.0, 0.0], 0.01).expect("the bottom edge");
        assert!(matches!(mid.item, SelItem::Edge(_)));
        assert!(face_of_triangle(&c, 0).is_some());
        assert!(face_of_triangle(&c, c.triangles.len()).is_none());
    }
}
