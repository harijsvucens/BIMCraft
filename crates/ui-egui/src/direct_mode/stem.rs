//! The yellow face-normal stem: a short line out of a selected face's centre along its normal.
//! Dragging it starts Push/Pull; the drag is measured by the existing `drag::drive` protocol.

use bimcraft_core::dm::kernel::{FaceId, TessCache};
use bimcraft_core::view3d::P3;

/// How far the handle sticks out of the face, in millimetres.
pub const LENGTH: f64 = 300.0;

fn unit(n: P3) -> Option<P3> {
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    (len > 1e-9).then(|| [n[0] / len, n[1] / len, n[2] / len])
}

/// The stem of one face, from its centroid outward. A face with no normal has no handle.
pub fn line(cache: &TessCache, face: FaceId) -> Option<[P3; 2]> {
    let c = cache.faces.get(face.0 as usize)?.centroid;
    let n = normal(cache, face)?;
    Some([c, [c[0] + n[0] * LENGTH, c[1] + n[1] * LENGTH, c[2] + n[2] * LENGTH]])
}

/// The direction a stem drag measures along.
pub fn normal(cache: &TessCache, face: FaceId) -> Option<P3> {
    unit(cache.faces.get(face.0 as usize)?.normal)
}
