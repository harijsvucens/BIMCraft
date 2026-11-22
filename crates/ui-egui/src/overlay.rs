//! The one display question every overlay asks: these triangles, this camera, this look — what paints, in what order,
//! which edges. Push/Pull, the Direct Model box, Move Mesh and the release stand-in all paint through here, so culling,
//! shading, painter sorting and edge selection are answered once and inherited by every future overlay (M5 ghosts, M6
//! gizmo, M7 previews). It stays in the fork's own module rather than `view3d.rs`, so a release merge does not grow a
//! conflict in the renderer file official main keeps changing.

use bimcraft_core::graphics::Look;
use bimcraft_core::model::Id;
use bimcraft_core::render::Rgb;
use bimcraft_core::vec3::{cross, dot, sub};
use bimcraft_core::view3d::{self, Camera, P3};
use egui::{Color32, Painter, Pos2, Stroke};

use crate::canvas::color;

/// Triangles and edges ready for an overlay, each with the colour the picture gives it.
pub(crate) type Painted = Vec<([P3; 3], Color32)>;
pub(crate) type Edged = Vec<([P3; 2], Color32)>;

/// The display question: the shaded fill and the front edges of one mesh, in far-to-near order, with the picture's own
/// light. `world` places the form in the model, so a rotated or tilted component is culled and sorted where it stands.
pub(crate) fn paint(tris: &[[P3; 3]], world: &dyn Fn(P3) -> P3, cam: &Camera, id: Id, base: Rgb, look: &Look) -> (Painted, Edged) {
    let drawn: Vec<([P3; 3], Rgb)> = front_faces(tris, world, cam).into_iter().map(|i| (tris[i], base)).collect();
    let edge = color(look.edge(id, view3d::EDGE));
    let edged = front_edges(bimcraft_core::mesh::edges(tris), tris, world, cam).into_iter().map(|e| (e.map(world), edge)).collect();
    let painted = far_first(drawn, cam, world)
        .into_iter()
        .map(|(t, base)| {
            let placed = t.map(world);
            (placed, shaded(&placed, cam, id, base, look))
        })
        .collect();
    (painted, edged)
}

/// The shaded triangles of an overlay, in the order they were painted: one egui mesh, so a shared diagonal gets no
/// per-triangle seam.
pub(crate) fn mesh_shape(p: &Painter, painted: &[([P3; 3], Color32)], screen: &dyn Fn(P3) -> Pos2) {
    let mut mesh = egui::Mesh::default();
    for (t, c) in painted {
        let pts: Vec<Pos2> = t.iter().map(|q| screen(*q)).collect();
        if pts.iter().all(|s| s.is_finite()) {
            let first = mesh.vertices.len() as u32;
            for q in pts {
                mesh.colored_vertex(q, *c);
            }
            mesh.add_triangle(first, first + 1, first + 2);
        }
    }
    p.add(egui::Shape::mesh(mesh));
}

/// The front edges of an overlay, in the view's own edge colour.
pub(crate) fn edge_lines(p: &Painter, edged: &[([P3; 2], Color32)], screen: &dyn Fn(P3) -> Pos2, ppp: f32) {
    for ([a, b], c) in edged {
        let (a, b) = (screen(*a), screen(*b));
        if a.is_finite() && b.is_finite() {
            p.line_segment([a, b], Stroke::new(1.0 / ppp.max(1.0), *c));
        }
    }
}

/// The colour a world-space triangle is shaded in the view: the core's one answer (`view3d::shade`, the picture's own),
/// turned to an egui colour.
fn shaded(t: &[P3; 3], cam: &Camera, id: Id, base: Rgb, look: &Look) -> Color32 {
    let Rgb(r, g, b) = view3d::shade(t, cam, id, base, look);
    Color32::from_rgb(r, g, b)
}

/// Toward the camera, from the target: the core's `Camera::axes` is private to `bimcraft-core`, so rebuild the one axis
/// this module needs from the same two angles (clamped as the core clamps them).
pub(crate) fn back_axis(cam: &Camera) -> P3 {
    let (a, e) = (cam.azimuth.to_radians(), cam.elevation.clamp(-89.0, 89.0).to_radians());
    [e.cos() * a.cos(), e.cos() * a.sin(), e.sin()]
}

/// The triangles in the order an overlay with no depth buffer must paint them, each with what paints it: farthest from
/// the eye first. `world` places the family's axes in the model, so a rotated or tilted component is ordered where it
/// stands. The key is the placed centroid's *linear* depth along the view axis: `project().2` is non-linear in
/// perspective and `-inf` behind the eye, which can sort a far bottom triangle after a near front one. A triangle with
/// a vertex behind the eye is dropped — it cannot be painted on this picture at all.
pub(crate) fn far_first<T>(mut tris: Vec<([P3; 3], T)>, cam: &Camera, world: &dyn Fn(P3) -> P3) -> Vec<([P3; 3], T)> {
    let back = back_axis(cam);
    let depth = |t: &[P3; 3]| {
        let c: P3 = std::array::from_fn(|i| t.iter().map(|q| world(*q)[i]).sum::<f64>() / 3.0);
        c[0] * back[0] + c[1] * back[1] + c[2] * back[2]
    };
    tris.retain(|(t, _)| {
        t.iter().all(|q| {
            let placed = world(*q);
            cam.in_front(placed, placed).is_some()
        })
    });
    tris.sort_by(|a, b| depth(&a.0).total_cmp(&depth(&b.0)));
    tris
}

/// Which triangles of a sealed solid face the camera, as indices into `tris`. One answer for the fill, the edges and
/// the release stand-in, so a back face can never be painted over a front one. An open sheet has no outward normal:
/// both sides are meant to be seen, so every triangle is kept. `world` places the family's axes, so a mirrored or
/// turned placement carries its winding — the normal is read from the placed corners, and the solid's signed volume
/// turns the normals back outward when the placement mirrors.
fn front_faces(tris: &[[P3; 3]], world: &dyn Fn(P3) -> P3, cam: &Camera) -> Vec<usize> {
    let placed: Vec<[P3; 3]> = tris.iter().map(|t| t.map(world)).collect();
    if bimcraft_core::mesh::sealed(tris).is_err() {
        return (0..tris.len()).collect();
    }
    let origin = placed.first().map_or([0.0; 3], |t| t[0]);
    let volume6: f64 = placed.iter().map(|[a, b, c]| dot(sub(*a, origin), cross(sub(*b, origin), sub(*c, origin)))).sum();
    let outward = if volume6 < 0.0 { -1.0 } else { 1.0 };
    (0..placed.len())
        .filter(|&i| {
            let t = placed[i];
            let normal = cross(sub(t[1], t[0]), sub(t[2], t[0]));
            dot(normal, cam.toward(t[0])) * outward > 0.0
        })
        .collect()
}

/// The edges a shaded overlay may draw: only those on a front face [`front_faces`] keeps. A sealed solid's back edges
/// are hidden by its front faces in the picture; an overlay has no depth buffer, so it must leave them out rather
/// than stroke them over its own front. An open sheet keeps all of them.
pub(crate) fn front_edges(edges: Vec<[P3; 2]>, tris: &[[P3; 3]], world: &dyn Fn(P3) -> P3, cam: &Camera) -> Vec<[P3; 2]> {
    let front: Vec<[P3; 3]> = front_faces(tris, world, cam).into_iter().map(|i| tris[i].map(world)).collect();
    // The edges come from these triangles, so one on a front face has its middle on one of that face's sides;
    // the collinear test keeps it if a split put the middle in the middle of a longer side.
    let eps2 = 0.01 * 0.01;
    let on_front = |edge: &[P3; 2]| {
        let w = edge.map(world);
        let mid = std::array::from_fn::<f64, 3, _>(|i| (w[0][i] + w[1][i]) * 0.5);
        front.iter().any(|t| {
            (0..3).any(|k| {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let ab = sub(b, a);
                let len2 = dot(ab, ab);
                if len2 == 0.0 {
                    return false;
                }
                let s = dot(sub(mid, a), ab) / len2;
                if !(0.0..=1.0).contains(&s) {
                    return false;
                }
                let gap = sub(mid, std::array::from_fn::<f64, 3, _>(|i| a[i] + s * ab[i]));
                dot(gap, gap) <= eps2
            })
        })
    };
    edges.into_iter().filter(|e| on_front(e)).collect()
}
