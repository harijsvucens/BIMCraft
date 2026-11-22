//! The Move Mesh gesture: take a point or an edge of a whole mesh component in the 3D view, drag it, and let go to put the
//! component there. A rigid move changes only where the component stands — its family's mesh is untouched, so the kernel
//! has no part in it (that stays Push/Pull's engine). The drag's measure is a translation of the component's placed mesh;
//! the release commits the same step with one `mesh_move` command. The grab and the drop both snap: the first to the held
//! object's own vertices and edges, the second to every other element's corners and edges the scene can see.

use bimcraft_core::graphics::Look;
use bimcraft_core::mesh;
use bimcraft_core::model::{Element, Id, Model};
use bimcraft_core::render::Rgb;
use bimcraft_core::units::Units;
use bimcraft_core::view3d::{self, Camera, P3, Scene};
use egui::{Color32, Painter, Pos2, Stroke, vec2};

use crate::canvas;
use crate::style::pal;

/// What one frame of a Move Mesh drag measured: the step from where the grab stood, where that grab points now, and the
/// picture of the whole component at its new place (every form of its family, hidden while the overlay paints them).
#[derive(Clone)]
pub(crate) struct Preview {
    pub by: P3,
    pub target: P3,
    pub painted: Vec<([P3; 3], Color32)>,
    pub edged: Vec<([P3; 2], Color32)>,
}

/// A point of the component's placed mesh within a snap's reach of pixel `at`: a vertex first, else the nearest point of
/// one of its edges. This is what the pointer takes hold of, so the object follows that point exactly.
pub(crate) fn grab(model: &Model, cam: &Camera, id: Id, at: (f64, f64), ppp: f32) -> Option<P3> {
    let (n, family) = model.push_form(id)?;
    let Element::Component(c) = model.elements.get(id)? else { return None };
    let form = family.forms.get(n)?;
    let placement = model.component_placement(c);
    let world = |q: P3| placement.place(if c.mirrored { [q[0], -q[1], q[2]] } else { q });
    let reach = canvas::SNAP * f64::from(ppp);
    let screen = |q: P3| {
        let (x, y, _) = cam.project(q);
        (x, y)
    };
    let mut best: Option<(f64, P3)> = None;
    for t in form.mesh.iter() {
        for q in t {
            let p = world(*q);
            let (x, y) = screen(p);
            let d = (x - at.0).hypot(y - at.1);
            if d.is_finite() && d <= reach && best.is_none_or(|(b, _)| d < b) {
                best = Some((d, p));
            }
        }
    }
    if best.is_some() {
        return best.map(|(_, p)| p);
    }
    for [a, b] in mesh::edges(&form.mesh) {
        let (pa, pb) = (world(a), world(b));
        let ((x0, y0, _), (x1, y1, _)) = (cam.project(pa), cam.project(pb));
        let (dx, dy) = (x1 - x0, y1 - y0);
        let len2 = dx * dx + dy * dy;
        let t = if len2 > 0.0 { (((at.0 - x0) * dx + (at.1 - y0) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
        let d = (x0 + dx * t - at.0).hypot(y0 + dy * t - at.1);
        if d.is_finite() && d <= reach && best.is_none_or(|(b, _)| d < b) {
            best = Some((d, std::array::from_fn(|i| pa[i] + (pb[i] - pa[i]) * t)));
        }
    }
    best.map(|(_, p)| p)
}

/// Where the moving grab lands: the nearest corner of what the view shows, else the nearest edge's point, within a snap's
/// reach of pixel `at`. The component being moved is hidden from this look, so it cannot snap to its own old place.
pub(crate) fn snap(scene: &Scene, cam: &Camera, at: (f64, f64), look: Look, moving: Id, ppp: f32) -> Option<P3> {
    let hidden = look.with_hidden(moving);
    let reach = canvas::SNAP * f64::from(ppp);
    scene
        .corner_near(cam, at, reach, &hidden)
        .or_else(|| scene.edge_near(cam, at, f64::from(canvas::PICK) * f64::from(ppp), &hidden).map(|(_, p)| p))
}

/// The component's whole family translated by `by`, shaded and front-culled exactly as the view paints it: the preview and
/// the release stand in for every form, as Push/Pull's does. `grab` is where the held point stood, for the reading.
pub(crate) fn measure(model: &Model, id: Id, grab: P3, by: P3, cam: &Camera, look: &Look) -> Option<Preview> {
    let (n, family) = model.push_form(id)?;
    let Element::Component(c) = model.elements.get(id)? else { return None };
    let form = family.forms.get(n)?;
    let placement = model.component_placement(c);
    let world = |q: P3| {
        let p = placement.place(if c.mirrored { [q[0], -q[1], q[2]] } else { q });
        std::array::from_fn(|i| p[i] + by[i])
    };
    let base = |material: &String| model.materials.get(material).map_or(view3d::COMPONENT, |m| Rgb(m.color[0], m.color[1], m.color[2]));
    let (mut painted, mut edged) = crate::overlay::paint(&form.mesh, &world, cam, id, base(&form.material), look);
    for (j, other) in family.forms.iter().enumerate() {
        if j == n || other.mesh.is_empty() {
            continue;
        }
        let (p, e) = crate::overlay::paint(&other.mesh, &world, cam, id, base(&other.material), look);
        painted.extend(p);
        edged.extend(e);
    }
    let target = std::array::from_fn(|i| grab[i] + by[i]);
    Some(Preview { by, target, painted, edged })
}

/// The marker at the point being moved and its step, drawn over the picture (which hides the component while a drag runs).
pub(crate) fn paint(p: &Painter, target: P3, by: P3, units: Units, screen: &dyn Fn(P3) -> Pos2) {
    let at = screen(target);
    if !at.is_finite() {
        return;
    }
    let d = [vec2(0.0, -6.0), vec2(6.0, 0.0), vec2(0.0, 6.0), vec2(-6.0, 0.0)].map(|q| at + q).to_vec();
    p.add(egui::Shape::closed_line(d, Stroke::new(2.0, pal().accent)));
    let len = by.iter().map(|v| v * v).sum::<f64>().sqrt();
    if len > 0.0 {
        canvas::tag(p, at + vec2(12.0, -12.0), egui::Align2::LEFT_BOTTOM, units.with_unit(len));
    }
}

impl crate::BimApp {
    /// A Move Mesh drag let go: put the component where the grabbed point ended up. The preview's step is exactly what the
    /// release measured, so there is nothing to re-derive. True when the command succeeded, for the rebuild stand-in.
    pub(crate) fn move_release(&mut self, id: Id, by: P3) -> bool {
        let Some(guid) = self.session.model.elements.guid(id) else { return false };
        self.run("mesh_move", serde_json::json!({"component": guid, "by": by})).is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bimcraft_core::engine::Session;
    use serde_json::json;
    use std::sync::Arc;

    fn setup() -> (Session, Id, Arc<bimcraft_core::model::Model>) {
        let mut s = Session::new();
        let made = s
            .execute(
                "mesh",
                &json!({
                    "points": [[0,0,0],[1000,0,0],[1000,1000,0],[0,1000,0],[0,0,1000],[1000,0,1000],[1000,1000,1000],[0,1000,1000]],
                    "triangles": [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
                }),
            )
            .unwrap();
        let id = made["component"].as_u64().unwrap();
        let model = Arc::clone(&s.model);
        (s, id, model)
    }

    /// A camera looking down at the placed box, where its eight corners are well apart on screen.
    fn cam() -> Camera {
        Camera { azimuth: -45.0, elevation: 60.0, scale: 0.3, target: [500.0, 500.0, 500.0], width: 800, height: 600, lens: None }
    }

    #[test]
    fn the_grab_takes_a_corner_of_the_held_object_and_nothing_far_away() {
        let (_s, id, model) = setup();
        let cam = cam();
        let (x, y, _) = cam.project([1000.0, 1000.0, 1000.0]);
        assert_eq!(grab(&model, &cam, id, (x, y), 1.0), Some([1000.0, 1000.0, 1000.0]));
        assert_eq!(grab(&model, &cam, id, (x + 1000.0, y), 1.0), None, "a hundred pixels off is no grab");
    }

    #[test]
    fn a_step_moves_every_form_of_the_component_and_keeps_the_mesh() {
        let (_s, id, model) = setup();
        let cam = cam();
        let by = [500.0, 250.0, 300.0];
        let preview = measure(&model, id, [1000.0, 1000.0, 1000.0], by, &cam, &Look::of_3d(&model)).unwrap();
        assert_eq!(preview.target, [1500.0, 1250.0, 1300.0]);
        let zs = preview.painted.iter().flat_map(|(t, _)| t.iter().map(|q| q[2]));
        assert_eq!(zs.clone().fold(f64::MIN, f64::max), 1300.0, "the top is painted at its new height");
        assert_eq!(zs.fold(f64::MAX, f64::min), 300.0, "the bottom went up with it");
        assert!(!preview.edged.is_empty(), "its edges go with it");
    }
}
