//! The Push/Pull gesture (MOD-04): what a drag on a mesh face measures, what the view shows while it runs, and what
//! letting go commits — one file the next slice reads whole. The press's protocol is `drag.rs`'s own; the drive call
//! and the hide sit in `orbit::view3d`, beside every other gesture's drive, and read this module's answers.

use bimcraft_core::graphics::{Look, VisualStyle};
use bimcraft_core::model::{Element, Id, Model};
use bimcraft_core::render::Rgb;
use bimcraft_core::view3d::{self, Camera, P3, Surface};
use egui::{Color32, Painter, Pos2, Stroke, vec2};

use crate::Tool;
use crate::canvas::color;
use crate::drag::Held;
use crate::style::pal;
use serde_json::json;

/// What a Push/Pull drag measures: the face the pick found, pushed as far as the mesh allows — the clamp and the
/// triangles together, so the preview shows and the release commits the same thing. The clamp itself is the core's
/// (`mesh::push_clamped`); this is the model's way to it, from the pick, through the form the push reaches
/// (`Model::push_form`).
pub(crate) fn measure(model: &Model, id: Id, at: P3, by: f64) -> Option<(f64, Vec<[P3; 3]>)> {
    let local = model.form_point(id, at)?;
    let (n, family) = model.push_form(id)?;
    let form = &family.forms[n];
    let face = bimcraft_core::mesh::face_at(&form.mesh, local).ok()?;
    Some(bimcraft_core::mesh::push_clamped(&form.mesh, &face, by))
}

/// What the drag hides while its preview runs: the whole component, so the picture drops every form of its family
/// and [`paint`] stands in for them.
pub(crate) fn hidden(push_drag: &Option<Held<(Id, P3, P3)>>, measured: bool) -> Option<Id> {
    push_drag.as_ref().filter(|_| measured).map(|h| h.target().0)
}

/// The view's facts the gesture's paint reads, as `orbit::view3d` holds them for the frame.
pub(crate) struct Paint<'a> {
    pub model: &'a Model,
    pub cam: &'a Camera,
    pub look3: &'a Look,
    pub p: &'a Painter,
    /// A model point on screen, as the view draws it.
    pub screen: &'a dyn Fn(P3) -> Pos2,
    /// What the picture shows at a screen point, the same surface a click would take.
    pub under: &'a dyn Fn(Pos2) -> Option<Surface>,
    pub ppp: f32,
    /// Where the pointer is, whether another drag holds the view, and whether a work plane is being set or picked.
    pub hover: Option<Pos2>,
    pub drag_idle: bool,
    pub sketch_free: bool,
}

/// The gesture's paint: the face held — or, before one is held, the one under the cursor, so the pick looks like it
/// will land — filled where it stands, the pushed mesh shaded as the model is while a drag runs, and what's typed
/// beside the face. The face comes back through the same `form_point` the command will take, and a hover lights only
/// what a click would take: the same surface the click asks, gated by `Model::push_form`.
/// shortcut: the face is re-derived from the point every frame (a weld over the form's triangles), carried from
/// the pick instead by M0.2's face id; a held Direct Model box is 12 triangles, a 100k import is not.
pub(crate) fn paint(tool: &Tool, typed: &str, push_drag: &Option<Held<(Id, P3, P3)>>, push_by: Option<(f64, Vec<[P3; 3]>)>, v: &Paint) {
    let lit_face = match tool {
        Tool::PushPull(Some((id, at))) => Some((*id, *at)),
        Tool::PushPull(None) if v.sketch_free => {
            v.hover.filter(|_| v.drag_idle).and_then(|at| (v.under)(at)).filter(|s| v.model.push_form(s.id).is_some()).map(|s| (s.id, s.at))
        }
        _ => None,
    };
    let Some((id, at)) = lit_face else { return };
    let Some(local) = v.model.form_point(id, at) else { return };
    let Some(Element::Component(c)) = v.model.elements.get(id) else { return };
    let Some((form_no, family)) = v.model.push_form(id) else { return };
    let Some(form) = family.forms.get(form_no) else { return };
    let placement = v.model.component_placement(c);
    let world = |q: P3| placement.place(if c.mirrored { [q[0], -q[1], q[2]] } else { q });
    let in_style = v.look3.style() == VisualStyle::Shaded;
    let [r, g, b, _] = pal().accent.to_array();
    // A quarter's accent: the face keeps its own colour, yet the pick still reads on a light and a dark canvas.
    let fill = egui::Color32::from_rgba_unmultiplied(r, g, b, 25);
    // The face held or under the cursor: a quiet promise of the pick, in the view's own style. Once the drag
    // measures a distance the moving mesh is the feedback, and the old face is out of the picture — so the
    // face is derived (a weld over the form) only while no drag runs; the drag's own measure has the geometry.
    if in_style
        && push_drag.is_none()
        && let Ok(face) = bimcraft_core::mesh::face_at(&form.mesh, local)
    {
        for i in &face.tris {
            let pts: Vec<Pos2> = form.mesh[*i].iter().map(|q| (v.screen)(world(*q))).collect();
            if pts.iter().all(|s| s.is_finite()) {
                v.p.add(egui::Shape::convex_polygon(pts, fill, Stroke::NONE));
            }
        }
    }
    // Where the face would land if the drag let go here: the very mesh the commit will make — what the drag's
    // measure already clamped and returned — shaded as the model is and edged where its faces meet, so a sheet
    // rising into a solid reads as the solid it becomes.
    if let (Some((by, out)), Some(h)) = (push_by.filter(|_| push_drag.is_some()), push_drag.clone()) {
        let n = h.target().2;
        let base = v.model.materials.get(&form.material).map_or(view3d::COMPONENT, |m| Rgb(m.color[0], m.color[1], m.color[2]));
        // The hide is the whole component, so the overlay redraws every mesh form of the family, not only the
        // one the push reaches: a two-material import keeps its other part in the picture while one is pushed.
        let mut drawn: Vec<([P3; 3], Rgb)> = out.iter().map(|t| (*t, base)).collect();
        let mut edges = bimcraft_core::mesh::edges(&out);
        for (j, other) in family.forms.iter().enumerate() {
            if j == form_no || other.mesh.is_empty() {
                continue;
            }
            let other_base = v.model.materials.get(&other.material).map_or(view3d::COMPONENT, |m| Rgb(m.color[0], m.color[1], m.color[2]));
            drawn.extend(other.mesh.iter().map(|t| (*t, other_base)));
            edges.extend(bimcraft_core::mesh::edges(&other.mesh));
        }
        if in_style {
            // The whole component, not just what changed: the component itself is hidden, so every face
            // must be drawn. egui paints in add order, so far faces go first — an overlay has no depth buffer.
            for (t, base) in far_first(drawn, v.cam, &world) {
                let pts: Vec<Pos2> = t.iter().map(|q| (v.screen)(world(*q))).collect();
                if pts.iter().all(|s| s.is_finite()) {
                    let world_t = t.map(world);
                    v.p.add(egui::Shape::convex_polygon(pts, shaded(&world_t, v.cam, id, base, v.look3), Stroke::NONE));
                }
            }
        }
        let edge = color(v.look3.edge(id, view3d::EDGE));
        for [a, b] in edges {
            let (a, b) = ((v.screen)(world(a)), (v.screen)(world(b)));
            if a.is_finite() && b.is_finite() {
                v.p.line_segment([a, b], Stroke::new(1.0 / v.ppp.max(1.0), edge));
            }
        }
        let read = (v.screen)([at[0] + n[0] * by, at[1] + n[1] * by, at[2] + n[2] * by]);
        if read.is_finite() {
            crate::canvas::tag(v.p, read + vec2(12.0, -12.0), egui::Align2::LEFT_BOTTOM, v.model.units.with_unit(by));
        }
    }
    // A held face reads its distance from the first frame, zero until digits arrive: the tag is the tool's
    // feedback while it waits, not a reward for typing. A hover has no pick to measure yet, so no tag there.
    if matches!(tool, Tool::PushPull(Some(_))) {
        let text = if typed.is_empty() {
            v.model.units.with_unit(0.0)
        } else {
            v.model.units.parse(typed).map_or_else(|| typed.to_string(), |d| v.model.units.with_unit(d))
        };
        crate::canvas::tag(v.p, (v.screen)(at) + vec2(12.0, -12.0), egui::Align2::LEFT_BOTTOM, text);
    }
}

/// The colour a world-space triangle is shaded in the view: the core's one answer (`view3d::shade`, the picture's own),
/// turned to an egui colour.
fn shaded(t: &[P3; 3], cam: &Camera, id: Id, base: Rgb, look: &Look) -> Color32 {
    let Rgb(r, g, b) = view3d::shade(t, cam, id, base, look);
    Color32::from_rgb(r, g, b)
}

/// The triangles in the order an overlay with no depth buffer must paint them, each with what paints it: farthest from
/// the eye first. `world` places the family's axes in the model, so a rotated or tilted component is ordered where it stands.
fn far_first<T>(mut tris: Vec<([P3; 3], T)>, cam: &Camera, world: &dyn Fn(P3) -> P3) -> Vec<([P3; 3], T)> {
    let near = |t: &[P3; 3]| t.iter().map(|q| cam.project(world(*q)).2).sum::<f64>();
    tris.sort_by(|a, b| near(&a.0).total_cmp(&near(&b.0)));
    tris
}

impl crate::BimApp {
    /// A Push/Pull drag let go: the face moves by what the drag showed. Esc or a tool switch during the drag let
    /// go of the pick first, and the push goes with it — keeping whatever tool the user chose instead.
    pub(crate) fn push_release(&mut self, pushed: Option<(Id, P3, f64)>) {
        let Some((id, at, by)) = pushed else { return };
        let Tool::PushPull(Some((held, _))) = &self.tool else { return };
        if held != &id {
            return;
        }
        // The release commits exactly what the preview drew, clamp and all: the drag's measure returned the
        // distance, so there is nothing to re-derive here.
        self.run("mesh_push_pull", json!({"component": id, "at": at, "distance": by}));
        self.tool = Tool::PushPull(None);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bimcraft_core::view3d::Camera;

    #[test]
    fn far_to_near_orders_the_placed_triangles_not_the_family_axes() {
        let cam = Camera { azimuth: -45.0, elevation: 30.0, scale: 0.5, target: [0.0; 3], width: 640, height: 480, lens: None };
        let east = [[100.0, 0.0, 0.0], [110.0, 0.0, 0.0], [100.0, 10.0, 0.0]];
        let west = [[-100.0, 0.0, 0.0], [-110.0, 0.0, 0.0], [-100.0, 10.0, 0.0]];
        // A half turn about z puts the family's east triangle at the model's west, farther from the eye.
        let turn = |q: P3| [-q[0], -q[1], q[2]];
        let order = far_first(vec![(west, ()), (east, ())], &cam, &turn);
        assert_eq!(order[0].0, east, "the triangle farther after placement must be painted first");
    }
}
