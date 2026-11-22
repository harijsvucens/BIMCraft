//! The Push/Pull gesture (MOD-04): what a drag on a mesh face measures, what the view shows while it runs, and what
//! letting go commits — one file the next slice reads whole. The press's protocol is `drag.rs`'s own; the drive call
//! and the hide sit in `orbit::view3d`, beside every other gesture's drive, and read this module's answers.

use std::cell::RefCell;
use std::sync::Arc;

use bimcraft_core::graphics::{Look, VisualStyle};
use bimcraft_core::mesh::{self, Measure, TopoFace};
use bimcraft_core::model::{Element, Id, Model};
use bimcraft_core::render::Rgb;
use bimcraft_core::units::Units;
use bimcraft_core::view3d::{self, Camera, P3, Surface};
use egui::{Color32, Painter, Pos2, Rect, Stroke, vec2};

use crate::Tool;
use crate::drag::Held;
use crate::overlay::{edge_lines, mesh_shape, paint as paint_mesh};
use crate::style::pal;
use serde_json::json;

/// A manageable face's cached shape: the triangles it is made of, its plane and its boundary in the form's own axes.
#[derive(Clone)]
struct FaceShape {
    id: usize,
    tris: Vec<usize>,
    normal: P3,
    area: f64,
    outline: Vec<[P3; 2]>,
}

struct CachedFace {
    mesh: *const Vec<[P3; 3]>,
    form: usize,
    shape: FaceShape,
}

thread_local! {
    /// The last face a hover or a press found. A pointer crossing a face must not weld the whole mesh every frame:
    /// the form's `Arc` is the revision, and a cheap in-plane point-in-triangle test says when the cursor has left.
    static FACE_CACHE: RefCell<Option<CachedFace>> = const { RefCell::new(None) };
}

/// Whether `at` still lies on one of the cached face's triangles: the core's one plane-and-surface rule, so what the
/// overlay lights is exactly what a pick would hold and what a push would move.
fn holds(shape: &FaceShape, mesh: &[[P3; 3]], at: P3) -> bool {
    shape.tris.iter().filter_map(|i| mesh.get(*i)).any(|t| mesh::holds_at(*t, shape.normal, at))
}

/// The flat face under a form point and its boundary, welding the mesh only when the cached one no longer holds.
fn face_for(prepared: &mesh::Prepared, form: usize, mesh: &Arc<Vec<[P3; 3]>>, at: P3) -> Option<(TopoFace, Vec<[P3; 2]>)> {
    let ptr = Arc::as_ptr(mesh);
    let hit = FACE_CACHE.with(|c| {
        c.borrow()
            .as_ref()
            .and_then(|cached| (cached.mesh == ptr && cached.form == form && holds(&cached.shape, mesh, at)).then(|| cached.shape.clone()))
    });
    let shape = match hit {
        Some(shape) => shape,
        None => {
            let face = prepared.face_at(mesh, at).ok()?;
            let face_tris: Vec<[P3; 3]> = face.tris.iter().filter_map(|i| mesh.get(*i).copied()).collect();
            let shape =
                FaceShape { id: face.id, tris: face.tris.clone(), normal: face.normal, area: face.area, outline: mesh::edges(&face_tris) };
            FACE_CACHE.with(|c| *c.borrow_mut() = Some(CachedFace { mesh: ptr, form, shape: shape.clone() }));
            shape
        }
    };
    Some((TopoFace { id: shape.id, tris: shape.tris, normal: shape.normal, area: shape.area }, shape.outline))
}

/// What a drag's tag reads: the distance, the solid's new volume and how much the push changed it. One composition for
/// the picture and the test that pins it.
fn drag_reading(units: Units, by: f64, volume: f64, delta: f64) -> String {
    format!("{} · {} · Δ{}", units.with_unit(by), volume_text(units, volume), volume_text(units, delta))
}

/// The volume of a mesh in the project's units, as the Direct Model reading writes it: one formatter for both tools.
pub(crate) fn volume_text(units: Units, mm3: f64) -> String {
    let per_metre = units.area(1e6).0.sqrt();
    let unit = units.area(1e6).1.replace('²', "³");
    format!("{:.2} {}", mm3 / 1e9 * per_metre.powi(3), unit)
}

/// A mesh's area in the project's units, the other half of the Direct Model reading.
pub(crate) fn area_text(units: Units, mm2: f64) -> String {
    let (n, unit) = units.area(mm2);
    format!("{n:.2} {unit}")
}

/// What a Push/Pull drag measures: the face the pick found, pushed as far as the mesh allows — the clamp, the triangles,
/// the mesh's new volume and the picture of it, all worked out once when the drag's distance changes. The preview the
/// overlay draws and the release commits are the same face at the same distance; where the build has the kernel, the
/// release refines that move through the exact B-rep and the commit's triangles are the engine's own.
#[derive(Clone)]
pub(crate) struct Preview {
    pub by: f64,
    pub tris: Vec<[P3; 3]>,
    pub measure: Measure,
    pub delta: f64,
    pub painted: Vec<([P3; 3], Color32)>,
    pub edged: Vec<([P3; 2], Color32)>,
}

pub(crate) fn measure(model: &Model, id: Id, at: P3, by: f64, cam: &Camera) -> Option<Preview> {
    let local = model.form_point(id, at)?;
    let Element::Component(c) = model.elements.get(id)? else { return None };
    let (n, family) = model.push_form(id)?;
    let form = &family.forms[n];
    let prepared = mesh::prepare(&form.mesh).ok()?;
    let (face, _) = face_for(&prepared, n, &form.mesh, local)?;
    // The same push the release commits: clamp, engine choice and all live in the core's one seam,
    // so what the overlay draws is exactly what the command writes. The source is welded once and
    // reused for the face question and the push.
    let landed = bimcraft_core::dm::push_prepared(&prepared, &form.mesh, &face, by).ok()?;
    let (by, tris, after) = (landed.by, landed.tris, landed.measure);
    let before = mesh::measure(&form.mesh).map(|m| m.volume).unwrap_or(0.0);
    let placement = model.component_placement(c);
    let world = |q: P3| placement.place(if c.mirrored { [q[0], -q[1], q[2]] } else { q });
    let base = |material: &String| model.materials.get(material).map_or(view3d::COMPONENT, |m| Rgb(m.color[0], m.color[1], m.color[2]));
    let look = Look::of_3d(model);
    let (mut painted, mut edged) = paint_mesh(&tris, &world, cam, id, base(&form.material), &look);
    for (j, other) in family.forms.iter().enumerate() {
        if j == n || other.mesh.is_empty() {
            continue;
        }
        let (p, e) = paint_mesh(&other.mesh, &world, cam, id, base(&other.material), &look);
        painted.extend(p);
        edged.extend(e);
    }
    Some(Preview { by, tris, measure: after, delta: after.volume - before, painted, edged })
}

/// The face the gesture is acting on this frame: the held drag while it runs, or the pending push whose commit runs at
/// the end of the view. `drag::drive` empties the held slot on the release frame but still hands back what it measured,
/// so that frame must hide and paint exactly the one before it. A quick press, Esc and a tool switch leave no pending
/// push, so they drop the hide and the preview at once.
pub(crate) fn active(drag: &Option<Held<(Id, P3, P3)>>, pushed: &Option<(Id, P3, P3, f64)>, tool: &Tool) -> Option<(Id, P3, P3)> {
    if let Some(held) = drag {
        return Some(*held.target());
    }
    let (id, at, n, _) = pushed.as_ref()?;
    match tool {
        Tool::PushPull(Some((held, _))) if held == id => Some((*id, *at, *n)),
        _ => None,
    }
}

/// What the drag hides while its preview runs: the whole component, so the picture drops every form of its family
/// and [`paint`] stands in for them.
pub(crate) fn hidden(active: &Option<(Id, P3, P3)>, measured: bool) -> Option<Id> {
    active.as_ref().filter(|_| measured).map(|(id, ..)| *id)
}

/// The view's facts the gesture's paint reads, as `orbit::view3d` holds them for the frame.
pub(crate) struct Paint<'a> {
    pub model: &'a Model,
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
    /// The processor's picture took the dragged preview: its ink overlay must stand down or the shade is laid twice.
    pub drafted: bool,
}

/// The gesture's paint: the face held — or, before one is held, the one under the cursor, so the pick looks like it
/// will land — filled where it stands, the pushed mesh shaded as the model is while a drag runs, and what's typed
/// beside the face. The face comes back through the same `form_point` the command will take, and a hover lights only
/// what a click would take: the same surface the click asks, gated by `Model::push_form`.
/// shortcut: the face is re-derived from the point every frame (a weld over the form's triangles), carried from
/// the pick instead by M0.2's face id; a held Direct Model box is 12 triangles, a 100k import is not.
pub(crate) fn paint(tool: &Tool, typed: &str, active: &Option<(Id, P3, P3)>, push_by: &Option<Preview>, v: &Paint) {
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
    // The face under the cursor, or held while it waits for a distance: a quiet promise of the pick. The weld is cached
    // by form, so moving across one face costs a point-in-triangle test, not a weld of the whole mesh.
    if active.is_none()
        && let Ok(prepared) = mesh::prepare(&form.mesh)
        && let Some((face, outline)) = face_for(&prepared, form_no, &form.mesh, local)
    {
        if in_style {
            let [r, g, b, _] = pal().accent.to_array();
            let fill = egui::Color32::from_rgba_unmultiplied(r, g, b, 25);
            for i in &face.tris {
                let Some(t) = form.mesh.get(*i) else { continue };
                let pts: Vec<Pos2> = t.iter().map(|q| (v.screen)(world(*q))).collect();
                if pts.iter().all(|s| s.is_finite()) {
                    v.p.add(egui::Shape::convex_polygon(pts, fill, Stroke::NONE));
                }
            }
        } else {
            // Wireframe and Hidden Line have no fill to speak of: stroke the face's own boundary so the pick still reads.
            let ink = pal().accent;
            for [a, b] in &outline {
                let (a, b) = ((v.screen)(world(*a)), (v.screen)(world(*b)));
                if a.is_finite() && b.is_finite() {
                    v.p.line_segment([a, b], Stroke::new(2.0 / v.ppp.max(1.0), ink));
                }
            }
        }
    }
    // Where the face would land if the drag let go here: the picture the drag's measure made once, already front-culled,
    // shaded and far-to-near sorted. The component is hidden, so that picture includes every mesh form of its family.
    if let (Some(preview), Some(h)) = (push_by.as_ref().filter(|_| active.is_some()), *active) {
        // Wire styles promise only edges: the shade stays out, the front edges stay in. Where the picture took the
        // preview there is nothing to lay over it: the ink would paint the same faces again, X-ray, over the depth test.
        if in_style && !v.drafted {
            mesh_shape(v.p, &preview.painted, v.screen);
        }
        if !v.drafted {
            edge_lines(v.p, &preview.edged, v.screen, v.ppp);
        }
        let n = h.2;
        let read = (v.screen)([at[0] + n[0] * preview.by, at[1] + n[1] * preview.by, at[2] + n[2] * preview.by]);
        if read.is_finite() {
            let text = drag_reading(v.model.units, preview.by, preview.measure.volume, preview.delta);
            crate::canvas::tag(v.p, read + vec2(12.0, -12.0), egui::Align2::LEFT_BOTTOM, text);
        }
    }
    // A held face reads its distance from the first frame, zero until digits arrive: the tag is the tool's
    // feedback while it waits, not a reward for typing. A hover has no pick to measure yet, so no tag there.
    if active.is_none() && matches!(tool, Tool::PushPull(Some(_))) {
        let text = if typed.is_empty() {
            v.model.units.with_unit(0.0)
        } else {
            v.model.units.parse(typed).map_or_else(|| typed.to_string(), |d| v.model.units.with_unit(d))
        };
        crate::canvas::tag(v.p, (v.screen)(at) + vec2(12.0, -12.0), egui::Align2::LEFT_BOTTOM, text);
    }
}

/// The just-committed push, kept over the picture while the scene and the card mesh are rebuilt away from the interface
/// thread. The drag's overlay is egui ink laid over a picture taken with the component hidden; that picture is re-blitted
/// during the wait, so the same shaded mesh and edges are laid on it again or the pushed box blinks away. No distance tag:
/// the gesture is over.
pub(crate) struct StandIn {
    /// The exact model revision this stands in for; any other revision clears it.
    pub(crate) model: Arc<Model>,
    /// The camera the re-blitted picture was drawn with, so the ink lands on it pixel for pixel.
    cam: Camera,
    /// Far-to-near world triangles with the colour the picture shades them.
    painted: Vec<([P3; 3], Color32)>,
    /// Front edges in world space, and their colour.
    edged: Vec<([P3; 2], Color32)>,
}

impl StandIn {
    /// `model` is the revision the drag measured against (placement, materials and the other forms are unchanged by the
    /// push); `after` is the revision the commit just made and the wait will redraw.
    pub(crate) fn build(
        model: &Model,
        after: Arc<Model>,
        cam: &Camera,
        look3: &Look,
        active: (Id, P3, P3),
        out: &[[P3; 3]],
    ) -> Option<Self> {
        let id = active.0;
        let Element::Component(c) = model.elements.get(id)? else { return None };
        let (form_no, family) = model.push_form(id)?;
        let form = family.forms.get(form_no)?;
        let placement = model.component_placement(c);
        let world = |q: P3| placement.place(if c.mirrored { [q[0], -q[1], q[2]] } else { q });
        let base = |material: &String| model.materials.get(material).map_or(view3d::COMPONENT, |m| Rgb(m.color[0], m.color[1], m.color[2]));
        // The hide drops the whole component, so stand in for every form of its family, as the drag's paint did.
        // The same front-face rule the drag's paint uses, so the stand-in shows exactly what the last drag frame showed:
        // no back or bottom face, in the same far-to-near order.
        let (mut painted, mut edged) = paint_mesh(out, &world, cam, id, base(&form.material), look3);
        for (j, other) in family.forms.iter().enumerate() {
            if j == form_no || other.mesh.is_empty() {
                continue;
            }
            let (p, e) = paint_mesh(&other.mesh, &world, cam, id, base(&other.material), look3);
            painted.extend(p);
            edged.extend(e);
        }
        Some(Self { model: after, cam: *cam, painted, edged })
    }

    /// A stand-in for a whole-object Move Mesh release: the component drawn where the drag was taking it, so the
    /// re-blitted picture during the rebuild shows the same thing the overlay showed on the last drag frame. `model` is
    /// the revision the drag measured against; `measure` adds the step in world space, so the family's own placement
    /// (mirror and all) needs no taking apart here.
    pub(crate) fn build_moved(model: &Model, after: Arc<Model>, cam: &Camera, look3: &Look, id: Id, grab: P3, by: P3) -> Option<Self> {
        let preview = crate::move_mesh::measure(model, id, grab, by, cam, look3)?;
        Some(Self { model: after, cam: *cam, painted: preview.painted, edged: preview.edged })
    }

    /// Lay the held mesh over the re-blitted picture, between the picture and the wait's wash.
    pub(crate) fn paint(&self, p: &Painter, rect: Rect, ppp: f32) {
        let screen = |q: P3| {
            let (x, y, _) = self.cam.project(q);
            rect.min + vec2(x as f32 / ppp, y as f32 / ppp)
        };
        mesh_shape(p, &self.painted, &screen);
        edge_lines(p, &self.edged, &screen, ppp);
    }
}

impl crate::BimApp {
    /// A Push/Pull drag let go: the face moves by what the drag showed. Esc or a tool switch during the drag let
    /// go of the pick first, and the push goes with it — keeping whatever tool the user chose instead. True when the
    /// command succeeded, so the caller can hold a stand-in over the rebuild that follows.
    pub(crate) fn push_release(&mut self, pushed: &Option<(Id, P3, P3, f64)>) -> bool {
        // The same rule the hide and the preview use: a pending push for the face the tool still holds. A tool switch
        // mid-drag leaves no pending push, so the chosen tool's release stays its own.
        let Some((id, at, _, by)) = pushed.as_ref().filter(|_| active(&None, pushed, &self.tool).is_some()) else { return false };
        // The release commits exactly what the preview drew, clamp and all: the drag's measure returned the
        // distance, so there is nothing to re-derive here.
        let committed = self.run("mesh_push_pull", json!({"component": id, "at": at, "distance": by})).is_some();
        self.tool = Tool::PushPull(None);
        committed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::overlay::{back_axis, far_first, front_edges};
    use bimcraft_core::engine::Session;
    use bimcraft_core::view3d::Camera;

    #[test]
    fn the_drag_reading_shows_distance_volume_and_its_delta_in_the_projects_units() {
        let units = Units::Millimetres;
        let text = drag_reading(units, 500.0, 1.5e9, 0.5e9);
        assert_eq!(text, "500 mm · 1.50 m³ · Δ0.50 m³");
        assert_eq!(volume_text(Units::Feet, 1e9), "35.31 ft³", "the cube is a cubic metre in feet too");
        assert_eq!(volume_text(Units::Millimetres, -0.25e9), "-0.25 m³", "a shrink reads as a negative delta");
    }

    #[test]
    fn stand_in_edges_carry_the_components_placement() {
        let script = r#"mesh {"points": [[0,0,0],[1000,0,0],[1000,1000,0],[0,1000,0],[0,0,1000],[1000,0,1000],[1000,1000,1000],[0,1000,1000]], "triangles": [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]}"#;
        let mut s = Session::new();
        s.script(script).unwrap();
        let id = *s.model.elements.iter().find(|(_, e)| matches!(e, Element::Component(_))).unwrap().0;
        s.execute("rotate", &json!({"ids": [id], "about": [0, 0], "angle": 45.0})).unwrap();
        let model = Arc::clone(&s.model);
        let cam = Camera { azimuth: -45.0, elevation: 30.0, scale: 0.5, target: [0.0; 3], width: 640, height: 480, lens: None };
        let out = box_tris();
        let standin =
            StandIn::build(&model, Arc::clone(&model), &cam, &Look::of_3d(&model), (id, [500.0, 500.0, 1000.0], [0.0, 0.0, 1.0]), &out)
                .unwrap();
        // Same placement the fill uses: an edge left in the family's axes would land by the origin, not on the box.
        let Element::Component(c) = model.elements.get(id).unwrap() else { panic!("a mesh component was made") };
        let placement = model.component_placement(c);
        let world = |q: P3| placement.place(if c.mirrored { [q[0], -q[1], q[2]] } else { q });
        let local = front_edges(bimcraft_core::mesh::edges(&out), &out, &world, &cam);
        let expected: Vec<[P3; 2]> = local.iter().map(|[a, b]| [world(*a), world(*b)]).collect();
        assert!(!expected.is_empty() && expected != local, "the turn must move the edges, or this proves nothing");
        let painted_edges: Vec<[P3; 2]> = standin.edged.iter().map(|(e, _)| *e).collect();
        assert_eq!(painted_edges, expected, "the held edges must follow the placed box");
    }

    /// A Move Mesh release holds the component's picture at the place the drag took it while the worker rebuilds the
    /// scene. The stand-in must paint the post-move world place, the same picture `StandIn::build` makes for the
    /// committed model.
    #[test]
    fn a_moved_mesh_stand_in_is_painted_where_the_release_put_it() {
        let mut s = Session::new();
        s.script(BOX).unwrap();
        let id = *s.model.elements.iter().find(|(_, e)| matches!(e, Element::Component(_))).unwrap().0;
        let before = Arc::clone(&s.model);
        let by = [1000.0, 0.0, 500.0];
        let guid = s.model.elements.guid(id).unwrap();
        s.execute("mesh_move", &json!({"component": guid, "by": by})).unwrap();
        let after = Arc::clone(&s.model);
        let cam =
            Camera { azimuth: -35.0, elevation: 25.0, scale: 0.5, target: [500.0, 500.0, 750.0], width: 640, height: 480, lens: None };
        let look = Look::of_3d(&after);
        let mesh = &after.families["Direct Model"].forms[0].mesh;
        let expected =
            StandIn::build(&after, Arc::clone(&after), &cam, &look, (id, [500.0, 500.0, 1000.0], [0.0, 0.0, 1.0]), mesh).unwrap();
        let moved = StandIn::build_moved(&before, after, &cam, &look, id, [0.0; 3], by).unwrap();
        assert!(!moved.painted.is_empty(), "the stand-in must have a picture to hold");
        assert_eq!(moved.painted, expected.painted, "the stand-in must paint the box where the release put it");
        assert_eq!(moved.edged, expected.edged, "its edges must follow the box to the new place");
    }

    #[test]
    fn only_a_pending_push_for_the_held_face_stays_active_after_release() {
        let (id, at, n) = (100002, [500.0, 500.0, 1000.0], [0.0, 0.0, 1.0]);
        let pushed = Some((id, at, n, 500.0));
        let held = Tool::PushPull(Some((id, at)));
        assert_eq!(active(&None, &pushed, &held), Some((id, at, n)), "the release frame keeps the gesture until the commit");
        assert_eq!(active(&None, &None, &held), None, "a quick press or Esc leaves no pending push");
        assert_eq!(active(&None, &pushed, &Tool::PushPull(None)), None, "no face held: nothing to push");
        assert_eq!(active(&None, &pushed, &Tool::Wall(Vec::new())), None, "a tool switch drops the old drag's push");
    }

    /// A perspective camera of the kind that makes the bottom face bleed through the front-right one: the sum of
    /// `project().2` over a triangle's corners is non-linear (`e²/ahead - e`) and `-inf` when a corner is behind
    /// the eye, so a far bottom triangle can sort *after* a near front one. The fix keys on the centroid's linear
    /// view-axis depth; with every corner in front, the key is always finite and monotone with distance.
    #[test]
    fn far_first_uses_linear_centroid_depth_even_in_close_perspective() {
        let cam =
            Camera { azimuth: -60.0, elevation: 25.0, scale: 0.5, target: [0.0, 1000.0, 217.5], width: 640, height: 480, lens: Some(50.0) };
        assert!(cam.eye().is_some(), "a perspective camera");
        let identity = |q: P3| q;
        let old_key = |t: &[P3; 3]| t.iter().map(|q| cam.project(*q).2).sum::<f64>();
        let pushed = box_tris_top(435.0); // a 565 mm push down, the geometry that inverts
        let mut sorted = pushed.clone();
        sorted.sort_by(|a, b| old_key(a).total_cmp(&old_key(b)));
        let bottom_max = sorted.iter().filter(|t| t.iter().all(|q| q[2] == 0.0)).map(&old_key).fold(f64::NEG_INFINITY, f64::max);
        let front_right_min =
            sorted.iter().filter(|t| t.iter().all(|q| q[0] == 1000.0 || q[1] == 0.0)).map(&old_key).fold(f64::INFINITY, f64::min);
        assert!(bottom_max > front_right_min, "the old key must invert here, or this proves nothing: {bottom_max} vs {front_right_min}");
        let (a, e) = (cam.azimuth.to_radians(), cam.elevation.to_radians());
        let back = [e.cos() * a.cos(), e.cos() * a.sin(), e.sin()];
        assert_eq!(back_axis(&cam), back, "the key's axis is the core camera's view axis");
        let eye = cam.eye().unwrap();
        let past = [eye[0] + back[0] * 1000.0, eye[1] + back[1] * 1000.0, eye[2] + back[2] * 1000.0];
        let mut behind = pushed.clone();
        behind[0] = [[past[0], past[1], past[2]], [past[0] + 10.0, past[1], past[2]], [past[0], past[1] + 10.0, past[2]]];
        let poisoned = behind.iter().map(&old_key).filter(|z| z.is_infinite()).count();
        assert!(poisoned > 0, "the old key must be -inf-poisoned behind the eye");
        // The new key: linear, finite, area-independent, and never poisoned by one far corner.
        let linear = |t: &[P3; 3]| -> f64 {
            let c: P3 = std::array::from_fn(|i| t.iter().map(|q| q[i]).sum::<f64>() / 3.0);
            c[0] * back[0] + c[1] * back[1] + c[2] * back[2]
        };
        let ordered = far_first(pushed.into_iter().map(|t| (t, ())).collect(), &cam, &identity);
        let keys: Vec<f64> = ordered.iter().map(|(t, _)| linear(t)).collect();
        assert!(keys.windows(2).all(|w| w[0] <= w[1]), "far_first must be monotone in linear depth: {keys:?}");
        assert!(ordered.iter().all(|(t, _)| linear(t).is_finite()), "no -inf poisoning");
        assert!(linear(&behind[0]).is_finite(), "one far corner must not poison the key");
        let kept = far_first(behind.into_iter().map(|t| (t, ())).collect(), &cam, &identity);
        assert_eq!(kept.len(), 11, "a triangle with a vertex behind the eye is dropped, not poisoned");

        // Both a pull up (+2318 mm) and a push down (-565 mm), in perspective and in a parallel view.
        for lens in [Some(50.0), None] {
            for top in [3318.0, 435.0] {
                let c = Camera { lens, ..cam };
                let want = box_tris_top(top);
                let (a, e) = (c.azimuth.to_radians(), c.elevation.to_radians());
                let back = [e.cos() * a.cos(), e.cos() * a.sin(), e.sin()];
                let depth = |t: &[P3; 3]| {
                    let c: P3 = std::array::from_fn(|i| t.iter().map(|q| q[i]).sum::<f64>() / 3.0);
                    c[0] * back[0] + c[1] * back[1] + c[2] * back[2]
                };
                let order = far_first(want.into_iter().map(|t| (t, ())).collect(), &c, &identity);
                let keys: Vec<f64> = order.iter().map(|(t, _)| depth(t)).collect();
                assert!(keys.windows(2).all(|w| w[0] <= w[1]), "lens {lens:?} top {top}: {keys:?}");
            }
        }
    }

    const BOX: &str = r#"mesh {"points": [[0,0,0],[1000,0,0],[1000,1000,0],[0,1000,0],[0,0,1000],[1000,0,1000],[1000,1000,1000],[0,1000,1000]], "triangles": [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]}"#;

    /// Both the drag overlay and the release stand-in must paint only the faces that face the eye. A sealed solid's
    /// bottom is back-facing from above; an open sheet has no outward normal and keeps both sides.
    #[test]
    fn sealed_solid_faces_along_the_camera_are_the_only_ones_painted() {
        let cam = Camera {
            azimuth: -60.0,
            elevation: 25.0,
            scale: 0.5,
            target: [1000.0, 1000.0, 500.0], // the placed box's middle: the eye sees its top, south and east faces
            width: 640,
            height: 480,
            lens: Some(50.0),
        };
        let held = (0, [500.0, 500.0, 1000.0], [0.0, 0.0, 1.0]);
        let mut s = Session::new();
        s.script(BOX).unwrap();
        let id = *s.model.elements.iter().find(|(_, e)| matches!(e, Element::Component(_))).unwrap().0;
        let out = box_tris();
        let model = Arc::clone(&s.model);
        let standin = StandIn::build(&model, Arc::clone(&model), &cam, &Look::of_3d(&model), (id, held.1, held.2), &out).unwrap();
        assert_eq!(standin.painted.len(), 6, "the top and two near sides show; the bottom and two far sides do not: {:?}", standin.painted);
        assert!(
            standin.painted.iter().all(|(t, _)| t.iter().any(|q| q[2] != 0.0)),
            "no whole bottom-face triangle survives: {:?}",
            standin.painted
        );

        // A mirror reverses every winding; the signed volume puts the normals back, so the same rule holds.
        s.execute("mirror", &json!({"ids": [id], "axis": [[0, 0], [0, 1000]]})).unwrap();
        let model = Arc::clone(&s.model);
        let mirrored = StandIn::build(&model, Arc::clone(&model), &cam, &Look::of_3d(&model), (id, held.1, held.2), &out).unwrap();
        assert_eq!(mirrored.painted.len(), 6, "mirrored back faces must be dropped too: {:?}", mirrored.painted);
        assert!(mirrored.painted.iter().all(|(t, _)| t.iter().any(|q| q[2] != 0.0)), "no whole mirrored bottom-face triangle survives");

        // An open sheet has no outward normal: keep every triangle so both sides still show.
        let sheet: Vec<[P3; 3]> = out[2..4].to_vec();
        let mut s = Session::new();
        s.script(BOX).unwrap();
        let id = *s.model.elements.iter().find(|(_, e)| matches!(e, Element::Component(_))).unwrap().0;
        let name = match s.model.elements.get(id) {
            Some(Element::Component(c)) => c.family.clone(),
            _ => panic!("a component"),
        };
        Arc::make_mut(&mut s.model).families.get_mut(&name).unwrap().forms[0].mesh = Arc::new(sheet.clone());
        let model = Arc::clone(&s.model);
        let open = StandIn::build(&model, Arc::clone(&model), &cam, &Look::of_3d(&model), (id, held.1, held.2), &sheet).unwrap();
        assert_eq!(open.painted.len(), sheet.len(), "an open sheet shows both sides");
    }

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

    fn box_tris() -> Vec<[P3; 3]> {
        box_tris_top(1000.0)
    }

    fn box_tris_top(top: f64) -> Vec<[P3; 3]> {
        let p: Vec<P3> = vec![
            [0.0, 0.0, 0.0],
            [1000.0, 0.0, 0.0],
            [1000.0, 1000.0, 0.0],
            [0.0, 1000.0, 0.0],
            [0.0, 0.0, top],
            [1000.0, 0.0, top],
            [1000.0, 1000.0, top],
            [0.0, 1000.0, top],
        ];
        [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]]
            .into_iter()
            .map(|[a, b, c]| [p[a], p[b], p[c]])
            .collect()
    }

    #[test]
    fn a_solid_s_back_edges_are_left_out_and_an_open_sheet_keeps_its_own() {
        let cam =
            Camera { azimuth: -45.0, elevation: 30.0, scale: 0.5, target: [500.0, 500.0, 500.0], width: 640, height: 480, lens: None };
        let cube = box_tris();
        let identity = |q: P3| q;
        let all = bimcraft_core::mesh::edges(&cube);
        assert_eq!(all.len(), 12, "a box has twelve edges to begin with");
        let drawn = front_edges(all.clone(), &cube, &identity, &cam);
        assert_eq!(drawn.len(), 9, "the three edges no front face owns are left out: {drawn:?}");
        let is_edge = |a: P3, b: P3| drawn.iter().any(|[p, q]| (*p == a && *q == b) || (*p == b && *q == a));
        for (a, b) in [
            ([0.0, 0.0, 0.0], [0.0, 1000.0, 0.0]),
            ([0.0, 1000.0, 0.0], [1000.0, 1000.0, 0.0]),
            ([0.0, 1000.0, 0.0], [0.0, 1000.0, 1000.0]),
        ] {
            assert!(!is_edge(a, b), "the back edge {a:?}..{b:?} must be left out");
        }
        // A mirror reverses every winding; the signed volume puts the normals back. The mirrored box now lies to
        // the west, so the back corner is the opposite one: those three edges, not the first three, must go.
        let mirror = |q: P3| [-q[0], q[1], q[2]];
        let mirrored = front_edges(all, &cube, &mirror, &cam);
        assert_eq!(mirrored.len(), 9);
        let mirrored_has = |a: P3, b: P3| mirrored.iter().any(|[p, q]| (*p == a && *q == b) || (*p == b && *q == a));
        for (a, b) in [
            ([1000.0, 0.0, 0.0], [1000.0, 1000.0, 0.0]),
            ([1000.0, 1000.0, 0.0], [1000.0, 1000.0, 1000.0]),
            ([0.0, 1000.0, 0.0], [1000.0, 1000.0, 0.0]),
        ] {
            assert!(!mirrored_has(a, b), "the mirrored back edge {a:?}..{b:?} must be left out");
        }
        let sheet: Vec<[P3; 3]> = cube[2..4].to_vec();
        let all = bimcraft_core::mesh::edges(&sheet);
        assert_eq!(front_edges(all.clone(), &sheet, &identity, &cam), all, "an open sheet shows both sides");
    }
}
