//! The 3D view as a working view: Shift and a middle drag orbit round the point under the cursor and the wheel zooms toward it, and the drawing
//! tools (walls, beams and braces, pipes, conduits and trays, columns, components) take their points on the face under the cursor,
//! snapped as a plan snaps; over empty space each says what it needs. In both profiles, on the processor's picture and, where the
//! machine has a graphics adapter, on the card's.

use super::*;
use crate::gpu::{Card, headless};
use crate::orbit::box_reading;
use crate::structure::{direct_box, direct_mesh};
use bimcraft_core::units::Units;
use bimcraft_core::view3d::P3;

/// The 3D view of `script` in each window it is driven in: BIMCraft's and Rev's, drawn on the processor and on the card (where
/// there is one: a runner without a graphics adapter says so and drives the processor's alone), each named for its failures.
fn views(script: &str) -> Vec<(String, Harness<'static, BimApp>)> {
    let mut out = Vec::new();
    for rev in [false, true] {
        for card in [false, true] {
            let mut h = if rev { app(script) } else { bimcraft(script) };
            if card {
                let Some(gpu) = headless() else {
                    eprintln!("no graphics adapter here: the card's 3D view is not driven");
                    continue;
                };
                h.state_mut().card = Card::with(gpu);
            }
            shortcut(&mut h, "3D");
            assert_eq!(h.state().sheet, Sheet::View3D);
            assert_eq!(h.state().orbit.on_card, card, "{:?}", h.state().card.note);
            let name = format!("{} on the {}", if rev { "Rev" } else { "BIMCraft" }, if card { "card" } else { "processor" });
            out.push((name, h));
        }
    }
    out
}

/// A point on the south wall's outer face: `BOX`'s walls are 200 mm thick about their lines.
const FACE: P3 = [5000.0, -100.0, 1500.0];

/// A cubic metre of mesh, as the `mesh` command takes it: what a Push/Pull pick has to find.
const MESH_BOX: &str = r#"mesh {"points": [[0,0,0],[1000,0,0],[1000,1000,0],[0,1000,0],[0,0,1000],[1000,0,1000],[1000,1000,1000],[0,1000,1000]], "triangles": [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]}"#;

#[test]
fn shift_and_a_middle_drag_orbit_round_the_point_under_the_cursor_which_stays_where_it_is_on_screen() {
    for (name, mut h) in views(BOX) {
        let before = h.state().orbit.camera.unwrap();
        let at = h.state().screen3(FACE);
        orbit_screen(&mut h, at, at + vec2(90.0, 40.0));
        let cam = h.state().orbit.camera.unwrap();
        assert!((cam.azimuth - before.azimuth).abs() > 10.0 && (cam.elevation - before.elevation).abs() > 5.0, "{name}: it turned");
        let now = h.state().screen3(FACE);
        assert!(now.distance(at) < 0.5, "{name}: the face under the cursor went from {at:?} to {now:?}");
        // From the background, round the target as ever.
        let corner = h.state().orbit.rect.min + vec2(6.0, 6.0);
        orbit_screen(&mut h, corner, corner + vec2(60.0, 0.0));
        let turned = h.state().orbit.camera.unwrap();
        assert!(turned.azimuth != cam.azimuth && turned.target == cam.target, "{name}");
    }
}

#[test]
fn a_primary_drag_never_moves_the_camera_in_any_tool() {
    for (name, mut h) in views(BOX) {
        let before = h.state().orbit.camera;
        let at = h.state().screen3(FACE);
        drag_screen(&mut h, at, at + vec2(90.0, 40.0), PointerButton::Primary);
        assert_eq!(h.state().orbit.camera, before, "{name}: Modify's drag is a box");
        h.state_mut().set_tool(Tool::Measure(None));
        h.run();
        drag_screen(&mut h, at, at + vec2(90.0, 40.0), PointerButton::Primary);
        assert_eq!(h.state().orbit.camera, before, "{name}: another tool's drag leaves the view be");
    }
}

#[test]
fn in_perspective_the_wheel_zooms_toward_the_point_under_the_cursor() {
    for (name, mut h) in views(BOX) {
        h.state_mut().run("perspective", json!({}));
        h.run();
        let before = h.state().orbit.camera.unwrap();
        assert!(before.lens.is_some(), "{name}: in perspective");
        let at = h.state().screen3(FACE);
        h.hover_at(at);
        h.event(Event::MouseWheel {
            unit: MouseWheelUnit::Point,
            delta: vec2(0.0, 120.0),
            phase: TouchPhase::Move,
            modifiers: Modifiers::NONE,
        });
        h.run();
        let cam = h.state().orbit.camera.unwrap();
        let k = cam.scale / before.scale;
        assert!(k > 1.2, "{name}: zoomed in");
        let now = h.state().screen3(FACE);
        assert!(now.distance(at) < 0.5, "{name}: the face under the cursor went from {at:?} to {now:?}");
        // The eye closed in on the face by as much as it zoomed: through it, it would never go.
        let far = |c: bimcraft_core::view3d::Camera| c.eye().map(|e| (0..3).map(|i| (e[i] - FACE[i]).powi(2)).sum::<f64>().sqrt());
        let (d0, d1) = (far(before).unwrap(), far(cam).unwrap());
        assert!((d0 / d1 - k).abs() < 1e-6 * k, "{name}: {d0} mm from the face, then {d1}, zoomed by {k}");
    }
}

#[test]
fn the_wall_tool_and_a_component_draw_on_a_floors_top_in_the_3d_view_and_stand_on_it() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    for (name, mut h) in views(&script) {
        let level = h.state().session.model.levels()[0].0;
        shortcut(&mut h, "WA");
        // A wall's face stands up: nothing to draw on, and the status line says so.
        let at = h.state().screen3(FACE);
        click_at(&mut h, at);
        assert!(h.state().status.error && matches!(&h.state().tool, Tool::Wall(c) if c.is_empty()), "{name}: {:?}", h.state().tool);
        for p in [[1500.0, 5000.0, 500.0], [4500.0, 5000.0, 500.0]] {
            let at = h.state().screen3(p);
            click_at(&mut h, at);
        }
        key(&mut h, Key::Escape);
        let m = &h.state().session.model;
        let walls = m.elements.values().filter_map(|e| if let Element::Wall(w) = e { Some(w) } else { None });
        // Not one of the box's, which all start on its corners.
        let made: Vec<_> = walls.filter(|w| w.start.0 > 0.0 && w.start.0 < 8000.0).collect();
        let w = made.first().unwrap_or_else(|| panic!("{name}: no wall: {}", h.state().status.text));
        assert_eq!((w.start, w.end, made.len()), (P2(1500.0, 5000.0), P2(4500.0, 5000.0), 1), "{name}");
        assert_eq!((m.elements.id(w.level), w.base_offset, m.own_base(w)), (Some(level), 500.0, 500.0), "{name}: on the floor's top");
        // A component on it too: on the floor's level, 500 up.
        key(&mut h, Key::Escape);
        shortcut(&mut h, "CM");
        let at = h.state().screen3([3000.0, 4000.0, 500.0]);
        click_at(&mut h, at);
        let m = &h.state().session.model;
        let placed: Vec<_> = m.elements.values().filter_map(|e| if let Element::Component(c) = e { Some(c) } else { None }).collect();
        let c = placed.first().unwrap_or_else(|| panic!("{name}: no component: {}", h.state().status.text));
        assert_eq!((c.at, m.elements.id(c.level), c.offset), (P2(3000.0, 4000.0), Some(level), 500.0), "{name}");
    }
}

#[test]
fn a_direct_model_box_is_drawn_from_two_corners_in_the_3d_view() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    for (name, mut h) in views(&script) {
        h.state_mut().set_tool(Tool::DirectModel(None));
        for p in [[1500.0, 5000.0, 500.0], [4500.0, 4000.0, 500.0]] {
            let at = h.state().screen3(p);
            click_at(&mut h, at);
        }
        assert!(!h.state().status.error, "{name}: {}", h.state().status.text);
        let m = &h.state().session.model;
        let made: Vec<_> = m
            .elements
            .values()
            .filter_map(|e| if let Element::Component(c) = e { Some(c) } else { None })
            .filter(|c| c.family == "Direct Model")
            .collect();
        let c = made.first().unwrap_or_else(|| panic!("{name}: no Direct Model component: {}", h.state().status.text));
        assert_eq!(c.at, P2(3000.0, 4500.0), "{name}: the box's plan middle");
        assert_eq!(c.offset, 500.0, "{name}: on the floor's top");
        let mesh = &m.families["Direct Model"].forms[0].mesh;
        assert_eq!(mesh.len(), 12, "{name}: a closed box, one `mesh` command");
        let span = |i: usize| {
            let all: Vec<f64> = mesh.iter().flatten().map(|p| p[i]).collect();
            (all.iter().fold(f64::MAX, |a, b| a.min(*b)), all.iter().fold(f64::MIN, |a, b| a.max(*b)))
        };
        assert_eq!(span(0), (-1500.0, 1500.0), "{name}: 3 m east-west");
        assert_eq!(span(1), (-500.0, 500.0), "{name}: 1 m north-south");
        assert_eq!(span(2), (0.0, 3000.0), "{name}: 3 m high, the Options Bar's height");
    }
}

#[test]
fn escape_drops_the_first_corner_of_a_direct_model_box() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    for (name, mut h) in views(&script) {
        h.state_mut().set_tool(Tool::DirectModel(None));
        let at = h.state().screen3([1500.0, 5000.0, 500.0]);
        click_at(&mut h, at);
        assert!(matches!(&h.state().tool, Tool::DirectModel(Some(_))), "{name}: the first corner is held");
        key(&mut h, Key::Escape);
        assert!(matches!(&h.state().tool, Tool::DirectModel(None)), "{name}: Esc drops it");
        // A lone corner after that makes no box: the tool is waiting for its first pick again.
        let at = h.state().screen3([4500.0, 4000.0, 500.0]);
        click_at(&mut h, at);
        assert!(h.state().session.model.families.get("Direct Model").is_none(), "{name}: no box made");
    }
}

#[test]
fn a_direct_models_preview_and_its_commit_are_the_same_box() {
    // One definition, reached from the hover the preview draws and from the second click the command commits.
    let (lo, hi) = direct_box(P2(1500.0, 5000.0), P2(4500.0, 4000.0), 500.0, 3000.0);
    assert_eq!(lo, [1500.0, 4000.0, 500.0]);
    assert_eq!(hi, [4500.0, 5000.0, 3500.0]);
    let (points, triangles) = direct_mesh((lo, hi));
    assert_eq!((points.len(), triangles.len()), (8, 12));
    let span = |i: usize| {
        let all: Vec<f64> = points.iter().map(|p| p[i]).collect();
        (all.iter().copied().fold(f64::MAX, f64::min), all.iter().copied().fold(f64::MIN, f64::max))
    };
    assert_eq!(span(0), (1500.0, 4500.0));
    assert_eq!(span(1), (4000.0, 5000.0));
    assert_eq!(span(2), (500.0, 3500.0));
}

#[test]
fn a_direct_models_reading_shows_area_and_volume_in_the_projects_units() {
    assert_eq!(box_reading(Units::Millimetres, (3000.0, 4000.0, 2000.0)), "12.00 m² · 24.00 m³");
    assert_eq!(box_reading(Units::Feet, (3048.0, 3048.0, 3048.0)), "100.00 ft² · 1000.00 ft³");
}

#[test]
fn a_direct_model_box_is_previewed_between_its_two_clicks() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    for (name, mut h) in views(&script) {
        h.state_mut().set_tool(Tool::DirectModel(None));
        let second = h.state().screen3([4500.0, 4000.0, 500.0]);
        // The same hover with no first corner draws only the snap ink.
        h.hover_at(second);
        h.run();
        let bare = h.render().unwrap();
        // With the first corner held it also draws the box's wireframe.
        let first = h.state().screen3([1500.0, 5000.0, 500.0]);
        click_at(&mut h, first);
        h.hover_at(second);
        h.run();
        let preview = h.render().unwrap();
        let changed = bare.pixels().zip(preview.pixels()).filter(|(a, b)| a != b).count();
        assert!(changed > 30, "{name}: the preview drew only {changed} pixels");
        // Halfway up the edge under the cursor: the from-line is down on the face, so only the box's wireframe draws here.
        let edge = h.state().screen3([4500.0, 4000.0, 2000.0]);
        let (x, y) = (edge.x.round() as u32, edge.y.round() as u32);
        let ink =
            |img: &image::RgbaImage| (x - 1..=x + 1).flat_map(|x| (y - 1..=y + 1).map(move |y| *img.get_pixel(x, y))).collect::<Vec<_>>();
        assert!(ink(&bare) != ink(&preview), "{name}: no wireframe on the box's edge");
        // The reading sits above the cursor. The wireframe is in the same area and the card's picture makes its tag
        // background blend in, so this is a coarse count; the pure test above checks the exact wording and units.
        let hud = second + vec2(12.0, -36.0);
        let (x0, y0) = ((hud.x - 6.0) as u32, (hud.y - 24.0) as u32);
        let (x1, y1) = ((hud.x + 160.0) as u32, (hud.y + 4.0) as u32);
        let changed =
            (x0..x1).flat_map(|x| (y0..y1).map(move |y| (x, y))).filter(|&(x, y)| bare.get_pixel(x, y) != preview.get_pixel(x, y)).count();
        assert!(changed > 150, "{name}: no area/volume reading: {changed} pixels");
        assert!(h.state().session.model.families.get("Direct Model").is_none(), "{name}: hovering made no box");
        // Esc drops the corner and the wireframe with it.
        key(&mut h, Key::Escape);
        h.hover_at(second);
        h.run();
        let dropped = h.render().unwrap();
        let changed = bare.pixels().zip(dropped.pixels()).filter(|(a, b)| a != b).count();
        assert!(changed < 10, "{name}: Esc left {changed} preview pixels");
    }
}

#[test]
fn a_direct_model_boxs_height_can_be_dragged_in_the_view() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    for (name, mut h) in views(&script) {
        h.state_mut().set_tool(Tool::DirectModel(None));
        let first = h.state().screen3([1500.0, 5000.0, 500.0]);
        click_at(&mut h, first);
        assert!(matches!(&h.state().tool, Tool::DirectModel(Some(_))), "{name}: the first corner is held");
        // A primary drag upward in the view pulls the box taller than the Options Bar's 3 m; a plain click still places it.
        let from = h.state().orbit.rect.center();
        drag_screen(&mut h, from, from - vec2(0.0, 160.0), PointerButton::Primary);
        assert!(matches!(&h.state().tool, Tool::DirectModel(Some(_))), "{name}: the drag did not place the box");
        let second = h.state().screen3([4500.0, 4000.0, 500.0]);
        click_at(&mut h, second);
        assert!(matches!(&h.state().tool, Tool::DirectModel(None)), "{name}: the box was placed");
        let m = &h.state().session.model;
        let made = m.elements.values().any(|e| matches!(e, Element::Component(c) if c.family == "Direct Model"));
        assert!(made, "{name}: no box made: {}", h.state().status.text);
        let mesh = &m.families["Direct Model"].forms[0].mesh;
        let top = mesh.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max);
        assert!(top > 3000.0, "{name}: the drag left the height at {top} mm");
    }
}

#[test]
fn the_massing_tools_say_what_they_want_in_the_3d_view() {
    // Direct Model and Push/Pull are the 3D view's own tools; the status bar's prompt is Push/Pull's only
    // feedback once a face is picked, so it must not claim they work in plan views alone.
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::DirectModel(None));
        assert!(h.state().prompt().contains("first corner"), "{name}: {}", h.state().prompt());
        h.state_mut().set_tool(Tool::PushPull(None));
        assert!(h.state().prompt().contains("Click a face of a mesh component"), "{name}: {}", h.state().prompt());
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        click_at(&mut h, at);
        assert!(matches!(&h.state().tool, Tool::PushPull(Some(_))), "{name}: a face was picked");
        assert!(h.state().prompt().contains("Type a distance"), "{name}: {}", h.state().prompt());
    }
}

/// How many pixels differ between two renders round `at`: the status bar's prompt also changes when a tool changes
/// state, so a test of what a highlight drew looks only near the point it lit.
fn changed_near(at: Pos2, a: &image::RgbaImage, b: &image::RgbaImage) -> usize {
    let (x0, y0) = ((f64::from(at.x) - 90.0).max(0.0) as u32, (f64::from(at.y) - 70.0).max(0.0) as u32);
    let (x1, y1) =
        ((f64::from(at.x) + 90.0).min(f64::from(a.width()) - 1.0) as u32, (f64::from(at.y) + 70.0).min(f64::from(a.height()) - 1.0) as u32);
    (x0..x1).flat_map(|x| (y0..y1).map(move |y| (x, y))).filter(|&(x, y)| a.get_pixel(x, y) != b.get_pixel(x, y)).count()
}

#[test]
fn the_face_under_the_cursor_is_lit_before_it_is_picked() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::PushPull(None));
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        // Over the view's top corner, off the box: nothing is lit where nothing can be picked.
        let off = h.state().orbit.rect.min + vec2(30.0, 110.0);
        h.hover_at(off);
        h.run();
        let bare = h.render().unwrap();
        // The face under the cursor lights before the click, so the pick looks like it will land.
        h.hover_at(at);
        h.run();
        let lit = h.render().unwrap();
        let changed = changed_near(at, &bare, &lit);
        assert!(changed > 100, "{name}: the hovered face drew only {changed} pixels near the cursor");
        // Moving off the box drops the light.
        h.hover_at(off);
        h.run();
        let gone = h.render().unwrap();
        let changed = changed_near(at, &bare, &gone);
        assert!(changed < 10, "{name}: {changed} pixels stayed lit off the box");
        // What the click takes is the face that was lit: the light holds as the pick.
        h.hover_at(at);
        h.run();
        click_at(&mut h, at);
        assert!(matches!(&h.state().tool, Tool::PushPull(Some(_))), "{name}: a face was picked: {}", h.state().status.text);
        h.hover_at(at);
        h.run();
        let held = h.render().unwrap();
        // The held tag reads above the pick now, so compare the strip of face below it: the tint itself holds.
        let below = |a: &image::RgbaImage, b: &image::RgbaImage| {
            let (x0, y0) = ((at.x - 60.0).max(0.0) as u32, (at.y + 20.0).max(0.0) as u32);
            let (x1, y1) = ((at.x + 60.0).min(a.width() as f32 - 1.0) as u32, (at.y + 70.0).min(a.height() as f32 - 1.0) as u32);
            (x0..x1).flat_map(|x| (y0..y1).map(move |y| (x, y))).filter(|&(x, y)| a.get_pixel(x, y) != b.get_pixel(x, y)).count()
        };
        let moved = below(&lit, &held);
        assert_eq!(moved, 0, "{name}: the held light moved {moved} pixels from the hovered one");
        // Esc hands the light straight back to the hover: the same face stays lit, with the pointer still on it.
        key(&mut h, Key::Escape);
        h.hover_at(at);
        h.run();
        let back = h.render().unwrap();
        let changed = changed_near(at, &lit, &back);
        assert!(changed < 10, "{name}: the handback moved {changed} pixels");
    }
}

#[test]
fn the_face_push_pull_holds_is_lit_while_it_waits_for_its_distance() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::PushPull(None));
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        // Off the box, over the view's corner: nothing is lit before a face is picked.
        let off = h.state().orbit.rect.min + vec2(30.0, 110.0);
        h.hover_at(off);
        h.run();
        let bare = h.render().unwrap();
        click_at(&mut h, at);
        h.hover_at(at);
        h.run();
        let lit = h.render().unwrap();
        let changed = changed_near(at, &bare, &lit);
        assert!(changed > 100, "{name}: the held face drew only {changed} pixels near the pick");
        // What's typed reads beside the picked face, as a plan's reading does.
        h.event(Event::Text("500".into()));
        h.run();
        let typed = h.render().unwrap();
        let changed = lit.pixels().zip(typed.pixels()).filter(|(a, b)| a != b).count();
        assert!(changed > 10, "{name}: the typed distance drew {changed} pixels");
        // Esc drops the pick, the reading and the highlight with it; the pointer is off the box, so nothing is lit.
        key(&mut h, Key::Escape);
        h.event(Event::Text("".into()));
        h.run();
        h.hover_at(off);
        h.run();
        let dropped = h.render().unwrap();
        let changed = bare.pixels().zip(dropped.pixels()).filter(|(a, b)| a != b).count();
        assert!(changed < 10, "{name}: Esc left {changed} highlight pixels");
    }
}

#[test]
fn a_mesh_face_is_pushed_by_dragging_it_in_the_3d_view() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::PushPull(None));
        // The top face, dragged up the screen: the box grows by what the drag shows, and the release commits it.
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        drag_screen(&mut h, at, at + vec2(0.0, -120.0), PointerButton::Primary);
        assert!(!h.state().status.error, "{name}: {}", h.state().status.text);
        let mesh = &h.state().session.model.families["Direct Model"].forms[0].mesh;
        let top = mesh.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max);
        assert!(top > 1000.0, "{name}: the drag left the top at {top}");
        assert!(matches!(&h.state().tool, Tool::PushPull(None)), "{name}: the release committed and let go");
        // Dragging the face back down the screen shrinks it again.
        let at = h.state().screen3([500.0, 500.0, top]);
        drag_screen(&mut h, at, at + vec2(0.0, 120.0), PointerButton::Primary);
        assert!(!h.state().status.error, "{name}: {}", h.state().status.text);
        let mesh = &h.state().session.model.families["Direct Model"].forms[0].mesh;
        let shrunk = mesh.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max);
        assert!(shrunk < top && shrunk >= 1000.0, "{name}: the drag back left the top at {shrunk} from {top}");
        // A quick press still picks the face for a typed distance.
        let on = h.state().screen3([500.0, 500.0, shrunk]);
        click_at(&mut h, on);
        assert!(matches!(&h.state().tool, Tool::PushPull(Some(_))), "{name}: a quick press picks: {}", h.state().status.text);
        // Esc during a drag drops it: the face does not move, and the pick lets go.
        h.event(Event::PointerButton { pos: on, button: PointerButton::Primary, pressed: true, modifiers: Modifiers::NONE });
        h.run();
        h.hover_at(on + vec2(0.0, -60.0));
        h.run();
        key(&mut h, Key::Escape);
        h.event(Event::PointerButton {
            pos: on + vec2(0.0, -60.0),
            button: PointerButton::Primary,
            pressed: false,
            modifiers: Modifiers::NONE,
        });
        h.run();
        let mesh = &h.state().session.model.families["Direct Model"].forms[0].mesh;
        let still = mesh.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max);
        assert_eq!(still, shrunk, "{name}: Esc mid-drag moved the face");
        assert!(matches!(&h.state().tool, Tool::PushPull(None)), "{name}: Esc mid-drag let go of the pick");
        // A tool switched mid-drag survives the release: the drag goes with the tool, and nothing is pushed.
        h.event(Event::PointerButton { pos: on, button: PointerButton::Primary, pressed: true, modifiers: Modifiers::NONE });
        h.run();
        h.hover_at(on + vec2(0.0, -60.0));
        h.run();
        h.state_mut().set_tool(Tool::Wall(Vec::new()));
        h.event(Event::PointerButton {
            pos: on + vec2(0.0, -60.0),
            button: PointerButton::Primary,
            pressed: false,
            modifiers: Modifiers::NONE,
        });
        h.run();
        assert!(matches!(&h.state().tool, Tool::Wall(_)), "{name}: the release kept the chosen tool");
        let mesh = &h.state().session.model.families["Direct Model"].forms[0].mesh;
        let still = mesh.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max);
        assert_eq!(still, shrunk, "{name}: the release after a tool switch moved the face");
    }
}

/// A picked face reads zero straight away: the distance tag is the tool's feedback while it waits for digits, not
/// something that appears only after a digit is typed.
#[test]
fn a_held_face_reads_zero_before_anything_is_typed() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::PushPull(None));
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        click_at(&mut h, at);
        assert!(matches!(&h.state().tool, Tool::PushPull(Some(_))), "{name}: a face was picked");
        h.hover_at(at);
        h.run();
        let held = h.render().unwrap();
        h.event(Event::Text("0".into()));
        h.run();
        let typed = h.render().unwrap();
        let changed = changed_near(at, &held, &typed);
        assert!(changed < 30, "{name}: typing zero drew the tag that was missing at held: {changed} pixels");
    }
}

/// A drag hides the whole component, so the overlay must stand in for every mesh form of the family, not only the one
/// the push reaches: a two-material import is two forms, and while one is pushed the other must stay in the picture.
#[test]
fn a_push_pull_drag_keeps_the_family_s_other_parts_in_the_picture() {
    // A directory of our own: CI's temp dir has no `opencode` folder, and a fixed file name would collide between runs.
    let dir = std::env::temp_dir().join(format!("bimcraft-ui-two-parts-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let obj = dir.join("two-parts.obj");
    let mtl = dir.join("two-parts.mtl");
    let mut text = String::from("mtllib two-parts.mtl\no Two\n");
    // A cube a metre each way, its points and faces as `MESH_BOX`'s are: the proven order, so the mesh closes a solid
    // and a push of its top goes.
    const PTS: [[f64; 3]; 8] = [
        [0.0, 0.0, 0.0],
        [1000.0, 0.0, 0.0],
        [1000.0, 1000.0, 0.0],
        [0.0, 1000.0, 0.0],
        [0.0, 0.0, 1000.0],
        [1000.0, 0.0, 1000.0],
        [1000.0, 1000.0, 1000.0],
        [0.0, 1000.0, 1000.0],
    ];
    const FACES: [[usize; 3]; 12] = [
        [0, 2, 1],
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
    ];
    for (half, name, base) in [(0.0, "A", 0usize), (2000.0, "B", 8usize)] {
        text.push_str(&format!("usemtl {name}\n"));
        for p in PTS {
            text.push_str(&format!("v {} {} {}\n", half + p[0], p[1], p[2]));
        }
        for f in FACES {
            text.push_str(&format!("f {} {} {}\n", f[0] + base + 1, f[1] + base + 1, f[2] + base + 1));
        }
    }
    std::fs::write(&obj, &text).unwrap();
    std::fs::write(&mtl, "newmtl A\nKd 0.8 0.2 0.2\nnewmtl B\nKd 0.2 0.2 0.8\n").unwrap();
    let script = format!("import_mesh {}", json!({"path": obj.to_string_lossy(), "at": [0, 0], "units": "mm", "up": "z"}));
    for (name, mut h) in views(&script) {
        assert_eq!(h.state().session.model.families["Two"].forms.len(), 2, "{name}: two materials, two mesh forms");
        h.state_mut().set_tool(Tool::PushPull(None));
        // Part A's top face, and part B's middle, a metre of air apart: their pixels are apart too.
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        let other = h.state().screen3([2500.0, 500.0, 500.0]);
        h.hover_at(at);
        h.run();
        let bare = h.render().unwrap();
        // Press part A's top and drag it upward, the way drag_screen does but with the drag held mid-way: while it
        // runs, part B stays in the picture. A card shades a pixel its own rounding's way, so a small drift is not a
        // vanishing part; the box stays inside the face, off its edges, where the ink is the same either way.
        h.event(Event::PointerButton { pos: at, button: PointerButton::Primary, pressed: true, modifiers: Modifiers::NONE });
        h.run();
        h.hover_at(at + vec2(0.0, -30.0));
        h.run();
        assert!(
            matches!(&h.state().tool, Tool::PushPull(Some(_))),
            "{name}: the press at {at:?} picked nothing: {}",
            h.state().status.text
        );
        h.hover_at(at + vec2(0.0, -90.0));
        h.run();
        let during = h.render().unwrap();
        let (x0, y0) = ((other.x - 25.0).max(0.0) as u32, (other.y - 18.0).max(0.0) as u32);
        let (x1, y1) = ((other.x + 25.0).min(bare.width() as f32 - 1.0) as u32, (other.y + 18.0).min(bare.height() as f32 - 1.0) as u32);
        // What the test means: the part keeps its ink. The overlay paints a triangle at a time, so the seams along a
        // face's inner diagonal blend a shade off the scene's own fill — ink either way, not a vanishing part. A pixel
        // counts only where the scene had ink and the drag left the canvas's ground: a part gone is a thousand, a kept
        // one none.
        let bg = bare
            .get_pixel(
                ((h.state().orbit.rect.min.x + 8.0) as u32).min(bare.width() - 1),
                ((h.state().orbit.rect.min.y + 8.0) as u32).min(bare.height() - 1),
            )
            .0;
        let far = |p: &[u8; 4]| p.iter().zip(bg.iter()).map(|(u, v)| u.abs_diff(*v)).max().unwrap_or(0);
        let gone = (x0..x1)
            .flat_map(|x| (y0..y1).map(move |y| (x, y)))
            .filter(|&(x, y)| far(&bare.get_pixel(x, y).0) > 30 && far(&during.get_pixel(x, y).0) < 30)
            .count();
        assert!(gone < 40, "{name}: the drag blanked {gone} pixels of the part it is not pushing");
        // The release commits the push it showed.
        h.event(Event::PointerButton {
            pos: at + vec2(0.0, -90.0),
            button: PointerButton::Primary,
            pressed: false,
            modifiers: Modifiers::NONE,
        });
        h.run();
        assert!(!h.state().status.error, "{name}: {}", h.state().status.text);
        let mesh = &h.state().session.model.families["Two"].forms[0].mesh;
        let top = mesh.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max);
        assert!(top > 1000.0, "{name}: the drag left the top at {top}");
    }
    let _ = std::fs::remove_file(obj);
    let _ = std::fs::remove_file(mtl);
    let _ = std::fs::remove_dir(dir);
}

/// The Direct Model box stands on the plane its first corner was clicked on: the opposite corner aimed over empty
/// space lands where that plane puts the view's ray, not where the view's default level would — a metre and a half
/// away here. The tool carries its plane, so the preview and the commit agree wherever the pointer roams.
#[test]
fn a_direct_model_box_stands_on_its_first_click_s_plane() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::DirectModel(None));
        // The first corner on the box's top face, a plane a metre over its level's.
        let first = h.state().screen3([400.0, 400.0, 1000.0]);
        click_at(&mut h, first);
        // The plane is a fact of the tool now, not something recovered by matching a corner against the view's slot.
        let Tool::DirectModel(Some((corner, plane))) = &h.state().tool else { panic!("{name}: the first corner was not taken") };
        assert!((corner.0 - 400.0).abs() < 1.0 && (corner.1 - 400.0).abs() < 1.0, "{name}: the tool kept corner {corner:?}");
        assert!((plane.z - 1000.0).abs() < 1.0, "{name}: the tool's plane is at z {}, not the face's 1000", plane.z);
        // The opposite corner over empty space beyond the box, on the first plane's z: the ray through this pixel
        // meets the level's plane (z = 0) far to the south-west of where it meets z = 1000.
        let far = h.state().screen3([2500.0, 700.0, 1000.0]);
        click_at(&mut h, far);
        let m = &h.state().session.model;
        let Element::Component(c) = m.elements.values().next_back().unwrap() else { panic!("{name}: no component") };
        // The family's mesh is centred on the box's middle and the component carries it back: the plan extents in the model.
        let mesh = &m.families[&c.family].forms[0].mesh;
        let (lo, hi) = mesh
            .iter()
            .flatten()
            .fold(([f64::MAX; 2], [f64::MIN; 2]), |(lo, hi), p| ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])]));
        let (west, east, south, north) = (c.at.0 + lo[0], c.at.0 + hi[0], c.at.1 + lo[1], c.at.1 + hi[1]);
        assert!((east - 2500.0).abs() < 60.0, "{name}: the box reaches x {east}, not the first plane's 2500");
        assert!((north - 700.0).abs() < 60.0, "{name}: the box reaches y {north}, not the first plane's 700");
        assert!(west > 300.0 && south > 300.0, "{name}: the box stands at ({west}, {south})");
    }
}

/// The faint held tint still reads against both grounds, the light canvas's and the dark one's: the tool once looked
/// dead for having no feedback, so the alpha-0 baseline and the tint must differ on either.
#[test]
fn the_held_tint_reads_on_a_light_and_a_dark_canvas() {
    for dark in [false, true] {
        for (name, mut h) in views(MESH_BOX) {
            if dark {
                h.ctx.set_theme(egui::ThemePreference::Dark);
                h.run();
                h.run();
            }
            h.state_mut().set_tool(Tool::PushPull(None));
            let at = h.state().screen3([500.0, 500.0, 1000.0]);
            let off = h.state().orbit.rect.min + vec2(30.0, 110.0);
            h.hover_at(off);
            h.run();
            let bare = h.render().unwrap();
            click_at(&mut h, at);
            h.hover_at(at);
            h.run();
            let held = h.render().unwrap();
            let changed = changed_near(at, &bare, &held);
            assert!(changed > 200, "{name}, dark={dark}: the held tint drew only {changed} pixels near the pick");
            // Not just a different pixel — one that moves enough on the face itself to read at arm's length.
            let q = h.state().screen3([500.0, 500.0, 1000.0]);
            let before = bare.get_pixel(q.x as u32, q.y as u32).0;
            let after = held.get_pixel(q.x as u32, q.y as u32).0;
            let step = before.iter().zip(after.iter()).map(|(a, b)| a.abs_diff(*b)).max().unwrap_or(0);
            assert!(step >= 6, "{name}, dark={dark}: the faint tint moved a channel by only {step}");
        }
    }
}

/// While a Push/Pull drag runs, the pulled face and the walls it stretches are shaded like the model — the
/// material's colour turned to the view's light — not a flat accent painted over the canvas.
#[test]
fn a_push_pull_drag_preview_is_shaded_like_the_model() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::PushPull(None));
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        let to = at + vec2(0.0, -120.0);
        h.hover_at(at);
        h.run();
        h.event(Event::PointerButton { pos: at, button: PointerButton::Primary, pressed: true, modifiers: Modifiers::NONE });
        h.run();
        for t in [0.25, 0.5, 0.75, 1.0] {
            h.hover_at(at + (to - at) * t);
            h.run();
        }
        let cam = h.state().orbit.camera.unwrap();
        let o = &h.state().orbit;
        let px = |p: Pos2| (f64::from((p.x - o.rect.min.x) * o.ppp), f64::from((p.y - o.rect.min.y) * o.ppp));
        let by = cam.along_dir([0.0, 0.0, 1.0], [500.0, 500.0, 1000.0], px(at), px(to)).unwrap().round();
        assert!(by > 100.0, "{name}: the drag pulled only {by} mm");
        // Half way up a stretched wall: over the original box this is empty canvas, so only the preview draws here.
        let wall = h.state().screen3([1000.0, 500.0, 1000.0 + by / 2.0]);
        let [r, g, b, _] = h.render().unwrap().get_pixel(wall.x as u32, wall.y as u32).0;
        assert!(r > b && g > b, "{name}: the stretched wall was [{r},{g},{b}], an accent fill not the material's shade");
    }
}

/// A drag past where the solid turns inside out does not vanish: the release commits the last push that was legal,
/// so the face stops at the limit instead of the command refusing and leaving the model untouched.
#[test]
fn an_overshooting_push_pull_drag_commits_the_last_valid_push() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::PushPull(None));
        let before = h.state().session.model.families["Direct Model"].forms[0].mesh.clone();
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        let probe = at + vec2(0.0, -120.0);
        h.hover_at(at);
        h.run();
        h.event(Event::PointerButton { pos: at, button: PointerButton::Primary, pressed: true, modifiers: Modifiers::NONE });
        h.run();
        for t in [0.25, 0.5, 0.75, 1.0] {
            h.hover_at(at + (probe - at) * t);
            h.run();
        }
        // Scale the probe to a push that would turn the solid inside out, along whichever face the pick found.
        let cam = h.state().orbit.camera.unwrap();
        let held = *h.state().orbit.push_drag.as_ref().expect("{name}: a face was picked").target();
        let o = &h.state().orbit;
        let px = |p: Pos2| (f64::from((p.x - o.rect.min.x) * o.ppp), f64::from((p.y - o.rect.min.y) * o.ppp));
        let per = cam.along_dir(held.2, held.1, px(at), px(probe)).unwrap().abs();
        let to = at + (probe - at) * (-1200.0 / per) as f32;
        for t in [0.5, 1.0] {
            h.hover_at(at + (to - at) * t);
            h.run();
        }
        h.event(Event::PointerButton { pos: to, button: PointerButton::Primary, pressed: false, modifiers: Modifiers::NONE });
        h.run();
        assert!(!h.state().status.error, "{name}: {}", h.state().status.text);
        let after = &h.state().session.model.families["Direct Model"].forms[0].mesh;
        assert!(before.as_ref() != after.as_ref(), "{name}: the overshoot was refused instead of clamped");
    }
}

/// While a face is dragged the component itself is out of the picture: the overlay draws the whole pushed mesh, so a
/// shrink pulls the face in rather than leaving the old one standing behind it.
#[test]
fn a_shrink_drag_leaves_the_old_face_out_of_the_picture() {
    for (name, mut h) in views(MESH_BOX) {
        h.run_steps(4);
        h.state_mut().set_tool(Tool::PushPull(None));
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        let probe = at + vec2(0.0, -120.0);
        h.hover_at(at);
        h.run();
        h.event(Event::PointerButton { pos: at, button: PointerButton::Primary, pressed: true, modifiers: Modifiers::NONE });
        h.run();
        for t in [0.25, 0.5, 0.75, 1.0] {
            h.hover_at(at + (probe - at) * t);
            h.run();
        }
        let cam = h.state().orbit.camera.unwrap();
        let held = *h.state().orbit.push_drag.as_ref().expect("{name}: a face was picked").target();
        assert!(held.2[2] > 0.9, "{name}: the top face was not picked: {held:?}");
        let o = &h.state().orbit;
        let px = |p: Pos2| (f64::from((p.x - o.rect.min.x) * o.ppp), f64::from((p.y - o.rect.min.y) * o.ppp));
        let per = cam.along_dir(held.2, held.1, px(at), px(probe)).unwrap().abs();
        let to = at + (probe - at) * (-400.0 / per) as f32;
        for t in [0.5, 1.0] {
            h.hover_at(at + (to - at) * t);
            h.run();
        }
        let shot = h.render().unwrap();
        // The highest corner of the old top in the picture: a point the shorter box cannot cover at any x, y.
        let old = h.state().screen3([100.0, 900.0, 1000.0]);
        let empty = h.state().orbit.rect.min + vec2(30.0, 110.0);
        let wall = shot.get_pixel(old.x as u32, old.y as u32).0;
        let paper = shot.get_pixel(empty.x as u32, empty.y as u32).0;
        assert_eq!(wall, paper, "{name}: the old face still shows at {old:?}: {wall:?}");
    }
}

/// In a wireframe view the preview keeps to edges: no fills, so the view never promises a shaded face it would not draw.
#[test]
fn a_wireframe_view_shows_only_edges_while_a_face_is_dragged() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().run("visual_style", json!({"style": "wireframe"}));
        h.run();
        h.state_mut().set_tool(Tool::PushPull(None));
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        let to = at + vec2(0.0, -120.0);
        let off = h.state().orbit.rect.min + vec2(30.0, 110.0);
        h.run();
        let shot = h.render().unwrap();
        let paper = shot.get_pixel(off.x as u32, off.y as u32).0;
        let rect = h.state().orbit.rect;
        let filled = |shot: &image::RgbaImage| {
            (rect.min.x as u32..rect.max.x as u32)
                .flat_map(|x| (rect.min.y as u32..rect.max.y as u32).map(move |y| (x, y)))
                .filter(|&(x, y)| shot.get_pixel(x, y).0 != paper)
                .count()
        };
        let bare = filled(&shot);
        h.hover_at(at);
        h.run();
        h.event(Event::PointerButton { pos: at, button: PointerButton::Primary, pressed: true, modifiers: Modifiers::NONE });
        h.run();
        for t in [0.25, 0.5, 0.75, 1.0] {
            h.hover_at(at + (to - at) * t);
            h.run();
        }
        let dragging = filled(&h.render().unwrap());
        // Edges and the distance tag add a little ink; a shaded or accent fill would colour tens of thousands of pixels.
        assert!(dragging.saturating_sub(bare) < 12_000, "{name}: a wireframe drag filled {} pixels", dragging.saturating_sub(bare));
    }
}

#[test]
fn a_mesh_face_is_pushed_by_a_typed_distance_in_the_3d_view() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::PushPull(None));
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        click_at(&mut h, at);
        assert!(matches!(&h.state().tool, Tool::PushPull(Some(_))), "{name}: a face was picked: {}", h.state().status.text);
        h.event(Event::Text("500".into()));
        h.run();
        key(&mut h, Key::Enter);
        assert!(!h.state().status.error, "{name}: {}", h.state().status.text);
        let mesh = &h.state().session.model.families["Direct Model"].forms[0].mesh;
        let top = mesh.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max);
        assert_eq!(top, 1500.0, "{name}: the top face moved up 500 mm");
        assert!(matches!(&h.state().tool, Tool::PushPull(None)), "{name}: ready for the next face");
        // A negative distance shrinks it back; the leading sign is captured for this tool.
        let at = h.state().screen3([500.0, 500.0, 1500.0]);
        click_at(&mut h, at);
        h.event(Event::Text("-500".into()));
        h.run();
        key(&mut h, Key::Enter);
        let mesh = &h.state().session.model.families["Direct Model"].forms[0].mesh;
        let top = mesh.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max);
        assert_eq!(top, 1000.0, "{name}: the top face moved back down 500 mm");
    }
}

#[test]
fn escape_drops_the_picked_mesh_face() {
    for (name, mut h) in views(MESH_BOX) {
        h.state_mut().set_tool(Tool::PushPull(None));
        let at = h.state().screen3([500.0, 500.0, 1000.0]);
        click_at(&mut h, at);
        assert!(matches!(&h.state().tool, Tool::PushPull(Some(_))), "{name}: a face was picked");
        key(&mut h, Key::Escape);
        assert!(matches!(&h.state().tool, Tool::PushPull(None)), "{name}: Esc drops it");
        h.event(Event::Text("500".into()));
        h.run();
        key(&mut h, Key::Enter);
        let mesh = &h.state().session.model.families["Direct Model"].forms[0].mesh;
        let top = mesh.iter().flatten().map(|p| p[2]).fold(f64::MIN, f64::max);
        assert_eq!(top, 1000.0, "{name}: no face picked, nothing pushed");
    }
}

/// The runs of `category` in the model, as they are.
fn runs(h: &Harness<BimApp>, category: &str) -> Vec<bimcraft_core::model::Run> {
    let m = &h.state().session.model;
    m.elements.values().filter(|e| e.category() == category).filter_map(|e| e.run().cloned()).collect()
}

#[test]
fn a_beam_runs_between_two_points_on_a_floors_top_in_the_3d_view_and_the_next_snaps_to_its_end() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    for (name, mut h) in views(&script) {
        let level = h.state().session.model.levels()[0].0;
        shortcut(&mut h, "BM");
        assert!(matches!(h.state().tool, Tool::Run("beam", None)), "{name}: {:?}", h.state().tool);
        // Over the south wall, where the view sees the floor.
        for p in [[1500.0, 5000.0, 500.0], [4500.0, 5000.0, 500.0]] {
            let at = h.state().screen3(p);
            click_at(&mut h, at);
        }
        key(&mut h, Key::Escape);
        let made = runs(&h, "beam");
        let b = made.first().unwrap_or_else(|| panic!("{name}: no beam: {}", h.state().status.text));
        assert_eq!(made.len(), 1, "{name}");
        assert_eq!((b.start.0, b.end.0), (1500.0, 4500.0), "{name}");
        assert!((b.start.1 - b.end.1).abs() < 1e-6, "the picked ends lie on one run: {b:?}");
        assert!((b.start.dist(b.end) - 3000.0).abs() < 1e-6, "{name}: {b:?}");
        let m = &h.state().session.model;
        assert_eq!((m.elements.id(b.level), b.offset, b.end_offset), (Some(level), 500.0, None), "{name}: its top on the floor's");
        // The next one's start, a few points off the first one's end, snaps to it as a plan's does.
        let at = h.state().screen3([b.end.0, b.end.1, 500.0]) + vec2(4.0, -3.0);
        click_at(&mut h, at);
        let at = h.state().screen3([2500.0, 3000.0, 500.0]);
        click_at(&mut h, at);
        let made = runs(&h, "beam");
        assert_eq!(made.len(), 2, "{name}: {}", h.state().status.text);
        // Its start snaps to the first beam's end's outline — a corner of it, not its line's end alone: the
        // outline's corners snap now, as the plan's do.
        assert!(made[1].start.dist(b.end) <= 150.0, "{name}: start {:?} is by {:?}", made[1].start, b.end);
        assert_ne!(made[1].start, b.end, "{name}: it took a corner of the outline, not the line's end");
        assert_ne!(made[1].end, made[1].start, "{name}: the run has two picked ends");
    }
}

#[test]
fn a_brace_from_a_floors_top_to_a_walls_top_climbs_from_one_face_to_the_other() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    for (name, mut h) in views(&script) {
        let m = &h.state().session.model;
        let (level, top) = (m.levels()[0].0, m.top_z(m.wall(ids(&h, "wall")[0]).unwrap()));
        shortcut(&mut h, "BM");
        // From the floor's top to the north wall's, its end on the wall's centre line.
        for p in [[3000.0, 4500.0, 500.0], [3000.0, 6000.0, top]] {
            let at = h.state().screen3(p);
            click_at(&mut h, at);
        }
        let made = runs(&h, "beam");
        let b = made.first().unwrap_or_else(|| panic!("{name}: no brace: {}", h.state().status.text));
        assert_eq!((b.start, b.end), (P2(3000.0, 4500.0), P2(3000.0, 6000.0)), "{name}");
        let m = &h.state().session.model;
        assert_eq!(
            (m.elements.id(b.level), b.offset, b.end_offset),
            (Some(level), 500.0, Some(top)),
            "{name}: from 500 up to the wall's top"
        );
    }
}

#[test]
fn pipes_conduits_and_trays_run_on_a_floors_top_in_the_3d_view() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    for (name, mut h) in views(&script) {
        for (k, category) in ["pipe", "conduit", "cabletray"].into_iter().enumerate() {
            h.state_mut().set_tool(Tool::Run(category, None));
            let y = 4000.0 + 500.0 * k as f64;
            for p in [[1000.0, y, 500.0], [3500.0, y, 500.0]] {
                let at = h.state().screen3(p);
                click_at(&mut h, at);
            }
            key(&mut h, Key::Escape);
            let made = runs(&h, category);
            assert_eq!(made.len(), 1, "{name}: {category}: {}", h.state().status.text);
            assert_eq!((made[0].start, made[0].end, made[0].offset), (P2(1000.0, y), P2(3500.0, y), 500.0), "{name}: {category}");
        }
    }
}

#[test]
fn a_column_and_a_structural_column_stand_on_the_face_they_are_placed_on() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    for (name, mut h) in views(&script) {
        let level = h.state().session.model.levels()[0].0;
        for (tool, category, at) in [
            (Tool::Column, "column", [3000.0, 4000.0, 500.0]),
            (Tool::Stand("structuralcolumn"), "structuralcolumn", [2000.0, 4500.0, 500.0]),
        ] {
            h.state_mut().set_tool(tool);
            let on = h.state().screen3(at);
            click_at(&mut h, on);
            let m = &h.state().session.model;
            let placed: Vec<_> = m
                .elements
                .values()
                .filter(|e| e.category() == category)
                .filter_map(|e| match e {
                    Element::Column(c) | Element::StructuralColumn(c) => Some(c),
                    _ => None,
                })
                .collect();
            let c = placed.first().unwrap_or_else(|| panic!("{name}: no {category}: {}", h.state().status.text));
            assert_eq!((c.at, m.elements.id(c.level), c.base_offset), (P2(at[0], at[1]), Some(level), 500.0), "{name}: {category}");
        }
    }
}

#[test]
fn a_floor_is_sketched_in_the_3d_view_on_a_level_face_picked_for_the_work_plane() {
    let script = format!("{BOX}\nfloor {{\"offset\": 500}}");
    let mut h = app(&script);
    press(&mut h, "Default 3D View");
    press(&mut h, "Work Plane");
    pick(&mut h, Role::RadioButton, "Pick a plane");
    press(&mut h, "OK");
    // The floor's top, a level face, where the view sees it over the south wall: the click sets it.
    let at = h.state().screen3([3000.0, 4500.0, 500.0]);
    click_at(&mut h, at);
    assert!(h.state().work_plane.is_none(), "set by the click: {}", h.state().status.text);
    tool(&mut h, "Floor");
    assert!(matches!(h.state().sketch.as_ref().map(|s| s.of), Some(crate::sketch::Of::Floor(_))), "{}", h.state().status.text);
    press(&mut h, "Rectangle");
    for p in [[2000.0, 2000.0, 500.0], [4000.0, 3000.0, 500.0]] {
        let at = h.state().screen3(p);
        click_at(&mut h, at);
    }
    press(&mut h, "Finish Edit Mode");
    assert!(h.state().sketch.is_none(), "{}", h.state().status.text);
    let floors = ids(&h, "floor");
    let new = *floors.last().unwrap();
    assert_eq!((floors.len(), field(&h, new, "offset"), field(&h, new, "area_m2")), (2, json!(500.0), json!(2.0)), "on the floor's top");
}

#[test]
fn over_empty_space_each_tool_says_what_it_needs_in_the_3d_view() {
    let mut h = app(BOX);
    shortcut(&mut h, "3D");
    // Looking level at the model: the ground is edge-on, and nothing is at the view's top corner.
    let cam = h.state().orbit.camera.unwrap();
    h.state_mut().orbit.camera = Some(bimcraft_core::view3d::Camera { elevation: 0.0, ..cam });
    h.run();
    // Under the view's tabs and the Options Bar, which float over its top left corner.
    let empty = h.state().orbit.rect.min + vec2(30.0, 110.0);
    let walls = ids(&h, "wall").len();
    let before = h.state().session.model.elements.len();
    for (tool, says) in [
        (Tool::Wall(Vec::new()), "Place Wall"),
        (Tool::Run("beam", None), "Place Beam"),
        (Tool::Run("pipe", None), "Place Pipe"),
        (Tool::Run("conduit", None), "Place Conduit"),
        (Tool::Run("cabletray", None), "Place Cable Tray"),
        (Tool::Column, "Place Column"),
        (Tool::Stand("structuralcolumn"), "Place Structural Column"),
        (Tool::Stand("footing"), "Place Isolated Footing"),
        (Tool::Component, "Place Component"),
    ] {
        h.state_mut().set_tool(tool.clone());
        h.state_mut().status = crate::Status::default();
        click_at(&mut h, empty);
        let s = &h.state().status;
        assert!(s.error && s.text.contains(says) && s.text.contains("level face"), "{tool:?}: {}", s.text);
    }
    // Floors and ceilings sketch on the work plane, which a level view sees edge-on.
    for name in ["Floor", "Ceiling"] {
        tool(&mut h, name);
        click_at(&mut h, empty);
        let s = &h.state().status;
        assert!(s.error && s.text.contains("edge-on"), "{name}: {}", s.text);
        press(&mut h, "Cancel Edit Mode");
    }
    assert_eq!((ids(&h, "wall").len(), h.state().session.model.elements.len()), (walls, before), "nothing made");
}
