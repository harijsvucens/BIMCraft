//! The press → measure → release lifecycle the 3D view's drags share: a level grip, a section-box face, an element
//! handle, a shape lift, a Direct Model box's height and a Push/Pull face. Each site keeps its own hit test, its own
//! measurement and its own commit; this module owns the rules they all follow. It touches no model and runs no command.

use egui::Pos2;

/// The pointer facts a drag needs this frame, read once in the view and passed to every site.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Frame {
    /// A primary press began this frame.
    pub pressed: bool,
    /// The primary button came up this frame.
    pub stopped: bool,
    /// The press was quick enough to be a click, not a drag.
    pub quick: bool,
    /// Where the primary press began, if any.
    pub origin: Option<Pos2>,
    /// Where the pointer is now, if over the view.
    pub latest: Option<Pos2>,
}

/// A held drag: the site's target and the press origin it measures from, both private to the module.
#[derive(Clone)]
pub(crate) struct Held<T> {
    target: T,
    origin: Pos2,
}

impl<T> Held<T> {
    /// The target the site held, for its commit or its paint.
    pub(crate) fn target(&self) -> &T {
        &self.target
    }

    /// Take the target out, as a live drag is taken on release.
    pub(crate) fn into_target(self) -> T {
        self.target
    }
}

/// What a drag gives its caller this frame. `T` is the thing the site holds, `V` what it measured.
#[derive(Debug)]
pub(crate) struct Out<T, V> {
    /// The measured value now, for the preview and for the state the commit reads.
    pub preview: Option<V>,
    /// The value a real drag released: the site's own commit consumes it.
    pub release: Option<(T, V)>,
    /// A held drag ended: the click its release would otherwise make does not go through.
    pub suppressed: bool,
}

impl<T, V> Default for Out<T, V> {
    fn default() -> Self {
        Self { preview: None, release: None, suppressed: false }
    }
}

/// Advance one drag slot. Retire first when its owner is gone, then capture a press, measure against the latest
/// pointer, and release only on a real drag. The press origin the site measures from lives here, not in the payload.
///
/// Ordering is fixed: a retired drag cannot measure or release, even if the button comes up on the same frame. That
/// is what keeps a tool switched mid-drag from being reset by a release that belongs to a gesture it no longer owns.
/// `quick` presses leave no commit; the site's click path still sees them. Measurements that fail (a camera axis
/// pointing at the eye, a plane behind it) are a normal state: no preview, no commit.
pub(crate) fn drive<T, V: Clone>(
    frame: &Frame,
    slot: &mut Option<Held<T>>,
    owned: bool,
    hit: impl FnOnce(Pos2) -> Option<T>,
    measure: impl Fn(&T, Pos2, Pos2) -> Option<V>,
) -> Out<T, V> {
    if slot.is_some() && !owned {
        *slot = None;
        return Out::default();
    }
    if frame.pressed
        && slot.is_none()
        && let Some(origin) = frame.origin
        && let Some(target) = hit(origin)
    {
        *slot = Some(Held { target, origin });
    }
    let measured = match (slot.as_ref(), frame.latest) {
        (Some(held), Some(now)) => measure(&held.target, held.origin, now),
        _ => None,
    };
    let held = frame.stopped.then(|| slot.take()).flatten();
    let was_held = held.is_some();
    let mut out = Out { preview: measured.clone(), release: None, suppressed: was_held };
    if let (Some(held), Some(value)) = (held, measured)
        && !frame.quick
    {
        out.release = Some((held.into_target(), value));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::pos2;

    fn frame(pressed: bool, stopped: bool, quick: bool) -> Frame {
        Frame { pressed, stopped, quick, origin: Some(pos2(10.0, 20.0)), latest: Some(pos2(40.0, 60.0)) }
    }

    fn across(_: &i32, from: Pos2, now: Pos2) -> Option<i32> {
        Some((now.x - from.x) as i32)
    }

    #[test]
    fn a_real_release_commits_once_and_suppresses_the_click() {
        let mut slot = None;
        let press = drive(&frame(true, false, false), &mut slot, true, |at| Some(at.x as i32), across);
        assert!(press.release.is_none());
        assert_eq!(press.preview, Some(30));
        assert!(!press.suppressed);
        assert!(slot.is_some());
        let release = drive(&frame(false, true, false), &mut slot, true, |_| None, across);
        assert_eq!(release.release, Some((10, 30)));
        assert_eq!(release.preview, Some(30));
        assert!(release.suppressed);
        assert!(slot.is_none(), "the slot is taken even when it commits");
    }

    #[test]
    fn a_quick_press_commits_nothing_and_keeps_the_click_path() {
        let mut slot = None;
        drive(&frame(true, false, true), &mut slot, true, |at| Some(at.x as i32), across);
        let release = drive(&frame(false, true, true), &mut slot, true, |_| None, across);
        assert!(release.release.is_none());
        assert!(release.suppressed, "a held press still suppresses the click, as a quick pick does today");
        assert!(slot.is_none());
    }

    #[test]
    fn a_tool_switch_retires_the_drag_so_its_release_commits_nothing() {
        let mut slot = None;
        drive(&frame(true, false, false), &mut slot, true, |at| Some(at.x as i32), across);
        // The tool changed between frames: `owned` is false on the release frame.
        let release = drive(&frame(false, true, false), &mut slot, false, |_| None, across);
        assert!(release.release.is_none(), "the retired drag cannot push");
        assert!(!release.suppressed, "the chosen tool's click is left alone");
        assert!(slot.is_none());
    }

    #[test]
    fn no_press_origin_or_no_hit_means_no_drag() {
        let mut slot = None;
        let mut no_origin = frame(true, false, false);
        no_origin.origin = None;
        drive(&no_origin, &mut slot, true, |at| Some(at.x as i32), across);
        assert!(slot.is_none());
        drive(&frame(true, false, false), &mut slot, true, |_| None, across);
        assert!(slot.is_none());
    }

    #[test]
    fn a_measurement_that_fails_previews_and_commits_nothing() {
        let mut slot = None;
        drive(&frame(true, false, false), &mut slot, true, |at| Some(at.x as i32), |_, _, _| None::<i32>);
        let release = drive(&frame(false, true, false), &mut slot, true, |_| None, |_, _, _| None::<i32>);
        assert!(release.preview.is_none());
        assert!(release.release.is_none());
    }
}
