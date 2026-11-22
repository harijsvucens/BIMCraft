//! The studio's working state, outside `Model` until Finish. It holds several bodies so a session
//! can make a second solid and union or subtract it, and its own `Arc` undo stack so `Ctrl+Z` costs
//! a pointer swap instead of a whole-project scene rebuild.

use std::sync::Arc;

use super::kernel::{BoolOp, BrepData, Edit, Kernel, TessCache};
use crate::vec3::P3;

const HISTORY: usize = 100;

/// A draft-local body id. It stays stable through edits, unlike a kernel face id.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BodyId(pub u64);

#[derive(Clone, Debug, PartialEq)]
pub struct DraftBody {
    pub id: BodyId,
    pub name: String,
    /// Draft-local coordinates around the session origin. A move transforms this geometry, so
    /// booleans, picking and previews always share one coordinate space.
    pub data: BrepData,
    pub cache: TessCache,
}

#[derive(Clone, Debug)]
struct Snapshot(Arc<Vec<DraftBody>>);

/// A multi-body direct-modeling session. Nothing here is written to the project until Finish.
#[derive(Clone, Debug)]
pub struct DirectDraft {
    bodies: Vec<DraftBody>,
    next: u64,
    undo: Vec<Snapshot>,
    redo: Vec<Snapshot>,
}

impl Default for DirectDraft {
    fn default() -> Self {
        Self::new()
    }
}

impl DirectDraft {
    pub fn new() -> Self {
        Self { bodies: Vec::new(), next: 0, undo: Vec::new(), redo: Vec::new() }
    }

    pub fn bodies(&self) -> &[DraftBody] {
        &self.bodies
    }

    pub fn body(&self, id: BodyId) -> Option<&DraftBody> {
        self.bodies.iter().find(|b| b.id == id)
    }

    /// Add a sealed body as one undo step and return its stable draft-local id.
    pub fn add(&mut self, name: impl Into<String>, data: BrepData, cache: TessCache) -> BodyId {
        let id = BodyId(self.next);
        self.next += 1;
        let mut bodies = self.bodies.clone();
        bodies.push(DraftBody { id, name: name.into(), data, cache });
        self.push(bodies);
        id
    }

    /// Replace one body's exact solid and cache, keeping its id and name.
    pub fn replace(&mut self, id: BodyId, data: BrepData, cache: TessCache) -> Result<(), String> {
        let i = self.bodies.iter().position(|b| b.id == id).ok_or("there is no such body")?;
        let mut bodies = self.bodies.clone();
        bodies[i] = DraftBody { data, cache, ..bodies[i].clone() };
        self.push(bodies);
        Ok(())
    }

    /// Drop one body, committing the change as an undo step.
    pub fn remove(&mut self, id: BodyId) -> Result<(), String> {
        let i = self.bodies.iter().position(|b| b.id == id).ok_or("there is no such body")?;
        let mut bodies = self.bodies.clone();
        bodies.remove(i);
        self.push(bodies);
        Ok(())
    }

    /// A rigid move of one body's local geometry, committed as one undo step.
    pub fn move_body(&mut self, id: BodyId, delta: P3) -> Result<(), String> {
        let i = self.bodies.iter().position(|b| b.id == id).ok_or("there is no such body")?;
        let out = Kernel::from_data(&self.bodies[i].data)?.edit(Edit::Move { delta })?;
        let mut bodies = self.bodies.clone();
        bodies[i] = DraftBody { data: out.data, cache: out.cache, ..bodies[i].clone() };
        self.push(bodies);
        Ok(())
    }

    /// Boolean one body with another, keeping the target's id and name. The tool is consumed.
    /// Ordered selections make the first pick the target, so Union/Cut/Intersect read the card
    /// the same way.
    pub fn boolean(&mut self, target: BodyId, tool: BodyId, op: BoolOp) -> Result<(), String> {
        let t = self.bodies.iter().position(|b| b.id == target).ok_or("there is no target body")?;
        let u = self.bodies.iter().position(|b| b.id == tool).ok_or("there is no tool body")?;
        if t == u {
            return Err("a body cannot cut itself".into());
        }
        let out = super::kernel::boolean(&self.bodies[t].data, &self.bodies[u].data, op)?;
        let mut bodies = self.bodies.clone();
        bodies[t] = DraftBody { data: out.data, cache: out.cache, ..bodies[t].clone() };
        bodies.remove(u);
        self.push(bodies);
        Ok(())
    }

    /// Commit one step. The previous state is shared, not copied — an undo takes ~1–2 ms.
    pub fn push(&mut self, bodies: Vec<DraftBody>) {
        self.undo.push(Snapshot(Arc::new(std::mem::replace(&mut self.bodies, bodies))));
        if self.undo.len() > HISTORY {
            self.undo.remove(0);
        }
        self.redo.clear();
    }

    pub fn undo(&mut self) -> bool {
        let Some(was) = self.undo.pop() else { return false };
        self.redo.push(Snapshot(Arc::new(std::mem::replace(&mut self.bodies, (*was.0).clone()))));
        true
    }

    pub fn redo(&mut self) -> bool {
        let Some(next) = self.redo.pop() else { return false };
        self.undo.push(Snapshot(Arc::new(std::mem::replace(&mut self.bodies, (*next.0).clone()))));
        true
    }

    pub fn undo_len(&self) -> usize {
        self.undo.len()
    }
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body(tag: u8) -> DraftBody {
        let mut bytes = b"SCBREP05".to_vec();
        bytes.push(tag);
        DraftBody {
            id: BodyId(tag as u64),
            name: format!("Body {tag}"),
            data: BrepData::from_bytes(bytes).expect("bytes"),
            cache: TessCache::default(),
        }
    }

    #[test]
    fn a_draft_holds_several_bodies_and_undo_steps_through_them() {
        let mut d = DirectDraft::new();
        d.push(vec![body(1)]);
        d.push(vec![body(1), body(2)]);
        assert_eq!(d.bodies().len(), 2);
        assert!(d.undo());
        assert_eq!(d.bodies().len(), 1, "undo drops the second body");
        assert!(d.undo());
        assert!(d.is_empty(), "undo reaches the empty session");
        assert!(!d.undo(), "there is no earlier step");
        assert!(d.redo());
        assert_eq!(d.bodies().len(), 1);
        assert!(d.redo());
        assert_eq!(d.bodies().len(), 2);
    }

    #[test]
    fn a_new_edit_clears_the_redo_stack_and_history_is_bounded() {
        let mut d = DirectDraft::new();
        d.push(vec![body(1)]);
        assert!(d.undo());
        d.push(vec![body(2)]);
        assert!(!d.redo(), "an edit after undo drops the old future");
        for _ in 0..150 {
            d.push(vec![body(3)]);
        }
        assert_eq!(d.undo_len(), HISTORY, "the stack never grows past its cap");
    }

    fn solid(at: [f64; 3]) -> (BrepData, TessCache) {
        let made = Kernel::create(super::super::kernel::Prim::Box { origin: at, size: [1000.0, 800.0, 600.0] }).expect("box");
        (made.data, made.cache)
    }

    #[test]
    fn a_rigid_move_transforms_one_body_and_undo_puts_it_back() {
        let (data, cache) = solid([0.0; 3]);
        let mut d = DirectDraft::new();
        let id = d.add("Box", data, cache);
        let before = d.body(id).expect("the body").cache.vertices.clone();
        d.move_body(id, [500.0, 0.0, 0.0]).expect("move");
        assert_eq!(d.bodies().len(), 1, "a move does not add a body");
        assert_ne!(d.body(id).expect("the body").cache.vertices, before);
        assert!(d.undo());
        assert_eq!(d.body(id).expect("the body").cache.vertices, before);
    }

    #[test]
    fn a_boolean_keeps_the_target_id_and_consumes_the_tool() {
        let (a, ac) = solid([0.0; 3]);
        let (b, bc) = solid([500.0, 0.0, 0.0]);
        let mut d = DirectDraft::new();
        let target = d.add("A", a, ac);
        let tool = d.add("B", b, bc);
        d.boolean(target, tool, super::super::kernel::BoolOp::Union).expect("union");
        assert_eq!(d.bodies().len(), 1, "the tool is gone");
        assert_eq!(d.body(target).expect("the target").name, "A");
        assert!(d.body(tool).is_none());
        assert!(d.undo());
        assert_eq!(d.bodies().len(), 2, "one undo brings the tool back");
    }
}
