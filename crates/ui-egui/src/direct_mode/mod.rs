//! The Plasticity-style studio shell. Owns selection, the active operation and the draft;
//! `lib.rs` keeps one `Option<DirectMode>` field and `orbit.rs` one delegation guard.

use bimcraft_core::dm::draft::{BodyId, DirectDraft};
use bimcraft_core::dm::kernel::{BoolOp, FaceId, Kernel, Prim, TessCache};
pub use bimcraft_core::dm::pick::SelItem;
use bimcraft_core::model::{Element, Id, Model};
use bimcraft_core::view3d::Surface;

pub mod panels;
pub mod stem;

/// The five selection modes. `All` cycles through the others with `Tab`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SelKind {
    Vertex,
    Edge,
    Face,
    Body,
    #[default]
    All,
}

impl SelKind {
    pub const ALL: [SelKind; 5] = [Self::Vertex, Self::Edge, Self::Face, Self::Body, Self::All];
    pub fn key(self) -> &'static str {
        match self {
            Self::Vertex => "1",
            Self::Edge => "2",
            Self::Face => "3",
            Self::Body => "4",
            Self::All => "Tab",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Vertex => "Vertex",
            Self::Edge => "Edge",
            Self::Face => "Face",
            Self::Body => "Body",
            Self::All => "All",
        }
    }
}

/// An ordered selection: the first item is the target of a boolean, the second the tool.
#[derive(Clone, Debug, Default)]
pub struct Selection {
    pub kind: SelKind,
    pub items: Vec<SelItem>,
}

impl Selection {
    /// A click replaces; Shift+Click appends or toggles, keeping the pick order.
    pub fn pick(&mut self, item: SelItem, additive: bool) {
        if !additive {
            self.items.clear();
            self.items.push(item);
            return;
        }
        if let Some(i) = self.items.iter().position(|q| *q == item) {
            self.items.remove(i);
        } else {
            self.items.push(item);
        }
    }

    pub fn clear(&mut self) {
        self.items.clear();
    }

    pub fn first_face(&self) -> Option<FaceId> {
        self.items.iter().find_map(|q| match q {
            SelItem::Face(f) => Some(*f),
            _ => None,
        })
    }

    pub fn first_body(&self) -> Option<BodyId> {
        self.items.iter().find_map(|q| match q {
            SelItem::Body(b) => Some(BodyId(*b)),
            _ => None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpKind {
    Box,
    Cylinder,
    PushPull,
    Bevel,
    Move,
    Union,
    Cut,
    Intersect,
}

impl OpKind {
    pub const ALL: [OpKind; 8] =
        [Self::Box, Self::Cylinder, Self::PushPull, Self::Bevel, Self::Move, Self::Union, Self::Cut, Self::Intersect];
    pub fn label(self) -> &'static str {
        match self {
            Self::Box => "Box",
            Self::Cylinder => "Cylinder",
            Self::PushPull => "Push/Pull",
            Self::Bevel => "Bevel/Fillet",
            Self::Move => "Move",
            Self::Union => "Union",
            Self::Cut => "Cut",
            Self::Intersect => "Intersect",
        }
    }
    pub fn key(self) -> &'static str {
        match self {
            Self::Box => "B",
            Self::Cylinder => "C",
            Self::PushPull => "E",
            Self::Bevel => "F",
            Self::Move => "G",
            Self::Union => "Q",
            Self::Cut => "W",
            Self::Intersect => "⇧E",
        }
    }
    pub fn boolean(self) -> Option<BoolOp> {
        match self {
            Self::Union => Some(BoolOp::Union),
            Self::Cut => Some(BoolOp::Subtract),
            Self::Intersect => Some(BoolOp::Intersect),
            _ => None,
        }
    }
    pub fn primitive(self) -> Option<&'static str> {
        match self {
            Self::Box => Some("box"),
            Self::Cylinder => Some("cylinder"),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyMode {
    NewBody,
    Union,
    Cut,
}

/// The floating operation card: a numbered step, live fields, and whether the release makes a
/// new body or joins one.
#[derive(Clone, Debug)]
pub struct ActiveOp {
    pub kind: OpKind,
    pub step: usize,
    pub fields: Vec<(String, String)>,
    pub mode: BodyMode,
}

impl ActiveOp {
    pub fn new(kind: OpKind) -> Self {
        let fields = match kind {
            OpKind::Box => vec![("Length".into(), "1000".into()), ("Width".into(), "800".into()), ("Height".into(), "600".into())],
            OpKind::Cylinder => vec![("Radius".into(), "500".into()), ("Height".into(), "600".into())],
            OpKind::PushPull | OpKind::Move => vec![("Distance".into(), "0".into())],
            OpKind::Bevel => vec![("Radius".into(), "50".into()), ("Segments".into(), "1".into())],
            _ => Vec::new(),
        };
        Self { kind, step: 1, fields, mode: BodyMode::NewBody }
    }

    pub fn number(&self, key: &str) -> Option<f64> {
        self.fields.iter().find(|(k, _)| k == key).and_then(|(_, v)| v.parse::<f64>().ok()).filter(|v| v.is_finite())
    }

    pub fn set_number(&mut self, key: &str, value: f64) {
        if let Some((_, v)) = self.fields.iter_mut().find(|(k, _)| k == key) {
            *v = format!("{value:.0}");
        }
    }
}

/// Non-destructive context: a runtime `Look` decision only. No project VG is written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BimContext {
    Full,
    Hidden,
}

impl BimContext {
    pub fn label(self) -> &'static str {
        match self {
            Self::Full => "Full",
            Self::Hidden => "Hidden",
        }
    }
}

/// What the studio asks `lib.rs` to do after a panel or a key.
#[derive(Clone, Debug)]
pub enum Intent {
    None,
    Cancel,
    Finish,
    SetKind(SelKind),
    SetOp(OpKind),
    SetContext(BimContext),
    Undo,
    Redo,
    Confirm,
    /// An operation card was confirmed: `lib.rs` runs the matching command.
    Run(ActiveOp),
    Field(String, String),
    SetMode(BodyMode),
    CancelOp,
}

/// One mode instance. `draft` is the core-owned session outside `Model`; the rest is UI state.
#[derive(Clone, Debug)]
pub struct DirectMode {
    pub draft: DirectDraft,
    pub selection: Selection,
    pub op: Option<ActiveOp>,
    pub context: BimContext,
    /// The element whose place the draft took, hidden through `Shown.hidden` only.
    pub hidden: Option<Id>,
    /// The session origin: every `BrepData` is in draft-local world coordinates around it.
    pub origin: [f64; 3],
}

impl Default for DirectMode {
    fn default() -> Self {
        Self::new()
    }
}

impl DirectMode {
    pub fn new() -> Self {
        Self {
            draft: DirectDraft::new(),
            selection: Selection::default(),
            op: None,
            context: BimContext::Full,
            hidden: None,
            origin: [0.0; 3],
        }
    }

    pub fn undo(&mut self) -> bool {
        self.draft.undo()
    }

    pub fn redo(&mut self) -> bool {
        self.draft.redo()
    }

    pub fn begin(&mut self, kind: OpKind) {
        self.op = Some(ActiveOp::new(kind));
    }

    pub fn confirm(&mut self) -> Intent {
        let Some(op) = self.op.take() else { return Intent::None };
        Intent::Run(op)
    }

    /// Bodies as JSON for the `finish_direct` command.
    pub fn bodies_json(&self) -> serde_json::Value {
        let bodies: Vec<serde_json::Value> = self
            .draft
            .bodies()
            .iter()
            .map(|b| {
                serde_json::json!({
                    "name": b.name,
                    "brep": bimcraft_core::dm::brep::encode(&b.data),
                })
            })
            .collect();
        serde_json::Value::Array(bodies)
    }

    pub fn create_prim(&self, op: &ActiveOp) -> Option<Prim> {
        let at = self.origin;
        match op.kind {
            OpKind::Box => Some(Prim::Box { origin: at, size: [op.number("Length")?, op.number("Width")?, op.number("Height")?] }),
            OpKind::Cylinder => Some(Prim::Cylinder { origin: at, radius: op.number("Radius")?, height: op.number("Height")? }),
            _ => None,
        }
    }
}

/// The exact solid's cache for a component already in the project, so a first pick on a BIM
/// element can address its faces and edges in the seam's own id space.
pub fn cache_of_model(m: &Model, id: Id) -> Option<TessCache> {
    let Element::Component(c) = m.elements.get(id)? else { return None };
    let form = m.families.get(&c.family)?.forms.first()?;
    let data = bimcraft_core::dm::brep::of(form)?.ok()?;
    let k = Kernel::from_data(&data).ok()?;
    k.commit().ok().map(|c| c.cache)
}

/// Turn a surface the 3D view picked into an ordered-selection item. Face ids come straight from
/// the tessellated cache; vertices and edges are a nearest-hit on that same world point.
pub fn pick_from_surface(cache: &TessCache, kind: SelKind, s: &Surface) -> Option<SelItem> {
    match kind {
        SelKind::Vertex => bimcraft_core::dm::pick::vertex(cache, s.at, 25.0).map(|p| p.item),
        SelKind::Edge => bimcraft_core::dm::pick::edge(cache, s.at, 25.0).map(|p| p.item),
        SelKind::Face => s.face.map(|f| SelItem::Face(FaceId(f))),
        SelKind::Body | SelKind::All => None,
    }
}
