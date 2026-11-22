//! The five studio panels. They only describe what the user asked for; `lib.rs` runs commands.

use egui::Ui;

use super::{ActiveOp, BimContext, BodyMode, DirectMode, Intent, OpKind, SelKind};

/// Top studio bar: category, context, history, cancel and finish.
pub fn top_bar(ui: &mut Ui, dm: &DirectMode) -> Intent {
    let mut out = Intent::None;
    ui.horizontal_centered(|ui| {
        ui.strong("Direct Mode");
        ui.separator();
        ui.label("Target:");
        ui.label("Generic Model");
        ui.separator();
        for context in [BimContext::Full, BimContext::Hidden] {
            if ui.selectable_label(dm.context == context, context.label()).clicked() {
                out = Intent::SetContext(context);
            }
        }
        ui.separator();
        if ui.button("Undo").clicked() {
            out = Intent::Undo;
        }
        if ui.button("Redo").clicked() {
            out = Intent::Redo;
        }
        ui.separator();
        if ui.button("Cancel").clicked() {
            out = Intent::Cancel;
        }
        if ui.button("Finish").clicked() {
            out = Intent::Finish;
        }
    });
    out
}

/// Left icon rail: only operations that work today. Stage 3 adds face sketching and arrays.
pub fn rail(ui: &mut Ui, dm: &DirectMode) -> Intent {
    let mut out = Intent::None;
    ui.vertical(|ui| {
        for op in OpKind::ALL {
            let active = dm.op.as_ref().is_some_and(|a| a.kind == op);
            if ui.selectable_label(active, format!("{}  {}", op.key(), op.label())).clicked() {
                out = Intent::SetOp(op);
            }
        }
    });
    out
}

/// Bottom selection pill: `1` vertex, `2` edge, `3` face, `4` body, `Tab` all.
pub fn pill(ui: &mut Ui, dm: &DirectMode) -> Intent {
    let mut out = Intent::None;
    ui.horizontal(|ui| {
        for kind in SelKind::ALL {
            if ui.selectable_label(dm.selection.kind == kind, format!("{} {}", kind.key(), kind.label())).clicked() {
                out = Intent::SetKind(kind);
            }
        }
        ui.separator();
        ui.label(format!("{} selected", dm.selection.items.len()));
    });
    out
}

/// Bottom-left floating operation card: numbered steps, live fields and mode pills.
pub fn card(ui: &mut Ui, dm: &DirectMode) -> Intent {
    let Some(op) = dm.op.as_ref() else { return Intent::None };
    let mut out = Intent::None;
    ui.vertical(|ui| {
        ui.strong(format!("{}  Step {}", op.kind.label(), op.step));
        for (key, value) in &op.fields {
            ui.horizontal(|ui| {
                ui.label(key);
                let mut v = value.clone();
                if ui.text_edit_singleline(&mut v).changed() {
                    out = Intent::Field(key.clone(), v);
                }
            });
        }
        ui.horizontal(|ui| {
            for mode in [BodyMode::NewBody, BodyMode::Union, BodyMode::Cut] {
                let label = match mode {
                    BodyMode::NewBody => "New body",
                    BodyMode::Union => "Union",
                    BodyMode::Cut => "Cut",
                };
                if ui.selectable_label(op.mode == mode, label).clicked() {
                    out = Intent::SetMode(mode);
                }
            }
        });
        if ui.button("OK ↵").clicked() {
            out = Intent::Confirm;
        }
        if ui.button("Esc").clicked() {
            out = Intent::CancelOp;
        }
    });
    out
}

/// Bottom-right dynamic keymap: the keys that mean something for this selection and operation.
pub fn keymap(ui: &mut Ui, dm: &DirectMode) {
    ui.vertical(|ui| {
        ui.strong("Keys");
        for kind in SelKind::ALL {
            ui.label(format!("{}  {}", kind.key(), kind.label()));
        }
        if let Some(op) = dm.op.as_ref() {
            ui.separator();
            ui.label(format!("{}  {}", op.kind.key(), op.kind.label()));
            ui.label("↵  confirm     Esc  cancel");
        }
    });
}

/// The compact bodies outliner beside the draft.
pub fn outliner(ui: &mut Ui, dm: &DirectMode) {
    ui.vertical(|ui| {
        ui.strong("Bodies");
        for (i, body) in dm.draft.bodies().iter().enumerate() {
            ui.label(format!("{}  {}", i + 1, body.name));
        }
        if dm.draft.is_empty() {
            ui.label("(none)");
        }
    });
}

/// Helper the card uses to keep an edited field in the operation state.
pub fn apply_field(op: &mut Option<ActiveOp>, key: &str, value: String) {
    if let Some(op) = op.as_mut()
        && let Some(field) = op.fields.iter_mut().find(|(k, _)| k == key)
    {
        field.1 = value;
    }
}
