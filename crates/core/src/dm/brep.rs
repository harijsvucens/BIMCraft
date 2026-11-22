//! The one writer between an exact solid and a `Form`. `brep` is authoritative; `mesh` is only
//! the tessellated display and picking cache regenerated from it by [`super::kernel`].

use base64::Engine;
use base64::engine::general_purpose::STANDARD;

use super::kernel::{BrepData, Committed};
use crate::family::Form;

/// Base64 of the SCBREP05 bytes. A compact string, not a JSON array of byte integers.
pub fn encode(data: &BrepData) -> String {
    STANDARD.encode(data.bytes())
}

/// Read the exact solid a form carries. Empty or damaged bytes are a refusal, never a panic.
pub fn decode(text: &str) -> Result<BrepData, String> {
    let bytes = STANDARD.decode(text).map_err(|e| format!("that form's exact solid is not base64: {e}"))?;
    BrepData::from_bytes(bytes)
}

/// The form's exact solid, if it has one.
pub fn of(form: &Form) -> Option<Result<BrepData, String>> {
    form.brep.as_deref().map(decode)
}

/// Write one committed result: exact bytes and the display cache in one step, so they cannot
/// disagree. The face tags are re-derived from the bytes on load instead of stored a second time.
pub fn write(form: &mut Form, committed: &Committed) {
    form.brep = Some(encode(&committed.data));
    form.mesh = committed.cache.triangles.clone();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dm::kernel::{Kernel, Prim};

    #[test]
    fn an_exact_solid_written_to_a_form_survives_a_serde_round_trip() {
        let made = Kernel::create(Prim::Box { origin: [0.0; 3], size: [1000.0, 800.0, 600.0] }).expect("box");
        let mut form = Form::default();
        write(&mut form, &made);
        assert!(form.brep.is_some() && !form.mesh.is_empty());
        let text = serde_json::to_string(&form).expect("write form");
        let back: Form = serde_json::from_str(&text).expect("read form");
        let data = of(&back).expect("brep").expect("decode");
        assert_eq!(data, made.data, "the exact bytes survive the file unchanged");
    }

    #[test]
    fn a_damaged_exact_solid_is_refused_not_unwrapped() {
        assert!(decode("not base64 !!").is_err());
        let plain = STANDARD.encode(b"not a solidcore record");
        assert!(decode(&plain).is_err(), "base64 alone is not an exact solid");
    }
}
