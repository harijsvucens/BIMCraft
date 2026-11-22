//! One built element into one exact solid. A BIM element can render as several touching prisms
//! (a wall split by openings, a layered slab); the kernel sews each and unions them, which is the
//! part the old triangle-only convert refused.

use super::kernel::Committed;
use crate::ids::Id;
use crate::model::Model;
use crate::vec3::P3;

/// Convert one built element into a single watertight exact solid, without touching the model.
pub fn element(m: &Model, id: Id) -> Result<Committed, String> {
    let solids = crate::view3d::solids_of(m, id);
    if solids.is_empty() {
        return Err(format!("{id} has no solid geometry to convert"));
    }
    let parts: Vec<Vec<[P3; 3]>> = solids.iter().map(|s| s.triangles()).collect();
    super::kernel::Kernel::from_meshes(&parts)?.commit()
}
