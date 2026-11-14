//! A plan's guides come from the plan, not from every storey of the project: what a tower's drawing costs.

use bimcraft_core::alignment::{AlignmentConfig, AlignmentGuideService};
use bimcraft_core::sample::tower;
use std::time::Instant;

/// The fastest of three builds: the same answer each time, and the machine's noise left out.
fn built(m: &bimcraft_core::model::Model, level: bimcraft_core::model::Id) -> (std::time::Duration, usize) {
    let mut best = std::time::Duration::MAX;
    let mut refs = 0;
    for _ in 0..3 {
        let t = Instant::now();
        let s = AlignmentGuideService::from_model(m, level, AlignmentConfig::default());
        best = best.min(t.elapsed());
        refs = s.references().len();
        std::hint::black_box(&s);
    }
    (best, refs)
}

/// A storey's guides are its own: the tower's other storeys add no reference point, and are not walked wall by wall to
/// find that out. The first half is the answer; the second is what made a ten-storey project's plan take 29 s to build
/// its guides, a wall of every storey measured against the cut plane one at a time.
#[test]
fn a_storey_s_guides_are_built_from_the_storey_and_not_from_the_whole_project() {
    let one = tower::tower(1).expect("tower");
    let ten = tower::tower(10).expect("tower");
    let level = one.levels()[0].0;
    let (fast, refs) = built(&one, level);
    let (tall, tall_refs) = built(&ten, level);
    assert_eq!(tall_refs, refs, "the storeys above add nothing to this plan's guides");
    assert!(
        tall < fast * 6 + std::time::Duration::from_millis(250),
        "ten storeys took {tall:?} against one storey's {fast:?}: the plan's guides walked the project"
    );
}
