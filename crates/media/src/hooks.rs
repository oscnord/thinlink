//! What the vehicle adds to the sender: regional video on every frame and the taps that feed the
//! detector, the tracker and the landing tag thread.

use std::sync::Arc;

use crate::region::Regional;
use crate::tags::TagFeed;
use sat_teleop_video::sender::Hooks;

pub fn sender_hooks(regional: &Arc<Regional>, tags: Option<&Arc<TagFeed>>) -> Hooks {
    let mut taps = regional.taps();
    taps.extend(tags.map(|feed| feed.tap()));
    Hooks {
        frame: Some(regional.frame_hook()),
        taps,
        roi_qp_offset: regional.roi_qp_offset(),
    }
}
