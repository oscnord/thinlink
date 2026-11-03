//! What the drone adds to the video path (`sat-teleop-video`): regional softening and outlines
//! with the detector and tracker frames, the landing tag feed, the predictive display and the
//! detail inset the ground draws, and what retrieval asks of adaptation.

pub mod decorate;
pub mod hooks;
pub mod inset;
pub mod region;
pub mod retrieval;
pub mod tags;
