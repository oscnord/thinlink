//! Retained detail: the vehicle's store of periodic full-resolution snapshots, and the bounded
//! HTTP path that carries a snapshot or a crop of one to the ground at lower priority than live
//! video and control. The relay holds the job table (`relay`), the vehicle works the jobs
//! (`vehicle`), the ground and the lab ask for them (`client`).

use std::io;

pub mod assemble;
pub mod client;
pub mod crop;
pub mod jobs;
pub mod relay;
pub mod store;
pub mod vehicle;
pub mod wire;

#[derive(Debug, thiserror::Error)]
pub enum DetailError {
    #[error("{what}: {source}")]
    Io { what: String, source: io::Error },
    #[error("invalid config: {0}")]
    Config(String),
    #[error("cannot serve on {addr}: {reason}")]
    Bind { addr: String, reason: String },
    #[error("the job server stopped")]
    Stopped,
}

impl DetailError {
    pub(crate) fn io(what: impl Into<String>, source: io::Error) -> Self {
        Self::Io {
            what: what.into(),
            source,
        }
    }
}
