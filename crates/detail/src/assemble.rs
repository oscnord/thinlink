//! Assembling an object from the chunks of an upload, each checked on arrival and the whole at the
//! end. Nothing is trusted: a chunk's index, size and hash, and the declared totals, all have to
//! hold before bytes are kept or handed on.

use std::collections::BTreeMap;

use sat_teleop_policy::retrieval::MAX_CHUNK;

use crate::wire::sha256_hex;

/// The smallest chunk the vehicle sends but the last; bounds how many chunks a job may hold.
pub const MIN_CHUNK: usize = 512;

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ChunkError {
    #[error("a chunk holds 1 to {MAX_CHUNK} bytes")]
    Size,
    #[error("the chunk index is beyond the job's chunk count")]
    BeyondCount,
    #[error("the chunk does not match its hash")]
    BadHash,
    #[error("chunk already received with other bytes")]
    Conflict,
    #[error("the chunks exceed the job's byte limit")]
    TooLarge,
    #[error("the chunks received are not the chunks declared")]
    Missing,
    #[error("the object does not match its declared size and hash")]
    TotalMismatch,
}

/// What a chunk did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Accepted {
    Stored,
    /// The same bytes arrived before, a retry after a lost answer.
    Repeated,
}

/// The chunks of one job, in whatever order they arrive.
#[derive(Debug)]
pub struct Assembly {
    max_bytes: usize,
    max_chunks: usize,
    chunks: BTreeMap<usize, Vec<u8>>,
    bytes: usize,
}

impl Assembly {
    pub fn new(max_bytes: u32) -> Self {
        let max_bytes = usize::try_from(max_bytes).unwrap_or(usize::MAX);
        Self {
            max_bytes,
            max_chunks: max_bytes.div_ceil(MIN_CHUNK),
            chunks: BTreeMap::new(),
            bytes: 0,
        }
    }

    pub fn chunks(&self) -> usize {
        self.chunks.len()
    }

    /// Keeps chunk `index` if it is within the job's bounds and matches `sha256` (lowercase hex).
    pub fn put(&mut self, index: usize, data: &[u8], sha256: &str) -> Result<Accepted, ChunkError> {
        if data.is_empty() || data.len() > MAX_CHUNK {
            return Err(ChunkError::Size);
        }
        if index >= self.max_chunks {
            return Err(ChunkError::BeyondCount);
        }
        if sha256_hex(data) != sha256 {
            return Err(ChunkError::BadHash);
        }
        if let Some(kept) = self.chunks.get(&index) {
            return if kept == data {
                Ok(Accepted::Repeated)
            } else {
                Err(ChunkError::Conflict)
            };
        }
        if self.bytes.saturating_add(data.len()) > self.max_bytes {
            return Err(ChunkError::TooLarge);
        }
        self.bytes += data.len();
        self.chunks.insert(index, data.to_vec());
        Ok(Accepted::Stored)
    }

    /// The object, if exactly the `chunks` chunks `0..chunks` arrived and together weigh `bytes`
    /// and hash to `sha256`.
    pub fn finish(&self, chunks: usize, bytes: usize, sha256: &str) -> Result<Vec<u8>, ChunkError> {
        let contiguous =
            chunks > 0 && self.chunks.len() == chunks && self.chunks.keys().copied().eq(0..chunks);
        if !contiguous {
            return Err(ChunkError::Missing);
        }
        let object: Vec<u8> = self.chunks.values().flatten().copied().collect();
        if object.len() != bytes || sha256_hex(&object) != sha256 {
            return Err(ChunkError::TotalMismatch);
        }
        Ok(object)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A step of an upload: a chunk and the hash it claims, or the end with its declared totals.
    enum Step {
        Send(usize, Vec<u8>, String, Result<Accepted, ChunkError>),
        End(usize, usize, String, Result<Vec<u8>, ChunkError>),
    }

    fn send(index: usize, data: &[u8], expected: Result<Accepted, ChunkError>) -> Step {
        Step::Send(index, data.to_vec(), sha256_hex(data), expected)
    }

    fn end(chunks: usize, parts: &[&[u8]], expected: Result<Vec<u8>, ChunkError>) -> Step {
        let whole = parts.concat();
        Step::End(chunks, whole.len(), sha256_hex(&whole), expected)
    }

    #[test]
    fn keeps_only_chunks_that_hold_to_their_bounds_and_hashes() {
        use Accepted::{Repeated, Stored};
        let (a, b, c) = (&[1_u8; 600][..], &[2_u8; 600][..], &[3_u8; 100][..]);
        let oversize = vec![9_u8; MAX_CHUNK + 1];
        let cases: Vec<(&str, u32, Vec<Step>)> = vec![
            (
                "out of order, and the end",
                2000,
                vec![
                    send(2, c, Ok(Stored)),
                    send(0, a, Ok(Stored)),
                    send(1, b, Ok(Stored)),
                    end(3, &[a, b, c], Ok([a, b, c].concat())),
                ],
            ),
            (
                "the same chunk twice is one chunk",
                2000,
                vec![
                    send(0, a, Ok(Stored)),
                    send(0, a, Ok(Repeated)),
                    send(1, b, Ok(Stored)),
                    end(2, &[a, b], Ok([a, b].concat())),
                ],
            ),
            (
                "the same index with other bytes",
                2000,
                vec![
                    send(0, a, Ok(Stored)),
                    send(0, b, Err(ChunkError::Conflict)),
                ],
            ),
            (
                "a chunk that does not match its hash",
                2000,
                vec![Step::Send(
                    0,
                    a.to_vec(),
                    sha256_hex(b),
                    Err(ChunkError::BadHash),
                )],
            ),
            (
                "an empty chunk",
                2000,
                vec![send(0, &[], Err(ChunkError::Size))],
            ),
            (
                "a chunk over the chunk limit",
                100_000,
                vec![send(0, &oversize, Err(ChunkError::Size))],
            ),
            (
                "an index beyond what the job can hold",
                2000,
                vec![send(4, a, Err(ChunkError::BeyondCount))],
            ),
            (
                "more bytes than the job allows",
                1000,
                vec![
                    send(0, a, Ok(Stored)),
                    send(1, b, Err(ChunkError::TooLarge)),
                ],
            ),
            (
                "the end with a chunk missing",
                2000,
                vec![
                    send(0, a, Ok(Stored)),
                    send(2, c, Ok(Stored)),
                    end(3, &[a, c], Err(ChunkError::Missing)),
                ],
            ),
            (
                "the end with more chunks than declared",
                2000,
                vec![
                    send(0, a, Ok(Stored)),
                    send(1, b, Ok(Stored)),
                    end(1, &[a], Err(ChunkError::Missing)),
                ],
            ),
            (
                "the end with a wrong total size",
                2000,
                vec![
                    send(0, a, Ok(Stored)),
                    Step::End(1, 599, sha256_hex(a), Err(ChunkError::TotalMismatch)),
                ],
            ),
            (
                "the end with a wrong total hash",
                2000,
                vec![
                    send(0, a, Ok(Stored)),
                    Step::End(1, 600, sha256_hex(b), Err(ChunkError::TotalMismatch)),
                ],
            ),
        ];
        for (name, max_bytes, steps) in cases {
            let mut assembly = Assembly::new(max_bytes);
            for step in steps {
                match step {
                    Step::Send(index, data, hash, expected) => {
                        assert_eq!(assembly.put(index, &data, &hash), expected, "{name}");
                    }
                    Step::End(chunks, bytes, hash, expected) => {
                        let finished = std::mem::replace(&mut assembly, Assembly::new(0));
                        assert_eq!(finished.finish(chunks, bytes, &hash), expected, "{name}");
                    }
                }
            }
        }
    }
}
