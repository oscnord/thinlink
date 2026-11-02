//! End-to-end authentication of control messages: a controller signs every envelope, lease request
//! and lease release with its Ed25519 key (`ed25519-dalek`) and the vehicle checks it against the
//! controller's public key before anything changes. The relay holds neither key, so it cannot make
//! one. What is signed starts with a constant per message kind and the vehicle's id, so a signature
//! made for another kind of message or another vehicle verifies nowhere else. An envelope's
//! signature covers its JSON without the signature itself: epoch, lease, sequence, challenge, kind,
//! frame and send time, so the freshness rules keep covering replay. Keys are one line of base64
//! (32 bytes) in a file, like the `WireGuard` keys; `generate` makes a pair.

use std::fs;
use std::io::{self, Read, Write};
use std::os::unix::fs::{MetadataExt, OpenOptionsExt};
use std::path::{Path, PathBuf};

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD;
use ed25519_dalek::{Signature, Signer as _, SigningKey, VerifyingKey};
use serde::Deserialize;

use sat_teleop_policy::token::exposed;

use crate::envelope::{Envelope, Refusal};

const KEY_LEN: usize = 32;
const ENVELOPE: &str = "sat-teleop/envelope/v1";
const LEASE_REQUEST: &str = "sat-teleop/lease-request/v1";
const LEASE_RELEASE: &str = "sat-teleop/lease-release/v1";
const MAX_ID_LEN: usize = 64;

#[derive(Debug, thiserror::Error)]
pub enum KeyError {
    #[error("cannot read the key {path}: {source}")]
    Read { path: PathBuf, source: io::Error },
    #[error("{path} is not a key: expected one line of base64 holding 32 bytes")]
    Format { path: PathBuf },
    #[error("{path} can be used by others (mode {mode:o}): run chmod 600 {path}")]
    Mode { path: PathBuf, mode: u32 },
    #[error("cannot write the key {path}: {source}")]
    Write { path: PathBuf, source: io::Error },
    #[error("cannot read random bytes: {0}")]
    Random(io::Error),
    #[error("cannot encode the message to sign: {0}")]
    Encode(serde_json::Error),
}

/// Why a vehicle id is refused.
#[derive(Debug, thiserror::Error)]
#[error("a vehicle id is 1 to {MAX_ID_LEN} printable ASCII characters")]
pub struct BadVehicleId;

/// The vehicle a signature is made for; the vehicle's config and the controller's name the same
/// one.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(try_from = "String")]
pub struct VehicleId(String);

impl TryFrom<String> for VehicleId {
    type Error = BadVehicleId;

    fn try_from(id: String) -> Result<Self, BadVehicleId> {
        let printable = id.bytes().all(|b| b.is_ascii_graphic());
        if id.is_empty() || id.len() > MAX_ID_LEN || !printable {
            return Err(BadVehicleId);
        }
        Ok(Self(id))
    }
}

impl std::str::FromStr for VehicleId {
    type Err = BadVehicleId;

    fn from_str(id: &str) -> Result<Self, BadVehicleId> {
        id.to_owned().try_into()
    }
}

/// What a signature covers.
enum Message<'a> {
    Envelope(&'a Envelope),
    LeaseRequest { nonce: u64 },
    LeaseRelease { epoch: u64, lease: u64 },
}

/// A controller's private key, with the vehicle it signs for.
pub struct Signer {
    key: SigningKey,
    vehicle: VehicleId,
}

/// A controller's public key, as the vehicle holds it.
#[derive(Debug, Clone)]
pub struct Verifier {
    key: VerifyingKey,
    vehicle: VehicleId,
}

impl Signer {
    pub fn load(path: &Path, vehicle: &VehicleId) -> Result<Self, KeyError> {
        Ok(Self {
            key: SigningKey::from_bytes(&read_private_key(path)?),
            vehicle: vehicle.clone(),
        })
    }

    /// A key that exists only in memory; the lab uses it as the wrong key.
    pub fn random(vehicle: &VehicleId) -> Result<Self, KeyError> {
        Ok(Self {
            key: SigningKey::from_bytes(&random_seed()?),
            vehicle: vehicle.clone(),
        })
    }

    /// The same key, signing for another vehicle; the lab uses it to show a vehicle refusing a
    /// signature made for a different one.
    #[must_use]
    pub fn for_vehicle(&self, vehicle: &VehicleId) -> Self {
        Self {
            key: self.key.clone(),
            vehicle: vehicle.clone(),
        }
    }

    pub fn verifier(&self) -> Verifier {
        Verifier {
            key: self.key.verifying_key(),
            vehicle: self.vehicle.clone(),
        }
    }

    /// `envelope` with its signature set.
    pub fn sign(&self, envelope: Envelope) -> Result<Envelope, KeyError> {
        let sig = Some(self.signature(&Message::Envelope(&envelope))?);
        Ok(Envelope { sig, ..envelope })
    }

    /// The signature of a lease request with `nonce`.
    pub fn sign_request(&self, nonce: u64) -> Result<String, KeyError> {
        self.signature(&Message::LeaseRequest { nonce })
    }

    /// The signature of the release of `lease` of the vehicle's session `epoch`.
    pub fn sign_release(&self, epoch: u64, lease: u64) -> Result<String, KeyError> {
        self.signature(&Message::LeaseRelease { epoch, lease })
    }

    fn signature(&self, message: &Message) -> Result<String, KeyError> {
        let bytes = signed_bytes(&self.vehicle, message)?;
        Ok(STANDARD.encode(self.key.sign(&bytes).to_bytes()))
    }
}

impl Verifier {
    pub fn load(path: &Path, vehicle: &VehicleId) -> Result<Self, KeyError> {
        let bytes = read_key(path)?;
        let key = VerifyingKey::from_bytes(&bytes).map_err(|_| KeyError::Format {
            path: path.to_owned(),
        })?;
        Ok(Self {
            key,
            vehicle: vehicle.clone(),
        })
    }

    /// `Unsigned` without a signature, `BadSignature` for one that is not this key's over this
    /// envelope.
    pub fn verify(&self, envelope: &Envelope) -> Result<(), Refusal> {
        self.check(&Message::Envelope(envelope), envelope.sig.as_deref())
    }

    pub fn verify_request(&self, nonce: u64, signature: Option<&str>) -> Result<(), Refusal> {
        self.check(&Message::LeaseRequest { nonce }, signature)
    }

    pub fn verify_release(
        &self,
        epoch: u64,
        lease: u64,
        signature: Option<&str>,
    ) -> Result<(), Refusal> {
        self.check(&Message::LeaseRelease { epoch, lease }, signature)
    }

    fn check(&self, message: &Message, signature: Option<&str>) -> Result<(), Refusal> {
        let text = signature.ok_or(Refusal::Unsigned)?;
        let bytes = STANDARD.decode(text).map_err(|_| Refusal::BadSignature)?;
        let signature = Signature::from_slice(&bytes).map_err(|_| Refusal::BadSignature)?;
        let signed = signed_bytes(&self.vehicle, message).map_err(|_| Refusal::BadSignature)?;
        self.key
            .verify_strict(&signed, &signature)
            .map_err(|_| Refusal::BadSignature)
    }
}

/// What a signature covers: the message kind's constant, the vehicle's id, then the message (an
/// envelope as its JSON with `sig` left out).
fn signed_bytes(vehicle: &VehicleId, message: &Message) -> Result<Vec<u8>, KeyError> {
    let (context, payload) = match message {
        Message::Envelope(envelope) => {
            let unsigned = Envelope {
                sig: None,
                ..(*envelope).clone()
            };
            let json = serde_json::to_vec(&unsigned).map_err(KeyError::Encode)?;
            (ENVELOPE, json)
        }
        Message::LeaseRequest { nonce } => (LEASE_REQUEST, nonce.to_be_bytes().to_vec()),
        Message::LeaseRelease { epoch, lease } => (
            LEASE_RELEASE,
            [epoch.to_be_bytes(), lease.to_be_bytes()].concat(),
        ),
    };
    Ok([
        context.as_bytes(),
        b"\0",
        vehicle.0.as_bytes(),
        b"\0",
        &payload,
    ]
    .concat())
}

/// A private key, from a file only its owner can use.
fn read_private_key(path: &Path) -> Result<[u8; KEY_LEN], KeyError> {
    let read = |source| KeyError::Read {
        path: path.to_owned(),
        source,
    };
    let mode = fs::metadata(path).map_err(read)?.mode() & 0o777;
    if exposed(mode) {
        return Err(KeyError::Mode {
            path: path.to_owned(),
            mode,
        });
    }
    read_key(path)
}

fn read_key(path: &Path) -> Result<[u8; KEY_LEN], KeyError> {
    let text = fs::read_to_string(path).map_err(|source| KeyError::Read {
        path: path.to_owned(),
        source,
    })?;
    let format = || KeyError::Format {
        path: path.to_owned(),
    };
    let bytes = STANDARD.decode(text.trim()).map_err(|_| format())?;
    bytes.try_into().map_err(|_| format())
}

fn random_seed() -> Result<[u8; KEY_LEN], KeyError> {
    let mut seed = [0_u8; KEY_LEN];
    fs::File::open("/dev/urandom")
        .and_then(|mut random| random.read_exact(&mut seed))
        .map_err(KeyError::Random)?;
    Ok(seed)
}

/// Writes `<dir>/ctl-<name>.key` (private, mode 600) and `<dir>/ctl-<name>.pub`, keeping an
/// existing private key; returns whether the pair was made now.
pub fn generate(dir: &Path, name: &str) -> Result<bool, KeyError> {
    let private = dir.join(format!("ctl-{name}.key"));
    let public = dir.join(format!("ctl-{name}.pub"));
    let created = !private.exists();
    if created {
        write(&private, &STANDARD.encode(random_seed()?), 0o600)?;
    }
    let key = SigningKey::from_bytes(&read_private_key(&private)?);
    write(
        &public,
        &STANDARD.encode(key.verifying_key().to_bytes()),
        0o644,
    )?;
    Ok(created)
}

fn write(path: &Path, text: &str, mode: u32) -> Result<(), KeyError> {
    let error = |source| KeyError::Write {
        path: path.to_owned(),
        source,
    };
    fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .mode(mode)
        .open(path)
        .and_then(|mut file| writeln!(file, "{text}"))
        .map_err(error)
}

#[cfg(test)]
mod tests {
    use crate::envelope::Kind;

    use super::*;

    fn vehicle(id: &str) -> VehicleId {
        id.parse().unwrap()
    }

    fn envelope() -> Envelope {
        Envelope {
            epoch: 7,
            lease: 1,
            seq: 3,
            challenge: 9,
            kind: Kind::Continuous,
            mavlink: Some("AAAA".to_owned()),
            sent_unix_ms: 1000,
            sig: None,
        }
    }

    #[test]
    fn only_the_signers_own_untouched_envelope_verifies() {
        let signer = Signer::random(&vehicle("v1")).unwrap();
        let other = Signer::random(&vehicle("v1")).unwrap();
        let signed = signer.sign(envelope()).unwrap();
        let tampered = [
            Envelope {
                seq: 4,
                ..signed.clone()
            },
            Envelope {
                challenge: 10,
                ..signed.clone()
            },
            Envelope {
                mavlink: Some("AAAB".to_owned()),
                ..signed.clone()
            },
            Envelope {
                kind: Kind::Liveness,
                ..signed.clone()
            },
        ];
        assert_eq!(signer.verifier().verify(&signed), Ok(()));
        for envelope in tampered {
            assert_eq!(
                signer.verifier().verify(&envelope),
                Err(Refusal::BadSignature)
            );
        }
        assert_eq!(other.verifier().verify(&signed), Err(Refusal::BadSignature));
        assert_eq!(
            signer.verifier().verify(&envelope()),
            Err(Refusal::Unsigned)
        );
    }

    /// A signature is good for one kind of message to one vehicle, made by the key the vehicle holds.
    #[test]
    fn a_signature_verifies_only_for_its_kind_and_vehicle() {
        let ours = Signer::random(&vehicle("ours")).unwrap();
        let verifier = ours.verifier();
        let stranger = Signer::random(&vehicle("ours")).unwrap();
        let request = ours.sign_request(5).unwrap();
        let release = ours.sign_release(7, 1).unwrap();
        let for_another_vehicle = ours.for_vehicle(&vehicle("theirs"));
        let cases = [
            (
                "an envelope signed for another vehicle",
                verifier.verify(&for_another_vehicle.sign(envelope()).unwrap()),
            ),
            (
                "a request signed for another vehicle",
                verifier.verify_request(5, Some(&for_another_vehicle.sign_request(5).unwrap())),
            ),
            (
                "a release signed for another vehicle",
                verifier.verify_release(
                    7,
                    1,
                    Some(&for_another_vehicle.sign_release(7, 1).unwrap()),
                ),
            ),
            (
                "a release signed by no key",
                verifier.verify_release(7, 1, None),
            ),
            (
                "a release signed by the wrong key",
                verifier.verify_release(7, 1, Some(&stranger.sign_release(7, 1).unwrap())),
            ),
            (
                "a release of another lease",
                verifier.verify_release(7, 2, Some(&release)),
            ),
            (
                "a release of another session",
                verifier.verify_release(8, 1, Some(&release)),
            ),
            (
                "a request signed by no key",
                verifier.verify_request(5, None),
            ),
            (
                "a request with another nonce",
                verifier.verify_request(6, Some(&request)),
            ),
        ];
        for (name, outcome) in cases {
            assert!(outcome.is_err(), "{name} verified");
        }
        assert_eq!(verifier.verify_request(5, Some(&request)), Ok(()));
        assert_eq!(verifier.verify_release(7, 1, Some(&release)), Ok(()));
    }

    #[test]
    fn a_private_key_that_others_can_use_is_refused_with_the_fix() {
        use std::os::unix::fs::PermissionsExt;

        let dir = std::env::temp_dir().join(format!("sat-teleop-key-mode-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        generate(&dir, "t").unwrap();
        let key = dir.join("ctl-t.key");
        let load = || {
            Signer::load(&key, &vehicle("v1"))
                .err()
                .map(|e| e.to_string())
        };
        assert_eq!(load(), None, "a key made by `generate` is mode 600");
        fs::set_permissions(&key, fs::Permissions::from_mode(0o640)).unwrap();
        let message = load().unwrap();
        assert!(
            message.contains("chmod 600") && message.contains("ctl-t.key"),
            "{message}"
        );
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn vehicle_ids_are_short_printable_names() {
        for id in [
            "",
            "has space",
            "tab\t",
            "nul\0",
            &"x".repeat(MAX_ID_LEN + 1),
        ] {
            assert!(id.parse::<VehicleId>().is_err(), "{id:?}");
        }
        assert!("pi-4_bench.1".parse::<VehicleId>().is_ok());
    }
}
