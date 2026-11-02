//! The bearer token of the management endpoints (the ground's lease endpoints, the relay's job API,
//! the applications' metrics). With a token configured every request must present it; without one
//! an endpoint listens on loopback only (`listen_address`), so an endpoint on a network address is
//! never open by omission.

use std::net::{Ipv4Addr, SocketAddr};

use subtle::ConstantTimeEq;

/// Shorter tokens are refused: they are guessable.
const MIN_LEN: usize = 16;

#[derive(Debug, thiserror::Error)]
#[error("a token must be at least {MIN_LEN} characters")]
pub struct TooShort;

#[derive(Clone, PartialEq, Eq)]
pub struct Token(Vec<u8>);

impl std::fmt::Debug for Token {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Token(..)")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    Allowed,
    /// A token is required and the request had none or another.
    Unauthorized,
}

impl Access {
    /// The HTTP status to refuse with, `None` when allowed.
    pub fn refusal_status(self) -> Option<u16> {
        match self {
            Self::Allowed => None,
            Self::Unauthorized => Some(401),
        }
    }
}

/// Whether a secret file's `mode` gives its group or others any access. A private key or a token
/// is refused then, whoever else can read it can use it.
pub fn exposed(mode: u32) -> bool {
    mode & 0o077 != 0
}

/// Where an endpoint listens: `configured` when a token guards it, otherwise the same port on
/// loopback, where only this machine can reach it.
pub fn listen_address(configured: SocketAddr, token: Option<&Token>) -> SocketAddr {
    match token {
        Some(_) => configured,
        None => SocketAddr::from((Ipv4Addr::LOCALHOST, configured.port())),
    }
}

impl Token {
    /// The token in `text`, whitespace around it ignored.
    pub fn new(text: &str) -> Result<Self, TooShort> {
        let text = text.trim();
        if text.len() < MIN_LEN {
            return Err(TooShort);
        }
        Ok(Self(text.as_bytes().to_vec()))
    }

    /// The `Authorization` header value a client sends.
    pub fn header_value(&self) -> String {
        format!("Bearer {}", String::from_utf8_lossy(&self.0))
    }

    fn matches(&self, authorization: &str) -> bool {
        let Some(presented) = authorization.strip_prefix("Bearer ") else {
            return false;
        };
        self.0.ct_eq(presented.as_bytes()).into()
    }
}

/// Whether a request may be served: it presents `token` (compared in constant time). Without a
/// token the endpoint listens on loopback only (`listen_address`), so every request is this
/// machine's.
pub fn admit(token: Option<&Token>, authorization: Option<&str>) -> Access {
    match token {
        Some(token) if authorization.is_some_and(|a| token.matches(a)) => Access::Allowed,
        Some(_) => Access::Unauthorized,
        None => Access::Allowed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_request_needs_the_token_when_there_is_one() {
        let token = Token::new("0123456789abcdef0123\n").unwrap();
        let good = token.header_value();
        let cases = [
            (Some(&token), Some(good.as_str()), Access::Allowed),
            (Some(&token), None, Access::Unauthorized),
            (
                Some(&token),
                Some("Bearer 0123456789abcdef012"),
                Access::Unauthorized,
            ),
            (
                Some(&token),
                Some("Bearer 0123456789abcdef0123x"),
                Access::Unauthorized,
            ),
            (
                Some(&token),
                Some("0123456789abcdef0123"),
                Access::Unauthorized,
            ),
            (None, None, Access::Allowed),
        ];
        for (token, header, expected) in cases {
            assert_eq!(admit(token, header), expected, "{header:?}");
        }
        assert!(Token::new("short").is_err());
    }

    #[test]
    fn a_secret_file_is_for_its_owner_alone() {
        for (mode, exposed_to_others) in [
            (0o600, false),
            (0o400, false),
            (0o700, false),
            (0o640, true),
            (0o604, true),
            (0o644, true),
            (0o620, true),
            (0o666, true),
        ] {
            assert_eq!(exposed(mode), exposed_to_others, "{mode:o}");
        }
    }

    #[test]
    fn without_a_token_an_endpoint_listens_on_loopback_only() {
        let token = Token::new("0123456789abcdef0123").unwrap();
        let lan: SocketAddr = "192.168.233.10:9100".parse().unwrap();
        let cases = [
            (Some(&token), lan, lan),
            (None, lan, "127.0.0.1:9100".parse().unwrap()),
            (
                None,
                "0.0.0.0:9100".parse().unwrap(),
                "127.0.0.1:9100".parse().unwrap(),
            ),
        ];
        for (token, configured, expected) in cases {
            assert_eq!(listen_address(configured, token), expected, "{configured}");
        }
    }
}
