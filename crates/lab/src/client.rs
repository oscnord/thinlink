use std::collections::BTreeMap;
use std::path::Path;
use std::sync::OnceLock;
use std::time::Duration;

use sat_teleop_video_core::token::Token;

use ureq::Body;
use ureq::http::Response;

const TIMEOUT: Duration = Duration::from_secs(30);

/// The bearer tokens of the applications' endpoints and the relay's job API by base URL, set once by
/// the CLI: each service has a token of its own, and a client sends only its endpoint's.
static TOKENS: OnceLock<BTreeMap<String, Token>> = OnceLock::new();

/// The token in `path`: one line, in a file only its owner can read.
pub fn load_token(path: &Path) -> Result<Token, sat_teleop_video::MediaError> {
    sat_teleop_video::metrics::load_token(path)
}

/// Sets the tokens the clients send, by base URL; the first call wins.
pub fn use_tokens(tokens: impl IntoIterator<Item = (String, Token)>) {
    let tokens = tokens
        .into_iter()
        .map(|(url, token)| (url.trim_end_matches('/').to_owned(), token))
        .collect();
    let _ = TOKENS.set(tokens);
}

/// The token of the endpoint at `base_url`, if it has one.
pub fn token_for(base_url: &str) -> Option<&'static Token> {
    TOKENS.get()?.get(base_url.trim_end_matches('/'))
}

#[derive(Debug, thiserror::Error)]
pub enum ClientError {
    #[error("{method} {url} failed: {source}")]
    Request {
        method: &'static str,
        url: String,
        source: ureq::Error,
    },
    #[error("{method} {url} returned {status}: {body}")]
    Status {
        method: &'static str,
        url: String,
        status: u16,
        body: String,
    },
}

/// HTTP client for the router agent and the applications' `ctl_net` endpoints. Non-2xx responses
/// become [`ClientError::Status`].
pub struct AgentClient {
    base: String,
    http: ureq::Agent,
    authorization: String,
}

impl AgentClient {
    /// `base_url` such as `http://192.168.233.254:9000`.
    pub fn new(base_url: &str) -> Self {
        let http = ureq::Agent::config_builder()
            .http_status_as_error(false)
            .timeout_global(Some(TIMEOUT))
            .build()
            .into();
        Self {
            base: base_url.trim_end_matches('/').to_owned(),
            http,
            authorization: token_for(base_url)
                .map(Token::header_value)
                .unwrap_or_default(),
        }
    }

    pub fn get(&self, path: &str) -> Result<String, ClientError> {
        let url = format!("{}{path}", self.base);
        let response = self
            .http
            .get(&url)
            .header("Authorization", &self.authorization)
            .call();
        body("GET", url, response)
    }

    pub fn post(&self, path: &str, body_text: &str) -> Result<String, ClientError> {
        let url = format!("{}{path}", self.base);
        let response = self
            .http
            .post(&url)
            .header("Authorization", &self.authorization)
            .send(body_text);
        body("POST", url, response)
    }

    pub fn delete(&self, path: &str) -> Result<String, ClientError> {
        let url = format!("{}{path}", self.base);
        let response = self
            .http
            .delete(&url)
            .header("Authorization", &self.authorization)
            .call();
        body("DELETE", url, response)
    }
}

fn body(
    method: &'static str,
    url: String,
    response: Result<Response<Body>, ureq::Error>,
) -> Result<String, ClientError> {
    let read = response.and_then(|mut r| Ok((r.status(), r.body_mut().read_to_string()?)));
    let (status, text) = match read {
        Ok(ok) => ok,
        Err(source) => {
            return Err(ClientError::Request {
                method,
                url,
                source,
            });
        }
    };
    if !status.is_success() {
        return Err(ClientError::Status {
            method,
            url,
            status: status.as_u16(),
            body: text,
        });
    }
    Ok(text)
}
