use std::fs::{self, File};
use std::io::{self, Read};
use std::path::{Component, Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, bail, ensure};
use serde::Deserialize;
use sha2::{Digest, Sha256};

const DOWNLOAD_TIMEOUT: Duration = Duration::from_secs(300);

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Fetch,
    CheckOnly,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    fixture: Vec<Fixture>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    name: String,
    url: String,
    sha256: String,
    size_bytes: u64,
    license: String,
    source: String,
    #[allow(dead_code)] // required so every entry is documented
    notes: String,
}

pub fn run(root: &Path, mode: Mode) -> Result<()> {
    let manifest_path = root.join("fixtures/manifest.toml");
    let text = fs::read_to_string(&manifest_path)
        .with_context(|| format!("reading {}", manifest_path.display()))?;
    let manifest: Manifest = toml::from_str(&text).context("parsing fixtures manifest")?;
    let dir = root.join(".cache/fixtures");
    for fixture in &manifest.fixture {
        ensure_fixture(&dir, fixture, mode).with_context(|| format!("fixture {}", fixture.name))?;
        println!("ok {}", fixture.name);
    }
    Ok(())
}

fn ensure_fixture(dir: &Path, fixture: &Fixture, mode: Mode) -> Result<()> {
    let path = fixture_path(dir, &fixture.name)?;
    if !path.exists() && mode == Mode::CheckOnly {
        bail!("missing {}; run `cargo xtask fixtures`", path.display());
    }
    if !path.exists() {
        download(fixture, &path)?;
    }
    verify(&path, &fixture.sha256)
}

fn fixture_path(dir: &Path, name: &str) -> Result<PathBuf> {
    let mut components = Path::new(name).components();
    ensure!(
        matches!(components.next(), Some(Component::Normal(_))) && components.next().is_none(),
        "name must be a plain file name"
    );
    Ok(dir.join(name))
}

fn download(fixture: &Fixture, path: &Path) -> Result<()> {
    let dir = path.parent().context("fixture path has no parent")?;
    fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    let partial = path.with_extension("part");
    println!(
        "downloading {} (license: {}; source: {})",
        fixture.url, fixture.license, fixture.source
    );
    let result = fetch(fixture, &partial).and_then(|()| verify(&partial, &fixture.sha256));
    if let Err(err) = result {
        fs::remove_file(&partial).ok();
        return Err(err);
    }
    fs::rename(&partial, path).with_context(|| format!("moving into {}", path.display()))
}

fn fetch(fixture: &Fixture, partial: &Path) -> Result<()> {
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(DOWNLOAD_TIMEOUT))
        .build()
        .into();
    let body = agent
        .get(&fixture.url)
        .call()
        .with_context(|| format!("GET {}", fixture.url))?
        .into_body();
    let mut file =
        File::create(partial).with_context(|| format!("creating {}", partial.display()))?;
    let cap = fixture.size_bytes.saturating_add(1);
    let written =
        io::copy(&mut body.into_reader().take(cap), &mut file).context("writing download")?;
    ensure!(
        written == fixture.size_bytes,
        "download size mismatch: expected {} bytes, got {}{}",
        fixture.size_bytes,
        written,
        if written > fixture.size_bytes {
            " or more"
        } else {
            ""
        }
    );
    Ok(())
}

fn verify(path: &Path, expected: &str) -> Result<()> {
    let mut file = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    let actual = sha256_hex(&mut file).with_context(|| format!("hashing {}", path.display()))?;
    ensure!(
        actual.eq_ignore_ascii_case(expected),
        "sha256 mismatch for {}: expected {expected}, got {actual}",
        path.display()
    );
    Ok(())
}

fn sha256_hex(reader: &mut impl Read) -> io::Result<String> {
    let mut hasher = Sha256::new();
    let mut buf = vec![0; 1 << 16];
    loop {
        let n = reader.read(&mut buf)?;
        if n == 0 {
            break;
        }
        hasher.update(buf.get(..n).unwrap_or_default());
    }
    let digest = hasher.finalize();
    Ok(digest
        .iter()
        .flat_map(|byte| [byte >> 4, byte & 0xf])
        .filter_map(|nibble| char::from_digit(u32::from(nibble), 16))
        .collect())
}
