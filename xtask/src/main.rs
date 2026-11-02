mod fixtures;
mod layers;

use std::path::Path;
use std::process::ExitCode;

use anyhow::Result;

use fixtures::Mode;

const USAGE: &str = "usage: cargo xtask <layers | fixtures [--check]>";

fn main() -> ExitCode {
    match run() {
        Ok(code) => code,
        Err(err) => {
            eprintln!("error: {err:#}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<ExitCode> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args
        .iter()
        .map(String::as_str)
        .collect::<Vec<_>>()
        .as_slice()
    {
        ["layers"] => check_layers(&root),
        ["fixtures"] => fixtures::run(&root, Mode::Fetch).map(|()| ExitCode::SUCCESS),
        ["fixtures", "--check"] => {
            fixtures::run(&root, Mode::CheckOnly).map(|()| ExitCode::SUCCESS)
        }
        _ => {
            eprintln!("{USAGE}");
            Ok(ExitCode::from(2))
        }
    }
}

fn check_layers(root: &Path) -> Result<ExitCode> {
    let violations = layers::run(root)?;
    for line in &violations {
        eprintln!("{line}");
    }
    Ok(if violations.is_empty() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    })
}
