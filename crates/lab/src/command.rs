use std::process::Command as Process;

#[derive(Debug, thiserror::Error)]
pub enum ExecError {
    #[error("empty command")]
    Empty,
    #[error("cannot start `{command}`: {source}")]
    Spawn {
        command: String,
        source: std::io::Error,
    },
    #[error("`{command}` failed ({status}): {stderr}")]
    Failed {
        command: String,
        status: String,
        stderr: String,
    },
}

/// An external command as an argv, built by pure functions and run by [`Command::run`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Command {
    pub argv: Vec<String>,
    /// A non-zero exit is not an error, for idempotent cleanup such as deleting a missing qdisc.
    pub tolerate_failure: bool,
}

impl Command {
    pub fn new(argv: &[&str]) -> Self {
        Self::from_args(argv.iter().map(|a| (*a).to_owned()).collect())
    }

    pub fn from_args(argv: Vec<String>) -> Self {
        Self {
            argv,
            tolerate_failure: false,
        }
    }

    #[must_use]
    pub fn tolerant(mut self) -> Self {
        self.tolerate_failure = true;
        self
    }

    pub fn display(&self) -> String {
        self.argv.join(" ")
    }

    /// Returns stdout.
    pub fn run(&self) -> Result<String, ExecError> {
        let (program, args) = self.argv.split_first().ok_or(ExecError::Empty)?;
        let output = Process::new(program)
            .args(args)
            .output()
            .map_err(|source| ExecError::Spawn {
                command: self.display(),
                source,
            })?;
        if !output.status.success() && !self.tolerate_failure {
            return Err(ExecError::Failed {
                command: self.display(),
                status: output.status.to_string(),
                stderr: String::from_utf8_lossy(&output.stderr).trim().to_owned(),
            });
        }
        Ok(String::from_utf8_lossy(&output.stdout).into_owned())
    }
}
