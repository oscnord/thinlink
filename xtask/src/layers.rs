use std::collections::BTreeSet;
use std::fmt;
use std::fs;
use std::path::Path;

use anyhow::{Context, Result};
use cargo_metadata::{DependencyKind, MetadataCommand};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Normal,
    Build,
    Dev,
}

impl fmt::Display for Kind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Normal => "normal",
            Self::Build => "build",
            Self::Dev => "dev",
        })
    }
}

pub struct Dependency {
    pub name: String,
    pub kind: Kind,
}

pub struct Package {
    pub name: String,
    pub lints_workspace: bool,
    pub deps: Vec<Dependency>,
}

pub struct Graph {
    pub packages: Vec<Package>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Layer {
    VideoCore,
    Codec,
    Video,
    Policy,
    Media,
    Sibling,
    Lab,
    Testkit,
    App,
    Tool,
}

impl Layer {
    /// Testkit and tool packages sit outside the layer ordering.
    fn rank(self) -> Option<u8> {
        match self {
            Self::VideoCore | Self::Codec => Some(0),
            Self::Policy | Self::Video => Some(1),
            Self::Media | Self::Sibling => Some(2),
            Self::Lab => Some(3),
            Self::App => Some(4),
            Self::Testkit | Self::Tool => None,
        }
    }
}

/// Every planned package, including those later tasks add.
fn registry(name: &str) -> Option<Layer> {
    match name {
        "sat-teleop-video-core" => Some(Layer::VideoCore),
        "sat-teleop-x264" => Some(Layer::Codec),
        "sat-teleop-video" => Some(Layer::Video),
        "sat-teleop-policy" => Some(Layer::Policy),
        "sat-teleop-media" => Some(Layer::Media),
        "sat-teleop-control" | "sat-teleop-detail" | "sat-teleop-detect" => Some(Layer::Sibling),
        "sat-teleop-lab" => Some(Layer::Lab),
        "sat-teleop-testkit" => Some(Layer::Testkit),
        "sat-teleop-vehicle" | "sat-teleop-relay" | "sat-teleop-ground" | "sat-teleop-lab-cli" => {
            Some(Layer::App)
        }
        "sat-teleop-xtask" => Some(Layer::Tool),
        _ => None,
    }
}

const POLICY_FORBIDDEN: [&str; 4] = ["tiny_http", "ureq", "rusqlite", "tokio"];

pub fn check(graph: &Graph) -> Vec<String> {
    let members: BTreeSet<&str> = graph.packages.iter().map(|p| p.name.as_str()).collect();
    let mut violations = Vec::new();
    for package in &graph.packages {
        if !package.lints_workspace {
            violations.push(format!(
                "{}: missing [lints] workspace = true",
                package.name
            ));
        }
        let Some(layer) = registry(&package.name) else {
            violations.push(format!("{}: not in the layer registry", package.name));
            continue;
        };
        for dep in &package.deps {
            let reason = if members.contains(dep.name.as_str()) {
                workspace_edge_violation(layer, dep)
            } else {
                external_violation(layer, &dep.name)
            };
            if let Some(reason) = reason {
                violations.push(format!(
                    "{}: {} dependency on {} ({reason})",
                    package.name, dep.kind, dep.name
                ));
            }
        }
    }
    violations
}

fn workspace_edge_violation(from: Layer, dep: &Dependency) -> Option<&'static str> {
    let to = registry(&dep.name)?;
    match (from, to) {
        (_, Layer::App) => Some("apps cannot be depended on"),
        (_, Layer::Tool) => Some("tools cannot be depended on"),
        (Layer::Testkit | Layer::Tool, _) => Some("has no workspace dependencies"),
        (_, Layer::Testkit) if dep.kind != Kind::Dev => Some("sat-teleop-testkit is dev-only"),
        (_, Layer::Testkit) => None,
        _ if from.rank() <= to.rank() => Some("same or higher layer"),
        _ => None,
    }
}

fn external_violation(from: Layer, dep: &str) -> Option<&'static str> {
    if dep.starts_with("gstreamer")
        && !matches!(
            from,
            Layer::Codec | Layer::Video | Layer::Media | Layer::App
        )
    {
        return Some(
            "only sat-teleop-x264, sat-teleop-video, sat-teleop-media and apps may use GStreamer",
        );
    }
    let forbidden = dep.starts_with("mavlink") || POLICY_FORBIDDEN.contains(&dep);
    (from == Layer::Policy && forbidden).then_some("sat-teleop-policy must stay pure")
}

/// Returns the violations found in the workspace rooted at `root`.
pub fn run(root: &Path) -> Result<Vec<String>> {
    let graph = load_graph(root)?;
    Ok(check(&graph))
}

fn load_graph(root: &Path) -> Result<Graph> {
    let metadata = MetadataCommand::new()
        .manifest_path(root.join("Cargo.toml"))
        .exec()
        .context("running cargo metadata")?;
    let packages = metadata
        .workspace_packages()
        .into_iter()
        .map(|package| {
            let manifest = package.manifest_path.as_std_path();
            Ok(Package {
                name: package.name.to_string(),
                lints_workspace: opts_into_workspace_lints(manifest)?,
                deps: package
                    .dependencies
                    .iter()
                    .filter_map(|dep| {
                        let kind = match dep.kind {
                            DependencyKind::Normal => Kind::Normal,
                            DependencyKind::Build => Kind::Build,
                            DependencyKind::Development => Kind::Dev,
                            _ => return None,
                        };
                        Some(Dependency {
                            name: dep.name.clone(),
                            kind,
                        })
                    })
                    .collect(),
            })
        })
        .collect::<Result<_>>()?;
    Ok(Graph { packages })
}

fn opts_into_workspace_lints(manifest: &Path) -> Result<bool> {
    let text =
        fs::read_to_string(manifest).with_context(|| format!("reading {}", manifest.display()))?;
    let table: toml::Table = text
        .parse()
        .with_context(|| format!("parsing {}", manifest.display()))?;
    Ok(table
        .get("lints")
        .and_then(|lints| lints.get("workspace"))
        .and_then(toml::Value::as_bool)
        == Some(true))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pkg(name: &str, deps: &[(&str, Kind)]) -> Package {
        Package {
            name: name.into(),
            lints_workspace: true,
            deps: deps
                .iter()
                .map(|&(name, kind)| Dependency {
                    name: name.into(),
                    kind,
                })
                .collect(),
        }
    }

    fn graph(packages: Vec<Package>) -> Graph {
        Graph { packages }
    }

    #[test]
    fn clean_graph_passes() {
        let g = graph(vec![
            pkg("sat-teleop-policy", &[("serde", Kind::Normal)]),
            pkg(
                "sat-teleop-media",
                &[
                    ("sat-teleop-policy", Kind::Normal),
                    ("gstreamer", Kind::Normal),
                ],
            ),
            pkg(
                "sat-teleop-lab",
                &[
                    ("sat-teleop-policy", Kind::Normal),
                    ("sat-teleop-media", Kind::Normal),
                ],
            ),
            pkg("sat-teleop-testkit", &[]),
            pkg("sat-teleop-x264", &[("gstreamer-video", Kind::Normal)]),
            pkg("sat-teleop-video", &[("sat-teleop-x264", Kind::Normal)]),
            pkg(
                "sat-teleop-vehicle",
                &[
                    ("sat-teleop-lab", Kind::Normal),
                    ("gstreamer-app", Kind::Normal),
                    ("sat-teleop-testkit", Kind::Dev),
                ],
            ),
            pkg("sat-teleop-xtask", &[("anyhow", Kind::Normal)]),
        ]);
        assert_eq!(check(&g), Vec::<String>::new());
    }

    #[test]
    fn violations_are_reported() {
        let cases: Vec<(&str, Graph, &str)> = vec![
            (
                "sideways L1 edge",
                graph(vec![
                    pkg("sat-teleop-media", &[("sat-teleop-control", Kind::Normal)]),
                    pkg("sat-teleop-control", &[]),
                ]),
                "sat-teleop-media: normal dependency on sat-teleop-control (same or higher layer)",
            ),
            (
                "video knows nothing of the drone",
                graph(vec![
                    pkg("sat-teleop-video", &[("sat-teleop-policy", Kind::Normal)]),
                    pkg("sat-teleop-policy", &[]),
                ]),
                "sat-teleop-video: normal dependency on sat-teleop-policy (same or higher layer)",
            ),
            (
                "upward edge",
                graph(vec![
                    pkg("sat-teleop-policy", &[("sat-teleop-lab", Kind::Build)]),
                    pkg("sat-teleop-lab", &[]),
                ]),
                "sat-teleop-policy: build dependency on sat-teleop-lab (same or higher layer)",
            ),
            (
                "upward dev edge",
                graph(vec![
                    pkg("sat-teleop-media", &[("sat-teleop-lab", Kind::Dev)]),
                    pkg("sat-teleop-lab", &[]),
                ]),
                "sat-teleop-media: dev dependency on sat-teleop-lab (same or higher layer)",
            ),
            (
                "app dependency",
                graph(vec![
                    pkg("sat-teleop-ground", &[("sat-teleop-vehicle", Kind::Normal)]),
                    pkg("sat-teleop-vehicle", &[]),
                ]),
                "sat-teleop-ground: normal dependency on sat-teleop-vehicle (apps cannot be depended on)",
            ),
            (
                "testkit as normal dependency",
                graph(vec![
                    pkg("sat-teleop-lab", &[("sat-teleop-testkit", Kind::Normal)]),
                    pkg("sat-teleop-testkit", &[]),
                ]),
                "sat-teleop-lab: normal dependency on sat-teleop-testkit (sat-teleop-testkit is dev-only)",
            ),
            (
                "testkit with workspace dependency",
                graph(vec![
                    pkg("sat-teleop-testkit", &[("sat-teleop-policy", Kind::Normal)]),
                    pkg("sat-teleop-policy", &[]),
                ]),
                "sat-teleop-testkit: normal dependency on sat-teleop-policy (has no workspace dependencies)",
            ),
            (
                "forbidden dependency in policy",
                graph(vec![pkg("sat-teleop-policy", &[("tokio", Kind::Normal)])]),
                "sat-teleop-policy: normal dependency on tokio (sat-teleop-policy must stay pure)",
            ),
            (
                "gstreamer outside media and apps",
                graph(vec![pkg(
                    "sat-teleop-lab",
                    &[("gstreamer-video", Kind::Normal)],
                )]),
                "sat-teleop-lab: normal dependency on gstreamer-video (only sat-teleop-x264, sat-teleop-video, sat-teleop-media and apps may use GStreamer)",
            ),
            (
                "unregistered member",
                graph(vec![pkg("sat-teleop-mystery", &[])]),
                "sat-teleop-mystery: not in the layer registry",
            ),
            (
                "missing lint opt-in",
                graph(vec![Package {
                    lints_workspace: false,
                    ..pkg("sat-teleop-policy", &[])
                }]),
                "sat-teleop-policy: missing [lints] workspace = true",
            ),
        ];
        for (name, g, expected) in cases {
            assert_eq!(check(&g), vec![expected.to_string()], "{name}");
        }
    }
}
