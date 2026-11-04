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
        "thinlink-video-core" => Some(Layer::VideoCore),
        "thinlink-x264" => Some(Layer::Codec),
        "thinlink-video" => Some(Layer::Video),
        "thinlink-policy" => Some(Layer::Policy),
        "thinlink-media" => Some(Layer::Media),
        "thinlink-control" | "thinlink-detail" | "thinlink-detect" => Some(Layer::Sibling),
        "thinlink-lab" => Some(Layer::Lab),
        "thinlink-testkit" => Some(Layer::Testkit),
        "thinlink-vehicle" | "thinlink-relay" | "thinlink-ground" | "thinlink-lab-cli" => {
            Some(Layer::App)
        }
        "thinlink-xtask" => Some(Layer::Tool),
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
        (_, Layer::Testkit) if dep.kind != Kind::Dev => Some("thinlink-testkit is dev-only"),
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
            "only thinlink-x264, thinlink-video, thinlink-media and apps may use GStreamer",
        );
    }
    let forbidden = dep.starts_with("mavlink") || POLICY_FORBIDDEN.contains(&dep);
    (from == Layer::Policy && forbidden).then_some("thinlink-policy must stay pure")
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
            pkg("thinlink-policy", &[("serde", Kind::Normal)]),
            pkg(
                "thinlink-media",
                &[
                    ("thinlink-policy", Kind::Normal),
                    ("gstreamer", Kind::Normal),
                ],
            ),
            pkg(
                "thinlink-lab",
                &[
                    ("thinlink-policy", Kind::Normal),
                    ("thinlink-media", Kind::Normal),
                ],
            ),
            pkg("thinlink-testkit", &[]),
            pkg("thinlink-x264", &[("gstreamer-video", Kind::Normal)]),
            pkg("thinlink-video", &[("thinlink-x264", Kind::Normal)]),
            pkg(
                "thinlink-vehicle",
                &[
                    ("thinlink-lab", Kind::Normal),
                    ("gstreamer-app", Kind::Normal),
                    ("thinlink-testkit", Kind::Dev),
                ],
            ),
            pkg("thinlink-xtask", &[("anyhow", Kind::Normal)]),
        ]);
        assert_eq!(check(&g), Vec::<String>::new());
    }

    #[test]
    fn violations_are_reported() {
        let cases: Vec<(&str, Graph, &str)> = vec![
            (
                "sideways L1 edge",
                graph(vec![
                    pkg("thinlink-media", &[("thinlink-control", Kind::Normal)]),
                    pkg("thinlink-control", &[]),
                ]),
                "thinlink-media: normal dependency on thinlink-control (same or higher layer)",
            ),
            (
                "video knows nothing of the drone",
                graph(vec![
                    pkg("thinlink-video", &[("thinlink-policy", Kind::Normal)]),
                    pkg("thinlink-policy", &[]),
                ]),
                "thinlink-video: normal dependency on thinlink-policy (same or higher layer)",
            ),
            (
                "upward edge",
                graph(vec![
                    pkg("thinlink-policy", &[("thinlink-lab", Kind::Build)]),
                    pkg("thinlink-lab", &[]),
                ]),
                "thinlink-policy: build dependency on thinlink-lab (same or higher layer)",
            ),
            (
                "upward dev edge",
                graph(vec![
                    pkg("thinlink-media", &[("thinlink-lab", Kind::Dev)]),
                    pkg("thinlink-lab", &[]),
                ]),
                "thinlink-media: dev dependency on thinlink-lab (same or higher layer)",
            ),
            (
                "app dependency",
                graph(vec![
                    pkg("thinlink-ground", &[("thinlink-vehicle", Kind::Normal)]),
                    pkg("thinlink-vehicle", &[]),
                ]),
                "thinlink-ground: normal dependency on thinlink-vehicle (apps cannot be depended on)",
            ),
            (
                "testkit as normal dependency",
                graph(vec![
                    pkg("thinlink-lab", &[("thinlink-testkit", Kind::Normal)]),
                    pkg("thinlink-testkit", &[]),
                ]),
                "thinlink-lab: normal dependency on thinlink-testkit (thinlink-testkit is dev-only)",
            ),
            (
                "testkit with workspace dependency",
                graph(vec![
                    pkg("thinlink-testkit", &[("thinlink-policy", Kind::Normal)]),
                    pkg("thinlink-policy", &[]),
                ]),
                "thinlink-testkit: normal dependency on thinlink-policy (has no workspace dependencies)",
            ),
            (
                "forbidden dependency in policy",
                graph(vec![pkg("thinlink-policy", &[("tokio", Kind::Normal)])]),
                "thinlink-policy: normal dependency on tokio (thinlink-policy must stay pure)",
            ),
            (
                "gstreamer outside media and apps",
                graph(vec![pkg(
                    "thinlink-lab",
                    &[("gstreamer-video", Kind::Normal)],
                )]),
                "thinlink-lab: normal dependency on gstreamer-video (only thinlink-x264, thinlink-video, thinlink-media and apps may use GStreamer)",
            ),
            (
                "unregistered member",
                graph(vec![pkg("thinlink-mystery", &[])]),
                "thinlink-mystery: not in the layer registry",
            ),
            (
                "missing lint opt-in",
                graph(vec![Package {
                    lints_workspace: false,
                    ..pkg("thinlink-policy", &[])
                }]),
                "thinlink-policy: missing [lints] workspace = true",
            ),
        ];
        for (name, g, expected) in cases {
            assert_eq!(check(&g), vec![expected.to_string()], "{name}");
        }
    }
}
