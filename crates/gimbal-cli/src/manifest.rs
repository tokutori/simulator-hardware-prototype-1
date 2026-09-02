// SPDX-License-Identifier: MIT

use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::error::Error;
use std::fs;
use std::path::{Component, Path, PathBuf};

fn sha256_file(path: &Path) -> Result<String, std::io::Error> {
    let bytes = fs::read(path)?;
    Ok(format!("{:x}", Sha256::digest(bytes)))
}

pub(crate) fn optional_artifact_paths(output: &Path) -> Vec<PathBuf> {
    [
        "model/gimbal-prototype.blend",
        "preview/isometric.png",
        "preview/top-z.png",
        "preview/left-side-minus-y.png",
        "preview/front-plus-x.png",
        "preview/drive-unit-detail.png",
        "preview/pitch-gearbox-detail.png",
        "preview/roll-gearbox-detail.png",
        "preview/pitch-sector-reinforcement-detail.png",
        "preview/gimbal-motion.mp4",
        "preview/pitch-gearbox-motion.mp4",
        "preview/roll-gearbox-motion.mp4",
        "validation-report.json",
        "validation-report-structural.json",
        "validation-report-full.json",
    ]
    .into_iter()
    .map(|relative| output.join(relative))
    .collect()
}

pub(crate) fn artifact_manifest(
    workspace: &Path,
    artifact_paths: &[PathBuf],
) -> Result<Vec<Value>, Box<dyn Error>> {
    artifact_paths
        .iter()
        .map(|path| {
            let relative = path.strip_prefix(workspace).unwrap_or(path);
            Ok(json!({
                "path": relative.to_string_lossy().replace('\\', "/"),
                "bytes": fs::metadata(path)?.len(),
                "sha256": sha256_file(path)?
            }))
        })
        .collect()
}

pub(crate) fn staged_artifact_manifest(
    staging_output: &Path,
    artifact_paths: &[PathBuf],
) -> Result<Vec<Value>, Box<dyn Error>> {
    artifact_paths
        .iter()
        .map(|path| {
            let relative = path
                .strip_prefix(staging_output)
                .map_err(|_| "staged artifact is outside the staging output")?;
            let logical = Path::new("output").join(relative);
            Ok(json!({
                "path": logical.to_string_lossy().replace('\\', "/"),
                "bytes": fs::metadata(path)?.len(),
                "sha256": sha256_file(path)?
            }))
        })
        .collect()
}

pub(crate) fn refresh_manifest(workspace: &Path) -> Result<(), Box<dyn Error>> {
    let output = workspace.join("output");
    let manifest_path = output.join("manifest.json");
    let mut manifest: Value = serde_json::from_slice(&fs::read(&manifest_path)?)?;
    let mut paths = Vec::<PathBuf>::new();
    for artifact in manifest
        .get("artifacts")
        .and_then(Value::as_array)
        .ok_or("manifest has no artifact list")?
    {
        let relative = artifact
            .get("path")
            .and_then(Value::as_str)
            .ok_or("manifest artifact has no path")?;
        let relative_path = Path::new(relative);
        if relative_path.is_absolute()
            || relative_path.components().any(|component| {
                matches!(
                    component,
                    Component::ParentDir | Component::RootDir | Component::Prefix(_)
                )
            })
            || !relative_path.starts_with("output")
        {
            return Err(format!("unsafe artifact path in manifest: {relative:?}").into());
        }
        let path = workspace.join(relative_path);
        if path.is_file() && !paths.contains(&path) {
            paths.push(path);
        }
    }
    for path in optional_artifact_paths(&output) {
        if path.is_file() && !paths.contains(&path) {
            paths.push(path);
        }
    }
    manifest["artifacts"] = Value::Array(artifact_manifest(workspace, &paths)?);
    fs::write(&manifest_path, serde_json::to_vec_pretty(&manifest)?)?;
    println!("refreshed {} artifact hashes", paths.len());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn staged_artifacts_are_published_under_the_logical_output_prefix() {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("clock is after epoch")
            .as_nanos();
        let root = std::env::temp_dir().join(format!(
            "gimbal-manifest-staging-{}-{nonce}",
            std::process::id()
        ));
        let staging = root.join(".gimbal-output-staging");
        let artifact = staging.join("model").join("assembly.obj");
        fs::create_dir_all(artifact.parent().unwrap()).expect("staging directory can be created");
        fs::write(&artifact, b"mesh").expect("staged artifact can be written");

        let manifest = staged_artifact_manifest(&staging, &[artifact])
            .expect("staged artifact can be described");
        assert_eq!(manifest[0]["path"], "output/model/assembly.obj");
        assert_eq!(manifest[0]["bytes"], 4);

        fs::remove_dir_all(root).expect("temporary directory can be removed");
    }
}
