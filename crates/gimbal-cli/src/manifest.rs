// SPDX-License-Identifier: MIT

use gimbal_export::sha256_file;
use serde_json::{Value, json};
use std::error::Error;
use std::fs;
use std::path::{Component, Path, PathBuf};

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
