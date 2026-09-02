// SPDX-License-Identifier: MIT

use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

const OUTPUT_NAME: &str = "output";
const STAGING_NAME: &str = ".gimbal-output-staging";
const BACKUP_NAME: &str = ".gimbal-output-previous";

pub(crate) fn clean_output(workspace: &Path) -> Result<(), Box<dyn Error>> {
    let output = checked_child(workspace, OUTPUT_NAME)?;
    if output.is_dir() {
        fs::remove_dir_all(&output)?;
        println!("removed {}", output.display());
    }
    Ok(())
}

pub(crate) fn prepare_staging_output(workspace: &Path) -> Result<PathBuf, Box<dyn Error>> {
    let staging = checked_child(workspace, STAGING_NAME)?;
    if staging.exists() {
        fs::remove_dir_all(&staging)?;
    }
    fs::create_dir(&staging)?;
    Ok(staging)
}

pub(crate) fn publish_staging_output(workspace: &Path) -> Result<PathBuf, Box<dyn Error>> {
    let output = checked_child(workspace, OUTPUT_NAME)?;
    let staging = checked_child(workspace, STAGING_NAME)?;
    let backup = checked_child(workspace, BACKUP_NAME)?;
    if !staging.is_dir() {
        return Err("staging output does not exist".into());
    }
    if backup.exists() {
        fs::remove_dir_all(&backup)?;
    }
    if output.exists() {
        fs::rename(&output, &backup)?;
    }
    if let Err(error) = fs::rename(&staging, &output) {
        if backup.exists() {
            fs::rename(&backup, &output)?;
        }
        return Err(error.into());
    }
    if backup.exists() {
        fs::remove_dir_all(&backup)?;
    }
    Ok(output)
}

fn checked_child(workspace: &Path, name: &str) -> Result<PathBuf, Box<dyn Error>> {
    let canonical_workspace = workspace.canonicalize()?;
    let child = canonical_workspace.join(name);
    if child.parent() != Some(canonical_workspace.as_path())
        || child.file_name().and_then(|value| value.to_str()) != Some(name)
    {
        return Err("refusing to operate on an unexpected output path".into());
    }
    Ok(child)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temporary_workspace() -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("clock is after epoch")
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "gimbal-output-transaction-{}-{nonce}",
            std::process::id()
        ));
        fs::create_dir(&path).expect("temporary workspace can be created");
        path
    }

    #[test]
    fn publishes_a_complete_staging_tree_without_retaining_old_artifacts() {
        let workspace = temporary_workspace();
        let old_output = workspace.join(OUTPUT_NAME);
        fs::create_dir(&old_output).expect("old output can be created");
        fs::write(old_output.join("obsolete.txt"), b"old").expect("old artifact can be written");

        let staging = prepare_staging_output(&workspace).expect("staging can be prepared");
        fs::write(staging.join("current.txt"), b"new").expect("new artifact can be written");
        assert_eq!(
            fs::read(old_output.join("obsolete.txt")).unwrap(),
            b"old",
            "the previous output remains available until publication"
        );
        let output = publish_staging_output(&workspace).expect("staging can be published");

        assert_eq!(fs::read(output.join("current.txt")).unwrap(), b"new");
        assert!(!output.join("obsolete.txt").exists());
        assert!(!workspace.join(STAGING_NAME).exists());
        assert!(!workspace.join(BACKUP_NAME).exists());
        fs::remove_dir_all(workspace).expect("temporary workspace can be removed");
    }
}
