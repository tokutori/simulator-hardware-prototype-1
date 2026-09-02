// SPDX-License-Identifier: MIT

use std::error::Error;
use std::fs;
use std::path::Path;

pub(crate) fn clean_output(workspace: &Path) -> Result<(), Box<dyn Error>> {
    let output = workspace.join("output");
    let canonical_workspace = workspace.canonicalize()?;
    if output.exists() {
        let canonical_output = output.canonicalize()?;
        if canonical_output.parent() != Some(canonical_workspace.as_path())
            || canonical_output.file_name().and_then(|name| name.to_str()) != Some("output")
        {
            return Err("refusing to remove an unexpected output path".into());
        }
        fs::remove_dir_all(&canonical_output)?;
        println!("removed {}", canonical_output.display());
    }
    Ok(())
}
