//! Persistent, metadata-based build receipts. A failed stage never stays current.

use crate::util::error::Result;
use serde::{Deserialize, Serialize};
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::SystemTime;

#[derive(Debug, PartialEq, Serialize, Deserialize)]
struct Entry {
    path: PathBuf,
    length: u64,
    modified: SystemTime,
    directory: bool,
}

#[derive(PartialEq, Serialize, Deserialize)]
struct Receipt {
    inputs: Vec<Entry>,
    outputs: Vec<Entry>,
    settings: String,
}

fn snapshot(paths: &[PathBuf]) -> Result<Vec<Entry>> {
    let mut entries = Vec::new();
    for path in paths {
        let metadata = fs::metadata(path)?;
        entries.push(Entry {
            path: path.clone(),
            length: if metadata.is_dir() { 0 } else { metadata.len() },
            modified: if metadata.is_dir() {
                SystemTime::UNIX_EPOCH
            } else {
                metadata.modified()?
            },
            directory: metadata.is_dir(),
        });
        if metadata.is_dir() {
            let mut children = fs::read_dir(path)?
                .map(|entry| entry.map(|entry| entry.path()))
                .collect::<std::io::Result<Vec<_>>>()?;
            children.sort();
            entries.extend(snapshot(&children)?);
        }
    }
    Ok(entries)
}

pub fn fingerprint(paths: &[PathBuf], settings: &str) -> Result<String> {
    let mut dependencies = paths.to_vec();
    dependencies.push(std::env::current_exe()?);
    Ok(serde_json::to_string(&Receipt {
        inputs: snapshot(&dependencies)?,
        outputs: Vec::new(),
        settings: settings.into(),
    })?)
}

pub fn run(
    receipt: &Path,
    inputs: &[PathBuf],
    outputs: &[PathBuf],
    settings: &str,
    action: impl FnOnce() -> Result<()>,
) -> Result<()> {
    let mut dependencies = inputs.to_vec();
    dependencies.push(std::env::current_exe()?);
    let inputs = snapshot(&dependencies)?;
    let previous = fs::read(receipt)
        .ok()
        .and_then(|bytes| serde_json::from_slice::<Receipt>(&bytes).ok());
    if let Some(previous) = previous {
        if previous.inputs == inputs
            && previous.settings == settings
            && snapshot(outputs).is_ok_and(|outputs| outputs == previous.outputs)
        {
            println!("up to date: {}", receipt.display());
            return Ok(());
        }
    }
    match fs::remove_file(receipt) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => return Err(error.into()),
    }
    action()?;
    let record = Receipt {
        inputs,
        outputs: snapshot(outputs)?,
        settings: settings.into(),
    };
    let parent = receipt.parent().unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let mut file = tempfile::NamedTempFile::new_in(parent)?;
    file.write_all(&serde_json::to_vec(&record)?)?;
    file.persist(receipt)?;
    Ok(())
}

#[cfg(test)]
#[path = "tests/stage.rs"]
mod tests;
