//! Checksums and atomic file writes for imagery preparation.
use crate::location::Result;
use std::{fs, io::Read, path::Path};

pub(crate) fn checksum(path: &Path) -> Result<u32> {
    let mut file = fs::File::open(path)?;
    let mut hash = crc32fast::Hasher::new();
    let mut buf = vec![0; 1024 * 1024];
    loop {
        let n = file.read(&mut buf)?;
        if n == 0 {
            break;
        }
        hash.update(&buf[..n]);
    }
    Ok(hash.finalize())
}
