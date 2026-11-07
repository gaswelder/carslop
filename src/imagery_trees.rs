//! Checksums and atomic file writes for imagery preparation.
use crate::location_compiler::Result;
use std::{
    fs,
    io::{Read, Write},
    path::Path,
};
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
pub(crate) fn atomic(path: &Path, data: &[u8]) -> Result<()> {
    let mut file = tempfile::NamedTempFile::new_in(path.parent().unwrap())?;
    file.write_all(data)?;
    file.as_file().sync_all()?;
    file.persist(path)?;
    Ok(())
}
