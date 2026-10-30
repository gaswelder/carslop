//! Versioned, little-endian triangle mesh used by file-backed car artwork.
use super::Vertex;
use std::{io, path::Path};

pub(super) fn load(path: &Path) -> io::Result<Vec<Vertex>> {
    decode(&std::fs::read(path)?)
}

fn decode(bytes: &[u8]) -> io::Result<Vec<Vertex>> {
    let invalid = || io::Error::new(io::ErrorKind::InvalidData, "invalid car mesh");
    if bytes.len() < 12 || &bytes[..8] != b"NFSCAR01" {
        return Err(invalid());
    }
    let count = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
    if count == 0
        || count % 3 != 0
        || count.checked_mul(44).and_then(|n| n.checked_add(12)) != Some(bytes.len())
    {
        return Err(invalid());
    }
    let mut vertices = Vec::with_capacity(count);
    for record in bytes[12..].chunks_exact(44) {
        let values: [f32; 11] = std::array::from_fn(|i| {
            f32::from_le_bytes(record[i * 4..i * 4 + 4].try_into().unwrap())
        });
        if values.iter().any(|v| !v.is_finite()) {
            return Err(invalid());
        }
        vertices.push(Vertex {
            position: values[0..3].try_into().unwrap(),
            color: values[3..6].try_into().unwrap(),
            normal: values[6..9].try_into().unwrap(),
            material: values[9..11].try_into().unwrap(),
        });
    }
    Ok(vertices)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_corrupt_meshes() {
        let valid = include_bytes!("../data/cars/kestrel/body.mesh");
        assert!(decode(valid).is_ok());
        assert!(decode(&valid[..valid.len() - 1]).is_err());
        let mut bad = valid.to_vec();
        bad[12..16].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(decode(&bad).is_err());
        bad[0] = 0;
        assert!(decode(&bad).is_err());
    }
}
