//! Required, lossless runtime data for street surfaces and their collision grids.
//! The text remains the source of truth. Its package hash invalidates stale caches.
//! Bump MAGIC when changing grid cell sizes, padding or collision index semantics.
use super::{Curb, RoadPaths, SourceChecksums, StreetSurface, SurfaceTriangle};
use sha2::{Digest, Sha256};
use std::{
    collections::HashMap,
    fs::{File, OpenOptions},
    io::{self, BufReader, BufWriter, Read, Write},
    path::Path,
};

const MAGIC: &[u8; 8] = b"NFSCOL01";
type Grid = HashMap<(i32, i32), Vec<usize>>;
type Loaded = (StreetSurface, RoadPaths, SourceChecksums);
fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}
struct Checked<R> {
    inner: R,
    hash: crc32fast::Hasher,
}
impl<R: Read> Read for Checked<R> {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        let n = self.inner.read(out)?;
        self.hash.update(&out[..n]);
        Ok(n)
    }
}
struct Decoder<R> {
    input: R,
    remaining: u64,
}
impl<R: Read> Decoder<R> {
    fn bytes<const N: usize>(&mut self) -> io::Result<[u8; N]> {
        if self.remaining < N as u64 {
            return Err(invalid("truncated collision cache"));
        }
        let mut out = [0; N];
        self.input.read_exact(&mut out)?;
        self.remaining -= N as u64;
        Ok(out)
    }
    fn count(&mut self, stride: u64) -> io::Result<usize> {
        let n = u32::from_le_bytes(self.bytes()?) as usize;
        if n as u64 * stride > self.remaining {
            return Err(invalid("invalid collision cache count"));
        }
        Ok(n)
    }
    fn grid(&mut self, limit: usize) -> io::Result<Grid> {
        let n = self.count(12)?;
        let mut grid = HashMap::new();
        grid.try_reserve(n)
            .map_err(|_| invalid("collision grid too large"))?;
        for _ in 0..n {
            let x = i32::from_le_bytes(self.bytes()?);
            let z = i32::from_le_bytes(self.bytes()?);
            let count = self.count(4)?;
            let mut ids = reserve(count)?;
            for _ in 0..count {
                let id = u32::from_le_bytes(self.bytes()?) as usize;
                if id >= limit || ids.last().is_some_and(|last| *last >= id) {
                    return Err(invalid("invalid collision grid reference"));
                }
                ids.push(id);
            }
            if count == 0 || grid.insert((x, z), ids).is_some() {
                return Err(invalid("invalid collision grid cell"));
            }
        }
        if grid.is_empty() {
            return Err(invalid("empty collision grid"));
        }
        Ok(grid)
    }
}
fn reserve<T>(n: usize) -> io::Result<Vec<T>> {
    let mut v = Vec::new();
    v.try_reserve_exact(n)
        .map_err(|_| invalid("collision cache too large"))?;
    Ok(v)
}
fn floats<const N: usize>(bytes: &[u8]) -> io::Result<[f32; N]> {
    let values =
        std::array::from_fn(|i| f32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap()));
    if values.iter().any(|x| !x.is_finite()) {
        return Err(invalid("nonfinite collision coordinate"));
    }
    Ok(values)
}

pub(super) fn load(directory: &Path, source: &str, roads: usize) -> io::Result<Loaded> {
    let file = crate::package::Reader::open(&directory.join("streets.bin"))?;
    let body_len = file
        .len()?
        .checked_sub(4)
        .ok_or_else(|| invalid("truncated collision cache"))?;
    let checked = Checked {
        inner: file.take(body_len),
        hash: crc32fast::Hasher::new(),
    };
    let mut r = Decoder {
        input: BufReader::with_capacity(256 * 1024, checked),
        remaining: body_len,
    };
    if &r.bytes::<8>()? != MAGIC {
        return Err(invalid("unsupported collision cache version"));
    }
    let checksums = SourceChecksums {
        city: u32::from_le_bytes(r.bytes()?),
        streets: u32::from_le_bytes(r.bytes()?),
    };
    let fingerprint = r.bytes::<64>()?;
    if checksums.city != super::crc32(source)
        || fingerprint.as_slice()
            != crate::package::content_hash(&directory.join("streets.txt"))?.as_bytes()
    {
        return Err(invalid("stale collision cache"));
    }
    if r.count(4)? != roads {
        return Err(invalid("collision road count mismatch"));
    }
    let junction = u32::from_le_bytes(r.bytes()?);
    #[cfg(not(test))]
    let _ = junction;
    let mut paths = reserve(roads)?;
    for _ in 0..roads {
        let n = r.count(8)?;
        if n < 2 {
            return Err(invalid("invalid collision road path"));
        }
        let mut path = reserve(n)?;
        for _ in 0..n {
            let p = floats::<2>(&r.bytes::<8>()?)?;
            path.push([p[0], 0.0, p[1]]);
        }
        paths.push(path);
    }
    let n = r.count(28)?;
    let mut triangles = reserve(n)?;
    for _ in 0..n {
        let bytes = r.bytes::<28>()?;
        let style = u32::from_le_bytes(bytes[..4].try_into().unwrap()) as usize;
        let p = floats::<6>(&bytes[4..])?;
        let points = [[p[0], p[1]], [p[2], p[3]], [p[4], p[5]]];
        if style > 7 || super::cross(points[0], points[1], points[2]) <= 0.00001 {
            return Err(invalid("invalid cached surface triangle"));
        }
        triangles.push(SurfaceTriangle { style, points });
    }
    let n = r.count(28)?;
    let mut curbs = reserve(n)?;
    for _ in 0..n {
        let bytes = r.bytes::<28>()?;
        let style = u32::from_le_bytes(bytes[..4].try_into().unwrap()) as usize;
        let p = floats::<6>(&bytes[4..])?;
        if style > 2 {
            return Err(invalid("invalid cached curb"));
        }
        curbs.push(Curb {
            style,
            ends: [[p[0], p[1]], [p[2], p[3]]],
            inward: [p[4], p[5]],
        });
    }
    if triangles.is_empty() || curbs.is_empty() {
        return Err(invalid("incomplete collision cache"));
    }
    let triangles_grid = r.grid(triangles.len())?;
    let curbs_grid = r.grid(curbs.len())?;
    if r.remaining != 0 {
        return Err(invalid("trailing collision cache bytes"));
    }
    let checked = r.input.into_inner();
    let actual = checked.hash.finalize();
    let mut file = checked.inner.into_inner();
    let mut footer = [0; 4];
    file.read_exact(&mut footer)?;
    if actual != u32::from_le_bytes(footer) {
        return Err(invalid("damaged collision cache checksum"));
    }
    Ok((
        StreetSurface {
            pavement_height: super::DEFAULT_PAVEMENT_HEIGHT,
            triangles,
            curbs,
            triangles_grid,
            curbs_grid,
            #[cfg(test)]
            preview_junction: (junction != u32::MAX).then_some(junction as usize),
        },
        paths,
        checksums,
    ))
}

fn number(out: &mut impl Write, n: usize) -> io::Result<()> {
    out.write_all(
        &u32::try_from(n)
            .map_err(|_| invalid("collision count exceeds u32"))?
            .to_le_bytes(),
    )
}
fn grid(out: &mut impl Write, cells: &Grid) -> io::Result<()> {
    number(out, cells.len())?;
    let mut entries: Vec<_> = cells.iter().collect();
    entries.sort_unstable_by_key(|(key, _)| **key);
    for (&(x, z), ids) in entries {
        out.write_all(&x.to_le_bytes())?;
        out.write_all(&z.to_le_bytes())?;
        number(out, ids.len())?;
        for &id in ids {
            number(out, id)?;
        }
    }
    Ok(())
}
fn write(
    path: &Path,
    surfaces: &StreetSurface,
    paths: &RoadPaths,
    checksums: SourceChecksums,
    fingerprint: &str,
) -> io::Result<()> {
    let mut out = BufWriter::with_capacity(256 * 1024, File::create(path)?);
    out.write_all(MAGIC)?;
    out.write_all(&checksums.city.to_le_bytes())?;
    out.write_all(&checksums.streets.to_le_bytes())?;
    out.write_all(fingerprint.as_bytes())?;
    number(&mut out, paths.len())?;
    #[cfg(test)]
    let junction = surfaces.preview_junction.map_or(u32::MAX, |n| n as u32);
    #[cfg(not(test))]
    let junction = u32::MAX;
    out.write_all(&junction.to_le_bytes())?;
    for path in paths {
        number(&mut out, path.len())?;
        for p in path {
            out.write_all(&p[0].to_le_bytes())?;
            out.write_all(&p[2].to_le_bytes())?;
        }
    }
    number(&mut out, surfaces.triangles.len())?;
    for t in &surfaces.triangles {
        number(&mut out, t.style)?;
        for p in t.points {
            for x in p {
                out.write_all(&x.to_le_bytes())?;
            }
        }
    }
    number(&mut out, surfaces.curbs.len())?;
    for c in &surfaces.curbs {
        number(&mut out, c.style)?;
        for p in c.ends.into_iter().chain([c.inward]) {
            for x in p {
                out.write_all(&x.to_le_bytes())?;
            }
        }
    }
    grid(&mut out, &surfaces.triangles_grid)?;
    grid(&mut out, &surfaces.curbs_grid)?;
    out.flush()?;
    drop(out);
    let mut input = File::open(path)?;
    let mut hash = crc32fast::Hasher::new();
    let mut buffer = vec![0; 256 * 1024];
    loop {
        let n = input.read(&mut buffer)?;
        if n == 0 {
            break;
        }
        hash.update(&buffer[..n]);
    }
    let mut out = OpenOptions::new().append(true).open(path)?;
    out.write_all(&hash.finalize().to_le_bytes())?;
    out.sync_all()
}

pub(crate) fn bake(directory: &Path) -> crate::location::Result<()> {
    let started = std::time::Instant::now();
    let source = crate::package::read_to_string(directory.join("city.txt"))?;
    let streets = crate::package::read_to_string(directory.join("streets.txt"))?;
    let city = crate::city::City::parse(&source)?;
    let checksums = SourceChecksums::new(&source, &streets);
    let (surfaces, paths) = StreetSurface::load(&streets, &source, city.streets.len())?;
    let fingerprint: String = Sha256::digest(streets.as_bytes())
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect();
    if fingerprint != crate::package::content_hash(&directory.join("streets.txt"))? {
        return Err(invalid("street source changed during collision bake").into());
    }
    let packaged = crate::package::is_package(directory);
    let parent = if packaged {
        directory.parent().unwrap_or(Path::new("."))
    } else {
        directory
    };
    let temporary = tempfile::NamedTempFile::new_in(parent)?;
    write(temporary.path(), &surfaces, &paths, checksums, &fingerprint)?;
    let bytes = temporary.as_file().metadata()?.len();
    if packaged {
        crate::package::replace_asset(directory, "streets.bin", temporary.path())?;
    } else {
        temporary.persist(directory.join("streets.bin"))?;
    }
    println!(
        "Baked collision cache: {} triangles, {} curbs, {:.1} MiB; {:.2}s",
        surfaces.triangles.len(),
        surfaces.curbs.len(),
        bytes as f64 / 1048576.0,
        started.elapsed().as_secs_f64()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture(root: &Path) -> String {
        let source = "CITY 2\nbounds 0 0 20 20\nn 0 0\nn 20 0\nr 0 1 8 0\n";
        std::fs::write(root.join("city.txt"), source).unwrap();
        let streets = format!(
            "STREETS 1 1 {:08x}\nj 0\np 0 3 0 0 10 1 20 0\nt 0 0 0 20 0 0 20\nt 4 20 0 20 20 0 20\nc 0 0 0 20 0 0 1\n",
            super::super::crc32(source)
        );
        std::fs::write(root.join("streets.txt"), streets).unwrap();
        source.into()
    }
    fn compare(expected: &StreetSurface, actual: &StreetSurface) {
        assert_eq!(expected.triangles.len(), actual.triangles.len());
        assert_eq!(expected.curbs.len(), actual.curbs.len());
        assert_eq!(expected.triangles_grid, actual.triangles_grid);
        assert_eq!(expected.curbs_grid, actual.curbs_grid);
        assert_eq!(expected.preview_junction, actual.preview_junction);
        for (a, b) in expected.triangles.iter().zip(&actual.triangles) {
            assert_eq!(a.style, b.style);
            assert_eq!(a.points, b.points);
        }
        for (a, b) in expected.curbs.iter().zip(&actual.curbs) {
            assert_eq!(a.style, b.style);
            assert_eq!(a.ends, b.ends);
            assert_eq!(a.inward, b.inward);
        }
        for x in -2..24 {
            for z in -2..24 {
                let p = [x as f32, z as f32];
                assert_eq!(expected.contains(p), actual.contains(p));
                assert_eq!(expected.road_color(p), actual.road_color(p));
                assert_eq!(expected.paving_height(p), actual.paving_height(p));
                assert_eq!(
                    expected.crossed_curb(p, [p[0], p[1] + 2.0]),
                    actual.crossed_curb(p, [p[0], p[1] + 2.0])
                );
            }
        }
    }
    #[test]
    fn binary_collision_round_trip_preserves_geometry_paths_grids_and_queries() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path();
        let source = fixture(root);
        let streets = std::fs::read_to_string(root.join("streets.txt")).unwrap();
        let (expected, paths) = StreetSurface::load(&streets, &source, 1).unwrap();
        bake(root).unwrap();
        let (actual, actual_paths, sums) = load(root, &source, 1).unwrap();
        compare(&expected, &actual);
        assert_eq!(paths, actual_paths);
        assert_eq!(sums.streets, super::super::crc32(&streets));
        // Exercise the same cache through the compressed SQLite reader.
        for name in [
            "manifest.json",
            "furniture.txt",
            "landscape.txt",
            "elevation/heightfield.txt",
            "scene.pack",
        ] {
            let path = root.join(name);
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, "fixture").unwrap();
        }
        // Keep the output outside the source tree being inventoried.
        let package_temp = tempfile::tempdir().unwrap();
        let packaged = package_temp.path().join("fixture.sqlite");
        crate::package::pack(root, &packaged, false).unwrap();
        bake(&packaged).unwrap();
        let db = rusqlite::Connection::open(&packaged).unwrap();
        let compressed: i64 = db.query_row(
            "SELECT COUNT(*) FROM blocks WHERE asset_id=(SELECT id FROM assets WHERE name='streets.bin') AND codec<>0",
            [], |row| row.get(0),
        ).unwrap();
        assert_eq!(
            compressed, 0,
            "startup collision blocks should be uncompressed"
        );
        drop(db);
        let (packed, packed_paths, _) = load(&packaged, &source, 1).unwrap();
        compare(&expected, &packed);
        assert_eq!(paths, packed_paths);
        let changed = streets.replace("10 1", "10 2");
        std::fs::write(root.join("streets.txt"), &changed).unwrap();
        crate::package::replace_asset(&packaged, "streets.txt", &root.join("streets.txt")).unwrap();
        assert!(StreetSurface::load_city(&packaged, &source, 1).is_err());
        bake(&packaged).unwrap();
        let (_, updated_paths, sums) = StreetSurface::load_city(&packaged, &source, 1).unwrap();
        assert_eq!(updated_paths[0][1], [10.0, 0.0, 2.0]);
        assert_eq!(sums.streets, super::super::crc32(&changed));
    }
    #[test]
    fn required_collision_data_rejects_missing_stale_or_damaged_assets() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path();
        let source = fixture(root);
        let error = StreetSurface::load_city(root, &source, 1).err().unwrap();
        assert!(error.contains("bake-collision --location"));
        bake(root).unwrap();
        let path = root.join("streets.bin");
        let original = std::fs::read(&path).unwrap();
        let streets = std::fs::read_to_string(root.join("streets.txt")).unwrap();
        for offset in [0, 8, 16, 80, 96, original.len() - 1] {
            let mut bad = original.clone();
            bad[offset] ^= 0xff;
            std::fs::write(&path, bad).unwrap();
            assert!(StreetSurface::load_city(root, &source, 1).is_err());
        }
        for length in [0, 16, original.len() - 1] {
            std::fs::write(&path, &original[..length]).unwrap();
            assert!(StreetSurface::load_city(root, &source, 1).is_err());
        }
        std::fs::write(&path, &original).unwrap();
        assert!(StreetSurface::load_city(root, &(source.clone() + "\n"), 1).is_err());
        assert!(StreetSurface::load_city(root, &source, 2).is_err());
        // A valid street edit invalidates the cache via its content hash.
        let updated = streets.replace("p 0 3 0 0 10 1 20 0", "p 0 3 0 0 10 2 20 0");
        std::fs::write(root.join("streets.txt"), &updated).unwrap();
        assert!(StreetSurface::load_city(root, &source, 1).is_err());
        bake(root).unwrap();
        let (_, paths, _) = StreetSurface::load_city(root, &source, 1).unwrap();
        assert_eq!(paths[0][1], [10.0, 0.0, 2.0]);
    }

    #[test]
    #[ignore = "requires NFS_CITY_DATA pointing to a location with baked collision data"]
    fn full_collision_cache_matches_source() {
        let root = crate::city_assets::directory();
        let source = crate::package::read_to_string(root.join("city.txt")).unwrap();
        let streets = crate::package::read_to_string(root.join("streets.txt")).unwrap();
        let city = crate::city::City::parse(&source).unwrap();
        let (expected, expected_paths) =
            StreetSurface::load(&streets, &source, city.streets.len()).unwrap();
        let (actual, actual_paths, sums) = load(root, &source, city.streets.len()).unwrap();
        // A release bake omits the preview-only junction marker.
        let mut expected = expected;
        expected.preview_junction = actual.preview_junction;
        compare(&expected, &actual);
        assert_eq!(expected_paths, actual_paths);
        assert_eq!(sums.city, super::super::crc32(&source));
        assert_eq!(sums.streets, super::super::crc32(&streets));
        eprintln!(
            "Binary cache exactly matches {} source triangles and {} curbs",
            actual.triangles.len(),
            actual.curbs.len()
        );
    }
}
