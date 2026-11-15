use crate::geometry;
use crate::geometry::{XYZ, XZ};
use crate::location;
use crate::street_surface;
use crate::street_surface::{cross, Curb, RoadPaths, StreetSurface};
use crate::world;
use std::{collections::HashMap, fs::File, io, path::Path};

const MAGIC: &[u8; 8] = b"NFSCOL01";
type Grid = HashMap<(i32, i32), Vec<usize>>;

#[cfg(test)]
type Loaded = (StreetSurface, RoadPaths);

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

struct Decoder<R> {
    input: R,
    remaining: u64,
}

fn bytes<R: io::Read, const N: usize>(decoder: &mut Decoder<R>) -> io::Result<[u8; N]> {
    if decoder.remaining < N as u64 {
        return Err(invalid("truncated collision cache"));
    }
    let mut out = [0; N];
    decoder.input.read_exact(&mut out)?;
    decoder.remaining -= N as u64;
    Ok(out)
}
fn count<R: io::Read>(decoder: &mut Decoder<R>, stride: u64) -> io::Result<usize> {
    let n = u32::from_le_bytes(bytes(decoder)?) as usize;
    if n as u64 * stride > decoder.remaining {
        return Err(invalid("invalid collision cache count"));
    }
    Ok(n)
}
fn decoder_grid<R: io::Read>(decoder: &mut Decoder<R>, limit: usize) -> io::Result<()> {
    let n = count(decoder, 12)?;
    let mut seen = std::collections::HashSet::new();
    for _ in 0..n {
        let x = i32::from_le_bytes(bytes(decoder)?);
        let z = i32::from_le_bytes(bytes(decoder)?);
        let count = count(decoder, 4)?;
        let mut ids = reserve(count)?;
        for _ in 0..count {
            let id = u32::from_le_bytes(bytes(decoder)?) as usize;
            if id >= limit || ids.last().is_some_and(|last| *last >= id) {
                return Err(invalid("invalid collision grid reference"));
            }
            ids.push(id);
        }
        if count == 0 || !seen.insert((x, z)) {
            return Err(invalid("invalid collision grid cell"));
        }
    }
    if n == 0 && limit != 0 {
        return Err(invalid("empty collision grid"));
    }
    Ok(())
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

#[cfg(test)]
fn load(directory: &Path, roads: usize) -> io::Result<Loaded> {
    street_surface::from_geometry(
        load_geometry(directory, roads)?,
        street_surface::DEFAULT_PAVEMENT_HEIGHT,
    )
    .map_err(|message| invalid(&message))
}

pub fn load_geometry(directory: &Path, roads: usize) -> io::Result<location::Streets> {
    let file = std::fs::File::open(&directory.join("streets.bin"))?;
    let body_len = file
        .metadata()?
        .len()
        .checked_sub(4)
        .ok_or_else(|| invalid("truncated collision cache"))?;
    let mut r = Decoder {
        input: io::BufReader::with_capacity(256 * 1024, io::Read::take(file, body_len)),
        remaining: body_len,
    };
    if &bytes::<_, 8>(&mut r)? != MAGIC {
        return Err(invalid("unsupported collision cache version"));
    }
    // Reserved fields preserve compatibility with existing collision packages.
    let _ = bytes::<_, 72>(&mut r)?;
    if count(&mut r, 4)? != roads {
        return Err(invalid("collision road count mismatch"));
    }
    // Reserved junction marker from older caches.
    let _ = bytes::<_, 4>(&mut r)?;
    let mut paths = reserve(roads)?;
    for _ in 0..roads {
        let n = count(&mut r, 8)?;
        if n < 2 {
            return Err(invalid("invalid collision road path"));
        }
        let mut path = reserve(n)?;
        for _ in 0..n {
            let p = floats::<2>(&bytes::<_, 8>(&mut r)?)?;
            path.push(XYZ {
                x: p[0],
                y: 0.0,
                z: p[1],
            });
        }
        paths.push(path);
    }
    let n = count(&mut r, 28)?;
    let mut triangles = reserve(n)?;
    for _ in 0..n {
        let bytes = bytes::<_, 28>(&mut r)?;
        let style = u32::from_le_bytes(bytes[..4].try_into().unwrap()) as usize;
        let p = floats::<6>(&bytes[4..])?;
        let points = [
            XZ { x: p[0], z: p[1] },
            XZ { x: p[2], z: p[3] },
            XZ { x: p[4], z: p[5] },
        ];
        if cross(points[0], points[1], points[2]) <= 0.00001 {
            return Err(invalid("invalid cached surface triangle"));
        }
        triangles.push(street_surface::triangle_from_street_id(style, points).map_err(invalid)?);
    }
    let n = count(&mut r, 28)?;
    let mut curbs = reserve(n)?;
    for _ in 0..n {
        let bytes = bytes::<_, 28>(&mut r)?;
        let style = u32::from_le_bytes(bytes[..4].try_into().unwrap()) as usize;
        let p = floats::<6>(&bytes[4..])?;
        let style =
            world::CurbStyle::try_from(style).map_err(|_| invalid("invalid cached curb"))?;
        curbs.push(Curb {
            style,
            ends: [XZ { x: p[0], z: p[1] }, XZ { x: p[2], z: p[3] }],
            inward: XZ { x: p[4], z: p[5] },
        });
    }
    if triangles.is_empty() != curbs.is_empty() {
        return Err(invalid("incomplete collision cache"));
    }
    decoder_grid(&mut r, triangles.len())?;
    decoder_grid(&mut r, curbs.len())?;
    if r.remaining != 0 {
        return Err(invalid("trailing collision cache bytes"));
    }
    Ok(location::Streets {
        triangles,
        curbs,
        paths: paths
            .into_iter()
            .map(|p| p.into_iter().map(|p| geometry::xz(p)).collect())
            .collect(),
    })
}

fn number(out: &mut impl io::Write, n: usize) -> io::Result<()> {
    out.write_all(
        &u32::try_from(n)
            .map_err(|_| invalid("collision count exceeds u32"))?
            .to_le_bytes(),
    )
}
fn grid(out: &mut impl io::Write, cells: &Grid) -> io::Result<()> {
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

#[cfg(test)]
mod tests {
    use crate::city_assets;
    use crate::scene;
    use crate::street_surface;
    use crate::vehicle_location;
    use crate::world;

    use super::*;
    use crate::build_collision;
    use crate::error;
    use crate::source_location;
    use crate::test_support;
    fn bake(city: &Path, streets: &Path, output: &Path) -> error::Result<()> {
        let city = source_location::parse(&std::fs::read_to_string(city)?)?;
        build_collision::build_collisions(output, &city, std::fs::read_to_string(streets)?)
    }
    fn fixture(root: &Path) -> String {
        let source = "CITY 2\nbounds 0 0 20 20\nn 0 0\nn 20 0\nr 0 1 8 0 1\n";
        std::fs::write(root.join("city.txt"), source).unwrap();
        let streets = format!(
            "STREETS 1 1 {:08x}\nj 0\np 0 3 0 0 10 1 20 0\nt 0 0 0 20 0 0 20\nt 4 20 0 20 20 0 20\nc 0 0 0 20 0 0 1\n",
            0
        );
        std::fs::write(root.join("streets.txt"), streets).unwrap();
        source.into()
    }
    fn compare(expected: StreetSurface, actual: StreetSurface) {
        assert_eq!(expected.road_triangles.len(), actual.road_triangles.len());
        assert_eq!(expected.curbs.len(), actual.curbs.len());
        assert_eq!(expected.triangles_grid, actual.triangles_grid);
        assert_eq!(expected.curbs_grid, actual.curbs_grid);
        for (a, b) in expected.road_triangles.iter().zip(&actual.road_triangles) {
            assert_eq!(a.material, b.material);
            assert_eq!(a.usage, b.usage);
            assert_eq!(a.points, b.points);
        }
        for (a, b) in expected.curbs.iter().zip(&actual.curbs) {
            assert_eq!(a.style, b.style);
            assert_eq!(a.ends, b.ends);
            assert_eq!(a.inward, b.inward);
        }
        let mut expected_world = test_support::empty_world();
        test_support::install_surfaces(&mut expected_world, expected);
        let expected = expected_world;
        let mut actual_world = test_support::empty_world();
        test_support::install_surfaces(&mut actual_world, actual);
        let actual = actual_world;
        for x in -2..24 {
            for z in -2..24 {
                let p = XZ {
                    x: x as f32,
                    z: z as f32,
                };
                assert_eq!(
                    street_surface::contains_road(&(expected), p),
                    street_surface::contains_road(&(actual), p)
                );
                assert_eq!(
                    scene::road_color(&(expected), p),
                    scene::road_color(&(actual), p)
                );
                assert_eq!(
                    street_surface::paving_height(&(expected), p),
                    street_surface::paving_height(&(actual), p)
                );
                assert_eq!(
                    vehicle_location::crossed_curb(
                        &(expected),
                        p,
                        XZ {
                            x: p.x,
                            z: p.z + 2.0
                        }
                    ),
                    vehicle_location::crossed_curb(
                        &(actual),
                        p,
                        XZ {
                            x: p.x,
                            z: p.z + 2.0
                        }
                    )
                );
            }
        }
    }
    #[test]
    fn binary_collision_round_trip_preserves_geometry_paths_grids_and_queries() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path();
        fixture(root);
        let streets = std::fs::read_to_string(root.join("streets.txt")).unwrap();
        let (expected, paths) = street_surface::load(&streets, 1).unwrap();
        bake(
            &root.join("city.txt"),
            &root.join("streets.txt"),
            &root.join("streets.bin"),
        )
        .unwrap();
        let (actual, actual_paths) = load(root, 1).unwrap();
        compare(expected, actual);
        assert_eq!(paths, actual_paths);
        let changed = streets.replace("10 1", "10 2");
        std::fs::write(root.join("streets.txt"), &changed).unwrap();
        assert!(load(root, 1).is_ok());
        bake(
            &root.join("city.txt"),
            &root.join("streets.txt"),
            &root.join("streets.bin"),
        )
        .unwrap();
        let (_, updated_paths) = load(root, 1).unwrap();
        assert_eq!(
            updated_paths[0][1],
            XYZ {
                x: 10.0,
                y: 0.0,
                z: 2.0
            }
        );

        let city_path = root.join("map.data");
        let streets_path = root.join("roads.data");
        let output_directory = root.join("collision");
        std::fs::create_dir(&output_directory).unwrap();
        let output_path = output_directory.join("cache.bin");
        std::fs::rename(root.join("city.txt"), &city_path).unwrap();
        std::fs::rename(root.join("streets.txt"), &streets_path).unwrap();
        bake(&city_path, &streets_path, &output_path).unwrap();
        assert_eq!(
            std::fs::read(output_path).unwrap(),
            std::fs::read(root.join("streets.bin")).unwrap()
        );
        assert_eq!(std::fs::read_dir(output_directory).unwrap().count(), 1);
    }
    #[test]
    fn collision_data_ignores_reserved_bytes_and_requires_valid_geometry() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path();
        fixture(root);
        assert!(load(root, 1).is_err());
        bake(
            &root.join("city.txt"),
            &root.join("streets.txt"),
            &root.join("streets.bin"),
        )
        .unwrap();
        let path = root.join("streets.bin");
        let original = std::fs::read(&path).unwrap();
        let mut legacy = original.clone();
        legacy[8..80].fill(0xff);
        let footer = legacy.len() - 4;
        legacy[footer..].fill(0xff);
        std::fs::write(&path, legacy).unwrap();
        assert!(load(root, 1).is_ok());
        std::fs::remove_file(root.join("streets.txt")).unwrap();
        assert!(load(root, 1).is_ok());
        assert!(load(root, 2).is_err());
        for length in [0, 16, original.len() - 1] {
            std::fs::write(&path, &original[..length]).unwrap();
            assert!(load(root, 1).is_err());
        }
    }

    #[test]
    fn generated_collision_cache_matches_source() {
        let world = test_support::city_world();
        let assets = world::assets(&world).unwrap();
        let root = city_assets::directory(assets);
        let source = std::fs::read_to_string(root.join("city.txt")).unwrap();
        let streets = std::fs::read_to_string(root.join("streets.txt")).unwrap();
        let city = source_location::parse(&source).unwrap();
        let (expected, expected_paths) =
            street_surface::load(&streets, city.road_lines.len()).unwrap();
        let (actual, actual_paths) = load(root, city.road_lines.len()).unwrap();
        assert_eq!(expected_paths, actual_paths);
        eprintln!(
            "Binary cache exactly matches {} source triangles and {} curbs",
            actual.road_triangles.len(),
            actual.curbs.len()
        );
        compare(expected, actual);
    }
}

pub fn write_binary_street_surface(
    path: &Path,
    surfaces: &StreetSurface,
    paths: &RoadPaths,
) -> io::Result<()> {
    let mut out = io::BufWriter::with_capacity(256 * 1024, File::create(path)?);
    io::Write::write_all(&mut out, MAGIC)?;
    io::Write::write_all(&mut out, &[0; 72])?;
    number(&mut out, paths.len())?;
    // Reserved junction marker retained for compatibility with existing caches.
    io::Write::write_all(&mut out, &u32::MAX.to_le_bytes())?;
    for path in paths {
        number(&mut out, path.len())?;
        for p in path {
            io::Write::write_all(&mut out, &p.x.to_le_bytes())?;
            io::Write::write_all(&mut out, &p.z.to_le_bytes())?;
        }
    }
    number(&mut out, surfaces.road_triangles.len())?;
    for t in &surfaces.road_triangles {
        number(
            &mut out,
            street_surface::triangle_street_id(t).map_err(invalid)?,
        )?;
        for p in t.points {
            for x in [p.x, p.z] {
                io::Write::write_all(&mut out, &x.to_le_bytes())?;
            }
        }
    }
    number(&mut out, surfaces.curbs.len())?;
    for c in &surfaces.curbs {
        number(&mut out, c.style as usize)?;
        for p in c.ends.into_iter().chain([c.inward]) {
            for x in p {
                io::Write::write_all(&mut out, &x.to_le_bytes())?;
            }
        }
    }
    grid(&mut out, &surfaces.triangles_grid)?;
    grid(&mut out, &surfaces.curbs_grid)?;
    // Reserved footer preserves the existing binary layout.
    io::Write::write_all(&mut out, &[0; 4])?;
    io::Write::flush(&mut out)?;
    out.get_ref().sync_all()
}

/// Encode plain stored geometry. The legacy file includes acceleration grids;
/// derive them here rather than making them part of the persisted data model.
pub fn write_geometry(path: &Path, data: &location::Streets) -> io::Result<()> {
    let mut triangles_grid = Grid::new();
    let mut curbs_grid = Grid::new();
    for (i, t) in data.triangles.iter().enumerate() {
        street_surface::index(&mut triangles_grid, &t.points, i, 0.001);
    }
    for (i, c) in data.curbs.iter().enumerate() {
        street_surface::index(&mut curbs_grid, &c.ends, i, 32.0);
    }
    let mut out = io::BufWriter::new(File::create(path)?);
    io::Write::write_all(&mut out, MAGIC)?;
    io::Write::write_all(&mut out, &[0; 72])?;
    number(&mut out, data.paths.len())?;
    io::Write::write_all(&mut out, &u32::MAX.to_le_bytes())?;
    for path in &data.paths {
        number(&mut out, path.len())?;
        for p in path {
            io::Write::write_all(&mut out, &p.x.to_le_bytes())?;
            io::Write::write_all(&mut out, &p.z.to_le_bytes())?;
        }
    }
    number(&mut out, data.triangles.len())?;
    for t in &data.triangles {
        number(
            &mut out,
            street_surface::triangle_street_id(t).map_err(invalid)?,
        )?;
        for p in t.points {
            io::Write::write_all(&mut out, &p.x.to_le_bytes())?;
            io::Write::write_all(&mut out, &p.z.to_le_bytes())?;
        }
    }
    number(&mut out, data.curbs.len())?;
    for c in &data.curbs {
        number(&mut out, c.style as usize)?;
        for p in c.ends.into_iter().chain([c.inward]) {
            io::Write::write_all(&mut out, &p.x.to_le_bytes())?;
            io::Write::write_all(&mut out, &p.z.to_le_bytes())?;
        }
    }
    grid(&mut out, &triangles_grid)?;
    grid(&mut out, &curbs_grid)?;
    io::Write::write_all(&mut out, &[0; 4])?;
    io::Write::flush(&mut out)
}
