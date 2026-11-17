use crate::model;
use crate::model::world::{CurbStyle, Material, Triangle, TriangleUsage};
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use crate::world::street_surface::cross;
use crate::world::street_surface::Curb;
use std::{collections::HashMap, fs::File, io, path::Path};

const MAGIC: &[u8; 8] = b"NFSCOL01";
type Grid = HashMap<(i32, i32), Vec<usize>>;

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

pub fn load_geometry(directory: &Path, roads: usize) -> io::Result<model::location::Streets> {
    let file = std::fs::File::open(&directory.join(FILE_NAME))?;
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
        triangles.push(triangle_from_street_id(style, points).map_err(invalid)?);
    }
    let n = count(&mut r, 28)?;
    let mut curbs = reserve(n)?;
    for _ in 0..n {
        let bytes = bytes::<_, 28>(&mut r)?;
        let style = u32::from_le_bytes(bytes[..4].try_into().unwrap()) as usize;
        let p = floats::<6>(&bytes[4..])?;
        let style = curb_style(style).map_err(|_| invalid("invalid cached curb"))?;
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
    Ok(model::location::Streets {
        triangles,
        curbs,
        paths: paths
            .into_iter()
            .map(|p| p.into_iter().map(|p| util::geometry::xz(p)).collect())
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

/// Encode plain stored geometry. The legacy file includes acceleration grids;
/// derive them here rather than making them part of the persisted data model.
fn write_geometry(path: &Path, data: &model::location::Streets) -> io::Result<()> {
    let mut triangles_grid = Grid::new();
    let mut curbs_grid = Grid::new();
    for (i, t) in data.triangles.iter().enumerate() {
        world::street_surface::index(&mut triangles_grid, &t.points, i, 0.001);
    }
    for (i, c) in data.curbs.iter().enumerate() {
        world::street_surface::index(&mut curbs_grid, &c.ends, i, 32.0);
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
        number(&mut out, triangle_street_id(t).map_err(invalid)?)?;
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

const FILE_NAME: &str = "streets.bin";

fn curb_style(value: usize) -> Result<CurbStyle, &'static str> {
    match value {
        0 => Ok(CurbStyle::MajorRoad),
        1 => Ok(CurbStyle::Road),
        2 => Ok(CurbStyle::ServiceRoad),
        _ => Err("unknown curb style"),
    }
}

/// Decode persisted triangle categories.
fn triangle_usage_from_street_id(id: usize) -> Result<TriangleUsage, &'static str> {
    match id {
        0 => Ok(TriangleUsage::Road(CurbStyle::MajorRoad)),
        1 => Ok(TriangleUsage::Road(CurbStyle::Road)),
        2 => Ok(TriangleUsage::Road(CurbStyle::ServiceRoad)),
        3 => Ok(TriangleUsage::Gutter),
        4 => Ok(TriangleUsage::Marking),
        5 => Ok(TriangleUsage::Pavement(CurbStyle::MajorRoad)),
        6 => Ok(TriangleUsage::Pavement(CurbStyle::Road)),
        7 => Ok(TriangleUsage::Pavement(CurbStyle::ServiceRoad)),
        8..=16 => Ok(TriangleUsage::Road(curb_style((id - 8) % 3)?)),
        _ => Err("unknown street triangle category"),
    }
}

fn triangle_usage_street_id(triangle_usage: TriangleUsage) -> Result<usize, &'static str> {
    match triangle_usage {
        TriangleUsage::Road(CurbStyle::MajorRoad) => Ok(0),
        TriangleUsage::Road(CurbStyle::Road) => Ok(1),
        TriangleUsage::Road(CurbStyle::ServiceRoad) => Ok(2),
        TriangleUsage::Gutter => Ok(3),
        TriangleUsage::Marking => Ok(4),
        TriangleUsage::Pavement(CurbStyle::MajorRoad) => Ok(5),
        TriangleUsage::Pavement(CurbStyle::Road) => Ok(6),
        TriangleUsage::Pavement(CurbStyle::ServiceRoad) => Ok(7),
        TriangleUsage::Ground => Err("street format does not support ground triangles"),
    }
}

/// Existing street formats imply material from the geometry category.
fn street_material(triangle_usage: TriangleUsage) -> Result<Material, &'static str> {
    match triangle_usage {
        TriangleUsage::Road(_) => Ok(Material::Asphalt),
        TriangleUsage::Gutter | TriangleUsage::Pavement(CurbStyle::MajorRoad) => {
            Ok(Material::Concrete)
        }
        TriangleUsage::Marking => Ok(Material::Paint),
        TriangleUsage::Pavement(CurbStyle::Road) => Ok(Material::Paving),
        TriangleUsage::Pavement(CurbStyle::ServiceRoad) => Ok(Material::BareGround),
        TriangleUsage::Ground => Err("street format does not support ground triangles"),
    }
}

fn triangle_from_street_id(id: usize, points: [XZ<f32>; 3]) -> Result<Triangle, &'static str> {
    let usage = triangle_usage_from_street_id(id)?;
    Ok(Triangle {
        material: match id {
            8..=10 => Material::Concrete,
            11..=13 => Material::Paving,
            14..=16 => Material::BareGround,
            _ => street_material(usage)?,
        },
        usage,
        points,
    })
}

fn triangle_street_id(triangle: &Triangle) -> Result<usize, &'static str> {
    if let TriangleUsage::Road(style) = triangle.usage {
        let base = match triangle.material {
            Material::Concrete => Some(8),
            Material::Paving => Some(11),
            Material::BareGround => Some(14),
            _ => None,
        };
        if let Some(base) = base {
            return Ok(base + style as usize);
        }
    }
    if triangle.material != street_material(triangle.usage)? {
        return Err("street format cannot encode this material for the triangle category");
    }
    triangle_usage_street_id(triangle.usage)
}

/// Publish compiler geometry atomically after its builder has decoded the intermediate.
pub fn write_compiled(directory: &Path, data: &model::location::Streets) -> io::Result<()> {
    let temporary = tempfile::NamedTempFile::new_in(directory)?;
    write_geometry(temporary.path(), data)?;
    temporary.as_file().sync_all()?;
    temporary.persist(directory.join(FILE_NAME))?;
    Ok(())
}
