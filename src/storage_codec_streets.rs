//! Text street geometry and category IDs shared with the binary street codec.

use crate::model_location;
use crate::model_world::{Curb, CurbStyle, Material, Triangle, TriangleUsage};
use crate::util_geometry::XZ;
use crate::world_street_surface::cross;

pub fn parse(streets_txt_source: &str, count: usize) -> Result<model_location::Streets, String> {
    let mut lines123213 = streets_txt_source.lines();
    let header: Vec<_> = lines123213
        .next()
        .ok_or("missing surfaces header")?
        .split_whitespace()
        .collect();
    if header.len() < 3
        || header[0] != "STREETS"
        || header[1] != "1"
        || header[2].parse::<usize>().ok() != Some(count)
    {
        return Err("unsupported street surfaces header or road count".into());
    }
    let mut result = model_location::Streets {
        triangles: Vec::new(),
        curbs: Vec::new(),
        paths: Vec::new(),
    };
    let mut paths = vec![Vec::new(); count];
    for line in lines123213 {
        let f: Vec<_> = line.split_whitespace().collect();
        let n = |i: usize| -> Result<usize, String> {
            f.get(i)
                .ok_or("missing surface index")?
                .parse()
                .map_err(|_| "invalid surface index".into())
        };
        let v = |i: usize| -> Result<f32, String> {
            let value: f32 = f
                .get(i)
                .ok_or("missing surface coordinate")?
                .parse()
                .map_err(|_| "invalid surface coordinate")?;
            if value.is_finite() {
                Ok(value)
            } else {
                Err("nonfinite surface coordinate".into())
            }
        };
        let p = |i: usize| -> Result<XZ<f32>, String> {
            Ok(XZ {
                x: v(i)?,
                z: v(i + 1)?,
            })
        };
        match f.first().copied() {
            Some("j") => {
                n(1)?;
            }
            Some("p") => {
                let id = n(1)?;
                let length = n(2)?;
                if id >= count || length < 2 || f.len() != 3 + length * 2 || !paths[id].is_empty() {
                    return Err("invalid surface path".into());
                }
                paths[id] = (0..length)
                    .map(|i| {
                        Ok(XZ {
                            x: v(3 + i * 2)?,
                            z: v(4 + i * 2)?,
                        })
                    })
                    .collect::<Result<_, String>>()?;
            }
            Some("t") => {
                let id = n(1)?;
                if f.len() != 8 {
                    return Err("invalid surface triangle".into());
                }
                let points = [p(2)?, p(4)?, p(6)?];
                if cross(points[0], points[1], points[2]) > 0.00001 {
                    result.triangles.push(triangle_from_street_id(id, points)?);
                }
            }
            Some("c") => {
                let style = CurbStyle::try_from(n(1)?)?;
                if f.len() != 8 {
                    return Err("invalid curb".into());
                }
                result.curbs.push(Curb {
                    style,
                    ends: [p(2)?, p(4)?],
                    inward: p(6)?,
                });
            }
            _ => return Err("unknown surface record".into()),
        }
    }
    // An entirely grade-separated location has paths and no ground mesh.
    if result.curbs.is_empty() != result.triangles.is_empty() || paths.iter().any(Vec::is_empty) {
        return Err("incomplete street surfaces".into());
    }
    result.paths = paths;
    Ok(result)
}
impl TryFrom<usize> for CurbStyle {
    type Error = &'static str;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::MajorRoad),
            1 => Ok(Self::Road),
            2 => Ok(Self::ServiceRoad),
            _ => Err("unknown curb style"),
        }
    }
}

/// Decode the categories used by streets.txt and streets.bin.
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
        8..=16 => Ok(TriangleUsage::Road(CurbStyle::try_from((id - 8) % 3)?)),
        _ => Err("unknown street triangle category"),
    }
}

pub fn triangle_usage_street_id(triangle_usage: TriangleUsage) -> Result<usize, &'static str> {
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

pub fn triangle_from_street_id(id: usize, points: [XZ<f32>; 3]) -> Result<Triangle, &'static str> {
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

pub fn triangle_street_id(triangle: &Triangle) -> Result<usize, &'static str> {
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
