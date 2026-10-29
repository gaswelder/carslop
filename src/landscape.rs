//! Offline mapped ground cover and vegetation. Tree sizes are estimates where absent.
#[derive(Debug)]
pub struct GroundTriangle {
    pub style: usize,
    pub points: [[f32; 2]; 3],
}
#[derive(Debug)]
pub struct Tree {
    pub id: u64,
    pub point: [f32; 2],
    pub height: f32,
    pub radius: f32,
}
#[derive(Debug, Default)]
pub struct Landscape {
    pub ground: Vec<GroundTriangle>,
    pub trees: Vec<Tree>,
    grid: std::collections::HashMap<(i32, i32), Vec<usize>>,
}
impl Landscape {
    pub fn load(text: &str, city: &str, streets: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        let expected = format!(
            "LANDSCAPE 1 {:08x} {:08x}",
            crate::street_surface::crc32(city),
            crate::street_surface::crc32(streets)
        );
        if lines.next() != Some(expected.as_str()) {
            return Err(
                "stale landscape package; re-import the source with add-location under a new name"
                    .into(),
            );
        }
        let mut result = Self::default();
        for line in lines {
            let f: Vec<_> = line.split_whitespace().collect();
            let value = |i: usize| -> Result<f32, String> {
                let v: f32 = f
                    .get(i)
                    .ok_or("missing landscape coordinate")?
                    .parse()
                    .map_err(|_| "invalid landscape number")?;
                if v.is_finite() && v.abs() < 100_000.0 {
                    Ok(v)
                } else {
                    Err("invalid landscape coordinate".into())
                }
            };
            match f.first().copied() {
                Some("a") if f.len() == 8 => {
                    let style: usize = f[1].parse().map_err(|_| "invalid landscape style")?;
                    if style > 8 {
                        return Err("unknown ground style".into());
                    }
                    result.ground.push(GroundTriangle {
                        style,
                        points: [
                            [value(2)?, value(3)?],
                            [value(4)?, value(5)?],
                            [value(6)?, value(7)?],
                        ],
                    });
                }
                Some("t") if f.len() == 6 => {
                    let height = value(4)?;
                    let radius = value(5)?;
                    if !(2.5..=18.0).contains(&height) || !(0.7..=4.5).contains(&radius) {
                        return Err("invalid tree size".into());
                    }
                    result.trees.push(Tree {
                        id: f[1].parse().map_err(|_| "invalid tree id")?,
                        point: [value(2)?, value(3)?],
                        height,
                        radius,
                    });
                }
                _ => return Err("invalid landscape record".into()),
            }
        }
        for (i, t) in result.ground.iter().enumerate() {
            let low: [i32; 2] = std::array::from_fn(|axis| {
                (t.points
                    .iter()
                    .map(|p| p[axis])
                    .fold(f32::INFINITY, f32::min)
                    / 32.0)
                    .floor() as i32
            });
            let high: [i32; 2] = std::array::from_fn(|axis| {
                (t.points
                    .iter()
                    .map(|p| p[axis])
                    .fold(f32::NEG_INFINITY, f32::max)
                    / 32.0)
                    .floor() as i32
            });
            for x in low[0]..=high[0] {
                for z in low[1]..=high[1] {
                    result.grid.entry((x, z)).or_default().push(i);
                }
            }
        }
        Ok(result)
    }
    pub fn style(&self, p: [f32; 2]) -> Option<usize> {
        self.grid
            .get(&((p[0] / 32.0).floor() as i32, (p[1] / 32.0).floor() as i32))?
            .iter()
            .map(|i| &self.ground[*i])
            .find(|t| {
                if (0..2).any(|a| {
                    p[a] < t.points.iter().map(|q| q[a]).fold(f32::INFINITY, f32::min) - 0.001
                        || p[a]
                            > t.points
                                .iter()
                                .map(|q| q[a])
                                .fold(f32::NEG_INFINITY, f32::max)
                                + 0.001
                }) {
                    return false;
                }
                let edges: [f32; 3] = std::array::from_fn(|i| {
                    let a = t.points[i];
                    let b = t.points[(i + 1) % 3];
                    (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
                });
                edges.iter().all(|v| *v >= -0.00001) || edges.iter().all(|v| *v <= 0.00001)
            })
            .map(|t| t.style)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn landscape_rejects_stale_packages_and_nonfinite_geometry() {
        assert!(Landscape::load("LANDSCAPE 1 0 0", "city", "streets").is_err());
        let header = format!(
            "LANDSCAPE 1 {:08x} {:08x}\n",
            crate::street_surface::crc32("city"),
            crate::street_surface::crc32("streets")
        );
        for record in ["a 0 NaN 0 1 0 0 1", "a 9 0 0 1 0 0 1", "t 1 0 0 -2 1"] {
            assert!(Landscape::load(&(header.clone() + record), "city", "streets").is_err());
        }
    }
}
