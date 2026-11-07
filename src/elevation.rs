//! A single triangulated height surface for terrain, pavement and ground contact.
pub const SEA_LEVEL: f32 = -3.35;
#[derive(Debug)]
pub struct HeightField {
    pub columns: usize,
    pub rows: usize,
    pub origin: [f32; 2],
    pub step: f32,
    values: Vec<f32>,
}
impl HeightField {
    pub fn generate(bounds: [[f32; 2]; 2], mut height: impl FnMut([f32; 2]) -> f32) -> Self {
        let span = [bounds[1][0] - bounds[0][0], bounds[1][1] - bounds[0][1]];
        let step = 4.0_f32.max(span[0].max(span[1]) / 511.0);
        let columns = (span[0] / step).ceil() as usize + 1;
        let rows = (span[1] / step).ceil() as usize + 1;
        let mut values = Vec::with_capacity(columns * rows);
        for row in 0..rows {
            for column in 0..columns {
                values.push(height([
                    bounds[0][0] + column as f32 * step,
                    bounds[0][1] + row as f32 * step,
                ]));
            }
        }
        Self {
            columns,
            rows,
            origin: bounds[0],
            step,
            values,
        }
    }

    #[cfg(test)]
    pub fn load(text: &str, city: &str) -> Result<Self, String> {
        Self::load_with_checksum(text, crate::street_surface::crc32(city))
    }

    pub(crate) fn load_with_checksum(text: &str, city_checksum: u32) -> Result<Self, String> {
        let mut lines = text.lines();
        let header: Vec<_> = lines
            .next()
            .ok_or("missing elevation header")?
            .split_whitespace()
            .collect();
        if header.len() != 3
            || header[0] != "ELEVATION"
            || header[1] != "1"
            || u32::from_str_radix(header[2], 16).ok() != Some(city_checksum)
        {
            return Err(
                "stale elevation package; re-import the source with add-location under a new name"
                    .into(),
            );
        }
        let fields: Vec<_> = lines
            .next()
            .ok_or("missing grid")?
            .split_whitespace()
            .collect();
        if fields.len() != 7 || fields[0] != "grid" {
            return Err("invalid elevation grid".into());
        }
        let number = |i: usize| -> Result<f32, String> {
            let v: f32 = fields[i].parse().map_err(|_| "invalid grid number")?;
            if v.is_finite() {
                Ok(v)
            } else {
                Err("nonfinite grid number".into())
            }
        };
        let columns = fields[1].parse::<usize>().map_err(|_| "invalid columns")?;
        let rows = fields[2].parse::<usize>().map_err(|_| "invalid rows")?;
        let origin = [number(3)?, number(4)?];
        let step = number(5)?;
        number(6)?;
        if !(2..=4096).contains(&columns)
            || !(2..=4096).contains(&rows)
            || !(1.0..=100.0).contains(&step)
        {
            return Err("invalid grid dimensions".into());
        }
        let values = lines
            .flat_map(str::split_whitespace)
            .map(|s| {
                let v: f32 = s.parse().map_err(|_| "invalid elevation value")?;
                if v.is_finite() && v.abs() < 10000.0 {
                    Ok(v)
                } else {
                    Err("invalid elevation range")
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        if values.len() != columns * rows {
            return Err("incomplete elevation grid".into());
        }
        Ok(Self {
            columns,
            rows,
            origin,
            step,
            values,
        })
    }
    pub fn point(&self, column: usize, row: usize) -> [f32; 3] {
        [
            self.origin[0] + column as f32 * self.step,
            self.values[row * self.columns + column],
            self.origin[1] + row as f32 * self.step,
        ]
    }
    pub fn height(&self, p: [f32; 2]) -> f32 {
        let x = ((p[0] - self.origin[0]) / self.step).clamp(0.0, (self.columns - 1) as f32);
        let z = ((p[1] - self.origin[1]) / self.step).clamp(0.0, (self.rows - 1) as f32);
        let col = (x.floor() as usize).min(self.columns - 2);
        let row = (z.floor() as usize).min(self.rows - 2);
        let x = x - col as f32;
        let z = z - row as f32;
        let a = self.values[row * self.columns + col];
        let b = self.values[row * self.columns + col + 1];
        let d = self.values[(row + 1) * self.columns + col];
        let e = self.values[(row + 1) * self.columns + col + 1];
        if z <= x {
            a + (b - a) * x + (e - b) * z
        } else {
            a + (e - d) * x + (d - a) * z
        }
    }
    /// Split on grid lines and cell diagonals so a curb/wall edge follows the
    /// exact same planar surface as pavement, including where its slope changes.
    pub fn segment(&self, a: [f32; 2], b: [f32; 2]) -> Vec<[f32; 3]> {
        let delta = [b[0] - a[0], b[1] - a[1]];
        let mut ts = vec![0.0, 1.0];
        for axis in 0..2 {
            if delta[axis].abs() < 0.00001 {
                continue;
            }
            let count = if axis == 0 { self.columns } else { self.rows };
            let lo =
                (((a[axis].min(b[axis]) - self.origin[axis]) / self.step).floor() as i32).max(0);
            let hi = (((a[axis].max(b[axis]) - self.origin[axis]) / self.step).ceil() as i32)
                .min(count as i32 - 1);
            for i in lo..=hi {
                let t = (self.origin[axis] + i as f32 * self.step - a[axis]) / delta[axis];
                if t > 0.000001 && t < 0.999999 {
                    ts.push(t);
                }
            }
        }
        ts.sort_by(f32::total_cmp);
        ts.dedup_by(|a, b| (*a - *b).abs() < 0.000001);
        let mut extra = Vec::new();
        if (delta[0] - delta[1]).abs() > 0.00001 {
            for pair in ts.windows(2) {
                let t = (pair[0] + pair[1]) * 0.5;
                let cell: [f32; 2] = std::array::from_fn(|i| {
                    self.origin[i]
                        + ((a[i] + delta[i] * t - self.origin[i]) / self.step).floor() * self.step
                });
                let crossing = (cell[0] - cell[1] - a[0] + a[1]) / (delta[0] - delta[1]);
                if crossing > pair[0] + 0.000001 && crossing < pair[1] - 0.000001 {
                    extra.push(crossing);
                }
            }
        }
        ts.extend(extra);
        ts.sort_by(f32::total_cmp);
        ts.into_iter()
            .map(|t| {
                let x = a[0] + delta[0] * t;
                let z = a[1] + delta[1] * t;
                [x, self.height([x, z]), z]
            })
            .collect()
    }
    /// Clip flat map triangles onto individual height-grid planes. Merely
    /// lifting the original large triangles would bury roads or leave gaps.
    pub fn drape(&self, points: [[f32; 2]; 3], mut emit: impl FnMut([[f32; 3]; 3])) {
        let min: [f32; 2] =
            std::array::from_fn(|i| points.iter().map(|p| p[i]).fold(f32::INFINITY, f32::min));
        let max: [f32; 2] = std::array::from_fn(|i| {
            points
                .iter()
                .map(|p| p[i])
                .fold(f32::NEG_INFINITY, f32::max)
        });
        let lo: [usize; 2] = std::array::from_fn(|i| {
            (((min[i] - self.origin[i]) / self.step).floor().max(0.0) as usize).min(if i == 0 {
                self.columns - 2
            } else {
                self.rows - 2
            })
        });
        let hi: [usize; 2] = std::array::from_fn(|i| {
            (((max[i] - self.origin[i]) / self.step).floor().max(0.0) as usize).min(if i == 0 {
                self.columns - 2
            } else {
                self.rows - 2
            })
        });
        for row in lo[1]..=hi[1] {
            for column in lo[0]..=hi[0] {
                let x = self.origin[0] + column as f32 * self.step;
                let z = self.origin[1] + row as f32 * self.step;
                let mut polygon = points.to_vec();
                for (axis, value, sign) in [
                    (0, x, 1.0),
                    (0, x + self.step, -1.0),
                    (1, z, 1.0),
                    (1, z + self.step, -1.0),
                ] {
                    polygon = clip(&polygon, |p| (p[axis] - value) * sign);
                    if polygon.len() < 3 {
                        break;
                    }
                }
                if polygon.len() < 3 {
                    continue;
                }
                for side in [-1.0, 1.0] {
                    let part = clip(&polygon, |p| ((p[0] - x) - (p[1] - z)) * side);
                    for i in 1..part.len().saturating_sub(1) {
                        let tri = [part[0], part[i], part[i + 1]];
                        emit(tri.map(|p| [p[0], self.height(p), p[1]]));
                    }
                }
            }
        }
    }
}
fn clip(polygon: &[[f32; 2]], distance: impl Fn([f32; 2]) -> f32) -> Vec<[f32; 2]> {
    let mut result = Vec::new();
    let Some(&last) = polygon.last() else {
        return result;
    };
    let mut previous = last;
    let mut before = distance(previous);
    for &current in polygon {
        let after = distance(current);
        if (after >= 0.0) != (before >= 0.0) {
            let t = (before / (before - after)).clamp(0.0, 1.0);
            result.push(std::array::from_fn(|i| {
                previous[i] + (current[i] - previous[i]) * t
            }));
        }
        if after >= 0.0 {
            result.push(current);
        }
        previous = current;
        before = after;
    }
    result
}
#[cfg(test)]
mod tests {
    use super::*;
    fn grid() -> HeightField {
        HeightField {
            columns: 3,
            rows: 3,
            origin: [0.0; 2],
            step: 10.0,
            values: vec![0.0, 2.0, 4.0, 1.0, 6.0, 5.0, 2.0, 4.0, 8.0],
        }
    }
    #[test]
    fn sampling_and_draping_use_the_same_planes_across_cells() {
        let grid = grid();
        assert_eq!(grid.height([10.0, 10.0]), 6.0);
        let points = [[1.0, 1.0], [19.0, 3.0], [3.0, 19.0]];
        let mut area = 0.0;
        grid.drape(points, |tri| {
            let a = tri[0];
            let b = tri[1];
            let c = tri[2];
            area += ((b[0] - a[0]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[0] - a[0])).abs() * 0.5;
            let center: [f32; 3] = std::array::from_fn(|i| (a[i] + b[i] + c[i]) / 3.0);
            assert!((center[1] - grid.height([center[0], center[2]])).abs() < 0.0001);
        });
        assert!((area - 160.0).abs() < 0.001);
        let points = grid.segment([1.0, 2.0], [19.0, 14.0]);
        assert!(points.len() > 3);
        for pair in points.windows(2) {
            let p: [f32; 3] = std::array::from_fn(|i| (pair[0][i] + pair[1][i]) * 0.5);
            assert!((p[1] - grid.height([p[0], p[2]])).abs() < 0.0001);
        }
    }
    #[test]
    fn bad_and_stale_heightfields_are_rejected() {
        assert!(HeightField::load("ELEVATION 1 00000000", "city").is_err());
        let header = format!(
            "ELEVATION 1 {:08x}\ngrid 2 2 0 0 20 180\nNaN 0 0 0\n",
            crate::street_surface::crc32("city")
        );
        assert!(HeightField::load(&header, "city").is_err());
    }
}
