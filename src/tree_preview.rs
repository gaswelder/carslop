//! Experimental RGB vegetation/crown proposals for human review, not confirmed trees.
use image::RgbImage;

#[derive(Debug)]
pub struct Candidate {
    pub x: usize,
    pub y: usize,
    pub radius: f64,
}
#[derive(Clone, Copy)]
pub struct Geo {
    pub extent: [f64; 4],
    pub width: usize,
    pub height: usize,
}
impl Geo {
    pub fn lonlat(self, x: f64, y: f64) -> [f64; 2] {
        let [west, south, east, north] = self.extent;
        let mx = west + (east - west) * x / self.width as f64;
        let my = north - (north - south) * y / self.height as f64;
        [
            (mx / 6378137.).to_degrees(),
            (2. * (my / 6378137.).exp().atan() - std::f64::consts::FRAC_PI_2).to_degrees(),
        ]
    }
    pub fn metres_per_pixel(self) -> [f64; 2] {
        let lat = self.lonlat(self.width as f64 / 2., self.height as f64 / 2.)[1].to_radians();
        [
            (self.extent[2] - self.extent[0]) * lat.cos() / self.width as f64,
            (self.extent[3] - self.extent[1]) * lat.cos() / self.height as f64,
        ]
    }
}
pub fn proposals(im: &RgbImage, scale: [f64; 2]) -> Vec<Candidate> {
    let (w, h) = (im.width() as usize, im.height() as usize);
    if w < 5 || h < 5 {
        return vec![];
    }
    // Muted crowns can be almost grey. Admit weak olive colour only where local
    // brightness variation supports foliage; smooth roads/roofs must not qualify.
    let stride = w + 1;
    let mut light_sum = vec![0_f64; stride * (h + 1)];
    let mut light_square = vec![0_f64; stride * (h + 1)];
    for y in 0..h {
        let (mut sum, mut square) = (0., 0.);
        for x in 0..w {
            let p = im.get_pixel(x as u32, y as u32).0;
            let light = p.map(f64::from).iter().sum::<f64>() / 3.;
            sum += light;
            square += light * light;
            light_sum[(y + 1) * stride + x + 1] = light_sum[y * stride + x + 1] + sum;
            light_square[(y + 1) * stride + x + 1] = light_square[y * stride + x + 1] + square;
        }
    }
    let texture_radius = (1.0 / scale[0].min(scale[1])).round().clamp(2., 12.) as usize;
    let mask: Vec<bool> = im
        .enumerate_pixels()
        .map(|(x, y, p)| {
            let [r, g, b] = p.0.map(f32::from);
            let strong = g > 25. && g < 190. && g > r * 1.05 && g > b * 0.88 && 2. * g - r - b > 0.;
            let weak = g > 25. && g < 150. && g >= r * 0.94 && g > b * 1.02;
            if strong {
                return true;
            }
            if !weak {
                return false;
            }
            let (x, y) = (x as usize, y as usize);
            let (x0, y0) = (
                x.saturating_sub(texture_radius),
                y.saturating_sub(texture_radius),
            );
            let (x1, y1) = (
                (x + texture_radius + 1).min(w),
                (y + texture_radius + 1).min(h),
            );
            let rect = |plane: &[f64]| {
                plane[y1 * stride + x1] + plane[y0 * stride + x0]
                    - plane[y0 * stride + x1]
                    - plane[y1 * stride + x0]
            };
            let area = ((x1 - x0) * (y1 - y0)) as f64;
            let mean = rect(&light_sum) / area;
            let variance = (rect(&light_square) / area - mean * mean).max(0.);
            variance >= 36.
        })
        .collect();
    drop(light_sum);
    drop(light_square);
    // Fill tiny holes and discard isolated green pixels with a 5x5 majority filter.
    let mut integral = vec![0u32; (w + 1) * (h + 1)];
    for y in 0..h {
        let mut sum = 0;
        for x in 0..w {
            sum += u32::from(mask[y * w + x]);
            integral[(y + 1) * (w + 1) + x + 1] = integral[y * (w + 1) + x + 1] + sum;
        }
    }
    let mut dist = vec![0f32; w * h];
    for y in 2..h - 2 {
        for x in 2..w - 2 {
            let (x0, y0, x1, y1) = (x - 2, y - 2, x + 3, y + 3);
            let count = integral[y1 * (w + 1) + x1] + integral[y0 * (w + 1) + x0]
                - integral[y0 * (w + 1) + x1]
                - integral[y1 * (w + 1) + x0];
            if count >= 14 {
                dist[y * w + x] = 1e6;
            }
        }
    }
    let sx = scale[0] as f32;
    let sy = scale[1] as f32;
    let diag = sx.hypot(sy);
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let i = y * w + x;
            if dist[i] > 0. {
                dist[i] = dist[i]
                    .min(dist[i - 1] + sx)
                    .min(dist[i - w] + sy)
                    .min(dist[i - w - 1] + diag)
                    .min(dist[i - w + 1] + diag);
            }
        }
    }
    for y in (1..h - 1).rev() {
        for x in (1..w - 1).rev() {
            let i = y * w + x;
            if dist[i] > 0. {
                dist[i] = dist[i]
                    .min(dist[i + 1] + sx)
                    .min(dist[i + w] + sy)
                    .min(dist[i + w + 1] + diag)
                    .min(dist[i + w - 1] + diag);
            }
        }
    }
    let mut peaks = vec![];
    for y in 2..h - 2 {
        for x in 2..w - 2 {
            let d = dist[y * w + x];
            if !(1.7..=9.0).contains(&d) {
                continue;
            }
            if (y - 2..=y + 2).all(|yy| (x - 2..=x + 2).all(|xx| dist[yy * w + xx] <= d)) {
                peaks.push(Candidate {
                    x,
                    y,
                    radius: d as f64,
                });
            }
        }
    }
    peaks.sort_by(|a, b| {
        b.radius
            .total_cmp(&a.radius)
            .then(a.y.cmp(&b.y))
            .then(a.x.cmp(&b.x))
    });
    let mut selected: Vec<Candidate> = vec![];
    for p in peaks {
        if selected.iter().all(|q| {
            let dx = (p.x as f64 - q.x as f64) * scale[0];
            let dy = (p.y as f64 - q.y as f64) * scale[1];
            dx.hypot(dy) > (p.radius + q.radius) * 0.8
        }) {
            selected.push(p);
        }
    }
    selected.sort_by_key(|p| (p.y, p.x));
    selected
}
#[cfg(test)]
mod tests {
    use super::*;
    use image::Rgb;
    #[test]
    fn muted_textured_crowns_are_detected_but_smooth_muted_surfaces_are_not() {
        let mut im = RgbImage::from_pixel(160, 100, Rgb([110, 110, 110]));
        for y in 10..90 {
            for x in 10..150 {
                if (x as i32 - 45).pow(2) + (y as i32 - 50).pow(2) < 20 * 20 {
                    let noise = ((x * 13 + y * 7) % 25) as i16 - 12;
                    im.put_pixel(
                        x,
                        y,
                        Rgb([(65 + noise) as u8, (64 + noise) as u8, (59 + noise) as u8]),
                    );
                }
                if (x as i32 - 115).pow(2) + (y as i32 - 50).pow(2) < 20 * 20 {
                    im.put_pixel(x, y, Rgb([65, 64, 59]));
                }
            }
        }
        let found = proposals(&im, [0.25, 0.25]);
        assert!(found
            .iter()
            .any(|p| (p.x as i32 - 45).abs() < 5 && (p.y as i32 - 50).abs() < 5));
        assert!(
            found.iter().all(|p| p.x < 85),
            "smooth muted surfaces must not become crowns"
        );
    }
}
