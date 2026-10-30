//! Experimental RGB vegetation/crown proposals for human review, not confirmed trees.
use crate::location::{invalid, Result};
use image::{Rgb, RgbImage};
use serde_json::{json, Value};
use std::{fs, path::Path};

#[derive(Debug)]
pub(crate) struct Candidate {
    pub(crate) x: usize,
    pub(crate) y: usize,
    pub(crate) radius: f64,
}
#[derive(Clone, Copy)]
pub(crate) struct Geo {
    pub(crate) extent: [f64; 4],
    pub(crate) width: usize,
    pub(crate) height: usize,
}
impl Geo {
    pub(crate) fn lonlat(self, x: f64, y: f64) -> [f64; 2] {
        let [west, south, east, north] = self.extent;
        let mx = west + (east - west) * x / self.width as f64;
        let my = north - (north - south) * y / self.height as f64;
        [
            (mx / 6378137.).to_degrees(),
            (2. * (my / 6378137.).exp().atan() - std::f64::consts::FRAC_PI_2).to_degrees(),
        ]
    }
    fn pixel(self, lon: f64, lat: f64) -> [f64; 2] {
        let mx = lon.to_radians() * 6378137.;
        let my = (std::f64::consts::FRAC_PI_4 + lat.to_radians() / 2.)
            .tan()
            .ln()
            * 6378137.;
        [
            (mx - self.extent[0]) / (self.extent[2] - self.extent[0]) * self.width as f64,
            (self.extent[3] - my) / (self.extent[3] - self.extent[1]) * self.height as f64,
        ]
    }
    pub(crate) fn metres_per_pixel(self) -> [f64; 2] {
        let lat = self.lonlat(self.width as f64 / 2., self.height as f64 / 2.)[1].to_radians();
        [
            (self.extent[2] - self.extent[0]) * lat.cos() / self.width as f64,
            (self.extent[3] - self.extent[1]) * lat.cos() / self.height as f64,
        ]
    }
}
pub(crate) fn proposals(im: &RgbImage, scale: [f64; 2]) -> Vec<Candidate> {
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
fn osm_trees(path: &Path) -> Result<Vec<(String, f64, f64)>> {
    use quick_xml::{events::Event, Reader};
    let mut r = Reader::from_reader(std::io::BufReader::new(crate::package::Reader::open(path)?));
    let mut buf = vec![];
    let mut node = None;
    let mut trees = vec![];
    let mut tree = false;
    loop {
        match r.read_event_into(&mut buf)? {
            Event::Start(e) | Event::Empty(e) => {
                let name = e.name().as_ref().to_owned();
                if name == "remark" {
                    return Err(invalid("incomplete OSM snapshot"));
                }
                if name == "node" || name == "tag" {
                    let mut a = std::collections::BTreeMap::new();
                    for attr in e.attributes() {
                        let attr = attr?;
                        a.insert(
                            attr.key.as_ref().to_owned(),
                            attr.normalized_value(quick_xml::XmlVersion::Implicit1_0)?
                                .into_owned(),
                        );
                    }
                    if name == "node" {
                        tree = false;
                        node = Some((
                            a.get("id").cloned().unwrap_or_default(),
                            a.get("lon")
                                .ok_or_else(|| invalid("node missing longitude"))?
                                .parse::<f64>()?,
                            a.get("lat")
                                .ok_or_else(|| invalid("node missing latitude"))?
                                .parse::<f64>()?,
                        ));
                    } else if a.get("k").is_some_and(|s| s == "natural")
                        && a.get("v").is_some_and(|s| s == "tree")
                    {
                        tree = true;
                    }
                }
            }
            Event::End(e) if e.name().as_ref() == "node" => {
                if tree {
                    if let Some(n) = node.take() {
                        trees.push(n);
                    }
                }
                node = None;
                tree = false;
            }
            Event::Eof => break,
            _ => {}
        }
        buf.clear();
    }
    Ok(trees)
}
fn ring(im: &mut RgbImage, x: f64, y: f64, rx: f64, ry: f64, colour: [u8; 3]) {
    let steps = (rx.max(ry) * 10.).ceil().max(24.) as usize;
    for i in 0..steps {
        let angle = i as f64 / steps as f64 * std::f64::consts::TAU;
        let px = (x + rx * angle.cos()).round() as i64;
        let py = (y + ry * angle.sin()).round() as i64;
        for dy in -1..=1 {
            for dx in -1..=1 {
                let (xx, yy) = (px + dx, py + dy);
                if xx >= 0 && yy >= 0 && xx < im.width() as i64 && yy < im.height() as i64 {
                    im.put_pixel(xx as u32, yy as u32, Rgb(colour));
                }
            }
        }
    }
}
pub fn run(
    image_path: &Path,
    export: &Path,
    output: &Path,
    osm: Option<&Path>,
    city_data: Option<&Path>,
) -> Result<()> {
    let v: Value = serde_json::from_slice(&fs::read(export)?)?;
    let e = &v["extent"];
    let mut extent = [0.; 4];
    for (i, k) in ["xmin", "ymin", "xmax", "ymax"].iter().enumerate() {
        extent[i] = e[k]
            .as_f64()
            .filter(|n| n.is_finite())
            .ok_or_else(|| invalid(format!("invalid extent {k}")))?;
    }
    if extent[0] >= extent[2] || extent[1] >= extent[3] {
        return Err(invalid("invalid export bounds"));
    }
    let sr = &e["spatialReference"];
    if ![sr["wkid"].as_u64(), sr["latestWkid"].as_u64()]
        .iter()
        .any(|n| matches!(n, Some(3857 | 102100 | 102113)))
    {
        return Err(invalid(
            "tree preview requires Web Mercator (EPSG:3857) export metadata",
        ));
    }
    let im = image::ImageReader::open(image_path)?.decode()?.to_rgb8();
    if v["width"].as_u64().is_some_and(|w| w != im.width() as u64)
        || v["height"]
            .as_u64()
            .is_some_and(|h| h != im.height() as u64)
    {
        return Err(invalid("image dimensions differ from export metadata"));
    }
    let geo = Geo {
        extent,
        width: im.width() as usize,
        height: im.height() as usize,
    };
    let scale = geo.metres_per_pixel();
    if !scale.iter().all(|s| s.is_finite() && *s > 0.) {
        return Err(invalid("invalid image ground scale"));
    }
    let sw = geo.lonlat(0., geo.height as f64);
    let ne = geo.lonlat(geo.width as f64, 0.);
    let obstacles = city_data
        .map(|dir| crate::tree_obstacles::Obstacles::read(dir, [sw[0], sw[1], ne[0], ne[1]]))
        .transpose()?;
    let candidates = proposals(&im, scale);
    let mapped = if let Some(path) = osm {
        osm_trees(path)?
    } else {
        vec![]
    };
    let mapped: Vec<_> = mapped
        .into_iter()
        .filter_map(|(id, lon, lat)| {
            let [x, y] = geo.pixel(lon, lat);
            if x >= 0. && y >= 0. && x < geo.width as f64 && y < geo.height as f64 {
                Some((id, x, y, lon, lat))
            } else {
                None
            }
        })
        .collect();
    let mut overlay = im.clone();
    let mut features = vec![];
    let mut matches = 0;
    let mut road_rejections = 0;
    let mut building_rejections = 0;
    for (i, p) in candidates.iter().enumerate() {
        let nearby = mapped
            .iter()
            .filter(|(_, x, y, _, _)| {
                ((p.x as f64 - x) * scale[0]).hypot((p.y as f64 - y) * scale[1]) <= p.radius + 2.
            })
            .map(|(id, _, _, _, _)| id.clone())
            .collect::<Vec<_>>();
        if !nearby.is_empty() {
            matches += 1;
        }
        let ll = geo.lonlat(p.x as f64, p.y as f64);
        let rejection = obstacles.as_ref().and_then(|o| o.rejection(ll[0], ll[1]));
        match rejection {
            Some("road") => road_rejections += 1,
            Some("building") => building_rejections += 1,
            _ => {}
        }
        let status = if rejection.is_some() {
            "rejected"
        } else if obstacles.is_some() {
            "accepted"
        } else {
            "unreviewed"
        };
        let colour = if rejection.is_some() {
            [255, 80, 90]
        } else if obstacles.is_some() {
            [100, 255, 90]
        } else if nearby.is_empty() {
            [255, 210, 0]
        } else {
            [0, 230, 255]
        };
        ring(
            &mut overlay,
            p.x as f64,
            p.y as f64,
            p.radius / scale[0],
            p.radius / scale[1],
            colour,
        );
        if rejection.is_some() {
            let (x, y) = (p.x as i64, p.y as i64);
            for d in -6_i64..=6 {
                for sign in [-1_i64, 1] {
                    let (xx, yy) = (x + d, y + sign * d);
                    if xx >= 0
                        && yy >= 0
                        && xx < overlay.width() as i64
                        && yy < overlay.height() as i64
                    {
                        overlay.put_pixel(xx as u32, yy as u32, Rgb([255, 80, 90]));
                    }
                }
            }
        }
        let mut polygon: Vec<_> = (0..48)
            .map(|j| {
                let a = j as f64 / 48. * std::f64::consts::TAU;
                geo.lonlat(
                    p.x as f64 + p.radius / scale[0] * a.cos(),
                    p.y as f64 + p.radius / scale[1] * a.sin(),
                )
            })
            .collect();
        polygon.push(polygon[0]);
        features.push(json!({"type":"Feature","id":i+1,"geometry":{"type":"Polygon","coordinates":[polygon]},"properties":{"id":i+1,"status":status,"automatic_rejection":rejection,"decision_source":if obstacles.is_some(){"map_filter"}else{"unreviewed"},"method":"rgb-vegetation-texture-peaks-v2","pixel_center":[p.x,p.y],"center_lon_lat":geo.lonlat(p.x as f64,p.y as f64),"estimated_crown_radius_m":p.radius,"nearby_osm_tree_ids":nearby,"height_m":null}}));
    }
    let collection = json!({"type":"FeatureCollection","features":features});
    fs::create_dir_all(output)?;
    im.save(output.join("source.png"))?;
    overlay.save(output.join("annotated.png"))?;
    fs::write(
        output.join("candidates.geojson"),
        serde_json::to_vec_pretty(&collection)?,
    )?;
    if obstacles.is_some() {
        let kept: Vec<_> = collection["features"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|f| f["properties"]["status"] != "rejected")
            .cloned()
            .collect();
        fs::write(
            output.join("kept-candidates.geojson"),
            serde_json::to_vec_pretty(&json!({"type":"FeatureCollection","features":kept}))?,
        )?;
    }
    fs::write(
        output.join("report.json"),
        serde_json::to_vec_pretty(
            &json!({"candidate_count":candidates.len(),"map_filter_directory":city_data.map(|p|p.to_string_lossy()),"automatically_rejected_on_road":road_rejections,"automatically_rejected_on_building":building_rejections,"kept_after_map_filter":candidates.len()-road_rejections-building_rejections,"candidates_near_osm_trees":matches,"osm_trees_in_image":mapped.len(),"ground_metres_per_pixel":scale,"image":image_path.to_string_lossy(),"image_sha256":crate::location::sha(image_path)?,"export_metadata":v,"limitations":["Heuristic vegetation proposals, not a trained tree detector or measured accuracy.","Grass, shrubs, textured roofs and shadows can be false positives; dark, dry and overlapping trees can be missed.","Circles approximate visible canopy interiors, not measured crown outlines or trunk positions.","No heights or species inferred; no game assets modified."]}),
        )?,
    )?;
    let data = serde_json::to_string(&collection)?.replace('<', "\\u003c");
    let mapped_json = serde_json::to_string(&mapped)?;
    let html = include_str!("tree_review.html")
        .replace("__DATA__", &data)
        .replace("__MAPPED__", &mapped_json.replace('<', "\\u003c"))
        .replace("__SCALE__", &serde_json::to_string(&scale)?)
        .replace("__WIDTH__", &geo.width.to_string())
        .replace("__HEIGHT__", &geo.height.to_string());
    fs::write(output.join("review.html"), html)?;
    println!(
        "{} crown candidates; {road_rejections} excluded on roads, {building_rejections} on buildings; {matches} near mapped trees; {} mapped trees in image. Review: {}",
        candidates.len(),
        mapped.len(),
        output.join("review.html").display()
    );
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn detects_separate_canopies_and_ignores_neutral_roofs() {
        let mut im = RgbImage::from_pixel(160, 100, Rgb([120, 120, 120]));
        for (cx, cy, r) in [(35, 40, 13), (110, 55, 18)] {
            for y in 0..100 {
                for x in 0..160 {
                    if (x as i32 - cx).pow(2) + (y as i32 - cy).pow(2) < r * r {
                        im.put_pixel(x, y, Rgb([45, 85, 40]));
                    }
                }
            }
        }
        let p = proposals(&im, [0.25, 0.25]);
        assert_eq!(p.len(), 2);
        assert!((p[0].x as i32 - 35).abs() <= 1);
        assert!((p[1].x as i32 - 110).abs() <= 1);
        assert!(proposals(
            &RgbImage::from_pixel(80, 80, Rgb([90, 90, 90])),
            [0.25, 0.25]
        )
        .is_empty());
    }
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
    #[test]
    fn georeference_round_trip_and_review_outputs() {
        let g = Geo {
            extent: [3708893., 4185645., 3709437., 4186190.],
            width: 128,
            height: 128,
        };
        let ll = g.lonlat(31., 72.);
        let xy = g.pixel(ll[0], ll[1]);
        assert!((xy[0] - 31.).abs() < 1e-6);
        assert!((xy[1] - 72.).abs() < 1e-6);
        let t = tempfile::tempdir().unwrap();
        let image = t.path().join("image.png");
        let export = t.path().join("export.json");
        let output = t.path().join("review");
        let mut im = RgbImage::from_pixel(128, 128, Rgb([100, 100, 100]));
        for y in 50..70 {
            for x in 50..70 {
                im.put_pixel(x, y, Rgb([40, 85, 40]));
            }
        }
        im.save(&image).unwrap();
        fs::write(&export,json!({"width":128,"height":128,"extent":{"xmin":3708893.,"ymin":4185645.,"xmax":3709000.,"ymax":4185752.,"spatialReference":{"wkid":3857}}}).to_string()).unwrap();
        run(&image, &export, &output, None, None).unwrap();
        assert!(output.join("annotated.png").is_file());
        let html = fs::read_to_string(output.join("review.html")).unwrap();
        assert!(!html.contains("__DATA__"));
        let report: Value =
            serde_json::from_slice(&fs::read(output.join("report.json")).unwrap()).unwrap();
        assert_eq!(report["candidate_count"], 1);
        let candidates: Value =
            serde_json::from_slice(&fs::read(output.join("candidates.geojson")).unwrap()).unwrap();
        let ring = candidates["features"][0]["geometry"]["coordinates"][0]
            .as_array()
            .unwrap();
        assert_eq!(ring.first(), ring.last());
        let centre = candidates["features"][0]["properties"]["center_lon_lat"]
            .as_array()
            .unwrap();
        let osm = t.path().join("trees.osm");
        fs::write(&osm, format!(r#"<osm><node id="5" lon="{}" lat="{}"><tag k="natural" v="tree"/></node><node id="6" lon="0" lat="0"/></osm>"#, centre[0],centre[1])).unwrap();
        run(&image, &export, &output, Some(&osm), None).unwrap();
        let report: Value =
            serde_json::from_slice(&fs::read(output.join("report.json")).unwrap()).unwrap();
        assert_eq!(report["osm_trees_in_image"], 1);
        assert_eq!(report["candidates_near_osm_trees"], 1);
        fs::write(
            &export,
            r#"{"extent":{"xmin":0,"ymin":0,"xmax":1,"ymax":1,"spatialReference":{"wkid":4326}}}"#,
        )
        .unwrap();
        assert!(run(&image, &export, &output, None, None).is_err());
    }
}
