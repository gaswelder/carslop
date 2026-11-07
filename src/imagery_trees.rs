//! Compile accepted imagery proposals into an optional, repeatable tree supplement.
use crate::location_compiler::{invalid, Result};
use serde_json::{json, Value};
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
fn number(v: &Value, key: &str) -> Result<f64> {
    v[key]
        .as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| invalid(format!("invalid candidate {key}")))
}
pub(crate) fn atomic(path: &Path, data: &[u8]) -> Result<()> {
    let mut file = tempfile::NamedTempFile::new_in(path.parent().unwrap())?;
    file.write_all(data)?;
    file.as_file().sync_all()?;
    file.persist(path)?;
    Ok(())
}
pub fn import(candidates: &Path, directory: &Path) -> Result<()> {
    let source = fs::read_to_string(directory.join("city.txt"))?;
    let city = crate::city::City::parse(&source).map_err(invalid)?;
    let landscape = fs::read_to_string(directory.join("landscape.txt"))?;
    let header = format!(
        "LANDSCAPE 1 {:08x} {:08x}",
        crate::street_surface::crc32(&source),
        checksum(&directory.join("streets.txt"))?
    );
    if landscape.lines().next() != Some(&header) {
        return Err(invalid(
            "landscape does not match map; reimport before adding imagery trees",
        ));
    }
    let collection: Value = serde_json::from_slice(&fs::read(candidates)?)?;
    if collection["type"] != "FeatureCollection" {
        return Err(invalid("expected GeoJSON FeatureCollection"));
    }
    let features = collection["features"]
        .as_array()
        .ok_or_else(|| invalid("missing candidate features"))?;
    let mut input = vec![];
    let mut excluded_status = 0;
    for f in features {
        let p = &f["properties"];
        if p["status"] != "accepted" {
            excluded_status += 1;
            continue;
        }
        let centre = p["center_lon_lat"]
            .as_array()
            .filter(|p| p.len() == 2)
            .ok_or_else(|| invalid("missing candidate longitude/latitude"))?;
        let lon = centre[0]
            .as_f64()
            .filter(|x| x.is_finite() && (-180.0..=180.).contains(x))
            .ok_or_else(|| invalid("invalid longitude"))?;
        let lat = centre[1]
            .as_f64()
            .filter(|x| x.is_finite() && (-85.0..=85.).contains(x))
            .ok_or_else(|| invalid("invalid latitude"))?;
        let radius = number(p, "estimated_crown_radius_m")?;
        if !(0.7..=20.).contains(&radius) {
            return Err(invalid("candidate crown radius outside supported range"));
        }
        input.push((lon, lat, radius, p["id"].clone()));
    }
    if input.is_empty() {
        return Err(invalid("no accepted tree candidates"));
    }
    let bounds = [
        input.iter().map(|p| p.0).fold(f64::INFINITY, f64::min),
        input.iter().map(|p| p.1).fold(f64::INFINITY, f64::min),
        input.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max),
        input.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max),
    ];
    let obstacles = crate::tree_obstacles::Obstacles::read(directory, bounds)?;
    let mut existing = vec![];
    for line in landscape.lines().skip(1) {
        let f: Vec<_> = line.split_whitespace().collect();
        if f.first() == Some(&"t") && f.len() == 6 {
            existing.push([f[2].parse::<f64>()?, f[3].parse::<f64>()?]);
        }
    }
    let mut index = rstar::RTree::bulk_load(existing);
    input.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.total_cmp(&b.0)));
    let (mut conflicts, mut duplicates, mut outside) = (0, 0, 0);
    let mut trees = vec![];
    let mut output = format!("{header}\n");
    for (lon, lat, radius, candidate_id) in input {
        let p = obstacles.metres(lon, lat);
        if p[0] < city.bounds[0][0] as f64
            || p[1] < city.bounds[0][1] as f64
            || p[0] > city.bounds[1][0] as f64
            || p[1] > city.bounds[1][1] as f64
        {
            outside += 1;
            continue;
        }
        // Validate the stored precision too: rounding must not move a trunk onto asphalt.
        let p = [
            (p[0] * 1000.).round() / 1000.,
            (p[1] * 1000.).round() / 1000.,
        ];
        if obstacles.rejection_at(p).is_some() {
            conflicts += 1;
            continue;
        }
        if index
            .nearest_neighbor(&p)
            .is_some_and(|q| (q[0] - p[0]).hypot(q[1] - p[1]) < 2.)
        {
            duplicates += 1;
            continue;
        }
        let radius = radius.clamp(0.7, 4.5);
        let height = (radius * 2.8).clamp(3.5, 14.);
        let id = (1_u64 << 63) | trees.len() as u64;
        output.push_str(&format!(
            "t {id} {:.3} {:.3} {height:.3} {radius:.3}\n",
            p[0], p[1]
        ));
        index.insert(p);
        trees.push(json!({"id":id,"candidate_id":candidate_id,"position_metres":p,"center_lon_lat":[lon,lat],"estimated_height_m":height,"estimated_radius_m":radius}));
    }
    if trees.is_empty() {
        return Err(invalid(
            "no candidates remain after map and duplicate checks",
        ));
    }
    let metadata = json!({"source":candidates.to_string_lossy(),"source_sha256":crate::location_compiler::sha(candidates)?,"count":trees.len(),"skipped_road_or_building":conflicts,"skipped_duplicates":duplicates,"skipped_outside_city":outside,"skipped_unaccepted":excluded_status,"sizes":"Artistic estimates from crown radius, not measured heights or species.","scope":"Only the supplied imagery coverage; not all Engomi.","trees":trees});
    atomic(
        &directory.join("imagery-trees.json"),
        &serde_json::to_vec_pretty(&metadata)?,
    )?;
    atomic(&directory.join("imagery-trees.txt"), output.as_bytes())?;
    println!("Added {} imagery trees; skipped {conflicts} conflicts, {duplicates} duplicates, {outside} outside map. Rebuild visuals with bake-city --location NAME.",trees.len());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn import_is_repeatable_preserves_mapped_trees_and_rechecks_conflicts() {
        let temp = tempfile::tempdir().unwrap();
        let dir = temp.path();
        let city="CITY 2\norigin 0 0\nbounds -50 -50 50 50\nn -20 0\nn 20 0\nr 0 1 8 0 1\nb 1 10 1 4 10 10 20 10 20 20 10 20\n";
        let streets = format!(
            "STREETS 1 1 {:08x}\nt 0 -20 -2 20 -2 20 2\nt 0 -20 -2 20 2 -20 2\n",
            crate::street_surface::crc32(city)
        );
        let landscape = format!(
            "LANDSCAPE 1 {:08x} {:08x}\nt 42 -10 10 5 2\n",
            crate::street_surface::crc32(city),
            crate::street_surface::crc32(&streets)
        );
        fs::write(dir.join("city.txt"), city).unwrap();
        fs::write(dir.join("streets.txt"), &streets).unwrap();
        fs::write(dir.join("landscape.txt"), &landscape).unwrap();
        let features:Vec<_>=[(0.0_f64,10.0_f64,"accepted"),(0.5,10.,"accepted"),(-10.,10.,"accepted"),(0.,0.,"accepted"),(15.,15.,"accepted"),(70.,70.,"accepted"),(4.,20.,"rejected")].iter().enumerate().map(|(i,(x,z,status))|json!({"properties":{"id":i,"status":status,"center_lon_lat":[(x/6371000_f64).to_degrees(),(-z/6371000_f64).to_degrees()],"estimated_crown_radius_m":2.}})).collect();
        let input = dir.join("candidates.geojson");
        fs::write(
            &input,
            json!({"type":"FeatureCollection","features":features}).to_string(),
        )
        .unwrap();
        import(&input, dir).unwrap();
        let first = fs::read(dir.join("imagery-trees.txt")).unwrap();
        import(&input, dir).unwrap();
        assert_eq!(first, fs::read(dir.join("imagery-trees.txt")).unwrap());
        assert_eq!(
            landscape,
            fs::read_to_string(dir.join("landscape.txt")).unwrap()
        );
        let mut merged = crate::landscape::Landscape::load(&landscape, city, &streets).unwrap();
        merged
            .append_imagery(std::str::from_utf8(&first).unwrap(), city, &streets)
            .unwrap();
        assert_eq!(merged.trees.len(), 2);
        assert_eq!(merged.trees[0].id, 42);
        assert!(merged.trees[1].id >= 1 << 63);
        assert!(merged
            .append_imagery(std::str::from_utf8(&first).unwrap(), city, &streets)
            .is_err());
        let report: Value =
            serde_json::from_slice(&fs::read(dir.join("imagery-trees.json")).unwrap()).unwrap();
        assert_eq!(report["count"], 1);
        assert_eq!(report["skipped_road_or_building"], 2);
        assert_eq!(report["skipped_duplicates"], 2);
        assert_eq!(report["skipped_outside_city"], 1);
    }
}
