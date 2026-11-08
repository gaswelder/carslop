use std::fmt::Write;
use std::{fs, path::PathBuf};

use crate::{
    location::{invalid, Result},
    world::{LandscapeItem, LocationMap, TreeItem},
};

pub fn read_landscape_file(path: PathBuf, expected_header: String) -> Result<Vec<LandscapeItem>> {
    let src = fs::read_to_string(&path)?;
    let header = src
        .lines()
        .next()
        .ok_or_else(|| invalid("missing landscape header"))?;
    if header != expected_header {
        return Err(invalid("landscape does not match city and roads"));
    }
    let mut items = Vec::new();
    for line in src.lines() {
        let f: Vec<_> = line.split_whitespace().collect();
        if f.first() == Some(&"t") && f.len() == 6 {
            items.push(LandscapeItem {
                a: f[2].parse::<f64>()?,
                b: f[3].parse::<f64>()?,
            });
        }
    }
    Ok(items)
}

pub fn read_trees_file(path: &PathBuf) -> Result<Vec<TreeItem>> {
    let src = fs::read_to_string(path)?;
    let mut items = Vec::new();
    for line in src.lines().skip(1) {
        let f: Vec<_> = line.split_whitespace().collect();
        if f.len() == 6 && f[0] == "t" {
            items.push(TreeItem {
                a: f[2].parse()?,
                b: f[3].parse()?,
                radius: f[5].parse()?,
                height: f[4].parse()?,
            })
        }
    }
    Ok(items)
}

pub fn write_city_txt(location_map: &LocationMap, path_city_txt: &PathBuf) -> Result<u32> {
    //
    //
    //
    let mut text917 = format!(
        "CITY 2\norigin {} {}\nbounds {:.3} {:.3} {:.3} {:.3}\n",
        location_map.origin[0],
        location_map.origin[1],
        location_map.bounds[0],
        location_map.bounds[1],
        location_map.bounds[2],
        location_map.bounds[3]
    );

    //
    // write all nodes
    //
    for p in &location_map.nodes {
        writeln!(text917, "n {:.3} {:.3}", p[0], p[1])?;
    }

    //
    // write all roads
    //
    for r in &location_map.roads {
        writeln!(
            text917,
            "r {} {} {:.2} {} {} {} {}",
            r.ends[0], r.ends[1], r.width, r.direction, r.way, r.kind, r.lanes
        )?;
    }

    //
    // write all buildings
    //
    for b in &location_map.buildings {
        write!(
            text917,
            "b {} {:.2} {} {}",
            b.id,
            b.height,
            u8::from(b.tagged),
            b.points.len()
        )?;
        for p in &b.points {
            write!(text917, " {:.3} {:.3}", p[0], p[1])?;
        }
        text917.push('\n');
    }
    fs::write(&path_city_txt, &text917)?;
    let checksum = crate::street_surface::crc32(&text917);
    Ok(checksum)
}
