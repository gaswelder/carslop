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
    #[cfg(test)]
    pub fn load(text: &str, city: &str, streets: &str) -> Result<Self, String> {
        Self::load_with_checksums(
            text,
            crate::street_surface::SourceChecksums::new(city, streets),
        )
    }
    #[cfg(test)]
    fn load_with_checksums(
        text: &str,
        checksums: crate::street_surface::SourceChecksums,
    ) -> Result<Self, String> {
        let folder = tempfile::tempdir().map_err(|e| e.to_string())?;
        std::fs::write(folder.path().join("landscape.txt"), text).map_err(|e| e.to_string())?;
        crate::city_assets::CityAsset::new(folder.path().to_path_buf()).get_landscape(checksums)
    }

    pub(crate) fn index_ground(&mut self) {
        self.grid.clear();
        for (i, t) in self.ground.iter().enumerate() {
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
                    self.grid.entry((x, z)).or_default().push(i);
                }
            }
        }
    }
    #[cfg(test)]
    pub fn append_imagery(&mut self, text: &str, city: &str, streets: &str) -> Result<(), String> {
        self.append_imagery_with_checksums(
            text,
            crate::street_surface::SourceChecksums::new(city, streets),
        )
    }
    #[cfg(test)]
    fn append_imagery_with_checksums(
        &mut self,
        text: &str,
        checksums: crate::street_surface::SourceChecksums,
    ) -> Result<(), String> {
        let folder = tempfile::tempdir().map_err(|e| e.to_string())?;
        let header = format!(
            "LANDSCAPE 1 {:08x} {:08x}\n",
            checksums.city, checksums.streets
        );
        std::fs::write(folder.path().join("landscape.txt"), header).map_err(|e| e.to_string())?;
        std::fs::write(folder.path().join("imagery-trees.txt"), text).map_err(|e| e.to_string())?;
        let extra = crate::city_assets::CityAsset::new(folder.path().to_path_buf())
            .get_landscape(checksums)?;
        self.append_imagery_trees(extra.trees)
    }

    pub(crate) fn append_imagery_trees(&mut self, trees: Vec<Tree>) -> Result<(), String> {
        let mut ids: std::collections::HashSet<_> = self.trees.iter().map(|t| t.id).collect();
        if trees
            .iter()
            .any(|t| t.id < (1_u64 << 63) || !ids.insert(t.id))
        {
            return Err("invalid or duplicate imagery tree id".into());
        }
        self.trees.extend(trees);
        Ok(())
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
    fn shared_checksums_accept_matching_assets_and_reject_stale_sources() {
        use crate::street_surface::SourceChecksums;
        let checksums = SourceChecksums::new("city", "streets");
        let text = format!(
            "LANDSCAPE 1 {:08x} {:08x}\nt 1 0 0 5 1\n",
            checksums.city, checksums.streets
        );
        let mut landscape = Landscape::load_with_checksums(&text, checksums).unwrap();
        assert_eq!(landscape.trees.len(), 1);
        let supplement = text.replace("t 1 ", "t 9223372036854775808 ");
        landscape
            .append_imagery_with_checksums(&supplement, checksums)
            .unwrap();
        assert_eq!(landscape.trees.len(), 2);
        for stale in [
            SourceChecksums::new("changed city", "streets"),
            SourceChecksums::new("city", "changed streets"),
        ] {
            assert!(Landscape::load_with_checksums(&text, stale).is_err());
            assert!(landscape
                .append_imagery_with_checksums(&supplement, stale)
                .is_err());
        }
        assert_eq!(landscape.trees.len(), 2);
    }

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
