use crate::geometry::XYZ;
use crate::track;
use crate::world::{Location, LocationId};

/// Authoring input for a closed circuit, discarded from runtime geometry after sampling.
pub struct Circuit {
    /// Closed spline control positions and full road widths in metres.
    pub control_points: Vec<ControlPoint>,
    /// Selects the authored coastal terrain and scenery preset.
    pub coastal: bool,
}

/// Spline control position and full road width, in metres.
#[derive(Clone, Copy, Debug)]
pub struct ControlPoint {
    pub position: XYZ,
    pub width: f32,
}

impl Circuit {
    /// Bound spline sampling and reject invalid geometry before allocating runtime data.
    pub fn validate(&self) -> Result<(), String> {
        let points = &self.control_points;
        if !(4..=4096).contains(&points.len())
            || points.iter().any(|p| {
                p.position
                    .into_iter()
                    .chain([p.width])
                    .any(|v| !v.is_finite() || v.abs() > 1_000_000.0)
            })
        {
            return Err(
                "track needs 4..4096 finite [x, y, z, width] points within one million metres"
                    .into(),
            );
        }
        if points
            .iter()
            .any(|point| !(2.0..=30.0).contains(&point.width))
        {
            return Err("point width must be 2..30 metres".into());
        }
        let mut perimeter = 0.0;
        for i in 0..points.len() {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            let distance = (a.position.x - b.position.x).hypot(a.position.z - b.position.z);
            if distance < 0.5 {
                return Err("adjacent track points must be at least 0.5 metres apart in XZ".into());
            }
            perimeter += distance;
        }
        if perimeter > 50_000.0 {
            return Err("track control perimeter exceeds 50 km".into());
        }
        Ok(())
    }

    /// Build the shared runtime world once; spline control points remain authoring input.
    pub fn build(&self, id: LocationId) -> Result<Location, String> {
        Location::prepare_authored(self, id)
    }
}

impl Location {
    pub fn build(id: LocationId, controls: &[XYZ]) -> Self {
        Self::build_points(
            id,
            &controls
                .iter()
                .map(|p| ControlPoint {
                    position: *p,
                    width: track::ROAD_WIDTH,
                })
                .collect::<Vec<_>>(),
            false,
        )
        .expect("invalid built-in route")
    }
    fn build_points(
        id: LocationId,
        controls: &[ControlPoint],
        coastal: bool,
    ) -> Result<Self, String> {
        Circuit {
            control_points: controls.to_vec(),
            coastal,
        }
        .build(id)
    }
}
