use geo::MultiPolygon;

pub type P = [f64; 2];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox {
    pub south: f64,
    pub west: f64,
    pub north: f64,
    pub east: f64,
}

impl BoundingBox {
    pub fn is_valid(&self) -> bool {
        self.to_array().iter().all(|v| v.is_finite())
            && self.south < self.north
            && self.west < self.east
            && self.south >= -90.0
            && self.north <= 90.0
            && self.west >= -180.0
            && self.east <= 180.0
    }

    /// Apply the imagery pipeline's conservative Web Mercator latitude limit.
    pub fn is_valid_for_imagery(&self) -> bool {
        self.is_valid() && self.south >= -85.0 && self.north <= 85.0
    }

    /// Ordering used by the existing OSM extraction and compiler APIs.
    pub fn to_array(self) -> [f64; 4] {
        [self.south, self.west, self.north, self.east]
    }
}

pub struct Boundary {
    pub relation: i64,
    pub geometry: MultiPolygon<f64>,
}

impl Boundary {
    // Returns the boundary's width and height in meters.
    pub fn size(&self) -> (f64, f64) {
        let width = (self.bbox().east - self.bbox().west).to_radians()
            * 6371000.
            * ((self.bbox().south + self.bbox().north) / 2.)
                .to_radians()
                .cos();
        let height = (self.bbox().north - self.bbox().south).to_radians() * 6371000.;

        return (width, height);
    }
}

// what's stored in landscape.txt
pub struct LandscapeItem {
    pub a: f64,
    pub b: f64,
}

// what's stored in trees.txt
pub struct TreeItem {
    pub a: f64,
    pub b: f64,
    pub radius: f64,
    pub height: f64,
}

pub struct Hgt {
    pub size: usize,
    pub rows: usize,
    pub cols: usize,
    pub south: f64,
    pub west: f64,
    pub samples: Vec<i16>,
}

#[derive(Clone)]
pub struct Road {
    pub ends: [usize; 2],
    pub width: f64,
    pub direction: i8,
    pub way: i64,
    pub kind: usize,
    pub lanes: usize,
}

pub struct Building {
    pub id: i64,
    pub height: f64,
    pub tagged: bool,
    pub points: Vec<P>,
}

pub struct LocationMap {
    pub boundary: Option<MultiPolygon<f64>>,
    pub origin: P,
    pub bounds: [f64; 4],
    pub nodes: Vec<P>,
    pub roads: Vec<Road>,
    pub buildings: Vec<Building>,
}
impl LocationMap {
    pub fn metres(&self, p: P) -> P {
        [
            (p[1] - self.origin[1]).to_radians() * 6371000.0 * self.origin[0].to_radians().cos(),
            -(p[0] - self.origin[0]).to_radians() * 6371000.0,
        ]
    }
    pub fn contains(&self, p: P) -> bool {
        p[0] >= self.bounds[0]
            && p[0] <= self.bounds[2]
            && p[1] >= self.bounds[1]
            && p[1] <= self.bounds[3]
            && self.boundary.as_ref().is_none_or(|b| {
                use geo::Intersects;
                b.intersects(&geo::Point::new(p[0], p[1]))
            })
    }
}
