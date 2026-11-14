//! Versioned asset formats shared by generators and filesystem adapters.
//!
//! CITY 2 keeps a strict header, optional geographic origin and optional road
//! metadata on read; imported and authored writers select explicit precision.
//! FURNITURE 3 and LANDSCAPE 1 continue accepting legacy trailing header fields.
//! Their runtime readers retain f32 validation; compiler landscape checks select
//! f64 so tree exclusion positions are not rounded into runtime coordinates.
//! Imagery-only trees still use LANDSCAPE 1; supplement/ID rules belong to asset
//! assembly. Street/elevation codecs and binary scene spools retain their existing
//! implementations. No writer here materializes a whole city mesh or output file.
pub mod barriers;
pub mod city;
pub mod furniture;
pub mod landscape;
pub mod road_rises;

use std::fmt;

/// None preserves shortest float text; Some preserves an existing fixed precision.
struct Number<T>(T, Option<usize>);
impl<T: fmt::Display> fmt::Display for Number<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.1 {
            Some(digits) => write!(f, "{:.*}", digits, self.0),
            None => self.0.fmt(f),
        }
    }
}
