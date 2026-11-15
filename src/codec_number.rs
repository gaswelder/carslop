//! Numeric formatting shared by the text asset codecs.
use std::fmt;

/// None preserves shortest float text; Some preserves an existing fixed precision.
pub struct Number<T> {
    value: T,
    precision: Option<usize>,
}

pub fn new<T>(value: T, precision: Option<usize>) -> Number<T> {
    Number::<T> { value, precision }
}

impl<T: fmt::Display> fmt::Display for Number<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.precision {
            Some(digits) => write!(f, "{:.*}", digits, self.value),
            None => self.value.fmt(f),
        }
    }
}
