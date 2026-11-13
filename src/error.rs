//! Shared error helpers.
use std::io;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub fn invalid(message: impl Into<String>) -> Box<dyn std::error::Error> {
    io::Error::new(io::ErrorKind::InvalidInput, message.into()).into()
}
