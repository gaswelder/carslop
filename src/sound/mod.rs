const SAMPLE_RATE: f32 = 48_000.0;

pub mod car;
mod engine_sample;
mod impacts;

mod spatial;
mod synth;
mod tires;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod test_support;
