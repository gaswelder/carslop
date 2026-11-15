//! Shared outline lettering for UI, loading screens, and world signs.
use lyon_tessellation::math;
use lyon_tessellation::path;
use lyon_tessellation::{BuffersBuilder, FillOptions, FillTessellator, FillVertex, VertexBuffers};
use std::sync::OnceLock;

pub const LABEL_BYTES: &[u8] = include_bytes!("../data/fonts/noto-sans/NotoSans-SemiBold.ttf");
pub const DIGIT_BYTES: &[u8] = include_bytes!("../data/fonts/noto-sans/NotoSans-Bold.ttf");
pub const REGULAR_BYTES: &[u8] = include_bytes!("../data/fonts/noto-sans/NotoSans-Regular.ttf");

pub fn regular() -> &'static Font {
    static FONT: OnceLock<Font> = OnceLock::new();
    FONT.get_or_init(|| new(REGULAR_BYTES))
}

pub fn labels() -> &'static Font {
    static FONT: OnceLock<Font> = OnceLock::new();
    FONT.get_or_init(|| new(LABEL_BYTES))
}

pub fn digits() -> &'static Font {
    static FONT: OnceLock<Font> = OnceLock::new();
    FONT.get_or_init(|| new(DIGIT_BYTES))
}

struct Glyph {
    advance: f32,
    triangles: Vec<[[f32; 2]; 3]>,
}

pub struct Font {
    face: ttf_parser::Face<'static>,
    scale: f32,
    glyphs: Vec<OnceLock<Glyph>>,
}

fn new(bytes: &'static [u8]) -> Font {
    let face = ttf_parser::Face::parse(bytes, 0).expect("valid bundled font");
    // Match the old seven-unit cap height so existing HUD sizes stay useful.
    let cap = face
        .glyph_bounding_box(face.glyph_index('H').unwrap())
        .unwrap();
    let scale = 7.0 / f32::from(cap.y_max);
    let glyphs = (0..face.number_of_glyphs())
        .map(|_| OnceLock::new())
        .collect();
    Font {
        face,
        scale,
        glyphs,
    }
}

fn character(font: &Font, ch: char) -> &Glyph {
    let id = ttf_parser::GlyphId(glyph_index(font, ch));
    font.glyphs[usize::from(id.0)].get_or_init(|| {
        let mut outline = Outline {
            builder: path::Path::builder(),
            open: false,
        };
        font.face.outline_glyph(id, &mut outline);
        finish_contour(&mut outline);
        let path = outline.builder.build();
        let mut mesh: VertexBuffers<[f32; 2], u32> = VertexBuffers::new();
        FillTessellator::new()
            .tessellate_path(
                &path,
                &FillOptions::default().with_tolerance(0.25),
                &mut BuffersBuilder::new(&mut mesh, |vertex: FillVertex| {
                    let p = vertex.position();
                    [p.x * font.scale, 7.0 - p.y * font.scale]
                }),
            )
            .expect("valid bundled font outline");
        Glyph {
            advance: f32::from(font.face.glyph_hor_advance(id).unwrap_or(0)) * font.scale,
            triangles: mesh
                .indices
                .chunks_exact(3)
                .map(|indices| {
                    let mut points = [
                        mesh.vertices[indices[0] as usize],
                        mesh.vertices[indices[1] as usize],
                        mesh.vertices[indices[2] as usize],
                    ];
                    let [a, b, c] = points;
                    if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0.0 {
                        points.swap(1, 2);
                    }
                    points
                })
                .collect(),
        }
    })
}

pub fn width(font: &Font, value: &str) -> f32 {
    // Measuring a new name must not tessellate glyphs on the render thread.
    value
        .chars()
        .map(|ch| {
            f32::from(
                font.face
                    .glyph_hor_advance(ttf_parser::GlyphId(glyph_index(font, ch)))
                    .unwrap_or(0),
            ) * font.scale
        })
        .sum()
}

pub fn glyph_index(font: &Font, ch: char) -> u16 {
    font.face
        .glyph_index(ch)
        .unwrap_or(ttf_parser::GlyphId(0))
        .0
}

/// Emit triangles with a top origin and downward Y, facing outward on signs.
pub fn draw(font: &Font, value: &str, mut triangle: impl FnMut([[f32; 2]; 3])) {
    let mut x = 0.0;
    for ch in value.chars() {
        let glyph = character(font, ch);
        for points in &glyph.triangles {
            triangle(points.map(|p| [x + p[0], p[1]]));
        }
        x += glyph.advance;
    }
}

struct Outline {
    builder: path::path::Builder,
    open: bool,
}

fn finish_contour(outline: &mut Outline) {
    if outline.open {
        outline.builder.end(true);
        outline.open = false;
    }
}

impl ttf_parser::OutlineBuilder for Outline {
    fn move_to(&mut self, x: f32, y: f32) {
        finish_contour(self);
        self.builder.begin(math::point(x, y));
        self.open = true;
    }

    fn line_to(&mut self, x: f32, y: f32) {
        self.builder.line_to(math::point(x, y));
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        self.builder
            .quadratic_bezier_to(math::point(x1, y1), math::point(x, y));
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        self.builder
            .cubic_bezier_to(math::point(x1, y1), math::point(x2, y2), math::point(x, y));
    }

    fn close(&mut self) {
        finish_contour(self);
    }
}

#[cfg(test)]
mod tests {
    use crate::font;

    use super::*;

    #[test]
    fn bundled_weights_keep_counters_open_and_measure_proportional_text() {
        for font in [labels(), digits()] {
            for ch in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789éö".chars() {
                let glyph = font::character(font, ch);
                assert!(glyph.advance > 0.0);
                assert!(!glyph.triangles.is_empty(), "missing outline for {ch}");
                assert!(glyph
                    .triangles
                    .iter()
                    .flatten()
                    .flatten()
                    .all(|n| n.is_finite()));
            }
            let glyph = font::character(font, 'O');
            let center = [glyph.advance * 0.5, 3.5];
            for triangle in &glyph.triangles {
                let cross = |a: [f32; 2], b: [f32; 2]| {
                    (b[0] - a[0]) * (center[1] - a[1]) - (b[1] - a[1]) * (center[0] - a[0])
                };
                let edges = [
                    cross(triangle[0], triangle[1]),
                    cross(triangle[1], triangle[2]),
                    cross(triangle[2], triangle[0]),
                ];
                assert!(
                    edges.iter().any(|n| *n > 0.0) && edges.iter().any(|n| *n < 0.0),
                    "O's counter must stay unfilled"
                );
            }
            assert!(font::width(font, "WWW") > font::width(font, "III"));
            assert!(font::width(font, " ") > 0.0);
            font::draw(font, " ", |_| panic!("spaces must not draw triangles"));
        }
    }

    #[test]
    fn greek_names_are_supported_by_both_weights() {
        for font in [labels(), digits()] {
            for ch in "ΟΔΟΣ ΛΗΔΡΑΣ Λευκωσία ΆΈΉΊΌΎΏάέήίόύώϊϋΐΰ".chars()
            {
                assert!(
                    font.face.glyph_index(ch).is_some(),
                    "missing Greek character {ch}"
                );
                if !ch.is_whitespace() {
                    assert!(!font::character(font, ch).triangles.is_empty());
                }
            }
        }
    }
}
