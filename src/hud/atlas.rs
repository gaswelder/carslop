use crate::font;
use std::cmp::Reverse;
use std::sync::OnceLock;

const WIDTH: usize = 4096;
const CAP_HEIGHT: f32 = 48.0;
const PADDING: usize = 4;

pub fn get() -> &'static Atlas {
    static ATLAS: OnceLock<Atlas> = OnceLock::new();
    ATLAS.get_or_init(Atlas::new)
}

pub struct Atlas {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<u8>,
    pub labels: Font,
    pub digits: Font,
    pub regular: Font,
}

pub struct Sprite {
    pub bounds: [[f32; 2]; 2],
    pub uv: [[f32; 2]; 2],
}

pub struct Font {
    metrics: &'static font::Font,
    glyphs: Vec<Glyph>,
}

#[derive(Default)]
struct Glyph {
    advance: f32,
    bounds: [[f32; 2]; 2],
    pixels: [[usize; 2]; 2],
}

struct Packing {
    pixels: Vec<u8>,
    x: usize,
    y: usize,
    row_height: usize,
}

impl Packing {
    fn font(&mut self, bytes: &[u8], metrics: &'static font::Font) -> Font {
        let rasterizer = fontdue::Font::from_bytes(bytes, fontdue::FontSettings::default())
            .expect("valid bundled Noto font");
        let size = CAP_HEIGHT * 64.0 / rasterizer.metrics('H', 64.0).bounds.height;
        let unit = 7.0 / CAP_HEIGHT;
        let mut order: Vec<_> = (0..rasterizer.glyph_count()).collect();
        order.sort_by_cached_key(|id| Reverse(rasterizer.metrics_indexed(*id, size).height));
        let mut glyphs: Vec<_> = order.iter().map(|_| Glyph::default()).collect();
        for id in order {
            let (glyph, bitmap) = rasterizer.rasterize_indexed(id, size);
            if self.x + glyph.width + PADDING > WIDTH {
                self.x = PADDING;
                self.y += self.row_height + PADDING;
                self.row_height = 0;
            }
            let right = self.x + glyph.width;
            let bottom = self.y + glyph.height;
            self.pixels
                .resize(self.pixels.len().max((bottom + PADDING) * WIDTH), 0);
            for row in 0..glyph.height {
                let start = (self.y + row) * WIDTH + self.x;
                self.pixels[start..start + glyph.width]
                    .copy_from_slice(&bitmap[row * glyph.width..(row + 1) * glyph.width]);
            }
            let sprite = Glyph {
                advance: glyph.advance_width * unit,
                bounds: [
                    [
                        glyph.xmin as f32 * unit,
                        7.0 - (glyph.ymin as f32 + glyph.height as f32) * unit,
                    ],
                    [
                        (glyph.xmin as f32 + glyph.width as f32) * unit,
                        7.0 - glyph.ymin as f32 * unit,
                    ],
                ],
                pixels: [[self.x, self.y], [right, bottom]],
            };
            self.x = right + PADDING;
            self.row_height = self.row_height.max(glyph.height);
            glyphs[usize::from(id)] = sprite;
        }
        Font { metrics, glyphs }
    }
}

impl Atlas {
    fn new() -> Self {
        let mut packing = Packing {
            pixels: Vec::new(),
            x: PADDING,
            y: PADDING,
            row_height: 0,
        };
        let labels = packing.font(font::LABEL_BYTES, font::labels());
        let digits = packing.font(font::DIGIT_BYTES, font::digits());
        let regular = packing.font(font::REGULAR_BYTES, font::regular());
        // Keep glyph boundaries aligned through both mip levels.
        let height = (packing.pixels.len() / WIDTH).div_ceil(4) * 4;
        packing.pixels.resize(WIDTH * height, 0);
        Self {
            width: WIDTH,
            height,
            pixels: packing.pixels,
            labels,
            digits,
            regular,
        }
    }
}

impl Font {
    pub fn width(&self, value: &str) -> f32 {
        self.metrics.width(value)
    }

    /// Sprite coordinates use the same seven-unit cap height as world outlines.
    pub fn draw(&self, value: &str, mut quad: impl FnMut(Sprite)) {
        let atlas = get();
        let mut x = 0.0;
        for ch in value.chars() {
            let glyph = &self.glyphs[usize::from(self.metrics.glyph_index(ch))];
            if glyph.pixels[0][0] != glyph.pixels[1][0] && glyph.pixels[0][1] != glyph.pixels[1][1]
            {
                quad(Sprite {
                    bounds: glyph.bounds.map(|p| [x + p[0], p[1]]),
                    uv: glyph.pixels.map(|p| {
                        [
                            p[0] as f32 / atlas.width as f32,
                            p[1] as f32 / atlas.height as f32,
                        ]
                    }),
                });
            }
            x += glyph.advance;
        }
    }
}

pub struct Texture {
    id: u32,
}

impl Texture {
    pub fn new() -> Self {
        let atlas = get();
        let mut id = 0;
        unsafe {
            let mut limit = 0;
            gl::GetIntegerv(gl::MAX_TEXTURE_SIZE, &mut limit);
            assert!(
                atlas.width <= limit as usize && atlas.height <= limit as usize,
                "font atlas exceeds the GPU texture size limit"
            );
            let mut binding = 0;
            gl::GetIntegerv(gl::TEXTURE_BINDING_2D, &mut binding);
            gl::GenTextures(1, &mut id);
            gl::BindTexture(gl::TEXTURE_2D, id);
            gl::TexImage2D(
                gl::TEXTURE_2D,
                0,
                gl::R8 as i32,
                atlas.width as i32,
                atlas.height as i32,
                0,
                gl::RED,
                gl::UNSIGNED_BYTE,
                atlas.pixels.as_ptr().cast(),
            );
            gl::TexParameteri(
                gl::TEXTURE_2D,
                gl::TEXTURE_MIN_FILTER,
                gl::LINEAR_MIPMAP_LINEAR as i32,
            );
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::LINEAR as i32);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::CLAMP_TO_EDGE as i32);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::CLAMP_TO_EDGE as i32);
            // Four empty pixels between glyphs keep the two mip levels isolated.
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAX_LEVEL, 2);
            gl::GenerateMipmap(gl::TEXTURE_2D);
            gl::BindTexture(gl::TEXTURE_2D, binding as u32);
        }
        Self { id }
    }

    pub fn bind(&self) {
        unsafe {
            gl::BindTexture(gl::TEXTURE_2D, self.id);
        }
    }
}

impl Drop for Texture {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteTextures(1, &self.id);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_weights_pack_greek_latin_and_spaces_without_overlap() {
        let atlas = get();
        assert_eq!(atlas.pixels.len(), atlas.width * atlas.height);
        for font in [&atlas.labels, &atlas.digits] {
            let mut count = 0;
            font.draw("A Ωά 09", |sprite| {
                count += 1;
                for p in sprite.uv {
                    assert!(p.iter().all(|n| (0.0..=1.0).contains(n)));
                }
                assert!(sprite.bounds[1][0] > sprite.bounds[0][0]);
                assert!(sprite.bounds[1][1] > sprite.bounds[0][1]);
            });
            assert_eq!(count, 5);
            for glyph in &font.glyphs {
                let [left, top] = glyph.pixels[0];
                let [right, bottom] = glyph.pixels[1];
                assert!(right < atlas.width && bottom < atlas.height);
                assert_eq!(atlas.pixels[top * atlas.width + left - 1], 0);
                assert_eq!(atlas.pixels[top * atlas.width + right], 0);
                assert!(atlas.pixels
                    [(top - 1) * atlas.width + left..(top - 1) * atlas.width + right]
                    .iter()
                    .all(|n| *n == 0));
            }
        }
    }
}
