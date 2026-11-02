//! The test card of `lab check regional`: a 1280x720 picture drawn the same way every time, on a
//! plain background with a mild texture. Inside the tracked box: a landing-pad mark and a
//! person-sized figure. Outside it: another figure, thin vertical lines 1, 2 and 4 px wide at two
//! contrasts, and 8 px blocks at two contrasts. The vehicle plays it as a one-frame y4m clip
//! (`y4m`) and the check renders its reference from the same bytes (`render`).

pub const WIDTH: usize = 1280;
pub const HEIGHT: usize = 720;
const FPS: u32 = 15;
const BACKGROUND: u8 = 104;
const DARK: u8 = 30;
const LIGHT: u8 = 210;
const LOW_DARK: u8 = 85;
const LOW_LIGHT: u8 = 135;
/// Line widths, repeated along each row of lines.
pub const LINE_WIDTHS: [usize; 3] = [1, 2, 4];
const BLOCK: usize = 8;

struct Canvas {
    luma: Vec<u8>,
}

impl Canvas {
    fn new() -> Self {
        let luma = (0..WIDTH * HEIGHT)
            .map(|i| BACKGROUND + texture(i % WIDTH / 4, i / WIDTH / 4))
            .collect();
        Self { luma }
    }

    fn rect(&mut self, x0: usize, y0: usize, x1: usize, y1: usize, value: u8) {
        for y in y0..y1.min(HEIGHT) {
            if let Some(row) = self.luma.get_mut(y * WIDTH..(y + 1) * WIDTH)
                && let Some(span) = row.get_mut(x0..x1.min(WIDTH))
            {
                span.fill(value);
            }
        }
    }

    fn disc(&mut self, cx: usize, cy: usize, radius: usize, value: u8) {
        for y in cy.saturating_sub(radius)..(cy + radius + 1).min(HEIGHT) {
            for x in cx.saturating_sub(radius)..(cx + radius + 1).min(WIDTH) {
                let (dx, dy) = (x.abs_diff(cx), y.abs_diff(cy));
                if dx * dx + dy * dy <= radius * radius
                    && let Some(pixel) = self.luma.get_mut(y * WIDTH + x)
                {
                    *pixel = value;
                }
            }
        }
    }

    /// A person about 110 px tall standing with its feet at (`x`, `feet`).
    fn figure(&mut self, x: usize, feet: usize, clothes: u8) {
        self.disc(x, feet - 100, 10, 150);
        self.rect(x - 14, feet - 90, x + 14, feet - 45, clothes);
        self.rect(x - 12, feet - 45, x - 2, feet, DARK);
        self.rect(x + 2, feet - 45, x + 12, feet, DARK);
    }

    /// Vertical lines 60 px tall in `value`, widths cycling through `LINE_WIDTHS`, 40 px apart.
    fn lines(&mut self, x: usize, y: usize, value: u8) {
        for k in 0..8 {
            let width = LINE_WIDTHS.get(k % LINE_WIDTHS.len()).copied().unwrap_or(1);
            self.rect(x + k * 40, y, x + k * 40 + width, y + 60, value);
        }
    }

    /// A checkerboard of 8 px blocks, 320 x 56 px, in two values.
    fn blocks(&mut self, x: usize, y: usize, dark: u8, light: u8) {
        for by in 0..56 / BLOCK {
            for bx in 0..320 / BLOCK {
                let value = if (bx + by) % 2 == 0 { dark } else { light };
                let (x0, y0) = (x + bx * BLOCK, y + by * BLOCK);
                self.rect(x0, y0, x0 + BLOCK, y0 + BLOCK, value);
            }
        }
    }
}

/// 0 to 11, the same for a 4x4 cell every time.
fn texture(cx: usize, cy: usize) -> u8 {
    let hash = cx.wrapping_mul(73_856_093) ^ cy.wrapping_mul(19_349_663);
    u8::try_from(hash % 12).unwrap_or(0)
}

/// The card as one I420 frame: luma, then the two chroma planes, which are neutral.
pub fn render() -> Vec<u8> {
    let mut card = Canvas::new();
    // Inside the box (35-65 % of the width, 30-70 % of the height): the landing pad and a figure.
    card.disc(700, 340, 90, LIGHT);
    card.disc(700, 340, 78, BACKGROUND);
    card.rect(670, 300, 682, 380, LIGHT);
    card.rect(718, 300, 730, 380, LIGHT);
    card.rect(682, 334, 718, 346, LIGHT);
    card.figure(540, 480, 60);
    // Outside it.
    card.figure(200, 560, 170);
    card.lines(60, 160, DARK);
    card.lines(60, 240, LIGHT);
    card.blocks(900, 160, DARK, LIGHT);
    card.blocks(900, 240, LOW_DARK, LOW_LIGHT);
    let mut frame = card.luma;
    frame.resize(WIDTH * HEIGHT * 3 / 2, 128);
    frame
}

/// The clip the vehicle plays: the card as one y4m frame, which its `still` source holds.
pub fn y4m() -> Vec<u8> {
    let mut clip = format!("YUV4MPEG2 W{WIDTH} H{HEIGHT} F{FPS}:1 Ip A1:1 C420jpeg\n").into_bytes();
    let frame = render();
    clip.extend_from_slice(b"FRAME\n");
    clip.extend_from_slice(&frame);
    clip
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The widths of the dark runs along one row through the first group of lines.
    fn run_widths(luma: &[u8], y: usize) -> Vec<usize> {
        let row = &luma[y * WIDTH..(y + 1) * WIDTH];
        let mut widths = Vec::new();
        let mut run = 0;
        for &p in &row[..400] {
            if p == DARK {
                run += 1;
            } else if run > 0 {
                widths.push(run);
                run = 0;
            }
        }
        widths
    }

    #[test]
    fn the_card_has_lines_of_every_width_and_fits_the_clip() {
        let frame = render();
        assert_eq!(frame.len(), WIDTH * HEIGHT * 3 / 2);
        assert_eq!(&run_widths(&frame, 180)[..3], &LINE_WIDTHS);
        assert!(y4m().len() < 12 * 1024 * 1024 / 4);
    }
}
