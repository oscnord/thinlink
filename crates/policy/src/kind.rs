use serde::{Deserialize, Serialize};

/// What the on-drone detector looks for: the COCO classes worth telling the pilot about.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Kind {
    Person,
    Bicycle,
    Car,
    Motorcycle,
    Bus,
    Truck,
    Boat,
}

impl Kind {
    pub const ALL: [Self; 7] = [
        Self::Person,
        Self::Bicycle,
        Self::Car,
        Self::Motorcycle,
        Self::Bus,
        Self::Truck,
        Self::Boat,
    ];

    /// The kind's COCO class index.
    pub const fn coco_class(self) -> usize {
        match self {
            Self::Person => 0,
            Self::Bicycle => 1,
            Self::Car => 2,
            Self::Motorcycle => 3,
            Self::Bus => 5,
            Self::Truck => 7,
            Self::Boat => 8,
        }
    }

    /// The word used in the config, the sightings and the pilot's messages.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Person => "person",
            Self::Bicycle => "bicycle",
            Self::Car => "car",
            Self::Motorcycle => "motorcycle",
            Self::Bus => "bus",
            Self::Truck => "truck",
            Self::Boat => "boat",
        }
    }

    /// The one letter drawn at the corner of the kind's box. A bus is U, since B is the bicycle.
    pub const fn letter(self) -> char {
        match self {
            Self::Person => 'P',
            Self::Bicycle => 'B',
            Self::Car => 'C',
            Self::Motorcycle => 'M',
            Self::Bus => 'U',
            Self::Truck => 'T',
            Self::Boat => 'S',
        }
    }

    /// The kind's outline colour in Y, U and V. Chosen to differ in luma as well as in hue, since
    /// a low bitrate keeps luma far better than chroma: amber (person, as ever), green, cyan,
    /// magenta, red, white and deep blue.
    pub const fn yuv(self) -> [u8; 3] {
        match self {
            Self::Person => [210, 16, 146],
            Self::Bicycle => [123, 96, 50],
            Self::Car => [152, 176, 29],
            Self::Motorcycle => [106, 202, 222],
            Self::Bus => [93, 98, 216],
            Self::Truck => [235, 128, 128],
            Self::Boat => [79, 218, 101],
        }
    }

    /// The letter as 7 rows of 5 pixels, the leftmost in bit 4.
    pub const fn glyph(self) -> [u8; 7] {
        match self {
            Self::Person => [
                0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000,
            ],
            Self::Bicycle => [
                0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110,
            ],
            Self::Car => [
                0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110,
            ],
            Self::Motorcycle => [
                0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001,
            ],
            Self::Bus => [
                0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110,
            ],
            Self::Truck => [
                0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100,
            ],
            Self::Boat => [
                0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110,
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_kind_has_its_own_class_name_letter_and_colour() {
        let letters: Vec<char> = Kind::ALL.iter().map(|k| k.letter()).collect();
        assert_eq!(letters, ['P', 'B', 'C', 'M', 'U', 'T', 'S']);
        let classes: Vec<usize> = Kind::ALL.iter().map(|k| k.coco_class()).collect();
        assert_eq!(classes, [0, 1, 2, 3, 5, 7, 8]);
        for (i, a) in Kind::ALL.iter().enumerate() {
            for b in &Kind::ALL[i + 1..] {
                assert_ne!(a.yuv(), b.yuv(), "{a:?} and {b:?}");
                assert_ne!(a.glyph(), b.glyph(), "{a:?} and {b:?}");
                assert_ne!(a.name(), b.name());
            }
        }
        assert_eq!(Kind::Person.yuv(), [210, 16, 146], "people stay amber");
    }
}
