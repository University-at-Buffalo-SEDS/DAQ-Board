#![no_std]

/// Incremental IEEE CRC-32 with the subset of crc32fast's API used by SEDSNet.
///
/// A single 1 KiB table avoids eight bitwise iterations per byte while staying
/// much smaller than crc32fast's slicing-by-16 table on 512 KiB STM32 targets.
const fn crc_table() -> [u32; 256] {
    let mut table = [0; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = i as u32;
        let mut bit = 0;
        while bit < 8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & 0u32.wrapping_sub(crc & 1));
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}
static CRC_TABLE: [u32; 256] = crc_table();

#[derive(Clone, Copy, Debug)]
pub struct Hasher {
    state: u32,
}

impl Hasher {
    #[inline]
    pub const fn new() -> Self {
        Self { state: u32::MAX }
    }

    pub fn update(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.state = (self.state >> 8) ^ CRC_TABLE[((self.state as u8) ^ byte) as usize];
        }
    }

    #[inline]
    pub const fn finalize(self) -> u32 {
        !self.state
    }
}

impl Default for Hasher {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::Hasher;

    #[test]
    fn lookup_matches_bitwise_crc_across_lengths_and_splits() {
        let data: [u8; 4096] = core::array::from_fn(|i| (i * 73 + i / 251) as u8);
        for len in [0, 1, 2, 14, 35, 64, 128, 255, 256, 1024, 4096] {
            let mut reference = u32::MAX;
            for &byte in &data[..len] {
                reference ^= u32::from(byte);
                for _ in 0..8 {
                    reference = (reference >> 1) ^ (0xEDB8_8320 & 0u32.wrapping_sub(reference & 1));
                }
            }
            for split in [0, len / 2, len] {
                let mut h = Hasher::new();
                h.update(&data[..split]);
                h.update(&data[split..len]);
                assert_eq!(h.finalize(), !reference, "length {len}, split {split}");
            }
        }
    }

    #[test]
    fn ieee_check_vector_and_incremental_updates() {
        let mut one = Hasher::new();
        one.update(b"123456789");
        assert_eq!(one.finalize(), 0xCBF4_3926);

        let mut split = Hasher::new();
        split.update(b"1234");
        split.update(b"56789");
        assert_eq!(split.finalize(), 0xCBF4_3926);
    }
}
