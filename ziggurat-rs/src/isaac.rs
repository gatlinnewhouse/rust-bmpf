//! ISAAC (Indirection, Shift, Accumulate, Add, and Count) PRNG
//!
//! By Bob Jenkins. Public Domain.
//! Modified by Bart Massey https://github.com/BartMassey/ziggurat
//! Ported to Rust by Gatlin Newhouse

const RAND_SIZL: usize = 8;
const RAND_SIZE: usize = 1 << RAND_SIZL; // 256

/// ISAAC random number generator context
pub struct IsaacRng {
    randcnt: usize,
    randrsl: [u32; RAND_SIZE],
    randmem: [u32; RAND_SIZE],
    randa: u32,
    randb: u32,
    randc: u32,
}

impl IsaacRng {
    /// Create a new uninitialized ISAAC context
    pub fn new() -> Self {
        Self {
            randcnt: 0,
            randrsl: [0; RAND_SIZE],
            randmem: [0; RAND_SIZE],
            randa: 0,
            randb: 0,
            randc: 0,
        }
    }

    /// Seed the random number generator
    pub fn seed(&mut self, seed: u32) {
        for i in 0..RAND_SIZE {
            self.randrsl[i] = seed;
        }
        self.init(true);
    }

    /// Initialize the generator
    /// If flag is true, use the contents of randrsl as the seed
    fn init(&mut self, flag: bool) {
        const GOLDEN_RATIO: u32 = 0x9e3779b9;

        self.randa = 0;
        self.randb = 0;
        self.randc = 0;

        let mut tmp = [GOLDEN_RATIO; 8];

        // Scramble it
        for _ in 0..4 {
            Self::mix(&mut tmp);
        }

        if flag {
            // Initialize using the contents of randrsl as the seed
            for i in (0..RAND_SIZE).step_by(8) {
                for j in 0..8 {
                    tmp[j] = tmp[j].wrapping_add(self.randrsl[i + j]);
                }
                Self::mix(&mut tmp);
                for j in 0..8 {
                    self.randmem[i + j] = tmp[j];
                }
            }

            // Do a second pass to make all of the seed affect all of randmem
            for i in (0..RAND_SIZE).step_by(8) {
                for j in 0..8 {
                    tmp[j] = tmp[j].wrapping_add(self.randmem[i + j]);
                }
                Self::mix(&mut tmp);
                for j in 0..8 {
                    self.randmem[i + j] = tmp[j];
                }
            }
        } else {
            // Fill in randmem with messy stuff
            for i in (0..RAND_SIZE).step_by(8) {
                Self::mix(&mut tmp);
                for j in 0..8 {
                    self.randmem[i + j] = tmp[j];
                }
            }
        }

        // Fill in the first set of results
        self.isaac();
    }

    /// Mix the array
    #[inline]
    fn mix(tmp: &mut [u32; 8]) {
        tmp[0] ^= tmp[1] << 11;
        tmp[3] = tmp[3].wrapping_add(tmp[0]);
        tmp[1] = tmp[1].wrapping_add(tmp[2]);
        tmp[1] ^= tmp[2] >> 2;
        tmp[4] = tmp[4].wrapping_add(tmp[1]);
        tmp[2] = tmp[2].wrapping_add(tmp[3]);
        tmp[2] ^= tmp[3] << 8;
        tmp[5] = tmp[5].wrapping_add(tmp[2]);
        tmp[3] = tmp[3].wrapping_add(tmp[4]);
        tmp[3] ^= tmp[4] >> 16;
        tmp[6] = tmp[6].wrapping_add(tmp[3]);
        tmp[4] = tmp[4].wrapping_add(tmp[5]);
        tmp[4] ^= tmp[5] << 10;
        tmp[7] = tmp[7].wrapping_add(tmp[4]);
        tmp[5] = tmp[5].wrapping_add(tmp[6]);
        tmp[5] ^= tmp[6] >> 4;
        tmp[0] = tmp[0].wrapping_add(tmp[5]);
        tmp[6] = tmp[6].wrapping_add(tmp[7]);
        tmp[6] ^= tmp[7] << 8;
        tmp[1] = tmp[1].wrapping_add(tmp[6]);
        tmp[7] = tmp[7].wrapping_add(tmp[0]);
        tmp[7] ^= tmp[0] >> 9;
        tmp[2] = tmp[2].wrapping_add(tmp[7]);
        tmp[0] = tmp[0].wrapping_add(tmp[1]);
    }

    /// Index into randmem
    #[inline]
    fn ind(&self, x: u32) -> u32 {
        self.randmem[((x >> 2) & (RAND_SIZE as u32 - 1)) as usize]
    }

    /// One step of the ISAAC generation
    #[inline]
    fn rngstep(&mut self, i: usize, j: usize, mix: u32) {
        let x = self.randmem[i];
        self.randa = (self.randa ^ mix).wrapping_add(self.randmem[j]);
        let y = self
            .ind(x)
            .wrapping_add(self.randa)
            .wrapping_add(self.randb);
        self.randmem[i] = y;
        self.randrsl[i] = self.ind(y >> RAND_SIZL).wrapping_add(x);
        self.randb = self.randrsl[i];
    }

    /// Generate 256 new random values
    fn isaac(&mut self) {
        self.randc = self.randc.wrapping_add(1);
        self.randb = self.randb.wrapping_add(self.randc);

        let mut i = 0;
        let mut j = RAND_SIZE / 2;

        while i < RAND_SIZE / 2 {
            self.rngstep(i, j, self.randa << 13);
            i += 1;
            j += 1;
            self.rngstep(i, j, self.randa >> 6);
            i += 1;
            j += 1;
            self.rngstep(i, j, self.randa << 2);
            i += 1;
            j += 1;
            self.rngstep(i, j, self.randa >> 16);
            i += 1;
            j += 1;
        }

        i = 0;
        j = RAND_SIZE / 2;

        while i < RAND_SIZE / 2 {
            self.rngstep(j, i, self.randa << 13);
            i += 1;
            j += 1;
            self.rngstep(j, i, self.randa >> 6);
            i += 1;
            j += 1;
            self.rngstep(j, i, self.randa << 2);
            i += 1;
            j += 1;
            self.rngstep(j, i, self.randa >> 16);
            i += 1;
            j += 1;
        }

        self.randcnt = RAND_SIZE;
    }

    /// Get the next random u32
    #[inline]
    pub fn next_u32(&mut self) -> u32 {
        if self.randcnt == 0 {
            self.isaac();
        }
        self.randcnt -= 1;
        self.randrsl[self.randcnt]
    }
}

impl Default for IsaacRng {
    fn default() -> Self {
        let mut rng = Self::new();
        rng.seed(0);
        rng
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_isaac_deterministic() {
        let mut rng1 = IsaacRng::new();
        rng1.seed(42);

        let mut rng2 = IsaacRng::new();
        rng2.seed(42);

        for _ in 0..1000 {
            assert_eq!(rng1.next_u32(), rng2.next_u32());
        }
    }

    #[test]
    fn test_isaac_different_seeds() {
        let mut rng1 = IsaacRng::new();
        rng1.seed(42);

        let mut rng2 = IsaacRng::new();
        rng2.seed(43);

        let val1 = rng1.next_u32();
        let val2 = rng2.next_u32();

        assert_ne!(val1, val2);
    }

    #[test]
    fn test_isaac_range() {
        let mut rng = IsaacRng::new();
        rng.seed(42);

        // Just verify it generates values without panicking
        for _ in 0..10000 {
            let _ = rng.next_u32();
        }
    }
}
