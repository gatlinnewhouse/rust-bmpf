//! Constants for the Ziggurat algorithm

pub const ZIGGURAT_TABLE_SIZE: usize = 256;

// Normal distribution constants
pub const ZIGGURAT_NOR_R: f64 = 3.6541528853610088;
pub const ZIGGURAT_NOR_INV_R: f64 = 0.27366123732975828;
pub const NOR_SECTION_AREA: f64 = 0.00492867323399;

// Exponential distribution constants
pub const ZIGGURAT_EXP_R: f64 = 7.69711747013104972;
pub const ZIGGURAT_EXP_INV_R: f64 = 0.129918765548341586;
pub const EXP_SECTION_AREA: f64 = 0.0039496598225815571993;

// Polynomial distribution constants (for future use)
pub const PN: i32 = 50;
pub const ZIGGURAT_POL_SECTION_AREA: f64 = 7.9277910491253e-05;
