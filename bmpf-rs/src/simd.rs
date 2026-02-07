//! SIMD-accelerated operations with runtime CPU feature dispatch
//!
//! Uses `multiversion` for runtime dispatch and relies on compiler auto-vectorization
//! with appropriate target features enabled. The `targets = "simd"` preset enables
//! SSE, AVX, AVX2, etc. and the compiler will auto-vectorize the loops.
//!
//! For transcendental functions (exp), we use `libm` for portability.

use multiversion::multiversion;

// =============================================================================
// Weight operations (pure arithmetic - excellent auto-vectorization)
// =============================================================================

/// Compute weighted circular mean of angles
///
/// Uses the proper method: avg_angle = atan2(sum(w*sin(t)), sum(w*cos(t)))
#[multiversion(targets = "simd")]
pub fn weighted_circular_mean(weights: &[f64], angles: &[f64]) -> f64 {
    debug_assert_eq!(weights.len(), angles.len());

    let mut sum_sin = 0.0;
    let mut sum_cos = 0.0;

    for (&w, &t) in weights.iter().zip(angles.iter()) {
        sum_sin += w * libm::sin(t);
        sum_cos += w * libm::cos(t);
    }

    libm::atan2(sum_sin, sum_cos)
}

/// Scale all elements in place: values[i] *= scale
#[multiversion(targets = "simd")]
pub fn scale_slice(values: &mut [f64], scale: f64) {
    // Simple loop - compiler will auto-vectorize with SIMD target features
    for v in values.iter_mut() {
        *v *= scale;
    }
}

/// Sum all elements
#[multiversion(targets = "simd")]
pub fn sum_slice(values: &[f64]) -> f64 {
    values.iter().sum()
}

/// Accumulate weighted sum: sum(weights[i] * values[i])
#[multiversion(targets = "simd")]
pub fn weighted_sum(weights: &[f64], values: &[f64]) -> f64 {
    debug_assert_eq!(weights.len(), values.len());
    weights.iter().zip(values.iter()).map(|(w, v)| w * v).sum()
}

/// Element-wise multiply: out[i] = a[i] * b[i]
#[multiversion(targets = "simd")]
pub fn mul_slice(a: &[f64], b: &[f64], out: &mut [f64]) {
    debug_assert_eq!(a.len(), b.len());
    debug_assert_eq!(a.len(), out.len());

    for ((x, y), z) in a.iter().zip(b.iter()).zip(out.iter_mut()) {
        *z = x * y;
    }
}

/// Element-wise multiply in place: a[i] *= b[i]
#[multiversion(targets = "simd")]
pub fn mul_slice_inplace(a: &mut [f64], b: &[f64]) {
    debug_assert_eq!(a.len(), b.len());

    for (x, y) in a.iter_mut().zip(b.iter()) {
        *x *= *y;
    }
}

/// Fill slice with a value
#[multiversion(targets = "simd")]
pub fn fill_slice(values: &mut [f64], value: f64) {
    for v in values.iter_mut() {
        *v = value;
    }
}

// =============================================================================
// Gaussian probability with libm exp()
// =============================================================================

/// Compute Gaussian probability: exp(-0.5 * (delta / sd)^2) for a slice of deltas
///
/// Uses libm::exp for portability across platforms.
#[multiversion(targets = "simd")]
pub fn gprob_slice(deltas: &[f64], inv_sd: f64, out: &mut [f64]) {
    debug_assert_eq!(deltas.len(), out.len());
    let neg_half_inv_sd_sq = -0.5 * inv_sd * inv_sd;

    for (d, o) in deltas.iter().zip(out.iter_mut()) {
        let exponent = neg_half_inv_sd_sq * d * d;
        *o = libm::exp(exponent);
    }
}

/// Compute Gaussian probability for two deltas (x and y) combined
/// Returns: exp(-0.5 * ((dx/sd)^2 + (dy/sd)^2))
#[multiversion(targets = "simd")]
pub fn gprob_2d_batch(dx: &[f64], dy: &[f64], inv_sd: f64, out: &mut [f64]) {
    debug_assert_eq!(dx.len(), dy.len());
    debug_assert_eq!(dx.len(), out.len());
    let neg_half_inv_sd_sq = -0.5 * inv_sd * inv_sd;

    for ((x, y), o) in dx.iter().zip(dy.iter()).zip(out.iter_mut()) {
        let exponent = neg_half_inv_sd_sq * (x * x + y * y);
        *o = libm::exp(exponent);
    }
}

/// Single-value Gaussian probability (scalar)
#[inline]
pub fn gprob(delta: f64, inv_sd: f64) -> f64 {
    let exponent = -0.5 * inv_sd * inv_sd * delta * delta;
    libm::exp(exponent)
}

/// Single-value 2D Gaussian probability (scalar)
#[inline]
pub fn gprob_2d(dx: f64, dy: f64, inv_sd: f64) -> f64 {
    let inv_sd_sq = inv_sd * inv_sd;
    let exponent = -0.5 * inv_sd_sq * (dx * dx + dy * dy);
    libm::exp(exponent)
}

// =============================================================================
// GPS and IMU probability batch operations
// =============================================================================

use crate::consts::{BOX_DIM, IMU_A_VAR, IMU_R_VAR, MAX_SPEED, NEG_BOX_DIM, TWO_PI};

/// Batch GPS probability calculation
///
/// For each particle i: if position is out of bounds, prob = 0
/// Otherwise: prob = gprob(px - gps_x, gps_var) * gprob(py - gps_y, gps_var)
#[multiversion(targets = "simd")]
pub fn gps_prob_batch(
    posn_x: &[f64],
    posn_y: &[f64],
    gps_x: f64,
    gps_y: f64,
    inv_gps_var: f64,
    out: &mut [f64],
) {
    debug_assert_eq!(posn_x.len(), posn_y.len());
    debug_assert_eq!(posn_x.len(), out.len());

    let neg_half_inv_var_sq = -0.5 * inv_gps_var * inv_gps_var;

    for ((&px, &py), o) in posn_x.iter().zip(posn_y.iter()).zip(out.iter_mut()) {
        // Bounds check
        if !(NEG_BOX_DIM..=BOX_DIM).contains(&px) || !(NEG_BOX_DIM..=BOX_DIM).contains(&py) {
            *o = 0.0;
        } else {
            let dx = px - gps_x;
            let dy = py - gps_y;
            let exponent = neg_half_inv_var_sq * (dx * dx + dy * dy);
            *o = libm::exp(exponent);
        }
    }
}

/// Batch IMU probability calculation
///
/// For each particle i: if velocity is out of bounds, prob = 0
/// Otherwise: prob = gprob(vr - imu_r, imu_r_var) * gprob(angle_diff, imu_a_var)
#[multiversion(targets = "simd")]
pub fn imu_prob_batch(
    vel_r: &[f64],
    vel_t: &[f64],
    imu_r: f64,
    imu_t: f64,
    inv_dt: f64,
    out: &mut [f64],
) {
    debug_assert_eq!(vel_r.len(), vel_t.len());
    debug_assert_eq!(vel_r.len(), out.len());

    let imu_r_var_scaled = IMU_R_VAR * inv_dt;
    let imu_a_var_scaled = IMU_A_VAR * inv_dt;
    let inv_r_var = 1.0 / imu_r_var_scaled;
    let inv_a_var = 1.0 / imu_a_var_scaled;
    let neg_half_inv_r_sq = -0.5 * inv_r_var * inv_r_var;
    let neg_half_inv_a_sq = -0.5 * inv_a_var * inv_a_var;

    for ((&vr, &vt), o) in vel_r.iter().zip(vel_t.iter()).zip(out.iter_mut()) {
        // Velocity bounds check
        if !(0.0..=MAX_SPEED).contains(&vr) {
            *o = 0.0;
        } else {
            let dr = vr - imu_r;
            // Angle difference with wraparound
            let dth_raw = (vt - imu_t).abs();
            let dth = dth_raw.min((dth_raw - TWO_PI).abs());

            let exp_r = neg_half_inv_r_sq * dr * dr;
            let exp_a = neg_half_inv_a_sq * dth * dth;
            *o = libm::exp(exp_r) * libm::exp(exp_a);
        }
    }
}

// =============================================================================
// Weight update: combined probability and weight multiplication
// =============================================================================

/// Update weights: weight[i] = gps_prob[i] * imu_prob[i] * weight[i]
/// Also returns the sum of all updated weights
#[multiversion(targets = "simd")]
pub fn update_weights(gps_probs: &[f64], imu_probs: &[f64], weights: &mut [f64]) -> f64 {
    debug_assert_eq!(gps_probs.len(), imu_probs.len());
    debug_assert_eq!(gps_probs.len(), weights.len());

    let mut sum = 0.0;
    for ((gp, ip), w) in gps_probs
        .iter()
        .zip(imu_probs.iter())
        .zip(weights.iter_mut())
    {
        *w *= gp * ip;
        sum += *w;
    }
    sum
}

// =============================================================================
// Resampling helpers with SIMD prefix sum
// =============================================================================

/// Compute prefix sum (cumulative sum): out[i] = sum(values[0..=i])
///
/// Prefix sum is inherently sequential, but we compute it efficiently.
/// Returns the total sum.
#[inline]
pub fn prefix_sum(values: &[f64], out: &mut [f64]) -> f64 {
    debug_assert_eq!(values.len(), out.len());

    if values.is_empty() {
        return 0.0;
    }

    let mut sum = 0.0;
    for (v, o) in values.iter().zip(out.iter_mut()) {
        sum += v;
        *o = sum;
    }
    sum
}

/// Find first index where cumsum[i] >= target
/// Assumes cumsum is sorted (which prefix sums are)
#[inline]
pub fn find_threshold_index(cumsum: &[f64], target: f64) -> usize {
    // Binary search for efficiency with large particle counts
    match cumsum.binary_search_by(|x| x.partial_cmp(&target).unwrap_or(std::cmp::Ordering::Less)) {
        Ok(i) => i,
        Err(i) => i.min(cumsum.len().saturating_sub(1)),
    }
}

/// Batch copy particles from src to dst based on indices
/// dst_particle[i] = src_particle[indices[i]]
#[multiversion(targets = "simd")]
pub fn gather_particles(
    src_posn_x: &[f64],
    src_posn_y: &[f64],
    src_vel_r: &[f64],
    src_vel_t: &[f64],
    src_weight: &[f64],
    indices: &[usize],
    dst_posn_x: &mut [f64],
    dst_posn_y: &mut [f64],
    dst_vel_r: &mut [f64],
    dst_vel_t: &mut [f64],
    dst_weight: &mut [f64],
) {
    let n = indices.len();
    debug_assert_eq!(dst_posn_x.len(), n);
    debug_assert_eq!(dst_posn_y.len(), n);
    debug_assert_eq!(dst_vel_r.len(), n);
    debug_assert_eq!(dst_vel_t.len(), n);
    debug_assert_eq!(dst_weight.len(), n);

    for (i, &idx) in indices.iter().enumerate() {
        dst_posn_x[i] = src_posn_x[idx];
        dst_posn_y[i] = src_posn_y[idx];
        dst_vel_r[i] = src_vel_r[idx];
        dst_vel_t[i] = src_vel_t[idx];
        dst_weight[i] = src_weight[idx];
    }
}

/// Find best (max weight) and worst (min weight) particle indices
#[multiversion(targets = "simd")]
pub fn find_best_worst(weights: &[f64]) -> (usize, usize, f64, f64) {
    debug_assert!(!weights.is_empty());

    let mut best_idx = 0;
    let mut worst_idx = 0;
    let mut best_weight = weights[0];
    let mut worst_weight = weights[0];

    for (i, &w) in weights.iter().enumerate().skip(1) {
        if w > best_weight {
            best_weight = w;
            best_idx = i;
        } else if w < worst_weight {
            worst_weight = w;
            worst_idx = i;
        }
    }

    (best_idx, worst_idx, best_weight, worst_weight)
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scale_slice() {
        let mut values = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        scale_slice(&mut values, 2.0);
        assert_eq!(values, vec![2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0]);
    }

    #[test]
    fn test_sum_slice() {
        let values = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert!((sum_slice(&values) - 15.0).abs() < 1e-10);
    }

    #[test]
    fn test_weighted_sum() {
        let weights = vec![0.1, 0.2, 0.3, 0.4];
        let values = vec![1.0, 2.0, 3.0, 4.0];
        let result = weighted_sum(&weights, &values);
        let expected = 0.1 * 1.0 + 0.2 * 2.0 + 0.3 * 3.0 + 0.4 * 4.0;
        assert!((result - expected).abs() < 1e-10);
    }

    #[test]
    fn test_gprob() {
        let result = gprob(0.0, 1.0);
        assert!((result - 1.0).abs() < 1e-10);

        let result = gprob(1.0, 1.0);
        let expected = libm::exp(-0.5);
        assert!((result - expected).abs() < 1e-10);
    }

    #[test]
    fn test_gprob_slice() {
        let deltas = vec![0.0, 1.0, 2.0];
        let mut out = vec![0.0; 3];
        gprob_slice(&deltas, 1.0, &mut out);

        for (d, o) in deltas.iter().zip(out.iter()) {
            let expected = gprob(*d, 1.0);
            assert!((o - expected).abs() < 1e-10);
        }
    }

    #[test]
    fn test_prefix_sum() {
        let values = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let mut out = vec![0.0; 5];
        let total = prefix_sum(&values, &mut out);

        assert_eq!(out, vec![1.0, 3.0, 6.0, 10.0, 15.0]);
        assert!((total - 15.0).abs() < 1e-10);
    }

    #[test]
    fn test_find_threshold_index() {
        let cumsum = vec![1.0, 3.0, 6.0, 10.0, 15.0];

        assert_eq!(find_threshold_index(&cumsum, 0.5), 0);
        assert_eq!(find_threshold_index(&cumsum, 1.0), 0);
        assert_eq!(find_threshold_index(&cumsum, 1.5), 1);
        assert_eq!(find_threshold_index(&cumsum, 3.0), 1);
        assert_eq!(find_threshold_index(&cumsum, 5.0), 2);
        assert_eq!(find_threshold_index(&cumsum, 15.0), 4);
        assert_eq!(find_threshold_index(&cumsum, 20.0), 4); // Clamps to last valid
    }

    #[test]
    fn test_find_best_worst() {
        let weights = vec![0.1, 0.5, 0.2, 0.8, 0.3];
        let (best, worst, best_w, worst_w) = find_best_worst(&weights);

        assert_eq!(best, 3);
        assert_eq!(worst, 0);
        assert!((best_w - 0.8).abs() < 1e-10);
        assert!((worst_w - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_update_weights() {
        let gps = vec![0.5, 0.8, 0.9];
        let imu = vec![0.6, 0.7, 0.4];
        let mut weights = vec![1.0, 1.0, 1.0];

        let sum = update_weights(&gps, &imu, &mut weights);

        assert!((weights[0] - 0.3).abs() < 1e-10);
        assert!((weights[1] - 0.56).abs() < 1e-10);
        assert!((weights[2] - 0.36).abs() < 1e-10);
        assert!((sum - 1.22).abs() < 1e-10);
    }
}
