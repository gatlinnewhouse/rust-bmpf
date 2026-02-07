use std::alloc::{Layout, alloc, dealloc, handle_alloc_error};
use std::marker::PhantomData;
use std::mem;
use std::ptr::NonNull;
use std::sync::OnceLock;

/// Default SIMD alignment - can be overridden at runtime
static SIMD_ALIGNMENT: OnceLock<usize> = OnceLock::new();

/// Get the current SIMD alignment (defaults based on CPU features)
#[inline]
pub fn simd_alignment() -> usize {
    *SIMD_ALIGNMENT.get_or_init(detect_optimal_alignment)
}

/// Set the SIMD alignment (must be called before any AVec is created)
/// Returns Err if alignment was already set or is invalid (not a power of 2)
pub fn set_simd_alignment(align: usize) -> Result<(), &'static str> {
    if !align.is_power_of_two() {
        return Err("Alignment must be a power of 2");
    }
    if align < mem::align_of::<f64>() {
        return Err("Alignment must be at least 8 bytes for f64");
    }
    SIMD_ALIGNMENT
        .set(align)
        .map_err(|_| "Alignment already set")
}

/// Detect optimal alignment based on CPU features
fn detect_optimal_alignment() -> usize {
    #[cfg(target_arch = "x86_64")]
    {
        if is_x86_feature_detected!("avx512f") {
            return 64; // AVX-512: 512 bits = 64 bytes
        }
        if is_x86_feature_detected!("avx") {
            return 32; // AVX: 256 bits = 32 bytes
        }
        return 16; // SSE: 128 bits = 16 bytes
    }

    #[cfg(target_arch = "aarch64")]
    {
        return 16; // NEON: 128 bits = 16 bytes
    }

    #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
    {
        return 16; // Conservative default
    }
}

/// Generic aligned vector for SIMD operations
///
/// Alignment is determined at runtime via `simd_alignment()` or `set_simd_alignment()`.
/// This allows SIMD crates to configure optimal alignment based on detected CPU features.
pub struct AVec<T> {
    ptr: NonNull<T>,
    len: usize,
    cap: usize,
    align: usize,
    _marker: PhantomData<T>,
}

// ============================================================================
// Methods that don't require Copy + Default (work on any T)
// ============================================================================
impl<T> AVec<T> {
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Get the alignment of this vector
    #[inline]
    pub fn alignment(&self) -> usize {
        self.align
    }

    #[inline]
    pub fn as_slice(&self) -> &[T] {
        if self.len == 0 {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(self.ptr.as_ptr(), self.len) }
        }
    }

    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        if self.len == 0 {
            &mut []
        } else {
            unsafe { std::slice::from_raw_parts_mut(self.ptr.as_ptr(), self.len) }
        }
    }

    /// Get aligned pointer for SIMD operations
    #[inline]
    pub fn as_ptr(&self) -> *const T {
        self.ptr.as_ptr()
    }

    #[inline]
    pub fn as_mut_ptr(&mut self) -> *mut T {
        self.ptr.as_ptr()
    }

    /// Iterator over elements
    #[inline]
    pub fn iter(&self) -> std::slice::Iter<'_, T> {
        self.as_slice().iter()
    }

    /// Mutable iterator over elements
    #[inline]
    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, T> {
        self.as_mut_slice().iter_mut()
    }
}

// ============================================================================
// Methods that require Copy (for swap)
// ============================================================================
impl<T: Copy> AVec<T> {
    /// Swap elements at indices `i` and `j`
    #[inline]
    pub fn swap(&mut self, i: usize, j: usize) {
        assert!(
            i < self.len,
            "index i={} out of bounds (len={})",
            i,
            self.len
        );
        assert!(
            j < self.len,
            "index j={} out of bounds (len={})",
            j,
            self.len
        );

        if i != j {
            unsafe {
                let ptr_i = self.ptr.as_ptr().add(i);
                let ptr_j = self.ptr.as_ptr().add(j);
                std::ptr::swap(ptr_i, ptr_j);
            }
        }
    }

    /// Fill all elements with a value
    #[inline]
    pub fn fill(&mut self, value: T) {
        for elem in self.as_mut_slice() {
            *elem = value;
        }
    }
}

// ============================================================================
// Methods that require Copy + Default (for construction)
// ============================================================================
impl<T: Copy + Default> AVec<T> {
    /// Create a new aligned vector with `size` elements, default-initialized
    pub fn new(size: usize) -> Self {
        Self::with_alignment(size, simd_alignment())
    }

    /// Create a new aligned vector with explicit alignment
    pub fn with_alignment(size: usize, align: usize) -> Self {
        assert!(align.is_power_of_two(), "Alignment must be a power of 2");

        if size == 0 {
            return Self {
                ptr: NonNull::dangling(),
                len: 0,
                cap: 0,
                align,
                _marker: PhantomData,
            };
        }

        let elem_size = mem::size_of::<T>();

        // Calculate capacity rounded up to alignment boundary
        let elems_per_align = if elem_size > 0 { align / elem_size } else { 1 };
        let cap = if elems_per_align > 1 {
            (size + elems_per_align - 1) / elems_per_align * elems_per_align
        } else {
            size
        };

        let layout = Layout::from_size_align(cap * elem_size, align).expect("Invalid layout");

        let ptr = unsafe {
            let raw = alloc(layout);
            if raw.is_null() {
                handle_alloc_error(layout);
            }
            // Initialize with default values
            let typed_ptr = raw as *mut T;
            for i in 0..size {
                std::ptr::write(typed_ptr.add(i), T::default());
            }
            NonNull::new_unchecked(typed_ptr)
        };

        Self {
            ptr,
            len: size,
            cap,
            align,
            _marker: PhantomData,
        }
    }

    /// Create from an iterator
    pub fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        Self::from_iter_aligned(iter, simd_alignment())
    }

    /// Create from an iterator with explicit alignment
    pub fn from_iter_aligned<I: IntoIterator<Item = T>>(iter: I, align: usize) -> Self {
        let vec: Vec<T> = iter.into_iter().collect();
        let mut result = Self::with_alignment(vec.len(), align);
        for (i, val) in vec.into_iter().enumerate() {
            result[i] = val;
        }
        result
    }
}

// ============================================================================
// Clone requires Copy + Default
// ============================================================================
impl<T: Copy + Default> Clone for AVec<T> {
    fn clone(&self) -> Self {
        let mut new = Self::with_alignment(self.len, self.align);
        if self.len > 0 {
            new.as_mut_slice().copy_from_slice(self.as_slice());
        }
        new
    }
}

// ============================================================================
// Drop works for any T
// ============================================================================
impl<T> Drop for AVec<T> {
    fn drop(&mut self) {
        if self.cap > 0 {
            let elem_size = mem::size_of::<T>();
            if elem_size > 0 {
                let layout = Layout::from_size_align(self.cap * elem_size, self.align)
                    .expect("Invalid layout");

                unsafe {
                    // Drop elements if T needs dropping
                    if mem::needs_drop::<T>() {
                        for i in 0..self.len {
                            std::ptr::drop_in_place(self.ptr.as_ptr().add(i));
                        }
                    }
                    dealloc(self.ptr.as_ptr() as *mut u8, layout);
                }
            }
        }
    }
}

// ============================================================================
// Index traits (work for any T)
// ============================================================================
impl<T> std::ops::Index<usize> for AVec<T> {
    type Output = T;

    #[inline]
    fn index(&self, i: usize) -> &T {
        assert!(i < self.len, "index {} out of bounds (len={})", i, self.len);
        unsafe { &*self.ptr.as_ptr().add(i) }
    }
}

impl<T> std::ops::IndexMut<usize> for AVec<T> {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut T {
        assert!(i < self.len, "index {} out of bounds (len={})", i, self.len);
        unsafe { &mut *self.ptr.as_ptr().add(i) }
    }
}

impl<T> std::ops::Index<std::ops::Range<usize>> for AVec<T> {
    type Output = [T];

    #[inline]
    fn index(&self, range: std::ops::Range<usize>) -> &[T] {
        &self.as_slice()[range]
    }
}

impl<T> std::ops::IndexMut<std::ops::Range<usize>> for AVec<T> {
    #[inline]
    fn index_mut(&mut self, range: std::ops::Range<usize>) -> &mut [T] {
        &mut self.as_mut_slice()[range]
    }
}

// ============================================================================
// Safety traits
// ============================================================================
unsafe impl<T: Send> Send for AVec<T> {}
unsafe impl<T: Sync> Sync for AVec<T> {}

// ============================================================================
// Debug (requires T: Debug, but not Copy + Default)
// ============================================================================
impl<T: std::fmt::Debug> std::fmt::Debug for AVec<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AVec")
            .field("len", &self.len)
            .field("align", &self.align)
            .field("data", &self.as_slice())
            .finish()
    }
}

// ============================================================================
// Tests
// ============================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_and_access() {
        let mut v: AVec<f64> = AVec::new(10);
        assert_eq!(v.len(), 10);

        for i in 0..10 {
            v[i] = i as f64;
        }

        for i in 0..10 {
            assert_eq!(v[i], i as f64);
        }
    }

    #[test]
    fn test_swap() {
        let mut v: AVec<f64> = AVec::new(5);
        v[0] = 1.0;
        v[1] = 2.0;
        v[2] = 3.0;
        v[3] = 4.0;
        v[4] = 5.0;

        v.swap(1, 3);

        assert_eq!(v[0], 1.0);
        assert_eq!(v[1], 4.0);
        assert_eq!(v[2], 3.0);
        assert_eq!(v[3], 2.0);
        assert_eq!(v[4], 5.0);
    }

    #[test]
    fn test_swap_same_index() {
        let mut v: AVec<f64> = AVec::new(3);
        v[0] = 1.0;
        v[1] = 2.0;
        v[2] = 3.0;

        v.swap(1, 1);

        assert_eq!(v[0], 1.0);
        assert_eq!(v[1], 2.0);
        assert_eq!(v[2], 3.0);
    }

    #[test]
    fn test_clone() {
        let mut v: AVec<f64> = AVec::new(3);
        v[0] = 1.0;
        v[1] = 2.0;
        v[2] = 3.0;

        let v2 = v.clone();

        assert_eq!(v2[0], 1.0);
        assert_eq!(v2[1], 2.0);
        assert_eq!(v2[2], 3.0);
    }

    #[test]
    fn test_alignment() {
        let v: AVec<f64> = AVec::new(100);
        let ptr = v.as_ptr() as usize;
        let align = v.alignment();
        assert_eq!(ptr % align, 0, "Pointer not aligned to {} bytes", align);
    }

    #[test]
    fn test_explicit_alignment() {
        let v: AVec<f64> = AVec::with_alignment(100, 64);
        let ptr = v.as_ptr() as usize;
        assert_eq!(v.alignment(), 64);
        assert_eq!(ptr % 64, 0, "Pointer not aligned to 64 bytes");
    }

    #[test]
    fn test_from_iter() {
        let v: AVec<f64> = AVec::from_iter([1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(v.len(), 5);
        assert_eq!(v[0], 1.0);
        assert_eq!(v[4], 5.0);
    }

    #[test]
    fn test_empty() {
        let v: AVec<f64> = AVec::new(0);
        assert!(v.is_empty());
        assert_eq!(v.len(), 0);
    }

    #[test]
    fn test_fill() {
        let mut v: AVec<f64> = AVec::new(5);
        v.fill(42.0);
        for i in 0..5 {
            assert_eq!(v[i], 42.0);
        }
    }

    #[test]
    fn test_range_index() {
        let mut v: AVec<f64> = AVec::from_iter([1.0, 2.0, 3.0, 4.0, 5.0]);
        let slice = &v[1..4];
        assert_eq!(slice, &[2.0, 3.0, 4.0]);

        v[1..4].copy_from_slice(&[20.0, 30.0, 40.0]);
        assert_eq!(v[1], 20.0);
        assert_eq!(v[2], 30.0);
        assert_eq!(v[3], 40.0);
    }

    #[test]
    fn test_detected_alignment() {
        let align = detect_optimal_alignment();
        assert!(align.is_power_of_two());
        assert!(align >= 16);
        println!("Detected optimal alignment: {} bytes", align);
    }

    #[test]
    fn test_iter() {
        let v: AVec<f64> = AVec::from_iter([1.0, 2.0, 3.0]);
        let sum: f64 = v.iter().sum();
        assert_eq!(sum, 6.0);
    }

    #[test]
    fn test_iter_mut() {
        let mut v: AVec<f64> = AVec::from_iter([1.0, 2.0, 3.0]);
        for x in v.iter_mut() {
            *x *= 2.0;
        }
        assert_eq!(v[0], 2.0);
        assert_eq!(v[1], 4.0);
        assert_eq!(v[2], 6.0);
    }
}

