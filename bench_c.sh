#!/usr/bin/env bash
#
# Benchmark script for original C BMPF implementation
# For comparison with rust-bmpf results
#
# Usage:
#   ./bench_c.sh <path_to_bmpf_c_binary> [data_file]
#
# The C binary should be the compiled 'bpf' from BartMassey's BMPF repo
#
# C CLI format (from source):
#   ./bpf [options] <nparticles> <resampler>[sort]
#   Input is read from stdin
#
# Examples:
#   ./bpf 10000 naive           # naive resampler, no sort
#   ./bpf 10000 naivesort       # naive resampler, with sort
#   ./bpf 10000 regular
#   ./bpf 10000 regularsort
#

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <path_to_bmpf_c_binary> [data_file]"
  echo ""
  echo "Example:"
  echo "  ./bench_c.sh ../BMPF/bpf vehicle-c2.dat"
  echo ""
  echo "C CLI format:"
  echo "  The C version uses positional args and reads from stdin:"
  echo "    ./bpf [options] <nparticles> <resampler>[sort] < data.dat"
  echo ""
  echo "  Resamplers: naive, regular, optimal, logm"
  echo "  Append 'sort' for sorting: naivesort, regularsort, etc."
  exit 1
fi

BPF_C_BIN="$1"
DATA_FILE="${2:-vehicle-c2.dat}"
NPARTICLES=10000
RESULTS_DIR="bench_results"
BRANCH="c-original"

# Check binary exists
if [[ ! -x "$BPF_C_BIN" ]]; then
  echo "Error: $BPF_C_BIN not found or not executable"
  echo ""
  echo "To build the C version:"
  echo "  cd /path/to/BMPF"
  echo "  make"
  exit 1
fi

# Check data file exists
if [[ ! -f "$DATA_FILE" ]]; then
  echo "Error: Data file $DATA_FILE not found"
  exit 1
fi

echo "========================================"
echo "Benchmarking C BMPF (Bart Massey)"
echo "========================================"
echo "Binary:      $BPF_C_BIN"
echo "Particles:   $NPARTICLES"
echo "Data file:   $DATA_FILE"
echo "Results dir: $RESULTS_DIR"
echo "========================================"
echo ""

mkdir -p "$RESULTS_DIR"

# C resampler names (same as Rust)
RESAMPLERS=("naive" "regular" "optimal" "logm")

run_c_benchmark() {
  local resampler="$1"
  local sort_flag="$2"
  local sort_suffix=""
  local c_resampler="$resampler"

  # C uses "naivesort" not "naive --sort"
  if [[ "$sort_flag" == "true" ]]; then
    sort_suffix="-sort"
    c_resampler="${resampler}sort"
  fi

  local output_file="${RESULTS_DIR}/${BRANCH}-${resampler}${sort_suffix}.txt"
  local timing_file="${RESULTS_DIR}/${BRANCH}-${resampler}${sort_suffix}.timing"

  echo -n "  ${resampler}${sort_suffix}: "

  # Write header
  {
    echo "# Implementation: C (Bart Massey BMPF)"
    echo "# Binary: $BPF_C_BIN"
    echo "# Resampler: $resampler"
    echo "# Sort: $sort_flag"
    echo "# Particles: $NPARTICLES"
    echo "# Data file: $DATA_FILE"
    echo "# Timestamp: $(date -Iseconds)"
    echo "# Command: $BPF_C_BIN $NPARTICLES $c_resampler < $DATA_FILE"
    echo "#"
  } >"$output_file"

  # Time the execution
  # C version reads from stdin: ./bpf <nparticles> <resampler> < data.dat
  local start_time
  local end_time
  local elapsed

  # Use portable timing (works on macOS and Linux)
  if command -v gdate &>/dev/null; then
    # macOS with coreutils
    start_time=$(gdate +%s.%N)
  elif date +%s.%N &>/dev/null 2>&1; then
    # Linux
    start_time=$(date +%s.%N)
  else
    # macOS without coreutils (seconds only)
    start_time=$(date +%s)
  fi

  if $BPF_C_BIN "$NPARTICLES" "$c_resampler" <"$DATA_FILE" >>"$output_file" 2>&1; then
    if command -v gdate &>/dev/null; then
      end_time=$(gdate +%s.%N)
    elif date +%s.%N &>/dev/null 2>&1; then
      end_time=$(date +%s.%N)
    else
      end_time=$(date +%s)
    fi

    elapsed=$(echo "$end_time - $start_time" | bc 2>/dev/null || echo "N/A")
    echo "${elapsed}s"

    # Save timing info
    {
      echo "branch=$BRANCH"
      echo "resampler=$resampler"
      echo "sort=$sort_flag"
      echo "nparticles=$NPARTICLES"
      echo "elapsed_seconds=$elapsed"
      echo "timestamp=$(date -Iseconds)"
      echo "binary=$BPF_C_BIN"
    } >"$timing_file"
  else
    local exit_code=$?
    echo "FAILED (exit code $exit_code)"
    echo "# FAILED with exit code $exit_code" >>"$output_file"

    # Save failed timing
    {
      echo "branch=$BRANCH"
      echo "resampler=$resampler"
      echo "sort=$sort_flag"
      echo "nparticles=$NPARTICLES"
      echo "elapsed_seconds=FAILED"
      echo "timestamp=$(date -Iseconds)"
      echo "binary=$BPF_C_BIN"
    } >"$timing_file"
  fi
}

echo "Running C benchmarks..."
echo ""

for resampler in "${RESAMPLERS[@]}"; do
  echo "Resampler: $resampler"
  run_c_benchmark "$resampler" "false"
  run_c_benchmark "$resampler" "true"
  echo ""
done

# Generate summary
SUMMARY_FILE="${RESULTS_DIR}/${BRANCH}-summary.txt"
echo "Generating summary..."
{
  echo "========================================"
  echo "C BMPF Benchmark Summary"
  echo "========================================"
  echo "Binary: $BPF_C_BIN"
  echo "Date: $(date)"
  echo "Particles: $NPARTICLES"
  echo "Data file: $DATA_FILE"
  echo ""
  echo "Timing Results:"
  echo "----------------------------------------"
  printf "%-20s %s\n" "Configuration" "Time (seconds)"
  echo "----------------------------------------"

  for timing_file in "${RESULTS_DIR}/${BRANCH}"*.timing; do
    if [[ -f "$timing_file" ]]; then
      # Source the timing file to get variables
      resampler=""
      sort=""
      elapsed_seconds=""
      source "$timing_file"

      config="${resampler}"
      if [[ "$sort" == "true" ]]; then
        config="${config}-sort"
      fi
      printf "%-20s %s\n" "$config" "$elapsed_seconds"
    fi
  done

  echo "----------------------------------------"
} >"$SUMMARY_FILE"

cat "$SUMMARY_FILE"

echo ""
echo "Results saved to: $RESULTS_DIR/"
echo "  - Output files: ${BRANCH}-<resampler>[-sort].txt"
echo "  - Timing files: ${BRANCH}-<resampler>[-sort].timing"
echo "  - Summary:      ${BRANCH}-summary.txt"
echo ""
echo "Run ./compare_branches.sh to compare with Rust results"
echo "Run ./validate_outputs.sh to verify output correctness"
