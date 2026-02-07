#!/usr/bin/env bash
#
# Benchmark script for rust-bmpf resamplers
# Runs each resampler with 10k particles, with and without --sort
# Saves outputs and timing to <BRANCH>-<RESAMPLER>[-sort].txt
#
# Usage:
#   ./bench.sh [branch_name]
#
# If branch_name is not provided, uses current branch name.
# Creates a results/ directory with all output files.
#

set -euo pipefail

# Configuration
NPARTICLES=10000
DATA_FILE="vehicle-c.dat"
RESULTS_DIR="bench_results"
RESAMPLERS=("naive" "regular" "optimal" "logm")

# Get branch name
if [[ $# -ge 1 ]]; then
  BRANCH="$1"
else
  BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
fi

# Sanitize branch name for filenames
BRANCH_SAFE=$(echo "$BRANCH" | tr '/' '-' | tr ' ' '_')

echo "========================================"
echo "Benchmarking rust-bmpf"
echo "========================================"
echo "Branch:      $BRANCH"
echo "Particles:   $NPARTICLES"
echo "Data file:   $DATA_FILE"
echo "Results dir: $RESULTS_DIR"
echo "========================================"
echo ""

# Create results directory
mkdir -p "$RESULTS_DIR"

# Build release binary
echo "Building release binary..."
RUSTFLAGS="-C target-cpu=native -C opt-level=3" cargo build --release --example bpf 2>&1 | tail -5
echo ""

# Check if binary exists
BPF_BIN="./target/release/examples/bpf"
if [[ ! -x "$BPF_BIN" ]]; then
  echo "Error: $BPF_BIN not found or not executable"
  exit 1
fi

# Check if data file exists
if [[ ! -f "$DATA_FILE" ]]; then
  echo "Error: Data file $DATA_FILE not found"
  exit 1
fi

# Function to run a single benchmark
run_benchmark() {
  local resampler="$1"
  local sort_flag="$2"
  local sort_suffix=""
  local sort_arg=""

  if [[ "$sort_flag" == "true" ]]; then
    sort_suffix="-sort"
    sort_arg="--sort"
  fi

  local output_file="${RESULTS_DIR}/${BRANCH_SAFE}-${resampler}${sort_suffix}.txt"
  local timing_file="${RESULTS_DIR}/${BRANCH_SAFE}-${resampler}${sort_suffix}.timing"

  echo -n "  ${resampler}${sort_suffix}: "

  # Run with timing, capture both output and timing
  {
    echo "# Branch: $BRANCH"
    echo "# Resampler: $resampler"
    echo "# Sort: $sort_flag"
    echo "# Particles: $NPARTICLES"
    echo "# Data file: $DATA_FILE"
    echo "# Timestamp: $(date -Iseconds)"
    echo "# Command: $BPF_BIN --nparticles $NPARTICLES --sampler $resampler --file $DATA_FILE $sort_arg"
    echo "#"
  } >"$output_file"

  # Portable timing
  local start_time
  local end_time
  local elapsed

  if command -v gdate &>/dev/null; then
    start_time=$(gdate +%s.%N)
  elif date +%s.%N &>/dev/null 2>&1; then
    start_time=$(date +%s.%N)
  else
    start_time=$(date +%s)
  fi

  if $BPF_BIN --nparticles "$NPARTICLES" --sampler "$resampler" --file "$DATA_FILE" $sort_arg >>"$output_file" 2>&1; then
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
    } >"$timing_file"
  else
    echo "FAILED (exit code $?)"
    echo "# FAILED" >>"$output_file"
  fi
}

# Run all benchmarks
echo "Running benchmarks..."
echo ""

for resampler in "${RESAMPLERS[@]}"; do
  echo "Resampler: $resampler"
  run_benchmark "$resampler" "false"
  run_benchmark "$resampler" "true"
  echo ""
done

# Generate summary (no 'local' outside function)
SUMMARY_FILE="${RESULTS_DIR}/${BRANCH_SAFE}-summary.txt"
echo "Generating summary..."
{
  echo "========================================"
  echo "Benchmark Summary: $BRANCH"
  echo "========================================"
  echo "Date: $(date)"
  echo "Particles: $NPARTICLES"
  echo "Data file: $DATA_FILE"
  echo ""
  echo "Timing Results:"
  echo "----------------------------------------"
  printf "%-20s %s\n" "Configuration" "Time (seconds)"
  echo "----------------------------------------"

  for timing_file in "${RESULTS_DIR}/${BRANCH_SAFE}"*.timing; do
    if [[ -f "$timing_file" ]]; then
      # Read variables from timing file
      resampler=""
      sort=""
      elapsed_seconds=""
      while IFS='=' read -r key value; do
        case "$key" in
        resampler) resampler="$value" ;;
        sort) sort="$value" ;;
        elapsed_seconds) elapsed_seconds="$value" ;;
        esac
      done <"$timing_file"

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
echo "  - Output files: ${BRANCH_SAFE}-<resampler>[-sort].txt"
echo "  - Timing files: ${BRANCH_SAFE}-<resampler>[-sort].timing"
echo "  - Summary:      ${BRANCH_SAFE}-summary.txt"
