#!/usr/bin/env bash
#
# Compare benchmark results across branches
# Run after running bench.sh on each branch
#
# Usage:
#   ./compare_branches.sh [results_dir]
#

set -euo pipefail

RESULTS_DIR="${1:-bench_results}"

if [[ ! -d "$RESULTS_DIR" ]]; then
  echo "Error: Results directory $RESULTS_DIR not found"
  echo "Run bench.sh on each branch first"
  exit 1
fi

echo "========================================"
echo "Cross-Branch Performance Comparison"
echo "========================================"
echo ""

# Collect all branches
BRANCHES=$(ls -1 "$RESULTS_DIR"/*.timing 2>/dev/null | xargs -I{} basename {} | cut -d'-' -f1 | sort -u)

if [[ -z "$BRANCHES" ]]; then
  echo "No timing files found in $RESULTS_DIR"
  exit 1
fi

echo "Branches found: $BRANCHES"
echo ""

# Print header
printf "%-15s" "Resampler"
for branch in $BRANCHES; do
  printf "%-15s" "$branch"
done
echo ""
printf "%-15s" "---------------"
for branch in $BRANCHES; do
  printf "%-15s" "---------------"
done
echo ""

# Configurations to compare
CONFIGS=("naive" "naive-sort" "regular" "regular-sort" "optimal" "optimal-sort" "logm" "logm-sort")

for config in "${CONFIGS[@]}"; do
  printf "%-15s" "$config"

  for branch in $BRANCHES; do
    timing_file="$RESULTS_DIR/${branch}-${config}.timing"
    if [[ -f "$timing_file" ]]; then
      elapsed=$(grep "elapsed_seconds=" "$timing_file" | cut -d'=' -f2)
      printf "%-15s" "${elapsed}s"
    else
      printf "%-15s" "N/A"
    fi
  done
  echo ""
done

echo ""

# Calculate speedups relative to slowest (usually gpoint or c-original)
BASELINE=""
for candidate in "c" "gpoint"; do
  if [[ -f "$RESULTS_DIR/${candidate}-naive.timing" ]]; then
    BASELINE="$candidate"
    break
  fi
done

if [[ -n "$BASELINE" ]]; then
  echo "========================================"
  echo "Speedup vs $BASELINE"
  echo "========================================"
  echo ""

  printf "%-15s" "Resampler"
  for branch in $BRANCHES; do
    if [[ "$branch" != "$BASELINE" ]]; then
      printf "%-15s" "$branch"
    fi
  done
  echo ""
  printf "%-15s" "---------------"
  for branch in $BRANCHES; do
    if [[ "$branch" != "$BASELINE" ]]; then
      printf "%-15s" "---------------"
    fi
  done
  echo ""

  for config in "${CONFIGS[@]}"; do
    printf "%-15s" "$config"

    baseline_file="$RESULTS_DIR/${BASELINE}-${config}.timing"
    if [[ -f "$baseline_file" ]]; then
      baseline_time=$(grep "elapsed_seconds=" "$baseline_file" | cut -d'=' -f2)

      for branch in $BRANCHES; do
        if [[ "$branch" != "$BASELINE" ]]; then
          timing_file="$RESULTS_DIR/${branch}-${config}.timing"
          if [[ -f "$timing_file" ]]; then
            branch_time=$(grep "elapsed_seconds=" "$timing_file" | cut -d'=' -f2)
            speedup=$(echo "scale=2; $baseline_time / $branch_time" | bc 2>/dev/null || echo "N/A")
            printf "%-15s" "${speedup}x"
          else
            printf "%-15s" "N/A"
          fi
        fi
      done
    else
      for branch in $BRANCHES; do
        if [[ "$branch" != "$BASELINE" ]]; then
          printf "%-15s" "N/A"
        fi
      done
    fi
    echo ""
  done
fi

echo ""
echo "========================================"
echo "Output Validation"
echo "========================================"
echo ""

# Run validation if script exists
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "$SCRIPT_DIR/validate_outputs.sh" ]]; then
  "$SCRIPT_DIR/validate_outputs.sh" "$RESULTS_DIR"
else
  echo "Run ./validate_outputs.sh to validate output correctness"
fi
