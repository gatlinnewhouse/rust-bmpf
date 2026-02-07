#!/usr/bin/env bash
#
# Compare resampler performance within each branch
# Shows which resampler is fastest for each branch/implementation
#
# Usage:
#   ./compare_resamplers.sh [results_dir]
#

set -euo pipefail

RESULTS_DIR="${1:-bench_results}"

if [[ ! -d "$RESULTS_DIR" ]]; then
  echo "Error: Results directory $RESULTS_DIR not found"
  echo "Run bench.sh on each branch first"
  exit 1
fi

echo "========================================"
echo "Resampler Comparison Within Branches"
echo "========================================"
echo ""

# Collect all branches
BRANCHES=()
for f in "$RESULTS_DIR"/*.timing; do
  if [[ -f "$f" ]]; then
    base=$(basename "$f" .timing)
    # Extract branch name
    for suffix in naive naive-sort regular regular-sort optimal optimal-sort logm logm-sort; do
      if [[ "$base" == *"-$suffix" ]]; then
        branch="${base%-$suffix}"
        found=0
        for b in "${BRANCHES[@]+"${BRANCHES[@]}"}"; do
          if [[ "$b" == "$branch" ]]; then
            found=1
            break
          fi
        done
        if [[ $found -eq 0 ]]; then
          BRANCHES+=("$branch")
        fi
        break
      fi
    done
  fi
done

if [[ ${#BRANCHES[@]} -eq 0 ]]; then
  echo "No timing files found in $RESULTS_DIR"
  exit 1
fi

# Sort branches
IFS=$'\n' BRANCHES=($(printf '%s\n' "${BRANCHES[@]}" | sort))
unset IFS

RESAMPLERS=("naive" "regular" "optimal" "logm")

# Function to get elapsed time from timing file
get_time() {
  local file="$1"
  if [[ -f "$file" ]]; then
    local val
    val=$(grep "elapsed_seconds=" "$file" 2>/dev/null | cut -d= -f2 || echo "N/A")
    if [[ -z "$val" ]]; then
      echo "N/A"
    else
      echo "$val"
    fi
  else
    echo "N/A"
  fi
}

# Function to format time for display
format_time() {
  local t="$1"
  if [[ "$t" == "N/A" || "$t" == "FAILED" || -z "$t" ]]; then
    echo "$t"
  else
    printf "%.3fs" "$t"
  fi
}

# Per-branch comparison
for branch in "${BRANCHES[@]}"; do
  echo "========================================"
  echo "Branch: $branch"
  echo "========================================"
  echo ""

  # Build timing data file for Python
  TIMING_DATA=$(mktemp)
  trap "rm -f $TIMING_DATA" EXIT

  for resampler in "${RESAMPLERS[@]}"; do
    t_nosort=$(get_time "$RESULTS_DIR/${branch}-${resampler}.timing")
    t_sort=$(get_time "$RESULTS_DIR/${branch}-${resampler}-sort.timing")
    echo "${resampler},nosort,${t_nosort}" >>"$TIMING_DATA"
    echo "${resampler},sort,${t_sort}" >>"$TIMING_DATA"
  done

  # Use Python for all the complex formatting
  python3 <<EOF
import sys

# Read timing data
data = {}
with open("$TIMING_DATA") as f:
    for line in f:
        parts = line.strip().split(',')
        if len(parts) == 3:
            resampler, sort_type, time_str = parts
            key = (resampler, sort_type)
            if time_str not in ('N/A', 'FAILED', ''):
                try:
                    data[key] = float(time_str)
                except ValueError:
                    data[key] = None
            else:
                data[key] = None

resamplers = ['naive', 'regular', 'optimal', 'logm']

# Display table
print(f"{'Resampler':<12} {'No Sort':>15} {'With Sort':>15} {'Sort Overhead':>15}")
print(f"{'-'*12:<12} {'-'*15:>15} {'-'*15:>15} {'-'*15:>15}")

for r in resamplers:
    t_nosort = data.get((r, 'nosort'))
    t_sort = data.get((r, 'sort'))
    
    nosort_str = f"{t_nosort:.3f}s" if t_nosort is not None else "N/A"
    sort_str = f"{t_sort:.3f}s" if t_sort is not None else "N/A"
    
    if t_nosort is not None and t_sort is not None and t_nosort > 0:
        overhead = ((t_sort - t_nosort) / t_nosort) * 100
        overhead_str = f"+{overhead:.1f}%" if overhead >= 0 else f"{overhead:.1f}%"
    else:
        overhead_str = "N/A"
    
    print(f"{r:<12} {nosort_str:>15} {sort_str:>15} {overhead_str:>15}")

print()

# Ranking (no sort)
print("Ranking (no sort):")
nosort_times = [(r, data.get((r, 'nosort'))) for r in resamplers]
nosort_times = [(r, t) for r, t in nosort_times if t is not None]
nosort_times.sort(key=lambda x: x[1])

if nosort_times:
    fastest = nosort_times[0][1]
    for rank, (name, t) in enumerate(nosort_times, 1):
        relative = t / fastest if fastest > 0 else 0
        print(f"  {rank}. {name:<12} {t:.3f}s ({relative:.2f}x)")
else:
    print("  No valid timing data")

print()

# Ranking (with sort)
print("Ranking (with sort):")
sort_times = [(r, data.get((r, 'sort'))) for r in resamplers]
sort_times = [(r, t) for r, t in sort_times if t is not None]
sort_times.sort(key=lambda x: x[1])

if sort_times:
    fastest = sort_times[0][1]
    for rank, (name, t) in enumerate(sort_times, 1):
        relative = t / fastest if fastest > 0 else 0
        print(f"  {rank}. {name:<12} {t:.3f}s ({relative:.2f}x)")
else:
    print("  No valid timing data")

print()
EOF

  rm -f "$TIMING_DATA"
done

echo "========================================"
echo "Cross-Branch Resampler Comparison"
echo "========================================"
echo ""

# Build complete timing data for cross-branch comparison
TIMING_DATA=$(mktemp)
trap "rm -f $TIMING_DATA" EXIT

for branch in "${BRANCHES[@]}"; do
  for resampler in "${RESAMPLERS[@]}"; do
    t_nosort=$(get_time "$RESULTS_DIR/${branch}-${resampler}.timing")
    t_sort=$(get_time "$RESULTS_DIR/${branch}-${resampler}-sort.timing")
    echo "${branch},${resampler},nosort,${t_nosort}" >>"$TIMING_DATA"
    echo "${branch},${resampler},sort,${t_sort}" >>"$TIMING_DATA"
  done
done

python3 <<EOF
import sys
from collections import defaultdict

# Read all timing data
data = {}
with open("$TIMING_DATA") as f:
    for line in f:
        parts = line.strip().split(',')
        if len(parts) == 4:
            branch, resampler, sort_type, time_str = parts
            key = (branch, resampler, sort_type)
            if time_str not in ('N/A', 'FAILED', ''):
                try:
                    data[key] = float(time_str)
                except ValueError:
                    data[key] = None
            else:
                data[key] = None

resamplers = ['naive', 'regular', 'optimal', 'logm']
branches = sorted(set(k[0] for k in data.keys()))

# Per-resampler comparison across branches
for resampler in resamplers:
    print(f"--- {resampler} ---")
    print(f"{'Branch':<15} {'No Sort':>15} {'With Sort':>15}")
    print(f"{'-'*15:<15} {'-'*15:>15} {'-'*15:>15}")
    
    branch_times = []
    for branch in branches:
        t_nosort = data.get((branch, resampler, 'nosort'))
        t_sort = data.get((branch, resampler, 'sort'))
        
        nosort_str = f"{t_nosort:.3f}s" if t_nosort is not None else "N/A"
        sort_str = f"{t_sort:.3f}s" if t_sort is not None else "N/A"
        
        print(f"{branch:<15} {nosort_str:>15} {sort_str:>15}")
        
        if t_nosort is not None:
            branch_times.append((branch, t_nosort))
    
    # Find fastest branch
    if branch_times:
        branch_times.sort(key=lambda x: x[1])
        fastest_branch, fastest_time = branch_times[0]
        if len(branch_times) > 1:
            slowest_branch, slowest_time = branch_times[-1]
            if fastest_time > 0:
                speedup = slowest_time / fastest_time
                print(f"\n  Fastest: {fastest_branch} ({fastest_time:.3f}s) - {speedup:.1f}x faster than {slowest_branch}")
            else:
                print(f"\n  Fastest: {fastest_branch} ({fastest_time:.3f}s)")
        else:
            print(f"\n  Fastest: {fastest_branch} ({fastest_time:.3f}s)")
    
    print()

# Summary: Best resampler per branch
print("=" * 40)
print("Summary: Best Resampler per Branch")
print("=" * 40)
print()
print(f"{'Branch':<15} {'Best (no sort)':<20} {'Best (sort)':<20}")
print(f"{'-'*15:<15} {'-'*20:<20} {'-'*20:<20}")

for branch in branches:
    # Find best no-sort
    best_nosort = None
    best_nosort_time = float('inf')
    for r in resamplers:
        t = data.get((branch, r, 'nosort'))
        if t is not None and t < best_nosort_time:
            best_nosort_time = t
            best_nosort = r
    
    # Find best sort
    best_sort = None
    best_sort_time = float('inf')
    for r in resamplers:
        t = data.get((branch, r, 'sort'))
        if t is not None and t < best_sort_time:
            best_sort_time = t
            best_sort = r
    
    nosort_str = f"{best_nosort} ({best_nosort_time:.3f}s)" if best_nosort else "N/A"
    sort_str = f"{best_sort} ({best_sort_time:.3f}s)" if best_sort else "N/A"
    
    print(f"{branch:<15} {nosort_str:<20} {sort_str:<20}")

print()
print("=" * 40)
print("Recommendation")
print("=" * 40)
print()

# Find overall best
all_results = []
for (branch, resampler, sort_type), t in data.items():
    if t is not None:
        sort_str = "sort" if sort_type == "sort" else "no-sort"
        all_results.append((t, branch, resampler, sort_str))

all_results.sort(key=lambda x: x[0])

print("Top 5 fastest configurations:")
print()
for i, (t, branch, resampler, sort_str) in enumerate(all_results[:5], 1):
    print(f"  {i}. {branch} + {resampler} ({sort_str}): {t:.3f}s")

if all_results:
    print()
    winner = all_results[0]
    print(f"🏆 Winner: {winner[1]} branch with {winner[2]} resampler ({winner[3]})")
    print(f"   Time: {winner[0]:.3f}s")
EOF

rm -f "$TIMING_DATA"
