#!/usr/bin/env bash
#
# Validate that outputs match across branches/implementations
# Compares the actual predictions (not headers/timing)
#
# Usage:
#   ./validate_outputs.sh [results_dir] [tolerance]
#

set -euo pipefail

RESULTS_DIR="${1:-bench_results}"
TOLERANCE="${2:-1e-9}"

if [[ ! -d "$RESULTS_DIR" ]]; then
  echo "Error: Results directory $RESULTS_DIR not found"
  exit 1
fi

echo "========================================"
echo "Output Validation Across Branches"
echo "========================================"
echo "Results dir: $RESULTS_DIR"
echo "Tolerance:   $TOLERANCE"
echo ""

# Extract just the prediction data (skip comment lines starting with #)
extract_predictions() {
  local file="$1"
  grep -v '^#' "$file" 2>/dev/null | grep -v '^$' || true
}

# Count lines of predictions
count_predictions() {
  local file="$1"
  extract_predictions "$file" | wc -l | tr -d ' '
}

# Compare two files numerically with tolerance using Python (portable)
numeric_compare() {
  local file1="$1"
  local file2="$2"
  local tol="$3"

  python3 <<EOF
import sys

def extract_predictions(filename):
    lines = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                lines.append(line)
    return lines

def parse_values(line):
    """Parse a line into float values where possible."""
    values = []
    for token in line.split():
        try:
            values.append(float(token))
        except ValueError:
            values.append(token)
    return values

file1 = "$file1"
file2 = "$file2"
tol = float("$tol")

lines1 = extract_predictions(file1)
lines2 = extract_predictions(file2)

if len(lines1) != len(lines2):
    print(f"LINE_COUNT_MISMATCH:{len(lines1)}:{len(lines2)}")
    sys.exit(0)

if len(lines1) == 0:
    print("EMPTY")
    sys.exit(0)

max_diff = 0.0
diff_count = 0
total_values = 0
first_diff_line = 0
first_diff_col = 0
first_diff_val1 = ""
first_diff_val2 = ""

for line_num, (l1, l2) in enumerate(zip(lines1, lines2), 1):
    vals1 = parse_values(l1)
    vals2 = parse_values(l2)
    
    # Handle different number of columns
    max_cols = max(len(vals1), len(vals2))
    vals1.extend([None] * (max_cols - len(vals1)))
    vals2.extend([None] * (max_cols - len(vals2)))
    
    for col, (v1, v2) in enumerate(zip(vals1, vals2), 1):
        total_values += 1
        
        if isinstance(v1, float) and isinstance(v2, float):
            diff = abs(v1 - v2)
            if diff > max_diff:
                max_diff = diff
            if diff > tol:
                diff_count += 1
                if first_diff_line == 0:
                    first_diff_line = line_num
                    first_diff_col = col
                    first_diff_val1 = str(v1)
                    first_diff_val2 = str(v2)
        elif v1 != v2:
            diff_count += 1
            if first_diff_line == 0:
                first_diff_line = line_num
                first_diff_col = col
                first_diff_val1 = str(v1)
                first_diff_val2 = str(v2)

if diff_count == 0:
    print(f"MATCH:max_diff={max_diff:.2e}:values={total_values}")
else:
    print(f"DIFFER:diffs={diff_count}:max_diff={max_diff:.2e}:first_line={first_diff_line}:col={first_diff_col}:val1={first_diff_val1}:val2={first_diff_val2}")
EOF
}

# Show first N differing lines between two files
show_diff_sample() {
  local file1="$1"
  local file2="$2"
  local label1="$3"
  local label2="$4"
  local num_lines="${5:-5}"

  echo "    First differing lines:"
  echo "    Line | $label1"
  echo "         | $label2"
  echo "    -----|"

  python3 <<EOF
def extract_predictions(filename):
    lines = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                lines.append(line)
    return lines

lines1 = extract_predictions("$file1")
lines2 = extract_predictions("$file2")

count = 0
for i, (l1, l2) in enumerate(zip(lines1, lines2), 1):
    if l1 != l2:
        print(f"    {i:4d} | {l1}")
        print(f"         | {l2}")
        count += 1
        if count >= $num_lines:
            break
EOF
  echo ""
}

# Configurations to validate
CONFIGS=("naive" "naive-sort" "regular" "regular-sort" "optimal" "optimal-sort" "logm" "logm-sort")

# Collect all branches from .txt files
BRANCHES=()
for f in "$RESULTS_DIR"/*.txt; do
  if [[ -f "$f" ]]; then
    base=$(basename "$f" .txt)
    # Extract branch name (everything before the last hyphen-separated component that's a resampler)
    for cfg in "${CONFIGS[@]}"; do
      if [[ "$base" == *"-$cfg" ]]; then
        branch="${base%-$cfg}"
        # Check if branch already in array
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
  echo "No output files found in $RESULTS_DIR"
  exit 1
fi

# Sort branches for consistent display
IFS=$'\n' BRANCHES=($(printf '%s\n' "${BRANCHES[@]}" | sort))
unset IFS

echo "Branches found: ${BRANCHES[*]}"
echo ""

# Determine reference branch
REFERENCE=""
for candidate in "c-original" "c" "${BRANCHES[@]}"; do
  for cfg in "${CONFIGS[@]}"; do
    if [[ -f "$RESULTS_DIR/${candidate}-${cfg}.txt" ]]; then
      REFERENCE="$candidate"
      break 2
    fi
  done
done

if [[ -z "$REFERENCE" ]]; then
  echo "Error: No reference branch found"
  exit 1
fi

echo "Reference branch: $REFERENCE"
echo "Tolerance: $TOLERANCE"
echo ""

# Summary counters
TOTAL_COMPARISONS=0
MATCH_COUNT=0
DIFFER_COUNT=0

echo "========================================"
echo "Per-Config Summary vs $REFERENCE"
echo "========================================"
echo ""

printf "%-15s" "Config"
for branch in "${BRANCHES[@]}"; do
  if [[ "$branch" != "$REFERENCE" ]]; then
    printf "%-18s" "$branch"
  fi
done
echo ""

printf "%-15s" "---------------"
for branch in "${BRANCHES[@]}"; do
  if [[ "$branch" != "$REFERENCE" ]]; then
    printf "%-18s" "------------------"
  fi
done
echo ""

for config in "${CONFIGS[@]}"; do
  printf "%-15s" "$config"

  ref_file="$RESULTS_DIR/${REFERENCE}-${config}.txt"

  for branch in "${BRANCHES[@]}"; do
    if [[ "$branch" == "$REFERENCE" ]]; then
      continue
    fi

    test_file="$RESULTS_DIR/${branch}-${config}.txt"

    if [[ ! -f "$ref_file" ]]; then
      printf "%-18s" "NO REF"
    elif [[ ! -f "$test_file" ]]; then
      printf "%-18s" "MISSING"
    elif grep -q "^# FAILED" "$test_file" 2>/dev/null; then
      printf "%-18s" "FAILED"
    else
      result=$(numeric_compare "$ref_file" "$test_file" "$TOLERANCE")
      status=$(echo "$result" | cut -d: -f1)

      ((TOTAL_COMPARISONS++)) || true

      case "$status" in
      MATCH)
        max_diff=$(echo "$result" | sed 's/.*max_diff=\([^:]*\).*/\1/')
        printf "%-18s" "✓ ($max_diff)"
        ((MATCH_COUNT++)) || true
        ;;
      DIFFER)
        max_diff=$(echo "$result" | sed 's/.*max_diff=\([^:]*\).*/\1/')
        diffs=$(echo "$result" | sed 's/.*diffs=\([^:]*\).*/\1/')
        printf "%-18s" "✗ $diffs ($max_diff)"
        ((DIFFER_COUNT++)) || true
        ;;
      LINE_COUNT_MISMATCH)
        counts=$(echo "$result" | cut -d: -f2-3)
        printf "%-18s" "✗ lines $counts"
        ((DIFFER_COUNT++)) || true
        ;;
      *)
        printf "%-18s" "? $status"
        ;;
      esac
    fi
  done
  echo ""
done

echo ""
echo "========================================"
echo "Detailed Differences"
echo "========================================"
echo ""

for config in "${CONFIGS[@]}"; do
  ref_file="$RESULTS_DIR/${REFERENCE}-${config}.txt"

  if [[ ! -f "$ref_file" ]]; then
    continue
  fi

  for branch in "${BRANCHES[@]}"; do
    if [[ "$branch" == "$REFERENCE" ]]; then
      continue
    fi

    test_file="$RESULTS_DIR/${branch}-${config}.txt"

    if [[ ! -f "$test_file" ]]; then
      continue
    fi

    result=$(numeric_compare "$ref_file" "$test_file" "$TOLERANCE")
    status=$(echo "$result" | cut -d: -f1)

    if [[ "$status" == "DIFFER" || "$status" == "LINE_COUNT_MISMATCH" ]]; then
      echo "--- $config: $REFERENCE vs $branch ---"
      echo "  Result: $result"

      if [[ "$status" == "DIFFER" ]]; then
        first_line=$(echo "$result" | sed 's/.*first_line=\([^:]*\).*/\1/')
        echo "  First diff at line: $first_line"
      fi

      show_diff_sample "$ref_file" "$test_file" "$REFERENCE" "$branch" 3
    fi
  done
done

echo "========================================"
echo "Summary"
echo "========================================"
echo "Reference:    $REFERENCE"
echo "Tolerance:    $TOLERANCE"
echo "Comparisons:  $TOTAL_COMPARISONS"
echo "Matches:      $MATCH_COUNT"
echo "Differences:  $DIFFER_COUNT"
echo ""

if [[ $DIFFER_COUNT -gt 0 ]]; then
  echo "⚠️  Some outputs differ! (This is expected if RNG differs between C and Rust)"
else
  echo "✓ All outputs match within tolerance!"
fi
