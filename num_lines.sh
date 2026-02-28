#!/bin/bash

# Use simple glob-style patterns (* supported)
IGNORE_PATTERNS=(
    multiscan-driver/src/sick_scan_xd/
    phoenix-driver/libs/
    foxglove_layout.json
    csm-sim/assets/
)

declare -A COLORS=(
    ["cpp"]="\033[0;32m" ["hpp"]="\033[0;34m" ["py"]="\033[0;31m"
    ["sh"]="\033[0;35m"  ["cmake"]="\033[0;36m" ["c"]="\033[0;32m"
    ["h"]="\033[0;34m"   ["json"]="\033[0;33m"
    # ["xml"]="\033[1;30m" ["md"]="\033[0;90m"
)


BASE_DIR=$(dirname "$(readlink -f "$0")")
VERBOSE=0
while getopts "v" opt; do
    case "$opt" in
        v) VERBOSE=1 ;;
    esac
done

# Convert ignore patterns to regex OR chain
IGNORE_PATTERN="^$"
if [[ ${#IGNORE_PATTERNS[@]} -gt 0 ]]; then
    IGNORE_PATTERN=$(printf "%s\n" "${IGNORE_PATTERNS[@]}" | \
        sed 's/\./\\./g; s/\*/.*/g; s/\//\\\//g' | \
        tr '\n' '|' | sed 's/|$//')
fi

declare -A counts
total=0

if [[ $VERBOSE -eq 1 ]]; then
    echo -e "\033[1mScanning recursively...\033[0m"
    echo "--------------------------------------------------------"
fi

# --- Main Logic ---
while IFS= read -r file; do
    # Skip ignored paths
    [[ -n "$IGNORE_PATTERN" && "$file" =~ $IGNORE_PATTERN ]] && continue

    # Extract extension / identify CMake
    ext="${file##*.}"
    [[ "$(basename "$file")" == "CMakeLists.txt" ]] && ext="cmake"

    # Only process supported types
    if [[ -n "${COLORS[$ext]}" ]]; then
        lines=$(wc -l < "$file" 2>/dev/null)

        if [[ $VERBOSE -eq 1 ]]; then
            printf "${COLORS[$ext]}%-8s\033[0m %s\n" "$lines" "${file#$BASE_DIR/}"
        fi

        counts[$ext]=$((counts[$ext] + lines))
        total=$((total + lines))
    fi
done < <(find "$BASE_DIR" -type f 2>/dev/null)

# --- Summary ---
echo "--------------------------------------------------------"
printf "\033[1m%-15s %-10s\033[0m\n" "Language" "Lines"

for ext in "${!counts[@]}"; do
    printf "${COLORS[$ext]}%-15s\033[0m %d\n" ".$ext" "${counts[$ext]}"
done | sort -rn -k2

echo "--------------------------------------------------------"
echo -e "\033[1mTotal Lines: $total\033[0m"
