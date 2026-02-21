#!/bin/bash

# --- Config ---
IGNORE_FILE=".lineignore"
BASE_DIR=$(dirname "$(readlink -f "$0")")

declare -A COLORS=(
    ["cpp"]="\033[0;32m" ["hpp"]="\033[0;34m" ["py"]="\033[0;31m"
    ["sh"]="\033[0;35m"  ["cmake"]="\033[0;36m" ["c"]="\033[0;32m"
    ["h"]="\033[0;34m"   ["xml"]="\033[1;30m"  ["json"]="\033[0;33m"
    ["md"]="\033[0;90m"
)

# Load ignore patterns into a regex-friendly string
IGNORE_PATTERN="^$" # Default to empty line if no ignore file
if [[ -f "$BASE_DIR/$IGNORE_FILE" ]]; then
    # Convert file lines into a regex OR chain (pattern1|pattern2|pattern3)
    # Also handles basic wildcards like * and escaping dots
    IGNORE_PATTERN=$(grep -v '^#' "$BASE_DIR/$IGNORE_FILE" | grep -v '^$' | \
                     sed 's/\./\\./g; s/\*/.*/g; s/\//\\\//g' | tr '\n' '|' | sed 's/|$//')
fi

declare -A counts
total=0

echo -e "\033[1mScanning recursively (respecting $IGNORE_FILE)...\033[0m"
echo "--------------------------------------------------------"

# --- Main Logic ---
while IFS= read -r file; do
    # 1. Skip if the file path matches any ignore pattern
    [[ -n "$IGNORE_PATTERN" && "$file" =~ $IGNORE_PATTERN ]] && continue
    
    # 2. Extract extension / identify CMake
    ext="${file##*.}"
    [[ "$(basename "$file")" == "CMakeLists.txt" ]] && ext="cmake"

    # 3. Only process if we have a color/mapping for it
    if [[ -n "${COLORS[$ext]}" ]]; then
        lines=$(wc -l < "$file" 2>/dev/null)
        
        # Display relative path
        printf "${COLORS[$ext]}%-8s\033[0m %s\n" "$lines" "${file#$BASE_DIR/}"
        
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
echo -e "\033[1mTotal Lines Across Project: $total\033[0m"