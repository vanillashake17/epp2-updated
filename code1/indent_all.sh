#!/usr/bin/env bash
# indent_all.sh — re-indent all source files under code1/ using Vim's gg=G.
# Usage: bash indent_all.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

shopt -s nullglob

files=()
while IFS= read -r -d '' f; do
    files+=("$f")
done < <(find "$SCRIPT_DIR" \
    -type f \( -name '*.ino' -o -name '*.h' -o -name '*.cpp' -o -name '*.c' \) \
    ! -path '*/__pycache__/*' \
    ! -path '*/env/*' \
    -print0 | sort -z)

if [ ${#files[@]} -eq 0 ]; then
    echo "No source files found."
    exit 0
fi

echo "Re-indenting ${#files[@]} file(s)..."
for f in "${files[@]}"; do
    printf '  %s\n' "${f#"$SCRIPT_DIR"/}"
    vim -es -c 'normal gg=G' -c 'wq' "$f"
done
echo "Done."
