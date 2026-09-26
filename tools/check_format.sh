#!/usr/bin/env bash
# Fails if any of our sources isn't laid out as .clang-format says. Run from the repository
# root. To fix: clang-format -i on the files listed.
set -euo pipefail

mapfile -t sources < <(find core render ros can app tests -name '*.cpp' -o -name '*.h' | sort)
if clang-format --dry-run --Werror "${sources[@]}" 2>/dev/null; then
    echo "format: ok (${#sources[@]} files)"
else
    clang-format --dry-run "${sources[@]}" 2>&1 | grep -oP '^[^:]+(?=:\d+:\d+: )' | sort -u |
        sed 's/^/format: needs clang-format -i: /' >&2
    exit 1
fi
