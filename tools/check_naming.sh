#!/usr/bin/env bash
# Enforces the naming rules in .clang-tidy: lower_case functions, Upper_Snake structs,
# lower_case variables and fields, UPPER_CASE macros and enum values.
# Usage: tools/check_naming.sh [build-dir]   (the build dir needs compile_commands.json)
# Checks exactly the files this build configuration compiles, so optional adapters are
# checked in the configurations that build them.
set -euo pipefail

build=${1:-build}
database=$build/compile_commands.json
if [[ ! -f $database ]]; then
    echo "check_naming: no $database; configure with CMake first" >&2
    exit 1
fi

root=$(pwd)
mapfile -t sources < <(grep -oP '"file":\s*"\K[^"]+' "$database" |
    grep -E "^$root/(core|render|can|app|tests|ros)/" | sort -u)
if [[ ${#sources[@]} -eq 0 ]]; then
    echo "check_naming: no sources of ours in $database" >&2
    exit 1
fi

if clang-tidy --quiet -p "$build" "${sources[@]}" 2>/dev/null; then
    echo "naming: ok (${#sources[@]} files)"
else
    echo "naming: violations above" >&2
    exit 1
fi
