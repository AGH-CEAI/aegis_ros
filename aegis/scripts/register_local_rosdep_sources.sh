#!/usr/bin/env bash
# Discovers and runs every package-local rosdep registration script under a
# workspace 'src' directory, then refreshes the rosdep cache once.
# Usage: register_local_rosdep_sources.sh [SRC_DIR]   (default: ./src)
set -euo pipefail

SRC_DIR="${1:-src}"
HOOK_NAME="register_rosdep_sources.sh"

if [[ ! -d "${SRC_DIR}" ]]; then
  echo "ERROR: source directory not found: ${SRC_DIR}" >&2
  exit 1
fi
SRC_DIR="$(cd "${SRC_DIR}" && pwd)"

mapfile -t hooks < <(
  find "${SRC_DIR}" \
    \( -name build -o -name install -o -name log -o -name .git \) -prune -o \
    -type f -path "*/rosdep/${HOOK_NAME}" -print | sort
)

if ((${#hooks[@]} == 0)); then
  echo "No package-local rosdep sources found under ${SRC_DIR}."
else
  echo "Found ${#hooks[@]} package-local rosdep source(s):"
  for hook in "${hooks[@]}"; do
    echo "==> ${hook#"${SRC_DIR}"/}"
    bash "${hook}"
  done
fi
