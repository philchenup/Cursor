#!/usr/bin/env bash
# Exercise M3TSetupGLEW.cmake against a fake Windows-style GLEW prefix.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT

FAKE="${WORK}/glew-2.1.0"
mkdir -p "${FAKE}/include/GL" \
         "${FAKE}/lib/Release/x64" \
         "${FAKE}/bin/Release/x64"

cat > "${FAKE}/include/GL/glew.h" <<'EOF'
#ifndef GLEW_H
#define GLEW_H
#endif
EOF

# find_library on Linux looks for lib<name>.so / lib<name>.a
# Official Windows zip uses glew32.lib; we emulate both layouts.
echo 'fake glew' > "${FAKE}/lib/Release/x64/glew32.lib"
echo 'fake glew' > "${FAKE}/lib/Release/x64/libglew32.a"
echo 'fake glew' > "${FAKE}/bin/Release/x64/glew32.dll"

run_case() {
  local name="$1"
  shift
  local build="${WORK}/build-${name}"
  rm -rf "${build}"
  mkdir -p "${build}"
  echo "=== GLEW finder case: ${name} ==="
  cmake -S "${ROOT}/tests/glew_finder" -B "${build}" \
    -DM3T_CMAKE_DIR="${ROOT}/cmake" \
    "$@"
}

# Case 1: GLEW_ROOT with official zip layout (lib/Release/x64)
run_case root -DGLEW_ROOT="${FAKE}"

# Case 2: plural cache vars people actually pass (and a DIRECTORY for GLEW_LIBRARIES)
run_case plural \
  -DGLEW_INCLUDE_DIRS="${FAKE}/include" \
  -DGLEW_LIBRARIES="${FAKE}/lib/Release/x64"

# Case 3: singular file path for the library
run_case file \
  -DGLEW_INCLUDE_DIR="${FAKE}/include" \
  -DGLEW_LIBRARY="${FAKE}/lib/Release/x64/libglew32.a"

echo "All GLEW finder cases passed."
