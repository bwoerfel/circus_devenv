#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# build_docs.sh – build Sphinx + Breathe + Exhale docs for ca1_motor_ctrl
#
# Usage (inside the devcontainer or a ROS 2 Humble environment):
#   ./build_docs.sh [--serve]
#
# Options:
#   --serve   Start a live-reload server on http://localhost:8000 after build
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCS_DIR="$SCRIPT_DIR/docs"
SERVE=0

for arg in "$@"; do
  [[ "$arg" == "--serve" ]] && SERVE=1
done

# ---------------------------------------------------------------------------
# 1. Source ROS 2 environment (needed for any ROS-aware extensions)
# ---------------------------------------------------------------------------
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # ROS setup.bash uses variables that may be unset; temporarily relax -u.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  echo "[docs] ROS 2 Humble sourced."
else
  echo "[docs] WARNING: /opt/ros/humble/setup.bash not found – skipping ROS source."
fi

# ---------------------------------------------------------------------------
# 2. Check Doxygen
# ---------------------------------------------------------------------------
if ! command -v doxygen &>/dev/null; then
  echo "[docs] ERROR: doxygen not found. Install with: sudo apt-get install -y doxygen"
  exit 1
fi
echo "[docs] Doxygen $(doxygen --version) found."

# ---------------------------------------------------------------------------
# 3. Resolve pip and install / upgrade Python doc dependencies
# ---------------------------------------------------------------------------
if ! command -v python3 &>/dev/null; then
  echo "[docs] ERROR: python3 not found."
  exit 1
fi

# Ensure python3-venv is installed (needed on Debian/Ubuntu, package name is
# version-specific, e.g. python3.14-venv).
# Probe via ensurepip – it is the component stripped by Debian; its absence is
# exactly what causes `python3 -m venv` to silently produce a broken venv.
# (Alternative: `python3 -c "help('modules')" | grep venv` works but is slow.)
if ! python3 -c "import ensurepip" &>/dev/null 2>&1; then
  PY_VER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
  echo "[docs] python${PY_VER}-venv missing – installing via apt ..."
  sudo apt-get install -y "python${PY_VER}-venv"
fi

# Use a dedicated venv to avoid PEP 668 system-package restrictions.
# Remove a partial/broken venv (no pip binary) before re-creating.
VENV="$DOCS_DIR/.venv"
if [[ -d "$VENV" && ! -x "$VENV/bin/pip" ]]; then
  echo "[docs] Removing broken venv ..."
  rm -rf "$VENV"
fi
if [[ ! -d "$VENV" ]]; then
  echo "[docs] Creating virtual environment at docs/.venv ..."
  python3 -m venv "$VENV"
fi

PIP="$VENV/bin/pip"
SPHINXBUILD="$VENV/bin/sphinx-build"
SPHINXAUTOBUILD="$VENV/bin/sphinx-autobuild"

echo "[docs] Installing Python dependencies from docs/requirements.txt ..."
"$PIP" install --quiet --upgrade pip
"$PIP" install --quiet -r "$DOCS_DIR/requirements.txt"

# ---------------------------------------------------------------------------
# 4. Clean previous build artifacts
# ---------------------------------------------------------------------------
echo "[docs] Cleaning previous build ..."
make -C "$DOCS_DIR" clean

# ---------------------------------------------------------------------------
# 5. Run Doxygen (generates XML consumed by Breathe/Exhale)
# ---------------------------------------------------------------------------
echo "[docs] Running Doxygen ..."
(cd "$DOCS_DIR" && doxygen Doxyfile)

# ---------------------------------------------------------------------------
# 6. Build HTML
# ---------------------------------------------------------------------------
echo "[docs] Building HTML documentation ..."
make -C "$DOCS_DIR" html SPHINXBUILD="$SPHINXBUILD"

HTML_INDEX="$DOCS_DIR/build/html/index.html"
echo "[docs] Done. Output: $HTML_INDEX"

# ---------------------------------------------------------------------------
# 6. Optional live-reload server
# ---------------------------------------------------------------------------
if [[ "$SERVE" -eq 1 ]]; then
  echo "[docs] Starting live-reload server on http://localhost:8000 ..."
  "$PIP" install --quiet sphinx-autobuild
  make -C "$DOCS_DIR" livehtml SPHINXAUTOBUILD="$SPHINXAUTOBUILD"
fi
