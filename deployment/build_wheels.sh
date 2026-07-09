#!/usr/bin/env bash
# build_wheels.sh
# Run this script on the LOTUSim simulation machine to assemble the deployment
# bundle that is given to external developers (PhD students, partners, etc.).
#
# The deployment/ folder IS the remote colcon workspace: the PhD receives it,
# runs `colcon build` inside it, sources install/setup.bash, and runs an agent.
#
# Output:
#   deployment/dist/   - lotusim_sdk-*.whl     (pure Python, agent abstractions)
#                      - lotusim_client-*.whl  (pure Python, run_agent CLI)
#   deployment/src/    - lotusim_msgs/         (SOURCE pkg — colcon-built on the remote)
#
# Why is lotusim_msgs shipped as SOURCE and not as a wheel?
#   lotusim_msgs contains compiled rosidl typesupport (.so) that is tied to the
#   exact ROS distro AND Python version it was built against. The simulation
#   machine runs Ubuntu 24 / Jazzy / Python 3.12; the remote machine typically
#   runs Ubuntu 22 / Humble / Python 3.10. A wheel built here will NOT import
#   there (ABI mismatch on both CPython and the ROS libraries). The only robust
#   way is to rebuild the messages from source against the remote's own ROS.
#   The sources are tiny (a handful of .msg/.srv/.action text files).
#
# Requirements: ROS 2 sourced, pip, wheel

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DIST_DIR="$SCRIPT_DIR/dist"
SRC_DIR="$SCRIPT_DIR/src"

mkdir -p "$DIST_DIR" "$SRC_DIR"

echo "=== Building deployment bundle into $SCRIPT_DIR ==="

# ---------------------------------------------------------------------------
# 1. lotusim_msgs (shipped as SOURCE — built with colcon on the remote)
# ---------------------------------------------------------------------------
echo ""
echo "--- [1/3] lotusim_msgs (source) ---"

# Allow overriding LOTUSIM_WS via environment variable, fallback to $HOME/lotusim_ws
LOTUSIM_WS_DIR="${LOTUSIM_WS:-$HOME/lotusim_ws}"

MSGS_SRC=""
for WS_DIR in "$LOTUSIM_WS_DIR" "$REPO_ROOT"; do
    [ -d "$WS_DIR/src" ] || continue
    FOUND=$(find "$WS_DIR/src" -type f -name package.xml -path '*lotusim_msgs*' 2>/dev/null \
            | grep -vE '/(build|install|log)/' | head -1)
    if [ -n "$FOUND" ]; then
        MSGS_SRC=$(dirname "$FOUND")
        break
    fi
done

if [ -z "$MSGS_SRC" ] || [ ! -d "$MSGS_SRC" ]; then
    echo "ERROR: lotusim_msgs SOURCE package not found under $LOTUSIM_WS_DIR/src or $REPO_ROOT/src."
    echo "       Set LOTUSIM_WS to the workspace that contains src/.../lotusim_msgs."
    exit 1
fi

MSGS_OUT="$SRC_DIR/lotusim_msgs"
rm -rf "$MSGS_OUT"
mkdir -p "$MSGS_OUT"
cp -r "$MSGS_SRC/." "$MSGS_OUT/"
# Strip any build artifacts that may sit next to the sources
rm -rf "$MSGS_OUT/build" "$MSGS_OUT/install" "$MSGS_OUT/log"
echo "    OK: lotusim_msgs source copied to src/ from $MSGS_SRC"

# ---------------------------------------------------------------------------
# 2. lotusim_sdk wheel (pure Python)
# ---------------------------------------------------------------------------
echo ""
echo "--- [2/3] lotusim_sdk ---"
pip wheel "$REPO_ROOT/src/lotusim_sdk" --no-deps -w "$DIST_DIR" --quiet
echo "    OK: lotusim_sdk wheel created."

# ---------------------------------------------------------------------------
# 3. lotusim_client wheel (pure Python)
# ---------------------------------------------------------------------------
echo ""
echo "--- [3/3] lotusim_client ---"
pip wheel "$REPO_ROOT/src/lotusim_client" --no-deps -w "$DIST_DIR" --quiet
echo "    OK: lotusim_client wheel created."

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "=== Done. Deployment workspace ready in $SCRIPT_DIR ==="
echo "  dist/ :"; ls -1 "$DIST_DIR"
echo "  src/  :"; ls -1 "$SRC_DIR"
echo ""
echo "Hand the whole deployment/ folder to the remote machine. There:"
echo "  pip install dist/*.whl  &&  colcon build  &&  source install/setup.bash"
