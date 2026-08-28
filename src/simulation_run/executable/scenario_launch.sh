#!/bin/bash
#
# @file scenario_lotusim.sh
# @author Naval Group
# @brief Launcher script for Lotusim simulations with ROS2, XDyn, and Unity.
#
# @details
# Automates the setup and execution of Lotusim scenarios:
# - Cleans up old processes
# - Sets environment variables, ROS_DOMAIN_ID and ROS_IP
# - Launches XDyn agents with their YML configs
# - Optionally starts ROS TCP Endpoint and Unity renderer
# - Starts main Python simulation with debug mode support
#
# @version 0.1
# @date 2026-04-08SS
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at:
# http://www.eclipse.org/legal/epl-2.0
#
# SPDX-License-Identifier: EPL-2.0
#
#Copyright (c) 2025 Naval Group

# ============================================================
# Lotusim Launcher Script
# ============================================================

# -------------------- Colors --------------------
YELLOW='\033[0;33m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

# -------------------- Detect ROS Distro --------------------
UBUNTU_VERSION="$(lsb_release -rs 2>/dev/null || true)"
case "$UBUNTU_VERSION" in
  24.04) ROS_DISTRO="jazzy" ;;
  *)
    echo -e "${RED}[ERROR] Unsupported Ubuntu version: ${UBUNTU_VERSION:-unknown}. Supported: 24.04 (Jazzy) only.${NC}"
    exit 1
    ;;
esac
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
echo -e "${GREEN}[INFO] Detected Ubuntu ${UBUNTU_VERSION} → ROS 2 ${ROS_DISTRO^}${NC}"

# -------------------- Parameters --------------------
# Set ROS 2 Domain ID used across all processes. 0 is the project default and
# matches deployment/setup_ros_network.sh; override from the environment if a
# specific run must be isolated on another domain.
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Set ROS_IP to the first local IP address if it is not already defined. 
# To use a specific IP, set ROS_IP manually before using this file, 
# e.g. export ROS_IP=192.168.1.42 
export ROS_IP="${ROS_IP:-$(hostname -I | awk '{print $1}')}"

# Unity mode options:
#   "exe"    → run pre-built Unity executable (default)
#   "editor" → open Unity project in Editor for debugging

# Setup Logging Directory
export LOG_DIR="$(pwd)/scenario_logs/$(date +%Y-%m-%d_%H-%M-%S)"
mkdir -p "$LOG_DIR"

# Redirect ALL script output (stdout and stderr) to both the terminal and a main log file
exec > >(tee -i "$LOG_DIR/main_simulation.log") 2>&1

echo -e "${GREEN}[INFO] Scenario execution logs will be saved to: $LOG_DIR${NC}"
# "exe" (default) launches the pre-built Unity player; "editor" skips that
# and just prints a reminder — open the Unity project in the Editor yourself
# (via Unity Hub) and press Play, e.g.:
#   UNITY_MODE=editor ./scenario_launch.sh --config raj_scenario.json
UNITY_MODE="${UNITY_MODE:-exe}"

# XDYN_VERBOSE=1 passes -v -w to every xdyn-for-cs instance launched below,
# logging every message and every websocket event (connect/disconnect/
# payload) to $LOG_DIR/xdyn_<agent_type>.log -- off by default (very chatty):
#   XDYN_VERBOSE=1 ./scenario_launch.sh --config current_examples/station_keeping_ekman.json

# Extra command-line args passed to the Unity player (only in "exe" mode).
# Default = windowed 1280x720 (Unity would otherwise go fullscreen). Graphics
# API is the build's default (Vulkan). The inspection-camera SIGSEGV was a render-
# thread ("UnityGfxDeviceW") crash caused by InspectionDetector capturing/reading the
# camera mid-frame; it is fixed in the Unity source (capture now runs at end-of-frame
# via WaitForEndOfFrame), so no graphics-API override is needed with an up-to-date build.
# Env-overridable, e.g. for fullscreen or driver workarounds:
#   UNITY_EXTRA_ARGS="-screen-fullscreen 1" ./scenario_launch.sh ...
#   UNITY_EXTRA_ARGS="-force-glcore" ...   (needs OpenGL in the build)
#   UNITY_EXTRA_ARGS="-force-gfx-direct" ...   (single-threaded renderer; use as a
#       stop-gap on an OLD build that predates the end-of-frame fix — it removes the
#       separate render thread the crash happened on)
UNITY_EXTRA_ARGS="${UNITY_EXTRA_ARGS:--screen-fullscreen 0 -screen-width 1280 -screen-height 720}"

# -------------------- Paths --------------------
# LOTUSIM_* variables come from ~/.bashrc (set by the install script); the
# fallbacks match that installer's layout. Everything is exported so child
# processes (xdyn, ROS nodes, the lotusim CLI) always see these paths.
export LOTUSIM_WS="${LOTUSIM_WS:-$HOME/lotusim_ws}"
export LOTUSIM_PATH="${LOTUSIM_PATH:-$LOTUSIM_WS/src/LOTUSim}"
export PATH=$LOTUSIM_PATH/physics:$LOTUSIM_PATH/launch:$PATH
export LD_LIBRARY_PATH=$LOTUSIM_PATH/physics:$LD_LIBRARY_PATH
# Trailing slash required: xdyn joins this to a model YAML's relative
# "mesh:" path by plain concatenation, so without it lrauv.yml's
# "lrauv/tethys_hydrostatic_hull.stl" resolves to ".../assets/modelslrauv/..."
# and xdyn refuses to simulate the vehicle at all. The installer's own export
# already carries the slash; this one overrides it.
export LOTUSIM_MODELS_PATH=$LOTUSIM_PATH/assets/models/

# --- Updated paths for your scenario workspace ---
LOTUSIM_SCENARIO_WS="${LOTUSIM_SCENARIO_WS:-$HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario}"
CONFIG_DIR="$LOTUSIM_SCENARIO_WS/src/simulation_run/config"
UNITY_EXECUTABLES_DIR="$LOTUSIM_SCENARIO_WS/lotusim_unity_executables"

# -------------------- Detect Unity build platform --------------------
# Each Unity build ships as <name>_linux/ (*.x86_64) and <name>_windows/ (*.exe).
# WSL runs this same bash script but can still launch native Windows .exe via
# interop, so it needs the "windows" build, not the "linux" one.
detect_unity_platform() {
  if grep -qi microsoft /proc/version 2>/dev/null; then
    echo "windows"
  else
    case "$(uname -s)" in
      MINGW*|MSYS*|CYGWIN*) echo "windows" ;;
      *) echo "linux" ;;
    esac
  fi
}
UNITY_PLATFORM="$(detect_unity_platform)"
case "$UNITY_PLATFORM" in
  windows) UNITY_EXE_GLOB="*.exe" ;;
  *)       UNITY_EXE_GLOB="*.x86_64" ;;
esac
echo -e "${GREEN}[INFO] Unity build platform: $UNITY_PLATFORM${NC}"

# Map each world file to its Unity executable folder base name.
declare -A WORLD_UNITY_BASENAMES
WORLD_UNITY_BASENAMES["defenseScenario.world"]="silentStorm"
WORLD_UNITY_BASENAMES["energy.world"]="lotusimenergy"
# Accelerated variants keep world name "energy", so they reuse the same Unity build.
WORLD_UNITY_BASENAMES["energy_accelerated5x.world"]="lotusimenergy"
WORLD_UNITY_BASENAMES["energy_accelerated50x.world"]="lotusimenergy"
# bluerov_ekman.world's own header comment: "Derived from energy.world" — same scene.
WORLD_UNITY_BASENAMES["bluerov_ekman.world"]="lotusimenergy"
WORLD_UNITY_BASENAMES["bluerov_ekman_accelerated50x.world"]="lotusimenergy"

# -------------------- Functions --------------------
die() {
  echo -e "${RED}[ERROR] $1${NC}"
  exit 1
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

# Unity .exe launched from WSL runs as a native Windows process — it never
# shows up in `ps`/`pkill` inside WSL, so it needs killing via taskkill.exe.
kill_unity_processes() {
  if [[ "$UNITY_PLATFORM" == "windows" ]] && command_exists taskkill.exe; then
    for name in silentStorm.exe LOTUSimEnergy.exe; do
      taskkill.exe /IM "$name" /F >/dev/null 2>&1 \
        && echo -e "${GREEN}[INFO] Killed $name (Windows)${NC}" \
        || echo -e "${YELLOW}[INFO] No $name (Windows) found.${NC}"
    done
  else
    pkill -f "lotusim_unity_executables/.*\.x86_64" \
      && echo -e "${GREEN}[INFO] Killed lotusim_unity_executables/.*\.x86_64${NC}" \
      || echo -e "${YELLOW}[INFO] No lotusim_unity_executables/.*\.x86_64 found.${NC}"
  fi
}

# -------------------- Source ROS --------------------
[[ -f "$ROS_SETUP" ]] || die "ROS 2 setup not found at $ROS_SETUP. Is ROS 2 ${ROS_DISTRO^} installed?"
source "$ROS_SETUP"
source "$LOTUSIM_WS/install/setup.bash"
source "$LOTUSIM_SCENARIO_WS/install/setup.bash"

# Force this core workspace to the FRONT of every resolution path.
#
# Sourcing the overlay above is not enough: whatever a shell already had in
# these variables stays ahead, because colcon's prepend-unique dedupes an
# existing entry rather than moving it. Another overlay carrying a different
# lotusim_msgs then wins, and the two are not interchangeable:
#   * C++ (LD_LIBRARY_PATH): plugins load from this install
#     (GZ_SIM_SYSTEM_PLUGIN_PATH points here) while the message typesupport
#     comes from the other one, so the serializer walks a struct laid out by
#     the other definition — segfault on the first pose published.
#   * Python (PYTHONPATH/AMENT_PREFIX_PATH): tasks import the other
#     definition, so their subscription's type hash never matches the
#     publisher's and no pose is ever delivered.
if [[ -d "$LOTUSIM_WS/install/lib" ]]; then
    export LD_LIBRARY_PATH="$LOTUSIM_WS/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
_core_pythonpath="$LOTUSIM_WS/install/lib/python${PYTHON_VERSION:-3.12}/site-packages"
if [[ -d "$_core_pythonpath" ]]; then
    export PYTHONPATH="$_core_pythonpath${PYTHONPATH:+:$PYTHONPATH}"
fi
if [[ -d "$LOTUSIM_WS/install" ]]; then
    export AMENT_PREFIX_PATH="$LOTUSIM_WS/install${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
fi

# Apply domain ID globally
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export ROS_IP=$ROS_IP

# ============================================================
# Cleanup Old Processes
# ============================================================
echo -e "${YELLOW}[INFO] Cleaning up old processes...${NC}"
# Single-word patterns
for proc in xdyn-for-cs gzserver gzclient; do
    pkill -f "$proc" && echo -e "${GREEN}[INFO] Killed $proc${NC}" || echo -e "${YELLOW}[INFO] No $proc found.${NC}"
done
kill_unity_processes

# Multi-word patterns.
#   - "gz sim": the modern gazebo binary; "gzserver/gzclient" above do not match
#     it. A stale one double-publishes /<world>/poses and /stats.
#   - "simulation_run/lib/simulation_run/main": the agent-manager python process.
#     Its cmdline does not contain "ros2 run", so it is not matched above; if its
#     "ros2 run" wrapper died it is orphaned and keeps publishing velocity
#     commands on /<world>/vessel_cmd_array for the same vessel names as the new
#     run, so the host applies competing commands.
#   - "run_agent"/"ros_tcp_endpoint": decentralised agent launcher and Unity TCP
#     bridge, same orphaned-python-child case.
for proc in "ros2 launch" "ros2 run" "gz sim" "simulation_run/lib/simulation_run/main" "run_agent" "ros_tcp_endpoint"; do
    pkill -f "$proc" && echo -e "${GREEN}[INFO] Killed '$proc'${NC}" || echo -e "${YELLOW}[INFO] No '$proc' found.${NC}"
done
sleep 2
# Force-reap survivors (SIGTERM is unreliable for gz sim and an orphaned manager
# mid-callback) so the new run shares its ROS domain with no ghost.
for proc in "gz sim" "simulation_run/lib/simulation_run/main" "run_agent"; do
    pkill -9 -f "$proc" 2>/dev/null && echo -e "${GREEN}[INFO] Force-killed lingering '$proc'${NC}"
done
sleep 3


# ============================================================
# Parse Input Arguments
# ============================================================
CONFIG_FILE=""
DEBUG_MODE="false"
GZ_GUI="false"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --config) CONFIG_FILE="$2"; shift 2 ;;
        --debug) DEBUG_MODE="true"; shift ;;
        --gui) GZ_GUI="true"; shift ;;
        *) echo -e "${YELLOW}[WARN] Unknown argument: $1${NC}"; shift ;;
    esac
done

if [[ -z "$CONFIG_FILE" ]]; then
    die "Missing --config argument. Usage: $0 --config path/to/config.json"
fi

# Resolve relative path
if [[ ! -f "$CONFIG_FILE" ]]; then
    ALT_PATH="$CONFIG_DIR/$CONFIG_FILE"
    [[ -f "$ALT_PATH" ]] && CONFIG_FILE="$ALT_PATH" || die "Config file not found at $CONFIG_FILE or $ALT_PATH"
fi

echo -e "${YELLOW}[INFO] Loading config from: $CONFIG_FILE${NC}"
USE_UNITY=$(jq -r '.renderer_unity // false' "$CONFIG_FILE")
WORLD_FILE=$(jq -r '.world_file // ""' "$CONFIG_FILE")
# Ocean-current condition for Bluerov2_heavy_current agents (see XDYN_CONFIGS
# below). Lives in the scenario json like everything else here; BLUEROV_CURRENT
# in the environment is only an escape hatch for a one-off override, not the
# normal way to select it.
BLUEROV_CURRENT="${BLUEROV_CURRENT:-$(jq -r '.bluerov_current // "ekman"' "$CONFIG_FILE")}"
echo -e "${YELLOW}Unity Rendering: $USE_UNITY${NC}"
echo -e "${YELLOW}World file: $WORLD_FILE${NC}"
# The aerial world is not a config field: simulation_runner always launches
# utils.AERIAL_WORLD_FILE alongside the scenario's own world, whether or not
# the scenario has an aerial agent. Whether a scenario has wind is decided by
# whether anything publishes /aerialWorld/wind, not by a flag.
echo -e "${YELLOW}Bluerov current: $BLUEROV_CURRENT${NC}"

# -------------------- PX4 SITL --------------------
# The installer clones and builds PX4 into $HOME/PX4-Autopilot, which is also
# X500's own default (agents/entity/physical/x500.py). Resolve it here so the
# value the agents will use is printed once, up front, rather than discovered
# per agent; an explicit PX4_AUTOPILOT_PATH still wins, for a checkout kept
# elsewhere.
export PX4_AUTOPILOT_PATH="${PX4_AUTOPILOT_PATH:-$HOME/PX4-Autopilot}"
# .agents[] iterates values for both shapes the config allows, a list of
# agent objects and an object keyed by agent type, so this needs no split.
SCENARIO_USES_PX4=$(jq -r '[.agents[] | select(.px4 == true)] | length > 0' "$CONFIG_FILE")
if [[ "$SCENARIO_USES_PX4" == "true" ]]; then
    echo -e "${YELLOW}PX4-Autopilot: $PX4_AUTOPILOT_PATH${NC}"
    # Fail here rather than let every "px4": true agent report the missing
    # binary separately, several hundred log lines into the run.
    [[ -x "$PX4_AUTOPILOT_PATH/build/px4_sitl_default/bin/px4" ]] || die \
"This scenario has \"px4\": true agents but no PX4 SITL binary at
        $PX4_AUTOPILOT_PATH/build/px4_sitl_default/bin/px4
  Build it once with:
        (cd $PX4_AUTOPILOT_PATH && make px4_sitl gz_x500)
  or point PX4_AUTOPILOT_PATH at an existing built checkout."
fi

# ============================================================
# Snapshot the scenario config into $LOG_DIR/config/
# ============================================================
# So $LOG_DIR alone (config/ + main_simulation.log + csv/) is enough to
# understand or relaunch this exact run later, even if the live config, the
# world file, or the agent models have since changed — point a future
# --config at the copy saved here instead of the original.
CONFIG_SNAPSHOT_DIR="$LOG_DIR/config"
AGENTS_SNAPSHOT_DIR="$CONFIG_SNAPSHOT_DIR/agents"
mkdir -p "$AGENTS_SNAPSHOT_DIR"

cp "$CONFIG_FILE" "$CONFIG_SNAPSHOT_DIR/$(basename "$CONFIG_FILE")"

if [[ -n "$WORLD_FILE" ]]; then
    WORLD_FILE_PATH="$LOTUSIM_PATH/assets/worlds/$WORLD_FILE"
    if [[ -f "$WORLD_FILE_PATH" ]]; then
        cp "$WORLD_FILE_PATH" "$CONFIG_SNAPSHOT_DIR/$WORLD_FILE"
    else
        echo -e "${YELLOW}[WARN] World file not found at $WORLD_FILE_PATH — not saved to $LOG_DIR.${NC}"
    fi
fi

# One SDF per distinct (class, sdf_file) pair actually used, mirroring the
# assets/models/<model_dir>/ layout so agent instances sharing a class don't
# duplicate the copy. <model_dir> is the class name lowercased — the same
# convention every folder under assets/models/ already follows (bluerov2_heavy,
# mine, wamv, ...). Python's own class->folder mapping (each PhysicalEntity
# subclass's MODEL_NAME) isn't reachable from bash, so this mirrors the
# convention rather than importing it.
#
# Exception: agent classes whose model folder isn't their own lowercased name
# because they reuse another class's physical SDF/mesh (e.g. Bluerov2_heavy_current
# is a distinct xdyn wiring around the same assets/models/bluerov2_heavy/ as
# plain Bluerov2_heavy).
declare -A AGENT_CLASS_MODEL_DIR_OVERRIDES
AGENT_CLASS_MODEL_DIR_OVERRIDES["bluerov2_heavy_current"]="bluerov2_heavy"
if [[ "$(jq -r '.agents | type' "$CONFIG_FILE")" == "array" ]]; then
    AGENT_SDF_PAIRS=$(jq -r '.agents[] | (.class // .type // .id // "") as $c | (.sdf_file // "model.sdf") as $s | select($c | length > 0) | "\($c)|\($s)"' "$CONFIG_FILE")
else
    AGENT_SDF_PAIRS=$(jq -r '.agents | to_entries[] | .key as $c | (.value.sdf_file // "model.sdf") as $s | "\($c)|\($s)"' "$CONFIG_FILE")
fi
while IFS='|' read -r agent_class sdf_name; do
    [[ -n "$agent_class" ]] || continue
    model_dir=$(echo "$agent_class" | tr '[:upper:]' '[:lower:]')
    model_dir="${AGENT_CLASS_MODEL_DIR_OVERRIDES[$model_dir]:-$model_dir}"
    sdf_src="$LOTUSIM_MODELS_PATH/$model_dir/$sdf_name"
    if [[ -f "$sdf_src" ]]; then
        mkdir -p "$AGENTS_SNAPSHOT_DIR/$model_dir"
        cp "$sdf_src" "$AGENTS_SNAPSHOT_DIR/$model_dir/$sdf_name"
    else
        echo -e "${YELLOW}[WARN] SDF not found at $sdf_src for class '$agent_class' — not saved to $LOG_DIR.${NC}"
    fi
done <<< "$(echo "$AGENT_SDF_PAIRS" | sort -u)"

echo -e "${GREEN}[INFO] Saved scenario config + world file + agent SDFs to $CONFIG_SNAPSHOT_DIR${NC}"

# Resolve the Unity executable directory for this world (only needed if Unity is enabled)
UNITY_EXE_DIR=""
if [[ "$USE_UNITY" == "true" ]]; then
    UNITY_BASENAME="${WORLD_UNITY_BASENAMES[$WORLD_FILE]}"
    [[ -n "$UNITY_BASENAME" ]] || die "No Unity executable mapped for world_file '$WORLD_FILE'. Add an entry to WORLD_UNITY_BASENAMES in this script."
    UNITY_EXE_DIR="$UNITY_EXECUTABLES_DIR/${UNITY_BASENAME}_${UNITY_PLATFORM}"
fi

# ============================================================
# XDyn Config Map
# ============================================================
declare -A XDYN_CONFIGS
XDYN_CONFIGS["Dtmb_hull"]="$LOTUSIM_PATH/assets/models/dtmb_hull/dtmb-xdyn-faster.yml 12345"
XDYN_CONFIGS["Lrauv"]="$LOTUSIM_PATH/assets/models/lrauv/lrauv.yml 12346"
XDYN_CONFIGS["Bluerov2_heavy"]="$LOTUSIM_PATH/assets/models/bluerov2_heavy/BlueROV2.yml 12347"
XDYN_CONFIGS["Wamv"]="$LOTUSIM_PATH/assets/models/wamv/wamv.yaml 12348"
XDYN_CONFIGS["Fremm"]="$LOTUSIM_PATH/assets/models/fremm/fremmConfig.yaml 12349"
XDYN_CONFIGS["Mine"]="$LOTUSIM_PATH/assets/models/mine/mineConfig.yaml 12350"
XDYN_CONFIGS["Pha"]="$LOTUSIM_PATH/assets/models/pha/phaConfig.yaml 12351"
XDYN_CONFIGS["Commando"]="$LOTUSIM_PATH/assets/models/commando/commandoConfig.yaml 12352"

# Ocean-current runs (package src/external_packages/bluerov_gnc). xdyn
# model distinct from BlueROV2.yml: commands are thrusts in newtons instead
# of rpm, and the hydrostatics and thruster geometry are corrected. Same
# port as Bluerov2_heavy, so the two cannot run in the same scenario.
#
# BLUEROV_CURRENT (set above, from the scenario json's "bluerov_current")
# selects the current condition, by picking the matching model file:
#   "ekman" the layered, depth-resolved Ekman model;
#   "none"  no current, the control condition;
#   "gauss" a uniform first-order Gauss-Markov current. xdyn has no such
#           environment model, so that file declares NO current and the host
#           plugin injects the process instead, parametrised by the agent's
#           "gauss_markov_current" block in the scenario json.
#   "copernicus" replay of a measured Copernicus depth profile. Same story as
#           "gauss": no xdyn environment model, the file declares NO current,
#           and the host plugin injects the profile named by the agent's
#           "copernicus_current" block. Mutually exclusive with "gauss".
# See src/simulation_run/config/current_examples/README.md.
XDYN_CONFIGS["Bluerov2_heavy_current"]="$LOTUSIM_PATH/assets/models/bluerov2_heavy/BlueROV2_current_${BLUEROV_CURRENT}.yml 12347"

# ============================================================
# Agent Types
# ============================================================
# ".agents" is either a dict keyed by class name (legacy form) or a list of
# per-instance objects (mission system, e.g. [{"id","class","missions",...}]).
# `jq '.agents | keys[]'` only makes sense for the dict form — on an array it
# returns numeric indices ("0","1",...), and the later `.agents["$agent_type"]`
# lookup then fails with "Cannot index array with string". Detect the shape
# once and, for the list form, derive each entry's type the same way
# AgentsManager._iter_agents (Python) does: class, else type, else id.
AGENTS_IS_ARRAY="false"
if [[ "$(jq -r '.agents | type' "$CONFIG_FILE")" == "array" ]]; then
    AGENTS_IS_ARRAY="true"
    AGENT_TYPES=$(jq -r '.agents[] | (.class // .type // .id // "") | select(length > 0)' "$CONFIG_FILE") || die "Failed to parse agents"
else
    AGENT_TYPES=$(jq -r '.agents | keys[]' "$CONFIG_FILE") || die "Failed to parse agents"
fi
if [[ -z "$AGENT_TYPES" ]]; then
    echo -e "${YELLOW}[INFO] No agents to spawn in config.${NC}"
fi

# ============================================================
# Cleanup on Exit
# ============================================================
declare -a CHILD_PIDS=()
CLEANUP_DONE=""

cleanup() {
  # Re-entrancy guard: a second SIGINT/SIGTERM arriving while this function
  # is already running (e.g. an impatient extra Ctrl+C, or a signal bouncing
  # back from one of the pkills below) would otherwise re-enter cleanup()
  # from the top — observed producing several repeated "Cleaning up..."
  # cycles and, worse, a redundant kill sent to PYTHON_PID while it was
  # already mid-shutdown (a native rclpy/torch call in flight getting killed
  # out-of-band is exactly the segfault case the comment below warns about).
  # Disarm immediately so everything after this line runs at most once.
  if [[ -n "$CLEANUP_DONE" ]]; then
    return
  fi
  CLEANUP_DONE=1
  trap '' SIGINT SIGTERM

  echo -e "${YELLOW}[INFO] Cleaning up all child processes...${NC}"

  # simulation_run/main.py (PYTHON_PID) is in the same foreground process
  # group, so it already received this same SIGINT and is doing its own
  # graceful shutdown (draining the ROS executor, halting BT missions,
  # destroying nodes) — e.g. FaultInspectionTask may still be mid YOLO model
  # load/inference on a worker thread. Give it a moment to finish that BEFORE
  # the broad pkills below (in particular "ros2 run") force-kill it: killing
  # it out-of-band while a native call (torch/rclpy) is in flight aborts with
  # "terminate called without an active exception" and leaves an orphaned
  # process still printing after this script has already exited.
  if [[ -n "$PYTHON_PID" ]] && kill -0 "$PYTHON_PID" 2>/dev/null; then
    echo -e "${YELLOW}[INFO] Waiting for simulation_run (PID $PYTHON_PID) to shut down gracefully...${NC}"
    for _ in $(seq 1 100); do  # up to ~10s
      kill -0 "$PYTHON_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$PYTHON_PID" 2>/dev/null; then
      echo -e "${YELLOW}[WARN] simulation_run did not exit in time — forcing termination.${NC}"
      kill -TERM "$PYTHON_PID" 2>/dev/null
      sleep 1
      kill -KILL "$PYTHON_PID" 2>/dev/null
    fi
  fi

  pkill -f xdyn-for-cs
  pkill -f ros_tcp_endpoint
  kill_unity_processes
  pkill -f gzserver
  pkill -f gzclient
  pkill -f "gz sim"
  pkill -f "ros2 launch"
  pkill -f "ros2 run"
  # The agent-manager python child is orphaned if "ros2 run" is reaped first;
  # match it by its own cmdline so it can't linger and double-publish into the
  # next run (see the startup-cleanup comment for the full failure mode).
  pkill -f "simulation_run/lib/simulation_run/main"
  pkill -f yolo_server_corrosion_crack

  for pid in "${CHILD_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      echo -e "${YELLOW}[INFO] Killing PID $pid${NC}"
      kill "$pid"
    fi
  done
  echo -e "${GREEN}[INFO] Cleanup done.${NC}"
  exit 0
}
trap cleanup SIGINT SIGTERM

# ============================================================
# Unity Network Setup and ROS TCP Endpoint
# ============================================================
if [[ "$USE_UNITY" == "true" ]]; then
  echo -e "${YELLOW}[INFO] Unity rendering enabled — preparing ROS network bridge.${NC}"

  # Require user-defined ROS_IP
  if [[ -z "$ROS_IP" ]]; then
    echo -e "${RED}[ERROR] ROS_IP is not defined.${NC}"
    echo -e "${YELLOW}Please define your local IP address for Unity communication, e.g.:${NC}"
    echo -e "${YELLOW}  export ROS_IP=192.168.50.34${NC}"
    echo -e "${YELLOW}Or edit this script and fill in the ROS_IP variable at the top.${NC}"
    exit 1
  fi

  echo -e "${GREEN}[INFO] Using ROS_IP=$ROS_IP${NC}"
  export ROS_IP=$ROS_IP

  # Launch ROS–Unity bridge. Log at WARN by default: at DEBUG the endpoint logs
  # several lines per forwarded message, which at a high real_time_factor (pose
  # traffic scales with RTF) floods its single thread with disk I/O and stalls
  # forwarding to Unity. --debug restores DEBUG.
  TCP_LOG_LEVEL="WARN"
  [[ "$DEBUG_MODE" == "true" ]] && TCP_LOG_LEVEL="DEBUG"
  gnome-terminal -- bash -c "
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source \"$LOTUSIM_WS/install/setup.bash\"
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    export ROS_IP=$ROS_IP
    ros2 run ros_tcp_endpoint default_server_endpoint --address 0.0.0.0 --tcp_ip 127.0.0.1 --ros-args --log-level $TCP_LOG_LEVEL 2>&1 | tee \"$LOG_DIR/ros_tcp_endpoint.log\"
    exec bash
  " &
  CHILD_PIDS+=($!)
  echo -e "${GREEN}[INFO] ROS–Unity TCP bridge started (logging to $LOG_DIR/ros_tcp_endpoint.log).${NC}"
  sleep 1
fi

# ============================================================
# Launch XDyn
# ============================================================
declare -A XDYN_LAUNCHED

for agent_type in $AGENT_TYPES; do
    # Read xdyn field from JSON — same dict-vs-list distinction as above. In
    # the list form, several instances can share one "class" (e.g. 3
    # bluerovs); take the first matching entry's xdyn value.
    if [[ "$AGENTS_IS_ARRAY" == "true" ]]; then
        xdyn_enabled=$(jq -r --arg t "$agent_type" \
            '[.agents[] | select((.class // .type // .id // "") == $t) | .xdyn][0] // false' \
            "$CONFIG_FILE")
    else
        xdyn_enabled=$(jq -r ".agents[\"$agent_type\"].xdyn // false" "$CONFIG_FILE")
    fi

    if [[ "$xdyn_enabled" != "true" ]]; then
        echo -e "${YELLOW}[SKIP] XDyn disabled for $agent_type${NC}"
        continue
    fi

    # List form only: several instances share one class/type, but XDyn is
    # launched once per type (one process, one port) — skip if already done.
    if [[ "$AGENTS_IS_ARRAY" == "true" && -n "${XDYN_LAUNCHED[$agent_type]}" ]]; then
        echo -e "${YELLOW}[SKIP] XDyn for $agent_type already launched${NC}"
        continue
    fi

    # Find XDyn config
    xdyn_entry="${XDYN_CONFIGS[$agent_type]}"
    if [[ -z "$xdyn_entry" ]]; then
        for key in "${!XDYN_CONFIGS[@]}"; do
            if [[ "$agent_type" == *"$key"* ]]; then
                xdyn_entry="${XDYN_CONFIGS[$key]}"
                echo -e "${YELLOW}[INFO] No direct XDyn config for $agent_type — using $key${NC}"
                agent_type="$key"
                break
            fi
        done
    fi

    # Launch XDyn if config exists
    if [[ -n "$xdyn_entry" ]]; then
        read -r yml_file port <<< "$xdyn_entry"
        if [ -f "$yml_file" ]; then
            gnome-terminal -- bash -c "
                export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
                export ROS_IP=$ROS_IP
                xdyn-for-cs \"$yml_file\" --address 127.0.0.1 --port $port --dt 0.2 ${XDYN_VERBOSE:+-v -w} 2>&1 | tee \"$LOG_DIR/xdyn_${agent_type}.log\";
                exec bash
            " &
            CHILD_PIDS+=($!)
            echo -e "${GREEN}xdyn-for-cs for $agent_type started with PID ${CHILD_PIDS[-1]}${NC}"
            XDYN_LAUNCHED[$agent_type]=1
            sleep 0.5
        else
            echo -e "${YELLOW}[SKIP] YML file not found for $agent_type (${yml_file}).${NC}"
        fi
    else
        echo -e "${YELLOW}[SKIP] No XDyn config available for $agent_type${NC}"
    fi
done


# ============================================================
# Launch Unity Renderer
# ============================================================
if [[ "$USE_UNITY" == "true" ]]; then
  echo -e "${YELLOW}[INFO] Unity Mode: $UNITY_MODE${NC}"
  if [[ "$UNITY_MODE" == "exe" ]]; then
    UNITY_EXE_PATH=$(find "$UNITY_EXE_DIR" -maxdepth 1 -type f -name "$UNITY_EXE_GLOB" 2>/dev/null | head -n1)
    if [[ -n "$UNITY_EXE_PATH" && -f "$UNITY_EXE_PATH" ]]; then
      [[ "$UNITY_PLATFORM" == "linux" && ! -x "$UNITY_EXE_PATH" ]] && chmod +x "$UNITY_EXE_PATH"
      # shellcheck disable=SC2206  # intentional word-splitting of the arg string
      UNITY_ARGS=($UNITY_EXTRA_ARGS)
      echo -e "${GREEN}[INFO] Launching Unity executable: $UNITY_EXE_PATH ${UNITY_ARGS[*]}${NC}"
      "$UNITY_EXE_PATH" "${UNITY_ARGS[@]}" &
      CHILD_PIDS+=($!)
      sleep 1
    else
      die "No Unity $UNITY_EXE_GLOB executable found in '$UNITY_EXE_DIR'"
    fi
  else
    echo -e "${YELLOW}[INFO] Open Unity project manually in the Editor.${NC}"
  fi
fi

# ============================================================
# Launch Main Simulation
# ============================================================
CONFIG_BASENAME=$(basename "$CONFIG_FILE")
echo -e "${GREEN}[INFO] Using config: ${NC}$CONFIG_BASENAME"

DEBUG_ARG=""
if [[ "$DEBUG_MODE" == "true" ]]; then
    DEBUG_ARG="--debug"
fi

# Build argument list
ARGS=("--config" "$CONFIG_FILE")
[[ "$DEBUG_MODE" == "true" ]] && ARGS+=("--debug")
[[ "$GZ_GUI" == "true" ]] && ARGS+=("--gui")

echo -e "${YELLOW}[INFO] Launching simulation with debug: $DEBUG_MODE${NC}"

PYTHON_PID=""
# Single command with all arguments
ros2 run simulation_run main "${ARGS[@]}" &
PYTHON_PID=$!

CHILD_PIDS+=($PYTHON_PID)

# Wait for the Python simulation to finish
wait $PYTHON_PID
cleanup