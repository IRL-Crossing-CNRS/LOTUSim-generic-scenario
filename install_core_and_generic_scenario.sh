#!/usr/bin/env bash
# =============================================================================
#  LOTUSim Full Setup Script
#  Ubuntu 24.04  →  ROS 2 Jazzy   (also works in WSL2 — see README)
#  LOTUSim core + LOTUSim-generic-scenario
#  Optional: PX4 SITL for aerial X500 drones (question 3 / INSTALL_PX4=1)
#  Optional: YOLO detection on an NVIDIA GPU (question 4 / INSTALL_GPU=1)
#  Unity player(s) to download (question 5 / INSTALL_UNITY_PLAYER=linux|windows|all)
#
#  Installs the LOTUSim core (~/lotusim_ws) and the generic scenario
#  workspace (~/Documents/workspace/lotusim), built in their nix devshells,
#  with LOTUSIM_* shell variables and the lotusim_core / lotusim_shell
#  commands. Naval Group's own 'lotusim' program is left untouched.
# =============================================================================
set -euo pipefail
IFS=$'\n\t'

# -----------------------------------------------------------------------------
# Colour helpers
# -----------------------------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
info()    { echo -e "${CYAN}ℹ  $*${NC}"; }
success() { echo -e "${GREEN}✅ $*${NC}"; }
warn()    { echo -e "${YELLOW}⚠  $*${NC}"; }
die()     { echo -e "${RED}❌ $*${NC}" >&2; exit 1; }

# ROS setup files reference variables that may be unset (e.g. AMENT_TRACE_SETUP_FILES).
# Temporarily disable -u/-e around any 'source' of a ROS/colcon setup file.
ros_source() {
  local setup_file="$1"
  set +u +e
  # shellcheck source=/dev/null
  source "$setup_file"
  set -u -e
  success "Sourced $setup_file"
}

# colcon --symlink-install writes absolute symlinks into build/install/log.
# A workspace copied/synced from another machine carries dangling ones,
# which crash colcon. Wipe those dirs only when a dangling link is found.
# -----------------------------------------------------------------------------
purge_foreign_colcon_artifacts() {
  local ws="$1"
  local dangling=""
  local link
  # 'find -xtype l' is GNU-only; use the portable form below instead:
  # list every symlink inside build/install/log, then flag those that
  # no longer resolve ([[ -e ]] follows symlinks -> false when broken).
  while IFS= read -r link; do
    if [[ ! -e "$link" ]]; then
      dangling="$link"
      break
    fi
  done < <(find "$ws/build" "$ws/install" "$ws/log" -type l 2>/dev/null || true)
  if [[ -n "$dangling" ]]; then
    warn "Stale colcon artefacts found in $ws -- e.g. broken link: $dangling"
    warn "They were created on another machine (copied/synced folder) and would break the build. Removing them..."
    rm -rf "$ws/build" "$ws/install" "$ws/log"
    success "Removed stale build/, install/, log/ from $ws"
  fi
}

# A legacy ros2.list and the deb822 ros2.sources (from ros2-apt-source) can
# both define the ROS repo with a different Signed-By, which makes every
# 'apt-get update' abort with a Signed-By conflict. Must run before the
# first 'apt-get update'.
# -----------------------------------------------------------------------------
remove_conflicting_ros_repo_entries() {
  # Only act when a deb822 .sources definition of the ROS repo exists.
  local has_deb822=false
  if compgen -G "/etc/apt/sources.list.d/*.sources" >/dev/null; then
    if grep -qs "packages\.ros\.org" /etc/apt/sources.list.d/*.sources 2>/dev/null; then
      has_deb822=true
    fi
  fi
  [ "$has_deb822" = true ] || return 0

  # 1) Legacy one-line entries in dedicated .list files (ros2.list,
  #    ros-latest.list, ...). Keeping the package-managed deb822 definition
  #    means it survives future 'apt upgrade' of ros2-apt-source.
  local f
  for f in /etc/apt/sources.list.d/*.list; do
    [ -f "$f" ] || continue
    if grep -qs "packages\.ros\.org" "$f"; then
      warn "Removing conflicting legacy ROS repo entry $f"
      warn "(it declares a different Signed-By than the deb822 .sources entry"
      warn " from ros2-apt-source -- two definitions of one repo break apt)"
      sudo rm -f "$f"
    fi
  done

  # 2) Rare case: the entry was written directly into the main sources.list.
  if grep -qs "packages\.ros\.org" /etc/apt/sources.list 2>/dev/null; then
    warn "Commenting out packages.ros.org line(s) in /etc/apt/sources.list (backup kept as sources.list.bak)"
    sudo sed -i.bak '/packages\.ros\.org/ s/^deb/# deb/' /etc/apt/sources.list
  fi
}

# -----------------------------------------------------------------------------
# Check Ubuntu version — only Ubuntu 24.04 (ROS 2 Jazzy) is supported
# -----------------------------------------------------------------------------
UBUNTU_VERSION="$(lsb_release -rs 2>/dev/null || true)"

case "$UBUNTU_VERSION" in
  24.04)
    ROS_DISTRO="jazzy"
    ;;
  *)
    die "Unsupported OS: Ubuntu ${UBUNTU_VERSION:-unknown}. This script supports Ubuntu 24.04 (Jazzy) only."
    ;;
esac

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

echo "============================================================="
echo "  LOTUSim full setup"
echo "  Ubuntu ${UBUNTU_VERSION}  ->  ROS 2 ${ROS_DISTRO^}"
echo "============================================================="

# -----------------------------------------------------------------------------
# Variables
# -----------------------------------------------------------------------------
LOTUSIM_WS="$HOME/lotusim_ws"
LOTUSIM_SRC="$LOTUSIM_WS/src"
LOTUSIM_PATH="$LOTUSIM_SRC/LOTUSim"
SCENARIO_WS="$HOME/Documents/workspace/lotusim"
LOTUSIM_MODELS_PATH="$LOTUSIM_PATH/assets/models/"

# The public repositories, over HTTPS. LOTUSIM_CORE_REPO_URL /
# LOTUSIM_SCENARIO_REPO_URL override them, e.g. to test the installer against
# local mirrors.
CORE_REPO_URL="${LOTUSIM_CORE_REPO_URL:-https://github.com/IRL-Crossing-CNRS/LOTUSim}"
SCENARIO_REPO_URL="${LOTUSIM_SCENARIO_REPO_URL:-https://github.com/IRL-Crossing-CNRS/LOTUSim-generic-scenario}"

# Export so child processes (e.g. sudo -E) can see them
export LOTUSIM_WS LOTUSIM_SRC LOTUSIM_PATH SCENARIO_WS LOTUSIM_MODELS_PATH ROS_DISTRO

# The only system packages this install needs: git and its LFS (clones), ssh
# and curl (repository access, nix), jq (scenario_launch.sh reads the scenario
# JSON with it) and pip (the user mode builds the deployment wheels).
# ROS 2, Gazebo, colcon and every Python package the simulation imports come
# from the nix flakes, not from apt or pip.
# -----------------------------------------------------------------------------
info "Installing the system packages (git, git-lfs, openssh-client, curl, jq, pip)..."

# Must run BEFORE the first 'apt-get update': a legacy ros2.list sitting next
# to the deb822 ros2.sources (from the ros2-apt-source package) makes apt
# abort with "Conflicting values set for option Signed-By".
remove_conflicting_ros_repo_entries

# A third-party repository that fails to refresh (an expired key, a mirror
# down) makes 'apt-get update' exit non-zero; the packages above all come from
# Ubuntu, so carry on with the lists already on the machine.
sudo apt-get update -qq \
  || warn "apt-get update reported errors (a third-party repository?); continuing with the package lists already on this machine"
sudo apt-get install -y --no-install-recommends git git-lfs openssh-client curl lsb-release jq python3-pip
success "System packages installed"

# Must run before any clone: LFS-tracked files otherwise check out as pointer stubs.
info "Registering git-lfs..."
git lfs install
success "git-lfs registered"

# Check that both repositories are reachable before doing anything heavy.
# -----------------------------------------------------------------------------
check_repo_access() {
  local url="$1"
  info "Checking access to ${url}..."
  if ! git ls-remote "$url" HEAD &>/dev/null; then
    die "Cannot access ${url}
   Check the internet connection, then: git ls-remote ${url}"
  fi
  success "Access to ${url##*/} confirmed"
}
check_repo_access "$CORE_REPO_URL"
check_repo_access "$SCENARIO_REPO_URL"

# -----------------------------------------------------------------------------
# Question 1 — usage mode
# -----------------------------------------------------------------------------
echo ""
echo "-------------------------------------------------------------"
echo "  Would you like to use LOTUSim as:"
echo "    (1) a black-box to test some algorithms - plug and play (user)"
echo "    (2) a developer of the core / generic scenario (developer)"
echo "-------------------------------------------------------------"
USAGE_MODE=""
while [[ -z "$USAGE_MODE" ]]; do
  read -rp "Enter your choice [1/2]: " choice
  case "$choice" in
    1) USAGE_MODE="user" ;;
    2) USAGE_MODE="developer" ;;
    *) warn "Invalid choice '$choice' -- please enter 1 or 2" ;;
  esac
done
success "Usage mode: ${USAGE_MODE}"

# -----------------------------------------------------------------------------
# Question 2 — branch selection
# Asked separately per repo: the core and the generic scenario don't
# necessarily share branch names (e.g. core on 'dev', scenario on
# 'feat/wave-sync'), so picking from one shared, identically-named list
# would hide the branch a user actually wants in the other repo.
# -----------------------------------------------------------------------------
# LC_ALL=C forces plain byte-order sorting, independent of the machine locale.
prompt_branch() {
  local repo_url="$1" repo_label="$2"
  local branches
  branches="$(git ls-remote --heads "$repo_url" | awk '{print $2}' | sed 's#refs/heads/##' | LC_ALL=C sort)"
  [[ -n "$branches" ]] || die "No branches found on ${repo_url}."

  local options=()
  while IFS= read -r b; do options+=("$b"); done <<< "$branches"

  {
    echo ""
    echo "-------------------------------------------------------------"
    echo "  What branch of ${repo_label} would you like to be on?"
    echo "-------------------------------------------------------------"
  } >&2
  local picked=""
  PS3="Enter the branch number: "
  select picked in "${options[@]}"; do
    if [[ -n "${picked:-}" ]]; then
      break
    else
      warn "Invalid selection -- please enter a valid number" >&2
    fi
  done
  echo "$picked"
}

info "Listing branches on ${CORE_REPO_URL##*/}..."
CORE_BRANCH="$(prompt_branch "$CORE_REPO_URL" "LOTUSim (core)")"
success "Core branch: ${CORE_BRANCH}"

info "Listing branches on ${SCENARIO_REPO_URL##*/}..."
SCENARIO_BRANCH="$(prompt_branch "$SCENARIO_REPO_URL" "LOTUSim-generic-scenario")"
success "Scenario branch: ${SCENARIO_BRANCH}"

# -----------------------------------------------------------------------------
# Question 3 — PX4 SITL support (optional, aerial X500 drones)
#
# An X500 agent with "px4": true in its scenario JSON is flown by an external
# PX4 SITL process (see the LOTUSim-generic-scenario README, section
# "PX4 SITL (aerial drones)"). PX4 is a separate checkout built once; this
# step automates exactly what that README describes.
#
# Non-interactive override: INSTALL_PX4=1 ./install_core_and_generic_scenario.sh
# -----------------------------------------------------------------------------
if [[ "${INSTALL_PX4:-}" == "1" || "${INSTALL_PX4:-}" == "0" ]]; then
  INSTALL_PX4_FLAG="${INSTALL_PX4}"
else
  echo ""
  echo "-------------------------------------------------------------"
  echo "  Install PX4 SITL support (aerial X500 drones)?"
  echo ""
  echo "  Clones PX4-Autopilot to ~/PX4-Autopilot and builds it"
  echo "  (first build takes roughly 30-60 minutes)."
  echo "  Only needed for scenarios whose agents set \"px4\": true;"
  echo "  every other agent type is unaffected."
  echo "-------------------------------------------------------------"
  INSTALL_PX4_FLAG=""
  while [[ -z "$INSTALL_PX4_FLAG" ]]; do
    read -rp "Install PX4 SITL? [y/N]: " yn
    case "$yn" in
      [yY]|[yY][eE][sS])  INSTALL_PX4_FLAG="1" ;;
      [nN]|[nN][oO]|"")   INSTALL_PX4_FLAG="0" ;;
      *) warn "Please answer y or n" ;;
    esac
  done
fi
if [[ "$INSTALL_PX4_FLAG" == "1" ]]; then
  success "PX4 SITL: will be installed"
else
  info "PX4 SITL: skipped (re-run with INSTALL_PX4=1 to add it)"
fi

# -----------------------------------------------------------------------------
# Question 4 — YOLO detection on the GPU (optional)
#
# The crack and corrosion detection tasks run on the CPU by default. On a
# machine with an NVIDIA GPU and its driver, the scenario workspace's "gpu"
# devshell runs them with CUDA instead, for about 7 GB more of downloads.
# The choice is stored in ~/.config/lotusim/devshell ("default" or
# "gpu"), which lotusim_shell reads; edit it to switch later.
#
# Non-interactive override: INSTALL_GPU=1 ./install_core_and_generic_scenario.sh
# -----------------------------------------------------------------------------
HAS_NVIDIA_GPU=0
if command -v nvidia-smi &>/dev/null && nvidia-smi -L &>/dev/null; then
  HAS_NVIDIA_GPU=1
fi
if [[ "${INSTALL_GPU:-}" == "1" || "${INSTALL_GPU:-}" == "0" ]]; then
  INSTALL_GPU_FLAG="${INSTALL_GPU}"
else
  echo ""
  echo "-------------------------------------------------------------"
  echo "  Run the YOLO detection on an NVIDIA GPU?"
  echo ""
  echo "  Downloads NVIDIA's CUDA build of PyTorch (about 7 GB more)."
  echo "  Without it the crack and corrosion detection runs on the CPU,"
  echo "  slower but with the same results."
  if [[ "$HAS_NVIDIA_GPU" == "1" ]]; then
    echo "  An NVIDIA GPU was found on this machine."
  else
    echo "  No NVIDIA GPU (nvidia-smi) was found on this machine."
  fi
  echo "-------------------------------------------------------------"
  if [[ "$HAS_NVIDIA_GPU" == "1" ]]; then GPU_PROMPT="[Y/n]"; GPU_DEFAULT="1"; else GPU_PROMPT="[y/N]"; GPU_DEFAULT="0"; fi
  INSTALL_GPU_FLAG=""
  while [[ -z "$INSTALL_GPU_FLAG" ]]; do
    read -rp "Use the GPU for detection? ${GPU_PROMPT}: " yn
    case "$yn" in
      [yY]|[yY][eE][sS])  INSTALL_GPU_FLAG="1" ;;
      [nN]|[nN][oO])      INSTALL_GPU_FLAG="0" ;;
      "")                 INSTALL_GPU_FLAG="$GPU_DEFAULT" ;;
      *) warn "Please answer y or n" ;;
    esac
  done
fi
if [[ "$INSTALL_GPU_FLAG" == "1" ]]; then
  SCENARIO_DEVSHELL="gpu"
  success "Detection: on the GPU"
else
  SCENARIO_DEVSHELL="default"
  info "Detection: on the CPU (re-run with INSTALL_GPU=1 to use the GPU)"
fi

# -----------------------------------------------------------------------------
# Question 5 — which Unity player to download
#
# The scenario repository ships the Unity player built for Linux and for
# Windows, about 1 GB each, and Git LFS keeps a second copy of what it
# downloads. Only the player this machine runs is needed: scenario_launch.sh
# starts the Linux one on Linux and the Windows one under WSL (through
# interop), so that is the default answer.
#
# Non-interactive override: INSTALL_UNITY_PLAYER=linux|windows|all
# -----------------------------------------------------------------------------
if grep -qi microsoft /proc/version 2>/dev/null; then
  DETECTED_UNITY_PLAYER="windows"
else
  DETECTED_UNITY_PLAYER="linux"
fi
case "${INSTALL_UNITY_PLAYER:-}" in
  linux|windows|all) UNITY_PLAYER="$INSTALL_UNITY_PLAYER" ;;
  *)
    echo ""
    echo "-------------------------------------------------------------"
    echo "  Which Unity player should be downloaded?"
    echo ""
    echo "  (1) Linux     (2) Windows     (3) both"
    echo "  This machine runs the ${DETECTED_UNITY_PLAYER} one; each player is about"
    echo "  1 GB, stored twice by Git LFS."
    echo "-------------------------------------------------------------"
    if [[ "$DETECTED_UNITY_PLAYER" == "windows" ]]; then UNITY_DEFAULT_CHOICE="2"; else UNITY_DEFAULT_CHOICE="1"; fi
    UNITY_PLAYER=""
    while [[ -z "$UNITY_PLAYER" ]]; do
      read -rp "Enter your choice [1/2/3, default ${UNITY_DEFAULT_CHOICE}]: " choice
      case "${choice:-$UNITY_DEFAULT_CHOICE}" in
        1) UNITY_PLAYER="linux" ;;
        2) UNITY_PLAYER="windows" ;;
        3) UNITY_PLAYER="all" ;;
        *) warn "Please answer 1, 2 or 3" ;;
      esac
    done
    ;;
esac
if [[ "$UNITY_PLAYER" != "all" && "$UNITY_PLAYER" != "$DETECTED_UNITY_PLAYER" ]]; then
  warn "Unity player: ${UNITY_PLAYER}, but scenario_launch.sh will look for the ${DETECTED_UNITY_PLAYER} one on this machine"
fi
success "Unity player: ${UNITY_PLAYER}"

# git lfs pull and checkouts skip the excluded player, leaving its files as
# small pointer files.
case "$UNITY_PLAYER" in
  linux)   UNITY_LFS_EXCLUDE="lotusim_unity_executables/*_windows/**" ;;
  windows) UNITY_LFS_EXCLUDE="lotusim_unity_executables/*_linux/**" ;;
  *)       UNITY_LFS_EXCLUDE="" ;;
esac

# -----------------------------------------------------------------------------
# Create workspace and clone LOTUSim core
# -----------------------------------------------------------------------------
check_existing_remote() {
  local dir="$1" expected_url="$2"
  local current_url
  current_url="$(git -C "$dir" remote get-url origin 2>/dev/null || echo '<none>')"
  # Tolerate the missing-.git-suffix variant of the same URL
  if [[ "${current_url%.git}" != "${expected_url%.git}" ]]; then
    die "Directory $dir already exists but its 'origin' is:
     ${current_url}
   expected:
     ${expected_url}
   Move or remove it, then re-run this script."
  fi
}

info "Creating LOTUSim workspace at ${LOTUSIM_WS}..."
mkdir -p "$LOTUSIM_SRC"
cd "$LOTUSIM_SRC"

if [[ ! -d "LOTUSim/.git" ]]; then
  info "Cloning LOTUSim core (branch: ${CORE_BRANCH})..."
  git clone -b "$CORE_BRANCH" "$CORE_REPO_URL"
  success "LOTUSim cloned"
else
  check_existing_remote "LOTUSim" "$CORE_REPO_URL"
  info "LOTUSim already present -- fetching latest changes..."
  git -C LOTUSim fetch origin "$CORE_BRANCH" 2>/dev/null \
    || warn "Could not fetch LOTUSim (network issue?); continuing with existing version"
fi

# Always end up on the requested branch, regardless of the repo's prior state
info "Checking out branch '${CORE_BRANCH}' in LOTUSim..."
git -C LOTUSim checkout "$CORE_BRANCH"
git -C LOTUSim merge --ff-only "origin/$CORE_BRANCH" 2>/dev/null \
  || warn "Could not fast-forward LOTUSim on branch ${CORE_BRANCH} (local changes?); continuing with existing version"
success "LOTUSim on branch ${CORE_BRANCH}"

# Sanity-check expected directory structure
[[ -f "$LOTUSIM_PATH/flake.nix" && -f "$LOTUSIM_PATH/mise.toml" ]] \
  || die "Expected $LOTUSIM_PATH/flake.nix and mise.toml not found after clone. Check the repo structure."

# -----------------------------------------------------------------------------
# ~/.bashrc configuration (idempotent)
# ROS_DISTRO and UBUNTU_VERSION are embedded literally at write-time.
# All other variables use escaped $ so they expand at shell startup.
#
# The block defines the LOTUSIM_* variables and two commands:
#   - 'lotusim_core', which runs the core's tasks (build, test, sim <world>,
#     ...) inside the core's nix devshell
#   - 'lotusim_shell', which opens the scenario workspace's devshell
# Neither is called 'lotusim': that is Naval Group's own program. The block
# sources no ROS 2: ROS 2, Gazebo and xdyn come from the core's devshell.
# -----------------------------------------------------------------------------
BASHRC_MARKER="# >>> LOTUSim setup >>>"
BASHRC_END="# <<< LOTUSim setup <<<"

# A block written by an earlier installer is replaced rather than kept: the
# apt-based one sources a system ROS 2 and points at directories the core no
# longer has, and the first nix one always opened the CPU devshell.
if grep -qF "$BASHRC_MARKER" ~/.bashrc && ! grep -qF ".config/lotusim/devshell" ~/.bashrc; then
  info "Replacing the previous LOTUSim block in ~/.bashrc (backup: ~/.bashrc.lotusim.bak)..."
  cp ~/.bashrc ~/.bashrc.lotusim.bak
  sed -i "/^${BASHRC_MARKER}\$/,/^${BASHRC_END}\$/d" ~/.bashrc
fi

if ! grep -qF "$BASHRC_MARKER" ~/.bashrc; then
  info "Adding LOTUSim block to ~/.bashrc..."
  cat >> ~/.bashrc <<BASHRC_BLOCK

# >>> LOTUSim setup >>>
export LOTUSIM_WS="\$HOME/lotusim_ws"
export LOTUSIM_PATH="\$LOTUSIM_WS/src/LOTUSim"
# Trailing slash is load-bearing: xdyn concatenates it onto mesh paths.
export LOTUSIM_MODELS_PATH="\$LOTUSIM_PATH/assets/models/"
export LOTUSIM_SCENARIO_WS="\$HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario"

# 'lotusim_core <task> [args]' runs a task of the core in its devshell:
#   lotusim_core build | test | check | sim [--gui] <world>
lotusim_core() {
  ( cd "\$LOTUSIM_PATH" && nix develop --command mise run "\$@" )
}

# 'lotusim_shell' opens the scenario workspace's devshell, built
# against the core, with both workspaces sourced: scenario_launch.sh is
# run from there. ~/.config/lotusim/devshell names the devshell:
# "default" (detection on the CPU) or "gpu".
lotusim_shell() {
  local devshell
  devshell="\$(cat "\$HOME/.config/lotusim/devshell" 2>/dev/null || echo default)"
  cd "\$LOTUSIM_SCENARIO_WS" && \\
    nix develop ".#\$devshell" --override-input lotusim "\$LOTUSIM_PATH" \\
      --command bash --rcfile "\$HOME/.config/lotusim/shellrc"
}
# <<< LOTUSim setup <<<
BASHRC_BLOCK
  success "~/.bashrc updated"
else
  info "~/.bashrc already contains LOTUSim config -- skipping"
fi

# The rc file lotusim_shell starts its shell with. It sources the two
# workspaces only, never ~/.bashrc: an older block there may source a system
# ROS 2, which must not be mixed with the devshell's.
mkdir -p "$HOME/.config/lotusim"
echo "$SCENARIO_DEVSHELL" > "$HOME/.config/lotusim/devshell"
cat > "$HOME/.config/lotusim/shellrc" <<'SHELLRC'
PS1="(lotusim_core) $PS1"
[[ -f "$LOTUSIM_PATH/install/setup.bash" ]] && source "$LOTUSIM_PATH/install/setup.bash"
[[ -f install/setup.bash ]] && source install/setup.bash
SHELLRC

# Apply exports for the remainder of this session
export LOTUSIM_MODELS_PATH="$LOTUSIM_PATH/assets/models/"

# -----------------------------------------------------------------------------
# Make launch scripts executable
# -----------------------------------------------------------------------------
info "Setting execute permissions on launch scripts..."
chmod +x "$LOTUSIM_PATH"/scripts/*.sh 2>/dev/null || true
success "Permissions set"

# -----------------------------------------------------------------------------
# Nix — the core provides ROS 2, Gazebo and xdyn through its flake, so this is
# the only toolchain the installer puts on the machine.
# -----------------------------------------------------------------------------
if command -v nix &>/dev/null; then
  info "nix already installed -- skipping"
else
  info "Installing nix (multi-user; this asks for your password)..."
  curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install | sh -s -- --daemon
  success "nix installed"
fi

# shellcheck disable=SC1091
[[ -f /etc/profile.d/nix.sh ]] && ros_source /etc/profile.d/nix.sh
export PATH="/nix/var/nix/profiles/default/bin:$PATH"

# Flakes are what "nix develop" needs; the binary caches are what keep it
# minutes rather than hours: ROS 2 and Gazebo come from the ROS cache, xdyn
# from Naval Group's (the same two caches as the core's README).
NIX_CONF=/etc/nix/nix.conf
# A single-user nix, or one installed by hand, may have no /etc/nix at all.
sudo mkdir -p /etc/nix
if ! grep -q 'experimental-features' "$NIX_CONF" 2>/dev/null; then
  info "Enabling flakes..."
  echo 'experimental-features = nix-command flakes' | sudo tee -a "$NIX_CONF" >/dev/null
  NIX_CONF_CHANGED=1
fi
if ! grep -q 'ros.cachix.org' "$NIX_CONF" 2>/dev/null; then
  info "Adding the ROS binary cache..."
  sudo tee -a "$NIX_CONF" >/dev/null <<'NIXCONF'
extra-substituters = https://ros.cachix.org
extra-trusted-public-keys = ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=
NIXCONF
  NIX_CONF_CHANGED=1
fi
if ! grep -q 'naval-group.cachix.org' "$NIX_CONF" 2>/dev/null; then
  info "Adding Naval Group's binary cache..."
  sudo tee -a "$NIX_CONF" >/dev/null <<'NIXCONF'
extra-substituters = https://naval-group.cachix.org
extra-trusted-public-keys = naval-group.cachix.org-1:ytTEzFEeuzQrC9IRYLzHGa5OnM65G95M6/sbPd0fy28=
NIXCONF
  NIX_CONF_CHANGED=1
fi
if [[ "${NIX_CONF_CHANGED:-0}" == "1" ]]; then
  sudo systemctl restart nix-daemon || warn "Could not restart nix-daemon; restart it yourself before continuing."
fi

# -----------------------------------------------------------------------------
# Build the core, inside its own flake environment. The core builds at its own
# root, and the scenario packages depend on lotusim_msgs, which only exists
# after this build. The first entry into the devshell downloads the whole ROS 2
# and Gazebo closure.
# -----------------------------------------------------------------------------
CORE_INSTALL="$LOTUSIM_PATH/install"
if [[ ! -f "$CORE_INSTALL/setup.bash" ]]; then
  info "Building the LOTUSim core in its devshell (the first run downloads a lot)..."
  purge_foreign_colcon_artifacts "$LOTUSIM_PATH"
  ( cd "$LOTUSIM_PATH" && nix develop --command mise run build ) \
    || die "Core build failed. Enter it yourself with: cd $LOTUSIM_PATH && nix develop"
  success "Core built"
else
  info "Core already built -- skipping (rebuild with 'nix develop' then 'mise run build')"
fi

# -----------------------------------------------------------------------------
# Scenario workspace -- clone & update submodules
# -----------------------------------------------------------------------------
info "Setting up scenario workspace at ${SCENARIO_WS}..."
mkdir -p "$SCENARIO_WS"
cd "$SCENARIO_WS"

if [[ ! -d "LOTUSim-generic-scenario/.git" ]]; then
  info "Cloning LOTUSim-generic-scenario (branch: ${SCENARIO_BRANCH})..."
  # Cloned without its LFS content: the player not chosen above is excluded
  # before anything is downloaded.
  GIT_LFS_SKIP_SMUDGE=1 git clone --recurse-submodules -b "$SCENARIO_BRANCH" "$SCENARIO_REPO_URL"
  success "Scenario repo cloned"
else
  check_existing_remote "LOTUSim-generic-scenario" "$SCENARIO_REPO_URL"
  info "Scenario repo already present -- fetching latest changes..."
  git -C LOTUSim-generic-scenario fetch origin "$SCENARIO_BRANCH" 2>/dev/null \
    || warn "Could not fetch LOTUSim-generic-scenario (network issue?); continuing with existing version"
fi

cd LOTUSim-generic-scenario

# Always end up on the requested branch, regardless of the repo's prior state
info "Checking out branch '${SCENARIO_BRANCH}' in LOTUSim-generic-scenario..."
git checkout "$SCENARIO_BRANCH"
git merge --ff-only "origin/$SCENARIO_BRANCH" 2>/dev/null \
  || warn "Could not fast-forward LOTUSim-generic-scenario on branch ${SCENARIO_BRANCH} (local changes?); continuing with existing version"
success "LOTUSim-generic-scenario on branch ${SCENARIO_BRANCH}"

git submodule update --init --remote --merge
success "Submodules up to date"

if [[ -n "$UNITY_LFS_EXCLUDE" ]]; then
  git config lfs.fetchexclude "$UNITY_LFS_EXCLUDE"
else
  git config --unset lfs.fetchexclude 2>/dev/null || true
fi
info "Ensuring the LFS content is fetched (Unity ${UNITY_PLAYER} player, models)..."
git lfs pull
success "LFS content up to date"

# -----------------------------------------------------------------------------
# Build
#
# purge_foreign_colcon_artifacts(): a scenario folder copied/synced from
# another computer carries stale build/, install/, log/ full of absolute
# symlinks that dangle here and crash colcon (e.g. "can't copy ... doesn't
# exist or not a regular file"). Healthy same-machine incremental builds are
# left untouched.
# -----------------------------------------------------------------------------
# Built inside the core's devshell, and against the core it just produced: a
# workspace built against any other ROS 2 gets different message definitions and
# never exchanges a message with the simulation.
PWD_SCENARIO="$PWD"
# This workspace has its own devshell: the core's, plus the Python packages the
# agents import. Its lotusim input is pointed at the core just cloned, so both
# are built against one ROS 2 rather than against whatever github holds.
info "Building LOTUSim generic scenario with colcon, in its devshell..."
purge_foreign_colcon_artifacts "$PWD"
( cd "$PWD_SCENARIO" && nix develop ".#$SCENARIO_DEVSHELL" --override-input lotusim "$LOTUSIM_PATH" \
    --command bash -c \
    "source '$LOTUSIM_PATH/install/setup.bash' && colcon build --symlink-install" ) \
  || die "Scenario build failed. Enter the shell yourself with: cd $PWD_SCENARIO && nix develop --override-input lotusim $LOTUSIM_PATH"
success "Scenario built"

# -----------------------------------------------------------------------------
# PX4 SITL (optional — answer 3 / INSTALL_PX4=1)
#
# Automates the manual steps documented in the LOTUSim-generic-scenario
# README ("PX4 SITL (aerial drones)" section):
#   1. system + Python build dependencies
#   2. clone ~/PX4-Autopilot (recursive)
#   3. known clang>=18 VLA fix
#   4. make px4_sitl gz_x500
#   5. persist PX4_AUTOPILOT_PATH in ~/.bashrc
# Idempotent: an existing built checkout is only submodule-updated.
# -----------------------------------------------------------------------------
if [[ "$INSTALL_PX4_FLAG" == "1" ]]; then
  PX4_DIR="$HOME/PX4-Autopilot"

  # An interrupted install can leave $PX4_DIR without .git (partial build/
  # tree only). 'git clone' then refuses to clone into it. Wipe such
  # leftovers, but never touch a directory with unexpected content.
  if [[ -e "$PX4_DIR" && ! -d "$PX4_DIR/.git" ]]; then
    PX4_STRAY="$(find "$PX4_DIR" -mindepth 1 -maxdepth 1 \
                  ! -name build ! -name logs ! -name .git -print -quit 2>/dev/null || true)"
    if [[ -n "$PX4_STRAY" ]]; then
      die "Found '$PX4_DIR' without a .git directory but containing unexpected
   content (e.g. $PX4_STRAY). Inspect and move/remove it manually, then re-run."
    fi
    warn "'$PX4_DIR' is a leftover from an interrupted install (no .git, only regenerable build artefacts)"
    warn "Removing it so PX4-Autopilot can be cloned cleanly..."
    rm -rf "$PX4_DIR"
  fi

  # A plain 'git clone --recursive' of PX4-Autopilot costs ~4.4 GB, and a SITL
  # build uses a fraction of it: the git history, the NuttX board sources and
  # the three simulators LOTUSim never launches are most of the weight. None of
  # them is referenced by any target in the generated px4_sitl_default build,
  # and the version string survives a tagless clone because PX4 generates it
  # with 'git describe --always' (src/lib/version/px_update_git_header.py).
  # Net: ~1.1 GB instead of ~4.4 GB, still rebuildable in place.
  # ---------------------------------------------------------------------------
  PX4_UNUSED_SUBMODULES=(
    platforms/nuttx/NuttX/nuttx                          # flight-controller board sources; SITL is a POSIX build
    platforms/nuttx/NuttX/apps
    Tools/simulation/gazebo-classic/sitl_gazebo-classic  # simulators LOTUSim never launches; it uses
    Tools/simulation/jmavsim/jMAVSim                     # Tools/simulation/gz, which stays
    Tools/simulation/flightgear/flightgear_bridge
    Tools/simulation/jsbsim/jsbsim_bridge
  )

  # Every submodule .gitmodules records, minus those. There is no way to tell
  # 'git submodule update --init' to skip one: --init runs 'submodule init',
  # which sets submodule.<name>.active=true and overrides a 'false' set
  # beforehand (checked on git 2.43), and a plain 'deinit' is undone the same
  # way. Naming the wanted paths explicitly is what actually holds.
  px4_wanted_submodules() {
    local skip_list
    skip_list="$(printf '%s\n' "${PX4_UNUSED_SUBMODULES[@]}")"
    git -C "$PX4_DIR" config -f .gitmodules --get-regexp '^submodule\..*\.path$' 2>/dev/null \
      | awk '{print $2}' | grep -vxF "$skip_list"
  }

  if [[ ! -d "$PX4_DIR/.git" ]]; then
    info "Cloning PX4-Autopilot into $PX4_DIR (shallow, SITL submodules only)..."
    git clone --depth 1 https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" \
      || die "Failed to clone PX4-Autopilot. Check your internet connection."
    success "PX4-Autopilot cloned"
  else
    info "PX4-Autopilot already present at $PX4_DIR -- updating submodules..."
  fi

  # Self-repair: (re-)populate the submodules the build needs. 'git clone' can
  # silently leave one empty after a flaky fetch, which breaks CMake configure
  # with a cascade of 'non-existent target' errors. Idempotent, cheap when
  # already healthy.
  # A shallow submodule fetch asks the server for an arbitrary SHA; github.com
  # serves those, but a mirror or corporate proxy may not -- fall back to full
  # depth rather than fail the install over disk space we only meant to save.
  # ---------------------------------------------------------------------------
  info "Populating/verifying PX4 submodules..."
  mapfile -t PX4_WANTED_SUBMODULES < <(px4_wanted_submodules)
  [[ ${#PX4_WANTED_SUBMODULES[@]} -gt 0 ]] \
    || die "No submodule paths found in $PX4_DIR/.gitmodules -- the clone looks incomplete."
  git -C "$PX4_DIR" submodule sync --recursive || true
  git -C "$PX4_DIR" submodule update --init --recursive --depth 1 --force \
       -- "${PX4_WANTED_SUBMODULES[@]}" \
    || git -C "$PX4_DIR" submodule update --init --recursive --force \
       -- "${PX4_WANTED_SUBMODULES[@]}" \
    || die "PX4 submodules could not be synced (network?). Re-run this script once online."
  success "PX4-Autopilot submodules up to date"

  # 'submodule status' prefixes an unpopulated submodule with '-' and NO space
  # before the SHA, so the '^- ' this check used to look for never matched
  # anything. With '^-' it does -- including the ones we skip on purpose, which
  # therefore have to come back out before judging what is left.
  BROKEN_SUBS="$(git -C "$PX4_DIR" submodule status --recursive | grep -E '^-' || true)"
  for PX4_SKIPPED in "${PX4_UNUSED_SUBMODULES[@]}"; do
    BROKEN_SUBS="$(printf '%s\n' "$BROKEN_SUBS" | grep -v -F " ${PX4_SKIPPED}" || true)"
  done
  if [[ -n "$(printf '%s' "$BROKEN_SUBS" | tr -d '[:space:]')" ]]; then
    warn "These PX4 submodules are STILL empty after repair:"
    echo "$BROKEN_SUBS"
    die "Re-run this script with a stable internet connection to github.com."
  fi

  # A checkout from before this change still carries the unused submodules.
  # Deleting ~580 MB someone may be using is not this script's call, so just
  # point at the command.
  PX4_PRUNABLE=()
  for PX4_SKIPPED in "${PX4_UNUSED_SUBMODULES[@]}"; do
    [[ -n "$(ls -A "$PX4_DIR/$PX4_SKIPPED" 2>/dev/null)" ]] && PX4_PRUNABLE+=("$PX4_SKIPPED")
  done
  if [[ ${#PX4_PRUNABLE[@]} -gt 0 ]]; then
    info "This checkout carries ${#PX4_PRUNABLE[@]} submodule(s) no SITL build uses (~580 MB for the full set)."
    info "Reclaim the space when you want to, with:"
    for PX4_SKIPPED in "${PX4_PRUNABLE[@]}"; do
      info "  git -C $PX4_DIR submodule deinit -f $PX4_SKIPPED"
    done
  fi

  # In-source CMake fossils (CMakeCache.txt/CMakeFiles/cmake_install.cmake in
  # the repo root) break later 'make' runs. PX4 is out-of-source only.
  # ---------------------------------------------------------------------------
  for PX4_FOSSIL in "$PX4_DIR/CMakeCache.txt" "$PX4_DIR/CMakeFiles" "$PX4_DIR/cmake_install.cmake"; do
    if [[ -e "$PX4_FOSSIL" ]]; then
      warn "Removing in-source CMake leftover $PX4_FOSSIL"
      rm -rf "$PX4_FOSSIL"
    fi
  done

  # A build dir copied/synced from another machine/path caches the old
  # toolchain settings. If the recorded source dir differs from here, wipe it.
  # ---------------------------------------------------------------------------
  PX4_CACHE_FILE="$PX4_DIR/build/px4_sitl_default/CMakeCache.txt"
  if [[ -f "$PX4_CACHE_FILE" ]]; then
    PX4_CACHED_HOME="$(grep -m1 '^CMAKE_HOME_DIRECTORY' "$PX4_CACHE_FILE" | cut -d= -f2-)"
    if [[ -n "$PX4_CACHED_HOME" && "$PX4_CACHED_HOME" != "$PX4_DIR" ]]; then
      warn "$PX4_CACHE_FILE references '$PX4_CACHED_HOME'"
      warn "-- a build configuration from another location/machine. Removing it..."
      rm -rf "$PX4_DIR/build"
      success "Foreign PX4 build configuration removed"
    fi
  fi

  # A stale/half-finished build directory is not worth diagnosing: remove it so
  # CMake configures from scratch. (Only reached when we just repaired things --
  # an already-built binary below short-circuits before any of this matters.)
  if [[ -d "$PX4_DIR/build" && ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
    warn "Removing incomplete $PX4_DIR/build (previous build never finished)..."
    rm -rf "$PX4_DIR/build"
    success "Stale PX4 build directory removed"
  fi
  success "PX4-Autopilot checkout verified"

  # Python packages required by PX4's build tools. Ubuntu 24.04's PEP 668-managed
  # Python needs --break-system-packages; this only touches the user site dir.
  # PX4 is a system build, not a nix one: its own setup script installs what
  # it builds and runs against -- compilers and build tools, its Python
  # requirements, and Gazebo Harmonic from packages.osrfoundation.org, whose
  # gz libraries the launcher hands PX4 at run time. --no-nuttx: SITL only.
  info "Installing PX4's dependencies (Tools/setup/ubuntu.sh --no-nuttx: build tools, Python requirements, Gazebo Harmonic)..."
  # The script fetches the Gazebo repository key with wget, which a minimal
  # Ubuntu lacks, and installs gcc only: the build below is pinned to clang.
  sudo apt-get install -y --no-install-recommends wget clang
  ( cd "$PX4_DIR" && bash Tools/setup/ubuntu.sh --no-nuttx ) \
    || die "PX4's dependency script (Tools/setup/ubuntu.sh) failed; see its output above."
  success "PX4 dependencies installed"

  # Known build issue (clang >= 18): variable-length arrays in C++ are a hard
  # error (-Werror + -Wvla-cxx-extension). Patch once, exactly as the README
  # describes, when the flag isn't already present.
  if command -v clang++ &>/dev/null \
     && [[ "$(clang++ -dumpversion | cut -d. -f1)" -ge 18 ]] \
     && ! grep -q 'Wno-error=vla-cxx-extension' "$PX4_DIR/cmake/px4_add_common_flags.cmake"; then
    warn "clang >= 18 detected -- applying the known VLA build fix to PX4..."
    sed -i.bak 's/-Wno-c99-designator/-Wno-c99-designator\n                      -Wno-error=vla-cxx-extension/' \
      "$PX4_DIR/cmake/px4_add_common_flags.cmake"
    success "PX4 clang-18 VLA fix applied"
  fi

  if [[ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
    info "Building PX4 SITL gz_x500 (this may take 30-60 minutes; output in $PX4_DIR/build_px4_install.log)..."
    # 'make px4_sitl gz_x500' ends by launching an interactive pxh> shell
    # that would block forever, so run it in the background, wait for the
    # auto-launched px4 process, kill it, then verify the binary directly.
    #
    # CC/CXX pinned to clang: if the system's 'cc'/'c++' alternatives resolve
    # to different compiler families (e.g. cc -> gcc, c++ -> clang++), PX4's
    # CMake adds clang-only flags for C++ that GCC then rejects on .c files.
    ( cd "$PX4_DIR" && CC=clang CXX=clang++ make px4_sitl gz_x500 > build_px4_install.log 2>&1 ) &
    MAKE_PID=$!
    for _ in $(seq 1 480); do   # up to ~2 h
      if ! kill -0 "$MAKE_PID" 2>/dev/null; then break; fi
      if pgrep -f "$PX4_DIR/build/px4_sitl_default/bin/px4" >/dev/null 2>&1; then
        info "Build finished -- stopping the auto-launched pxh shell..."
        sleep 5
        pkill -f "$PX4_DIR/build/px4_sitl_default/bin/px4" 2>/dev/null || true
        sleep 2
        break
      fi
      sleep 15
    done
    kill "$MAKE_PID" 2>/dev/null || true
    wait "$MAKE_PID" 2>/dev/null || true

    # Best-effort cleanup of the few seconds of pxh session logs
    find "$PX4_DIR/build" -type d -name microsd -exec rm -rf "{}"/* \; 2>/dev/null || true

    if [[ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
      # Show WHY the build failed before dying (compiler error, OOM kill,
      # or the ~2 h watcher timeout on slow machines).
      warn "PX4 build did not produce the binary -- last 40 lines of $PX4_DIR/build_px4_install.log:"
      tail -n 40 "$PX4_DIR/build_px4_install.log" || true
      die "PX4 binary not found after build. Inspect $PX4_DIR/build_px4_install.log, fix, then re-run this script (it will resume)."
    fi
    success "PX4 SITL built: $PX4_DIR/build/px4_sitl_default/bin/px4"

    # The one-shot 'make ... gz_x500' run may leave a stray px4 process behind
    pkill -f 'build/px4_sitl_default/bin/px4' 2>/dev/null || true
  else
    info "PX4 SITL binary already built -- skipping (delete $PX4_DIR/build to force a rebuild)"
  fi

  # Persist PX4_AUTOPILOT_PATH for future shells. Separate idempotent block,
  # so adding PX4 later still works even though the main Draft block exists.
  PX4_MARKER="# >>> LOTUSim PX4 setup >>>"
  if ! grep -qF "$PX4_MARKER" ~/.bashrc; then
    cat >> ~/.bashrc <<PX4_BLOCK

# >>> LOTUSim PX4 setup >>>
export PX4_AUTOPILOT_PATH="\$HOME/PX4-Autopilot"
# <<< LOTUSim PX4 setup <<<
PX4_BLOCK
    success "~/.bashrc updated (PX4_AUTOPILOT_PATH)"
  else
    info "~/.bashrc already contains the LOTUSim PX4 block -- skipping"
  fi
  export PX4_AUTOPILOT_PATH="$PX4_DIR"
fi

# -----------------------------------------------------------------------------
# User mode only: build the deployment wheels, then copy the whole
# deployment/ folder from the generic scenario repo into ~/Documents/workspace/.
# Developer mode (answer 2 to question 1) skips this entirely.
# -----------------------------------------------------------------------------
if [[ "$USAGE_MODE" == "user" ]]; then
  DEPLOYMENT_SRC="$SCENARIO_WS/LOTUSim-generic-scenario/deployment"
  DEPLOYMENT_DEST_DIR="$HOME/Documents/workspace"

  if [[ -d "$DEPLOYMENT_SRC" ]]; then
    if [[ -f "$DEPLOYMENT_SRC/build_wheels.sh" ]]; then
      info "Building deployment wheels..."
      chmod +x "$DEPLOYMENT_SRC/build_wheels.sh"
      ( cd "$DEPLOYMENT_SRC" && ./build_wheels.sh )
      success "Deployment wheels built"
    else
      warn "$DEPLOYMENT_SRC/build_wheels.sh not found -- skipping wheel build"
    fi

    info "Copying deployment/ to ${DEPLOYMENT_DEST_DIR}..."
    mkdir -p "$DEPLOYMENT_DEST_DIR"
    cp -r "$DEPLOYMENT_SRC" "$DEPLOYMENT_DEST_DIR/"
    success "deployment/ copied to ${DEPLOYMENT_DEST_DIR}/deployment"
  else
    warn "$DEPLOYMENT_SRC not found -- skipping deployment copy step"
  fi
fi

# -----------------------------------------------------------------------------
# Done
# -----------------------------------------------------------------------------
echo ""
echo "=================================================================="
echo "  LOTUSim Setup Completed!"
echo ""
echo "  Ubuntu ${UBUNTU_VERSION}  |  ROS 2 ${ROS_DISTRO^}"
echo "  Usage mode: ${USAGE_MODE}  |  PX4 SITL: $([[ "$INSTALL_PX4_FLAG" == "1" ]] && echo yes || echo no)  |  Detection: $([[ "$INSTALL_GPU_FLAG" == "1" ]] && echo GPU || echo CPU)  |  Unity player: ${UNITY_PLAYER}"
echo "  Core branch: ${CORE_BRANCH}  |  Scenario branch: ${SCENARIO_BRANCH}"
echo ""
echo "  Core workspace:     ${LOTUSIM_WS}"
echo "  Scenario workspace: ${SCENARIO_WS}/LOTUSim-generic-scenario"
echo "  Core build:         lotusim_core build"
if [[ "$USAGE_MODE" == "user" ]]; then
echo "  Deployment folder:  ${HOME}/Documents/workspace/deployment"
fi
if [[ "$INSTALL_PX4_FLAG" == "1" ]]; then
echo "  PX4 SITL:           ${PX4_DIR} (\$PX4_AUTOPILOT_PATH)"
fi
echo ""
echo "  Next steps:"
echo "    1. Open a new terminal  (or: source ~/.bashrc)"
echo "    2. lotusim_shell     (the scenario devshell, on the core)"
echo "    3. Follow the scenario README to launch the simulation"
if grep -qi microsoft /proc/version 2>/dev/null; then
echo ""
echo "  WSL notes:"
echo "    - Gazebo runs headless by default; '--gui' needs WSLg (Win11, or"
echo "      an up-to-date WSL on Win10). If rendering glitches occur, try:"
echo "        export LIBGL_ALWAYS_SOFTWARE=1"
echo "    - PX4 agents ('px4': true) run fine headless in WSL; QGroundControl"
echo "      is easiest inside WSLg too (AppImage), connecting to localhost."
fi
echo "=================================================================="
