#!/usr/bin/env bash
# Download the Unity player <name> (e.g. lotusimenergy_linux) pinned in
# lotusim_unity_executables/players.json, unless it is already there.
#
# The players are release assets of this repository, not git files: a player
# is about 1 GB, and every version pushed through Git LFS stays in the
# organisation's LFS storage for good. The repository is private, so the
# download goes through the GitHub CLI, logged in once with `gh auth login`.
#
#   fetch_unity_player.sh lotusimenergy_linux     # download if missing or outdated
#   fetch_unity_player.sh --check lotusimenergy_linux   # exit 1 if a download is needed
#   fetch_unity_player.sh --path lotusimenergy_windows  # print the player folder
#
# The archive's sha256 is written to <player>/.player_sha256 once extracted, so
# a player already matching players.json is never downloaded again.
#
# A player goes in lotusim_unity_executables/<player>/, except a Windows player
# under WSL: Windows cannot start a Unity 6.6 player from the WSL filesystem
# (\\wsl.localhost\...), it fails to load DirectStorage (dstorage.dll), so that
# one goes on the Windows drive, in %LOCALAPPDATA%\LOTUSim\players\<player>\.
set -euo pipefail

MODE=fetch
case "${1:-}" in
  --check) MODE=check; shift ;;
  --path)  MODE=path; shift ;;
esac
NAME="${1:?usage: fetch_unity_player.sh [--check|--path] <player name, e.g. lotusimenergy_linux>}"

WS="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
EXE_DIR="$WS/lotusim_unity_executables"
MANIFEST="$EXE_DIR/players.json"
[[ -f "$MANIFEST" ]] || { echo "[ERROR] $MANIFEST not found." >&2; exit 2; }

read_manifest() { python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); p=m["players"].get(sys.argv[2]); print(m["repository"], p["release"], p["asset"], p["sha256"]) if p else sys.exit(3)' "$MANIFEST" "$NAME"; }
if ! read -r REPO RELEASE ASSET SHA256 < <(read_manifest); then
  echo "[ERROR] No player '$NAME' in $MANIFEST." >&2
  exit 3
fi

PLAYERS_DIR="$EXE_DIR"
if [[ "$NAME" == *_windows ]] && grep -qi microsoft /proc/version 2>/dev/null; then
  LOCALAPPDATA_WIN="$(cd / && cmd.exe /c "echo %LOCALAPPDATA%" 2>/dev/null | tr -d '\r')"
  [[ -n "$LOCALAPPDATA_WIN" && "$LOCALAPPDATA_WIN" != *%* ]] \
    || { echo "[ERROR] Could not read %LOCALAPPDATA% from Windows (cmd.exe)." >&2; exit 2; }
  PLAYERS_DIR="$(wslpath -u "$LOCALAPPDATA_WIN")/LOTUSim/players"
fi
TARGET="$PLAYERS_DIR/$NAME"
if [[ "$MODE" == path ]]; then echo "$TARGET"; exit 0; fi

if [[ -f "$TARGET/.player_sha256" && "$(cat "$TARGET/.player_sha256")" == "$SHA256" ]]; then
  [[ "$MODE" == check ]] || echo "[INFO] Unity player $NAME is up to date ($RELEASE)."
  exit 0
fi
if [[ "$MODE" == check ]]; then
  echo "[INFO] Unity player $NAME must be downloaded ($RELEASE)."
  exit 1
fi

# The GitHub CLI: installed, or else from nixpkgs.
if command -v gh >/dev/null 2>&1; then
  GH=(gh)
elif command -v nix >/dev/null 2>&1; then
  GH=(nix --extra-experimental-features "nix-command flakes" run nixpkgs#gh --)
else
  echo "[ERROR] The GitHub CLI (gh) is needed to download the Unity player: install it, then run 'gh auth login'." >&2
  exit 4
fi
if ! "${GH[@]}" auth status >/dev/null 2>&1; then
  echo "[ERROR] Log in to GitHub once to download the Unity player: ${GH[*]} auth login" >&2
  exit 4
fi

# Downloaded next to the target, so that swapping it in is a rename.
mkdir -p "$PLAYERS_DIR"
TMP="$(mktemp -d "$PLAYERS_DIR/.download.XXXXXX")"
trap 'rm -rf "$TMP"' EXIT
echo "[INFO] Downloading Unity player $NAME ($RELEASE/$ASSET)..."
"${GH[@]}" release download "$RELEASE" --repo "$REPO" --pattern "$ASSET" --dir "$TMP"
GOT="$(sha256sum "$TMP/$ASSET" | cut -d' ' -f1)"
if [[ "$GOT" != "$SHA256" ]]; then
  echo "[ERROR] $ASSET: sha256 $GOT, players.json expects $SHA256." >&2
  exit 5
fi
mkdir -p "$TMP/x"
tar -xzf "$TMP/$ASSET" -C "$TMP/x"
[[ -d "$TMP/x/$NAME" ]] || { echo "[ERROR] $ASSET has no top-level folder $NAME/." >&2; exit 5; }
echo "$SHA256" > "$TMP/x/$NAME/.player_sha256"
# Swap in the new player only once it is complete.
if [[ -e "$TARGET" ]]; then mv "$TARGET" "$TMP/old"; fi
mv "$TMP/x/$NAME" "$TARGET"
echo "[INFO] Unity player $NAME installed in $TARGET."
