#!/usr/bin/env bash
# Lets Unity players on other machines reach a simulation that runs in WSL.
#
# By default WSL sits behind a NAT: the simulation listens on an address
# (172.x.x.x) that only exists inside this PC, so a player on another machine
# cannot reach the ROS-TCP bridge on port 10000. Mirrored networking gives WSL
# the PC's own addresses instead, and two firewall rules (Windows and the
# Hyper-V firewall in front of WSL) let port 10000 in.
#
#   wsl_network.sh status   # the addresses Unity players connect to, as
#                           # scenario_launch.sh prints them at every run
#   wsl_network.sh setup    # once per PC: mirrored networking + firewall rules
#
# See doc/WINDOWS_WSL.md.
set -euo pipefail

ROS_TCP_PORT=10000
RULE_NAME="LOTUSim-ROS-TCP-$ROS_TCP_PORT"
RULE_DISPLAY="LOTUSim ROS-TCP $ROS_TCP_PORT"
# WSL's VM creator id, which the Hyper-V firewall rules for WSL are keyed on.
WSL_VM_CREATOR_ID='{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
SELF="$(readlink -f "$0")"
# The script's path as printed in hints, with ~ for the home folder.
SETUP_HINT="${SELF#"$HOME"/}"
# shellcheck disable=SC2088
[[ "$SETUP_HINT" == /* ]] || SETUP_HINT="~/$SETUP_HINT"

YELLOW='\033[0;33m'; RED='\033[0;31m'; GREEN='\033[0;32m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO] $*${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $*${NC}"; }
die()  { echo -e "${RED}[ERROR] $*${NC}" >&2; exit 1; }

grep -qi microsoft /proc/version 2>/dev/null || die "Not running in WSL: nothing to set up."

# Runs the PowerShell script read on stdin. Passed base64-encoded, so that no
# quoting survives two shells.
powershell_run() {
  local encoded
  encoded="$(iconv -f utf-8 -t utf-16le | base64 -w0)"
  (cd / && powershell.exe -NoProfile -NonInteractive -EncodedCommand "$encoded") | tr -d '\r'
}

networking_mode() { wslinfo --networking-mode 2>/dev/null || echo unknown; }

# One line per fact: "ip <address>" for each LAN address of the PC, then
# "rule <0|1>" and "hvrule <0|1>" for the two firewall rules.
windows_network_facts() {
  powershell_run <<EOF
Get-NetIPAddress -AddressFamily IPv4 -AddressState Preferred |
  Where-Object { \$_.InterfaceAlias -notlike 'vEthernet*' -and \$_.InterfaceAlias -notlike 'Loopback*' -and \$_.IPAddress -notlike '169.254.*' } |
  ForEach-Object { "ip " + \$_.IPAddress }
"rule " + [int](\$null -ne (Get-NetFirewallRule -Name '$RULE_NAME' -ErrorAction SilentlyContinue))
"hvrule " + [int](\$null -ne (Get-NetFirewallHyperVRule -Name '$RULE_NAME' -ErrorAction SilentlyContinue))
EOF
}

status() {
  local mode facts ips rules_ok
  mode="$(networking_mode)"
  facts="$(windows_network_facts 2>/dev/null || true)"
  ips="$(awk '$1 == "ip" { printf "%s%s", sep, $2; sep = ", " }' <<< "$facts")"
  rules_ok=false
  grep -qx 'rule 1' <<< "$facts" && grep -qx 'hvrule 1' <<< "$facts" && rules_ok=true

  if [[ "$mode" == mirrored ]]; then
    info "Unity player on this PC: 127.0.0.1, port $ROS_TCP_PORT"
    if $rules_ok; then
      info "Unity players on other machines: ${ips:-the network address of this PC}, port $ROS_TCP_PORT"
    else
      warn "The firewall blocks port $ROS_TCP_PORT: Unity players on other machines cannot connect."
      warn "  Once per PC: $SETUP_HINT setup   (see doc/WINDOWS_WSL.md)"
    fi
  else
    info "Unity player on this PC: $(hostname -I | awk '{print $1}'), port $ROS_TCP_PORT"
    warn "WSL networking is '$mode', not 'mirrored': Unity players on other machines cannot connect."
    warn "  Once per PC: $SETUP_HINT setup, then 'wsl --shutdown' in Windows   (see doc/WINDOWS_WSL.md)"
  fi
}

# Sets key=value in [section] of an ini file, keeping everything else.
ini_set() {
  python3 - "$@" <<'PY'
import os, re, sys
path, section, key, value = sys.argv[1:]
lines = open(path, encoding="utf-8-sig").read().splitlines() if os.path.exists(path) else []
header = re.compile(r"\s*\[(.+?)\]\s*$")
start = next((i for i, l in enumerate(lines) if (m := header.match(l)) and m.group(1).strip().lower() == section.lower()), None)
if start is None:
    if lines and lines[-1].strip():
        lines.append("")
    lines += [f"[{section}]", f"{key}={value}"]
else:
    end = next((i for i in range(start + 1, len(lines)) if header.match(lines[i])), len(lines))
    hit = next((i for i in range(start + 1, end) if re.match(rf"\s*{re.escape(key)}\s*=", lines[i], re.I)), None)
    if hit is None:
        while end > start + 1 and not lines[end - 1].strip():
            end -= 1
        lines.insert(end, f"{key}={value}")
    else:
        lines[hit] = f"{key}={value}"
open(path, "w", encoding="utf-8", newline="\r\n").write("\n".join(lines) + "\n")
PY
}

setup() {
  local userprofile wslconfig facts
  userprofile="$(cd / && cmd.exe /c "echo %USERPROFILE%" 2>/dev/null | tr -d '\r')"
  [[ -n "$userprofile" && "$userprofile" != *%* ]] || die "Could not read %USERPROFILE% from Windows (cmd.exe)."
  wslconfig="$(wslpath -u "$userprofile")/.wslconfig"

  # hostAddressLoopback lets this PC reach the simulation through its own
  # network address too, so one address works from every machine.
  ini_set "$wslconfig" wsl2 networkingMode mirrored
  ini_set "$wslconfig" experimental hostAddressLoopback true
  info "Mirrored networking set in $userprofile\\.wslconfig"

  facts="$(windows_network_facts 2>/dev/null || true)"
  if grep -qx 'rule 1' <<< "$facts" && grep -qx 'hvrule 1' <<< "$facts"; then
    info "Firewall rules for port $ROS_TCP_PORT already present."
  else
    info "Adding the firewall rules for port $ROS_TCP_PORT: accept Windows' administrator prompt."
    local admin_script
    admin_script="$(iconv -f utf-8 -t utf-16le <<EOF | base64 -w0
Remove-NetFirewallRule -Name '$RULE_NAME' -ErrorAction SilentlyContinue
New-NetFirewallRule -Name '$RULE_NAME' -DisplayName '$RULE_DISPLAY' -Direction Inbound -Protocol TCP -LocalPort $ROS_TCP_PORT -Action Allow -Profile Any | Out-Null
Remove-NetFirewallHyperVRule -Name '$RULE_NAME' -ErrorAction SilentlyContinue
New-NetFirewallHyperVRule -Name '$RULE_NAME' -DisplayName '$RULE_DISPLAY' -Direction Inbound -VMCreatorId '$WSL_VM_CREATOR_ID' -Protocol TCP -LocalPorts $ROS_TCP_PORT -Action Allow | Out-Null
EOF
)"
    powershell_run <<EOF || true
Start-Process powershell.exe -Verb RunAs -Wait -WindowStyle Hidden -ArgumentList '-NoProfile','-EncodedCommand','$admin_script'
EOF
    facts="$(windows_network_facts 2>/dev/null || true)"
    if grep -qx 'rule 1' <<< "$facts" && grep -qx 'hvrule 1' <<< "$facts"; then
      info "Firewall rules '$RULE_DISPLAY' added (Windows and Hyper-V firewalls)."
    else
      warn "The firewall rules were not added (administrator prompt declined?). Run '$SETUP_HINT setup' again."
    fi
  fi

  if [[ "$(networking_mode)" == mirrored ]]; then
    info "WSL already runs with mirrored networking."
  else
    echo ""
    warn "Last step, once: restart WSL so that it takes the new networking."
    warn "  1. Close every WSL terminal (this stops any running simulation)."
    warn "  2. In a Windows terminal (PowerShell or cmd):  wsl --shutdown"
    warn "  3. Open a new WSL terminal."
  fi
}

case "${1:-}" in
  status) status ;;
  setup)  setup ;;
  *) die "usage: wsl_network.sh status|setup" ;;
esac
