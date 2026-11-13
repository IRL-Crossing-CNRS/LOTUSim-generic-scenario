# Running LOTUSim from Windows (WSL)

How the simulation runs on a Windows PC, where the Unity player is kept, and
how Unity players on other machines connect to it.

On Windows, the simulation (ROS 2, Gazebo, xdyn) always runs in WSL with
Ubuntu 24.04. The Unity player is the Windows build, started from WSL by
`scenario_launch.sh`. The same scripts run on Linux and in WSL. They detect
WSL by themselves.

| Simulation | Unity player | What to do |
|---|---|---|
| Linux | Linux, same machine | Nothing: the launcher starts the player |
| Linux | Windows or Linux, other machine | Enter the address the launcher prints, port `10000` |
| Windows (WSL) | Windows, same PC | Nothing: the launcher starts the player |
| Windows (WSL) | Windows or Linux, other machine | [Once per PC: open WSL to the network](#once-per-pc-open-wsl-to-the-network), then enter the address the launcher prints |

A player on another machine is started by hand on that machine. To skip the
player on the simulation's own machine, launch with `UNITY_MODE=editor`.

## The Windows Unity player

Under WSL, `fetch_unity_player.sh` installs the Windows player in
`%LOCALAPPDATA%\LOTUSim\players\lotusimenergy_windows\`, not in
`lotusim_unity_executables/`. Windows cannot start a Unity 6.6 player from
the WSL filesystem (`\\wsl.localhost\...`). It stops on
`dstorage.dll` ("the file is empty"), although the same files start from
`C:`.

The launcher downloads the player there the first time, and again whenever
`lotusim_unity_executables/players.json` pins another version. To print the
folder:

```bash
./src/simulation_run/executable/fetch_unity_player.sh --path lotusimenergy_windows
```

To test another Windows build, copy it to a Windows drive and point
`UNITY_EXE_DIR` at it, for example
`UNITY_EXE_DIR=/mnt/c/builds/lotusimenergy_windows`. The launcher warns
when `UNITY_EXE_DIR` points a Windows player at the WSL filesystem.

## The address to enter in the player

At every run with Unity, the launcher prints the address to enter in the
player's lobby, after the ROS-TCP bridge has started. The port is always
`10000`.

On Linux:

```text
[INFO] Unity players connect to: 192.168.50.34, port 10000
```

Under WSL, once open to the network:

```text
[INFO] Unity player on this PC: 127.0.0.1, port 10000
[INFO] Unity players on other machines: 192.168.50.76, port 10000
```

Under WSL, before that:

```text
[INFO] Unity player on this PC: 172.19.233.190, port 10000
[WARN] WSL networking is 'nat', not 'mirrored': Unity players on other machines cannot connect.
[WARN]   Once per PC: ~/Documents/workspace/lotusim/LOTUSim-generic-scenario/src/simulation_run/executable/wsl_network.sh setup, then 'wsl --shutdown' in Windows   (see doc/WINDOWS_WSL.md)
```

To print the same lines without running a scenario:

```bash
./src/simulation_run/executable/wsl_network.sh status
```

## Once per PC: open WSL to the network

By default WSL sits behind a NAT. The simulation listens on an address
(`172.x.x.x`) that only exists inside the PC and changes at every reboot.
A player on another machine has nothing to connect to. Two changes fix it:

- **Mirrored networking:** WSL shares the PC's own network addresses
  (`192.168.x.x`). On the PC itself, `127.0.0.1` reaches the simulation too.
- **Port 10000 open:** an inbound rule in the Windows firewall, and one in the
  Hyper-V firewall that sits in front of WSL and blocks inbound connections by
  default.

The installer does both when you answer yes to its question 6 (the default,
or `INSTALL_WSL_NETWORK=1`). On a PC installed without it, run once, from WSL:

```bash
./src/simulation_run/executable/wsl_network.sh setup
```

It:

1. Sets `networkingMode=mirrored` in `[wsl2]` and `hostAddressLoopback=true`
   in `[experimental]` in `%USERPROFILE%\.wslconfig`, keeping the rest of the
   file. `hostAddressLoopback` lets the PC reach the simulation through its
   own `192.168.x.x` address too.
2. Adds the two firewall rules, both named `LOTUSim-ROS-TCP-10000`. Windows
   shows one administrator prompt: accept it. Running the script again is
   harmless. It skips the rules when they are there.
3. Prints the last step, which it cannot do itself because it runs inside WSL:
   close every WSL terminal (this stops any running simulation), run
   `wsl --shutdown` in a Windows terminal (PowerShell or cmd), then open a new
   WSL terminal.

To check, run `wsl_network.sh status` in the new terminal, then start a
scenario and, from the other machine:

```bash
nc -zv 192.168.50.76 10000     # Linux: "succeeded" when the port is open
```

```powershell
Test-NetConnection 192.168.50.76 -Port 10000   # Windows: TcpTestSucceeded : True
```

### The same steps by hand

`%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored

[experimental]
hostAddressLoopback=true
```

In a PowerShell run as administrator:

```powershell
New-NetFirewallRule -Name LOTUSim-ROS-TCP-10000 -DisplayName "LOTUSim ROS-TCP 10000" -Direction Inbound -Protocol TCP -LocalPort 10000 -Action Allow -Profile Any
New-NetFirewallHyperVRule -Name LOTUSim-ROS-TCP-10000 -DisplayName "LOTUSim ROS-TCP 10000" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol TCP -LocalPorts 10000 -Action Allow
```

Then `wsl --shutdown`.

### Undo

Remove the two lines from `%USERPROFILE%\.wslconfig`, then, in a PowerShell
run as administrator:

```powershell
Remove-NetFirewallRule -Name LOTUSim-ROS-TCP-10000
Remove-NetFirewallHyperVRule -Name LOTUSim-ROS-TCP-10000
```

Then `wsl --shutdown`.

### Limits

- Mirrored networking needs Windows 11 22H2 or later.
- With some VPN clients or Docker Desktop, mirrored networking misbehaves. The
  WSL documentation lists the known cases.
- The address is the PC's own, given by the router: it can change. The
  launcher always prints the current one.
- A company network policy can override local firewall rules. If
  `Test-NetConnection` fails with both rules in place, ask the network
  administrator to allow TCP 10000 to the PC.

## Simulation on Linux, player on Windows

Nothing to set up on the Windows side. Enter the address the Linux launcher
prints. If the Linux machine runs a firewall (`ufw status`), open the port:
`sudo ufw allow 10000/tcp`.
