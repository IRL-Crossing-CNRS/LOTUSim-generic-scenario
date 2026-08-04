"""
@file simulation_runner.py
@author Naval Group
@brief Manages the complete lifecycle of a Lotusim simulation.
@details
This module orchestrates the full simulation workflow for Lotusim environments, including:
- Building and launching the simulation world.
- Managing ROS 2 agent nodes and bridges.
- Handling cleanup, process termination, and state resets.

The script launches Lotusim in new GNOME terminal instances, initializes ROS 2 agents,
and ensures that all processes are safely terminated upon simulation completion or interruption.

@version 0.1
@date 2026-03-04

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

import logging
import os
import shutil
import signal
import subprocess
import time
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

import rclpy
from rclpy.executors import MultiThreadedExecutor

# Project modules
from simulation_run import ros_manager, utils

# -------------------------------------------------------------------------
# Global State
# -------------------------------------------------------------------------
process: Optional[subprocess.Popen] = None
executor: Optional[MultiThreadedExecutor] = None
cleanup_done: bool = False
lotusim_pids: List[int] = []


# -------------------------------------------------------------------------
# Simulation Launch Utilities
# -------------------------------------------------------------------------
def build_launch_command(
    world_file: str,
    aerial_domain: bool,
    debug: bool = False,
    gui: bool = False,
) -> List[str]:
    """
    Builds the shell commands required to launch the Lotusim simulation.

    Args:
        world_file: Path to the world file to launch.
        aerial_domain: Whether to also launch the aerialWorld physics world
            (needed for X500/aerial agents). The aerial world file is never
            configurable — always the hardcoded ``utils.AERIAL_WORLD_FILE``.
        debug: Enable debug mode (adds '--debug' flag to commands).
        gui: Enable Gazebo GUI (adds '--gui' flag to commands).

    Returns:
        A list of full command strings to execute.
    """
    base_command = "lotusim"
    debug_flag = "--debug" if debug else ""
    gui_flag = "--gui" if gui else ""
    commands: List[str] = []

    def _with_log(wf: str) -> str:
        """Append a tee into LOG_DIR/gz_<world>.log, matching the ros_tcp_endpoint
        / xdyn logging convention in scenario_launch.sh. Each gz sim world runs in
        its own gnome-terminal, outside that script's own stdout capture, so
        without this its -v4 output (world-load, plugin, and per-entity spawn
        messages — exactly what PX4/sensor issues need) never reaches
        scenario_logs and is lost once the terminal window closes. No-op if
        LOG_DIR isn't set (e.g. launched outside scenario_launch.sh).
        """
        cmd = f"{base_command} {debug_flag} {gui_flag} run {wf}".strip()
        log_dir = os.environ.get("LOG_DIR")
        if not log_dir:
            return cmd
        log_name = os.path.splitext(os.path.basename(wf))[0]
        return f'{cmd} 2>&1 | tee -a "{log_dir}/gz_{log_name}.log"'

    # The aerialWorld must come up first: the custom world's AerialEntityManager
    # waits on /aerialWorld/mas_cmd at load and logs "not available" if it's late.
    if aerial_domain:
        commands.append(_with_log(utils.AERIAL_WORLD_FILE))
    commands.append(_with_log(world_file))

    logger.debug("Launch commands: %s", commands)
    return commands


def start_simulation_process(commands: List[str]) -> List[subprocess.Popen]:
    """
    Starts Lotusim processes in separate GNOME terminals.

    Args:
        commands: List of Lotusim launch commands.

    Returns:
        List of subprocess.Popen objects for the started processes.

    Side effects:
        - Populates global `lotusim_pids` with tracked process IDs.
        - Launches GNOME terminal windows running the simulation.
    """

    global lotusim_pids
    processes: List[subprocess.Popen] = []

    for cmd in commands:
        full_cmd = f'gnome-terminal -- bash -c "{cmd}; exec bash"'
        process = subprocess.Popen(full_cmd, shell=True, preexec_fn=os.setsid)
        processes.append(process)
        time.sleep(1)  # Allow startup

        # Retrieve PIDs for tracking. cmd may now be "lotusim ... run world.world
        # 2>&1 | tee ...logfile" (see build_launch_command._with_log) — no single
        # process's cmdline contains a literal pipe, so pgrep -f on the full
        # string would never match anything. Match on the part before the pipe.
        pgrep_pattern = cmd.split(" 2>&1 |", 1)[0]
        pid_output = subprocess.run(["pgrep", "-f", pgrep_pattern], capture_output=True, text=True)
        pids = pid_output.stdout.strip().split("\n")

        for pid in pids:
            if pid.isdigit():
                lotusim_pids.append(int(pid))
                logger.info("Tracked Lotusim PID: %s", pid)

    return processes


# -------------------------------------------------------------------------
# Simulation Lifecycle Management
# -------------------------------------------------------------------------



def run_simulation(
    world_file: str,
    agents: Dict[str, Any],
    max_simulation_time: Optional[float] = None,
    aerial_domain: bool = False,
    debug_mode: bool = False,
    gui=False,
    config_dir: Optional[str] = None,
    record_csv: Any = False,
    scenario: Optional[Dict[str, Any]] = None,
) -> Any:
    """
    Orchestrates the full simulation lifecycle.

    Steps:
        1. Initializes ROS 2 and executor.
        2. Resets Gazebo state.
        3. Builds and starts Lotusim processes.
        4. Initializes ROS agents.
        5. Spins the executor until shutdown or timeout.
        6. Cleans up agents, processes, and ROS nodes on exit.

    Args:
        world_file: Simulation world file path.
        agents: Full agents dictionary as in JSON.
        max_simulation_time: Optional maximum simulation duration (seconds).
        aerial_domain: Whether to also launch the aerialWorld physics world.
        debug_mode: Enable verbose logging.
        gui: Enable Gazebo GUI.
        config_dir: Directory containing the scenario JSON (forwarded to
            ``ros_manager.initialize_ros_components``).
        record_csv: ``record_csv`` value from the scenario JSON (bool or dict);
            see ``simulation_run.csv_recorder``.
        scenario: The whole parsed scenario JSON dict (top-level ``"agents"``
            list included), forwarded to the CSV recorder so it can rebuild
            each agent's intended path and track mission/waypoint progress.
    """

    global process
    global executor

    if not rclpy.ok():
        rclpy.init()
        rclpy.signals.uninstall_signal_handlers()

    executor = MultiThreadedExecutor()

    # Enable verbose logs if debug mode is on
    if debug_mode:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.debug("Debug mode enabled — verbose logging active.")

    # Reset Gazebo and prepare launch command
    reset_gazebo_state()

    if ros_manager.shutdown_flag:
        stop_simulation(executor)
        return

    launch_commands = build_launch_command(world_file, aerial_domain, debug=debug_mode, gui=gui)

    # Start simulation process
    process = start_simulation_process(launch_commands)
    time.sleep(1)

    if ros_manager.shutdown_flag:
        stop_simulation(executor)
        return

    logger.info("Starting simulation...")

    # Initialize ROS agents and bridges using the full agents dictionary
    world_name = utils.get_world_name(world_file)

    # Optional CSV recording ("record_csv" in the scenario JSON): a pure
    # observer node — not an agent — recording every agent's pose + battery
    # to one CSV per agent.
    csv_recorder = None
    if record_csv:
        from simulation_run.csv_recorder import recorder_from_config

        try:
            csv_recorder = recorder_from_config(record_csv, world_name, scenario)
            if csv_recorder is not None:
                executor.add_node(csv_recorder)
        except Exception:
            logger.exception("Could not start the CSV recorder")

    # Optional fake ocean current ("ocean_current" in the scenario JSON): a
    # single latched publish consumed by the host's KinematicInterface, see
    # simulation_run.ocean_current for the full explanation.
    ocean_current_pub = None
    ocean_current_cfg = (scenario or {}).get("ocean_current")
    if ocean_current_cfg:
        from simulation_run.ocean_current import current_from_config

        try:
            ocean_current_pub = current_from_config(ocean_current_cfg, world_name)
            if ocean_current_pub is not None:
                executor.add_node(ocean_current_pub)
        except Exception:
            logger.exception("Could not start the ocean current publisher")

    agents_manager = ros_manager.initialize_ros_components(
        executor, agents, world_name, world_file, config_dir
    )

    if ros_manager.shutdown_flag:
        agents_manager.delete_agents()
        stop_simulation(executor)
        return

    # Scenario-wide "all missions complete" log: each agent only logs its own
    # mission completion (see lotusim_sdk.agents.agent.Agent._tick_missions),
    # this watches across all of them via the shared AgentsManager registry.
    mission_watcher = None
    try:
        from simulation_run.mission_watcher import MissionWatcher

        mission_watcher = MissionWatcher(agents_manager, world_name)
        executor.add_node(mission_watcher)
    except Exception:
        logger.exception("Could not start the mission watcher")

    try:
        # Run main execution loop
        return ros_manager.run_executor(executor, max_simulation_time)
    finally:
        agents_manager.delete_agents()
        if csv_recorder is not None:
            try:
                csv_recorder.destroy_node()  # closes the CSV files
            except Exception:
                pass
        if ocean_current_pub is not None:
            try:
                ocean_current_pub.destroy_node()
            except Exception:
                pass
        if mission_watcher is not None:
            try:
                mission_watcher.destroy_node()
            except Exception:
                pass
        stop_simulation(executor)


def stop_simulation(executor: Optional[MultiThreadedExecutor]) -> None:
    """
    Stops the simulation process, kills background tasks, cleans up nodes,
    and safely shuts down ROS 2 and Gazebo components.

    Args:
        executor: ROS 2 executor managing running nodes.
    """

    global process, cleanup_done

    if cleanup_done:
        logger.info("Cleanup already performed. Skipping.")
        return
    cleanup_done = True

    logger.info("Stopping simulation and performing cleanup...")

    # Kill tracked Lotusim processes
    for pid in lotusim_pids:
        try:
            os.kill(pid, signal.SIGTERM)
            logger.info("Sent SIGTERM to Lotusim PID %s", pid)
        except ProcessLookupError:
            logger.warning("Lotusim PID %s already terminated.", pid)
        except Exception:
            logger.warning("Error killing Lotusim PID %s", pid, exc_info=True)
    lotusim_pids.clear()

    # Kill any gnome-terminal windows (fallback cleanup)
    try:
        subprocess.run(["pkill", "-f", "gnome-terminal"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        logger.info("Terminated GNOME terminal processes.")
    except Exception:
        logger.warning("Error killing GNOME terminals", exc_info=True)

    # Gracefully stop tracked simulation processes
    for proc in (process or []):
        try:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=3)
                logger.info("Simulation process %s terminated.", proc.pid)
        except ProcessLookupError:
            pass
        except Exception:
            logger.warning("Error terminating process %s", proc.pid, exc_info=True)

    process = None
    time.sleep(1)

    # Kill bridge and Gazebo simulation processes
    for target in ["ros_gz_bridge/parameter_bridge", "gz sim"]:
        try:
            subprocess.run(["pkill", "-f", target], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            logger.info("Terminated '%s' processes.", target)
        except Exception:
            logger.warning("Error terminating '%s'", target, exc_info=True)

    # ROS 2 node cleanup
    if executor is not None:
        try:
            for node in list(executor.get_nodes()):
                try:
                    executor.remove_node(node)
                    logger.info("Removed ROS node: %s", node)
                except Exception:
                    logger.warning("Error removing node %s", node, exc_info=True)
            executor.shutdown()
            logger.info("ROS 2 executor shutdown complete.")
        except Exception:
            logger.warning("Error during ROS node cleanup", exc_info=True)

    # Shutdown rclpy
    try:
        if rclpy.ok():
            rclpy.shutdown()
            logger.info("ROS 2 client library shutdown successful.")
    except Exception:
        logger.warning("Error shutting down rclpy", exc_info=True)

    logger.info("Simulation cleanup completed.")


def reset_gazebo_state() -> None:
    """
    Resets Gazebo and related processes, ensuring a clean environment
    before starting a new simulation.

    Steps:
        - Terminates 'gz sim' and 'ros_gz_bridge' processes.
        - Deletes temporary Gazebo state directory (/tmp/gz/sim).
        - Ensures environment is ready for a fresh simulation run.
    """
    logger.info("Resetting Gazebo simulation state...")

    # Terminate simulation and bridge processes
    for process_name in ["gz sim", "ros_gz_bridge"]:
        try:
            subprocess.run(["pkill", "-f", process_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            logger.info("Terminated '%s' processes.", process_name)
        except Exception:
            logger.warning("Failed to terminate '%s'", process_name, exc_info=True)

    # Delete temporary Gazebo state directory
    gazebo_state_path = "/tmp/gz/sim"
    if os.path.exists(gazebo_state_path):
        try:
            shutil.rmtree(gazebo_state_path)
            logger.info("Deleted Gazebo state directory: %s", gazebo_state_path)
        except Exception:
            logger.warning("Failed to delete Gazebo state directory: %s", gazebo_state_path, exc_info=True)
    else:
        logger.info("No Gazebo state directory found at: %s", gazebo_state_path)

    time.sleep(2)