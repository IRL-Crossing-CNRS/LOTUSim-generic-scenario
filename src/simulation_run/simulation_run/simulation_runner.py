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
def build_launch_command(world_file: str, aerial_domain: bool, debug: bool = False, gui: bool = False) -> List[str]:
    """
    Builds the shell commands required to launch the Lotusim simulation.

    Args:
        world_file: Path to the world file to launch.
        aerial_domain: Whether to launch in aerial mode.
        debug: Enable debug mode (adds '--debug' flag to commands).
        gui: Enable Gazebo GUI (adds '--gui' flag to commands).

    Returns:
        A list of full command strings to execute.
    """
    base_command = "lotusim"
    debug_flag = "--debug" if debug else ""
    gui_flag = "--gui" if gui else ""
    commands: List[str] = []

    if aerial_domain:
        commands.append(f"{base_command} {debug_flag} {gui_flag} run aerialWorld.world".strip())
        commands.append(f"{base_command} {debug_flag} {gui_flag} run {world_file}".strip())
    else:
        commands.append(f"{base_command} {debug_flag} {gui_flag} run {world_file}".strip())

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

        # Retrieve PIDs for tracking
        pid_output = subprocess.run(["pgrep", "-f", cmd], capture_output=True, text=True)
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
        aerial_domain: Whether to launch an aerial domain.
        debug_mode: Enable verbose logging.
        gui: Enable Gazebo GUI.
        config_dir: Directory containing the scenario JSON (forwarded to
            ``ros_manager.initialize_ros_components``).
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

    agents_manager = ros_manager.initialize_ros_components(
        executor, agents, world_name, world_file, aerial_domain, config_dir
    )

    if ros_manager.shutdown_flag:
        agents_manager.delete_agents()
        stop_simulation(executor)
        return

    try:
        # Run main execution loop
        return ros_manager.run_executor(executor, max_simulation_time)
    finally:
        agents_manager.delete_agents()
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