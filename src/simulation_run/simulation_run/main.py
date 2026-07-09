#!/usr/bin/env python3
"""
@file main.py
@author Naval Group
@brief Primary launcher for generic LOTUSim simulation scenarios.

@details
This script serves as the main entry point for launching LOTUSim scenario simulations.
It provides the following key functionalities:

- Loads user-defined JSON configuration files specifying world, agents, and simulation parameters.
- Resolves configuration paths using ROS 2 package shares or fallback to local directories.
- Sets up signal handling (graceful shutdown on Ctrl+C / SIGINT).
- Parses agent definitions, SDFs, and simulation parameters.
- Delegates simulation execution to `simulation_run.simulation_runner`.
- Integrates configuration and SDF utilities via `simulation_run.utils`.

@note
Designed to be executed as the initial step of the simulation workflow.
It initializes the simulation environment and hands off runtime management
to the `simulation_run` subsystem.

@version 0.1
@date 2026-03-04

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""


import logging
import os
import signal

from ament_index_python.packages import get_package_share_directory

from simulation_run import ros_manager, simulation_runner, utils

logger = logging.getLogger(__name__)


# ----------------------------------------------------------------------
# Signal Handling
# ----------------------------------------------------------------------
def signal_handler(signum, frame):
    """
    Handle Ctrl+C (SIGINT) by signalling the executor loop to exit cleanly.
    Cleanup is handled by run_simulation's finally block — no exit() call needed.
    """
    logger.info("Ctrl+C received — shutting down simulation...")
    with ros_manager.shutdown_flag_lock:
        ros_manager.shutdown_flag = True


# ----------------------------------------------------------------------
# Main Entry Point
# ----------------------------------------------------------------------
def main():
    """
    Main entry point — loads config, sets up signal handling, and starts the simulation.
    """
    args = utils.get_cli_args()

    logger.debug("Debug mode is %s", "ENABLED" if args.debug else "DISABLED")
    logger.debug("GUI mode is %s", "ENABLED" if args.gui else "DISABLED")

    # Determine config path
    try:
        package_share_directory = get_package_share_directory("simulation_run")
        config_path = os.path.join(package_share_directory, "config", args.config)
    except Exception:
        package_share_directory = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
        config_path = os.path.join(package_share_directory, "config", args.config)

    config_path = os.path.normpath(config_path)
    if not os.path.exists(config_path):
        raise FileNotFoundError(
            f"Configuration file not found: {config_path}\nExpected in {os.path.dirname(config_path)}"
        )

    logger.info("Using configuration file: %s", config_path)

    # Setup signal handling
    signal.signal(signal.SIGINT, signal_handler)

    # Load config
    config = utils.load_config_from_json(config_path)
    logger.info("Configuration loaded.")

    # Inject AIS trajectory into agents
    agents = config.get("agents", {})

    # Inject AIS poses and trajectories
    utils.inject_first_ais_pose(agents, base_dir=os.path.dirname(config_path))

    # Parse config
    world_file, agents, aerial_enabled = utils.parse_simulation_config(config)

    # Run simulation
    simulation_runner.run_simulation(
        world_file, agents, aerial_domain=aerial_enabled,
        debug_mode=args.debug, gui=args.gui,
        config_dir=os.path.dirname(config_path),
    )


if __name__ == "__main__":
    main()