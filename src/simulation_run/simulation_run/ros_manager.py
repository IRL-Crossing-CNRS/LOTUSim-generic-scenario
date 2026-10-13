"""
@file ros_manager.py
@author Naval Group
@brief Core runtime orchestration for generic scenario simulations.

@details
This module manages the execution flow of simulations.

It provides:
- ROS 2 executor management with graceful shutdown
- Agent initialization via `AgentsManager`
- Pose monitoring for agent liveness tracking

@note
Designed to serve as the central runtime layer of the simulation system,
coordinating agent management, monitoring, and performance evaluation.

@version 0.1
@date 2026-03-03

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

import logging
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle
from simulation_run import agents_manager
from simulation_run.dynamic_spawn import DynamicSpawnService

logger = logging.getLogger(__name__)

# -------------------------------------------------------------------------
# Global Shutdown State
# -------------------------------------------------------------------------
shutdown_flag = False
shutdown_flag_lock = threading.Lock()


# -------------------------------------------------------------------------
# Initialization
# -------------------------------------------------------------------------
def initialize_ros_components(
    executor: Any,
    agents: Dict[str, Any],
    world_name: str,
    world_file: str = "",
    config_dir: Optional[str] = None,
    shard: int = 0,
    shards: int = 1,
) -> agents_manager.AgentsManager:
    """
    Initialize simulation components using a full agents dictionary.

    Args:
        executor: ROS 2 executor to register agent nodes.
        agents: Dictionary containing agent types, their number, poses, model paths, and xdyn flag.
        world_name: Name of the simulation world.
        config_dir: Directory containing the scenario JSON (forwarded to
            ``AgentsManager.add_agents`` — see its docstring).
        shard, shards: this process's index among the agent-hosting processes
            and how many there are, forwarded to ``DynamicSpawnService`` so a
            runtime spawn lands on the least loaded of them.

    Returns:
        AgentsManager instance with all agents initialized and registered.
    """
    time.sleep(1.0)

    # Initialize and spawn all agents
    manager = agents_manager.AgentsManager()
    manager.add_agents(agents, world_name, executor, world_file, config_dir)

    # Register dynamic spawn/despawn service
    spawn_service = DynamicSpawnService(manager, executor, world_name, world_file, shard=shard, shards=shards)
    executor.add_node(spawn_service)

    return manager


# -------------------------------------------------------------------------
# Executor Management
# -------------------------------------------------------------------------


def _spin_forever(executor: Any) -> None:
    """Drive the executor on its own thread so its worker pool is used.

    Spinning is resumed after a transient failure: rclpy raises
    InvalidHandle when an entity is destroyed while the wait set is being
    built, which happens whenever an agent is added or removed while the
    scenario runs. Letting that escape would stop every callback for good.
    """
    while True:
        with shutdown_flag_lock:
            if shutdown_flag:
                return
        if not rclpy.ok():
            return
        try:
            executor.spin()
            return
        except InvalidHandle:
            logger.debug("executor spin interrupted by an entity destruction, resuming")
        except Exception:
            logger.exception("executor spin stopped")
            return


def run_executor(executor: Any, max_simulation_time: Optional[float] = None) -> None:
    """
    Run a ROS 2 executor loop until a shutdown condition is met.

    The loop exits when one of the following occurs:
        - `shutdown_flag` becomes True
        - `rclpy.ok()` returns False
        - `max_simulation_time` (if provided) is exceeded
        - An exception is raised during executor spinning

    Args:
        executor: ROS 2 executor instance to spin.
        max_simulation_time: Optional maximum runtime in seconds.

    Notes:
        The executor spins on a dedicated thread so its worker pool is
        used; this loop only watches the shutdown conditions.
    """
    start_time = time.time()
    spin_thread = None
    try:
        while rclpy.ok():
            elapsed = time.time() - start_time

            # Check external shutdown flag
            with shutdown_flag_lock:
                if shutdown_flag:
                    logger.info("run_executor EXIT: shutdown_flag=True at t=%.1fs", elapsed)
                    break

            # Check maximum simulation time
            if max_simulation_time is not None and elapsed > max_simulation_time:
                logger.info("run_executor EXIT: max_simulation_time=%ss reached", max_simulation_time)
                break

            # Let the executor's own threads drain the callbacks. spin_once()
            # runs ONE callback per call on the calling thread and never
            # touches the MultiThreadedExecutor's pool, so a scenario with
            # dozens of agents serialised every timer and subscription onto
            # this loop: a 20 Hz control task measured 0.29 Hz.
            if spin_thread is None or not spin_thread.is_alive():
                spin_thread = threading.Thread(
                    target=_spin_forever, args=(executor,), name="executor-spin", daemon=True
                )
                spin_thread.start()
            time.sleep(0.05)

        if not rclpy.ok():
            logger.info("run_executor EXIT: rclpy.ok()=False at t=%.1fs", time.time() - start_time)

    except SystemExit:
        pass
    except BaseException as e:
        logger.exception("run_executor BASE EXCEPTION: %s", type(e).__name__)
        raise
