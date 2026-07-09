"""
@file agents_manager.py
@author Naval Group
@brief Manages the lifecycle of simulation agents within a ROS 2 environment.

@details
The AgentsManager class is responsible for dynamically loading, creating, spawning,
and deleting agent instances.

Key functionalities include:

- Supports multiple agent types such as BlueROV2, Lrauv, X500, Dtmb etc.
- Handles error logging and resource cleanup.
- Flexible configuration through mapping tables and JSON-to-class conversion.
- Registers agent nodes with a ROS2 executor.
- Queues and executes agent spawn commands with optional pose specification.

@note
Designed to serve as a centralized controller at the beginning of the simulation
to launch all agents and manage their lifecycles, enabling coordinated and
heterogeneous multi-agent simulation management.

@version 0.1
@date 2026-03-04

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

import logging
import traceback
from typing import Any, Dict, List, Optional, Tuple

import rclpy

from simulation_run import utils
from lotusim_sdk import Environment




class AgentsManager:
    """
    Manages the lifecycle of simulation agents within a ROS 2 environment.

    Handles agent creation, spawning, and deletion in coordination
    with a provided ROS2 executor.
    """

    def __init__(self) -> None:
        """Initialize the AgentsManager with an empty agent registry."""
        self.agents: Dict[str, Any] = {}

    # -------------------------------------------------------------------------
    # Public Methods
    # -------------------------------------------------------------------------
    def add_agents(
        self,
        agents: Any,
        world_name: str,
        executor: Any,
        aerial_domain: bool = False,
        world_file: str = "",
        config_dir: Optional[str] = None,
    ) -> None:
        """
        Adds and spawns multiple agents in the simulation using the 'agents' config.

        Two config shapes are accepted (backward compatible):

        - **dict** (legacy): ``{"AgentClass": {nb_agents, poses, ais, ...}}``.
        - **list** (mission system): ``[{"id", "class", "missions", "spawn", ...}]``
          where each entry carries its own ``id``/``class`` and optional BT
          ``missions`` (see doc/MISSIONS.md §7).

        Args:
            agents: Agent config as a dict (legacy) or a list (mission system).
            world_name: Name of the simulation world.
            executor: ROS2 executor where nodes will be added.
            aerial_domain: Whether aerial domain is enabled.
            config_dir: Directory containing the scenario JSON, exposed to each
                agent as ``_config_dir`` so a ``waypoint_follower`` task can
                resolve a relative ``waypoints_file`` — the host-side
                counterpart of run_agent's ``_config_dir``.
        """
        if executor is None or not rclpy.ok():
            raise RuntimeError("Executor is not initialized or ROS2 is shut down!")

        spawn_queue: List[Tuple[Any, Any]] = []

        # World geographic origin (lat0, lon0) from the world's
        # <spherical_coordinates>, exposed to each agent as ``_world_origin`` so a
        # WaypointFollowerTask can project lat/lon waypoints into the world ENU
        # frame — the host-side counterpart of run_agent's --origin.
        world_origin = self._world_origin_from_file(world_file)

        for agent_type, agent_info in self._iter_agents(agents):
            # Process each agent type individually
            self._process_single_agent_type(
                agent_type, agent_info, world_name, executor, spawn_queue,
                aerial_domain, world_origin, config_dir,
            )

        # Spawn all agents after registration
        self._spawn_all_agents(spawn_queue)

    @staticmethod
    def _world_origin_from_file(world_file: str) -> Optional[Tuple[float, float]]:
        """Read (lat0, lon0) from the world's <spherical_coordinates>, or None."""
        if not world_file:
            return None
        try:
            return utils._extract_world_spherical_coords(world_file)
        except Exception as e:
            logging.warning("Could not read world geo origin from '%s': %s", world_file, e)
            return None

    @staticmethod
    def _iter_agents(agents: Any) -> List[Tuple[str, Dict[str, Any]]]:
        """Normalize the dict (legacy) or list (mission) form into (type, info) pairs."""
        if isinstance(agents, list):
            pairs = []
            for agent_info in agents:
                agent_type = agent_info.get("class") or agent_info.get("type") or agent_info.get("id")
                pairs.append((agent_type, agent_info))
            return pairs
        return list(agents.items())

    def delete_agents(self) -> None:
        """Sends delete commands to all managed agents and destroys their nodes."""
        if not rclpy.ok():
            return
        for agent in self.agents.values():
            if isinstance(agent, Environment):
                continue
            try:
                agent.send_single_delete_cmd()
            except Exception as e:
                logging.warning(f"Could not delete {agent.agent_name}: {e}")
            try:
                agent.destroy_node()
            except Exception as e:
                logging.warning(f"Could not destroy node {agent.agent_name}: {e}")

    def get_agent(self, name: str) -> Optional[Any]:
        """
        Retrieve an agent instance by name.

        Args:
            name: The unique name of the agent.

        Returns:
            The agent instance if found, otherwise None.
        """
        return self.agents.get(name)

    # -------------------------------------------------------------------------
    # Internal Helper Methods
    # -------------------------------------------------------------------------
    def _process_single_agent_type(
        self,
        agent_type: str,
        agent_info: Dict[str, Any],
        world_name: str,
        executor: Any,
        spawn_queue: List[Tuple[Any, Any]],
        aerial_domain: bool = False,
        world_origin: Optional[Tuple[float, float]] = None,
        config_dir: Optional[str] = None,
    ) -> None:
        """
        Registers and queues all agents of a given type from the agent_info dictionary.

        Args:
            agent_type: Type of the agent.
            agent_info: Dictionary containing nb_agents, poses, model, xdyn, etc.
            world_name: Simulation world name.
            executor: ROS2 executor managing active nodes.
            spawn_queue: List that accumulates agents and their target poses.
            aerial_domain: Whether aerial domain is enabled.
            config_dir: Directory containing the scenario JSON (see ``add_agents``).
        """
        try:
            # Convert JSON/mission name -> Python class
            class_name = utils.json_name_to_class_name(agent_type)
            agent_class = utils.find_agent_class_globally(class_name)
            if agent_class is None:
                logging.error(f"Agent class '{class_name}' not found")
                return

            # Environment agents (Wind, etc.) have no SDF model and no spawn queue
            if issubclass(agent_class, Environment):
                env_node = agent_class(world_name, **agent_info)
                executor.add_node(env_node)
                self.agents[agent_type.lower()] = env_node
                logging.info(f"Environment agent '{agent_type}' added to executor.")
                return

            nb_agents = agent_info.get("nb_agents", 1)
            poses = agent_info.get("poses", [])
            sdf_file = agent_info.get("sdf_file", "")
            xdyn_enabled = bool(agent_info.get("xdyn", False))
            # In the mission (list) form an explicit "id" defines the instance name.
            id_base = agent_info.get("id") or agent_class.__name__.lower()

            for i in range(nb_agents):
                instance_name = f"{id_base}{i}"

                unique_sdf = sdf_file.replace("{model_name}", instance_name) if sdf_file else ""

                logging.info(f"Creating agent '{instance_name}' of type '{agent_type}' with SDF from '{sdf_file}'")

                agent_node = self._create_agent_instance(
                    agent_class, unique_sdf, world_name, xdyn_enabled, agent_info, i
                )
                agent_node.agent_name = instance_name
                agent_node.sdf_file = sdf_file

                # Expose the world origin so a WaypointFollowerTask can project
                # its lat/lon waypoints into the world ENU frame. Set before
                # missions are installed so it is in place when the task reads it.
                if world_origin is not None:
                    agent_node._world_origin = world_origin

                # Expose the scenario JSON's directory so a WaypointFollowerTask
                # can resolve a relative "waypoints_file" param, same as
                # run_agent's "_config_dir" on the remote path.
                if config_dir is not None:
                    agent_node._config_dir = config_dir

                # Behaviour-tree missions (mission system): start the tick timer.
                self._maybe_set_missions(agent_node, agent_info)

                pose = self._resolve_spawn_pose(agent_info, i, poses, agent_node)

                self._register_agent(agent_node, executor, spawn_queue, pose)

        except Exception as e:
            logging.error(f"Unexpected error creating agent '{agent_type}': {e}")
            logging.error(traceback.format_exc())

    @staticmethod
    def _maybe_set_missions(agent_node: Any, agent_info: Dict[str, Any]) -> None:
        """If the config declares BT ``missions``, install them on the agent."""
        missions = agent_info.get("missions") if agent_info else None
        if missions and hasattr(agent_node, "set_missions"):
            tick_rate_hz = float(agent_info.get("tick_rate_hz", 1.0))
            agent_node.set_missions(missions, tick_rate_hz)

    @staticmethod
    def _resolve_spawn_pose(agent_info: Dict[str, Any], i: int, poses: List[Any], agent_node: Any) -> Any:
        """Pick the spawn pose: explicit poses > ``spawn`` block > geo > random."""
        if i < len(poses):
            return poses[i]
        spawn = agent_info.get("spawn") if agent_info else None
        if spawn is not None:
            return [
                float(spawn.get("x", 0.0)),
                float(spawn.get("y", 0.0)),
                float(spawn.get("z", 0.0)),
                float(spawn.get("roll", 0.0)),
                float(spawn.get("pitch", 0.0)),
                float(spawn.get("yaw", 0.0)),
            ]
        return utils.generate_random_pose(agent_node.get_first_domain())

    def _create_agent_instance(self, agent_class, agent_sdf, world_name, xdyn_enabled, agent_info=None, agent_index=0):
        """Instantiates a single agent node."""
        if agent_info is None:
            return agent_class(agent_sdf, world_name, xdyn_enabled)
        trajectories = agent_info.get("trajectories")
        loops = agent_info.get("loops")
        if trajectories:
            if agent_index < len(trajectories):
                trajectory = trajectories[agent_index]
                loop = loops[agent_index] if loops else agent_info.get("loop", True)
            else:
                logging.warning(
                    "Agent index %d has no matching patrol file (only %d provided); "
                    "falling back to first trajectory.",
                    agent_index, len(trajectories),
                )
                trajectory = trajectories[0]
                loop = loops[0] if loops else agent_info.get("loop", True)
        else:
            trajectory = agent_info.get("trajectory")
            loop = agent_info.get("loop", True)
        if trajectory is not None:
            return agent_class(agent_sdf, world_name, xdyn_enabled, trajectory=trajectory, loop=loop)
        return agent_class(agent_sdf, world_name, xdyn_enabled)

    def _register_agent(
        self,
        agent_node: Any,
        executor: Any,
        spawn_queue: List[Tuple[Any, Any]],
        pose: Any,
    ) -> None:
        """Registers an agent to the executor and queues it for spawning."""
        self.agents[agent_node.agent_name] = agent_node
        executor.add_node(agent_node)
        spawn_queue.append((agent_node, pose))

    def _spawn_all_agents(self, spawn_queue: List[Tuple[Any, Any]]) -> None:
        """Sends mission/spawn commands to all queued agents."""
        for agent_node, pose in spawn_queue:
            try:
                agent_node.send_single_mas_cmd(pose)
            except Exception as e:
                logging.error(f"Failed to send mission command for {agent_node.agent_name}: {e}")

    # -------------------------------------------------------------------------
    # Dynamic Spawn / Despawn (runtime, called from DynamicSpawnService)
    # -------------------------------------------------------------------------

    def spawn_one_agent(
        self,
        agent_type: str,
        agent_info: Dict[str, Any],
        world_name: str,
        executor: Any,
    ) -> Optional[str]:
        """
        Instantiate, register, and spawn a single agent at runtime.

        Args:
            agent_type: JSON-style agent type name (e.g. 'Bluerov2HeavyBase').
            agent_info: Dict of agent parameters (pose, sdf_file, xdyn, …).
            world_name: Name of the simulation world.
            executor:   Running ROS 2 executor to add the new node to.

        Returns:
            The unique agent name on success, or None on failure.
        """
        try:
            class_name = utils.json_name_to_class_name(agent_type)
            agent_class = utils.find_agent_class_globally(class_name)
            if agent_class is None:
                logging.error("Agent class '%s' not found", class_name)
                return None

            sdf_file = agent_info.get("sdf_file", "") if agent_info else ""
            xdyn_enabled = bool(agent_info.get("xdyn", False)) if agent_info else False

            if agent_info:
                single = {agent_type: agent_info}
                utils.inject_first_ais_pose(single)

            poses = agent_info.get("poses") if agent_info else None
            pose = poses[0] if poses else (agent_info.get("pose") if agent_info else None)

            agent_node = self._create_agent_instance(
                agent_class, sdf_file, world_name, xdyn_enabled, agent_info
            )
            agent_node.sdf_file = sdf_file

            if pose is None:
                pose = utils.generate_random_pose(agent_node.get_first_domain())

            self.agents[agent_node.agent_name] = agent_node
            executor.add_node(agent_node)
            agent_node.send_single_mas_cmd(pose)

            logging.info("Dynamically spawned agent '%s'", agent_node.agent_name)
            return agent_node.agent_name

        except Exception as e:
            logging.error("Failed to dynamically spawn agent '%s': %s", agent_type, e)
            logging.error(traceback.format_exc())
            return None

    def despawn_agent(self, agent_name: str, executor: Any) -> bool:
        """
        Delete a single running agent from the simulation and the executor.

        Args:
            agent_name: Unique name of the agent (e.g. 'bluerov2heavybase0').
            executor:   Running ROS 2 executor the agent node is registered in.

        Returns:
            True if the agent was found and removed, False otherwise.
        """
        agent = self.agents.pop(agent_name, None)
        if agent is None:
            logging.warning("Despawn: agent '%s' not found", agent_name)
            return False

        try:
            agent.send_single_delete_cmd()
        except Exception as e:
            logging.warning("Could not send delete command for '%s': %s", agent_name, e)

        try:
            executor.remove_node(agent)
        except Exception as e:
            logging.warning("Could not remove node '%s' from executor: %s", agent_name, e)

        try:
            agent.destroy_node()
        except Exception as e:
            logging.warning("Could not destroy node '%s': %s", agent_name, e)

        logging.info("Dynamically despawned agent '%s'", agent_name)
        return True
