"""
@file utils.py
@author Naval Group
@brief Utility functions for managing simulation management.

@details
This module provides helper functions for the LOTUSim simulation framework,
covering tasks such as configuration handling, SDF parsing, agent setup,
and simulation parameter generation.

Key functionalities include:

- Loading and parsing JSON configuration files.
- Extracting world names from Gazebo SDF files.
- Loading agent SDFs with parent-class fallback.
- Generating randomized spawn poses for agents.
- Creating LOTUSim XML blocks for agent connections (ROS2/XDyn).
- Comparing XML strings for structural equivalence.
- Converting JSON-style agent names to Python class names (PascalCase).
- Dynamic discovery of agent classes in the workspace.

These utilities support simulation initialization, agent spawning, and
bridge setup processes across the LOTUSim ecosystem.

These utilities are used across simulation initialization, agent spawning, and bridge setup processes.

@version 0.1
@date 2026-03-04

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

import argparse
import json
import logging
import os
import random
import re
import xml.etree.ElementTree as ET

from importlib.metadata import entry_points
from typing import Any, Dict, List, Tuple



logger = logging.getLogger(__name__)

# ----------------------------------------------------------------------
# CLI & Configuration Helpers
# ----------------------------------------------------------------------


def get_cli_args() -> argparse.Namespace:
    """
    Parse command-line arguments for simulation launch.

    Returns:
        Parsed arguments (config file, debug flag).
    """
    parser = argparse.ArgumentParser(description="Launch a Lotusim simulation")
    parser.add_argument("--config", type=str, required=True, help="Configuration JSON file name")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode for verbose simulation output")
    parser.add_argument("--gui", action="store_true", help="Enable Gazebo gui")
    return parser.parse_args()


def parse_simulation_config(config: Dict[str, Any]) -> Tuple[str, Any, bool]:
    """
    Parse the simulation configuration and extract only the essential data.

    Args:
        config: Loaded JSON configuration.

    Returns:
        tuple: (
            world_file (str),
            agents: full agent data as in JSON — a dict (legacy) or a list
                (mission system); both are accepted downstream,
            aerial_enabled (bool)
        )
    """
    world_file = config.get("world_file", "")
    agents = config.get("agents", {})
    aerial_enabled = bool(config.get("aerial_domain", False))

    return world_file, agents, aerial_enabled


# ----------------------------------------------------------------------
# Configuration & World Helpers
# ----------------------------------------------------------------------


def load_config_from_json(config_path: str) -> Dict[str, Any]:
    """
    Loads a JSON configuration file.

    Args:
        config_path (str): Path to the JSON configuration file.

    Returns:
        Dict[str, Any]: Parsed configuration as a dictionary.
    """
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def get_world_name(world_file_name: str) -> str:
    """
    Extracts the <world> name from a Gazebo world SDF file.

    Args:
        world_file_name (str): Name of the world file (e.g., 'ocean.world').

    Returns:
        str: The extracted world name.

    Raises:
        FileNotFoundError: If the world file does not exist.
        RuntimeError: If the world name cannot be parsed.
    """
    world_path = os.path.join(os.environ.get("LOTUSIM_PATH", ""), "assets", "worlds", world_file_name)
    if not os.path.exists(world_path):
        raise FileNotFoundError(f"World file not found: {world_path}")

    try:
        tree = ET.parse(world_path)
        world_elem = tree.getroot().find("world")
        if world_elem is None:
            raise ValueError(f"No <world> tag found in {world_file_name}")
        return world_elem.attrib.get("name", "")
    except Exception as e:
        raise RuntimeError(f"Error extracting world name from {world_file_name}: {e}")


# ----------------------------------------------------------------------
# Agent Spawn & Pose Utilities
# ----------------------------------------------------------------------

ENTITY_SPAWN_PARAMS = {
    "Underwater": {"x_range": (-100, 100), "y_range": (-100, 100), "z_range": (-80, -20)},
    "Surface": {"x_range": (-100, 100), "y_range": (-100, 10), "z_range": (0, 0)},
    "Aerial": {"x_range": (-100, 100), "y_range": (-100, 10), "z_range": (15, 50)},
}


def generate_random_pose(agent_first_domain: str) -> List[float]:
    """
    Generates a random [x, y, z, roll, pitch, yaw] pose based on agent domain.

    Args:
        agent_first_domain (str): Domain of the agent ('Underwater', 'Surface', 'Aerial').

    Returns:
        List[float]: Random pose [x, y, z, roll, pitch, yaw].
    """
    params = ENTITY_SPAWN_PARAMS[agent_first_domain]
    x = random.uniform(*params["x_range"])
    y = random.uniform(*params["y_range"])
    z = random.uniform(*params["z_range"])
    return [x, y, z, 0.0, 0.0, 0.0]


def inject_first_ais_pose(agents, base_dir: str = None) -> None:
    """
    Inject spawn position into agent configurations.

    - Mission (list) form: extracts spawn from the ``waypoints_file`` param of
      the first ``waypoint_follower`` task found in the BT tree, if no explicit
      ``spawn``/``pose``/``lat`` is already set.
    - Legacy (dict) form: reads the ``ais`` config block as before.
    """
    from lotusim_sdk.spawn_utils import extract_spawn_from_missions
    if isinstance(agents, list):
        for agent_cfg in agents:
            extract_spawn_from_missions(agent_cfg, config_dir=base_dir)
        return
    if not isinstance(agents, dict):
        return
    from lotusim_sdk.trajectory_providers import PatrolFileProvider, get_trajectory_provider
    for agent_name, agent_cfg in agents.items():
        ais_config = agent_cfg.get("ais")
        if not ais_config:
            continue

        patrol_file = ais_config.get("patrol_file")
        nb_agents = agent_cfg.get("nb_agents", 1)

        if isinstance(patrol_file, list):
            if len(patrol_file) > nb_agents:
                logger.warning(
                    "Agent '%s': %d patrol files provided but only %d agent(s) will be spawned; "
                    "extra patrol files will be ignored.",
                    agent_name, len(patrol_file), nb_agents,
                )
            elif len(patrol_file) < nb_agents:
                logger.warning(
                    "Agent '%s': %d patrol files provided but %d agent(s) requested; "
                    "agents beyond index %d will reuse the first trajectory.",
                    agent_name, len(patrol_file), nb_agents, len(patrol_file) - 1,
                )

            trajectories = []
            loops = []
            default_loop = agent_cfg.get("loop", True)
            for entry in patrol_file:
                if isinstance(entry, dict):
                    pf = entry["file"]
                    loops.append(entry.get("loop", default_loop))
                else:
                    pf = entry
                    loops.append(default_loop)
                traj = PatrolFileProvider(pf, base_dir=base_dir).load()
                if not traj:
                    raise ValueError(f"Trajectory '{pf}' for agent '{agent_name}' is empty.")
                trajectories.append(traj)
            agent_cfg["trajectories"] = trajectories
            agent_cfg["loops"] = loops
            agent_cfg["trajectory"] = trajectories[0]
            agent_cfg["lat"] = trajectories[0][0]["lat"]
            agent_cfg["lon"] = trajectories[0][0]["lon"]
            logger.info("Loaded %d trajectories for '%s'", len(trajectories), agent_name)
        else:
            if nb_agents > 1:
                logger.info(
                    "Agent '%s': single patrol file '%s' will be shared across all %d agents.",
                    agent_name, patrol_file, nb_agents,
                )
            provider = get_trajectory_provider(ais_config, base_dir=base_dir)
            trajectory = provider.load()
            if not trajectory:
                raise ValueError(f"Trajectory for agent '{agent_name}' is empty.")
            logger.info("Loaded trajectory for '%s': %d waypoints", agent_name, len(trajectory))
            agent_cfg["lat"] = trajectory[0]["lat"]
            agent_cfg["lon"] = trajectory[0]["lon"]
            agent_cfg["trajectory"] = trajectory


# ----------------------------------------------------------------------
# XML Utilities
# ----------------------------------------------------------------------


def xml_equivalent(xml1: str, xml2: str) -> bool:
    """
    Compares two XML strings for structural equivalence.

    Args:
        xml1 (str): First XML string.
        xml2 (str): Second XML string.

    Returns:
        bool: True if equivalent, False otherwise.
    """

    def normalize(elem):
        for e in elem.iter():
            e.text = (e.text or "").strip()
            e.tail = (e.tail or "").strip()
        return elem

    try:
        e1 = normalize(ET.fromstring(xml1))
        e2 = normalize(ET.fromstring(xml2))
        return ET.tostring(e1) == ET.tostring(e2)
    except ET.ParseError:
        logger.exception("XML parse error while comparing XML strings")
        return False


# ----------------------------------------------------------------------
# Agent Class Utilities
# ----------------------------------------------------------------------


def json_name_to_class_name(json_name: str) -> str:
    """
    Convert a JSON-style agent name (e.g., 'Bluerov2_heavy' or 'Lrauv_Propeller')
    into a proper Python class name in PascalCase (PEP8).

    Examples:
        'Bluerov2_heavy'  -> 'BlueROV2Heavy'
        'Lrauv_Propeller' -> 'LrauvPropeller'
    """
    parts = json_name.replace("-", "_").split("_")
    return "".join(part[0].upper() + part[1:] if part else "" for part in parts)


def normalize_agent_name(name: str) -> str:
    # Convert PascalCase or snake_case to lowercase with underscores
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)  # PascalCase -> Pascal_Case
    s2 = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1)
    return s2.lower().replace("-", "_")


def find_agent_class_globally(agent_name: str):
    """Load agent class from entry points declared by ROS2 agent packages."""
    eps = entry_points(group="lotusim.agents")
    if not eps:
        return None
    normalized = normalize_agent_name(agent_name)
    for ep in eps:
        if normalize_agent_name(ep.name) == normalized:
            return ep.load()


def _extract_world_spherical_coords(world_file_name: str) -> Tuple[float, float]:
    """Read (latitude_deg, longitude_deg) from a world SDF's <spherical_coordinates> block."""
    world_path = os.path.join(os.environ.get("LOTUSIM_PATH", ""), "assets", "worlds", world_file_name)
    tree = ET.parse(world_path)
    sc = tree.getroot().find(".//spherical_coordinates")
    if sc is None:
        raise RuntimeError(f"No <spherical_coordinates> block found in {world_file_name}")
    return float(sc.find("latitude_deg").text), float(sc.find("longitude_deg").text)


