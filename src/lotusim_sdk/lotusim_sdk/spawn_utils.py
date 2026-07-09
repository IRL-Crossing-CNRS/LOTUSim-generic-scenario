"""Helpers shared by the remote runner (lotusim_client) and the host runner (simulation_run)
for deriving a spawn position from a BT mission tree when no explicit spawn block is given."""

import logging

logger = logging.getLogger(__name__)


def find_waypoints_file_in_missions(nodes: list) -> "str | None":
    """Recursively scan a BT mission tree for the first waypoint_follower waypoints_file param."""
    for node in nodes:
        if node.get("type") == "action" and node.get("task") == "waypoint_follower":
            wf = node.get("params", {}).get("waypoints_file")
            if wf:
                return wf
        result = find_waypoints_file_in_missions(node.get("children", []))
        if result:
            return result
    return None


def extract_spawn_from_missions(agent_info: dict, config_dir: str = None) -> None:
    """
    If no explicit spawn is set, populate ``lat``/``lon``/``alt`` on ``agent_info``
    from the first waypoint of the ``waypoint_follower`` task's ``waypoints_file``
    param so the runner can derive a spawn position.

    No-op when ``spawn``, ``pose``, ``poses``, or ``lat`` is already present.
    """
    if any(k in agent_info for k in ("spawn", "pose", "poses", "lat")):
        return
    missions = agent_info.get("missions") or []
    waypoints_file = find_waypoints_file_in_missions(missions)
    if not waypoints_file:
        return
    from lotusim_sdk.trajectory_providers import PatrolFileProvider
    try:
        waypoints = PatrolFileProvider(waypoints_file, base_dir=config_dir).load()
    except Exception as e:
        logger.warning("Could not load '%s' for spawn fallback: %s", waypoints_file, e)
        return
    if not waypoints:
        return
    first = waypoints[0]
    agent_info["lat"] = first["lat"]
    agent_info["lon"] = first["lon"]
    logger.info("Spawn fallback from '%s': lat=%.6f lon=%.6f", waypoints_file, first["lat"], first["lon"])
