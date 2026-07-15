"""
@file mission_watcher.py
@brief Logs one clear message when every agent's mission has finished.

Each agent is its own ROS node and only knows about its own missions, so
``Agent._tick_missions`` (lotusim_sdk) logs completion per-mission
(``Mission '<id>' finished with SUCCESS; ...``) with no signal for "the whole
scenario is done" — that requires visibility across all agents, which only
this process's :class:`AgentsManager` registry has.

Missions marked ``TaskAgent.PERPETUAL`` (e.g. ``kinematic_anchor`` on mines,
which is designed to stay RUNNING forever so the mine keeps drifting with the
current) are excluded, otherwise this would never fire in a scenario that
has any such prop.
"""

from rclpy.node import Node

from lotusim_sdk.bt.status import Status


class MissionWatcher(Node):
    """Polls the shared :class:`AgentsManager` registry and logs once every
    non-perpetual mission root has left RUNNING."""

    def __init__(self, agents_manager, world_name: str, poll_period_s: float = 1.0) -> None:
        super().__init__(f"mission_watcher_{world_name}")
        self._agents_manager = agents_manager
        self._reported = False
        self._timer = self.create_timer(poll_period_s, self._check)

    def _trackable_roots(self):
        for agent in self._agents_manager.agents.values():
            for root in getattr(agent, "_missions", []):
                if not getattr(root, "PERPETUAL", False):
                    yield root

    def _check(self) -> None:
        if self._reported:
            return
        roots = list(self._trackable_roots())
        if not roots:
            return
        if all(root.status != Status.RUNNING for root in roots):
            self._reported = True
            self.get_logger().info(
                f"All missions complete ({len(roots)} tracked) — scenario finished."
            )
