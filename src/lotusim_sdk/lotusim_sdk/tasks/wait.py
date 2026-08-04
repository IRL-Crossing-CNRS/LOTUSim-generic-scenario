from __future__ import annotations

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent


class WaitTask(TaskAgent):
    """Stay RUNNING for ``duration_s``, then succeed. Any agent.

    A pure pause: it reads nothing and changes nothing but its own progress, so
    it is safe to drop into any ``sequence``/``parallel`` next to any other
    task — a ``waypoint_follower`` mission, a ``Wind`` schedule, anything.

    Exists mainly to space out a sequence of instantly-succeeding actions (e.g.
    ``set_wind`` -> wait -> ``set_wind`` -> wait -> ...): without it, a
    `Sequence` advances through every immediately-succeeding child in the same
    tick (see :class:`~lotusim_sdk.bt.composites.Sequence`), so several such
    actions in a row apply instantly and only the last one is ever observed.

    params: ``duration_s`` (float, default ``0.0``).

    Uses the node clock (``self.host.get_clock()``), same as everything else in
    this codebase — no node here sets ``use_sim_time`` or subscribes to
    ``/clock``, so this is wall-clock time throughout, at any Gazebo RTF (see
    ``doc/ACCELERATED_SIMULATION.md``: ROS-side timers already run independently
    of RTF).
    """

    def on_enter(self) -> None:
        self._start_s = self.host.get_clock().now().nanoseconds / 1e9

    def update(self) -> Status:
        duration_s = float(self.params.get("duration_s", 0.0))
        elapsed = self.host.get_clock().now().nanoseconds / 1e9 - self._start_s
        if elapsed >= duration_s:
            return Status.SUCCESS
        return Status.RUNNING
