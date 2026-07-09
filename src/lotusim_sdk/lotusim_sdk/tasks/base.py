from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

from lotusim_sdk.bt.node import BehaviorNode
from lotusim_sdk.bt.status import Status

if TYPE_CHECKING:
    from lotusim_sdk.agents.agent import Agent
    from lotusim_sdk.bt.blackboard import Blackboard


class TaskAgent(BehaviorNode, ABC):
    """Leaf base for a behaviour-tree mission task.

    A custom task is ``class MyTask(TaskAgent)`` implementing only
    :meth:`update`. The base provides a *template* :meth:`tick` that drives the
    per-activation lifecycle (``on_enter`` -> repeated ``update`` -> ``on_exit``);
    tasks must **not** override ``tick``.

    The two hooks ``on_enter`` / ``on_exit`` fire **per activation** — each time
    the leaf becomes RUNNING / leaves RUNNING — not once per spawn.
    """

    def __init__(
        self,
        host: "Agent",
        params: Optional[dict] = None,
        blackboard: Optional["Blackboard"] = None,
        id: str = "",
    ) -> None:
        super().__init__(id)
        self.host = host                 # the rclpy.Node — ROS / clock / logging
        self.params = params or {}       # the "params" block from JSON
        self.blackboard = blackboard     # shared store (unused by the demo tasks)
        self._running = False

    @abstractmethod
    def update(self) -> Status:
        """The work of one tick. Returns SUCCESS / FAILURE / RUNNING."""

    def on_enter(self) -> None:
        """Optional: set up on first RUNNING (clients, timers, counters)."""

    def on_exit(self, status: Status) -> None:
        """Optional: clean up when leaving RUNNING."""

    def tick(self) -> Status:  # template — DO NOT override in tasks
        if not self._running:
            self.on_enter()
            self._running = True
        status = self.update()
        if status != Status.RUNNING:
            self.on_exit(status)
            self._running = False
        self.status = status
        return status

    def halt(self) -> None:
        if self._running:
            self.on_exit(Status.FAILURE)
            self._running = False

    def reset(self) -> None:
        self._running = False
        self.status = Status.RUNNING
