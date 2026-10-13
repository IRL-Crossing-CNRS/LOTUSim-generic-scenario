from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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

    #: Set True on a task that is designed to stay RUNNING forever. Excludes
    #: it from scenario-wide "all missions complete" tracking
    #: (:class:`~simulation_run.mission_watcher.MissionWatcher`), which would
    #: otherwise never fire once such a task is part of the scenario.
    PERPETUAL: bool = False

    def __init__(
        self,
        host: "Agent",
        params: Optional[dict] = None,
        blackboard: Optional["Blackboard"] = None,
        id: str = "",
    ) -> None:
        super().__init__(id)
        self.host = host  # the rclpy.Node — ROS / clock / logging
        self.params = params or {}  # the "params" block from JSON
        self.blackboard = blackboard  # shared store (unused by the demo tasks)
        self._running = False
        self._callback_group: Optional[MutuallyExclusiveCallbackGroup] = None

    @property
    def callback_group(self) -> MutuallyExclusiveCallbackGroup:
        """This task's own executor queue. Pass it to every ``self.host.create_*``.

        Without it a task's timers and subscriptions land in the host node's
        *default* MutuallyExclusiveCallbackGroup — the same one that serves the
        agent's behaviour-tree tick. Only one thread at a time services such a
        group, so a callback that runs longer than the tick period holds the tick
        off; and because a missed timer deadline is dropped rather than queued,
        the mission tick rate silently halves instead of merely jittering.

        One group per task, not one shared by all: tasks then cannot block each
        other either, while each task's own callbacks stay serialised, which is
        what they were written to assume (no internal locking).
        """
        if self._callback_group is None:
            self._callback_group = MutuallyExclusiveCallbackGroup()
        return self._callback_group

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
