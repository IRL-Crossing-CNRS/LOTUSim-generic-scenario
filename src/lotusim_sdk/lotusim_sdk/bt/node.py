from __future__ import annotations

from abc import ABC, abstractmethod

from lotusim_sdk.bt.status import Status


class BehaviorNode(ABC):
    """Abstract base of every node in a behaviour tree.

    The only mandatory method is :meth:`tick`. This common interface is what
    lets a parent tick *any* child uniformly — a composite stores
    ``children: list[BehaviorNode]`` and calls ``child.tick()`` without knowing
    whether the child is another composite or a leaf task. That polymorphism is
    what makes arbitrary nesting work.
    """

    def __init__(self, id: str = "") -> None:
        self.id = id
        self.status = Status.RUNNING

    @abstractmethod
    def tick(self) -> Status:
        """Advance this node by one tick and return its :class:`Status`."""

    def halt(self) -> None:
        """Preempt: stop this subtree (called by a parent that moved on)."""

    def reset(self) -> None:
        """Return to the initial state so the subtree can run again."""
