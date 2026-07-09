from __future__ import annotations

from typing import List, Optional

from lotusim_sdk.bt.node import BehaviorNode
from lotusim_sdk.bt.status import Status


class Composite(BehaviorNode):
    """Abstract parent of all nodes that hold several children and define how
    control flows among them (the user's "sequential vs parallel")."""

    def __init__(self, id: str = "", children: Optional[List[BehaviorNode]] = None) -> None:
        super().__init__(id)
        self.children: List[BehaviorNode] = children or []

    def halt(self) -> None:
        for child in self.children:
            child.halt()

    def reset(self) -> None:
        for child in self.children:
            child.reset()


class Sequence(Composite):
    """Run children left-to-right (logical AND), with *memory* semantics.

    The currently-running child index is remembered so already-completed
    children are not re-ticked:

    - a child returns ``RUNNING``  -> return ``RUNNING`` (same child next tick)
    - a child returns ``FAILURE``  -> return ``FAILURE`` (abort the sequence)
    - a child returns ``SUCCESS``  -> advance to the next child
    - past the last child          -> return ``SUCCESS``
    """

    def __init__(self, id: str = "", children: Optional[List[BehaviorNode]] = None) -> None:
        super().__init__(id, children)
        self._current = 0

    def tick(self) -> Status:
        while self._current < len(self.children):
            status = self.children[self._current].tick()
            if status == Status.RUNNING:
                self.status = Status.RUNNING
                return self.status
            if status == Status.FAILURE:
                self.status = Status.FAILURE
                return self.status
            # SUCCESS -> advance to the next child
            self._current += 1

        self.status = Status.SUCCESS
        return self.status

    def halt(self) -> None:
        super().halt()
        self._current = 0

    def reset(self) -> None:
        super().reset()
        self._current = 0


class Parallel(Composite):
    """Tick all non-terminal children every tick.

    - ``success_policy="all"`` (default) -> ``SUCCESS`` once all children succeeded
    - ``success_policy="one"``           -> ``SUCCESS`` as soon as one succeeds
    - any child ``FAILURE``              -> ``FAILURE``
    - otherwise                          -> ``RUNNING``
    """

    def __init__(
        self,
        id: str = "",
        children: Optional[List[BehaviorNode]] = None,
        success_policy: str = "all",
    ) -> None:
        super().__init__(id, children)
        self.success_policy = success_policy
        self._results: List[Optional[Status]] = [None] * len(self.children)

    def tick(self) -> Status:
        if len(self._results) != len(self.children):
            self._results = [None] * len(self.children)

        for i, child in enumerate(self.children):
            if self._results[i] in (Status.SUCCESS, Status.FAILURE):
                continue  # already terminal — do not re-tick
            self._results[i] = child.tick()

        if any(r == Status.FAILURE for r in self._results):
            self.status = Status.FAILURE
            return self.status

        succeeded = sum(1 for r in self._results if r == Status.SUCCESS)
        if self.success_policy == "one":
            done = succeeded >= 1
        else:  # "all"
            done = succeeded == len(self.children)

        self.status = Status.SUCCESS if done else Status.RUNNING
        return self.status

    def halt(self) -> None:
        super().halt()
        self._results = [None] * len(self.children)

    def reset(self) -> None:
        super().reset()
        self._results = [None] * len(self.children)
