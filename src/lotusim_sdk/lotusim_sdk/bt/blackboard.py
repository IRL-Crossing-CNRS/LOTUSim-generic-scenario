from __future__ import annotations

from typing import Any, Dict


class Blackboard:
    """A simple key/value store owned by the agent and shared by the whole tree.

    It is the async->sync bridge for the mission: a ROS callback writes the
    latest value, and a task reads it back synchronously inside ``update()``.
    The three demo tasks do not share any data, so it is currently unused — it
    exists so the API is stable when real shared-state tasks arrive.
    """

    def __init__(self) -> None:
        self._data: Dict[str, Any] = {}

    def get(self, key: str, default: Any = None) -> Any:
        return self._data.get(key, default)

    def set(self, key: str, value: Any) -> None:
        self._data[key] = value

    def __contains__(self, key: str) -> bool:
        return key in self._data
