import json
import logging
import os
from abc import ABC, abstractmethod
from datetime import datetime, timedelta
from typing import Any, TypedDict

logger = logging.getLogger(__name__)


class Waypoint(TypedDict):
    timestamp: str
    lat: float
    lon: float


class TrajectoryProvider(ABC):
    """
    Abstract base for all trajectory providers.

    A provider is responsible for one thing: given an ``ais`` config block
    from the scenario JSON, load and return an ordered list of geo-timestamped
    waypoints.

    The three methods below form the full contract that every provider must
    implement.  The factory function ``get_trajectory_provider()`` calls them
    in this order:

        1. ``accepts()``    — "can I handle this config?"
        2. ``from_config()`` — "build me from this config"
        3. ``load()``       — "give me the waypoints"

    To add a new trajectory source, subclass this, implement the three methods,
    and append the class to ``_PROVIDERS``.  No other file needs to change.
    """

    @abstractmethod
    def load(self) -> list[Waypoint]:
        """
        Load and return the trajectory as an ordered list of waypoints.

        Returns
        -------
        list[Waypoint]
            Ordered list of ``{'timestamp': str, 'lat': float, 'lon': float}``
            dicts.  The first entry is used as the agent spawn position, so the
            list must never be empty (raise ``ValueError`` if it would be).
        """
        ...

    @classmethod
    @abstractmethod
    def accepts(cls, config: dict[str, Any]) -> bool:
        """Return ``True`` if this provider knows how to handle the given config."""
        ...

    @classmethod
    @abstractmethod
    def from_config(cls, config: dict[str, Any], base_dir: str = None) -> "TrajectoryProvider":
        """
        Construct and return a provider instance from the ``ais`` config block.

        Parameters
        ----------
        config:
            The ``ais`` dict extracted from the scenario JSON.
        base_dir:
            Optional directory used to resolve relative patrol file paths.
            Typically the directory containing the scenario JSON.
        """
        ...


class PatrolFileProvider(TrajectoryProvider):
    """
    Loads a patrol trajectory from a JSON file.

    The file is resolved in this order:
        1. Absolute path (if ``patrol_file`` is absolute).
        2. Relative to ``base_dir`` (the directory of the scenario JSON).
        3. Relative to the current working directory.

    Config keys
    -----------
    patrol_file : str
        Filename or path, e.g. ``"WP-WT1.json"``.

    Expected JSON format::

        {
            "waypoints": [
                {"timestamp": "2023-09-15T08:39:28", "lat": 50.648, "lon": -1.077},
                ...
            ]
        }
    """

    def __init__(self, patrol_file: str, base_dir: str = None) -> None:
        self._file = patrol_file
        self._base_dir = base_dir

    @classmethod
    def accepts(cls, config: dict[str, Any]) -> bool:
        return "patrol_file" in config

    @classmethod
    def from_config(cls, config: dict[str, Any], base_dir: str = None) -> "PatrolFileProvider":
        return cls(config["patrol_file"], base_dir=base_dir)

    def load(self) -> list[Waypoint]:
        path = self._resolve_path(self._file, self._base_dir)
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            waypoints = data.get("waypoints")
        else:
            waypoints = data
        if not waypoints:
            raise ValueError(f"Patrol file '{self._file}' contains no waypoints.")
        trajectory = [
            {"timestamp": wp["timestamp"], "lat": float(wp["lat"]), "lon": float(wp["lon"])}
            for wp in waypoints
        ]
        logger.debug("PatrolFileProvider: loaded %d waypoints from '%s'", len(trajectory), self._file)
        return trajectory

    @staticmethod
    def _resolve_path(filename: str, base_dir: str = None) -> str:
        if os.path.isabs(filename) and os.path.exists(filename):
            return filename
        candidates = []
        if base_dir:
            candidates.append(os.path.join(base_dir, filename))
        candidates.append(os.path.join(os.getcwd(), filename))
        for path in candidates:
            if os.path.exists(path):
                return path
        raise FileNotFoundError(
            f"Patrol file '{filename}' not found. Searched in: {', '.join(candidates)}"
        )


class WaypointListProvider(TrajectoryProvider):
    """
    Builds a trajectory from an inline waypoint list embedded in the config.

    Config keys
    -----------
    waypoints : list
        List of dicts ``{"lat": ..., "lon": ..., "timestamp": ...}`` (timestamp
        optional), or ``[lat, lon]`` pairs.
    start_time : str, optional
        ISO timestamp for the first generated timestamp (default: UTC now).
    interval_minutes : int, optional
        Minutes between generated timestamps when none are provided (default: 2).
    """

    def __init__(
        self,
        waypoints: list,
        start_time: datetime,
        interval_minutes: int,
    ) -> None:
        self._waypoints = waypoints
        self._start_time = start_time
        self._interval = interval_minutes

    @classmethod
    def accepts(cls, config: dict[str, Any]) -> bool:
        return "waypoints" in config

    @classmethod
    def from_config(cls, config: dict[str, Any], base_dir: str = None) -> "WaypointListProvider":
        raw_start = config.get("start_time")
        start_time = datetime.fromisoformat(raw_start) if raw_start else datetime.utcnow()
        return cls(
            waypoints=config["waypoints"],
            start_time=start_time,
            interval_minutes=config.get("interval_minutes", 2),
        )

    def load(self) -> list[Waypoint]:
        if not self._waypoints:
            raise ValueError("WaypointListProvider: waypoints list is empty.")
        trajectory = []
        for i, wp in enumerate(self._waypoints):
            if isinstance(wp, dict):
                lat = float(wp["lat"])
                lon = float(wp["lon"])
                timestamp = wp.get(
                    "timestamp",
                    (self._start_time + timedelta(minutes=i * self._interval)).isoformat(),
                )
            else:
                lat, lon = float(wp[0]), float(wp[1])
                timestamp = (self._start_time + timedelta(minutes=i * self._interval)).isoformat()
            trajectory.append({"timestamp": timestamp, "lat": lat, "lon": lon})
        logger.debug("WaypointListProvider: loaded %d waypoints", len(trajectory))
        return trajectory


# ---------------------------------------------------------------------------
# Registry & factory — add new providers here, nowhere else
# ---------------------------------------------------------------------------

_PROVIDERS: list[type[TrajectoryProvider]] = [
    PatrolFileProvider,
    WaypointListProvider,
]


def get_trajectory_provider(config: dict[str, Any], base_dir: str = None) -> TrajectoryProvider:
    """
    Return the first registered provider that accepts the given ``ais`` config block.

    Parameters
    ----------
    config:
        The ``ais`` dict from the scenario JSON.
    base_dir:
        Directory used to resolve relative patrol file paths (typically the
        directory containing the scenario JSON).

    Raises
    ------
    ValueError
        If no registered provider matches the config keys.
    """
    for cls in _PROVIDERS:
        if cls.accepts(config):
            return cls.from_config(config, base_dir=base_dir)
    raise ValueError(
        f"No trajectory provider found for config keys: {list(config.keys())}. "
        f"Expected one of: {', '.join(cls.__name__ for cls in _PROVIDERS)}"
    )
