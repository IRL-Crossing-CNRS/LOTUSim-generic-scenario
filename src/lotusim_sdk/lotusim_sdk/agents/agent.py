from abc import ABC

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from lotusim_sdk.bt.blackboard import Blackboard
from lotusim_sdk.bt.builder import build_tree, load_task_registry

DEFAULT_TICK_RATE_HZ = 1.0


class MissionSet(list):
    """A list of behaviour-tree roots, ticked together by the agent.

    ``add_task`` lets an agent register a code-built task directly, without
    going through a JSON mission spec: ``self._missions.add_task(my_task)``.
    """

    def add_task(self, task) -> None:
        self.append(task)


class Agent(Node, ABC):
    """Root abstract class for all LOTUSim agents.

    Carries the behaviour-tree mission engine: a set of BT roots ticked on a
    periodic timer. Missions can come from JSON specs (:meth:`set_missions`) or
    be built in code and added directly (``self._missions.add_task(...)``).
    """

    _next_model_num = 0

    @classmethod
    def get_unique_model_num(cls) -> int:
        """Return a unique incremental instance number per concrete class."""
        num = cls._next_model_num
        cls._next_model_num += 1
        return num

    def __init__(self, node_name: str, world_name: str):
        self.world_name = world_name
        super().__init__(node_name)
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Behaviour-tree mission engine. The timer always runs so a task added
        # in a subclass __init__ (no JSON) still gets ticked.
        self._missions = MissionSet()
        self._blackboard = Blackboard()
        self._tick_rate_hz = DEFAULT_TICK_RATE_HZ
        self._missions_started = False
        self._mission_timer = self.create_timer(
            1.0 / self._tick_rate_hz, self._tick_missions
        )

    # ------------------------------------------------------------------
    # Behaviour-tree missions
    # ------------------------------------------------------------------
    def set_missions(self, specs, tick_rate_hz: float = DEFAULT_TICK_RATE_HZ) -> None:
        """Build mission trees from their JSON specs and (re)start ticking them.

        The built trees are *added* to the mission set, so they coexist with any
        task a subclass registered in code via ``self._missions.add_task(...)``.
        """
        registry = load_task_registry()
        self._missions.extend(
            build_tree(spec, self, self._blackboard, registry) for spec in specs
        )
        self._tick_rate_hz = tick_rate_hz
        self._missions_started = False
        if self._mission_timer is not None:
            self._mission_timer.cancel()
        self._mission_timer = self.create_timer(1.0 / tick_rate_hz, self._tick_missions)

    def missions_ready(self) -> bool:
        """Gate for the first tick. Base agents are always ready; entities with a
        simulated model override this to wait until they are actually spawned."""
        return True

    def _tick_missions(self) -> None:
        if not self._missions:
            return
        if not self._missions_started:
            if not self.missions_ready():
                return
            self._missions_started = True
        for root in self._missions:
            root.tick()

    def destroy_node(self) -> None:
        if self._mission_timer is not None:
            self._mission_timer.cancel()
            self._mission_timer = None
        for root in self._missions:
            root.halt()
        super().destroy_node()
