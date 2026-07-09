"""Example remote LOTUSim agent with a code-built task (no JSON needed).

The agent builds its own HelloRemoteTask and registers it directly on the
behaviour-tree mission set. The mission timer ticks it once the agent is present
in the simulation — logging on the remote process where this agent runs.
"""

from lotusim_sdk import *
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class HelloRemoteTask(TaskAgent):
    def on_enter(self) -> None:
        self._next = 0.0  # next instant to print (0 => print right away)

    def update(self) -> Status:
        self.host.get_logger().info("hello world from the remote")
        return Status.RUNNING


class BlinkLightTask(TaskAgent):
    """Blink the ROV's LED by toggling /{world}/{agent}/light/cmd (std_msgs/Bool).

    The light_actuator subscribes TRANSIENT_LOCAL + KeepLast(1), so we MUST
    publish with the same QoS or DDS never links the two and nothing happens.
    """

    def on_enter(self) -> None:
        topic = f"/{self.host.world_name}/{self.host.agent_name}/light/cmd"
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.host.create_publisher(Bool, topic, qos)
        self._on = False

    def update(self) -> Status:
        self._on = not self._on
        self._pub.publish(Bool(data=self._on))
        return Status.RUNNING

    def on_exit(self, status: Status) -> None:
        # Leave the light off and drop the publisher when the task stops.
        self._pub.publish(Bool(data=False))
        self.host.destroy_publisher(self._pub)


class CountToFiveTask(TaskAgent):
    """Count up to a target (one step per tick), then SUCCEED.

    Returning SUCCESS is what lets a Sequence advance to its next child — here
    it gates the parallel hello+blink phase: nothing else runs until 5 is reached.
    """

    def on_enter(self) -> None:
        self._target = int(self.params.get("target", 5))
        self._count = 0

    def update(self) -> Status:
        self._count += 1
        self.host.get_logger().info(f"count {self._count}/{self._target}")
        if self._count >= self._target:
            return Status.SUCCESS
        return Status.RUNNING


class MyBluerov(Bluerov2Heavy):
    def __init__(self, sdf_string, world_name, xdyn_enabled, **kwargs):
        super().__init__(sdf_string, world_name, xdyn_enabled)
        self.renderer_type_name = "bluerov2_heavy_inspection"

        counter = CountToFiveTask(
            host=self, params={"target": 5}, blackboard=self._blackboard, id="counter"
        )
        hello = HelloRemoteTask(
            host=self, blackboard=self._blackboard, id="hello"
        )
        blink = BlinkLightTask(
            host=self, blackboard=self._blackboard, id="blink"
        )

        # count to 5  THEN  (hello & blink running together, forever).
        mission = Sequence(id="mission", children=[
            counter,
            Parallel(id="hello_and_blink", children=[hello, blink]),
        ])
        self._missions.add_task(mission)
