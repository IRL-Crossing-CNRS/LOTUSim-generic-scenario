"""
@file agent.py
@brief Minimal custom agent, with its own custom BT task, both in one file.

Shows the full "PhD ships their own wheel" story from doc/MISSIONS.md end to
end in one place: a thin agent subclass (registered via ``lotusim.agents``)
that wires up ``BlinkLightTask`` (also registered via ``lotusim.tasks``, so it
stays usable from a scenario JSON on any agent) directly in ``__init__`` via
``self._missions.add_task(...)`` — the same code-built pattern as
``deployment/src/example_agent/example_agent/my_agent.py``. No "missions" key
is needed in the scenario JSON; the agent runs its task the moment it is
spawned.

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0
"""

from __future__ import annotations

from lotusim_sdk.agents.entity.physical.bluerov2_heavy import Bluerov2Heavy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent


class BlinkLightTask(TaskAgent):
    """Blink the agent's LED at a fixed period, forever.

    Works with any stock agent (no custom agent class needed) as long as the
    spawned model exposes a ``/{world}/{agent}/light/cmd`` topic (e.g.
    ``model-battery.sdf``, which bundles the ``light_actuator`` plugin).

    Params (mission JSON ``"params"``):
        period_s  float  half-period of the blink in seconds (default 1.0).
    """

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        self._period_s = float(self.params.get("period_s", 1.0))
        self._pub = None
        self._timer = None
        self._on = False

    def on_enter(self) -> None:
        topic = f"/{self.host.world_name}/{self.host.agent_name}/light/cmd"
        # Must match the light_actuator's declared QoS or DDS never links the two.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.host.create_publisher(Bool, topic, qos)
        self._timer = self.host.create_timer(self._period_s, self._toggle)
        self.host.get_logger().info(
            f"BlinkLightTask active on {self.host.agent_name} (period={self._period_s}s)."
        )

    def update(self) -> Status:
        return Status.RUNNING

    def on_exit(self, _status: Status) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self.host.destroy_timer(self._timer)
            self._timer = None
        if self._pub is not None:
            self._pub.publish(Bool(data=False))
            self.host.destroy_publisher(self._pub)
            self._pub = None

    def _toggle(self) -> None:
        self._on = not self._on
        self._pub.publish(Bool(data=self._on))


class CustomTaskDemoAgent(Bluerov2Heavy):
    """BlueROV2 Heavy agent that blinks its LED, mission built in code.

    Unlike the other example scenarios (mission described in the JSON's
    ``missions`` block), this agent attaches its own ``BlinkLightTask``
    straight onto ``self._missions`` here, so the class is fully self-contained
    — a scenario just has to spawn ``CustomTaskDemoAgent``.
    """

    def __init__(self, sdf_string: str, world_name: str, xdyn_enabled: bool, **kwargs):
        super().__init__(sdf_string, world_name, xdyn_enabled)
        blink = BlinkLightTask(
            host=self,
            params={"period_s": 0.5},
            blackboard=self._blackboard,
            id="blink",
        )
        self._missions.add_task(blink)
