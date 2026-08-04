from __future__ import annotations

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent


class SetWindTask(TaskAgent):
    """Set the ambient wind vector, then succeed.

    params: ``x`` / ``y`` / ``z`` — the wind velocity in m/s in the world ENU
    frame (x=East, y=North, z=Up), i.e. exactly the vector the Unity wind sliders
    write. Omitted components keep their current value.

    Host must be a :class:`~lotusim_sdk.agents.environment.wind.Wind` agent, so
    this goes under that agent's ``missions``. In a ``sequence`` it is what makes
    a scripted wind history possible — set, run a mission, set again.
    """

    def update(self) -> Status:
        host = self.host
        if not hasattr(host, "set_wind"):
            raise TypeError(
                f"Task '{self.id or type(self).__name__}' needs a Wind agent as its "
                f"host, got {type(host).__name__}. Declare it under the Wind agent's "
                f"'missions' in the scenario JSON."
            )

        host.set_wind(
            x=self.params.get("x"),
            y=self.params.get("y"),
            z=self.params.get("z"),
        )
        x, y, z = host.get_wind()
        host.get_logger().info(f"[{self.id}] wind set to [{x:.2f}, {y:.2f}, {z:.2f}] m/s")
        return Status.SUCCESS
