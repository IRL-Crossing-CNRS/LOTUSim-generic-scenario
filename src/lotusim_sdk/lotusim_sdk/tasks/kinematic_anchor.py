from __future__ import annotations

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent


class KinematicAnchorTask(TaskAgent):
    """Keeps a thruster-less prop (e.g. a mine) spawned as a Kinematic entity
    without ever commanding it to move.

    ``PhysicalEntity._lotus_blocks()`` only emits ``<connection_type>Kinematic
    </connection_type>`` when ``host._kinematic_guidance`` was set truthy
    before spawn — normally done by ``WaypointFollowerTask``. A prop with no
    thrusters and no such task gets NO connection_type at all, so the host's
    ``KinematicInterface`` never integrates it — including the ocean current
    it applies to every kinematic entity's position update. This task's only
    job is to request that connection type; it never publishes a velocity
    command, so the agent's commanded ``u``/``w`` stay at their default zero
    and it only ever moves as much as the current pushes it.
    """

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        host._kinematic_guidance = True

    def update(self) -> Status:
        return Status.RUNNING
