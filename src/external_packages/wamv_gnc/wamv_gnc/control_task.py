"""WAMV Control: the vehicle-agnostic ControlTask. WAMV's damping
coefficients have not been characterized, so the current feedforward stays
off (zero) by default -- override ``xu``/``xuu``/``yv``/``yvv`` per
scenario once they are.

No xdyn Allocation task exists for WAMV: its xdyn model
(``assets/models/wamv/wamv.yaml``) is hull-only -- no thruster/maneuvering
force model at all, so there is no propulsion geometry to write an
allocator against. WAMV runs through the Kinematic path instead: this
``WamvControlTask`` plus ``lotusim_sdk.tasks.kinematic_allocation``
(``kinematic_allocation`` task name), the same as any other Kinematic
vehicle. If a thruster model is added to ``wamv.yaml``, a
``WamvAllocationTask`` can be added here following the
``dtmb_hull_gnc``/``lrauv_gnc`` pattern -- Navigation/Guidance/Control need
no changes either way.
"""

from __future__ import annotations

from lotusim_sdk.tasks.control import ControlTask


class WamvControlTask(ControlTask):
    """See ``ControlTask`` for params. No damping defaults yet for WAMV, so
    feedforward is off unless a scenario supplies xu/xuu/yv/yvv explicitly.
    """
