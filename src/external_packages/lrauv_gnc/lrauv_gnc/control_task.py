"""LRAUV Control: the vehicle-agnostic ControlTask, LRAUV has no measured
horizontal damping coefficients on hand yet, so the current feedforward
stays off (zero) by default -- override ``xu``/``xuu``/``yv``/``yvv`` per
scenario if/when they're characterized.

A real xdyn Allocation task for LRAUV (mapping surge force demand to its
propeller) exists but is not yet validated: the active propeller model
was a generic placeholder and produced diverging trajectories, now fixed
but not re-tested. Use the Kinematic path
(``lrauv_control`` + ``lotusim_sdk.tasks.kinematic_allocation``) --
see ``multi_vehicle_examples/lrauv_kinematic.json``.
"""

from __future__ import annotations

from lotusim_sdk.tasks.control import ControlTask


class LrauvControlTask(ControlTask):
    """See ``ControlTask`` for params. No damping defaults yet for LRAUV, so
    feedforward is off unless a scenario supplies xu/xuu/yv/yvv explicitly.
    """
