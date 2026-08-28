"""LRAUV Control: the vehicle-agnostic ControlTask. LRAUV's horizontal
damping coefficients have not been characterized, so the current
feedforward stays off (zero) by default -- override
``xu``/``xuu``/``yv``/``yvv`` per scenario once they are.

Both Kinematic and xdyn Allocation are available -- see
``multi_vehicle_examples/lrauv_kinematic.json`` and
``multi_vehicle_examples/lrauv_xdyn.json``.
"""

from __future__ import annotations

from lotusim_sdk.tasks.control import ControlTask


class LrauvControlTask(ControlTask):
    """See ``ControlTask`` for params. No damping defaults yet for LRAUV, so
    feedforward is off unless a scenario supplies xu/xuu/yv/yvv explicitly.
    """
