"""Dtmb_hull Control: the vehicle-agnostic ControlTask. No measured damping
coefficients on hand yet, so the current feedforward stays off (zero) by
default -- override ``xu``/``xuu``/``yv``/``yvv`` per scenario if/when
they're characterized.

A real xdyn Allocation task for dtmb_hull (mapping wrench demand to its
twin propeller+rudder actuators) exists but is not yet validated: gains
were never rescaled for this vessel's full-scale mass, and produced
diverging trajectories. Use the Kinematic path
(``dtmb_hull_control`` + ``lotusim_sdk.tasks.kinematic_allocation``) --
see ``multi_vehicle_examples/dtmb_hull_kinematic.json``.
"""

from __future__ import annotations

from lotusim_sdk.tasks.control import ControlTask


class DtmbHullControlTask(ControlTask):
    """See ``ControlTask`` for params. No damping defaults yet for
    dtmb_hull, so feedforward is off unless a scenario supplies
    xu/xuu/yv/yvv explicitly.
    """
