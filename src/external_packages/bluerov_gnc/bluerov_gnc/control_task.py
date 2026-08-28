"""BlueROV Control: the vehicle-agnostic ControlTask, with BlueROV2 Heavy's
own horizontal damping as the feedforward defaults.

All the control logic lives in ``lotusim_sdk.tasks.control.ControlTask``;
this subclass exists only to supply BlueROV2 Heavy's damping coefficients
(read off its xdyn YAML) as the feedforward defaults, so a scenario that
wants feedforward doesn't have to pass ``xu``/``xuu``/``yv``/``yvv`` itself.
"""

from __future__ import annotations

from lotusim_sdk.tasks.control import ControlTask


class BlueRovControlTask(ControlTask):
    """See ``ControlTask`` for params. Damping defaults: BlueROV2 Heavy
    horizontal damping, read off the vehicle's xdyn YAML (`linear damping` /
    `quadratic damping` rows 1 and 2). Defaults only: a scenario may
    override them, and must if the vehicle model changes.
    """

    DEFAULT_XU = 13.7
    DEFAULT_XUU = 141.0     # surge
    DEFAULT_YV = 0.0
    DEFAULT_YVV = 217.0     # sway (no linear term in the model)
