"""
@file wamv_inspection.py
@brief Defines the WamvInspection agent class for simulation.

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0
"""

from wamv import Wamv


class WamvInspection(Wamv):
    """WAM-V inspection agent, driven by the scenario JSON mission tree.

    No ``add_task`` / no mission wiring happens here — the runner calls
    ``set_missions()`` after instantiation and the BT tick timer drives the tree.
    Corrosion/crack detection is the SDK ``fault_inspection`` task, referenced
    from the mission JSON (see doc/MISSIONS.md).
    """

    def __init__(self, sdf_string: str, world_name: str, xdyn_enabled: bool, **kwargs):
        super().__init__(sdf_string, world_name, xdyn_enabled)
        self.renderer_type_name = "wamv_inspection"
