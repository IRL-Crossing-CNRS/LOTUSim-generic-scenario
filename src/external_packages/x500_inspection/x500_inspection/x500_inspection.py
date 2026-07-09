"""
@file x500_inspection.py
@brief Defines the X500Inspection agent class for simulation.

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0
"""

from x500 import X500


class X500Inspection(X500):
    """X500 aerial inspection drone, driven by the scenario JSON mission tree.

    No ``add_task`` / no mission wiring happens here — the runner calls
    ``set_missions()`` after instantiation and the BT tick timer drives the tree.
    Corrosion/crack detection is the SDK ``fault_inspection`` task, referenced
    from the mission JSON (see doc/MISSIONS.md).

    Always bypasses aerialWorld physics (``domains=[]``) — waypoint following,
    once migrated to a BT task, drives the Gazebo model directly.
    """

    def __init__(self, sdf_string: str, world_name: str, xdyn_enabled: bool, **kwargs):
        super().__init__(sdf_string, world_name, xdyn_enabled)
        self.renderer_type_name = "x500_inspection"
        self.domains = []  # bypass aerialWorld physics
