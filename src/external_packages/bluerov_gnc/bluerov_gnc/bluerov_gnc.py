"""BlueROV2 Heavy agent for the ocean-current xdyn model.

Differs from :class:`lotusim_sdk.agents.entity.physical.bluerov2_heavy.Bluerov2Heavy`
on two points, both dictated by the xdyn model it uses
(``assets/models/bluerov2_heavy/BlueROV2_current_ekman.yml``):

**THRUSTERS = []** -- ``PhysicalEntity._lotus_blocks()`` turns every name in
``THRUSTERS`` into an SDF ``<thrusterN>`` tag, which the plugin expands into
``<name>(rpm)`` / ``(P/D)`` / ``(beta)`` keys. That is the Wageningen B-series
convention of ``BlueROV2.yml``. This model uses `maneuvering` force models with a
single command ``T`` in newtons: those keys do not exist, and sending them would
fail every step. The list is therefore left empty, and the Allocation task
publishes all six commands as soon as it starts. (The plugin also accepts an
``<initial_commands>`` block for the same purpose, which is the cleaner route
once the SDF is generated with it.)

**DOMAINS = ["Surface", "Underwater"]** -- ``XdynWebsocket::getNewState()``
picks the domain from the immersion: deeper than ``<surface_depth>`` (10 m by
default) it is Underwater, shallower it is Surface. The three immersions used
here are 3, 25 and 55 m, so the vehicle crosses the boundary. Declaring both
domains avoids a ``Failed transition`` on every step.
"""

from __future__ import annotations

from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Bluerov2HeavyCurrent(PhysicalEntity):
    MODEL_NAME = "bluerov2_heavy"
    XDYN_PORT = 12347
    THRUSTERS = []  # see the docstring
    DOMAINS = ["Surface", "Underwater"]  # see the docstring
    # The six `maneuvering` force models of BlueROV2_current_*.yml, each
    # declaring a single command `T` in newtons. Seeded at zero so the very
    # first Gazebo step already carries every key xdyn expects, before the
    # Control task has published anything (props 6 and 7 are not in the model).
    INITIAL_COMMANDS = {f"bluerov2_heavy_prop_{i}(T)": 0.0 for i in (1, 2, 3, 4, 5, 8)}
