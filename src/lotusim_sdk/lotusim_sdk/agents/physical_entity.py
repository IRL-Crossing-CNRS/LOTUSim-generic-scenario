from __future__ import annotations

import json

from lotusim_sdk.agents.entity import Entity


class PhysicalEntity(Entity):
    """
    Abstract base for agents that have a physical SDF model AND a physics engine (XDyn).

    Leaf classes declare class-level constants; PhysicalEntity.__init__ reads them and
    wires up XDyn if enabled. Behaviour comes entirely from the behaviour-tree mission
    engine (``set_missions`` / :class:`~lotusim_sdk.tasks.base.TaskAgent`), which lives
    in the base :class:`~lotusim_sdk.agents.agent.Agent`.
    """

    MODEL_NAME: str = ""
    XDYN_PORT: int | None = None
    THRUSTERS: list = []
    DOMAINS: list = []
    # Command keys to seed the physics engine with, as {name: value}. xdyn fails
    # a whole step if any command its force models declare is missing, and the
    # very first step happens before any agent node can publish — so a model
    # whose commands are not the Wageningen rpm/(P/D)/beta triplet implied by
    # THRUSTERS must name them here instead, e.g.
    # {"bluerov2_heavy_prop_1(T)": 0.0, ...} for `maneuvering` models commanded
    # in newtons. Mutually exclusive with THRUSTERS in practice; if both are
    # set, the host prefers this one.
    INITIAL_COMMANDS: dict = {}

    def __init__(self, sdf_string: str, world_name: str, xdyn_enabled: bool):
        self.model_name = self.MODEL_NAME
        self.renderer_type_name = self.MODEL_NAME
        self.domains = list(self.DOMAINS)
        self.thrusters = list(self.THRUSTERS)
        self.initial_commands = dict(self.INITIAL_COMMANDS)
        if xdyn_enabled and self.XDYN_PORT is not None:
            self.xdyn_port = self.XDYN_PORT
            self.xdyn_ip = "127.0.0.1"
        else:
            self.xdyn_port = None
            self.xdyn_ip = None
        super().__init__(sdf_string, world_name, self.xdyn_port)

    def _lotus_blocks(self) -> str:
        base = super()._lotus_blocks()

        # A WaypointFollowerTask sets this at construction (before spawn) to ask
        # the host to integrate motion kinematically from the velocity set-point
        # the agent publishes on /<world>/vessel_cmd_array.
        kinematic = getattr(self, "_kinematic_guidance", False)

        if self.domains:
            block = "\n  <physics_engine_interface>"
            for domain in self.domains:
                d = domain.lower()
                block += f"\n    <{d}>"
                if kinematic:
                    # Remote-driven kinematic motion: the host KinematicInterface
                    # integrates the velocity set-point published by the agent's
                    # WaypointFollowerTask using Gazebo's own time step. Takes
                    # priority over the Aerial/XDyn paths below so an aerial agent
                    # driven by WaypointFollowerTask uses the kinematic path too.
                    block += """
                    <connection_type>Kinematic</connection_type>
                """
                elif domain == "Aerial":
                    # The host's ROS2 aerial interface mirrors this entity's pose
                    # from the aerial world's pose topic. The namespace is the aerial
                    # world's <world> name, always "aerialWorld" — hardcoded here and
                    # in the custom world's AerialEntityManager (<aerial_namespace>),
                    # a mismatch just makes the aerial MAS "not available".
                    block += """
                    <connection_type>ROS2</connection_type>
                    <namespace>aerialWorld</namespace>
                """
                elif self.xdyn_ip and self.xdyn_port:
                    block += f"""
                    <connection_type>XDynWebSocket</connection_type>
                    <uri>ws://{self.xdyn_ip}:{self.xdyn_port}</uri>
                """
                    # Emit at most one of the two command-seeding tags, and
                    # only when it has content: an EMPTY <thrusters/> is not
                    # the same as no <thrusters> to the host, which treats the
                    # tag's presence as "this is a Wageningen model" and then
                    # walks its (nonexistent) children.
                    if self.initial_commands:
                        block += f"""
                    <initial_commands>{json.dumps(self.initial_commands)}</initial_commands>
                """
                    elif self.thrusters:
                        thruster_xml = "".join(
                            f"\n        <thruster{i}>{t}</thruster{i}>"
                            for i, t in enumerate(self.thrusters, 1)
                        )
                        block += f"""
                    <thrusters>{thruster_xml}
                    </thrusters>
                """
                    # Optional uniform Gauss-Markov current, injected by the
                    # host's XdynWebsocket rather than by the vessel's own
                    # hydrodynamic YAML. Set from the scenario JSON's
                    # per-agent "gauss_markov_current" block; absent by
                    # default, in which case the current is whatever the YAML
                    # declares.
                    gm = getattr(self, "gauss_markov_current", None)
                    if gm:
                        gm_xml = "".join(
                            f"\n        <{k}>{v}</{k}>" for k, v in gm.items()
                        )
                        block += f"""
                    <gauss_markov_current>{gm_xml}
                    </gauss_markov_current>
                """
                    # Optional replay of a measured current profile, the same
                    # host-side injection slot as the Gauss-Markov current
                    # above (a scenario sets one or the other, never both).
                    # Set from the scenario JSON's per-agent
                    # "copernicus_current" block, whose "profile" key is the
                    # depth-profile CSV to replay.
                    cop = getattr(self, "copernicus_current", None)
                    if cop:
                        cop_xml = "".join(
                            f"\n        <{k}>{v}</{k}>" for k, v in cop.items()
                        )
                        block += f"""
                    <copernicus_current>{cop_xml}
                    </copernicus_current>
                """
                block += f"\n    </{d}>"
            block += f"\n    <init_state>{self.domains[0]}</init_state>"
            block += "\n  </physics_engine_interface>"
            base = base + block

        return base
