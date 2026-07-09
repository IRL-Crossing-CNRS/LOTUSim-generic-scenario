from __future__ import annotations

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

    def __init__(self, sdf_string: str, world_name: str, xdyn_enabled: bool):
        self.model_name = self.MODEL_NAME
        self.renderer_type_name = self.MODEL_NAME
        self.domains = list(self.DOMAINS)
        self.thrusters = list(self.THRUSTERS)
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
                if domain == "Aerial":
                    block += """
                    <connection_type>ROS2</connection_type>
                    <namespace>aerialWorld</namespace>
                """
                elif self.xdyn_ip and self.xdyn_port:
                    thruster_xml = "".join(
                        f"\n        <thruster{i}>{t}</thruster{i}>"
                        for i, t in enumerate(self.thrusters, 1)
                    )
                    block += f"""
                    <connection_type>XDynWebSocket</connection_type>
                    <uri>ws://{self.xdyn_ip}:{self.xdyn_port}</uri>
                    <thrusters>{thruster_xml}
                    </thrusters>
                """
                elif kinematic:
                    # Remote-driven kinematic motion: the host KinematicInterface
                    # integrates the velocity set-point published by the agent's
                    # WaypointFollowerTask using Gazebo's own time step.
                    block += """
                    <connection_type>Kinematic</connection_type>
                """
                block += f"\n    </{d}>"
            block += f"\n    <init_state>{self.domains[0]}</init_state>"
            block += "\n  </physics_engine_interface>"
            base = base + block

        return base
