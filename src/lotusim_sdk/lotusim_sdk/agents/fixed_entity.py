from lotusim_sdk.agents.entity import Entity


class FixedEntity(Entity):
    """
    Abstract base for agents that have a physical SDF model but no physics engine.

    Used for infrastructure placed in the simulation world (wind turbines, buoys, etc.).
    No locomotion, no XDyn — render block only in lotus_param.
    """

    MODEL_NAME: str = ""

    def __init__(self, sdf_string: str, world_name: str):
        self.model_name = self.MODEL_NAME
        self.renderer_type_name = self.MODEL_NAME
        self.domains = []
        self.thrusters = []
        self.xdyn_ip = None
        self.xdyn_port = None
        super().__init__(sdf_string, world_name, None)
