from lotusim_sdk.agents.agent import Agent


class Environment(Agent):
    """
    Abstract base for agents that simulate environmental physics.

    No physical SDF model, no MAS spawn/delete, no pose tracking.
    Subclasses publish environmental data (wind, current, waves, etc.) to ROS topics.
    """

    def __init__(self, world_name: str):
        super().__init__(self.__class__.__name__.lower(), world_name)


__all__ = ["Environment"]
