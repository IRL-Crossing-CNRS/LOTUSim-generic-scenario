from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Mine(PhysicalEntity):
    MODEL_NAME = "mine"
    XDYN_PORT = 12350
    THRUSTERS = [""]
    DOMAINS = ["Underwater"]
