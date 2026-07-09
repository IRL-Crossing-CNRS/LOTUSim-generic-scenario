from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Pha(PhysicalEntity):
    MODEL_NAME = "pha"
    XDYN_PORT = 12351
    THRUSTERS = [""]
    DOMAINS = ["Surface"]
