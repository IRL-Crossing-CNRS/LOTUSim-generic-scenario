from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Wamv(PhysicalEntity):
    MODEL_NAME = "wamv"
    XDYN_PORT = 12348
    THRUSTERS = [""]
    DOMAINS = ["Surface"]
