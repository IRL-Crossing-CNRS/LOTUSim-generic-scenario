from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Fremm(PhysicalEntity):
    MODEL_NAME = "fremm"
    XDYN_PORT = 12349
    THRUSTERS = [""]
    DOMAINS = ["Surface"]
