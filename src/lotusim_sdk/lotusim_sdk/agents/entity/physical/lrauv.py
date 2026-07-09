from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Lrauv(PhysicalEntity):
    MODEL_NAME = "lrauv"
    XDYN_PORT = 12346
    THRUSTERS = ["propeller"]
    DOMAINS = ["Underwater"]
