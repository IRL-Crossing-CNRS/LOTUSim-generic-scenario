from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Commando(PhysicalEntity):
    MODEL_NAME = "commando"
    XDYN_PORT = 12352
    THRUSTERS = [""]
    DOMAINS = ["Surface"]
