from lotusim_sdk.agents.physical_entity import PhysicalEntity


class DtmbHull(PhysicalEntity):
    MODEL_NAME = "dtmb_hull"
    XDYN_PORT = 12345
    THRUSTERS = [""]
    DOMAINS = ["Surface"]
