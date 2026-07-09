from lotusim_sdk.agents.physical_entity import PhysicalEntity


class X500(PhysicalEntity):
    MODEL_NAME = "x500"
    XDYN_PORT = None  # aerial agents never use XDyn
    THRUSTERS = []
    DOMAINS = ["Aerial"]
