from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Bluerov2Heavy(PhysicalEntity):
    MODEL_NAME = "bluerov2_heavy"
    XDYN_PORT = 12347
    THRUSTERS = ["thruster1", "thruster2", "thruster3", "thruster4",
                 "thruster5", "thruster6", "thruster7", "thruster8"]
    DOMAINS = ["Underwater"]
