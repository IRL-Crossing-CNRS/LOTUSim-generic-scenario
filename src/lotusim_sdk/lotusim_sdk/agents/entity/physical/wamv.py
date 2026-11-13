from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Wamv(PhysicalEntity):
    MODEL_NAME = "wamv"
    XDYN_PORT = 12348
    THRUSTERS = []
    DOMAINS = ["Surface"]
    # The two `maneuvering` thrusters of wamv.yaml, each commanded by a thrust
    # `T` in newtons. Seeded at zero so the first xdyn step already carries
    # both keys, and so a scenario with no Allocation task (drift only) keeps
    # them at zero for the whole run.
    INITIAL_COMMANDS = {"wamv_prop_left(T)": 0.0, "wamv_prop_right(T)": 0.0}
