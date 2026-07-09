from enum import Enum, auto


class Status(Enum):
    """The three-valued result returned by every :meth:`BehaviorNode.tick`.

    ``RUNNING`` is the keystone: it lets a node span several ticks, which is
    what makes a :class:`~lotusim_sdk.bt.composites.Sequence` wait for one
    child to finish before moving to the next.
    """

    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()
