"""BlueROV aliases for the vehicle-agnostic Guidance tasks.

None of these have BlueROV-specific behavior, so they're plain re-exports
kept for backward compatibility with the ``bluerov_guidance_*`` task-registry
names -- see ``lotusim_sdk.tasks.guidance`` for the implementations.
"""

from lotusim_sdk.tasks.guidance import HoldGuidanceTask as BlueRovHoldGuidanceTask
from lotusim_sdk.tasks.guidance import LOSGuidanceTask as BlueRovLOSGuidanceTask
from lotusim_sdk.tasks.guidance import (
    PurePursuitGuidanceTask as BlueRovPurePursuitGuidanceTask,
)
from lotusim_sdk.tasks.guidance import (
    LOSPolylineGuidanceTask as BlueRovLOSPolylineGuidanceTask,
)
from lotusim_sdk.tasks.guidance import (
    PurePursuitPolylineGuidanceTask as BlueRovPurePursuitPolylineGuidanceTask,
)

__all__ = [
    "BlueRovHoldGuidanceTask",
    "BlueRovLOSGuidanceTask",
    "BlueRovPurePursuitGuidanceTask",
    "BlueRovLOSPolylineGuidanceTask",
    "BlueRovPurePursuitPolylineGuidanceTask",
]
