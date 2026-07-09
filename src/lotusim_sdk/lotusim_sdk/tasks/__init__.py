from lotusim_sdk.tasks.base import TaskAgent
from lotusim_sdk.tasks.fault_inspection import FaultInspectionTask
from lotusim_sdk.tasks.check_battery_state import CheckBatteryStateTask
from lotusim_sdk.tasks.waypoint_follower import WaypointFollowerTask

__all__ = [
    "TaskAgent",
    "FaultInspectionTask",
    "CheckBatteryStateTask",
    "WaypointFollowerTask",
]
