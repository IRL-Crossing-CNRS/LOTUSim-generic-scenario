"""BlueROV alias for the vehicle-agnostic Navigation task.

Navigation has no BlueROV-specific behavior at all, so this is a plain
re-export kept for backward compatibility with the ``bluerov_navigation``
task-registry name -- see ``lotusim_sdk.tasks.navigation.NavigationTask``
for the implementation.
"""

from lotusim_sdk.tasks.navigation import NavigationTask as BlueRovNavigationTask

__all__ = ["BlueRovNavigationTask"]
