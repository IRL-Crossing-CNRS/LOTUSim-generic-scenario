from setuptools import find_packages, setup

package_name = "lotusim_sdk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["build*", "test*"]),
    # Ship the detection module + model weights inside the wheel so the
    # fault_inspection task can import them on host or remote machines alike.
    package_data={
        "lotusim_sdk.tasks.fault_inspection_assets": ["*.py", "*.pt"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        # Detection dependencies for the fault_inspection task.
        # In install_requires so a plain `pip install lotusim_sdk` pulls them
        # automatically — including the deployment bundle's base install.
        "ultralytics",
        "torch",
        "opencv-python",
        "pillow",
        "numpy",
    ],
    zip_safe=True,
    maintainer="Naval Group",
    maintainer_email="lotusim@naval-group.com",
    description="LOTUSim SDK — agent base classes and behaviour-tree mission engine for external agent development",
    license="EPL-2.0",
    entry_points={
        "console_scripts": [],
        "lotusim.tasks": [
            "fault_inspection = lotusim_sdk.tasks.fault_inspection:FaultInspectionTask",
            "check_battery_state = lotusim_sdk.tasks.check_battery_state:CheckBatteryStateTask",
            "waypoint_follower = lotusim_sdk.tasks.waypoint_follower:WaypointFollowerTask",
        ],
        "lotusim.agents": [
            "bluerov2_heavy = lotusim_sdk.agents.entity.physical.bluerov2_heavy:Bluerov2Heavy",
            "commando       = lotusim_sdk.agents.entity.physical.commando:Commando",
            "dtmb_hull      = lotusim_sdk.agents.entity.physical.dtmb_hull:DtmbHull",
            "fremm          = lotusim_sdk.agents.entity.physical.fremm:Fremm",
            "lrauv          = lotusim_sdk.agents.entity.physical.lrauv:Lrauv",
            "mine           = lotusim_sdk.agents.entity.physical.mine:Mine",
            "pha            = lotusim_sdk.agents.entity.physical.pha:Pha",
            "wamv           = lotusim_sdk.agents.entity.physical.wamv:Wamv",
            "x500           = lotusim_sdk.agents.entity.physical.x500:X500",
            "Wind           = lotusim_sdk.agents.environment.wind.wind:Wind",
        ],
    },
)
