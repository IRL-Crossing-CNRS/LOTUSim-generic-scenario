from setuptools import find_packages, setup

package_name = "bluerov_gnc"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="juliette",
    maintainer_email="jgrosset10@gmail.com",
    description="BlueROV2 under fixed-gain PID in layered ocean currents",
    license="EPL-2.0",
    entry_points={
        "lotusim.agents": [
            "bluerov2_heavy_pid = bluerov_gnc:Bluerov2HeavyPid",
        ],
        "lotusim.tasks": [
            "bluerov_navigation = bluerov_gnc:BlueRovNavigationTask",
            "bluerov_guidance_hold = bluerov_gnc:BlueRovHoldGuidanceTask",
            "bluerov_guidance_los = bluerov_gnc:BlueRovLOSGuidanceTask",
            "bluerov_guidance_pure_pursuit = bluerov_gnc:BlueRovPurePursuitGuidanceTask",
            "bluerov_guidance_los_polyline = bluerov_gnc:BlueRovLOSPolylineGuidanceTask",
            "bluerov_guidance_pure_pursuit_polyline = bluerov_gnc:BlueRovPurePursuitPolylineGuidanceTask",
            "bluerov_control = bluerov_gnc:BlueRovControlTask",
            "bluerov_allocation = bluerov_gnc:BlueRovAllocationTask",
            "bluerov_metrics_recorder = bluerov_gnc:BlueRovMetricsRecorderTask",
        ],
        "console_scripts": [],
    },
)
