from setuptools import find_packages, setup

package_name = "custom_task_demo"

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
    maintainer="Naval Group",
    maintainer_email="lotusim@naval-group.com",
    description="Example external package registering a custom BT task (no lotusim_sdk edit required)",
    license="EPL-2.0",
    entry_points={
        "lotusim.agents": [
            "CustomTaskDemoAgent = custom_task_demo.agent:CustomTaskDemoAgent",
        ],
        "lotusim.tasks": [
            "blink_light = custom_task_demo.agent:BlinkLightTask",
        ],
        "console_scripts": [],
    },
)
