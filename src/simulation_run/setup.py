import os
from glob import glob

from setuptools import find_packages, setup

package_name = "simulation_run"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
    ],
    install_requires=["setuptools", "lotusim_sdk"],
    zip_safe=True,
    maintainer="juliette",
    maintainer_email="jgrosset10@gmail.com",
    description="Description de ton package",
    license="SPDX-License-Identifier: EPL-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = simulation_run.main:main",
            "clean_simulation = simulation_run.simulation_runner:stop_simulation",
            "spawn_agent = simulation_run.dynamic_spawn.spawn_agent:main",
            "despawn_agent = simulation_run.dynamic_spawn.despawn_agent:main",
            "list_agents = simulation_run.dynamic_spawn.list_agents:main",
        ],
    },
)
