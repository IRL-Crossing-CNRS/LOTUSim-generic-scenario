from setuptools import find_packages, setup

package_name = "lrauv_gnc"

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
    description="LRAUV under the generic GNC pipeline: single-propeller allocation, no yaw actuation.",
    license="EPL-2.0",
    entry_points={
        "lotusim.tasks": [
            "lrauv_control = lrauv_gnc:LrauvControlTask",
            "lrauv_allocation = lrauv_gnc:LrauvAllocationTask",
        ],
        "console_scripts": [],
    },
)
