from setuptools import find_packages, setup

package_name = "dtmb_hull_gnc"

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
    maintainer="Juliette Grosset",
    maintainer_email="jgrosset10@gmail.com",
    description="Dtmb_hull under the generic GNC pipeline (Kinematic backend; xdyn Allocation not yet validated).",
    license="EPL-2.0",
    entry_points={
        "lotusim.tasks": [
            "dtmb_hull_control = dtmb_hull_gnc:DtmbHullControlTask",
        ],
        "console_scripts": [],
    },
)
