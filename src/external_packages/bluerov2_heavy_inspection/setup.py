from setuptools import find_packages, setup

package_name = "bluerov2_heavy_inspection"

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
    description="External Bluerov2HeavyInspection agent with YOLO corrosion/crack detection",
    license="EPL-2.0",
    entry_points={
        "lotusim.agents": [
            "bluerov2_heavy_inspection = bluerov2_heavy_inspection:Bluerov2HeavyInspection",
        ],
        "console_scripts": [],
    },
)
