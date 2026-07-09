from setuptools import find_packages, setup

package_name = "lotusim_client"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Naval Group",
    maintainer_email="lotusim@naval-group.com",
    description="LOTUSim client tools — run and manage agents from any ROS 2 machine",
    license="EPL-2.0",
    entry_points={
        "console_scripts": [
            "run_agent = lotusim_client.run_agent:main",
        ],
    },
)
