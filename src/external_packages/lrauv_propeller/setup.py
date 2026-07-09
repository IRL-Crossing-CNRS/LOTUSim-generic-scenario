from setuptools import find_packages, setup

package_name = "lrauv_propeller"

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
    description="External Agent: LrauvPropeller",
    license="EPL-2.0",
    entry_points={
        "lotusim.agents": [
            "lrauv_propeller = lrauv_propeller:LrauvPropeller",
        ],
        "console_scripts": [],
    },
)
