"""Setup script for lego_hub_ros2 package."""

from setuptools import find_packages, setup
from glob import glob
import os

package_name = "lego_hub_ros2"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Simon Wong",
    maintainer_email="smw2@ualberta.ca",
    description="ROS 2 interface for LEGO Robot Inventor Hub via Bluetooth",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hub_node = lego_hub_ros2.hub_node:main",
        ],
    },
)
