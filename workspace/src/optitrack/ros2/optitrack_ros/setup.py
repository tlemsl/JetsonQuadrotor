import os
from glob import glob

from setuptools import setup

# Install Python dependency first:
#   pip install -e ../../../optitrack

package_name = "optitrack_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="DroneXavier",
    maintainer_email="none@example.com",
    description="ROS 2 Foxy helpers for OptiTrack NatNet + VRPN.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "natnet_pose_node = optitrack_ros.natnet_pose_node:main",
            "pose_relay_node = optitrack_ros.pose_relay_node:main",
        ],
    },
)
