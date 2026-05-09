from setuptools import find_packages, setup

setup(
    name="optitrack_receiver",
    version="0.1.0",
    description="Receive OptiTrack Motive LAN streams via NatNet UDP (decode-only).",
    python_requires=">=3.8",
    packages=find_packages(include=["optitrack_receiver*"]),
    entry_points={
        "console_scripts": [
            "optitrack-natnet-print=optitrack_receiver.cli_natnet:main",
        ],
    },
)
