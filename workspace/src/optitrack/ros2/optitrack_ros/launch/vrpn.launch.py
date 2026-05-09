# Prerequisites:
#   Install ROS 2 Foxy VRPN client (example fork): https://github.com/NOKOV-MOCAP/vrpn_client_ros/tree/foxy-devel
# Tune config/vrpn_motive.yaml (`server`, `port`) to match Motive VRPN streaming on your LAN.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    motive_params = PathJoinSubstitution(
        [FindPackageShare("optitrack_ros"), "config", "vrpn_motive.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="vrpn_client_ros",
                executable="vrpn_client_node",
                name="vrpn_client_node",
                output="screen",
                emulate_tty=True,
                parameters=[motive_params],
            ),
        ]
    )
