# Prerequisites:
#   pip install -e <DroneXavier>/workspace/src/optitrack
# Build:
#   colcon build --packages-select optitrack_ros

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("optitrack_ros"), "config", "natnet_pose.yaml"])
    return LaunchDescription(
        [
            Node(
                package="optitrack_ros",
                executable="natnet_pose_node",
                output="screen",
                emulate_tty=True,
                parameters=[params],
            ),
        ]
    )
