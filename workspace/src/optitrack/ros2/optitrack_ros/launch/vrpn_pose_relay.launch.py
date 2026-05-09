# Republish a VRPN pose topic under a stable name (optional helper).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/vrpn_client_node/replace_with_vrpn_tracker_name/pose",
                description="Source PoseStamped topic from vrpn_client_ros.",
            ),
            DeclareLaunchArgument("output_topic", default_value="/optitrack/vrpn/pose"),
            Node(
                package="optitrack_ros",
                executable="pose_relay_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                    }
                ],
            ),
        ]
    )
