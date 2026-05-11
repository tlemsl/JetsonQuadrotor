# Prerequisites:
#   pip install -e <DroneXavier>/workspace/src/optitrack
#   px4_msgs in the same colcon workspace (e.g. clone PX4 px4_msgs, or use your PX4 ROS DDS ws):
#     source ~/px4_ros_dds_ws/install/setup.bash
# Topics:
#   /optitrack/natnet/pose  (geometry_msgs/PoseStamped, raw NatNet)
#   /fmu/in/vehicle_visual_odometry  (px4_msgs/VehicleOdometry; ENU internally, NED on wire if px4_convert_enu_to_ned)
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
