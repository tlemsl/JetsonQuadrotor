#!/bin/bash
source ~/px4_ros_dds_ws/install/setup.bash &&
MicroXRCEAgent serial --dev /dev/ttyTHS0 -b 921600
