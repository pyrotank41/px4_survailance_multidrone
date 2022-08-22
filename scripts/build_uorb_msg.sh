#!/bin/bash

WS_DIR=$(cd ../ && pwd)
echo "deleting file from PX4 autopilot repo"
rm $WS_DIR/PX4-Autopilot/msg/tools/urtps_bridge_topics.yaml
echo "current files"
ls $WS_DIR/PX4-Autopilot/msg/tools/
cp $WS_DIR/scripts/config_files/urtps_bridge_topics.yaml $WS_DIR/PX4-Autopilot/msg/tools/
cd $WS_DIR/PX4-Autopilot/msg/tools/

python3 uorb_to_ros_urtps_topics.py --input-file ./urtps_bridge_topics.yaml --output-file $WS_DIR/ros2_ws/src/px4_ros_com/templates/urtps_bridge_topics.yaml
cd ../../
make px4_sitl_rtps
cd $WS_DIR/ros2_ws
rm -r build/px4_ros_com
rm -r install/px4_ros_com
colcon build --packages-select px4_ros_com
