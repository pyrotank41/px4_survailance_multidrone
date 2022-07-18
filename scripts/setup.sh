#!/bin/bash

# source px4 paths
cd PX4-Autopilot
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
cd ..

# source our paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/sim_ws/models



