#!/bin/bash
colcon build --symlink-install --packages-select  world_models automove_engine 
colcon build --packages-select livox_laser_simulation_ros2

source install/setup.bash