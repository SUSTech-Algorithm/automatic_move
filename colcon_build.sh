#!/bin/bash
colcon build --symlink-install --packages-select  world_models automove_engine

source install/setup.bash