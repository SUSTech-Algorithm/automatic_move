cmake_minimum_required(VERSION 3.8)
project(world_models)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# install models and set GAZEBO_MODEL_PATH
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/test_robot_movement.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()