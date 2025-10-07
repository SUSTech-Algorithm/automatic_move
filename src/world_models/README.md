# Simple Robot Model for Gazebo

This package provides a simple robot model for Gazebo simulation with ROS2 integration.

## Robot Description

- Size: 0.7m x 0.7m x 0.5m (width x length x height)
- Color: Blue
- Sensors: Four single-point laser scanners facing front, rear, left, and right
- Laser height: 0.3cm above ground

## Launching the Robot

To launch the robot in the competition field with your existing launch file:

```bash
colcon build --packages-select world_models
source install/setup.bash
ros2 launch world_models gazebo_launch.py
```

The robot will be automatically spawned in Gazebo at the origin (0,0,0) with no rotation when you launch the simulation.

### Setting Initial Robot Position

You can specify the initial position and orientation of the robot using launch arguments:

```bash
ros2 launch world_models gazebo_launch.py robot_x:=1.0 robot_y:=2.0 robot_z:=0.5 robot_yaw:=1.57
```

Available arguments:
- `robot_x`, `robot_y`, `robot_z`: Position coordinates (default: 0.0)
- `robot_roll`, `robot_pitch`, `robot_yaw`: Orientation in radians (default: 0.0)

## Controlling the Robot

The robot can be controlled using the `/simple_robot/cmd_vel` topic with `geometry_msgs/Twist` messages.

To move the robot forward at 0.5 m/s:
```bash
ros2 topic pub /simple_robot/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1
```

## Laser Scanners

The robot has four single-point laser scanners:
- `/simple_robot/front_scan` - Front laser
- `/simple_robot/rear_scan` - Rear laser
- `/simple_robot/left_scan` - Left laser
- `/simple_robot/right_scan` - Right laser

## Files

- `models/simple_robot/robot.urdf` - Robot URDF model
- `models/simple_robot/model.config` - Model configuration file
- `src/test_robot_movement.py` - Simple test script for robot movement