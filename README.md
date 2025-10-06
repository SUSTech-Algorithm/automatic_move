# automatic_move

## gazebo
好像突然可以将solidworks的模型导入了。替换模型的时候记得让solidworks导出.stl文件

感觉唯一需要做的就是：需要将模型路径手动导入$GAZEBO_MODEL_PATH **当然，前提是你把gazebo安装了**``` .bashrc
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/automatic_move/src/world_models/models
```

## Robot State Management System

本项目包含一个灵活的机器人行为状态管理系统。该系统采用状态设计模式，便于扩展新的状态和状态之间的切换。

### Components

1. **State Base Class** (`state.py`): Defines the interface for all robot states.
2. **Concrete States** (`states.py`): Implements specific robot behaviors (Idle, Moving, Stopped, Obstacle Avoidance).
3. **State Manager** (`state_manager.py`): Manages state transitions and executes state logic.

### Usage

To use the state management system:

1. Create a StateManager instance
2. Set the initial state
3. Call update() regularly to execute state logic
4. Call handle_event() to process external events that may trigger state transitions

Example:
```python
from automove_engine.state_manager import StateManager

# Create state manager
sm = StateManager()

# Set initial state
sm.set_initial_state()

# In main loop
while True:
    sm.update()
    # Handle events as needed
    # sm.handle_event(event)
```

### Adding New States

To add a new state:
1. Create a new class that inherits from State
2. Implement the enter(), execute(), exit(), and handle_event() methods
3. Import the new state in __init__.py
4. Use the state in transitions as needed


MID 360 

https://github.com/Livox-SDK/Livox-SDK2/tree/master
https://github.com/Livox-SDK/livox_ros_driver2?tab=readme-ov-file
https://github.com/LCAS/livox_laser_simulation_ros2

Fastlio 安装
```
    cd <ros2_ws>/src # cd into a ros2 workspace folder
    git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
    cd ..
    rosdep install --from-paths src --ignore-src -y
    colcon build --symlink-install
    . ./install/setup.bash # use setup.zsh if use zsh
```
**Remember to source the livox_ros_driver before build**
可能会出现报错