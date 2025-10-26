# 自主移动机器人项目

本项目是一个基于 ROS2 的自主移动机器人系统，使用 FAST-LIO 进行定位和建图，并包含完整的状态管理系统。

## 子模块

本仓库包含以下子模块：

- [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2): 轻量级、快速的激光雷达惯性里程计框架
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2): Livox 激光雷达官方 SDK
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2): Livox 激光雷达的 ROS2 驱动
- [livox_gazebo_ros2_gpu_simulation](https://github.com/LCAS/livox_laser_simulation_ros2): 用于在 Gazebo 中模拟 Livox 激光雷达的 ROS2 包

要克隆包含所有子模块的仓库，请使用以下命令：

```bash
git clone --recurse-submodules https://github.com/SUSTech-Algorithm/automatic_move.git
```

如果您已经克隆了仓库，可以使用以下命令初始化并更新子模块：

```bash
git submodule update --init --recursive
```

## Gazebo 仿真

要使用 Gazebo 仿真，您需要将模型路径添加到环境变量中。将以下行添加到您的 `.bashrc` 文件中：

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/automatic_move/src/world_models/models
```

## ROS2 机器人状态管理系统

本项目的核心是一个完全兼容 ROS2 的机器人状态管理系统，采用状态设计模式，便于扩展和管理不同的机器人行为状态。

### 主要组件

1. **状态基类** (`state.py`): 定义所有机器人状态的接口
2. **具体状态类** (`states.py`): 实现具体的机器人行为（空闲、移动、停止、避障、导航）
3. **状态管理节点** (`state_manager.py`): ROS2 节点，管理状态转换并执行状态逻辑

### 支持的状态

- **IdleState**: 空闲状态，机器人等待命令
- **MovingState**: 移动状态，机器人执行基本移动
- **StoppedState**: 停止状态，机器人完全停止
- **AvoidingObstacleState**: 避障状态，机器人执行避障行为
- **NavigationState**: 导航状态，机器人执行复杂导航任务

### ROS2 话题

- **发布话题**:
  - `/cmd_vel` (geometry_msgs/Twist): 机器人速度命令
  - `/robot_state` (std_msgs/String): 当前机器人状态

- **订阅话题**:
  - `/robot_command` (std_msgs/String): 外部命令输入

### 使用方法

#### 启动状态管理系统

使用 launch 文件启动：

```bash
ros2 launch automove_engine state_manager_launch.py
```

或直接运行节点：

```bash
ros2 run automove_engine state_manager
```

#### 发送命令控制机器人

```bash
# 开始移动
ros2 topic pub /robot_command std_msgs/String "data: 'move'"

# 停止机器人
ros2 topic pub /robot_command std_msgs/String "data: 'stop'"

# 开始导航
ros2 topic pub /robot_command std_msgs/String "data: 'navigate'"

# 触发避障
ros2 topic pub /robot_command std_msgs/String "data: 'obstacle'"

# 返回空闲状态
ros2 topic pub /robot_command std_msgs/String "data: 'idle'"
```

#### 监控机器人状态

```bash
ros2 topic echo /robot_state
```

#### 运行测试

```bash
ros2 run automove_engine state_test
```

### 添加新状态

要添加新的机器人状态：

1. 在 `states.py` 中创建继承自 `State` 的新类
2. 实现 `enter()`、`execute()`、`exit()` 和 `handle_event()` 方法
3. 在其他状态的 `handle_event()` 方法中添加到新状态的转换逻辑
4. 根据需要在状态转换中使用新状态

### 示例：添加充电状态

```python
class ChargingState(State):
    """机器人充电状态。"""
    
    def __init__(self, context):
        super().__init__(context)
        self.charge_time = 0
        
    def enter(self):
        logger = self.context.get('logger')
        if logger:
            logger.info("进入充电状态")
        # 停止机器人移动
        cmd_vel_pub = self.context.get('cmd_vel_pub')
        if cmd_vel_pub:
            stop_msg = Twist()
            cmd_vel_pub.publish(stop_msg)
    
    def execute(self):
        self.charge_time += 1
        if self.charge_time >= 100:  # 充电完成
            return IdleState(self.context)
        return None
    
    def handle_event(self, event):
        if event == "charge_complete":
            return IdleState(self.context)
        return None
```

## FAST-LIO 安装

要安装 FAST-LIO，请按照以下步骤操作：

```bash
cd <ros2_ws>/src
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source ./install/setup.bash
```

**注意：** 在构建之前，请记得 source `livox_ros_driver` 的工作空间。

## 构建说明

要构建整个项目，请使用以下命令：

```bash
# 在工作空间根目录下
colcon build --symlink-install

# 或者使用提供的构建脚本
./colcon_build.sh

# source 环境
source install/setup.bash
```

## 相关资源

### Livox MID-360 激光雷达

- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [livox_laser_simulation_ros2](https://github.com/LCAS/livox_laser_simulation_ros2)

## 故障排除

如果在构建过程中遇到问题，请确保：

1. 已正确安装所有 ROS2 依赖项
2. 已 source 必要的环境设置
3. 子模块已正确初始化和更新
4. Livox 驱动已正确安装和配置