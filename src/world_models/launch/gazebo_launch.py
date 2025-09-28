import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 获取你的模型的路径
    pkg_world_models = get_package_share_directory('world_models')
    world_file_path = os.path.join(
        pkg_world_models, 'competition_field.world')

    # 声明 launch 参数
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Full path to the world model file to load')

    # 声明机器人初始位置参数
    robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='-1.5',
        description='Initial x position of the robot')

    robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='5.5',
        description='Initial y position of the robot')

    robot_z_arg = DeclareLaunchArgument(
        'robot_z',
        default_value='0.4',
        description='Initial z position of the robot')

    robot_roll_arg = DeclareLaunchArgument(
        'robot_roll',
        default_value='0.0',
        description='Initial roll orientation of the robot')

    robot_pitch_arg = DeclareLaunchArgument(
        'robot_pitch',
        default_value='0.0',
        description='Initial pitch orientation of the robot')

    robot_yaw_arg = DeclareLaunchArgument(
        'robot_yaw',
        default_value='0.0',
        description='Initial yaw orientation of the robot')

    # 启动 Gazebo 仿真器，并传递世界文件
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 启动 Gazebo 客户端（GUI）
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 添加机器人模型到Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_robot',
            '-file', os.path.join(pkg_world_models, 'simple_robot', 'robot.urdf'),
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
            '-R', LaunchConfiguration('robot_roll'),
            '-P', LaunchConfiguration('robot_pitch'),
            '-Y', LaunchConfiguration('robot_yaw')
        ],
        output='screen'
    )

    return LaunchDescription([
        world_file_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        robot_roll_arg,
        robot_pitch_arg,
        robot_yaw_arg,
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot
    ])