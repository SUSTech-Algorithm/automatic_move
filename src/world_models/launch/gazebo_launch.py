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

    # 声明一个 launch 参数来指定世界文件
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Full path to the world model file to load')

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

    return LaunchDescription([
        world_file_arg,
        gzserver_cmd,
        gzclient_cmd
    ])