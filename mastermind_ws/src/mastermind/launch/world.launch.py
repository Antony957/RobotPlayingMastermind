# mastermind/launch/world.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # 直接用 ament 的接口拿绝对路径
    pkg_share = get_package_share_directory('mastermind')

    # 默认 world
    default_world = os.path.join(pkg_share, 'world', 'mastermind.sdf')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo world file'
    )

    # 我们自己的资源目录：world + models
    world_dir  = os.path.join(pkg_share, 'world')
    models_dir = os.path.join(pkg_share, 'models')

    env_actions = []

    # 帮 Gazebo Sim 8 找到这些资源目录
    for var in ["GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"]:
        existing = os.environ.get(var, "")
        parts = [world_dir, models_dir]
        if existing:
            parts.append(existing)
        combined = ":".join(parts)
        env_actions.append(
            SetEnvironmentVariable(name=var, value=combined)
        )

    # （可选）兼容旧的 Gazebo Classic
    existing_gazebo_model = os.environ.get("GAZEBO_MODEL_PATH", "")
    parts = [models_dir]
    if existing_gazebo_model:
        parts.append(existing_gazebo_model)
    combined_gazebo = ":".join(parts)
    env_actions.append(
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=combined_gazebo)
    )

    # 启动 gz sim
    start_gz = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',
            LaunchConfiguration('world'),
        ],
        output='screen',
    )

    # ROS <-> Gazebo 图像 & 相机信息桥
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/mastermind/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mastermind/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    # RViz 显示
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        # 以后可以加自定义 rviz 配置：
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'mastermind_camera.rviz')],
    )

    return LaunchDescription(
        [world_arg] + env_actions + [start_gz, camera_bridge, rviz]
    )