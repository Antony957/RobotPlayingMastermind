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

    # default world
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


    # 启动 gz sim
    start_gz = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',
            LaunchConfiguration('world'),
        ],
        output='screen',
    )

    # ROS <-> Gazebo Image & Camera Info Bridge
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/realsense/image/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/realsense/image/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/realsense/image/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/realsense/image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        remappings=[
            # change gazebo topic name to what I want in ROS2
            ('/realsense/image/image',       '/mastermind/realsense/image'),
            ('/realsense/image/depth_image', '/mastermind/realsense/depth_image'),
            ('/realsense/image/camera_info', '/mastermind/realsense/camera_info'),
            ('/realsense/image/points',      '/mastermind/realsense/points'),
        ],
        output='screen',
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'launch', 'mastermind.rviz')],
    )

    # Fake transform: Tells ROS that "realsense_d435/link" exists 
    # and is located at the origin of the map.
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'realsense_d435/link'],
        output='screen',
    )

    return LaunchDescription(
        [world_arg] + env_actions + [start_gz, camera_bridge, rviz, tf_publisher]
    )