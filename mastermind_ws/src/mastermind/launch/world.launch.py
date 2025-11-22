# mastermind/launch/world.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # 直接用 ament 的接口拿绝对路径
    pkg_share = get_package_share_directory('mastermind')
    kortex_share = get_package_share_directory('kortex_description')
    kortex_resource_root = os.path.dirname(kortex_share)



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

    # 把 kortex_resource_root 也加进去
    resource_paths = [world_dir, models_dir, kortex_resource_root]

    for var in ["GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"]:
        existing = os.environ.get(var, "")
        parts = resource_paths.copy()
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


    # kortex_description 包路径

    # 你给的这个 xacro：gen3_lite_gen3_lite_2f.xacro
    gen3_xacro_path = os.path.join(
        kortex_share,
        'robots',
        'gen3_lite_gen3_lite_2f.xacro'
    )

    # 调用 xacro 生成 URDF 文本，放到 robot_description 里
    robot_description = ParameterValue(
        Command(['xacro ', gen3_xacro_path]),
        value_type=str
    )

    # 把 robot_description 发布出去（供 RViz / 其他节点用）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gen3_lite_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    spawn_gen3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'gen3_lite',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.75',
        ],
        output='screen',
    )

    # 给世界坐标和机械臂 base_link 一个静态 TF
    # 如果 URDF 里的根 link 不是 base_link，就把最后那个改成对应名字
    gen3_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
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


    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'realsense_d435/link'],
        output='screen',
    )

    return LaunchDescription(
        [world_arg] + env_actions + [start_gz, robot_state_publisher, spawn_gen3, gen3_tf, camera_bridge, rviz, tf_publisher]
    )