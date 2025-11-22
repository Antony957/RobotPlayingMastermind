import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # 1. Get Package Directories
    pkg_share = get_package_share_directory('mastermind')
    pkg_kortex_bringup = get_package_share_directory('kortex_bringup')

    # 2. Define World Path
    default_world = os.path.join(pkg_share, 'world', 'mastermind.sdf')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo world file'
    )

    # 3. Setup Resource Paths (World + Models)
    # This ensures Gazebo can find "model://gameboard", "model://cubes", etc.
    world_dir  = os.path.join(pkg_share, 'world')
    models_dir = os.path.join(pkg_share, 'models')

    env_actions = []

    # Append internal resource paths to Gazebo environment variables
    for var in ["GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"]:
        existing = os.environ.get(var, "")
        parts = [world_dir, models_dir]
        if existing:
            parts.append(existing)
        combined = ":".join(parts)
        env_actions.append(
            SetEnvironmentVariable(name=var, value=combined)
        )

    # 4. Launch Robot & Simulator (Kortex Bringup)
    # We use the Kortex launch file to handle the robot controllers and Gazebo startup.
    # We pass our custom world path to it.
    kortex_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_kortex_bringup, 'launch', 'kortex_sim_control.launch.py')
        ),
        launch_arguments={
            'robot_type': 'gen3_lite',
            'gripper': 'gen3_lite_2f',
            'robot_name': 'gen3_lite',
            'dof': '6',
            'use_sim_time': 'true',
            'launch_rviz': 'false', # We launch our own configured RViz below
            'robot_controller': 'joint_trajectory_controller',
            'sim_gazebo': 'true',
            'world': LaunchConfiguration('world') # Pass our world here
        }.items(),
    )

    # 5. ROS <-> Gazebo Bridges (Camera & PointCloud)
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
            # Remap Gazebo topics to custom ROS topics
            ('/realsense/image/image',       '/mastermind/realsense/image'),
            ('/realsense/image/depth_image', '/mastermind/realsense/depth_image'),
            ('/realsense/image/camera_info', '/mastermind/realsense/camera_info'),
            ('/realsense/image/points',      '/mastermind/realsense/points'),
        ],
        output='screen',
    )

    # 6. Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'launch', 'mastermind.rviz')],
    )

    # 7. Static Transform Publisher
    # Connects the camera link to the world frame so data appears correctly in RViz
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'realsense_d435/link'],
        output='screen',
    )

    return LaunchDescription(
        [world_arg] + env_actions + [kortex_sim, camera_bridge, rviz, tf_publisher]
    )