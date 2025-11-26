import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('mastermind')
    kortex_pkg = FindPackageShare('kortex_bringup')
    kortex_description_pkg = get_package_share_directory('kortex_description')

    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(kortex_description_pkg)
    )

    kortex_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([kortex_pkg, 'launch', 'kortex_sim_control.launch.py'])
        ),
        launch_arguments={
            'sim_gazebo': 'true',
            'robot_type': 'gen3_lite',
            'gripper': 'gen3_lite_2f',
            'robot_name': 'gen3_lite',
            'dof': '6',
            'use_sim_time': 'true',
            'launch_rviz': 'false', 
            # 'launch_rviz': 'true', 
            'robot_controller': 'joint_trajectory_controller',
        }.items()
    )


    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            'mastermind/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            'mastermind/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'launch', 'mastermind.rviz')],
    )


    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.6',    # x
            '0.0',    # y
            '0.8',    # z
            '3.1415', # yaw 
            '0.9',    # pitch 
            '0.0',    # roll
            'world',  
            'camera_mount/camera_link/mastermind_camera' # frame_id
        ],
        output='screen',
    )

    return LaunchDescription([
        set_resource_path, 
        kortex_launch,
        camera_bridge, 
        rviz,    
        tf_publisher, 
    ])