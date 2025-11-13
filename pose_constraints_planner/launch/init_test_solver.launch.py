from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pose_constraints_planner_dir = get_package_share_directory('pose_constraints_planner')
    ur_driver_dir = get_package_share_directory('ur_robot_driver')
    ur_moveit_dir = get_package_share_directory('ur_moveit_config')

    # --- UR Robot Driver ---
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_driver_dir, 'launch', 'ur10e.launch.py')),
        launch_arguments={
            'use_fake_hardware': 'true',
            'robot_ip': '0.0.0.1',
            'activate_joint_controller': 'true'
        }.items()
    )

    # --- MoveIt for UR10e ---
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_dir, 'launch', 'ur_moveit.launch.py')),
        launch_arguments={
            'ur_type': 'ur10e',
            'use_fake_hardware': 'true',
            'launch_rviz': 'false'
        }.items()
    )

    # --- RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pose_constraints_planner_dir, 'config', 'rviz_config.rviz')]
    )

    # --- Add delays between launches ---
    return LaunchDescription([
        ur_driver,
        TimerAction(period=3.0, actions=[moveit]),
        TimerAction(period=5.0, actions=[rviz]),
    ])
