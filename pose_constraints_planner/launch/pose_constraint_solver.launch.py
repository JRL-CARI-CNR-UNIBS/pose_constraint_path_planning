from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
import os

def generate_launch_description():
    pose_constraints_planner_dir = get_package_share_directory('pose_constraints_planner')


    # --- Launch parameters ---
    solver_config_file_arg = DeclareLaunchArgument(
        'solver_param_file',
        default_value='solver_config.yaml',
        description='Path to the test solver parameter file'
    )


    # --- Param servers ---
    test_solver_param_file_path = PathJoinSubstitution([
        pose_constraints_planner_dir,
        'config',
        "solver_config.yaml"
    ])
    test_solver_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', test_solver_param_file_path],
        output='screen'
    )

    ik_param_file_path = PathJoinSubstitution([
        pose_constraints_planner_dir,
        'config',
        "ik_params_sharework.yaml"
    ])

    ik_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', ik_param_file_path],
        output='screen'
    )
    constraint_visualizer = Node(
        package='pose_constraints_planner',
        executable='geometric_constraints_visualizer.py',
        output='screen',
        parameters=[{'constaints_description': test_solver_param_file_path}]
    )

    planner = Node(
        package='pose_constraints_planner',
        executable='pose_constraints_planner_node',
        output='screen',
        parameters=[]
    )

    # --- Add delays between launches ---
    return LaunchDescription([
        solver_config_file_arg,
        test_solver_params,
        ik_params,
        constraint_visualizer,
        TimerAction(period=3.0, actions=[planner]),
    ])
