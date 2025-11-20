from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pose_constraints_planner_dir = get_package_share_directory('pose_constraints_planner')

    # --- Param servers ---
    test_solver_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', os.path.join(pose_constraints_planner_dir, 'config', 'test_solver.yaml')],
        output='screen'
    )
    scene_manager_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', os.path.join(pose_constraints_planner_dir, 'config', 'scene.yaml')],
        output='screen'
    )
    ik_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', os.path.join(pose_constraints_planner_dir, 'config', 'ik_params.yaml')],
        output='screen'
    )

    # --- Scene manager and planner ---
    scene_manager = Node(
        package='cnr_scene_manager',
        executable='cnr_scene_manager',
        output='screen',
        parameters=[{'param_ns': '/pose_constraints_planner'}],
        prefix='gnome-terminal --'
    )

    constraint_visualizer = Node(
        package='pose_constraints_planner',
        executable='geometric_constraints_visualizer.py',
        output='screen',
        parameters=[{'constaints_description': os.path.join(pose_constraints_planner_dir, 'config', 'test_solver.yaml')}]
    )

    planner = Node(
        package='pose_constraints_planner',
        executable='test_rrt',
        output='screen'
    )

    # --- Add delays between launches ---
    return LaunchDescription([
        test_solver_params,
        scene_manager_params,
        ik_params,
        scene_manager,
        constraint_visualizer,
        TimerAction(period=3.0, actions=[planner]),
    ])
