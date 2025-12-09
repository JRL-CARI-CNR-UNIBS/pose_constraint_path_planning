from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
import os

def generate_launch_description():
    pose_constraints_planner_dir = get_package_share_directory('pose_constraints_planner')

    # --- Launch configurations ---
    test_mode = LaunchConfiguration('test_mode') 
    use_sharework = LaunchConfiguration('use_sharework', default='false')

    # --- Launch parameters ---
    test_solver_config_file_arg = DeclareLaunchArgument(
        'test_solver_param_file',
        default_value='test_solver.yaml',
        description='Path to the test solver parameter file'
    )

    test_solver_test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='0',
        description='Set to an int N>0 to enable testing mode (no action client), and repeat the test N times'
    )

    use_sharework_arg = DeclareLaunchArgument(
        'use_sharework',
        default_value='false',
        description='Set to true to use the Sharework cell robot configuration'
    )

    # --- Param servers ---
    test_solver_param_file_path = PathJoinSubstitution([
        pose_constraints_planner_dir,
        'config',
        PythonExpression([
            '"test_solver_sharework.yaml" if "',
            LaunchConfiguration('use_sharework'),
            '" == "true" else "test_solver.yaml"'
        ])
    ])
    test_solver_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', test_solver_param_file_path],
        output='screen'
    )

    scene_manager_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', os.path.join(pose_constraints_planner_dir, 'config', 'scene.yaml')],
        output='screen'
    )

    ik_param_file_path = PathJoinSubstitution([
        pose_constraints_planner_dir,
        'config',
        PythonExpression([
            '"ik_params_sharework.yaml" if "',
            LaunchConfiguration('use_sharework'),
            '" == "true" else "ik_params.yaml"'
        ])
    ])

    ik_params = ExecuteProcess(
        cmd=['cnr_param_server', '-p', ik_param_file_path],
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
        parameters=[{'constaints_description': test_solver_param_file_path}]
    )

    planner = Node(
        package='pose_constraints_planner',
        executable='test_rrt',
        output='screen',
        parameters=[{'test_mode': LaunchConfiguration('test_mode'), 
                     'use_sharework': LaunchConfiguration('use_sharework')}]
    )

    # --- Add delays between launches ---
    return LaunchDescription([
        test_solver_config_file_arg,
        test_solver_test_mode_arg,
        use_sharework_arg,
        test_solver_params,
        scene_manager_params,
        ik_params,
        scene_manager,
        constraint_visualizer,
        TimerAction(period=3.0, actions=[planner]),
    ])
