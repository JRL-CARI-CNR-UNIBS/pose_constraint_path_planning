# Copyright 2026 JRL-CARI CNR-STIIMA/UNIBS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Integration test: Sharework cell (fake hardware) + ik_solver + pose_constraints_planner + test node.

    ros2 launch pose_constraints_planner pose_constraints_planner_test.launch.py
    ros2 launch pose_constraints_planner pose_constraints_planner_test.launch.py scenarios:="[cancel, unknown_frame]"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('pose_constraints_planner')

    args = [
        DeclareLaunchArgument('start_cell', default_value='true',
                              description='Start the Sharework cell with fake hardware'),
        DeclareLaunchArgument('start_planner', default_value='true',
                              description='Start ik_solver and pose_constraints_planner'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('scenarios_file',
                              default_value=os.path.join(pkg, 'config', 'sharework_test_scenarios.yaml')),
        DeclareLaunchArgument('scenarios', default_value="['']",
                              description='Scenarios to run, e.g. "[cancel, unknown_frame]" (default: all)'),
        DeclareLaunchArgument('allowed_planning_time', default_value='5.0', description='Default planning time [s]'),
        DeclareLaunchArgument('check_resolution', default_value='0.01',
                              description='[rad] spacing of the constraint checks along the trajectory'),
        DeclareLaunchArgument('shutdown_when_done', default_value='true',
                              description='Stop everything when the test ends'),
    ]

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sharework_cembre_bringup'), 'launch', 'sharework_cembre_bringup.launch.py')),
        launch_arguments={
            'fake_ur': 'true',
            'fake_gripper': 'true',
            'robotiq_use_socket_communication': 'false',
            'enable_zed_camera': 'false',
            'enable_dual_realsense': 'false',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'launch_dashboard_client': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_cell')),
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'pose_constraint_solver.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_planner')),
    )

    test = Node(
        package='pose_constraints_planner',
        executable='pose_constraints_planner_test.py',
        name='pose_constraints_planner_test',
        output='screen',
        parameters=[{
            'scenarios_file': LaunchConfiguration('scenarios_file'),
            'scenarios': ParameterValue(LaunchConfiguration('scenarios'), value_type=list[str]),
            'allowed_planning_time': ParameterValue(LaunchConfiguration('allowed_planning_time'), value_type=float),
            'check_resolution': ParameterValue(LaunchConfiguration('check_resolution'), value_type=float),
            'server_timeout': 120.0,
        }],
    )

    shutdown = RegisterEventHandler(
        OnProcessExit(target_action=test, on_exit=[Shutdown(reason='pose constraints planner test finished')]),
        condition=IfCondition(LaunchConfiguration('shutdown_when_done')),
    )

    return LaunchDescription(args + [cell, planner, test, shutdown])
