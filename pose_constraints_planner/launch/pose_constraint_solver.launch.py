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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('pose_constraints_planner'), 'config')

    args = [
        DeclareLaunchArgument('solver_param_file', default_value=os.path.join(config_dir, 'solver_config.yaml'),
                              description='ROS 2 parameter file of the planner (pipeline, constraints, mirrt*)'),
        DeclareLaunchArgument('ik_param_file', default_value=os.path.join(config_dir, 'ik_params_sharework.yaml'),
                              description='Parameters of the in-process ik_solver plugin (forward kinematics)'),
        DeclareLaunchArgument('ik_solver_config', default_value=os.path.join(config_dir, 'ik_solver_sharework.yaml'),
                              description='Configuration of the ik_solver node (IK of the Cartesian goals)'),
        DeclareLaunchArgument('start_ik_solver', default_value='true',
                              description='Start the ik_solver node (set false if it is already running)'),
    ]

    ik_solver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ik_solver'), 'launch', 'ik_solver.launch.py')),
        launch_arguments={'config': LaunchConfiguration('ik_solver_config')}.items(),
        condition=IfCondition(LaunchConfiguration('start_ik_solver')),
    )

    constraint_visualizer = Node(
        package='pose_constraints_planner',
        executable='geometric_constraints_visualizer.py',
        output='screen',
    )

    planner = Node(
        package='pose_constraints_planner',
        executable='pose_constraints_planner_node',
        name='pose_constraints_planner',
        output='screen',
        parameters=[LaunchConfiguration('solver_param_file'), LaunchConfiguration('ik_param_file')],
    )

    return LaunchDescription(args + [
        ik_solver,
        constraint_visualizer,
        TimerAction(period=3.0, actions=[planner]),
    ])
