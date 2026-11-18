#!/usr/bin/env python3
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
"""Minimal client of /plan_with_constraints (Sharework cell, group ur_robot).

Moves the tool from `home` to either of two joint goals while
- keeping the tool above the plane z = 1.15 m of the world frame;
- keeping the tool z axis within 0.35 rad of its start direction (the other axes are free);
- staying within 2 m of a line defined in the robot base frame (frame_id = ur10e_base_link).

    ros2 run pose_constraints_planner plan_example.py
"""
import rclpy
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, Vector3
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from pose_constraints_msgs.action import PlanWithConstraints
from pose_constraints_msgs.msg import GeometricConstraint, GeometricConstraintArray

JOINTS = ['ur10e_shoulder_pan_joint', 'ur10e_shoulder_lift_joint', 'ur10e_elbow_joint',
          'ur10e_wrist_1_joint', 'ur10e_wrist_2_joint', 'ur10e_wrist_3_joint']
Q_START = [1.5707, -2.4434, 2.4434, -1.5707, 1.5707, 0.0]
Q_GOALS = [[1.2, -1.8, 1.9, -1.7, 1.57, 0.0],      # alternative goals: the planner reaches the cheapest
           [0.8, -2.0, 2.0, -1.5, 1.57, -0.3]]


def main():
    rclpy.init()
    node = rclpy.create_node('plan_example')
    client = ActionClient(node, PlanWithConstraints, '/plan_with_constraints')
    if not client.wait_for_server(timeout_sec=30.0):
        node.get_logger().error('/plan_with_constraints not available')
        return 1

    # --- constraints (each one in its own frame) ---
    above_plane = GeometricConstraint(
        name='above_plane', frame_id='world', type=GeometricConstraint.PLANE,
        plane_origin=Point(x=0.0, y=0.0, z=1.15),
        plane_normal=Vector3(x=0.0, y=0.0, z=1.0),
        plane_tolerance=0.0)
    keep_upright = GeometricConstraint(
        name='keep_upright', frame_id='world', type=GeometricConstraint.ANGLE,
        max_angle=Vector3(x=-1.0, y=-1.0, z=0.35))   # negative: axis free
    near_rail = GeometricConstraint(
        name='near_rail', frame_id='ur10e_base_link', type=GeometricConstraint.LINE,
        line_origin=Point(x=0.0, y=0.0, z=0.0),
        line_direction=Vector3(x=1.0, y=0.0, z=0.0),
        line_max_distance=2.0)

    goal = PlanWithConstraints.Goal()
    goal.constraints = GeometricConstraintArray(constraints=[above_plane, keep_upright, near_rail])
    goal.verbose = True

    # --- motion plan request: start state + alternative joint goals ---
    req = goal.motion_plan_request
    req.group_name = 'ur_robot'
    req.allowed_planning_time = 5.0
    req.start_state.joint_state.name = JOINTS
    req.start_state.joint_state.position = Q_START
    for q in Q_GOALS:
        req.goal_constraints.append(Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=v, weight=1.0) for n, v in zip(JOINTS, q)]))

    send = client.send_goal_async(goal, feedback_callback=lambda f: node.get_logger().info(f.feedback.status))
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if not handle.accepted:
        node.get_logger().error('goal rejected')
        return 1
    res_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    result = res_future.result().result

    code = result.motion_plan_response.error_code.val
    if code != MoveItErrorCodes.SUCCESS:
        node.get_logger().error(f'planning failed, error code {code}')
        return 1
    traj = result.motion_plan_response.trajectory.joint_trajectory
    node.get_logger().info(f'path with {len(traj.points)} waypoints, planning time {result.info.total_time:.3f} s, '
                           f'{result.info.total_rejections} constraint rejections '
                           f'(rejection rate {result.info.rejection_rate:.3f})')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
