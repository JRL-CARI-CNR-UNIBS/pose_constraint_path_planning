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
"""Integration test of the /plan_with_constraints action server.

Runs the scenarios of a YAML file (see config/sharework_test_scenarios.yaml) and checks every result:
- the error code (or the cancellation) is the expected one;
- on success, the trajectory starts at the start configuration and ends at one of the goals;
- on success, the constraints hold along the whole trajectory: it is interpolated every
  `check_resolution` rad, the tool pose of every point is computed with MoveIt /compute_fk, and the
  constraints are expressed in `world_frame` with TF (as the planner does).

The node exits with code 0 only if every scenario passes.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from moveit_msgs.srv import GetPositionFK
from pose_constraints_msgs.action import PlanWithConstraints
from pose_constraints_msgs.msg import GeometricConstraint

ERROR_NAMES = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int) and k.isupper()}
TYPES = {'plane': GeometricConstraint.PLANE, 'line': GeometricConstraint.LINE, 'angle': GeometricConstraint.ANGLE}


def quat_to_rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class WorldConstraint:
    """A constraint expressed in the world frame (same rules as transformConstraint() in C++)."""

    def __init__(self, msg, R_w_f, t_w_f):
        self.name = msg.name
        self.type = msg.type
        vec = lambda v: np.array([v.x, v.y, v.z])
        if msg.type == GeometricConstraint.PLANE:
            self.origin = R_w_f @ vec(msg.plane_origin) + t_w_f
            n = R_w_f @ vec(msg.plane_normal)
            self.normal = n / np.linalg.norm(n)
            self.tolerance = msg.plane_tolerance
        elif msg.type == GeometricConstraint.LINE:
            self.origin = R_w_f @ vec(msg.line_origin) + t_w_f
            d = R_w_f @ vec(msg.line_direction)
            self.direction = d / np.linalg.norm(d)
            self.max_distance = msg.line_max_distance
        else:
            self.R_ref_w = R_w_f.T
            a = vec(msg.max_angle)
            self.max_cos = [(-2.0 if (ai < 0.0 or ai >= math.pi) else math.cos(ai)) for ai in a]

    def violation(self, p, R, R_start, eps=1e-6):
        """Return a description of the violation, or None."""
        if self.type == GeometricConstraint.PLANE:
            d = (p - self.origin).dot(self.normal)
            return None if d >= self.tolerance - eps else f'{self.name}: distance {d:.4f} < {self.tolerance}'
        if self.type == GeometricConstraint.LINE:
            v = p - self.origin
            d = np.linalg.norm(v - v.dot(self.direction) * self.direction)
            return None if d < self.max_distance + eps else f'{self.name}: distance {d:.4f} >= {self.max_distance}'
        R1, R2 = self.R_ref_w @ R, self.R_ref_w @ R_start
        for i, mc in enumerate(self.max_cos):
            if mc > -1:
                c = R1[i].dot(R2[i]) / (np.linalg.norm(R1[i]) * np.linalg.norm(R2[i]))
                if c < mc - eps:
                    return f'{self.name}: axis {i} angle {math.degrees(math.acos(max(-1.0, min(1.0, c)))):.1f} deg'
        return None


class PoseConstraintsPlannerTest(Node):
    def __init__(self):
        super().__init__('pose_constraints_planner_test')
        p = self.declare_parameter
        self.scenarios_file = p('scenarios_file', '').value
        self.only = [s for s in p('scenarios', ['']).value if s]
        self.action_name = p('action_name', '/plan_with_constraints').value
        self.group = p('group_name', 'ur_robot').value
        self.world_frame = p('world_frame', 'world').value
        self.tool_frame = p('tool_frame', 'ur10e_tool0').value
        self.default_time = p('allowed_planning_time', 5.0).value
        self.check_resolution = p('check_resolution', 0.01).value
        self.server_timeout = p('server_timeout', 120.0).value
        self.joint_tolerance = 1e-3

        self.ac = ActionClient(self, PlanWithConstraints, self.action_name)
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ------------------------------------------------------------------ helpers
    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'service {client.srv_name} timed out')
        return future.result()

    def fk(self, q):
        req = GetPositionFK.Request()
        req.header.frame_id = self.world_frame
        req.fk_link_names = [self.tool_frame]
        req.robot_state.joint_state.name = self.joints
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self.call(self.fk_client, req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'compute_fk failed ({ERROR_NAMES.get(res.error_code.val)})')
        pose = res.pose_stamped[0].pose
        o = pose.orientation
        return np.array([pose.position.x, pose.position.y, pose.position.z]), quat_to_rot(o.x, o.y, o.z, o.w), pose

    def world_transform(self, frame):
        if frame == self.world_frame:
            return np.eye(3), np.zeros(3)
        deadline = time.time() + 5.0
        while True:
            try:
                tf = self.tf_buffer.lookup_transform(self.world_frame, frame, Time())
                break
            except Exception:
                if time.time() > deadline:
                    raise
                rclpy.spin_once(self, timeout_sec=0.1)
        t, r = tf.transform.translation, tf.transform.rotation
        return quat_to_rot(r.x, r.y, r.z, r.w), np.array([t.x, t.y, t.z])

    def constraint_msg(self, name):
        spec = self.constraints[name]
        c = GeometricConstraint(name=name, frame_id=spec.get('frame', ''), type=TYPES[spec['type']])
        v3 = lambda key: [float(x) for x in spec[key]]
        if c.type == GeometricConstraint.PLANE:
            c.plane_origin.x, c.plane_origin.y, c.plane_origin.z = v3('origin')
            c.plane_normal.x, c.plane_normal.y, c.plane_normal.z = v3('normal')
            c.plane_tolerance = float(spec.get('tolerance', 0.0))
        elif c.type == GeometricConstraint.LINE:
            c.line_origin.x, c.line_origin.y, c.line_origin.z = v3('origin')
            c.line_direction.x, c.line_direction.y, c.line_direction.z = v3('direction')
            c.line_max_distance = float(spec['max_distance'])
        else:
            c.max_angle.x, c.max_angle.y, c.max_angle.z = v3('max_angle')
        return c

    def goal_constraints(self, goals):
        out, targets = [], []
        for g in goals:
            c = Constraints()
            if 'joint' in g:
                q = self.configurations[g['joint']]
                c.joint_constraints = [JointConstraint(joint_name=n, position=v, weight=1.0)
                                       for n, v in zip(self.joints, q)]
                targets.append(('joint', np.array(q)))
            else:
                p, R, pose = self.fk(self.configurations[g['pose_of']])
                pc = PositionConstraint(link_name=self.tool_frame)
                pc.header.frame_id = self.world_frame
                pc.constraint_region = BoundingVolume(primitive_poses=[Pose(position=pose.position)])
                oc = OrientationConstraint(link_name=self.tool_frame, orientation=pose.orientation)
                oc.header.frame_id = self.world_frame
                c.position_constraints = [pc]
                c.orientation_constraints = [oc]
                targets.append(('pose', (p, R)))
            out.append(c)
        return out, targets

    # ------------------------------------------------------------------ checks
    def check_path(self, points, start, targets, world_constraints):
        if np.max(np.abs(points[0] - start)) > self.joint_tolerance:
            return 'trajectory does not start at the start configuration', 0
        end_p, end_R, _ = self.fk(points[-1])
        reached = False
        for kind, t in targets:
            if kind == 'joint' and np.max(np.abs(points[-1] - t)) <= self.joint_tolerance:
                reached = True
            if kind == 'pose' and np.linalg.norm(end_p - t[0]) < 1e-3 and np.linalg.norm(end_R - t[1]) < 1e-2:
                reached = True
        if not reached:
            return 'trajectory does not end at a goal', 0
        _, R_start, _ = self.fk(points[0])
        checked = 0
        for q0, q1 in zip(points[:-1], points[1:]):
            n = max(1, int(math.ceil(np.linalg.norm(q1 - q0) / self.check_resolution)))
            for k in range(n + 1):
                p, R, _ = self.fk(q0 + (q1 - q0) * k / n)
                checked += 1
                for c in world_constraints:
                    why = c.violation(p, R, R_start)
                    if why:
                        return f'constraint violated along the path: {why}', checked
        return None, checked

    def run_scenario(self, s):
        start = np.array(self.configurations[s.get('start', 'home')])
        expect = s.get('expect', 'success')
        msgs = [self.constraint_msg(n) for n in s.get('constraints', [])]

        goal = PlanWithConstraints.Goal()
        goal.constraints.header.frame_id = s.get('header_frame', '')
        goal.constraints.constraints = msgs
        goal.verbose = bool(s.get('verbose', False))
        req = MotionPlanRequest()
        req.group_name = self.group
        req.allowed_planning_time = float(s.get('allowed_planning_time', self.default_time))
        req.start_state.joint_state.name = self.joints
        req.start_state.joint_state.position = start.tolist()
        req.goal_constraints, targets = self.goal_constraints(s['goals'])
        goal.motion_plan_request = req

        t0 = time.time()
        send = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        gh = send.result()
        if not gh.accepted:
            return expect == 'rejected', 'rejected', time.time() - t0, None
        if 'cancel_after' in s:
            end = time.time() + float(s['cancel_after'])
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            gh.cancel_goal_async()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        wall = time.time() - t0
        wrapped = rf.result()
        result = wrapped.result
        info = result.info
        if wrapped.status == 5:  # GoalStatus.STATUS_CANCELED
            return expect == 'canceled', 'canceled', wall, info
        code = ERROR_NAMES.get(result.motion_plan_response.error_code.val,
                               str(result.motion_plan_response.error_code.val))
        if code != ('SUCCESS' if expect == 'success' else expect):
            return False, f'{code} (expected {expect})', wall, info
        if code != 'SUCCESS':
            return True, code, wall, info

        world_constraints = []
        for m in msgs:
            frame = m.frame_id or goal.constraints.header.frame_id or self.world_frame
            R, t = self.world_transform(frame)
            world_constraints.append(WorldConstraint(m, R, t))
        points = [np.array(p.positions) for p in result.motion_plan_response.trajectory.joint_trajectory.points]
        error, checked = self.check_path(points, start, targets, world_constraints)
        if error:
            return False, error, wall, info
        return True, f'success, {len(points)} points, {checked} poses checked', wall, info

    # ------------------------------------------------------------------ main
    def run(self):
        with open(self.scenarios_file) as f:
            data = yaml.safe_load(f)
        self.joints = data['joint_names']
        self.configurations = {k: [float(v) for v in q] for k, q in data['configurations'].items()}
        self.constraints = data.get('constraints') or {}
        scenarios = [s for s in data['scenarios'] if not self.only or s['name'] in self.only]

        if not self.ac.wait_for_server(timeout_sec=self.server_timeout):
            self.get_logger().error(f'action {self.action_name} not available')
            return False
        if not self.fk_client.wait_for_service(timeout_sec=self.server_timeout):
            self.get_logger().error('/compute_fk not available')
            return False

        rows, passed = [], 0
        for s in scenarios:
            try:
                ok, outcome, wall, info = self.run_scenario(s)
            except Exception as ex:  # report and continue with the next scenario
                ok, outcome, wall, info = False, f'exception: {ex}', 0.0, None
            passed += ok
            stats = (f'iterations={info.total_iterations} rejections={info.total_rejections} '
                     f'rejection_rate={info.rejection_rate:.3f}') if info else ''
            self.get_logger().info(f'[{"PASS" if ok else "FAIL"}] {s["name"]}: {outcome} ({wall:.3f} s) {stats}')
            rows.append((s['name'], 'PASS' if ok else 'FAIL', f'{wall:.3f}', outcome))

        width = max(len(r[0]) for r in rows) if rows else 10
        print(f'\n{"scenario":<{width}}  result  wall[s]  outcome', flush=True)
        for r in rows:
            print(f'{r[0]:<{width}}  {r[1]:<6}  {r[2]:>7}  {r[3]}', flush=True)
        print(f'{passed}/{len(rows)} passed', flush=True)
        return passed == len(rows)


def main():
    rclpy.init()
    node = PoseConstraintsPlannerTest()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
