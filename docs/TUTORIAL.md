# Pose-constrained path planning tutorial

This tutorial shows how to plan a joint-space path whose **tool pose** stays inside geometric
constraints along the whole motion. For example, the tool must:

- stay above a table (plane constraint);
- stay close to a rail (line constraint);
- not tilt more than a given angle (angle constraint), e.g. to carry a glass of water.

You send the constraints together with a MoveIt `MotionPlanRequest` to the action
`/plan_with_constraints`, and you get back a `MotionPlanResponse`.

## 1. How it works

`pose_constraints_planner_node` creates one `PoseConstraintsPlanner` for every planning group of the
robot model. Each planner is an RRT in joint space:

1. a random configuration is sampled;
2. its tool pose is computed with the forward kinematics of the IK solver;
3. if the pose violates a constraint, the sample is rejected;
4. otherwise the tree is extended towards it, and the new node is checked again.

When the goal is a pose, its IK solutions are computed first and the tree tries to reach any of them.
Optionally, the path is then shortened with a local optimizer that keeps the constraints satisfied.

The constraints are checked at the tree nodes. The motion between two nodes is checked for collisions but
**not** for the pose constraints, so keep `max_distance` small if the constraints are tight.

## 2. Current limitations

The node is still tailored to the UR10e test cell. The following values are fixed in
`pose_constraints_planner_node.cpp`:

| Value | Fixed to |
|---|---|
| IK plugin | `ik_solver/Ur10eIkSolver`, configured from the namespace `/ur_ik_solver` |
| tool frame | `ur10e_tool0` |
| world frame (frame of the constraints) | `world` |
| action name | `/plan_with_constraints` |

To use another robot, change these values in the node.

## 3. Start the planner

1. Start the robot and MoveIt (fake hardware is enough):

   ```bash
   ros2 launch pose_constraints_planner init_test_solver.launch.py
   ```

2. Start the planner:

   ```bash
   ros2 launch pose_constraints_planner pose_constraint_solver.launch.py
   ```

   The launch file loads `config/solver_config.yaml` and `config/ik_params_sharework.yaml` into
   `cnr_param`, starts the constraint visualizer, and after 3 s starts `pose_constraints_planner_node`.

The main parameters in `config/solver_config.yaml` are:

```yaml
pose_constraints_planner:
  solver_config:
    max_distance: 1.0          # tree extension step (smaller = constraints checked more densely)
    extend: true
    goal_bias: 0.1             # probability of trying to connect to the goal
    checker_resolution: 0.01   # collision checking step along connections
    libraries: ["libgraph_core.so", "libmoveit_collision_checker.so"]
    sampler_plugin: graph::core::InformedSamplerPlugin
    metrics_plugin: graph::core::EuclideanMetricsPlugin
    checker_plugin: graph::collision_check::ParallelMoveitCollisionCheckerPlugin
    use_local_optimizer: true  # shorten the path, keeping the constraints
    simplify_max_conn_length: 0.5
    warp_min_conn_length: 0.01
    warp_min_step_size: 0.01
```

## 4. Define the constraints

Constraints are `pose_constraints_msgs/GeometricConstraint` messages. All quantities are in the
`world` frame (the `frame_id` field is currently ignored).

| Type | Fields | Satisfied when |
|---|---|---|
| `PLANE` (0) | `plane_origin`, `plane_normal`, `plane_tolerance` | the signed distance of the tool origin from the plane, along the normal, is ≥ `plane_tolerance` |
| `LINE` (1) | `line_origin`, `line_direction`, `line_max_distance` | the distance of the tool origin from the line is < `line_max_distance` |
| `ANGLE` (2) | `max_angle` (x, y, z) | for each axis, the tool orientation differs from the **start** orientation by at most `max_angle` |

For `ANGLE`, set an axis to π (3.1416) or more to leave it free.

> **Note:** the message definition says that a negative value (e.g. -1.0) leaves an axis free. With the
> current code this is not true: -1.0 is converted to cos(-1.0) ≈ 0.54, so the axis is limited to 1 rad.
> Use π instead.

Remember also that the start and goal poses must satisfy all the constraints, otherwise the request is
rejected.

## 5. Send a request from Python

`plan_example.py`: move the tool to a pose 30 cm away while staying 10 cm above the table plane
(z = 0.8 m) and keeping the tool z axis within 0.3 rad of its start direction.

```python
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Vector3, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from pose_constraints_msgs.msg import GeometricConstraint, GeometricConstraintArray
from pose_constraints_msgs.action import PlanWithConstraints

JOINTS = ["ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint", "ur10e_elbow_joint",
          "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"]
q_start = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
q_goal = [0.8, -1.40, 1.40, -1.57, -1.57, 0.0]


def main():
    rclpy.init()
    node = rclpy.create_node("plan_example")
    client = ActionClient(node, PlanWithConstraints, "/plan_with_constraints")
    client.wait_for_server()

    # --- constraints ---
    above_table = GeometricConstraint(
        name="above_table", type=GeometricConstraint.PLANE,
        plane_origin=Point(x=0.0, y=0.0, z=0.8),
        plane_normal=Vector3(x=0.0, y=0.0, z=1.0),
        plane_tolerance=0.10)                       # at least 10 cm above the plane
    keep_upright = GeometricConstraint(
        name="keep_upright", type=GeometricConstraint.ANGLE,
        max_angle=Vector3(x=3.1416, y=3.1416, z=0.3))  # only the z axis is limited

    goal = PlanWithConstraints.Goal()
    goal.constraints = GeometricConstraintArray(constraints=[above_table, keep_upright])
    goal.verbose = True

    # --- motion plan request: start state + joint goal ---
    req = goal.motion_plan_request
    req.group_name = "ur_manipulator"               # a planning group of your robot
    req.allowed_planning_time = 10.0
    req.start_state.joint_state.name = JOINTS
    req.start_state.joint_state.position = q_start
    gc = Constraints()
    for name, value in zip(JOINTS, q_goal):
        gc.joint_constraints.append(JointConstraint(joint_name=name, position=value, weight=1.0))
    req.goal_constraints.append(gc)

    send = client.send_goal_async(goal, feedback_callback=lambda f: node.get_logger().info(f.feedback.status))
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if not handle.accepted:
        node.get_logger().error("goal rejected")
        return
    res_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    result = res_future.result().result

    if result.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
        node.get_logger().error(f"planning failed, error code {result.motion_plan_response.error_code.val}")
        return
    traj = result.motion_plan_response.trajectory.joint_trajectory
    node.get_logger().info(f"path with {len(traj.points)} waypoints, "
                           f"planning time {result.info.total_time:.2f} s, "
                           f"{result.info.total_rejections} rejected samples")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Pose goal instead of joint goal

Instead of joint constraints, fill one position constraint and one orientation constraint. The position
is taken from `constraint_region.primitive_poses[0].position`, the orientation from
`orientation_constraints[0].orientation`, and the pose is transformed to `world` with TF:

```python
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from geometry_msgs.msg import Pose, Quaternion

pc = PositionConstraint()
pc.header.frame_id = "world"
pc.link_name = "ur10e_tool0"
pc.constraint_region.primitive_poses.append(Pose(position=Point(x=0.5, y=0.3, z=1.1)))
oc = OrientationConstraint()
oc.header.frame_id = "world"
oc.link_name = "ur10e_tool0"
oc.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
gc = Constraints(position_constraints=[pc], orientation_constraints=[oc])
```

The goal pose must satisfy the constraints too.

## 6. Read the result

- `motion_plan_response.trajectory`: the path as a joint trajectory. The timing is only a placeholder
  (1 s between waypoints): retime the path, e.g. with the `time_parametrization` action server, before sending them to the
  robot.
- `info` (`pose_constraints_msgs/PlanningInfo`): total time, number of iterations, number of rejected
  samples, and min/max/average iteration time. Many rejections mean that the constraints leave a small
  feasible region: try a smaller `max_distance`, or relax the constraints.

The constraints of the last request are published on `/geometry_constraints`, and the
`geometric_constraints_visualizer.py` node shows them in RViz.

## 7. Use the planner from C++

You can also use `pose_constraints_planner::PoseConstraintsPlanner` directly in your own node:

```cpp
auto planner = std::make_shared<pose_constraints_planner::PoseConstraintsPlanner>(
    node, checker, ik_solver, sampler, metrics, logger, display, "world", "tool0");
planner->setConstraints(constraints_msg);
if (!planner->setStartConfiguration(q_start) || !planner->setGoalPose(T_world_goal))
  return;  // start or goal violates the constraints

graph::core::PathPtr solution;
pose_constraints_planner::info stats;
if (planner->solve(10.0, solution, stats))
  RCLCPP_INFO(node->get_logger(), "solved in %.2f s", stats.total_time);
```

See the Doxygen documentation (`doxygen Doxyfile` in the repository folder) for the full API.
