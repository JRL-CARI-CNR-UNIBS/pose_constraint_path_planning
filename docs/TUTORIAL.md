# Pose-constrained path planning tutorial

This tutorial shows how to plan a joint-space path whose **tool pose** stays inside geometric
constraints along the whole motion. For example, the tool must:

- stay above a table (plane constraint);
- stay close to a rail (line constraint);
- not tilt more than a given angle (angle constraint), e.g. to carry a glass of water.

You send the constraints together with a MoveIt `MotionPlanRequest` to the action
`/plan_with_constraints`, and you get back a `MotionPlanResponse`.

## 1. How it works

`pose_constraints_planner_node` runs the multi-goal pipeline of `parallel_multigoal_planner`
(`parallel_multigoal_planner::MultigoalPlanningPipeline`, solver MI-RRT\* from `mirrt_star`) with the
pose constraints added. For each request:

1. the planning scene is fetched from move_group (`/get_planning_scene`) and an in-process MoveIt
   collision checker is built on it;
2. the start configuration must satisfy the constraints (ANGLE constraints are relative to its tool pose);
3. every element of `goal_constraints` is an **alternative** goal: joint goals are used as they are,
   Cartesian goals are solved with the `ik_solver` node (`get_task_redundant_ik_array`), and only the
   solutions that are collision free **and** satisfy the constraints are kept;
4. MI-RRT\* grows the trees towards all the goals and keeps refining after the first solution, until
   the utopia threshold, the time budget, or `max_refine_time`;
5. optionally, the path is shortened with a local optimizer.

The constraints are enforced in two places:

- **sampling**: the per-goal samplers of MI-RRT\* are wrapped by `PoseConstrainedSampler`, which draws
  configurations and rejects those whose tool pose violates a constraint (rejection sampling);
- **edges**: the collision checker is wrapped by `PoseConstrainedCollisionChecker`, which also checks the
  constraints at points spaced `constraint_resolution` along every edge. Tree extensions, rewiring, goal
  connections and the local optimizer therefore keep the constraints along the **whole** path, not only
  at the nodes.

The tool pose is computed in-process with the forward kinematics of an `ik_solver` plugin.

## 2. Requirements

- move_group running (planning scene service);
- an `ik_solver` node for the IK of the Cartesian goals (started by the launch file, configuration
  `config/ik_solver_sharework.yaml`, namespace `ur_ik_solver`);
- the in-process ik_solver plugin parameters (`config/ik_params_sharework.yaml`, prefix `ur_ik_solver.`),
  used for the forward kinematics. Its joints must be joints of the planning group (the order may differ).

## 3. Start the planner

1. Start the robot and MoveIt (fake hardware is enough), e.g. the Sharework cell.

2. Start the planner:

   ```bash
   ros2 launch pose_constraints_planner pose_constraint_solver.launch.py
   ```

   The launch file starts the `ik_solver` node (`start_ik_solver:=false` if it is already running), the
   constraint visualizer and, after 3 s, `pose_constraints_planner_node` with the ROS 2 parameters in
   `config/solver_config.yaml` and `config/ik_params_sharework.yaml`.

The parameters in `config/solver_config.yaml` ("per request" ones are read at every request and can be
changed with `ros2 param set`):

| Parameter | Default | Meaning |
|---|---|---|
| `ik_solver_ns` | `ik_solver` | namespace of the ik_solver node |
| `planning_scene_service` | `/get_planning_scene` | planning scene service |
| `default_allowed_planning_time` | 5.0 | [s] used when `allowed_planning_time` ≤ 0 |
| `checker_resolution`, `checker_threads` | 0.02, 4 | collision checking along edges |
| `max_refine_time` | -1.0 | per request; < 0: refine until the time budget or the utopia threshold, 0: first solution |
| `collision_padding` | 0.0 | per request; [m] padding of the group links |
| `waypoint_dt_s` | 0.5 | [s] placeholder time between trajectory points |
| `world_frame` | `world` | planning frame: every constraint is expressed in it with TF |
| `tf_timeout` | 1.0 | [s] timeout of each TF lookup of the constraint frames |
| `tool_frame` | flange frame | constrained tool frame |
| `ik_plugin_prefix` | `ur_ik_solver` | prefix of the in-process ik_solver plugin parameters (plugin `<prefix>.type`) |
| `max_sampling_attempts` | 1000 | per request; maximum draws of the rejection sampler per sample |
| `constraint_resolution` | 0.01 | per request; [rad] spacing of the constraint checks along an edge |
| `use_local_optimizer` | false | per request; shorten the path (parameters under `local_optimizer.`) |
| `mirrt_star.*` | | per request; MI-RRT\* parameters (`max_distance`, `extend`, `utopia_tolerance`, ...) |

## 4. Define the constraints

Constraints are `pose_constraints_msgs/GeometricConstraint` messages, collected in a
`GeometricConstraintArray`.

| Type | Fields | Satisfied when |
|---|---|---|
| `PLANE` (0) | `plane_origin`, `plane_normal`, `plane_tolerance` | the signed distance of the tool origin from the plane, along the normal, is ≥ `plane_tolerance` |
| `LINE` (1) | `line_origin`, `line_direction`, `line_max_distance` | the distance of the tool origin from the line is < `line_max_distance` |
| `ANGLE` (2) | `max_angle` (x, y, z) | for each axis, the tool orientation differs from the **start** orientation by at most `max_angle` |

For `ANGLE`, a negative value (e.g. -1.0) or a value ≥ π leaves that axis free. Normals and directions
do not need to be unit vectors.

### Frames

Each constraint is defined in a frame, chosen in this order:

1. its own `frame_id`;
2. if empty, the `header.frame_id` of the `GeometricConstraintArray`;
3. if also empty, the `world_frame` parameter.

At the start of each request the planner reads the pose of that frame in `world_frame` from TF
(timeout `tf_timeout`) and expresses the constraint in `world_frame`: origins are transformed as points,
normals and directions are rotated. For `ANGLE`, the deviation from the start orientation is measured
along the axes of the constraint frame. The transform is read **once per request**, so a frame that moves
during the motion is frozen at its pose when the request starts. If a frame is not available, the request
fails with `FRAME_TRANSFORM_FAILURE`.

Example: in the Sharework cell the robot is ceiling-mounted (`ur10e_base_link` at z = 2.058 m, base z
axis pointing down), so the plane "tool above z = 1.15 m in world" can also be written in the base frame:

```python
GeometricConstraint(name="floor", frame_id="ur10e_base_link", type=GeometricConstraint.PLANE,
                    plane_origin=Point(x=0.0, y=0.0, z=0.908),     # 2.058 - 1.15
                    plane_normal=Vector3(x=0.0, y=0.0, z=-1.0))
```

### Start and goals

The start configuration must satisfy all the constraints, otherwise the request fails with
`START_STATE_VIOLATES_PATH_CONSTRAINTS`. Goal candidates that violate them are discarded; if none is
left, the request fails with `NO_IK_SOLUTION` (Cartesian goals) or `INVALID_GOAL_CONSTRAINTS`.

## 5. Send a request from Python

`scripts/plan_example.py` (installed, run it with `ros2 run pose_constraints_planner plan_example.py`
while the planner runs in the Sharework cell) moves the tool from the `home` configuration to either of
two joint goals while keeping the tool above a plane, keeping its z axis within 0.35 rad of the start
direction, and staying near a line defined in the robot base frame:

```python
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
```

### Pose goal instead of joint goal

Instead of joint constraints, fill one position constraint and one orientation constraint. The position
is taken from `constraint_region.primitive_poses[0].position`, the orientation from
`orientation_constraints[0].orientation`; the frame is the `header.frame_id` of the constraints, and the
IK is computed by the ik_solver node (as in `parallel_multigoal_planner`).
Several elements in `goal_constraints` are alternative goals: the planner reaches the cheapest one.

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

IK solutions of the goal pose that violate the constraints are discarded.

## 6. Read the result

- `motion_plan_response.trajectory`: the path as a joint trajectory. The timing is only a placeholder
  (`waypoint_dt_s` between waypoints): retime the path, e.g. with the `time_parametrization` action server,
  before sending it to the robot.
- `info` (`pose_constraints_msgs/PlanningInfo`), all times in seconds:
  - `total_time`: duration of the planning loop; `total_iterations`: MI-RRT\* iterations;
  - `min/max/average_iteration_time`;
  - `total_rejections`: sampler draws plus configurations/edges rejected by the constraints;
  - `rejection_rate`: fraction of sampler draws rejected; `overall_sampling_time`, `total_reject_time`.

  A rejection rate close to 1 means that the constraints leave a small feasible region: increase
  `max_sampling_attempts`, or relax the constraints.

The constraints of each request are published on `/geometry_constraints`, and the
`geometric_constraints_visualizer.py` node shows them in RViz.

## 7. Use the planner from C++

`pose_constraints_planner::PoseConstraintsPlanner` derives from
`parallel_multigoal_planner::MultigoalPlanningPipeline` and can be used without the action server:

```cpp
auto logger = std::make_shared<pose_constraints_planner::RclcppGraphLogger>(node->get_logger());
pose_constraints_planner::PoseConstraintsPlanner planner(node, logger);  // before adding node to the executor
// ... spin the node on another thread ...
pose_constraints_msgs::msg::PlanningInfo info;
auto result = planner.planWithConstraints(constraints_msg, motion_plan_request, /*verbose=*/true,
                                          /*feedback=*/nullptr, /*is_canceling=*/nullptr, info);
if (result.response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  RCLCPP_INFO(node->get_logger(), "solved in %.2f s", info.total_time);
```

The building blocks can also be used with other graph_core solvers: `PoseConstraintsEvaluator`
(forward kinematics + constraint check), `PoseConstrainedSampler`, `PoseConstrainedCollisionChecker`
and `PoseConstrainedMultigoalSolver`. See the Doxygen documentation (`doxygen Doxyfile` in the
repository folder) for the full API.

## 8. Tests and examples

### Unit tests

`test/test_pose_constraints.cpp` (gtest) tests the constraint checks, `transformConstraint`, the
evaluator (joint permutation), the constrained sampler (every sample feasible, statistics), the
constrained collision checker (an edge between two feasible configurations that leaves the feasible
set is rejected) and the constrained MI-RRT\* solver. It uses a fake 4-joint robot and needs no ROS
graph:

```bash
colcon build --packages-select pose_constraints_planner
colcon test --packages-select pose_constraints_planner
colcon test-result --verbose
```

### Integration test

`launch/pose_constraints_planner_test.launch.py` starts the Sharework cell with fake hardware,
the ik_solver node, the planner and `scripts/pose_constraints_planner_test.py`, and stops everything
when the test ends:

```bash
ros2 launch pose_constraints_planner pose_constraints_planner_test.launch.py
# a subset, with RViz
ros2 launch pose_constraints_planner pose_constraints_planner_test.launch.py \
    scenarios:="[plane_in_base_frame, cancel]" launch_rviz:=true
```

The scenarios are in `config/sharework_test_scenarios.yaml` (configurations, named constraints, goals,
expected outcome). For each successful plan, the test interpolates the trajectory every
`check_resolution` rad (default 0.01), computes the tool pose of each point with `/compute_fk`, and
checks the constraints in Python, independently of the planner. It also checks that the trajectory
starts at the start configuration and ends at one of the goals. The default scenarios cover:

| Group | Scenarios |
|---|---|
| success | unconstrained, angle with joint goals, angle + plane with Cartesian and mixed goals, far goals, goals that need tree growth, line + angle |
| frames | plane in the base frame, frame from the array header, goal violating a base-frame plane, unknown frame (`FRAME_TRANSFORM_FAILURE`) |
| failures | start violating (`START_STATE_VIOLATES_PATH_CONSTRAINTS`), joint goal violating (`INVALID_GOAL_CONSTRAINTS`), Cartesian goal violating (`NO_IK_SOLUTION`), cancellation |

The test node exits with code 0 only if all the scenarios pass, and prints a summary table. To run it
against an already running planner: `start_cell:=false start_planner:=false`.

To write scenarios for another robot, copy the YAML file and set `group_name`, `world_frame`,
`tool_frame` as parameters of the test node.
