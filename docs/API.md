# pose_constraint_path_planning API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

It covers the C++ classes and functions and the scripts. The ROS interface of the nodes
(actions, services, topics and parameters) is described in the [README](../README.md).

## Contents

- [`pose_constraints_planner/include/pose_constraints_planner/plan_with_constraints_action_server.hpp`](#pose_constraints_plannerincludepose_constraints_plannerplan_with_constraints_action_serverhpp)
  - [`pose_constraints_planner::PlanWithConstraintsActionServer`](#class-pose_constraints_plannerplanwithconstraintsactionserver)
- [`pose_constraints_planner/include/pose_constraints_planner/pose_constraints_manager.hpp`](#pose_constraints_plannerincludepose_constraints_plannerpose_constraints_managerhpp)
  - [`pose_constraints_planner::GeometricConstraint`](#struct-pose_constraints_plannergeometricconstraint)
  - [`pose_constraints_planner::PoseConstraintsManager`](#class-pose_constraints_plannerposeconstraintsmanager)
- [`pose_constraints_planner/include/pose_constraints_planner/pose_constraints_evaluator.hpp`](#pose_constraints_plannerincludepose_constraints_plannerpose_constraints_evaluatorhpp)
  - [`pose_constraints_planner::PoseConstraintsEvaluator`](#class-pose_constraints_plannerposeconstraintsevaluator)
- [`pose_constraints_planner/include/pose_constraints_planner/pose_constrained_sampler.hpp`](#pose_constraints_plannerincludepose_constraints_plannerpose_constrained_samplerhpp)
  - [`pose_constraints_planner::PoseConstrainedSampler`](#class-pose_constraints_plannerposeconstrainedsampler)
- [`pose_constraints_planner/include/pose_constraints_planner/pose_constrained_collision_checker.hpp`](#pose_constraints_plannerincludepose_constraints_plannerpose_constrained_collision_checkerhpp)
  - [`pose_constraints_planner::PoseConstrainedCollisionChecker`](#class-pose_constraints_plannerposeconstrainedcollisionchecker)
- [`pose_constraints_planner/include/pose_constraints_planner/pose_constrained_multigoal_solver.hpp`](#pose_constraints_plannerincludepose_constraints_plannerpose_constrained_multigoal_solverhpp)
  - [`pose_constraints_planner::ConstrainedSamplingStatistics`](#struct-pose_constraints_plannerconstrainedsamplingstatistics)
  - [`pose_constraints_planner::PoseConstrainedMultigoalSolver`](#class-pose_constraints_plannerposeconstrainedmultigoalsolver)
- [`pose_constraints_planner/include/pose_constraints_planner/pose_constraints_planner.hpp`](#pose_constraints_plannerincludepose_constraints_plannerpose_constraints_plannerhpp)
  - [`pose_constraints_planner::PoseConstraintsPlanner`](#class-pose_constraints_plannerposeconstraintsplanner)
- [Scripts](#scripts)
- [Tests](#tests)

## pose_constraints_planner/include/pose_constraints_planner/plan_with_constraints_action_server.hpp

Action server of pose_constraints_msgs/action/PlanWithConstraints.

### class `pose_constraints_planner::PlanWithConstraintsActionServer`

Action server of `pose_constraints_msgs/action/PlanWithConstraints`.

Each accepted goal is executed on its own detached thread by `PoseConstraintsPlanner::planWithConstraints()`: `motion_plan_request.goal_constraints` are alternative goals (multi-goal MI-RRT*), the path satisfies the geometric constraints of the goal, and the result carries the MotionPlanResponse and the PlanningInfo statistics. Cancellation is checked at every planner iteration. The constraints of each goal are also published on `/geometry_constraints` for visualization.

#### Public types

| Type | Definition |
|---|---|
| `Action` | `pose_constraints_msgs::action::PlanWithConstraints` |
| `GoalHandle` | `rclcpp_action::ServerGoalHandle<Action>` |

#### Public functions

##### `PlanWithConstraintsActionServer`

```cpp
PlanWithConstraintsActionServer(const rclcpp::Node::SharedPtr &node, const std::string &action_name="/plan_with_constraints")
```

Create the planner and start the action server.

**Parameters:**

- `node`: Node; it must not be added to an executor yet (the robot model is loaded here).
- `action_name`: Action name.

**Throws:**

- `std::runtime_error`: if the planner cannot be created.

## pose_constraints_planner/include/pose_constraints_planner/pose_constraints_manager.hpp

Geometric constraints on the tool pose and their evaluation.

### struct `pose_constraints_planner::GeometricConstraint`

A geometric constraint on the tool pose (all quantities in the world frame).

- `PLANE`: the signed distance of the tool origin from the plane (along `plane_normal`) must be at least `plane_tolerance`, i.e. the tool stays on the side the normal points to;
- `LINE`: the distance of the tool origin from the line must be less than `line_max_distance`;
- `ANGLE`: for each axis i (x, y, z), the angle between the i-th rows of the current and of the start rotation matrices must not exceed component i of `max_angle`. Use a value &gt;= pi to leave an axis free.

#### Public types

##### `ConstraintType`

```cpp
enum ConstraintType { PLANE, LINE, ANGLE }
```

Constraint type (same values as pose_constraints_msgs/GeometricConstraint).

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `name` | `std::string` | Name, used in reports. |
| `frame_id` | `std::string` | Frame in which the constraint was defined (informative after `transformConstraint`). |
| `type` | `ConstraintType` | Constraint type. |
| `plane_origin` | `Eigen::Vector3d` | PLANE: a point of the plane. |
| `plane_normal` | `Eigen::Vector3d` | PLANE: normal pointing to the allowed side. |
| `plane_tolerance` | `double` | PLANE: minimum signed distance from the plane. |
| `line_origin` | `Eigen::Vector3d` | LINE: a point of the line. |
| `line_dir` | `Eigen::Vector3d` | LINE: line direction. |
| `line_max_distance` | `double` | LINE: maximum distance from the line. |
| `max_angle` | `Eigen::Vector3d` | ANGLE: maximum angle for each axis [rad]. |
| `max_angle_cos` | `Eigen::Vector3d` | ANGLE: cosine of max_angle (computed by the conversion functions; -2 for a free axis, i.e. a negative max_angle or one ≥ π). |
| `R_ref_world` | `Eigen::Matrix3d` | ANGLE: rotation from the world frame to the frame whose axes are compared (identity: world axes; set by `transformConstraint`). |

### class `pose_constraints_planner::PoseConstraintsManager`

Lightweight container + evaluator for geometric pose constraints.

This class provides a small public API to manage constraints and evaluate whether a pose (T_current) respects the stored constraints w.r.t. a reference pose (T_start).

#### Public types

##### `Ptr`

```cpp
using pose_constraints_planner::PoseConstraintsManager::Ptr = std::shared_ptr<PoseConstraintsManager>
```

define shared pointer `PoseConstraintsManager::Ptr`

#### Public functions

##### `PoseConstraintsManager`

```cpp
PoseConstraintsManager()
```

Constructor with no constraints.

##### `PoseConstraintsManager`

```cpp
PoseConstraintsManager(const std::vector< GeometricConstraint > &constraints)
```

Constructor.

**Parameters:**

- `constraints`: Initial constraints.

##### `addConstraint`

```cpp
void addConstraint(const GeometricConstraint &constraint)
```

Add a single constraint to the existing set.

##### `setConstraints`

```cpp
void setConstraints(const std::vector< GeometricConstraint > &constraints)
```

Replace the existing set of constraints.

##### `setConstraints`

```cpp
void setConstraints(const pose_constraints_msgs::msg::GeometricConstraintArray &constraints)
```

Replace the existing set of constraints with those of a ROS message.

##### `checkConstraints`

```cpp
bool checkConstraints(const Eigen::Affine3d &T_current, const Eigen::Affine3d &T_start, std::stringstream *report=nullptr) const
```

Evaluate all stored constraints.

**Parameters:**

- `T_current`: Pose to be checked.
- `T_start`: Reference pose (used by ANGLE constraints).
- `report`: Optional stream to receive a human-readable report.

**Returns:** True if all constraints are satisfied, false otherwise.

##### `constraints`

```cpp
const std::vector< GeometricConstraint > & constraints() const noexcept
```

Read-only access to the current constraints.

#### Protected functions

##### `checkAngleConstraint`

```cpp
bool checkAngleConstraint(const Eigen::Affine3d &T_1, const Eigen::Affine3d &T_2, const Eigen::Vector3d &max_angle_cos, const Eigen::Matrix3d &R_ref_world, double &angle_cos) const
```

Checks axis-wise maximum angle constraints between two transforms.

Compares, axis by axis, the rows of `R_ref_world * R_1` and `R_ref_world * R_2`, i.e. the deviation measured along the axes of the constraint frame. Axes with `max_angle_cos` < -1 are free.

##### `checkPlaneConstraint`

```cpp
bool checkPlaneConstraint(const Eigen::Affine3d &T, const Eigen::Vector3d &plane_origin, const Eigen::Vector3d &plane_normal, double tolerance, double &distance) const
```

Checks point-to-plane distance constraint.

Checks if the pose defined by transformation T satisfies the plane constraint.

##### `checkLineConstraint`

```cpp
bool checkLineConstraint(const Eigen::Affine3d &T, const Eigen::Vector3d &line_origin, const Eigen::Vector3d &line_dir, double max_distance, double &distance) const
```

Checks point-to-line distance constraint.

Checks if the pose defined by transformation T satisfies the line constraint.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `constraints_` | `std::vector< GeometricConstraint >` | Stored constraints. |

### Free functions and types

#### `convertGeometricConstraint`

```cpp
GeometricConstraint convertGeometricConstraint(const pose_constraints_msgs::msg::GeometricConstraint &msg)
```

Convert a ROS constraint message to a `GeometricConstraint` (also computes max_angle_cos).

#### `convertGeometricConstraintArray`

```cpp
std::vector< pose_constraints_planner::GeometricConstraint > convertGeometricConstraintArray(const pose_constraints_msgs::msg::GeometricConstraintArray &msg_array)
```

Convert a ROS constraint array message to a list of `GeometricConstraint`. The frames are copied but not resolved: use `transformConstraint` to express them in the world frame.

#### `transformConstraint`

```cpp
GeometricConstraint transformConstraint(const GeometricConstraint &constraint, const Eigen::Affine3d &T_world_frame)
```

Express a constraint defined in a frame F in the world frame, given the pose `T_world_frame` of F. PLANE and LINE origins are transformed as points, normals and directions are rotated; for ANGLE, `R_ref_world` is composed with the inverse rotation, so the angles are measured along the axes of F.

## pose_constraints_planner/include/pose_constraints_planner/pose_constraints_evaluator.hpp

Checks the geometric constraints on the tool pose of a joint configuration.

### class `pose_constraints_planner::PoseConstraintsEvaluator`

Computes the tool pose of a configuration (forward kinematics of an ik_solver plugin) and checks it with a `PoseConstraintsManager`.

Configurations are given in the planner joint order, which may differ from the order of the IK solver: they are permuted before the forward kinematics. ANGLE constraints are relative to the start tool pose. The object is immutable after construction; it is shared by samplers and collision checkers. Concurrent use requires a thread-safe forward kinematics in the IK solver.

```cpp
PoseConstraintsEvaluator(const std::shared_ptr< ik_solver::IkSolver > &ik_solver, const PoseConstraintsManager::Ptr &constraints, const std::vector< std::string > &planner_joint_names, const Eigen::Affine3d &T_w_b, const Eigen::Affine3d &T_f_t, const Eigen::VectorXd &start_q)
```

**Parameters:**

- `ik_solver`: IK solver, used for the forward kinematics.
- `constraints`: Geometric constraints (all quantities in the world frame).
- `planner_joint_names`: Joint order of the configurations given to the evaluator.
- `T_w_b`: Pose of the robot base in the world frame.
- `T_f_t`: Pose of the tool in the flange frame.
- `start_q`: Start configuration (planner joint order): its tool pose is the reference of ANGLE constraints.

**Throws:** `std::invalid_argument` if a pointer is null or the joints of the IK solver are not all in `planner_joint_names`.

| Function | Description |
|---|---|
| `Eigen::Affine3d toolPose(const Eigen::VectorXd &q) const` | Tool pose of `q` (planner joint order) in the world frame. |
| `bool satisfies(const Eigen::VectorXd &q, std::stringstream *report=nullptr) const` | True if the tool pose of `q` satisfies all the constraints. |
| `const Eigen::Affine3d& startPose() const` | Tool pose of the start configuration in the world frame. |
| `const PoseConstraintsManager::Ptr& constraints() const` | The constraints. |

## pose_constraints_planner/include/pose_constraints_planner/pose_constrained_sampler.hpp

graph_core sampler that returns only configurations whose tool pose satisfies the geometric constraints.

### class `pose_constraints_planner::PoseConstrainedSampler`

Rejection sampler on the geometric pose constraints, derived from `graph::core::SamplerBase`.

Wraps another graph_core sampler (uniform, informed, ...): `sample()` draws configurations from it and returns the first one whose tool pose satisfies the constraints of a `PoseConstraintsEvaluator`. The samples are therefore uniformly distributed over the intersection of the wrapped sampler's set and the constrained set.

If no feasible configuration is found within `max_attempts` draws, `sample()` returns the last (infeasible) draw and `lastSampleFeasible()` returns false: the caller decides what to do.

`setCost()`, `collapse()`, `getLB()` and `getUB()` are forwarded to the wrapped sampler. `inBounds()` also checks the constraints (`graph::core::TubeInformedSampler` relies on it to keep its tube samples feasible). `getSpecificVolume()` is the wrapped sampler's value scaled by the measured acceptance rate, an estimate of the volume of the constrained set.

`clone()` shares the evaluator with the original: if clones are used by parallel threads, the forward kinematics of the IK solver must be thread-safe.

```cpp
PoseConstrainedSampler(const graph::core::SamplerPtr &inner, const PoseConstraintsEvaluatorPtr &evaluator, const unsigned int &max_attempts, const graph::core::LoggerPtr &logger)
```

**Throws:** `std::invalid_argument` if `inner` or `evaluator` is null.

| Function | Description |
|---|---|
| `Eigen::VectorXd sample()` | Draw a configuration that satisfies the constraints (see above). |
| `void setCost(const double &cost)` / `bool collapse()` | Forwarded to the wrapped sampler. |
| `bool inBounds(const Eigen::VectorXd &q)` | True if `q` is within the wrapped sampler's bounds and satisfies the constraints. |
| `const Eigen::VectorXd& getLB()` / `getUB()` | Bounds of the wrapped sampler. |
| `graph::core::SamplerPtr clone()` | Copy with a clone of the wrapped sampler and reset statistics. |
| `bool lastSampleFeasible() const` | True if the last value returned by `sample()` satisfies the constraints. |
| `void setMaxAttempts(const unsigned int &)` / `unsigned int getMaxAttempts() const` | Maximum number of draws per call to `sample()`. |
| `unsigned long getAttempts() const` | Total number of draws. |
| `unsigned long getRejections() const` | Number of rejected draws. |
| `unsigned long getFailedSamples() const` | Number of `sample()` calls that returned an infeasible configuration. |
| `double getRejectTime() const` | Time spent on rejected draws [s]. |
| `double getSamplingTime() const` | Time spent in `sample()` [s]. |
| `double getAcceptanceRate() const` | Fraction of accepted draws (1 if nothing has been drawn yet). |
| `void resetStatistics()` | Reset attempts, rejections, failed samples and times. |
| `getInnerSampler()`, `getEvaluator()` | The wrapped sampler and the evaluator. |

## pose_constraints_planner/include/pose_constraints_planner/pose_constrained_collision_checker.hpp

Collision checker that also rejects configurations and edges violating the pose constraints.

### class `pose_constraints_planner::PoseConstrainedCollisionChecker`

Wraps a collision checker and adds the geometric pose constraints, derived from `graph::core::CollisionCheckerBase`.

A configuration is valid if it satisfies the constraints and the wrapped checker accepts it. An edge is valid if the constraints hold at points spaced at most `resolution` (joint-space norm) along it, and the wrapped checker accepts it. The constraints are checked first, since the forward kinematics is cheaper than a collision check; the wrapped checker keeps its own (e.g. parallel) edge checking. With this checker the solver, the goal filtering and any path optimizer enforce the constraints on the whole path, not only on the nodes.

```cpp
PoseConstrainedCollisionChecker(const graph::core::CollisionCheckerPtr &inner, const PoseConstraintsEvaluatorPtr &evaluator, const double &resolution, const graph::core::LoggerPtr &logger)
```

**Throws:** `std::invalid_argument` if `inner` or `evaluator` is null, or `resolution` is not positive.

| Function | Description |
|---|---|
| `check`, `checkConnection` (3 overloads), `checkConnections`, `checkConnFromConf` | Constraints first, then the wrapped checker. |
| `graph::core::CollisionCheckerPtr clone()` | Copy with a clone of the wrapped checker (the evaluator is shared). |
| `bool constraintsHoldAlong(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2, Eigen::VectorXd *last_valid=nullptr) const` | True if the constraints hold along the segment (end points included); `last_valid` gets the last valid checked point. |
| `unsigned long getConstraintRejections() const` | Number of configurations or edges rejected because of the constraints. |
| `getInnerChecker()` | The wrapped checker. |

## pose_constraints_planner/include/pose_constraints_planner/pose_constrained_multigoal_solver.hpp

MI-RRT* (`graph::mirrt_star::MultigoalSolver`) sampling only configurations that satisfy the pose constraints.

### struct `pose_constraints_planner::ConstrainedSamplingStatistics`

Sampling statistics of the per-goal `PoseConstrainedSampler` instances: `attempts`, `rejections`, `failed_samples`, `reject_time` [s], `sampling_time` [s], and `rejectionRate()`.

### class `pose_constraints_planner::PoseConstrainedMultigoalSolver`

`graph::mirrt_star::MultigoalSolver` whose per-goal samplers draw only configurations that satisfy the geometric pose constraints.

MultigoalSolver creates, for every registered goal, an informed sampler and a tube sampler around the goal's current solution. This class wraps each informed sampler with a `PoseConstrainedSampler` (rejection sampling on the constraints) and rebuilds the tube sampler on top of it, so that both the informed and the tube samples are feasible. Sampling makes the nodes feasible; the edges must be checked with a `PoseConstrainedCollisionChecker`.

```cpp
PoseConstrainedMultigoalSolver(const graph::core::MetricsPtr &metrics, const graph::core::CollisionCheckerPtr &checker, const graph::core::SamplerPtr &sampler, const graph::core::GoalCostFunctionPtr &goal_cost_fcn, const PoseConstraintsEvaluatorPtr &evaluator, const unsigned int &max_sampling_attempts, const graph::core::LoggerPtr &logger)
```

| Function | Description |
|---|---|
| `bool addGoal(const graph::core::NodePtr &goal_node, const double &max_time, const double &apple_cost)` | Register a candidate goal (see `MultigoalSolver::addGoal()`) and replace its samplers with constrained ones. |
| `ConstrainedSamplingStatistics samplingStatistics() const` | Sum of the statistics of all the constrained samplers created so far. |

## pose_constraints_planner/include/pose_constraints_planner/pose_constraints_planner.hpp

Multi-goal MI-RRT* planner whose path must satisfy geometric constraints on the tool pose.

### class `pose_constraints_planner::PoseConstraintsPlanner`

Multi-goal planner (`parallel_multigoal_planner::MultigoalPlanningPipeline`, MI-RRT*) with geometric constraints on the tool pose (PLANE, LINE, ANGLE; see `GeometricConstraint`).

For each request the pipeline of `parallel_multigoal_planner::MultigoalPlanningPipeline` is run (planning scene fetched from move_group, IK of the Cartesian goals from the ik_solver node, multiple alternative goals, cancellation, refinement until the utopia threshold) with these changes:

- the start configuration must satisfy the constraints (ANGLE constraints are relative to its tool pose);
- the collision checker is wrapped by a `PoseConstrainedCollisionChecker`, so nodes, edges and goal candidates violating the constraints are rejected;
- the solver is a `PoseConstrainedMultigoalSolver`, whose per-goal samplers are `PoseConstrainedSampler` (rejection sampling on the constraints);
- optionally, the solution is shortened by a `graph::core::PathLocalOptimizer` using the constrained checker.

The tool pose is computed in-process with the forward kinematics of an ik_solver plugin.

Each constraint is defined in its `frame_id`; if empty, in the `header.frame_id` of the array; if also empty, in `world_frame`. It is expressed in `world_frame` with TF at the start of each request.

Parameters (besides those of `MultigoalPlanningPipeline`): `world_frame` ("world"), `tf_timeout` (1.0 s, timeout of each TF lookup), `tool_frame` ("": flange frame), `ik_plugin_prefix` ("ur_ik_solver"; plugin `<prefix>.type`), `max_sampling_attempts` (1000, per request), `constraint_resolution` (0.01, per request), `use_local_optimizer` (false, per request; parameters under `local_optimizer.`).

#### Public functions

##### `PoseConstraintsPlanner`

```cpp
PoseConstraintsPlanner(const rclcpp::Node::SharedPtr &node, const graph::core::LoggerPtr &logger)
```

Declare the parameters, load the robot model, create the clients and load the ik_solver plugin. The node must not be added to an executor yet.

**Throws:** `std::runtime_error` if the robot model or the ik_solver plugin cannot be loaded.

##### `planWithConstraints`

```cpp
parallel_multigoal_planner::PlanningResult planWithConstraints(const pose_constraints_msgs::msg::GeometricConstraintArray &constraints, const moveit_msgs::msg::MotionPlanRequest &req, bool verbose, const FeedbackFcn &feedback, const CancelFcn &is_canceling, pose_constraints_msgs::msg::PlanningInfo &info) const
```

Plan a motion that satisfies the geometric constraints.

**Parameters:**

- `constraints`: Geometric constraints, each in its frame (see the class description).
- `req`: Motion plan request; `goal_constraints` are alternative goals (Cartesian XOR joint-space each).
- `verbose`: Print a summary of the goal set and of the solution.
- `feedback`: Feedback callback (may be empty).
- `is_canceling`: Cancellation callback (may be empty).
- `info`: Planning statistics (output).

**Returns:** The outcome; on failure the response carries the error code (`START_STATE_VIOLATES_PATH_CONSTRAINTS` if the start violates the constraints, `FRAME_TRANSFORM_FAILURE` if a frame of the constraints is not available).

##### `worldFrame`

```cpp
const std::string& worldFrame() const
```

Planning frame: the constraints are expressed in it.

#### Protected functions

##### `lookupWorldTransform`

```cpp
bool lookupWorldTransform(const std::string &frame, Eigen::Affine3d &T_world_frame, std::string &error) const
```

Pose of `frame` in `world_frame` from TF (identity if `frame` is `world_frame`). Returns false, with a message in `error`, if the transform is not available within `tf_timeout`.

## Scripts

### `pose_constraints_planner/pose_constraints_planner/geometric_constraints_visualizer.py`

```
RViz visualization of the geometric pose constraints.

Subscribes to /geometry_constraints (pose_constraints_msgs/GeometricConstraintArray) and
publishes planes, lines and angle limits as markers on /geometric_constraints_markers.
```

### `pose_constraints_planner/scripts/plan_example.py`

Minimal client of `/plan_with_constraints` for the Sharework cell: two alternative joint goals, a plane and an angle constraint in `world`, a line constraint in `ur10e_base_link`.

### `pose_constraints_planner/scripts/pose_constraints_planner_test.py`

Integration test of the action server. Runs the scenarios of a YAML file (`scenarios_file`, default `config/sharework_test_scenarios.yaml`; `scenarios` selects a subset), checks the error code or the cancellation, and for every successful plan checks that the trajectory goes from the start to a goal and that the constraints hold along it (interpolation every `check_resolution` rad, FK from `/compute_fk`, constraint frames from TF). Exits with code 0 only if all scenarios pass. Other parameters: `action_name`, `group_name`, `world_frame`, `tool_frame`, `allowed_planning_time`, `server_timeout`.

## Tests

`pose_constraints_planner/test/test_pose_constraints.cpp` (gtest, `colcon test`): unit tests of `PoseConstraintsManager`, `transformConstraint`, `PoseConstraintsEvaluator`, `PoseConstrainedSampler`, `PoseConstrainedCollisionChecker` and `PoseConstrainedMultigoalSolver` on a fake 4-joint robot.
