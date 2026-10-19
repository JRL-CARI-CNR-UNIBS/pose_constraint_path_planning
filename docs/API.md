# pose_constraint_path_planning API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

It covers the C++ classes and functions and the scripts. The ROS interface of the nodes
(actions, services, topics and parameters) is described in the [README](../README.md).

## Contents

- [`pose_constraints_planner/include/pose_constraints_planner/plan_with_constraints_action_server.hpp`](#pose_constraints_plannerincludepose_constraints_plannerplan_with_constraints_action_serverhpp)
  - [`pose_constraints_planner::PlanWithConstraintsMuxActionServer`](#class-pose_constraints_plannerplanwithconstraintsmuxactionserver)
- [`pose_constraints_planner/include/pose_constraints_planner/pose_constraints_manager.hpp`](#pose_constraints_plannerincludepose_constraints_plannerpose_constraints_managerhpp)
  - [`pose_constraints_planner::GeometricConstraint`](#struct-pose_constraints_plannergeometricconstraint)
  - [`pose_constraints_planner::PoseConstraintsManager`](#class-pose_constraints_plannerposeconstraintsmanager)
- [`pose_constraints_planner/include/pose_constraints_planner/pose_constraints_path_local_optimizer.h`](#pose_constraints_plannerincludepose_constraints_plannerpose_constraints_path_local_optimizerh)
  - [`pose_constraints_planner::PoseConstrainedPathLocalOptimizer`](#class-pose_constraints_plannerposeconstrainedpathlocaloptimizer)
- [`pose_constraints_planner/include/pose_constraints_planner/pose_constraints_planner.hpp`](#pose_constraints_plannerincludepose_constraints_plannerpose_constraints_plannerhpp)
  - [`pose_constraints_planner::info`](#struct-pose_constraints_plannerinfo)
  - [`pose_constraints_planner::PoseConstraintsPlanner`](#class-pose_constraints_plannerposeconstraintsplanner)
- [Scripts](#scripts)

## pose_constraints_planner/include/pose_constraints_planner/plan_with_constraints_action_server.hpp

### class `pose_constraints_planner::PlanWithConstraintsMuxActionServer`

Multiplexing action server that dispatches planning requests to a set of `PoseConstraintsPlanner` implementations keyed by group name.

This lightweight server exposes a ROS2 action of type pose_constraints_msgs::action::PlanWithConstraints and selects the appropriate planner at runtime based on the incoming request's MotionPlanRequest.group_name.

Threading model:

- The action server accepts goals and executes each accepted goal on a detached std::thread.
- Internally planners are treated as stateful; a single mutex (planner_mtx_) is used to serialize access to planner instances to avoid concurrent state corruption.

#### Public types

##### `Action`

```cpp
using pose_constraints_planner::PlanWithConstraintsMuxActionServer::Action = pose_constraints_msgs::action::PlanWithConstraints
```

##### `GoalHandle`

```cpp
using pose_constraints_planner::PlanWithConstraintsMuxActionServer::GoalHandle = rclcpp_action::ServerGoalHandle<Action>
```

#### Public functions

##### `PlanWithConstraintsMuxActionServer`

```cpp
PlanWithConstraintsMuxActionServer(const rclcpp::Node::SharedPtr &node, const std::string &action_name, const std::string &world_frame, const std::shared_ptr< tf2_ros::Buffer > &tf_buffer, const std::map< std::string, pose_constraints_planner::PoseConstraintsPlanner::Ptr > &planners_map, const std::map< std::string, std::vector< std::string > > &joint_names_map, const std::map< std::string, pose_constraints_planner::PoseConstrainedPathLocalOptimizer::Ptr > &local_optimizers_map)
```

Construct a new `PlanWithConstraintsMuxActionServer`.

**Parameters:**

- `node`: Shared pointer to the rclcpp::Node used for logging and creating the action server
- `action_name`: Name of the action to create (e.g. "plan_with_constraints")
- `world_frame`: The world TF frame to which pose goals will be transformed
- `tf_buffer`: Shared pointer to the tf2_ros::Buffer used for transforms
- `planners_map`: Map from group_name to `PoseConstraintsPlanner::Ptr`. Owned externally; kept by reference
- `joint_names_map`: Map from group_name to the ordered list of joint names for that group. Owned externally; kept by reference

**Throws:**

- `std::runtime_error`: if node or tf_buffer is null

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
| `type` | `ConstraintType` | Constraint type. |
| `plane_origin` | `Eigen::Vector3d` | PLANE: a point of the plane. |
| `plane_normal` | `Eigen::Vector3d` | PLANE: normal pointing to the allowed side. |
| `plane_tolerance` | `double` | PLANE: minimum signed distance from the plane. |
| `line_origin` | `Eigen::Vector3d` | LINE: a point of the line. |
| `line_dir` | `Eigen::Vector3d` | LINE: line direction. |
| `line_max_distance` | `double` | LINE: maximum distance from the line. |
| `max_angle` | `Eigen::Vector3d` | ANGLE: maximum angle for each axis [rad]. |
| `max_angle_cos` | `Eigen::Vector3d` | ANGLE: cosine of max_angle (computed by the conversion functions). |

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
bool checkAngleConstraint(const Eigen::Affine3d &T_1, const Eigen::Affine3d &T_2, const Eigen::Vector3d &max_angle_cos, double &angle_cos) const
```

Checks axis-wise maximum angle constraints between two transforms.

Checks if the angle between the corresponding axes of two transformations exceeds the maximum allowed angles.

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

Convert a ROS constraint array message to a list of `GeometricConstraint`.

## pose_constraints_planner/include/pose_constraints_planner/pose_constraints_path_local_optimizer.h

Path local optimizer (warp and simplify) that preserves the pose constraints.

### class `pose_constraints_planner::PoseConstrainedPathLocalOptimizer`

Inherits from `PathLocalOptimizer`.

Derived class for local path optimization using warping and simplification techniques.

This class inherits from PathLocalOptimizer and provides

#### Public types

##### `Ptr`

```cpp
using pose_constraints_planner::PoseConstrainedPathLocalOptimizer::Ptr = std::shared_ptr<PoseConstrainedPathLocalOptimizer>
```

#### Public functions

##### `PoseConstrainedPathLocalOptimizer`

```cpp
EIGEN_MAKE_ALIGNED_OPERATOR_NEW PoseConstrainedPathLocalOptimizer(const CollisionCheckerPtr &checker, const MetricsPtr &metrics, const cnr_logger::TraceLoggerPtr &logger, const std::shared_ptr< ik_solver::IkSolver > ik_solver, const Eigen::Affine3d &T_w_b, const Eigen::Affine3d &T_f_t, const Eigen::Affine3d &T_w_start, const pose_constraints_planner::PoseConstraintsManager::Ptr &pose_constraints_manager)
```

Constructor.

**Parameters:**

- `checker`: Collision checker.
- `metrics`: Metrics.
- `logger`: Logger.
- `ik_solver`: IK solver, used for the forward kinematics.
- `T_w_b`: Pose of the robot base in the world frame.
- `T_f_t`: Pose of the tool in the flange frame.
- `T_w_start`: Tool pose of the start configuration (reference of ANGLE constraints).
- `pose_constraints_manager`: Constraints to preserve.

##### `config`

```cpp
virtual void config(const std::string &param_ns) override
```

Read the parameters `simplify_max_conn_length` (0.1), `warp_min_conn_length` (0.01) and `warp_min_step_size` (0.01), plus those of PathOptimizerBase.

**Parameters:**

- `param_ns`: Parameter namespace.

##### `simplify`

```cpp
virtual bool simplify(const double &min_conn_length=0.1)
```

Simplify the path by skipping nodes.

This method simplifies the path by removing connections shorter than a threshold. If the length of a connection is shorter than 'min_conn_length', the algorithm skips the connection's child by connecting the parent of the previous connection with the current child, if it results in a collision free connection.

**Parameters:**

- `min_conn_length`: The minimum connection's length threshold for node removal.

**Returns:** True if the path is simplified, false otherwise.

#### Protected functions

##### `bisection`

```cpp
bool bisection(const size_t &connection_idx, const Eigen::VectorXd &center, const Eigen::VectorXd &direction, const double min_step_size, double max_distance, double min_distance)
```

Move the node between connections `connection_idx-1` and `connection_idx` along `direction`, looking by bisection for the position that reduces the cost while keeping the connections collision-free and compliant with the pose constraints.

**Parameters:**

- `connection_idx`: Index of the second connection (&gt; 0).
- `center`: Starting point.
- `direction`: Search direction.
- `min_step_size`: Bisection stops when the interval is smaller than this.
- `max_distance`: Upper bound of the interval.
- `min_distance`: Lower bound of the interval.

**Returns:** True if the path was improved.

##### `checkConnectionSatisfiesPoseConstraints`

```cpp
bool checkConnectionSatisfiesPoseConstraints(const ConnectionPtr &connection)
```

Check if a connection satisfies pose constraints.

This function checks if the given connection satisfies the pose constraints defined in the `PoseConstraintsManager`. It evaluates the connection's configurations against the specified constraints and returns true if all constraints are satisfied, false otherwise.

**Parameters:**

- `connection`: The connection to be checked against pose constraints.

**Returns:** True if the connection satisfies all pose constraints, false otherwise.

##### `checkConnectionSatisfiesPoseConstraints`

```cpp
bool checkConnectionSatisfiesPoseConstraints(const NodePtr &n1, const NodePtr &n2)
```

Check if the connection between two nodes satisfies pose constraints.

This function checks if the connection between two nodes satisfies the pose constraints defined in the `PoseConstraintsManager`. It evaluates the configurations of both nodes against the specified constraints and returns true if all constraints are satisfied, false otherwise.

**Parameters:**

- `n1`: The first node of the connection.
- `n2`: The second node of the connection.

**Returns:** True if the connection between the two nodes satisfies all pose constraints, false otherwise.

##### `checkConnectionSatisfiesPoseConstraints`

```cpp
bool checkConnectionSatisfiesPoseConstraints(const Eigen::VectorXd &parent_config, const Eigen::VectorXd &child_config)
```

Check if the connection between two configurations satisfies pose constraints.

This function checks if the connection between two configurations satisfies the pose constraints defined in the `PoseConstraintsManager`. It evaluates both configurations against the specified constraints and returns true if all constraints are satisfied, false otherwise.

**Parameters:**

- `parent_config`: The configuration of the parent node.
- `child_config`: The configuration of the child node.

**Returns:** True if the connection between the two configurations satisfies all pose constraints, false otherwise.

### Free functions and types

#### `PoseConstrainedPathLocalOptimizerPtr`

```cpp
typedef std::shared_ptr<PoseConstrainedPathLocalOptimizer> pose_constraints_planner::PoseConstrainedPathLocalOptimizerPtr
```

## pose_constraints_planner/include/pose_constraints_planner/pose_constraints_planner.hpp

RRT planner whose nodes must satisfy geometric constraints on the tool pose.

### struct `pose_constraints_planner::info`

Statistics of a planning run.

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `total_time` | `double` | Total planning time [s]. |
| `total_iterations` | `int` | Number of RRT iterations. |
| `total_rejections` | `int` | Samples or extensions rejected because they violate the constraints. |
| `max_iteration_time` | `double` | Longest iteration [s]. |
| `min_iteration_time` | `double` | Shortest iteration [s]. |
| `average_iteration_time` | `double` | Average iteration time [s]. |
| `total_reject_time` | `double` | Time spent in rejected iterations [s]. |

### class `pose_constraints_planner::PoseConstraintsPlanner`

RRT planner in joint space whose nodes must satisfy geometric constraints on the tool pose.

The tool pose of each sample is computed with the forward kinematics of the IK solver and checked with a `PoseConstraintsManager` (ANGLE constraints are relative to the start pose). Samples and tree extensions that violate a constraint are rejected. With a goal bias the tree tries to connect to one of the goal configurations.

Usage: `setConstraints()`, `setStartConfiguration()`, then `setGoalConfiguration()` or `setGoalPose()` (the goal configurations are computed with the IK solver), then `solve()`.

#### Public types

##### `Ptr`

```cpp
using pose_constraints_planner::PoseConstraintsPlanner::Ptr = std::shared_ptr<PoseConstraintsPlanner>
```

define shared pointer `PoseConstraintsPlanner::Ptr`

#### Public functions

##### `PoseConstraintsPlanner`

```cpp
PoseConstraintsPlanner(rclcpp::Node::SharedPtr node, graph::core::CollisionCheckerPtr collision_checker, std::shared_ptr< ik_solver::IkSolver > ik_solver, graph::core::SamplerPtr sampler, graph::core::MetricsPtr metrics, cnr_logger::TraceLoggerPtr logger_ptr, graph::display::DisplayPtr display, std::string world_frame="world", std::string tool_frame="ur10_tool0")
```

Constructor.

Reads the world-base and flange-tool transforms from TF.

**Parameters:**

- `node`: ROS node, used for logging.
- `collision_checker`: Collision checker.
- `ik_solver`: IK solver of the robot (also used for the forward kinematics).
- `sampler`: Joint-space sampler.
- `metrics`: Metrics.
- `logger_ptr`: graph_core logger.
- `display`: Display used to show the tree and the path in RViz (may be nullptr).
- `world_frame`: Frame in which the constraints are expressed.
- `tool_frame`: Tool frame whose pose is constrained.

##### `setConstraints`

```cpp
void setConstraints(const pose_constraints_msgs::msg::GeometricConstraintArray &constraints)
```

Set geometric constraints from a GeometricConstraintArray message.

**Parameters:**

- `constraints`: The GeometricConstraintArray message containing the constraints

##### `setStartConfiguration`

```cpp
bool setStartConfiguration(const Eigen::VectorXd &start_config)
```

Set the start configuration and compute its pose.

**Parameters:**

- `start_config`: The start joint configuration

**Returns:** true if the start configuration respects the pose constraints, false otherwise

##### `setGoalConfiguration`

```cpp
bool setGoalConfiguration(const Eigen::VectorXd &goal_conf)
```

Set the goal configuration and compute its pose.

**Parameters:**

- `goal_conf`: The goal joint configuration

**Returns:** true if the goal configuration respects the pose constraints, false otherwise

##### `getConstraintsManager`

```cpp
PoseConstraintsManager::Ptr getConstraintsManager() const
```

Get Constraints Manager.

**Returns:** pointer to the `PoseConstraintsManager` instance

##### `setGoalPose`

```cpp
bool setGoalPose(const Eigen::Affine3d &goal_pose)
```

Set the goal pose and compute its IK solutions.

**Parameters:**

- `goal_pose`: The goal end-effector pose

**Returns:** true if the goal pose respects the pose constraints, false otherwise

##### `getDisplay`

```cpp
graph::display::DisplayPtr getDisplay() const
```

Get Display.

**Returns:** pointer to the Display instance

##### `setLogging`

```cpp
void setLogging(const bool &enable)
```

Enable or disable logging.

**Parameters:**

- `enable`: True to enable logging, false to disable

##### `solve`

```cpp
bool solve(const double &timeout, graph::core::PathPtr &solution, info &planning_info)
```

Solve the planning problem within a given timeout.

**Parameters:**

- `timeout`: The maximum time allowed for planning (in seconds)
- `solution`: The resulting path if a solution is found
- `planning_info`: Statistics of the planning run (times, iterations, rejections)

**Returns:** true if a solution is found, false otherwise

##### `getStartPose`

```cpp
Eigen::Affine3d getStartPose() const
```

Tool pose of the start configuration, in the world frame.

##### `getFlangeToToolPose`

```cpp
Eigen::Affine3d getFlangeToToolPose() const
```

Pose of the tool in the flange frame.

##### `getWorldToBaseTransform`

```cpp
Eigen::Affine3d getWorldToBaseTransform() const
```

Pose of the robot base in the world frame.

## Scripts

### `pose_constraints_planner/pose_constraints_planner/geometric_constraints_visualizer.py`

```
RViz visualization of the geometric pose constraints.

Subscribes to /geometry_constraints (pose_constraints_msgs/GeometricConstraintArray) and
publishes planes, lines and angle limits as markers on /geometric_constraints_markers.
```
