# Pose constrained path planning

Joint-space path planning in which the **tool pose** satisfies geometric constraints along the whole
path:

- **PLANE**: the tool stays on one side of a plane (e.g. above a table);
- **LINE**: the tool stays within a distance from a line (e.g. along a rail);
- **ANGLE**: the tool orientation stays within an angle of its start orientation, axis by axis (e.g. to
  carry a glass of water).

Each constraint can be defined in any TF frame. The planner is the multi-goal MI-RRT\* pipeline of
[`parallel_multigoal_planner`](https://github.com/JRL-CARI-CNR-UNIBS/parallel_multigoal_planner)
(alternative joint and Cartesian goals, planning scene from move_group, cancellation, refinement), with
rejection sampling on the constraints and constraint checks along every edge.

| Package | Content |
|---|---|
| [`pose_constraints_planner`](pose_constraints_planner) | library, `pose_constraints_planner_node` (action server), launch files, constraint visualizer, tests and examples |

The messages are in [`pose_constraints_msgs`](https://github.com/JRL-CARI-CNR-UNIBS/pose_constraints_msgs).

## Documentation

- [docs/TUTORIAL.md](docs/TUTORIAL.md): how it works, parameters, constraints and frames, Python and C++
  examples, tests.
- [docs/API.md](docs/API.md): API reference of the classes, functions and scripts.
- [CHANGELOG.rst](pose_constraints_planner/CHANGELOG.rst): release notes.
- Doxygen: `doxygen Doxyfile` in this folder, then open `doxygen/html/index.html`.

## Installation

Requirements: ROS 2 Jazzy, MoveIt 2 (`sudo apt install ros-jazzy-moveit`), and for the UR examples
`ros-jazzy-ur`.

Clone the dependencies in the `src` folder of a colcon workspace:

```bash
cd ~/ros2_ws/src
git clone -b modern_cmake https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader.git
git clone -b removing_cnr_logger_and_params https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git
git clone -b removing_cnr_logger_and_params https://github.com/JRL-CARI-CNR-UNIBS/mirrt_star.git
git clone -b removing_cnr_logger_and_params https://github.com/JRL-CARI-CNR-UNIBS/moveit_collision_checker.git
git clone -b removing_cnr_logger_and_params https://github.com/JRL-CARI-CNR-UNIBS/parallel_multigoal_planner.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/parallel_multigoal_planner_msgs.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/ik_solver.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/ur_ik_solver.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/pose_constraints_msgs.git
git clone -b removing_cnr_logger_and_params https://github.com/JRL-CARI-CNR-UNIBS/pose_constraint_path_planning.git
# Only for the examples and the integration test (Sharework cell)
git clone https://github.com/JRL-CARI-CNR-UNIBS/sharework_cell.git
```

Build:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If the build runs out of memory, limit the parallel jobs (`--parallel-workers 1`, or
`build: parallel-workers: 1` in `~/.colcon/defaults.yaml`).

## Usage

1. Start the robot and MoveIt, for example the Sharework cell with fake hardware:

   ```bash
   ros2 launch sharework_cembre_bringup sharework_cembre_bringup.launch.py fake_ur:=true fake_gripper:=true
   ```

2. Start the planner (ik_solver node, constraint visualizer, `pose_constraints_planner_node`):

   ```bash
   ros2 launch pose_constraints_planner pose_constraint_solver.launch.py
   ```

   Parameters: `config/solver_config.yaml` (planner) and `config/ik_params_sharework.yaml`
   (in-process ik_solver plugin, used for the forward kinematics).

3. Send a `pose_constraints_msgs/action/PlanWithConstraints` goal to `/plan_with_constraints`:

   ```bash
   ros2 run pose_constraints_planner plan_example.py
   ```

### ROS interface of `pose_constraints_planner_node`

| Interface | Type | Description |
|---|---|---|
| `/plan_with_constraints` | action `pose_constraints_msgs/PlanWithConstraints` | goal: constraints + MoveIt `MotionPlanRequest` (each `goal_constraints` element is an alternative goal); result: `MotionPlanResponse` + `PlanningInfo` statistics |
| `/geometry_constraints` | topic `pose_constraints_msgs/GeometricConstraintArray` (published) | constraints of each request, shown in RViz by `geometric_constraints_visualizer.py` |
| `/get_planning_scene`, `/compute_fk` | services (used) | planning scene from move_group |
| `<ik_solver_ns>/get_task_redundant_ik_array` | service (used) | IK of the Cartesian goals |
| TF | (used) | pose of the constraint frames in `world_frame` |

The parameters are listed in the [tutorial](docs/TUTORIAL.md#3-start-the-planner).

## Tests

```bash
# Unit tests (no ROS graph needed)
colcon test --packages-select pose_constraints_planner && colcon test-result --verbose

# Integration test: Sharework cell + ik_solver + planner + 15 scenarios, stops when done
ros2 launch pose_constraints_planner pose_constraints_planner_test.launch.py
```

The integration test checks every planned trajectory against the constraints independently of the
planner (dense interpolation, FK from `/compute_fk`). See the
[tutorial](docs/TUTORIAL.md#8-tests-and-examples).

## License

Apache License 2.0, see [LICENSE](LICENSE).
