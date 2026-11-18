# Documentation

| Document | Content |
|---|---|
| [TUTORIAL.md](TUTORIAL.md) | How the planner works, starting it, parameters, constraint types and frames, Python and C++ clients, unit and integration tests |
| [API.md](API.md) | API reference of the C++ classes and functions, scripts and tests |
| [../README.md](../README.md) | Installation, usage, ROS interface |
| [../pose_constraints_planner/CHANGELOG.rst](../pose_constraints_planner/CHANGELOG.rst) | Release notes |

HTML API documentation: run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

## Quick start

```bash
# terminal 1: Sharework cell with fake hardware
ros2 launch sharework_cembre_bringup sharework_cembre_bringup.launch.py fake_ur:=true fake_gripper:=true
# terminal 2: planner
ros2 launch pose_constraints_planner pose_constraint_solver.launch.py
# terminal 3: example request
ros2 run pose_constraints_planner plan_example.py
```

Or everything at once, with the checks:

```bash
ros2 launch pose_constraints_planner pose_constraints_planner_test.launch.py
```
