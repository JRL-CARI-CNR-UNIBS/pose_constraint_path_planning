^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pose_constraints_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-11-18)
------------------
* The planner is built on the multi-goal MI-RRT* pipeline of parallel_multigoal_planner
  (``MultigoalPlanningPipeline``): alternative joint and Cartesian goals, planning scene from
  move_group, IK of the Cartesian goals from the ik_solver node, cancellation, refinement until the
  utopia threshold.
* New ``PoseConstrainedSampler``: graph_core sampler that draws only configurations satisfying the
  constraints (rejection sampling), with statistics (attempts, rejections, times, acceptance rate).
* New ``PoseConstrainedCollisionChecker``: checks the constraints along every edge
  (``constraint_resolution``), so tree extensions, rewiring, goal connections and the local optimizer
  keep the constraints on the whole path.
* New ``PoseConstrainedMultigoalSolver``: MI-RRT* whose per-goal informed and tube samplers are
  constrained.
* New ``PoseConstraintsEvaluator``: forward kinematics of an ik_solver plugin and constraint check,
  with joint reordering between the planning group and the IK solver.
* Constraint frames: each constraint is defined in its ``frame_id`` (else the array
  ``header.frame_id``, else ``world_frame``) and expressed in ``world_frame`` with TF
  (``transformConstraint``, parameter ``tf_timeout``). ``FRAME_TRANSFORM_FAILURE`` if a frame is not
  available.
* ANGLE constraints: a negative ``max_angle`` leaves the axis free, as documented in the message.
* Goal candidates violating the constraints are discarded (``INVALID_GOAL_CONSTRAINTS`` /
  ``NO_IK_SOLUTION`` if none is left); ``START_STATE_VIOLATES_PATH_CONSTRAINTS`` if the start
  violates them.
* ROS 2 parameters and rclcpp logging (cnr_param and cnr_logger removed).
* Removed the old test node, ``PoseConstrainedPathLocalOptimizer`` (replaced by graph_core's
  ``PathLocalOptimizer`` with the constrained checker), the graph_display, cnr_scene_manager and
  cnr_tf_named_object_loader dependencies, and the old test configurations.
* Tests: gtest unit tests (``test/test_pose_constraints.cpp``) and an integration test with the
  Sharework cell (``launch/pose_constraints_planner_test.launch.py``, 15 scenarios, trajectories
  checked against the constraints independently of the planner).
* Example client ``scripts/plan_example.py``.
* Documentation: tutorial, API reference, Doxygen comments.
* License changed to Apache-2.0.

0.1.0
-----
* Orientation, plane and line constraints.
* Constraint checks on start and goal nodes, and after the tree extension.
* Performance monitoring.
* Constraint visualization in RViz.
* Automatic constraint parsing.
* Python launch files.
* Action client to simulate the trajectory.
* Automated testing with results logged to CSV.
