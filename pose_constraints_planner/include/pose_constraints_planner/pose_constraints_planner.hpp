// Copyright 2026 JRL-CARI CNR-STIIMA/UNIBS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file pose_constraints_planner.hpp
 * @brief Multi-goal MI-RRT* planner whose path must satisfy geometric constraints on the tool pose.
 */
#pragma once

#include <memory>
#include <string>

#include <pluginlib/class_loader.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <parallel_multigoal_planner/multigoal_planning_pipeline.hpp>

#include <pose_constraints_msgs/msg/geometric_constraint_array.hpp>
#include <pose_constraints_msgs/msg/planning_info.hpp>

#include <pose_constraints_planner/pose_constraints_evaluator.hpp>

namespace pose_constraints_planner
{

/**
 * @brief Multi-goal planner (parallel_multigoal_planner pipeline, MI-RRT*) with geometric constraints
 * on the tool pose (PLANE, LINE, ANGLE; see GeometricConstraint).
 *
 * For each request the pipeline of parallel_multigoal_planner::MultigoalPlanningPipeline is run
 * (planning scene fetched from move_group, IK of the Cartesian goals from the ik_solver node,
 * multiple alternative goals, cancellation, refinement until the utopia threshold) with these changes:
 * - the start configuration must satisfy the constraints (ANGLE constraints are relative to its tool pose);
 * - the collision checker is wrapped by a PoseConstrainedCollisionChecker, so nodes, edges and goal
 *   candidates violating the constraints are rejected;
 * - the solver is a PoseConstrainedMultigoalSolver, whose per-goal samplers are PoseConstrainedSampler
 *   (rejection sampling on the constraints);
 * - optionally, the solution is shortened by a graph::core::PathLocalOptimizer using the constrained checker.
 *
 * The tool pose is computed in-process with the forward kinematics of an ik_solver plugin.
 *
 * Parameters (besides those of MultigoalPlanningPipeline):
 * - @c world_frame ("world"): planning frame of the constraints. Each constraint is defined in its own
 *   @c frame_id (if empty, in the @c header.frame_id of the array; if also empty, in @c world_frame)
 *   and is expressed in @c world_frame with TF at the start of each request;
 * - @c tf_timeout (1.0 s): timeout of each TF lookup;
 * - @c tool_frame (""): constrained tool frame (empty: the flange frame of the IK solver);
 * - @c ik_plugin_prefix ("ur_ik_solver"): prefix of the parameters of the in-process ik_solver plugin;
 *   the plugin is @c <prefix>.type;
 * - @c max_sampling_attempts (1000): maximum draws per constrained sample (read at every request);
 * - @c constraint_resolution (0.01): joint-space spacing of the constraint checks along an edge (read at every request);
 * - @c use_local_optimizer (false): shorten the solution with a PathLocalOptimizer (read at every request);
 *   its parameters are under @c local_optimizer. (simplify_max_conn_length, warp_min_conn_length,
 *   warp_min_step_size, max_stall_gen).
 */
class PoseConstraintsPlanner : public parallel_multigoal_planner::MultigoalPlanningPipeline
{
public:
  /** define shared pointer PoseConstraintsPlanner::Ptr */
  using Ptr = std::shared_ptr<PoseConstraintsPlanner>;

  /// Prefix of the node parameters of the local optimizer.
  static constexpr const char* kLocalOptimizerPrefix = "local_optimizer";

  /**
   * @brief Declare the parameters, load the robot model, create the clients and load the ik_solver plugin.
   * @param node Node; it must not be added to an executor yet (see MultigoalPlanningPipeline).
   * @param logger graph_core logger.
   * @throw std::runtime_error if the robot model or the ik_solver plugin cannot be loaded.
   */
  PoseConstraintsPlanner(const rclcpp::Node::SharedPtr& node, const graph::core::LoggerPtr& logger);

  /**
   * @brief Plan a motion that satisfies the geometric constraints.
   * @param constraints Geometric constraints, each in its frame (see the class description).
   * @param req Motion plan request; goal_constraints are alternative goals (Cartesian XOR joint-space each).
   * @param verbose Print a summary of the goal set and of the solution.
   * @param feedback Feedback callback (may be empty).
   * @param is_canceling Cancellation callback (may be empty).
   * @param[out] info Planning statistics.
   * @return The outcome; on failure the response carries the error code
   * (START_STATE_VIOLATES_PATH_CONSTRAINTS if the start violates the constraints, FRAME_TRANSFORM_FAILURE
   * if a frame of the constraints is not available).
   */
  parallel_multigoal_planner::PlanningResult
  planWithConstraints(const pose_constraints_msgs::msg::GeometricConstraintArray& constraints,
                      const moveit_msgs::msg::MotionPlanRequest& req, bool verbose, const FeedbackFcn& feedback,
                      const CancelFcn& is_canceling, pose_constraints_msgs::msg::PlanningInfo& info) const;

  /** Frame of the constraints. */
  const std::string& worldFrame() const { return world_frame_; }

protected:
  /**
   * @brief Pose of @p frame in @c world_frame from TF.
   * @param frame Frame.
   * @param[out] T_world_frame Pose.
   * @param[out] error Error message.
   * @return False if the transform is not available within @c tf_timeout.
   */
  bool lookupWorldTransform(const std::string& frame, Eigen::Affine3d& T_world_frame, std::string& error) const;

  std::string world_frame_;  ///< Frame of the constraints.
  std::string tool_frame_;  ///< Constrained tool frame.
  double tf_timeout_;  ///< TF lookup timeout [s].
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  ///< TF buffer of the IK solver.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  ///< TF listener.
  std::unique_ptr<pluginlib::ClassLoader<ik_solver::IkSolver>> ik_loader_;  ///< Must outlive ik_solver_.
  std::shared_ptr<ik_solver::IkSolver> ik_solver_;  ///< In-process IK solver, used for the forward kinematics.
};

}  // namespace pose_constraints_planner
