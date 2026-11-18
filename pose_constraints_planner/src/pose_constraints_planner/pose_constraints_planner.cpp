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

#include <pose_constraints_planner/pose_constraints_planner.hpp>

#include <sstream>
#include <stdexcept>

#include <ik_solver/ros_params.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <pose_constraints_planner/graph_core_ros.hpp>
#include <pose_constraints_planner/pose_constrained_collision_checker.hpp>
#include <pose_constraints_planner/pose_constrained_multigoal_solver.hpp>

namespace pose_constraints_planner
{

PoseConstraintsPlanner::PoseConstraintsPlanner(const rclcpp::Node::SharedPtr& node,
                                               const graph::core::LoggerPtr& logger)
  : MultigoalPlanningPipeline(node, logger)
{
  world_frame_ = node_->declare_parameter<std::string>("world_frame", "world");
  tool_frame_ = node_->declare_parameter<std::string>("tool_frame", "");
  tf_timeout_ = node_->declare_parameter<double>("tf_timeout", 1.0);
  const std::string ik_prefix = node_->declare_parameter<std::string>("ik_plugin_prefix", "ur_ik_solver");
  // Read at every request
  node_->declare_parameter<int>("max_sampling_attempts", 1000);
  node_->declare_parameter<double>("constraint_resolution", 0.01);
  node_->declare_parameter<bool>("use_local_optimizer", false);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // In-process ik_solver plugin, used only for the forward kinematics of the constraint checks.
  ik_solver::ParamMap ik_params = ik_solver::paramsFromNode(node_, ik_prefix);
  const std::string plugin =
      node_->get_parameter_or<std::string>(ik_prefix + ".type", std::string("ik_solver/Ur10eIkSolver"));
  ik_params["robot_description"] = robot_model_loader_->getRDFLoader()->getURDFString();

  ik_loader_ = std::make_unique<pluginlib::ClassLoader<ik_solver::IkSolver>>("ik_solver", "ik_solver::IkSolver");
  ik_solver_ = ik_loader_->createSharedInstance(plugin);
  ik_solver_->setBuffer(tf_buffer_);
  ik_solver_->setLogger(std::make_shared<ik_solver::RclcppLogger>(node_->get_logger().get_child("ik_solver")));
  if (!ik_solver_->config(ik_params))
    throw std::runtime_error("PoseConstraintsPlanner: unable to configure the ik_solver plugin '" + plugin +
                             "' with the parameters under '" + ik_prefix + ".'");
  if (tool_frame_.empty())
    tool_frame_ = ik_solver_->flange_frame();

  RCLCPP_INFO(node_->get_logger(), "PoseConstraintsPlanner: ik plugin '%s', world='%s', base='%s', flange='%s', tool='%s'",
              plugin.c_str(), world_frame_.c_str(), ik_solver_->base_frame().c_str(),
              ik_solver_->flange_frame().c_str(), tool_frame_.c_str());
}

bool PoseConstraintsPlanner::lookupWorldTransform(const std::string& frame, Eigen::Affine3d& T_world_frame,
                                                  std::string& error) const
{
  if (frame == world_frame_)
  {
    T_world_frame.setIdentity();
    return true;
  }
  try
  {
    const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
        world_frame_, frame, tf2::TimePointZero, tf2::durationFromSec(std::max(0.0, tf_timeout_)));
    T_world_frame = tf2::transformToEigen(tf);
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    error = "cannot transform from '" + frame + "' to '" + world_frame_ + "': " + ex.what();
    return false;
  }
}

parallel_multigoal_planner::PlanningResult PoseConstraintsPlanner::planWithConstraints(
    const pose_constraints_msgs::msg::GeometricConstraintArray& constraints,
    const moveit_msgs::msg::MotionPlanRequest& req, bool verbose, const FeedbackFcn& feedback,
    const CancelFcn& is_canceling, pose_constraints_msgs::msg::PlanningInfo& info) const
{
  using MoveItErrorCodes = moveit_msgs::msg::MoveItErrorCodes;

  // Per-request state shared by the hooks (requests may run concurrently).
  struct RequestState
  {
    PoseConstraintsEvaluatorPtr evaluator;
    std::shared_ptr<PoseConstrainedCollisionChecker> checker;
    PoseConstrainedMultigoalSolverPtr solver;
  };
  auto state = std::make_shared<RequestState>();

  const unsigned int max_sampling_attempts =
      static_cast<unsigned int>(std::max<int64_t>(1, node_->get_parameter("max_sampling_attempts").as_int()));
  const double constraint_resolution = node_->get_parameter("constraint_resolution").as_double();
  const bool use_local_optimizer = node_->get_parameter("use_local_optimizer").as_bool();

  parallel_multigoal_planner::PlanningHooks hooks;

  hooks.on_start = [&, state](const parallel_multigoal_planner::PlanningContext& ctx, int32_t& error_code,
                              std::string& error)
  {
    Eigen::Affine3d T_w_b, T_f_t;
    if (!lookupWorldTransform(ik_solver_->base_frame(), T_w_b, error) ||
        !ik_solver_->getTF(ik_solver_->flange_frame(), tool_frame_, T_f_t))
    {
      error_code = MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
      if (error.empty())
        error = "cannot read the transform " + ik_solver_->flange_frame() + "->" + tool_frame_;
      return false;
    }

    // Express every constraint in the world frame: frame_id, else the array header frame, else world_frame.
    auto manager = std::make_shared<PoseConstraintsManager>();
    for (const auto& msg : constraints.constraints)
    {
      const std::string frame = !msg.frame_id.empty()                 ? msg.frame_id :
                                !constraints.header.frame_id.empty() ? constraints.header.frame_id :
                                                                       world_frame_;
      Eigen::Affine3d T_world_frame;
      if (!lookupWorldTransform(frame, T_world_frame, error))
      {
        error_code = MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
        error = "constraint '" + msg.name + "': " + error;
        return false;
      }
      GeometricConstraint c = convertGeometricConstraint(msg);
      c.frame_id = frame;
      manager->addConstraint(transformConstraint(c, T_world_frame));
    }
    try
    {
      state->evaluator =
          std::make_shared<PoseConstraintsEvaluator>(ik_solver_, manager, ctx.joint_names, T_w_b, T_f_t, ctx.start_q);
    }
    catch (const std::invalid_argument& ex)
    {
      error_code = MoveItErrorCodes::INVALID_GROUP_NAME;
      error = ex.what();
      return false;
    }

    std::stringstream report;
    if (!state->evaluator->satisfies(ctx.start_q, &report))
    {
      error_code = MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS;
      error = "start state violates the pose constraints:\n" + report.str();
      return false;
    }
    if (ctx.verbose)
      RCLCPP_INFO(node_->get_logger(), "%zu pose constraint(s) on '%s' in '%s'.", manager->constraints().size(),
                  tool_frame_.c_str(), world_frame_.c_str());
    return true;
  };

  hooks.make_checker = [&, state](const parallel_multigoal_planner::PlanningContext& ctx,
                                  const graph::core::CollisionCheckerPtr& base) -> graph::core::CollisionCheckerPtr
  {
    state->checker = std::make_shared<PoseConstrainedCollisionChecker>(base, state->evaluator, constraint_resolution,
                                                                      ctx.logger);
    return state->checker;
  };

  hooks.make_solver = [&, state](const parallel_multigoal_planner::PlanningContext& ctx,
                                 const graph::core::MetricsPtr& metrics, const graph::core::CollisionCheckerPtr& checker,
                                 const graph::core::SamplerPtr& sampler,
                                 const graph::core::GoalCostFunctionPtr& goal_cost_fcn)
      -> std::shared_ptr<graph::mirrt_star::MultigoalSolver>
  {
    state->solver = std::make_shared<PoseConstrainedMultigoalSolver>(
        metrics, checker, sampler, goal_cost_fcn, state->evaluator, max_sampling_attempts, ctx.logger);
    return state->solver;
  };

  if (use_local_optimizer)
  {
    hooks.post_process = [&](const parallel_multigoal_planner::PlanningContext& ctx,
                             const graph::core::CollisionCheckerPtr& checker, const graph::core::MetricsPtr& metrics,
                             const graph::core::PathPtr& solution) -> graph::core::PathPtr
    {
      // The constrained checker makes the optimizer keep the constraints along the shortened path.
      auto optimizer = std::make_shared<graph::core::PathLocalOptimizer>(checker, metrics, ctx.logger);
      optimizer->config(graphParamsFromNode(node_, kLocalOptimizerPrefix));
      const double cost_before = solution->cost();
      optimizer->setPath(solution);
      optimizer->solve();
      graph::core::PathPtr optimized = optimizer->getPath();
      if (ctx.verbose && optimized)
        RCLCPP_INFO(node_->get_logger(), "Local optimizer: cost %.4f -> %.4f.", cost_before, optimized->cost());
      return optimized;
    };
  }

  parallel_multigoal_planner::PlanningResult result = plan(req, verbose, feedback, is_canceling, hooks);

  info = pose_constraints_msgs::msg::PlanningInfo();
  info.total_time = result.planning_time;
  info.total_iterations = result.iterations;
  info.max_iteration_time = result.max_iteration_time;
  info.min_iteration_time = result.min_iteration_time;
  info.average_iteration_time = result.average_iteration_time;
  if (state->solver)
  {
    const ConstrainedSamplingStatistics stats = state->solver->samplingStatistics();
    info.rejection_rate = stats.rejectionRate();
    info.overall_sampling_time = stats.sampling_time;
    info.total_reject_time = stats.reject_time;
    info.total_rejections = static_cast<uint32_t>(stats.rejections);
  }
  if (state->checker)
    info.total_rejections += static_cast<uint32_t>(state->checker->getConstraintRejections());

  if (verbose)
    RCLCPP_INFO(node_->get_logger(),
                "Planning info: %u iterations, %u constraint rejections, rejection rate %.3f, sampling time %.3f s.",
                info.total_iterations, info.total_rejections, info.rejection_rate, info.overall_sampling_time);
  return result;
}

}  // namespace pose_constraints_planner
