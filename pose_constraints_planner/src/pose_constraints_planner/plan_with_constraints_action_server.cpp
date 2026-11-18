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

#include <pose_constraints_planner/plan_with_constraints_action_server.hpp>

#include <thread>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <pose_constraints_planner/graph_core_ros.hpp>

namespace pose_constraints_planner
{

PlanWithConstraintsActionServer::PlanWithConstraintsActionServer(const rclcpp::Node::SharedPtr& node,
                                                                 const std::string& action_name)
  : node_(node)
{
  // graph_core / mirrt* messages go to the child logger "<node>.graph_core".
  graph::core::LoggerPtr logger = std::make_shared<RclcppGraphLogger>(node_->get_logger().get_child("graph_core"));
  planner_ = std::make_unique<PoseConstraintsPlanner>(node_, logger);

  constraints_pub_ = node_->create_publisher<pose_constraints_msgs::msg::GeometricConstraintArray>(
      "/geometry_constraints", rclcpp::QoS(10));

  using std::placeholders::_1;
  using std::placeholders::_2;
  server_ = rclcpp_action::create_server<Action>(
      node_, action_name, std::bind(&PlanWithConstraintsActionServer::handle_goal, this, _1, _2),
      std::bind(&PlanWithConstraintsActionServer::handle_cancel, this, _1),
      std::bind(&PlanWithConstraintsActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(node_->get_logger(), "Action server '%s' ready (world_frame='%s', ik_solver_ns='%s').",
              action_name.c_str(), planner_->worldFrame().c_str(),
              node_->get_parameter("ik_solver_ns").as_string().c_str());
}

rclcpp_action::GoalResponse PlanWithConstraintsActionServer::handle_goal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                                         std::shared_ptr<const Action::Goal> goal)
{
  if (!goal)
    return rclcpp_action::GoalResponse::REJECT;

  if (goal->motion_plan_request.group_name.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected: motion_plan_request.group_name is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->motion_plan_request.goal_constraints.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected: goal_constraints is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanWithConstraintsActionServer::handle_cancel(const std::shared_ptr<GoalHandle> /*gh*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanWithConstraintsActionServer::handle_accepted(const std::shared_ptr<GoalHandle> gh)
{
  std::thread{ std::bind(&PlanWithConstraintsActionServer::execute, this, gh) }.detach();
}

void PlanWithConstraintsActionServer::execute(const std::shared_ptr<GoalHandle> gh)
{
  const auto goal = gh->get_goal();
  auto feedback = std::make_shared<Action::Feedback>();
  auto result = std::make_shared<Action::Result>();

  constraints_pub_->publish(goal->constraints);

  const parallel_multigoal_planner::PlanningResult outcome = planner_->planWithConstraints(
      goal->constraints, goal->motion_plan_request, goal->verbose,
      [&](const std::string& status, float progress)
      {
        feedback->status = status;
        feedback->progress = progress;
        gh->publish_feedback(feedback);
      },
      [&]() { return gh->is_canceling(); }, result->info);

  result->motion_plan_response = outcome.response;
  if (outcome.canceled)
  {
    gh->canceled(result);
    return;
  }
  if (outcome.response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    gh->abort(result);
    return;
  }

  feedback->status = "Done";
  feedback->progress = 1.0f;
  gh->publish_feedback(feedback);
  gh->succeed(result);
}

}  // namespace pose_constraints_planner
