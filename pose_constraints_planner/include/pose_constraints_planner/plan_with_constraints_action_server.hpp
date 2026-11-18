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
 * @file plan_with_constraints_action_server.hpp
 * @brief Action server of pose_constraints_msgs/action/PlanWithConstraints.
 */
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <pose_constraints_msgs/action/plan_with_constraints.hpp>
#include <pose_constraints_msgs/msg/geometric_constraint_array.hpp>

#include <pose_constraints_planner/pose_constraints_planner.hpp>

namespace pose_constraints_planner
{

/**
 * @brief Action server of pose_constraints_msgs/action/PlanWithConstraints.
 *
 * Each accepted goal is executed on its own detached thread by PoseConstraintsPlanner::planWithConstraints():
 * motion_plan_request.goal_constraints are alternative goals (multi-goal MI-RRT*), the path satisfies
 * the geometric constraints of the goal, and the result carries the MotionPlanResponse and the
 * PlanningInfo statistics. Cancellation is checked at every planner iteration. The constraints of
 * each goal are also published on @c /geometry_constraints for visualization.
 */
class PlanWithConstraintsActionServer
{
public:
  using Action = pose_constraints_msgs::action::PlanWithConstraints;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  /**
   * @brief Create the planner and start the action server.
   * @param node Node; it must not be added to an executor yet (the robot model is loaded here).
   * @param action_name Action name.
   * @throw std::runtime_error if the planner cannot be created.
   */
  PlanWithConstraintsActionServer(const rclcpp::Node::SharedPtr& node,
                                  const std::string& action_name = "/plan_with_constraints");

private:
  /** Accept every goal that has a group name and at least one goal constraint. */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const Action::Goal> goal);
  /** Accept cancellation; it is checked at every planner iteration. */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> gh);
  /** Run execute() in a detached thread. */
  void handle_accepted(const std::shared_ptr<GoalHandle> gh);
  /** Plan for one goal and report the result. */
  void execute(const std::shared_ptr<GoalHandle> gh);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<PoseConstraintsPlanner> planner_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Publisher<pose_constraints_msgs::msg::GeometricConstraintArray>::SharedPtr constraints_pub_;
};

}  // namespace pose_constraints_planner
