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

// pose_constraints_planner_node: action server /plan_with_constraints (pose_constraints_msgs/action/PlanWithConstraints).
//
// Multi-goal MI-RRT* planner (parallel_multigoal_planner pipeline) whose path satisfies geometric
// constraints on the tool pose. See PoseConstraintsPlanner for the parameters.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

#include <pose_constraints_planner/plan_with_constraints_action_server.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("pose_constraints_planner");

  auto action_server = std::make_shared<pose_constraints_planner::PlanWithConstraintsActionServer>(node);

  // Humble's rclcpp_action server is not safe under a MultiThreadedExecutor (data race in the
  // action server waitable, fixed in later releases): it segfaults when a goal completes.
  // All blocking work runs in the detached execute() thread, so a single executor thread is enough.
#if RCLCPP_VERSION_MAJOR >= 28
  rclcpp::executors::MultiThreadedExecutor executor;
#else
  rclcpp::executors::SingleThreadedExecutor executor;
#endif
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "pose_constraints_planner node spinning.");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
