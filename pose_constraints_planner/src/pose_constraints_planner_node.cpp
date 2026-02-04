// pose_constraints_planner_node.cpp
//
// “Convenient” single-file node with BOTH requested changes:
// 1) ONE action server: "/plan_with_constraints".
//    It selects the correct planner at runtime using motion_plan_request.group_name.
// 2) Goal pose is transformed to the planner world frame (e.g., "world") using TF.
//
// Notes / assumptions:
// - Uses PoseConstraintsPlanner API you provided:
//     setConstraints(msg), setStartConfiguration(q), setGoalConfiguration(q), setGoalPose(Affine3d),
//     setLogging(bool), solve(timeout, solution, planning_info)
// - Timeout is taken from motion_plan_request.allowed_planning_time (fallback 5.0).
// - Goal parsing:
//     * Joint goal if goal_constraints[0].joint_constraints not empty
//     * Pose goal if goal_constraints[0] has position_constraints + orientation_constraints
//       Pose is built from position_constraints[0].constraint_region.primitive_poses[0].position
//       and orientation_constraints[0].orientation, then TF-transformed into world_frame.
// - MotionPlanResponse trajectory is filled as JointTrajectory with 1s spacing between points,
//   using start + solution->getWaypoints() (as in your snippet).
//
// You get:
// - planners_map[group_name] = planner
// - joint_names_map[group_name] = active joint names (stable order for that group)
// - a single action server instance kept alive

#include <ament_index_cpp/get_package_share_directory.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

// TF
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// MoveIt
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>


#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Graph core + plugins
#include <graph_core/plugins/samplers/sampler_base_plugin.h>
#include <graph_core/plugins/metrics/metrics_base_plugin.h>
#include <graph_core/graph/path.h>

// Collision checker plugin
#include <moveit_collision_checker/plugins/collision_checkers/moveit_collision_checker_base_plugin.h>

// Display (optional)
#include <graph_display/graph_display.h>

// Class loader
#include <cnr_class_loader/multi_library_class_loader.hpp>

// IK solver
#include "ik_solver/internal/ik_solver_node.ros2.hpp"


// Planner
#include <pose_constraints_planner/pose_constraints_planner.hpp>
#include <pose_constraints_planner/plan_with_constraints_action_server.hpp>

// Action + msgs
#include <pose_constraints_msgs/action/plan_with_constraints.hpp>
#include <pose_constraints_msgs/msg/geometric_constraint_array.hpp>
#include <pose_constraints_msgs/msg/planning_info.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// -------------------------------------------------------------------------------------------------
// RobotDescriptionNode: reads /robot_description topic (std_msgs/String, transient_local)
// -------------------------------------------------------------------------------------------------
class RobotDescriptionNode : public rclcpp::Node
{
public:
  RobotDescriptionNode() : Node("robot_description_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&RobotDescriptionNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Started RobotDescriptionNode, waiting for /robot_description ...");
  }

  bool isRobotDescriptionReceived() const { return !robot_description_.empty(); }
  std::string getRobotDescription() const { return robot_description_; }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    robot_description_ = msg->data;
    const std::string truncated = robot_description_.substr(0, 200);
    RCLCPP_INFO(this->get_logger(), "Received robot description (first 200 chars):\n%s", truncated.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string robot_description_;
};

static std::string wait_for_robot_description(
    rclcpp::executors::MultiThreadedExecutor& executor,
    const std::shared_ptr<RobotDescriptionNode>& rd_node,
    const std::chrono::seconds& timeout)
{
  const auto t0 = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !rd_node->isRobotDescriptionReceived())
  {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    if (std::chrono::steady_clock::now() - t0 > timeout)
    {
      RCLCPP_ERROR(rd_node->get_logger(), "Timeout waiting for /robot_description.");
      return {};
    }
  }

  return rd_node->getRobotDescription();
}


// -------------------------------------------------------------------------------------------------
// MAIN
// -------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // One executor for everything
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 10);

  // 1) Wait for /robot_description
  auto rd_node = std::make_shared<RobotDescriptionNode>();
  executor.add_node(rd_node);

  const std::string robot_description = wait_for_robot_description(executor, rd_node, std::chrono::seconds(30));
  if (robot_description.empty())
    return 1;

  // 2) Main node
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("pose_constraints_planner", options);
  executor.add_node(node);

  // Ensure robot_description exists as a parameter on THIS node (RobotModelLoader reads parameters)
  node->declare_parameter<std::string>("robot_description", robot_description);

  // TF buffer/listener (used to transform goal pose)
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  (void)tf_listener;

  // 3) Logger
  const std::string package_name = "pose_constraints_planner";
  const std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
  if (package_path.empty())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to get path for package '" << package_name << "'");
    return 1;
  }

  const std::string logger_file = package_path + "/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger =
      std::make_shared<cnr_logger::TraceLogger>("pose_constraints_planner", logger_file);

  // Namespaces used by your existing params
  const std::string param_ns1 = "/" + package_name;
  const std::string param_ns2 = param_ns1 + "/solver_config";

  // 4) Robot model + planning scene
  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  if (!kinematic_model)
  {
    RCLCPP_ERROR(node->get_logger(), "RobotModelLoader returned null model");
    return 1;
  }

  planning_scene::PlanningScenePtr planning_scene =
      std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  // 5) Get planning scene from /get_planning_scene
  auto ps_client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!ps_client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(), "Unable to connect to /get_planning_scene");
    return 1;
  }

  auto ps_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  auto ps_future = ps_client->async_send_request(ps_req);

  if (executor.spin_until_future_complete(ps_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Call to /get_planning_scene failed");
    return 1;
  }

  if (!planning_scene->setPlanningSceneMsg(ps_future.get()->scene))
  {
    RCLCPP_ERROR(node->get_logger(), "Unable to update planning scene");
    return 1;
  }

  // 6) Plugin loader: load libraries
  cnr_class_loader::MultiLibraryClassLoader loader(false);
  std::vector<std::string> libraries;
  if (!graph::core::get_param(logger, param_ns2, "libraries", libraries))
    return 1;

  for (const auto& lib : libraries)
    loader.loadLibrary(lib);

  // IK solver loader
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader("ik_solver", "ik_solver::IkSolver");

  // 7) Build planners_map + joint_names_map
  std::map<std::string, pose_constraints_planner::PoseConstraintsPlanner::Ptr> planners_map;
  std::map<std::string, std::vector<std::string>> joint_names_map;
  std::map<std::string, pose_constraints_planner::PoseConstrainedPathLocalOptimizerPtr> local_optimizers_map;

  const std::vector<std::string> group_names = kinematic_model->getJointModelGroupNames();
  const std::string world_frame = "world";
  const std::string tool_frame  = "ur10e_tool0";  // align with your constructor usage

  for (const auto& group_name : group_names)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Configuring group: " << group_name);

    const moveit::core::JointModelGroup* jmg = kinematic_model->getJointModelGroup(group_name);
    if (!jmg)
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "Skipping group '" << group_name << "' (JointModelGroup not found)");
      continue;
    }

    const std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
    const unsigned int dof = static_cast<unsigned int>(joint_names.size());
    if (dof == 0)
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "Skipping group '" << group_name << "' (dof=0)");
      continue;
    }
    joint_names_map[group_name] = joint_names;

    // Bounds for sampler
    Eigen::VectorXd lb(dof), ub(dof);
    for (unsigned int i = 0; i < dof; ++i)
    {
      const auto& bounds = kinematic_model->getVariableBounds(joint_names.at(i));
      if (bounds.position_bounded_)
      {
        lb(i) = bounds.min_position_;
        ub(i) = bounds.max_position_;
      }
      else
      {
        lb(i) = -1e9;
        ub(i) =  1e9;
      }
    }

    // Collision checker plugin
    std::string checker_plugin_name;
    graph::core::get_param(logger, param_ns2, "checker_plugin", checker_plugin_name,
                          std::string("graph::ros1::ParallelMoveitCollisionCheckerPlugin"));

    RCLCPP_INFO_STREAM(node->get_logger(), "Loading checker: " << checker_plugin_name);
    auto checker_plugin =
        loader.createInstance<graph::collision_check::MoveitCollisionCheckerBasePlugin>(checker_plugin_name);
    checker_plugin->init(param_ns2, planning_scene, logger);
    graph::core::CollisionCheckerPtr checker = checker_plugin->getCollisionChecker();

    // Sampler plugin
    std::string sampler_plugin_name;
    graph::core::get_param(logger, param_ns2, "sampler_plugin", sampler_plugin_name,
                          std::string("graph::core::InformedSamplerPlugin"));

    RCLCPP_INFO_STREAM(node->get_logger(), "Loading sampler: " << sampler_plugin_name);
    auto sampler_plugin = loader.createInstance<graph::core::SamplerBasePlugin>(sampler_plugin_name);

    Eigen::VectorXd scale(dof);
    scale.setOnes();
    sampler_plugin->init(param_ns2, lb, ub, lb, ub, scale, logger);
    graph::core::SamplerPtr sampler = sampler_plugin->getSampler();

    // Metrics plugin
    std::string metrics_plugin_name;
    graph::core::get_param(logger, param_ns2, "metrics_plugin", metrics_plugin_name,
                          std::string("graph::core::EuclideanMetricsPlugin"));

    RCLCPP_INFO_STREAM(node->get_logger(), "Loading metrics: " << metrics_plugin_name);
    auto metrics_plugin = loader.createInstance<graph::core::MetricsBasePlugin>(metrics_plugin_name);
    metrics_plugin->init(param_ns2, logger);
    graph::core::MetricsPtr metrics = metrics_plugin->getMetrics();

    // IK solver
    auto ik_solver = ik_loader.createSharedInstance("ik_solver/Ur10eIkSolver");
    ik_solver->setBuffer(tf_buffer);

    const std::string ik_plugin_name = "/ur_ik_solver";
    std::string param_what;

    if (!cnr::param::set(ik_plugin_name + std::string("/robot_description"), robot_description, param_what))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Cannot set cnr::param(" << ik_plugin_name << "/robot_description) because: " << param_what);
    }

    if (!ik_solver->config(ik_plugin_name))
    {
      RCLCPP_ERROR(node->get_logger(), "Unable to configure ik_solver at ns '%s'", ik_plugin_name.c_str());
      return 1;
    }

    // Display (optional)
    auto display = std::make_shared<graph::display::Display>(node, planning_scene, group_name, tool_frame);
    
    // Build planner
    RCLCPP_INFO_STREAM(node->get_logger(), "Creating PoseConstraintsPlanner for group '" << group_name << "'");
    auto planner = std::make_shared<pose_constraints_planner::PoseConstraintsPlanner>(
        node, checker, ik_solver, sampler, metrics, logger, display, world_frame, tool_frame);



	// Build local optimizer
	pose_constraints_planner::PoseConstraintsManager::Ptr pcm = planner->getConstraintsManager();
	pose_constraints_planner::PoseConstrainedPathLocalOptimizer::Ptr local_optimizer;

	bool use_local_optimizer = false;
	graph::core::get_param(logger, param_ns2, "use_local_optimizer", use_local_optimizer, false);

	if (use_local_optimizer)
	{
  		local_optimizer =
        std::make_shared<pose_constraints_planner::PoseConstrainedPathLocalOptimizer>(
            checker, metrics, logger, ik_solver,
            planner->getWorldToBaseTransform(), planner->getFlangeToToolPose(), planner->getStartPose(), pcm);
		local_optimizer->config(param_ns2);
	}


    planners_map[group_name] = planner;
	local_optimizers_map[group_name] = local_optimizer;
  }

  // 8) Create ONE action server: "/plan_with_constraints"
  auto mux_server = std::make_shared<pose_constraints_planner::PlanWithConstraintsMuxActionServer>(
      node,
      "/plan_with_constraints",
      world_frame,
      tf_buffer,
      planners_map,
      joint_names_map,
	  local_optimizers_map);

  RCLCPP_INFO(node->get_logger(), "Ready. Spinning...");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
