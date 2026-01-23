#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <Eigen/Geometry>

#include <graph_core/graph/path.h>
#include <pose_constraints_planner/pose_constraints_planner.hpp>

#include <pose_constraints_msgs/action/plan_with_constraints.hpp>
#include <pose_constraints_msgs/msg/geometric_constraint_array.hpp>
#include <pose_constraints_msgs/msg/planning_info.hpp>


#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace pose_constraints_planner
{


// -------------------------------------------------------------------------------------------------
// Single Action Server: selects planner by request.group_name at runtime
// -------------------------------------------------------------------------------------------------

/**
 * @brief Multiplexing action server that dispatches planning requests to a set of PoseConstraintsPlanner
 * implementations keyed by group name.
 *
 * This lightweight server exposes a ROS2 action of type pose_constraints_msgs::action::PlanWithConstraints
 * and selects the appropriate planner at runtime based on the incoming request's MotionPlanRequest.group_name.
 *
 * Threading model:
 * - The action server accepts goals and executes each accepted goal on a detached std::thread.
 * - Internally planners are treated as stateful; a single mutex (planner_mtx_) is used to serialize access
 *   to planner instances to avoid concurrent state corruption.
 */
class PlanWithConstraintsMuxActionServer
{
public:
  using Action = pose_constraints_msgs::action::PlanWithConstraints;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  /**
   * @brief Construct a new PlanWithConstraintsMuxActionServer
   *
   * @param node Shared pointer to the rclcpp::Node used for logging and creating the action server
   * @param action_name Name of the action to create (e.g. "plan_with_constraints")
   * @param world_frame The world TF frame to which pose goals will be transformed
   * @param tf_buffer Shared pointer to the tf2_ros::Buffer used for transforms
   * @param planners_map Map from group_name to PoseConstraintsPlanner::Ptr. Owned externally; kept by reference
   * @param joint_names_map Map from group_name to the ordered list of joint names for that group. Owned externally; kept by reference
   *
   * @throws std::runtime_error if node or tf_buffer is null
   */
  PlanWithConstraintsMuxActionServer(
      const rclcpp::Node::SharedPtr& node,
      const std::string& action_name,
      const std::string& world_frame,
      const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
      const std::map<std::string, pose_constraints_planner::PoseConstraintsPlanner::Ptr>& planners_map,
      const std::map<std::string, std::vector<std::string>>& joint_names_map)
  : node_(node),
    action_name_(action_name),
    world_frame_(world_frame),
    tf_buffer_(tf_buffer),
    planners_map_(planners_map),
    joint_names_map_(joint_names_map)
  {
    if (!node_) throw std::runtime_error("MuxActionServer: node is null");
    if (!tf_buffer_) throw std::runtime_error("MuxActionServer: tf_buffer is null");

    using std::placeholders::_1;
    using std::placeholders::_2;

    server_ = rclcpp_action::create_server<Action>(
        node_,
        action_name_,
        std::bind(&PlanWithConstraintsMuxActionServer::handle_goal, this, _1, _2),
        std::bind(&PlanWithConstraintsMuxActionServer::handle_cancel, this, _1),
        std::bind(&PlanWithConstraintsMuxActionServer::handle_accepted, this, _1));

    // Publisher for geometric constraints
    geom_constraints_pub_ = node_->create_publisher<pose_constraints_msgs::msg::GeometricConstraintArray>("/geometry_constraints", rclcpp::QoS(10));

    RCLCPP_INFO(node_->get_logger(), "Action server created: '%s' (world_frame='%s')",
                action_name_.c_str(), world_frame_.c_str());
  }

private:
  /**
   * @brief Called by the rclcpp action server when a new goal arrives.
   *
   * Performs quick validation checks on the incoming goal and decides whether to accept or reject it.
   * Typical checks include presence of group_name, existence of a configured planner for the group,
   * and non-empty goal_constraints.
   *
   * @param uuid Goal UUID (unused)
   * @param goal Shared pointer to the received goal message
   * @return rclcpp_action::GoalResponse ACCEPT_AND_EXECUTE to accept, otherwise REJECT
   */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& /*uuid*/,
      std::shared_ptr<const Action::Goal> goal)
  {
    if (!goal) return rclcpp_action::GoalResponse::REJECT;

    const auto& req = goal->motion_plan_request;

    if (req.group_name.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Rejected: motion_plan_request.group_name is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (planners_map_.find(req.group_name) == planners_map_.end())
    {
      RCLCPP_ERROR(node_->get_logger(), "Rejected: unknown group_name '%s'", req.group_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (req.goal_constraints.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Rejected: goal_constraints is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Cancel handler for currently-running goals.
   *
   * This implementation accepts cancel requests but note that planner::solve is blocking; cancel is best-effort.
   *
   * @param gh Goal handle (unused)
   * @return rclcpp_action::CancelResponse ACCEPT
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> /*gh*/)
  {
    // Blocking solve; accept cancel best-effort.
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Called when a goal is accepted; spawns a detached thread to execute the goal.
   *
   * @param gh Shared pointer to the accepted goal handle
   */
  void handle_accepted(const std::shared_ptr<GoalHandle> gh)
  {
    std::thread{std::bind(&PlanWithConstraintsMuxActionServer::execute, this, gh)}.detach();
  }

  /**
   * @brief Main execution body for a planning goal.
   *
   * This method performs the following high-level steps:
   * - Publish feedback about selection and configuration
   * - Validate and parse start and goal states
   * - Lock the planner, configure it, and call solve()
   * - Convert resulting solution to a MoveIt MotionPlanResponse
   * - Publish final feedback and set the goal as succeeded or aborted
   *
   * The method uses the planners_map_ and joint_names_map_ to select the correct planner and joint ordering.
   *
   * @param gh Goal handle for the executing goal
   */
  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    const auto goal = gh->get_goal();
    const auto& req = goal->motion_plan_request;
    const auto& group_name = req.group_name;

    auto feedback = std::make_shared<Action::Feedback>();
    auto result   = std::make_shared<Action::Result>();

    feedback->status = "Selecting planner";
    feedback->progress = 0.05f;
    gh->publish_feedback(feedback);

    auto pit = planners_map_.find(group_name);
    auto jit = joint_names_map_.find(group_name);

    if (pit == planners_map_.end() || jit == joint_names_map_.end())
    {
      result->motion_plan_response = make_error_response_(req, group_name, moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME);
      result->info = pose_constraints_msgs::msg::PlanningInfo{};
      gh->abort(result);
      return;
    }

    auto planner = pit->second;
    const auto& joint_names = jit->second;

    // Publish the geometric constraints from the incoming goal (best-effort)
    if (geom_constraints_pub_)
      geom_constraints_pub_->publish(goal->constraints);
    // timeout
    const double timeout = (req.allowed_planning_time > 0.0) ? req.allowed_planning_time : 5.0;

    // start configuration
    Eigen::VectorXd start_q;
    std::string err;
    if (!vector_from_joint_state_(req.start_state.joint_state, joint_names, start_q, err))
    {
      RCLCPP_ERROR(node_->get_logger(), "Start state error: %s", err.c_str());
      result->motion_plan_response = make_error_response_(req, group_name, moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
      result->info = pose_constraints_msgs::msg::PlanningInfo{};
      gh->abort(result);
      return;
    }

    // goal parsing
    const auto& gc = req.goal_constraints.front();
    const bool has_joint_goal = !gc.joint_constraints.empty();
    const bool has_pose_goal  = (!gc.position_constraints.empty() && !gc.orientation_constraints.empty());

    feedback->status = "Configuring planner";
    feedback->progress = 0.20f;
    gh->publish_feedback(feedback);

    graph::core::PathPtr solution;
    pose_constraints_planner::info pi{};

    {
      // Planner is stateful. Lock globally to avoid concurrent requests corrupting internal state.
      std::lock_guard<std::mutex> lk(planner_mtx_);

      planner->setLogging(goal->verbose);
      planner->setConstraints(goal->constraints);

      if (!planner->setStartConfiguration(start_q))
      {
        RCLCPP_ERROR(node_->get_logger(), "setStartConfiguration() returned false");
        result->motion_plan_response = make_error_response_(req, group_name, moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION);
        result->info = pose_constraints_msgs::msg::PlanningInfo{};
        gh->abort(result);
        return;
      }

      if (has_joint_goal)
      {
        Eigen::VectorXd goal_q;
        if (!vector_from_joint_constraints_(gc.joint_constraints, joint_names, goal_q, err))
        {
          RCLCPP_ERROR(node_->get_logger(), "Goal joint_constraints error: %s", err.c_str());
          result->motion_plan_response = make_error_response_(req, group_name, moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
          result->info = pose_constraints_msgs::msg::PlanningInfo{};
          gh->abort(result);
          return;
        }

        if (!planner->setGoalConfiguration(goal_q))
        {
          RCLCPP_ERROR(node_->get_logger(), "setGoalConfiguration() returned false");
          result->motion_plan_response = make_error_response_(req, group_name, moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
          result->info = pose_constraints_msgs::msg::PlanningInfo{};
          gh->abort(result);
          return;
        }
      }
      else if (has_pose_goal)
      {
        Eigen::Affine3d goal_pose_world;
        if (!goal_pose_world_from_constraints_tf_(gc, goal_pose_world, err))
        {
          RCLCPP_ERROR(node_->get_logger(), "Goal pose extraction/TF error: %s", err.c_str());
          result->motion_plan_response = make_error_response_(req, group_name, moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
          result->info = pose_constraints_msgs::msg::PlanningInfo{};
          gh->abort(result);
          return;
        }

        if (!planner->setGoalPose(goal_pose_world))
        {
          RCLCPP_ERROR(node_->get_logger(), "setGoalPose() returned false");
          result->motion_plan_response = make_error_response_(req, group_name, moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
          result->info = pose_constraints_msgs::msg::PlanningInfo{};
          gh->abort(result);
          return;
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Unsupported goal: provide joint_constraints OR (position_constraints + orientation_constraints)");
        result->motion_plan_response = make_error_response_(req, group_name, moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
        result->info = pose_constraints_msgs::msg::PlanningInfo{};
        gh->abort(result);
        return;
      }

      feedback->status = "Solving";
      feedback->progress = 0.60f;
      gh->publish_feedback(feedback);

      const bool ok = planner->solve(timeout, solution, pi);

      if (!ok || !solution)
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed (ok=%d, solution=%s)", ok ? 1 : 0, solution ? "non-null" : "null");
        result->motion_plan_response = make_motion_plan_response_from_solution_(req, group_name, joint_names, start_q, solution, pi.total_time, false);
        result->info = convert_planning_info_(pi);
        gh->abort(result);
        return;
      }

      feedback->status = "Converting solution";
      feedback->progress = 0.85f;
      gh->publish_feedback(feedback);

      result->motion_plan_response = make_motion_plan_response_from_solution_(req, group_name, joint_names, start_q, solution, pi.total_time, true);
      result->info = convert_planning_info_(pi);
    }

    feedback->status = "Done";
    feedback->progress = 1.0f;
    gh->publish_feedback(feedback);

    gh->succeed(result);
  }

  // ---------------------- helpers ----------------------

  /**
   * @brief Extract a world-frame goal pose from MoveIt's Constraints message by applying a TF transform.
   *
   * This helper expects the Constraints to contain at least one position_constraint and one orientation_constraint.
   * It constructs a PoseStamped in the constraint frame (from position_constraints[0].header and primitive_pose) and
   * transforms it to world_frame_ using tf_buffer_. The resulting pose is returned as an Eigen::Affine3d.
   *
   * @param c Constraints containing position + orientation constraints
   * @param out_goal_pose_world Output pose in world_frame_ (valid on success)
   * @param err On failure, contains a human-readable error message
   * @return true if the pose was successfully extracted and transformed
   * @return false on error (err is populated)
   */
  bool goal_pose_world_from_constraints_tf_(
      const moveit_msgs::msg::Constraints& c,
      Eigen::Affine3d& out_goal_pose_world,
      std::string& err) const
  {
    if (c.position_constraints.empty() || c.orientation_constraints.empty())
    {
      err = "position_constraints or orientation_constraints empty";
      return false;
    }

    const auto& pc = c.position_constraints.front();
    const auto& oc = c.orientation_constraints.front();

    if (pc.constraint_region.primitive_poses.empty())
    {
      err = "position_constraints[0].constraint_region.primitive_poses empty";
      return false;
    }

    // Build PoseStamped in the constraint frame
    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = pc.header;  // frame_id is here
    pose_in.pose.position = pc.constraint_region.primitive_poses.front().position;
    pose_in.pose.orientation = oc.orientation;

    if (pose_in.header.frame_id.empty())
    {
      err = "position_constraints[0].header.frame_id empty (cannot TF-transform)";
      return false;
    }

    geometry_msgs::msg::PoseStamped pose_world;
    try
    {
      // Small TF timeout; adjust if needed
      const tf2::Duration tf_timeout = tf2::durationFromSec(0.2);
      pose_world = tf_buffer_->transform(pose_in, world_frame_, tf_timeout);
    }
    catch (const tf2::TransformException& ex)
    {
      err = std::string("TF transform failed: ") + ex.what();
      return false;
    }

    // Convert Pose -> Eigen::Affine3d
    const auto& p = pose_world.pose.position;
    const auto& q = pose_world.pose.orientation;

    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    if (quat.norm() < 1e-9)
    {
      err = "orientation quaternion has near-zero norm after TF";
      return false;
    }
    quat.normalize();

    out_goal_pose_world = Eigen::Translation3d(Eigen::Vector3d(p.x, p.y, p.z)) * quat;
    return true;
  }

  /**
   * @brief Convert a sensor_msgs::JointState to an Eigen::VectorXd following joint_order.
   *
   * The function requires that js.name and js.position are non-empty and that every name in joint_order
   * appears in js.name. The resulting vector has the same order as joint_order.
   *
   * @param js input JointState (names and positions)
   * @param joint_order expected joint ordering
   * @param out output Eigen vector with joint positions in joint_order
   * @param err on failure, populated with a message
   * @return true on success
   * @return false on failure
   */
  static bool vector_from_joint_state_(
      const sensor_msgs::msg::JointState& js,
      const std::vector<std::string>& joint_order,
      Eigen::VectorXd& out,
      std::string& err)
  {
    if (js.name.empty() || js.position.empty())
    {
      err = "JointState name/position empty";
      return false;
    }

    std::unordered_map<std::string, double> m;
    m.reserve(js.name.size());
    for (size_t i = 0; i < js.name.size() && i < js.position.size(); ++i)
      m[js.name[i]] = js.position[i];

    out = Eigen::VectorXd::Zero(static_cast<int>(joint_order.size()));
    for (size_t i = 0; i < joint_order.size(); ++i)
    {
      auto it = m.find(joint_order[i]);
      if (it == m.end())
      {
        err = "Missing joint in start_state: " + joint_order[i];
        return false;
      }
      out(static_cast<int>(i)) = it->second;
    }
    return true;
  }

  /**
   * @brief Build an Eigen::VectorXd from a vector of MoveIt JointConstraint messages.
   *
   * Each JointConstraint must name a joint and provide a position. The resulting vector follows joint_order.
   *
   * @param jcs JointConstraint array (typically from Constraints.joint_constraints)
   * @param joint_order expected ordering of joints
   * @param out output Eigen vector populated with joint positions
   * @param err on failure, populated with a message
   * @return true if all joint names in joint_order are present in jcs
   */
  static bool vector_from_joint_constraints_(
      const std::vector<moveit_msgs::msg::JointConstraint>& jcs,
      const std::vector<std::string>& joint_order,
      Eigen::VectorXd& out,
      std::string& err)
  {
    if (jcs.empty())
    {
      err = "joint_constraints empty";
      return false;
    }

    std::unordered_map<std::string, double> m;
    m.reserve(jcs.size());
    for (const auto& jc : jcs)
      m[jc.joint_name] = jc.position;

    out = Eigen::VectorXd::Zero(static_cast<int>(joint_order.size()));
    for (size_t i = 0; i < joint_order.size(); ++i)
    {
      auto it = m.find(joint_order[i]);
      if (it == m.end())
      {
        err = "Missing joint in goal joint_constraints: " + joint_order[i];
        return false;
      }
      out(static_cast<int>(i)) = it->second;
    }
    return true;
  }

  /**
   * @brief Convert internal planner info into the ROS message pose_constraints_msgs::msg::PlanningInfo.
   *
   * This is a straightforward field-by-field copy; it assumes the in and out types have matching fields.
   */
  static pose_constraints_msgs::msg::PlanningInfo convert_planning_info_(const pose_constraints_planner::info& in)
  {
    pose_constraints_msgs::msg::PlanningInfo out;
    // Assumes PlanningInfo.msg fields match pose_constraints_planner::info
    out.total_time = in.total_time;
    out.total_iterations = in.total_iterations;
    out.total_rejections = in.total_rejections;
    out.max_iteration_time = in.max_iteration_time;
    out.min_iteration_time = in.min_iteration_time;
    out.average_iteration_time = in.average_iteration_time;
    out.total_reject_time = in.total_reject_time;
    return out;
  }

  /**
   * @brief Helper that creates a MotionPlanResponse populated with an error code.
   *
   * The returned response contains the input request's trajectory_start and the requested group_name.
   */
  static moveit_msgs::msg::MotionPlanResponse make_error_response_(
      const moveit_msgs::msg::MotionPlanRequest& req,
      const std::string& group_name,
      int32_t error_code_val)
  {
    moveit_msgs::msg::MotionPlanResponse out;
    out.trajectory_start = req.start_state;
    out.group_name = group_name;
    out.planning_time = 0.0;
    out.error_code.val = error_code_val;
    return out;
  }

  /**
   * @brief Convert a planner solution (graph::core::Path) into a MoveIt MotionPlanResponse.
   *
   * This function builds a JointTrajectory that contains the start waypoint followed by the solver's
   * internal waypoints. Each waypoint becomes a JointTrajectoryPoint with positions filled and an incrementing
   * time_from_start (1s steps are used as a simple placeholder for timing).
   *
   * If success==false or solution==nullptr the function returns a response with PLANNING_FAILED.
   *
   * @param req original MotionPlanRequest
   * @param group_name group for which the plan was computed
   * @param joint_names ordered joint names for the trajectory
   * @param start_wp vector of joint positions for the start
   * @param solution pointer to the solver's Path (may be null if failed)
   * @param planning_time_sec planner reported time
   * @param success whether planning succeeded
   * @return moveit_msgs::msg::MotionPlanResponse filled with trajectory and error code
   */
  moveit_msgs::msg::MotionPlanResponse make_motion_plan_response_from_solution_(
      const moveit_msgs::msg::MotionPlanRequest& req,
      const std::string& group_name,
      const std::vector<std::string>& joint_names,
      const Eigen::VectorXd& start_wp,
      const graph::core::PathPtr& solution,
      double planning_time_sec,
      bool success) const
  {
    moveit_msgs::msg::MotionPlanResponse out;
    out.trajectory_start = req.start_state;
    out.group_name = group_name;
    out.planning_time = planning_time_sec;

    if (!success || !solution)
    {
      out.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return out;
    }

    // Build waypoint list: start + internal waypoints (as requested)
    std::vector<Eigen::VectorXd> waypoints;
    waypoints.reserve(solution->getWaypoints().size() + 1);
    waypoints.push_back(start_wp);
    for (const auto& wp : solution->getWaypoints())
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Waypoint: " << wp.transpose());
      waypoints.push_back(wp);
    }

    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = joint_names;
    jt.points.reserve(waypoints.size());

    rclcpp::Duration time_from_start = rclcpp::Duration::from_seconds(0.0);

    for (const auto& wp : waypoints)
    {
      if (static_cast<size_t>(wp.size()) != joint_names.size())
      {
        out.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
        return out;
      }

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.resize(joint_names.size());
      for (size_t i = 0; i < joint_names.size(); ++i)
        pt.positions[i] = wp(static_cast<int>(i));

	  const int64_t ns = time_from_start.nanoseconds();
	  pt.time_from_start.sec = static_cast<int32_t>(ns / 1000000000LL);
      pt.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
      jt.points.push_back(std::move(pt));
      time_from_start = time_from_start + rclcpp::Duration::from_seconds(1.0);
    }

    out.trajectory.joint_trajectory = std::move(jt);
    out.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return out;
  }

private:
  /**
   * @brief Node handle used for logging and creating ROS interfaces
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief Name of the ROS2 action this server exposes
   */
  std::string action_name_;

  /**
   * @brief The world TF frame used for pose goals
   */
  std::string world_frame_;

  /**
   * @brief TF buffer used to transform incoming goal poses to world_frame_
   */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp_action::Server<Action>::SharedPtr server_;

  /**
   * @brief Publisher for geometric constraints from goals
   */
  rclcpp::Publisher<pose_constraints_msgs::msg::GeometricConstraintArray>::SharedPtr geom_constraints_pub_;

  // References to externally-owned maps (kept alive in main)
  const std::map<std::string, pose_constraints_planner::PoseConstraintsPlanner::Ptr>& planners_map_;
  const std::map<std::string, std::vector<std::string>>& joint_names_map_;

  // Single lock: planners are stateful and this avoids cross-group concurrent corruption.
  std::mutex planner_mtx_;
};

}  // namespace pose_constraints_planner
