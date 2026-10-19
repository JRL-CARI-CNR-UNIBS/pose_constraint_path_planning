/**
 * @file pose_constraints_planner.hpp
 * @brief RRT planner whose nodes must satisfy geometric constraints on the tool pose.
 */
#pragma once

#include <graph_core/graph/path.h>
#include <pose_constraints_planner/pose_constraints_manager.hpp>
// Class loader
#include <cnr_class_loader/multi_library_class_loader.hpp>

// IK solver
#include "ik_solver/internal/ik_solver_node.ros2.hpp"
#include <graph_core/graph/tree.h>

#include <graph_display/graph_display.h>


namespace pose_constraints_planner
{

  /**
   * @brief Statistics of a planning run.
   */
  struct info
  {
    double total_time;  ///< Total planning time [s].
    int total_iterations;  ///< Number of RRT iterations.
    int total_rejections;  ///< Samples or extensions rejected because they violate the constraints.
    double max_iteration_time;  ///< Longest iteration [s].
    double min_iteration_time;  ///< Shortest iteration [s].
    double average_iteration_time;  ///< Average iteration time [s].
    double total_reject_time;  ///< Time spent in rejected iterations [s].
  };

  /**
   * @brief RRT planner in joint space whose nodes must satisfy geometric constraints on the tool pose.
   *
   * The tool pose of each sample is computed with the forward kinematics of the IK solver and
   * checked with a PoseConstraintsManager (ANGLE constraints are relative to the start pose).
   * Samples and tree extensions that violate a constraint are rejected. With a goal bias the
   * tree tries to connect to one of the goal configurations.
   *
   * Usage: setConstraints(), setStartConfiguration(), then setGoalConfiguration() or
   * setGoalPose() (the goal configurations are computed with the IK solver), then solve().
   */
  class PoseConstraintsPlanner
  {
  public:
    /**
     * @brief Constructor. Reads the world-base and flange-tool transforms from TF.
     * @param node ROS node, used for logging.
     * @param collision_checker Collision checker.
     * @param ik_solver IK solver of the robot (also used for the forward kinematics).
     * @param sampler Joint-space sampler.
     * @param metrics Metrics.
     * @param logger_ptr graph_core logger.
     * @param display Display used to show the tree and the path in RViz (may be nullptr).
     * @param world_frame Frame in which the constraints are expressed.
     * @param tool_frame Tool frame whose pose is constrained.
     */
    PoseConstraintsPlanner(rclcpp::Node::SharedPtr node,
                           graph::core::CollisionCheckerPtr collision_checker,
                           std::shared_ptr<ik_solver::IkSolver> ik_solver,
                           graph::core::SamplerPtr sampler,
                           graph::core::MetricsPtr metrics,
                           cnr_logger::TraceLoggerPtr logger_ptr,
                           graph::display::DisplayPtr display,
                           std::string world_frame = "world",
                           std::string tool_frame = "ur10_tool0");

    /** define shared pointer PoseConstraintsPlanner::Ptr*/
    using Ptr = std::shared_ptr<PoseConstraintsPlanner>;

    /** Set geometric constraints from a GeometricConstraintArray message
     * @param constraints The GeometricConstraintArray message containing the constraints
     */
    void setConstraints(const pose_constraints_msgs::msg::GeometricConstraintArray &constraints);

    /** Set the start configuration and compute its pose
     * @param start_config The start joint configuration
     * @return true if the start configuration respects the pose constraints, false otherwise
     */
    bool setStartConfiguration(const Eigen::VectorXd& start_config);

    /** Set the goal configuration and compute its pose
     * @param goal_conf The goal joint configuration
     * @return true if the goal configuration respects the pose constraints, false otherwise
     */
    bool setGoalConfiguration(const Eigen::VectorXd& goal_conf);


    /** Get Constraints Manager
     * @return pointer to the PoseConstraintsManager instance
     */
    PoseConstraintsManager::Ptr getConstraintsManager() const { return pose_constraints_manager_; }


    /** Set the goal pose and compute its IK solutions
     * @param goal_pose The goal end-effector pose
     * @return true if the goal pose respects the pose constraints, false otherwise
     */
    bool setGoalPose(const Eigen::Affine3d& goal_pose);


    /** Get Display
     * @return pointer to the Display instance
     */
    graph::display::DisplayPtr getDisplay() const { return display_; }
    
    

    /** Enable or disable logging
     * @param enable True to enable logging, false to disable
     */
    void setLogging(const bool& enable);

    /** Solve the planning problem within a given timeout
     * @param timeout The maximum time allowed for planning (in seconds)
     * @param solution The resulting path if a solution is found
     * @param planning_info Statistics of the planning run (times, iterations, rejections)
     * @return true if a solution is found, false otherwise
     */
    bool solve(const double& timeout,
               graph::core::PathPtr& solution,
               info& planning_info);

    /**
     * @brief Tool pose of the start configuration, in the world frame.
     */
    Eigen::Affine3d getStartPose() const { return T_w_start; }
    /**
     * @brief Pose of the tool in the flange frame.
     */
    Eigen::Affine3d getFlangeToToolPose() const { return T_f_t; }
    /**
     * @brief Pose of the robot base in the world frame.
     */
    Eigen::Affine3d getWorldToBaseTransform() const { return T_w_b; }
  protected:
    Eigen::VectorXd start_config_;
    Eigen::Affine3d goal_pose_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ik_solver::IkSolver> ik_solver_;
    graph::core::CollisionCheckerPtr checker_;
    cnr_logger::TraceLoggerPtr logger_;
    graph::core::SamplerPtr sampler_;
    graph::core::MetricsPtr metrics_;
    unsigned int desired_solutions = 32;
    unsigned int min_stall_iterations = 100;
    unsigned int max_stall_iterations = 1000;
    ik_solver::Configurations goal_configurations_;
    std::string world_frame_ = "world";
    std::string tool_frame_ = "ur10_tool0";
    double max_distance = 0.1;
    bool use_kdtree = true;
    double goal_bias = 0.05;

    Eigen::Affine3d T_w_b; // world to base
    Eigen::Affine3d T_f_t; // flange to tool
    Eigen::Affine3d T_b_w; // base to world
    Eigen::Affine3d T_t_f; // tool to flange
    Eigen::Affine3d T_w_start; // world to start pose

    PoseConstraintsManager::Ptr  pose_constraints_manager_;


    bool print_log_ = false;

    graph::display::DisplayPtr display_;
  };

}  // namespace pose_constraints_planner
