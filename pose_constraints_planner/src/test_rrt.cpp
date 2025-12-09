// ROS and Moveit related libraries
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Graph core libraries
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/plugins/solvers/tree_solver_plugin.h>
#include <graph_core/plugins/samplers/sampler_base_plugin.h>
#include <graph_core/plugins/metrics/metrics_base_plugin.h>

// Collision checker libraries
#include <moveit_collision_checker/plugins/collision_checkers/moveit_collision_checker_base_plugin.h>

// Display libraries (to see add a Marker in RViz, topic: /marker_visualization_topic)
#include <graph_display/graph_display.h>

// Class loader
#include <cnr_class_loader/multi_library_class_loader.hpp>

// IK solver
#include "ik_solver/internal/ik_solver_node.ros2.hpp"

#include <std_msgs/msg/string.hpp>

// Action server libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "rclcpp_components/register_node_macro.hpp"

// include fstream for logging
#include <fstream>

// class to read the robot description from topic
class RobotDescriptionNode : public rclcpp::Node
{
public:
    RobotDescriptionNode() : Node("robot_description_node")
    {
      RCLCPP_INFO(this->get_logger(), "Started RobotDescriptionNode...");
        // Subscriber to the "robot_description" topic

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
              rclcpp::QoS(1).transient_local().reliable(), // this is because the robot description is published only once
              std::bind(&RobotDescriptionNode::callback, this, std::placeholders::_1));
    }



    bool isRobotDescriptionReceived()
    {
        return !robot_description_.empty();
    }
    std::string getRobotDescription()
    {
        return robot_description_;
    }

private:

    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
      robot_description_ = msg->data; // Store the robot description
      std::string truncated_description = robot_description_.substr(0, 200);
      RCLCPP_INFO(this->get_logger(), "Received robot description:\n%s", truncated_description.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string robot_description_ = ""; // Variable to store the robot description
};

// Action client class
class TestRRTActionClient : public rclcpp::Node
{
  public:
    explicit TestRRTActionClient(const rclcpp::NodeOptions & options)
    : Node("test_rrt_action_client", options)
    {
      // Instantiate action client
      this->_client_ptr = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this,
        "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    }

    void set_trajectory(const std::vector<Eigen::VectorXd> & trajectory)
    {
      this->_trajectory = trajectory;
    }

    bool is_goal_done() const
    {
      return goal_done_.load();
    }

    void send_goal(std::vector<std::string> joint_names)
    {
      if (!this->_client_ptr->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available, waiting...");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Action server available, sending goal...");

      // Create a goal message
      auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

      // Fill in the goal message as needed
      goal_msg.trajectory.joint_names = joint_names; //{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
      trajectory_msgs::msg::JointTrajectoryPoint point;
      rclcpp::Duration time_from_start = rclcpp::Duration::from_seconds(0.0);

      goal_msg.trajectory.points.clear();
      
      for(const auto& waypoint : this->_trajectory)
      {
        point.positions.assign(waypoint.begin(), waypoint.end());
        point.time_from_start = time_from_start;
        goal_msg.trajectory.points.push_back(point);
        time_from_start = time_from_start + rclcpp::Duration::from_seconds(1.0); // increment time for next point
      }

      // Send the goal
      auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
      send_goal_options.goal_response_callback =
        std::bind(&TestRRTActionClient::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.result_callback =
        std::bind(&TestRRTActionClient::result_callback, this, std::placeholders::_1);

      this->_client_ptr->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr _client_ptr;
    rclcpp::TimerBase::SharedPtr _timer;
    std::vector<Eigen::VectorXd> _trajectory;
    std::atomic<bool> goal_done_{false};

    void goal_response_callback(
      rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        this->goal_done_.store(true);
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

    void result_callback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
        }
        this->goal_done_.store(true);
    }
  
};

/// @brief Checks if the angle between the corresponding axes of two transformations exceeds the maximum allowed angles.
/// @param T_1 First transformation.
/// @param T_2 Second transformation.
/// @param max_angle_cos Vector containing the cosine of the maximum allowed angles for each axis.
/// @param angle_cos Output parameter to store the computed cosine of the angle between the axes.
/// @return True if the angle constraints are satisfied, false otherwise.
bool check_angle_constraint(const Eigen::Affine3d& T_1,
                            const Eigen::Affine3d& T_2,
                            const Eigen::Vector3d& max_angle_cos,
                            double& angle_cos)
{
  // check angle constraints
  for (int i=0; i<3; i++)
  {
    if (max_angle_cos(i)>-1) // if max_angle_cos < -1 => no constraint
    {
      // linear returns the rotation matrix, col(i) -> axis
      // scalar product between axis_1 and axis_2 divided by their norms = cos(angle
      angle_cos = T_1.linear().row(i).dot(T_2.linear().row(i))/(T_1.linear().row(i).norm()*T_2.linear().row(i).norm()); 

      if (angle_cos<max_angle_cos(i))  // skip if the angle is too large
      {
        return false;
      }
    }
  }
  return true;
}

/// @brief Checks if the pose defined by transformation T satisfies the plane constraint defined by plane_origin and plane_normal.
/// @param T
/// @param plane_origin
/// @param plane_normal
/// @param tolerance
/// @param distance
/// @return True if the constraint is satisfied, false otherwise.
bool check_plane_constraint(const Eigen::Affine3d& T,
                            const Eigen::Vector3d& plane_origin,
                            const Eigen::Vector3d& plane_normal,
                            double tolerance,
                            double& distance)
{
  Eigen::Vector3d vec_plane_to_p = T.translation() - plane_origin;
  distance = vec_plane_to_p.dot(plane_normal.normalized()); // distance from point to plane  

  if (std::abs(distance)<tolerance) // tolerance
  {
    return false;
  }
  return true;
}

/// @brief Checks if the pose defined by transformation T satisfies the line constraint defined by line_origin and line_dir.
/// @param T 
/// @param line_origin 
/// @param line_dir 
/// @param max_distance 
/// @param distance 
/// @return True if the constraint is satisfied, false otherwise.
bool check_line_constraint(const Eigen::Affine3d& T,
                           const Eigen::Vector3d& line_origin,
                           const Eigen::Vector3d& line_dir,
                           double max_distance,
                           double& distance)
{
  Eigen::Vector3d p_to_line = T.translation() - line_origin;
  Eigen::Vector3d projection = p_to_line.dot(line_dir.normalized()) * line_dir.normalized();
  distance = (p_to_line - projection).norm();

  // Alternative computation using cross product
  // double distance = std::abs(p_to_line.cross(line_dir.normalized()).norm());

  if (distance < max_distance)
  {
    return true;
  }
  return false;
}

struct GeometricConstraint
{
  std::string name;

  enum ConstraintType {PLANE, LINE, ANGLE};
  ConstraintType type;

  // Plane constraint parameters
  Eigen::Vector3d plane_origin;
  Eigen::Vector3d plane_normal;
  double plane_tolerance;

  // Line constraint parameters
  Eigen::Vector3d line_origin;
  Eigen::Vector3d line_dir;
  double line_max_distance;

  // Angle constraint parameters
  Eigen::Vector3d max_angle;
  Eigen::Vector3d max_angle_cos;
};

bool check_constraints(const Eigen::Affine3d& T_current,
                       const Eigen::Affine3d& T_start,
                       const std::vector<GeometricConstraint>& constraints,
                       std::stringstream* report = nullptr)
{
  for (const auto& constraint : constraints)
  {
    double value;

    switch (constraint.type)
    {
      case GeometricConstraint::PLANE:
        if (!check_plane_constraint(T_current,
                                    constraint.plane_origin,
                                    constraint.plane_normal,
                                    constraint.plane_tolerance,
                                    value))
        {
          if (report)
            *report << "Plane constraint '" << constraint.name << "' violated. Distance: " << value << "\n";
          return false;
        }
        break;

      case GeometricConstraint::LINE:
        if (!check_line_constraint(T_current,
                                   constraint.line_origin,
                                   constraint.line_dir,
                                   constraint.line_max_distance,
                                   value))
        { 
          if (report)
            *report << "Line constraint '" << constraint.name << "' violated. Distance: " << value << "\n";
          return false;
        }
        break;

      case GeometricConstraint::ANGLE:
        if (!check_angle_constraint(T_current,
                                    T_start,
                                    constraint.max_angle_cos,
                                    value))
        {
          if (report)
            *report << "Angle constraint '" << constraint.name << "' violated. Cosine of angle: " << value << "\n";
          return false;
        }
        break;
    }
  }
  return true;
}

bool permutationName(  const std::vector<std::string>& order_names,
                       std::vector<std::string>& names,
                       std::vector<double>& position,
                       std::vector<double>& velocity,
                       std::vector<double>& effort,
                       std::stringstream* report)
{
  if (names.size()<order_names.size())
  {
    if(report)
      *report << "The vector of names to be sorted has size " << names.size()
                  << " that is smaller than vector of the sorted names (" << order_names.size() <<")";
    return false;
  }
  if (names.size()!=position.size())
  {
    if(report)
      *report << "Input Mismatch. The vector of names to be sorted has size " << names.size()
                 << " while position size is " << position.size();
    return false;
  }
  if (names.size()!=velocity.size())
  {
    if(report)
      *report << "Input Mismatch. The vector of names to be sorted has size " << names.size()
                << " while velocity size is " << velocity.size();
    return false;
  }
  if (names.size()!=effort.size())
  {
    if(report)
      *report << "Input Mismatch. The vector of names to be sorted has size " << names.size()
                << " while effort size is " << effort.size();
    return false;
  }



  for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
  {
    if (names.at(iOrder).compare(order_names.at(iOrder)))
    {
      for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
      {
        if (!order_names.at(iOrder).compare(names.at(iNames)))
        {
          std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
          std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
          std::iter_swap(velocity.begin()+iOrder, velocity.begin()+iNames);
          std::iter_swap(effort.begin()+iOrder,   effort.begin()+iNames);
          break;
        }
        if (iNames==(names.size()-1))
        {
          if(*report)
          {
            *report << "The Joint '" << order_names.at(iOrder) <<"' that is in the vector of the sorted names,"
                    << "is missing in the vector to be sorted.";
            *report << "Sorted Names: <";
            for( size_t i=0;i<order_names.size();i++)
                *report << order_names.at(i) <<",";
            *report << "> vs Names to be ordered: <";
            for( size_t i=0;i<names.size();i++)
              *report << names.at(i) <<",";
            *report <<">";
          }
          return false;
        }
      }
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  /* ----------------------------------------------------------------------------------------------------
   * WAITING FOR ROBOT DESCRIPTION
   * ----------------------------------------------------------------------------------------------------*/
  // Instantiate RobotDescriptionNode
  auto robot_description_node = std::make_shared<RobotDescriptionNode>();


  // Add RobotDescriptionNode to the executor

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),10);
  executor.add_node(robot_description_node);

  RCLCPP_INFO(robot_description_node->get_logger(), "Waiting for robot_description to be received...");
  auto start_time = std::chrono::steady_clock::now();
  std::chrono::seconds timeout_duration(30); // Timeout after 30 seconds

  while (rclcpp::ok() && !robot_description_node->isRobotDescriptionReceived())
  {
      executor.spin_some(); // Process callbacks
      rclcpp::sleep_for(std::chrono::milliseconds(100)); // Avoid busy-waiting

      auto elapsed_time = std::chrono::steady_clock::now() - start_time;
      if (elapsed_time > timeout_duration)
      {
          RCLCPP_ERROR(robot_description_node->get_logger(), "Timeout waiting for robot_description.");
          return 1; // Exit if the robot description is not received
      }
  }
  RCLCPP_INFO(robot_description_node->get_logger(), "Robot description successfully received.");
  std::string robot_description = robot_description_node->getRobotDescription();


  /* ----------------------------------------------------------------------------------------------------
   * READING PARAMETERS, LOADING PLUGINS, AND SETTING UP PLANNING TOOLS
   * ----------------------------------------------------------------------------------------------------*/
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("test_solver", options);

  // Extract parameters
  node->declare_parameter("test_mode", 0);
  int test_mode = node->get_parameter("test_mode").as_int();
  RCLCPP_INFO_STREAM(node->get_logger(),"Test mode: "<<(test_mode>0?std::string("ENABLED, %d iterations", test_mode):"DISABLED"));

  node->declare_parameter("use_sharework", false);
  bool use_sharework = false;
  use_sharework = node->get_parameter("use_sharework").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(),"Use Sharework configuration: "<<(use_sharework>0?std::string("ENABLED"):std::string("DISABLED")));

  // Load logger configuration file
  std::string package_name = "pose_constraints_planner";
  std::string package_path = ament_index_cpp::get_package_share_directory(package_name);

  if (package_path.empty())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Failed to get path for package '" << package_name);
    return 1;
  }

  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("test_solver",logger_file);

  // Get the robot description
  std::string param_ns1 = "/"+package_name;
  std::string param_ns2 = param_ns1+"/test_solver";
  std::string param_ns3 = param_ns1+"/geometric_constraints";
  std::string group_name;

  if(not graph::core::get_param(logger,param_ns2,"group_name",group_name))
  {
    return 1;
  }

  robot_model_loader::RobotModelLoader robot_model_loader(node,"robot_description");
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  const moveit::core::JointModelGroup* joint_model_group =  kinematic_model->getJointModelGroup(group_name);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof); // lower bounds
  Eigen::VectorXd ub(dof); // upper bounds

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const moveit::core::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }
  // creating the display
  graph::display::DisplayPtr display = std::make_shared<graph::display::Display>(node,planning_scene,group_name,kinematic_model->getLinkModelNames().back());
  kinematic_model->getLinkModelNames();
  rclcpp::sleep_for(std::chrono::seconds(1));

  // waiting for the planning scene
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr ps_client =
      node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  if (!ps_client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(),"Unable to connect to /get_planning_scene");
    return 1;
  }

  auto ps_srv = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  auto result = ps_client->async_send_request(ps_srv);
  if (rclcpp::spin_until_future_complete(node, result)!=rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(),"Call to srv not ok");
    return 1;
  }

  if (!planning_scene->setPlanningSceneMsg(result.get()->scene))
  {
    RCLCPP_ERROR(node->get_logger(),"unable to update planning scene");
    return 1;
  }

  // Read start and goal configurations
  Eigen::VectorXd start_conf, goal_conf;
  if(not graph::core::get_param(logger,param_ns2,"start_configuration",start_conf))
  {
    return 1;
  }
  if(not graph::core::get_param(logger,param_ns2,"goal_configuration",goal_conf))
  {
    return 1;
  }

  RCLCPP_INFO_STREAM(node->get_logger(),"Start conf: "<<start_conf.transpose());
  RCLCPP_INFO_STREAM(node->get_logger(),"Goal conf: " <<goal_conf.transpose());

  RCLCPP_INFO_STREAM(node->get_logger(),"LB conf: " <<lb.transpose());
  RCLCPP_INFO_STREAM(node->get_logger(),"UB conf: " <<ub.transpose());

  // Set-up planning tools
  graph::core::GoalCostFunctionPtr goal_cost_fcn = std::make_shared<graph::core::GoalCostFunctionBase>();

  // Set-up the class laoder
  cnr_class_loader::MultiLibraryClassLoader loader(false);
  std::vector<std::string> libraries;
  if(not graph::core::get_param(logger,param_ns2,"libraries",libraries))
  {
    return 1;
  }

  for(const std::string& lib:libraries)
    loader.loadLibrary(lib);

  // Load collision checker plugin
  std::string checker_plugin_name;
  graph::core::get_param(logger,param_ns2,"checker_plugin",checker_plugin_name,(std::string)"graph::ros1::ParallelMoveitCollisionCheckerPlugin");

  RCLCPP_INFO_STREAM(node->get_logger(),"Loading checker "<<checker_plugin_name);
  std::shared_ptr<graph::collision_check::MoveitCollisionCheckerBasePlugin> checker_plugin = loader.createInstance<graph::collision_check::MoveitCollisionCheckerBasePlugin>(checker_plugin_name);

  RCLCPP_INFO(node->get_logger(),"Configuring checker plugin ");
  checker_plugin->init(param_ns2,planning_scene,logger);
  graph::core::CollisionCheckerPtr checker = checker_plugin->getCollisionChecker();

  // Load sampler plugin
  std::string sampler_plugin_name;
  graph::core::get_param(logger,param_ns2,"sampler_plugin",sampler_plugin_name,(std::string)"graph::core::InformedSamplerPlugin");

  RCLCPP_INFO_STREAM(node->get_logger(),"Loading sampler "<<sampler_plugin_name);
  std::shared_ptr<graph::core::SamplerBasePlugin> sampler_plugin = loader.createInstance<graph::core::SamplerBasePlugin>(sampler_plugin_name);

  RCLCPP_INFO(node->get_logger(),"Configuring sampler plugin ");
  Eigen::VectorXd scale(dof); scale.setOnes(dof,1);

  sampler_plugin->init(param_ns2,start_conf,goal_conf,lb,ub,scale,logger);
  graph::core::SamplerPtr sampler = sampler_plugin->getSampler();

  // Load metrics plugin
  std::string metrics_plugin_name;
  graph::core::get_param(logger,param_ns2,"metrics_plugin",metrics_plugin_name,(std::string)"graph::core::EuclideanMetricsPlugin");

  RCLCPP_INFO_STREAM(node->get_logger(),"Loading metrics "<<metrics_plugin_name);
  std::shared_ptr<graph::core::MetricsBasePlugin> metrics_plugin = loader.createInstance<graph::core::MetricsBasePlugin>(metrics_plugin_name);

  RCLCPP_INFO(node->get_logger(),"Configuring metrics plugin ");
  metrics_plugin->init(param_ns2,logger);
  graph::core::MetricsPtr metrics = metrics_plugin->getMetrics();


  std::string ik_plugin_name="/ur_ik_solver";
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader("ik_solver", "ik_solver::IkSolver");
  std::shared_ptr<ik_solver::IkSolver> ik_solver = ik_loader.createSharedInstance("ik_solver/Ur10eIkSolver");
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  listener  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  ik_solver->setBuffer(tf_buffer);

  std::string param_what_;

  if(!cnr::param::set(ik_plugin_name+std::string("/robot_description"), robot_description, param_what_))
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot set cnr::param(" << node->get_namespace() << std::string("/robot_description") << ") because: " << param_what_);
  }
  if (!ik_solver->config(ik_plugin_name))
  {
    RCLCPP_ERROR(node->get_logger(), "unable to configure ik_solver %s",ik_plugin_name.c_str());
    rclcpp::sleep_for(std::chrono::seconds(100));

    return 1;
  }

  if (!checker->check(start_conf))
  {
    RCLCPP_ERROR(node->get_logger(),"Joints:");
    for(unsigned int i=0;i<joint_names.size();i++)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),joint_names.at(i) <<": "<<start_conf(i));
    }
    RCLCPP_ERROR(node->get_logger(),"Start configuration is in collision");
    return 1;
  }
  if (!checker->check(goal_conf))
  {
    RCLCPP_ERROR(node->get_logger(),"Goal configuration is in collision");
    return 1;
  }

  graph::core::PathPtr solution;
  graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start_conf,logger);
  graph::core::NodePtr goal_node = std::make_shared<graph::core::Node>(goal_conf,logger);


  // EXAMPLE: compute forward kinematics
  Eigen::Affine3d T_b_start=ik_solver->getFK(start_conf);

  // EXAMPLE: compute inverse kinematics
  ik_solver::Configurations seeds;
  unsigned int desired_solutions = 32;
  unsigned int min_stall_iterations = 100;
  unsigned int max_stall_iterations = 1000;
  ik_solver::Configurations solutions = ik_solver->getIk(T_b_start,
                                                         seeds,
                                                         desired_solutions,
                                                         min_stall_iterations,
                                                         max_stall_iterations).configurations();


  bool use_kdtree;
  if(not graph::core::get_param(logger,param_ns2,"use_kdtree",use_kdtree))
  {
    return 1;
  }
  double max_distance;
  if(not graph::core::get_param(logger,param_ns2,"max_distance",max_distance))
  {
    return 1;
  }
  double goal_bias;
  if(not graph::core::get_param(logger,param_ns2,"goal_bias",goal_bias))
  {
    return 1;
  }

  std::random_device rd;  // Seed source
  std::mt19937 gen(rd()); // Mersenne Twister engine
  std::uniform_real_distribution<> dis(0.0, 1.0);

  graph::core::NodePtr new_node;
  graph::core::TreePtr tree = std::make_shared<graph::core::Tree>(start_node,
                                                                  max_distance,
                                                                  checker,
                                                                  metrics,
                                                                  logger,
                                                                  use_kdtree);
  // Parse geometric constraints parameters
  YAML::Node constraints_array;
  std::string what;
  if(!cnr::param::get(param_ns3,constraints_array, what))
  {
    RCLCPP_ERROR(node->get_logger(),"Unable to load %s. error: %s",param_ns3.c_str(),what.c_str());
    return 1;
  }

  if(!constraints_array || !constraints_array.IsSequence()) 
  {    
    RCLCPP_ERROR(node->get_logger(),"test_yaml is missing or not a sequence\n");
    return 1;
  }

  std::vector<GeometricConstraint> geometric_constraints;

  for(const auto& yaml_node : constraints_array)
  {
    GeometricConstraint gc;

    gc.name = yaml_node["name"].as<std::string>();
    if(yaml_node["type"].as<std::string>()=="plane") 
      gc.type = GeometricConstraint::PLANE; // for example
    else if(yaml_node["type"].as<std::string>()=="line")
      gc.type = GeometricConstraint::LINE;
    else if(yaml_node["type"].as<std::string>()=="orientation")
      gc.type = GeometricConstraint::ANGLE;
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),"Unknown geometric constraint type: "<<yaml_node["type"].as<std::string>());
      return 1;
    }

    switch (gc.type)
    {
      case GeometricConstraint::PLANE:
        gc.plane_origin = Eigen::Vector3d(yaml_node["origin"][0].as<double>(),
                                          yaml_node["origin"][1].as<double>(),
                                          yaml_node["origin"][2].as<double>());
        gc.plane_normal = Eigen::Vector3d(yaml_node["normal"][0].as<double>(),
                                          yaml_node["normal"][1].as<double>(),
                                          yaml_node["normal"][2].as<double>());
        break;

      case GeometricConstraint::LINE:
        gc.line_origin = Eigen::Vector3d(yaml_node["origin"][0].as<double>(),
                                         yaml_node["origin"][1].as<double>(),
                                         yaml_node["origin"][2].as<double>());
        gc.line_dir = Eigen::Vector3d(yaml_node["direction"][0].as<double>(),
                                      yaml_node["direction"][1].as<double>(),
                                      yaml_node["direction"][2].as<double>());
        gc.line_max_distance = yaml_node["max_distance"].as<double>();
        break;

      case GeometricConstraint::ANGLE:
        gc.max_angle = Eigen::Vector3d(yaml_node["max_angle"][0].as<double>(),
                                       yaml_node["max_angle"][1].as<double>(),
                                       yaml_node["max_angle"][2].as<double>());
          for (int i=0; i<3; i++)
        {
          if (gc.max_angle(i)<0) // no constraint
            gc.max_angle_cos(i)=-1.1;
          else
            gc.max_angle_cos(i)=cos(gc.max_angle(i));
        }
        break;
    }

    geometric_constraints.push_back(gc);
  }

  RCLCPP_INFO(node->get_logger(),"Constraints laoded successfully.");

  // Check start and goal configurations against constraints
  Eigen::Affine3d T_b_goal=ik_solver->getFK(goal_conf);

  std::stringstream report;

  if(!check_constraints(T_b_start,T_b_start,geometric_constraints,&report))
  {
    RCLCPP_ERROR(node->get_logger(),"Start configuration violates geometric constraints.");
    RCLCPP_ERROR_STREAM(node->get_logger(),"Start matrix:\n"<<T_b_start.matrix());
    RCLCPP_ERROR_STREAM(node->get_logger(),"Report:\n"<<report.str());
    return 1;
  }

  if(!check_constraints(T_b_goal,T_b_start,geometric_constraints,&report))
  {
    RCLCPP_ERROR(node->get_logger(),"Goal configuration violates geometric constraints.");
    RCLCPP_ERROR_STREAM(node->get_logger(),"Start matrix:\n"<<T_b_start.matrix());
    RCLCPP_ERROR_STREAM(node->get_logger(),"Goal matrix:\n"<<T_b_start.matrix());
    RCLCPP_ERROR_STREAM(node->get_logger(),"Report:\n"<<report.str());
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),"Start and goal configurations satisfy the geometric constraints.");

  Eigen::VectorXd qrand;

  int cycles=0;
  int nodes=0;

  /* ----------------------------------------------------------------------------------------------------
   * BEGIN OF THE MAIN LOOP: RRT
   * ----------------------------------------------------------------------------------------------------*/

  using clock = std::chrono::steady_clock;
  

  int test_cycles = 0;

  if(test_mode>0)
  {
    std::ofstream stats_file;
    stats_file.open("test_solver_stats.txt",std::ios::app);
    if (!stats_file.is_open())
    {
      RCLCPP_ERROR(node->get_logger(),"Unable to open stats file");
      return 1;
    }
    stats_file << "Nodes added, Total iterations, RRT rejections, Extension rejections, Total rejections, Total time [ms], Minimum time [ms], Maximum time [ms], Average time [ms], Total rejection time [ms]\n";
    stats_file.close();
  }

  do{
    auto start_time_rrt = clock::now();
    auto start_time_rrt_i = clock::now();
    auto end_time_rrt_i = clock::now();
    double max_time_rrt = 0.0;
    double min_time_rrt = 0.0;
    double elapsed_rrt_i = 0.0;
    double reject_time = 0.0; // time spent to reject samples
    std::vector<double> rrt_iteration_times;
    
    int total_iterations = 0;
    int rrt_rejections = 0;
    int extension_rejections = 0;
    int total_rejections = 0;

    tree->cleanTree();
    nodes=0;
    while (rclcpp::ok())
    {
      // measure starting time of eachi RRT iteration
      start_time_rrt_i = clock::now();

      double random_value = dis(gen);
      if (random_value<goal_bias)
      {
        qrand=goal_conf;
      }
      else
      {
        total_iterations++;

        qrand=sampler->sample();

        Eigen::Affine3d T_b_rand=ik_solver->getFK(qrand); // transformation from base to tool in qrand;

        if(!check_constraints(T_b_rand,T_b_start,geometric_constraints,&report))
        {
          rrt_rejections++;
          total_rejections++;
          // measure end time of each RRT iteration
          end_time_rrt_i = clock::now();
          elapsed_rrt_i = std::chrono::duration<double, std::milli>(end_time_rrt_i - start_time_rrt_i).count();
          reject_time += elapsed_rrt_i;
          rrt_iteration_times.push_back(elapsed_rrt_i);
          max_time_rrt = std::max(max_time_rrt, elapsed_rrt_i);
          min_time_rrt = (min_time_rrt == 0.0) ? elapsed_rrt_i : std::min(min_time_rrt, elapsed_rrt_i);
          continue;
        }
        // LOG every 1000 iterations of feasible points
        if (cycles++>1000 && test_mode==0)
        {
          RCLCPP_INFO_STREAM(node->get_logger(),"Start matrix:\n"<<T_b_start.matrix());
          RCLCPP_INFO_STREAM(node->get_logger(),"Current matrix:\n"<<T_b_rand.matrix());
          RCLCPP_INFO_STREAM(node->get_logger(),"Current point: "<<T_b_rand.translation().transpose());
          RCLCPP_INFO(node->get_logger(),"Added %d nodes",nodes);
          RCLCPP_INFO_STREAM(node->get_logger(),"Tree extended, "<<nodes<<" nodes in the tree");
          cycles=0;
        }
      }
      if (tree->extend(qrand, new_node))
      {
        auto new_conf = new_node->getConfiguration();
        RCLCPP_DEBUG_STREAM(node->get_logger(),"New node added: "<<new_conf.transpose());
        if(!check_constraints(ik_solver->getFK(new_conf),
                              T_b_start,
                              geometric_constraints,
                              &report))
        {
          total_rejections++;
          extension_rejections++;

          if(test_mode==0)
          {
            RCLCPP_ERROR(node->get_logger(),"New configuration violates geometric constraints.");
            RCLCPP_ERROR_STREAM(node->get_logger(),"Start matrix:\n"<<T_b_start.matrix());
            RCLCPP_ERROR_STREAM(node->get_logger(),"Current matrix:\n"<<ik_solver->getFK(new_conf).matrix());
            RCLCPP_ERROR_STREAM(node->get_logger(),"Current point: "<<ik_solver->getFK(new_conf).translation().transpose());
            RCLCPP_ERROR_STREAM(node->get_logger(),"Report:\n"<<report.str());
            RCLCPP_INFO(node->get_logger(),"Added %d nodes",nodes);
            RCLCPP_INFO_STREAM(node->get_logger(),"Tree extended, "<<nodes<<" nodes in the tree");
          }

          tree->removeNode(new_node);
          // measure end time of each RRT iteration
          end_time_rrt_i = clock::now();
          elapsed_rrt_i = std::chrono::duration<double, std::milli>(end_time_rrt_i - start_time_rrt_i).count();
          reject_time += elapsed_rrt_i;
          rrt_iteration_times.push_back(elapsed_rrt_i);
          max_time_rrt = std::max(max_time_rrt, elapsed_rrt_i);
          min_time_rrt = (min_time_rrt == 0.0) ? elapsed_rrt_i : std::min(min_time_rrt, elapsed_rrt_i);
          continue;
        }
        
        nodes++;
        if ((new_node->getConfiguration()-goal_conf).norm()<max_distance)
        {
          RCLCPP_INFO(node->get_logger(),"Checking if tree can reach goal");
          if (checker->checkConnection(new_node->getConfiguration(),goal_conf))
          {
            tree->extend(qrand, goal_node);
            RCLCPP_INFO(node->get_logger(),"Goal reached");

            solution = std::make_shared<graph::core::Path>(tree->getConnectionToNode(goal_node), metrics, checker, logger);
            solution->setTree(tree);
            display->displayPathAndWaypoints(solution);
            display->displayTree(tree,"graph_display",{0.0,0.0,1.0,0.15});
            break;
          }
        }
      }
      // measure end time of each RRT iteration
      end_time_rrt_i = clock::now();
      elapsed_rrt_i = std::chrono::duration<double, std::milli>(end_time_rrt_i - start_time_rrt_i).count();
      rrt_iteration_times.push_back(elapsed_rrt_i);
      max_time_rrt = std::max(max_time_rrt, elapsed_rrt_i);
      min_time_rrt = (min_time_rrt == 0.0) ? elapsed_rrt_i : std::min(min_time_rrt, elapsed_rrt_i);
    }

    // measure end time of last RRT iteration
    end_time_rrt_i = clock::now();
    elapsed_rrt_i = std::chrono::duration<double, std::milli>(end_time_rrt_i - start_time_rrt_i).count();
    rrt_iteration_times.push_back(elapsed_rrt_i);
    max_time_rrt = std::max(max_time_rrt, elapsed_rrt_i);
    min_time_rrt = (min_time_rrt == 0.0) ? elapsed_rrt_i : std::min(min_time_rrt, elapsed_rrt_i);

    // measure end time of RRT
    auto end_time_rrt = clock::now();
    double elapsed_rrt = std::chrono::duration<double, std::milli>(end_time_rrt - start_time_rrt).count();
    RCLCPP_INFO_STREAM(node->get_logger(),"RRT found a solution in "<<elapsed_rrt<<" ms");
    RCLCPP_INFO_STREAM(node->get_logger(),"RRT iteration times over "<<rrt_iteration_times.size()<<" iterations: min "<<min_time_rrt<<" ms, max "<<max_time_rrt<<" ms"<<", mean "<<(std::accumulate(rrt_iteration_times.begin(), rrt_iteration_times.end(), 0.0) / rrt_iteration_times.size())<<" ms");
    if(test_mode>0)
    {
      std::ofstream stats_file;
      stats_file.open("test_solver_stats.txt",std::ios::app);
      if (!stats_file.is_open())
      {
        RCLCPP_ERROR(node->get_logger(),"Unable to open stats file");
        return 1;
      }
      stats_file << nodes << ", "
                 << total_iterations << ", "
                 << rrt_rejections << ", "
                 << extension_rejections << ", "
                 << total_rejections << ", "
                 << elapsed_rrt << ", "
                 << min_time_rrt << ", "
                 << max_time_rrt << ", "
                 << (std::accumulate(rrt_iteration_times.begin(), rrt_iteration_times.end(), 0.0) / rrt_iteration_times.size()) << ", "
                 << reject_time << "\n";
      stats_file.close();
    }
  }while(test_cycles++<test_mode);
  
  
  // simulate the trajectory execution using an action client
  // first, get the current joint states
  auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
  auto joint_state_node = rclcpp::Node::make_shared("joint_state_subscriber");

  auto joint_state_rg=joint_state_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions cb_options;
  cb_options.callback_group = joint_state_rg;
  auto joint_state_sub = joint_state_node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SensorDataQoS(),
    [&joint_state_msg](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::cout << "Joint states received." << std::endl;
      joint_state_msg = msg;
    },
    cb_options
  );
  executor.add_node(joint_state_node);


  RCLCPP_INFO(node->get_logger(),"Waiting for joint_states...");
  while (rclcpp::ok() && joint_state_msg->position.empty())
  {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(node->get_logger(),"joint_states received.");

  // prepare the waypoints
  Eigen::VectorXd start_wp(joint_state_msg->position.size());
  if (!permutationName(joint_names,
                      joint_state_msg->name,
                      joint_state_msg->position,
                      joint_state_msg->velocity,
                      joint_state_msg->effort,
                      nullptr))
  {
    RCLCPP_ERROR(node->get_logger(),"Unable to permutate joint states");
    return 1;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(),"Joint states permutated");
  }

  for (size_t i = 0; i < joint_state_msg->position.size(); ++i)
  {
    start_wp(i) = joint_state_msg->position[i];
    RCLCPP_INFO_STREAM(node->get_logger(),"Current joint "<<joint_state_msg->name[i]<<" position: "<<joint_state_msg->position[i]);
  }

  std::vector<Eigen::VectorXd> waypoints;
  waypoints.reserve(solution->getWaypoints().size()+1);
  waypoints.push_back(start_wp);
  for (const auto& wp : solution->getWaypoints())
  {
    RCLCPP_INFO_STREAM(node->get_logger(),"Waypoint: "<<wp.transpose());
    waypoints.push_back(wp);
  }

  // create the action client node
  auto action_client_node = std::make_shared<TestRRTActionClient>(options);
  executor.add_node(action_client_node);

  action_client_node->set_trajectory(waypoints);
  action_client_node->send_goal(joint_names);

  while (rclcpp::ok() && !action_client_node->is_goal_done()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(node->get_logger(),"Press Ctrl+C to kill the test");
  while(rclcpp::ok())
    rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node->get_logger(),"killing....");

  rclcpp::shutdown();

  return 0;
}