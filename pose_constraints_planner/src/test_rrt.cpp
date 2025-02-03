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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  /* ----------------------------------------------------------------------------------------------------
   * WAITING FOR ROBOT DESCRIPTION
   * ----------------------------------------------------------------------------------------------------*/
  // Instantiate RobotDescriptionNode
  auto robot_description_node = std::make_shared<RobotDescriptionNode>();


  // Add RobotDescriptionNode to the executor

  rclcpp::executors::MultiThreadedExecutor executor;
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
  RCLCPP_INFO_STREAM(node->get_logger(),"Goal conf: " <<goal_conf .transpose());

  RCLCPP_INFO_STREAM(node->get_logger(),"LB conf: " <<lb .transpose());
  RCLCPP_INFO_STREAM(node->get_logger(),"UB conf: " <<ub .transpose());


  double max_angle_z = 0.3; // rad
  if(not graph::core::get_param(logger,param_ns2,"max_angle_z",max_angle_z))
  {
    return 1;
  }
  double cos_max_angle_z = std::cos(max_angle_z);

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


  Eigen::VectorXd qrand;

  int cycles=0;
  int nodes=0;


  /* ----------------------------------------------------------------------------------------------------
   * BEGIN OF THE MAIN LOOP: RRT
   * ----------------------------------------------------------------------------------------------------*/
  while (rclcpp::ok())
  {
    double random_value = dis(gen);
    if (random_value<goal_bias)
      qrand=goal_conf;
    else
      qrand=sampler->sample();


    Eigen::Affine3d T_b_rand=ik_solver->getFK(qrand); // transformation from base to tool in qrand;

    // EXAMPLE check angle between z axis of the rand frame and the z axis of the start frame
    Eigen::Vector3d z_b_start = T_b_start.linear().col(2);
    Eigen::Vector3d z_b_rand = T_b_rand.linear().col(2);  // linear -> rotation matrix, col(2) -> z axis
    double cos_angle = z_b_start.dot(z_b_rand); // scalar product between z_b_start and z_b_rand
    if (cos_angle<cos_max_angle_z)  // skip if the angle is too large
      continue;

    // LOG
    if (cycles++>1000)
    {
      RCLCPP_DEBUG(node->get_logger(),"Added %d nodes",nodes);
      cycles=0;
    }
    if (tree->extend(qrand, new_node))
    {
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
  }


  RCLCPP_INFO(node->get_logger(),"Press Ctrl+C to kill the test");
  while(rclcpp::ok())
    rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node->get_logger(),"killing....");

  rclcpp::shutdown();

  return 0;
}


