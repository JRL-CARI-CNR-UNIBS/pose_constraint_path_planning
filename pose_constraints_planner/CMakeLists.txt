cmake_minimum_required(VERSION 3.8)
project(pose_constraints_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(graph_core REQUIRED)
find_package(mirrt_star REQUIRED)
find_package(ik_solver REQUIRED)
# The exported ik_solver targets link urdfdom::* without finding it
find_package(urdfdom REQUIRED)
find_package(parallel_multigoal_planner REQUIRED)
find_package(pose_constraints_msgs REQUIRED)

# Python module of the constraint visualizer
ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/pose_constraints_manager.cpp
  src/${PROJECT_NAME}/pose_constraints_evaluator.cpp
  src/${PROJECT_NAME}/pose_constrained_sampler.cpp
  src/${PROJECT_NAME}/pose_constrained_collision_checker.cpp
  src/${PROJECT_NAME}/pose_constrained_multigoal_solver.cpp
  src/${PROJECT_NAME}/pose_constraints_planner.cpp
  src/${PROJECT_NAME}/plan_with_constraints_action_server.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  graph_core::graph_core
  mirrt_star::mirrt_star
  ik_solver::ik_solver
  parallel_multigoal_planner::parallel_multigoal_planner
  ${pose_constraints_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${pluginlib_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_eigen_TARGETS}
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  pose_constraints_planner/geometric_constraints_visualizer.py
  scripts/pose_constraints_planner_test.py
  scripts/plan_example.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_action pluginlib tf2_ros tf2_eigen moveit_msgs
  graph_core mirrt_star ik_solver parallel_multigoal_planner pose_constraints_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Unit tests: constraints, evaluator, sampler and collision checker, with a fake kinematics (no ROS graph needed)
  ament_add_gtest(test_pose_constraints test/test_pose_constraints.cpp)
  target_link_libraries(test_pose_constraints ${PROJECT_NAME})
endif()

ament_package()
