cmake_minimum_required(VERSION 3.8)
project(pose_constraints_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_runtime REQUIRED)


find_package(moveit_collision_checker REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdfdom REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(graph_core REQUIRED)
find_package(graph_display REQUIRED)
find_package(ik_solver REQUIRED)
find_package(pose_constraints_msgs REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(control_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_python_install_package(${PROJECT_NAME})


set(DEPS_LIB
    graph_core
    )
set(DEPS_TARGET
    graph_core::graph_core
    )

set(ROS_DEPS
    rclcpp
    graph_display
    visualization_msgs
    moveit_visual_tools
    tf2_eigen
    ik_solver
    moveit_collision_checker
    moveit_core
    moveit_ros_planning_interface
    pose_constraints_msgs
    )

add_library(pose_constraints_planner SHARED
    src/pose_constraints_planner/pose_constraints_manager.cpp
    src/pose_constraints_planner/pose_constraints_planner.cpp
    src/pose_constraints_planner/plan_with_constraints_action_server.cpp
    src/pose_constraints_planner/pose_constraints_path_local_optimizer.cpp
)
target_include_directories(pose_constraints_planner PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(pose_constraints_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(pose_constraints_planner
  PUBLIC
    graph_core::graph_core
    ik_solver::ik_solver
    pose_constraints_msgs::pose_constraints_msgs__rosidl_generator_cpp
)

# Create an alias for the library to link against easily
add_library(pose_constraints_planner::pose_constraints_planner ALIAS pose_constraints_planner)

add_executable(pose_constraints_planner_node src/pose_constraints_planner_node.cpp)
target_include_directories(pose_constraints_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)



target_compile_features(pose_constraints_planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(pose_constraints_planner_node PUBLIC
    pose_constraints_planner::pose_constraints_planner
    graph_core::graph_core
    ik_solver::ik_solver
    moveit_collision_checker::moveit_collision_checker
    graph_display::graph_display
    pose_constraints_msgs::pose_constraints_msgs__rosidl_generator_cpp
    moveit_msgs::moveit_msgs__rosidl_generator_cpp
    pose_constraints_msgs::pose_constraints_msgs__rosidl_typesupport_cpp
)

ament_target_dependencies(pose_constraints_planner_node PUBLIC
        rclcpp
        rclcpp_action
        rosidl_default_runtime
        moveit_core
        moveit_ros_planning_interface)


install(TARGETS
        pose_constraints_planner
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib
)

install(TARGETS pose_constraints_planner_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS_LIB} ${ROS_DEPS})





install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
    pose_constraints_planner/geometric_constraints_visualizer.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
