cmake_minimum_required(VERSION 3.8)
project(pose_constraints_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(moveit_collision_checker REQUIRED)
find_package(ur_ik_solver REQUIRED)
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
find_package(ur_ik_solver REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


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
    ik_solver_core
    moveit_core
    )

add_executable(test_rrt src/test_rrt.cpp)
target_include_directories(test_rrt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(test_rrt PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_directories(test_rrt PUBLIC
graph_core::graph_core
ur_ik_solver::ur_ik_solver
#moveit_collision_checker::moveit_collision_checker
#moveit_msgs::moveit_msgs
#visualization_msgs::visualization_msgs
#moveit_visual_tools::moveit_visual_tools
#tf2_eigen::tf2_eigen
#urdf::urdfdom
)

ament_target_dependencies(test_rrt ${ROS_DEPS})

#ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS_LIB} ${ROS_DEPS})

install(TARGETS
        test_rrt
    DESTINATION lib/${PROJECT_NAME}
    )


install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

ament_package()
