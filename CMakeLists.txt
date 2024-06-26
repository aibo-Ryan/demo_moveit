cmake_minimum_required(VERSION 3.10.2)
project(demo_moveit)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  rviz_visual_tools
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
  moveit_servo
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})


ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(include)


# -------------------------------------------------------------- executable
add_executable(demo_moveit
  src/demo_moveit.cpp)

target_include_directories(demo_moveit
    PUBLIC include)

ament_target_dependencies(demo_moveit
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# -------------------------------------------------------------- executable
add_executable(visualize_in_rviz
  src/visualize_in_rviz.cpp)

target_include_directories(visualize_in_rviz
    PUBLIC include)

ament_target_dependencies(visualize_in_rviz
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# -------------------------------------------------------------- executable
add_executable(planning_around_objects
  src/planning_around_objects.cpp)

target_include_directories(planning_around_objects
    PUBLIC include)

ament_target_dependencies(planning_around_objects
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
# -------------------------------------------------------------- install
  
install(TARGETS demo_moveit visualize_in_rviz planning_around_objects
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
