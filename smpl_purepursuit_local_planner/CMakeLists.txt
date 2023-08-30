################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(smpl_purepursuit_local_planner)

add_compile_options(-std=c++17)
################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  smpl_msgs
  tf
  costmap_2d
  nav_core
  nav_msgs
  pluginlib
  tf2
  tf2_ros
  base_local_planner
  )

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs smpl_msgs tf costmap_2d nav_core nav_msgs pluginlib tf2 tf2_ros base_local_planner
  DEPENDS
)

################################################################################
# Build
################################################################################

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(smpl_purepursuit_local_planner
  src/smpl_purepursuit_local_planner.cpp
  )
add_dependencies(smpl_purepursuit_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(smpl_purepursuit_local_planner
  ${catkin_LIBRARIES}
  )


#add_library(regulated_pure_pursuit_controller
#  src/regulated_pure_pursuit_controller.cpp)
#
#target_link_libraries(regulated_pure_pursuit_controller ${catkin_LIBRARIES})
#add_dependencies(regulated_pure_pursuit_controller nav_msgs_gencpp)

################################################################################
# Install
################################################################################

install(TARGETS smpl_purepursuit_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES
  purepursuit_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )


################################################################################
# Test
################################################################################


