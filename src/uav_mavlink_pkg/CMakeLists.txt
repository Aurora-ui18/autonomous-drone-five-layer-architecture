cmake_minimum_required(VERSION 3.0.2)
project(uav_mavlink_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  actionlib_msgs
  message_generation
  tf
  tf2_ros
  cv_bridge
  visualization_msgs
  std_srvs
)

## System dependencies are found with CMake conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DetectionResult.msg
  UavState.msg
  Waypoint.msg
  WaypointList.msg
  MissionCommand.msg
  MissionStatus.msg
  SafetyEvent.msg
  FireSource.msg
  FireTruckCmd.msg
  TargetBucket.msg
  QRResult.msg
  GuidePath.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetUavState.srv
  GetLocalPosition.srv
  PlanRoute.srv
  DetectLandingPad.srv
  ExecuteLanding.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Takeoff.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uav_mavlink_pkg
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    actionlib_msgs
    message_runtime
    tf
    tf2_ros
    cv_bridge
    visualization_msgs
    std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# 框架骨架节点（执行层 / 定位层 / 安全旁路 / 记录层）。
# 业务层节点（perception / planning / decision）按 docs/框架设计文档.md 第 10 章扩展后,
# 将新节点追加到下方列表。
catkin_install_python(PROGRAMS
  scripts/execution/flight_controller_node.py
  scripts/execution/gimbal_executor_node.py
  scripts/execution/landing_executor_node.py
  scripts/execution/led_beeper_node.py
  scripts/localization/state_estimator_node.py
  scripts/localization/coordinate_transformer_node.py
  scripts/localization/landing_pad_locator_node.py
  scripts/logging/flight_recorder_node.py
  scripts/safety/safety_supervisor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY scripts/common/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/common
)
