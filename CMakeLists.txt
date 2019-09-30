cmake_minimum_required(VERSION 2.8.3)
project(fms_rob)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation geometry_msgs nav_msgs actionlib_msgs actionlib dynamic_reconfigure)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
	FILES
  RobActionSelect.msg
  RobActionStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
  dockPose.srv
  dockMove.srv
  dockRotate.srv
  parkPose.srv
)

## Generate actions in the 'action' folder
add_action_files(
	DIRECTORY action
	FILES
  dockUndock.action
)

## Generate added messages, services and actions with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs  
   actionlib_msgs 
   nav_msgs # Or other packages containing msgs
)

generate_dynamic_reconfigure_options(
  cfg/dynamic_params.cfg
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)


