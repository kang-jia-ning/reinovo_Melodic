cmake_minimum_required(VERSION 2.8.3)
project(auto_charging)

find_package(catkin REQUIRED COMPONENTS
  relative_move
  move_base_msgs
  actionlib
  geometry_msgs
  roscpp
  tf
  message_generation
  ar_pose
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetCharge.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auto_charging
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(auto_charge
  src/auto_charge.cpp
)
add_dependencies(auto_charge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(auto_charge
  ${catkin_LIBRARIES}
)
