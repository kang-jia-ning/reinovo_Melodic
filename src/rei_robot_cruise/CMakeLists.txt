cmake_minimum_required(VERSION 3.0.2)
project(rei_robot_cruise)

add_compile_options(-std=c++11 -fPIC)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  rospy
  visualization_msgs
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  NavGoal.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetGoals.srv
  SetNavOrder.srv
  StartStopNav.srv
  RemoveGoal.srv
#   Service2.srv
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
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rei_robot_cruise
  CATKIN_DEPENDS actionlib move_base_msgs roscpp visualization_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ library
add_library(${PROJECT_NAME}
  src/cruise.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(example_cruise test/example.cpp)
add_dependencies(example_cruise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_cruise
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


install(DIRECTORY
   launch
   params
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


