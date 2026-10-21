cmake_minimum_required(VERSION 3.1)
project(amcl1)

include(CheckIncludeFile)
include(CheckSymbolExists)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
)

find_package(Boost REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_msgs
    tf2_ros
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(src/include)

check_include_file(unistd.h HAVE_UNISTD_H)
if (HAVE_UNISTD_H)
  add_definitions(-DHAVE_UNISTD_H)
endif (HAVE_UNISTD_H)

check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)
if (HAVE_DRAND48)
  add_definitions(-DHAVE_DRAND48)
endif (HAVE_DRAND48)

add_library(amcl_pf
                    src/amcl/pf/pf.c
                    src/amcl/pf/pf_kdtree.c
                    src/amcl/pf/pf_pdf.c
                    src/amcl/pf/pf_vector.c
                    src/amcl/pf/eig3.c
                    src/amcl/pf/pf_draw.c)

add_library(amcl_map
                    src/amcl/map/map.c
                    src/amcl/map/map_cspace.cpp
                    src/amcl/map/map_range.c
                    src/amcl/map/map_store.c
                    src/amcl/map/map_draw.c)

add_library(amcl_sensors
                    src/amcl/sensors/amcl_sensor.cpp
                    src/amcl/sensors/amcl_odom.cpp
                    src/amcl/sensors/amcl_laser.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)


add_executable(amcl1
                       src/amcl_node.cpp)
add_dependencies(amcl1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(amcl1
    amcl_sensors amcl_map amcl_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install(TARGETS
    amcl1
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    amcl_sensors amcl_map amcl_pf
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/amcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY cfg/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)

