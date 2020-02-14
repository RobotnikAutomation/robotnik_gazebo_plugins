cmake_minimum_required(VERSION 2.8.3)
project(robotnik_gazebo_ground_truth_odom)

option(ENABLE_DISPLAY_TESTS "Enable the building of tests that requires a display" OFF)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  cv_bridge
  polled_camera
  diagnostic_updater
  camera_info_manager
  std_msgs
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(Boost REQUIRED COMPONENTS thread)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

generate_dynamic_reconfigure_options(
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 

  robotnik_gazebo_ground_truth_odom 


  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  camera_info_manager
  std_msgs
)
add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_definitions(-fPIC) # what is this for?

add_library(robotnik_gazebo_ground_truth_odom src/robotnik_gazebo_ground_truth_odom.cpp)
target_link_libraries(robotnik_gazebo_ground_truth_odom ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##
## Add your new plugin here
##

install(TARGETS
  robotnik_gazebo_ground_truth_odom
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

