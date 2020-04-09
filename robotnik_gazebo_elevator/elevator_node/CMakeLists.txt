cmake_minimum_required(VERSION 2.8.3)
project(elevator_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  robotnik_elevator_interface_msgs
  robotnik_elevator_component
  rcomponent
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  elevatorcabmsg.msg
  doorleftmsg.msg
  doorrightmsg.msg
  floorextensionmsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetControl.srv
   ReleaseControl.srv
   GoToFloor.srv
   SetDoor.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(elevator_node src/elevator_node.cpp)
target_link_libraries(elevator_node ${catkin_LIBRARIES})
add_dependencies(elevator_node elevator_node_generate_messages_cpp)

add_executable(fake_physics_elevator_node src/fake_physics_elevator_node.cpp)
target_link_libraries(fake_physics_elevator_node ${catkin_LIBRARIES})
add_dependencies(fake_physics_elevator_node elevator_node_generate_messages_cpp)
