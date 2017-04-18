cmake_minimum_required(VERSION 2.8.3)
project(overtaking_maneuver)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings -g ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  dynamic_reconfigure
  tf
  tf2
  message_generation
)

add_service_files(
  FILES
  PublishOvertakingTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

generate_dynamic_reconfigure_options(
  cfg/OvertakingManeuverInputs.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(overtaking_maneuver_node src/overtaking_maneuver_node.cpp src/overtaking_maneuver.cpp)
add_dependencies(overtaking_maneuver_node overtaking_maneuver_node_generate_messages_cpp ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(overtaking_maneuver_node ${catkin_LIBRARIES})

add_executable(overtaking_state_machine src/overtaking_state_machine.cpp)
add_dependencies(overtaking_state_machine overtaking_maneuver_node_generate_messages_cpp ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(overtaking_state_machine ${catkin_LIBRARIES})

add_executable(button_simulator src/button_simulator.cpp)		
target_link_libraries(button_simulator ${catkin_LIBRARIES})
