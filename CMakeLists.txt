cmake_minimum_required(VERSION 2.8.3)
project(comm_pipeline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  message_generation
)

add_service_files(
  FILES
  ActivateStag.srv
  GetTarget.srv
  FoundMarker.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
 CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(phases src/phase_manager.cpp src/phase.cpp src/phase_deliver.cpp src/phase_detected.cpp src/phase_ended.cpp src/phase_lost.cpp src/phase_scan.cpp src/phase_visual_servo.cpp src/scanner.cpp)
add_dependencies(phases comm_pipeline_generate_messages_cpp)

add_executable(planner src/planner_node.cpp src/planner.cpp)
target_link_libraries(planner phases ${catkin_LIBRARIES})
add_dependencies(planner comm_pipeline_generate_messages_cpp)
