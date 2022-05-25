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


add_executable(planner src/planner_node.cpp src/planner.cpp)
target_link_libraries(planner phases ${catkin_LIBRARIES})
add_dependencies(planner phases comm_pipeline_gencpp)

add_executable(service_client test/service_client.cpp)
target_link_libraries(service_client ${catkin_LIBRARIES})
add_dependencies(service_client comm_pipeline_gencpp)

add_executable(service_server test/service_server.cpp)
target_link_libraries(service_server ${catkin_LIBRARIES})
add_dependencies(service_server comm_pipeline_gencpp)

add_executable(fake_marker test/fake_marker.cpp)
target_link_libraries(fake_marker ${catkin_LIBRARIES})
add_dependencies(fake_marker comm_pipeline_gencpp)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(test-planner test/test_planner.test test/test_planner.cpp src/planner.cpp)
#   target_link_libraries(test-planner phases ${catkin_LIBRARIES})
#   add_dependencies(test-planner phases comm_pipeline_gencpp)
# endif()