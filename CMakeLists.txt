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
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
 CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_node src/cmd_test.cpp)
target_link_libraries(cmd_node ${catkin_LIBRARIES})

add_executable(mode_test_node src/mode_test.cpp)
target_link_libraries(mode_test_node ${catkin_LIBRARIES})

add_executable(comm_node src/comm_pipeline.cpp)
target_link_libraries(comm_node ${catkin_LIBRARIES})

add_executable(service_client src/service_client.cpp)
target_link_libraries(service_client ${catkin_LIBRARIES})
add_dependencies(service_client comm_pipeline_gencpp)

add_executable(service_server src/service_server.cpp)
target_link_libraries(service_server ${catkin_LIBRARIES})
add_dependencies(service_server comm_pipeline_gencpp)
