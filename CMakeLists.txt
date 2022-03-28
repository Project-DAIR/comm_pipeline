cmake_minimum_required(VERSION 2.8.3)
project(comm_pipeline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros_msgs
)

catkin_package(
#  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_node src/cmd_test.cpp)

target_link_libraries(cmd_node
  ${catkin_LIBRARIES}
)
