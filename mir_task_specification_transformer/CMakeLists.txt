cmake_minimum_required(VERSION 2.8.3)
project(mir_task_specification_transformer)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
)

add_definitions("-std=c++0x")
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(task_specification_transformer_node
  ros/src/task_specification_transformer_node.cpp
  ros/src/task_specification_transformer.cpp
)

target_link_libraries(task_specification_transformer_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)