cmake_minimum_required(VERSION 3.0.2)
project(simple_navigation_goals)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)

## Add include directories
include_directories(
 include ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)

## Link libraries
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

## Add dependencies
add_dependencies(simple_navigation_goals ${catkin_EXPORTED_TARGETS})
