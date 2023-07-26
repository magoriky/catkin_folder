cmake_minimum_required(VERSION 3.0.2)
project(ricardo_action)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(ricardo_action src/ricardo_action.cpp)

##Link libraries
target_link_libraries(ricardo_action ${catkin_LIBRARIES})

add_dependencies(ricardo_action ${catkin_EXPORTED_TARGETS})