cmake_minimum_required(VERSION 3.0.2)
project(adl200_core)


find_package(catkin REQUIRED COMPONENTS
  adl200_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)


catkin_package(
 LIBRARIES adl200_core
 CATKIN_DEPENDS roscpp std_msgs tf nav_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(adl200_driver src/adl200_driver.cpp)
target_link_libraries(adl200_driver ${catkin_LIBRARIES})

