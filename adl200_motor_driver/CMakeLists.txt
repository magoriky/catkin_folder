cmake_minimum_required(VERSION 3.0.2)
project(adl200_motor_driver)


find_package(catkin REQUIRED COMPONENTS
  adl200_msgs
  can_msgs
  roscpp
  std_msgs
  std_srvs
)


catkin_package(

  LIBRARIES adl200_motor_driver
  CATKIN_DEPENDS can_msgs roscpp std_msgs std_srvs 

)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(adl200_motor_driver src/can_handler.cpp src/motor_driver.cpp)
target_link_libraries(adl200_motor_driver ${catkin_LIBRARIES} )
