cmake_minimum_required(VERSION 3.0.2)
project(motor_driver)


find_package(catkin REQUIRED COMPONENTS
message_generation
  can_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
)


## Generate messages in the 'msg' folder
 add_message_files(
  FILES
   RobotMotor.msg
#   Message2.msg
 )



## Generate added messages and services with any dependencies listed here
#This line is still important for custom messages
 generate_messages(
   DEPENDENCIES
    std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES motor_driver
  CATKIN_DEPENDS can_msgs roscpp rospy std_msgs std_srvs message_runtime
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_executable(motor_driver src/can_handler.cpp src/motor_driver.cpp)
add_dependencies(motor_driver ${motor_driver_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_driver ${catkin_LIBRARIES} )
