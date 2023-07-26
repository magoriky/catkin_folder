#include <ros/ros.h>
#include <ros/console.h>
#include <adl200_core/adl200_driver.hpp>

using namespace aidl;

int main (int argc, char **argv) {
    ros::init(argc, argv, "adl200_driver");
    ros::NodeHandle nh;
    adl200BodyNode adl200 = adl200BodyNode(&nh);

	ros::AsyncSpinner spinner(0);
    spinner.start();
	
    ROS_INFO("ADL200 Driver Node is started");
	ros::waitForShutdown();
}