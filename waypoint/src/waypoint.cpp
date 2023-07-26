#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_localization/navsat_conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


double goalLatitude1 =  37.373856 - 0.00040;
double goalLongitude1 = 126.667110- 0.00040;
double goalLatitude2 =  37.374034 - 0.00040;
double goalLongitude2 = 126.667131- 0.00040;
double goalLatitude3 =  37.374034 - 0.00040;
double goalLongitude3 = 126.666857- 0.00040;
double goalLatitude4 =  37.374045 - 0.00040;
double goalLongitude4 = 126.666620- 0.00040;
double goalLatitude5 =  37.374313 - 0.00040;
double goalLongitude5 = 126.666576- 0.00040;
double goalLatitude6 =  37.374326 - 0.00040;
double goalLongitude6 = 126.666305- 0.00040;
//double goalLatitude5 = 37.373775;
//double goalLongitude5 = 126.667140;




double currentLatitude;
double currentLongitude;

double initialLatitude;
double initialLongitude;


double goalutm_x;
double goalutm_y;

double initialutm_x;
double initialutm_y;

double currentutm_x;
double currentutm_y;

std::string currentutm_zone;
std::string initialutm_zone;
std::string goalutm_zone;

geometry_msgs::TransformStamped globalTomap;

double distance = 1;
double initialangle;


void move_forward(double distance)
{
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;

    //We'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = distance;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal to move forward");
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the rover moved %f meter forward", distance);
    else
        ROS_INFO("The base failed to move forward %f meter for some reason", distance);
}



void get_initial_angle_and_frame(ros::NodeHandle& node, double& initial_angle, geometry_msgs::TransformStamped& transform)
{
    sensor_msgs::NavSatFix::ConstPtr msg1 = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/gps/fix", node);
    initialLatitude = msg1->latitude;
    initialLongitude = msg1->longitude;
    RobotLocalization::NavsatConversions::LLtoUTM(initialLatitude, initialLongitude, initialutm_y, initialutm_x, initialutm_zone);

    move_forward(distance);

    sensor_msgs::NavSatFix::ConstPtr msg2 = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/gps/fix", node);
    currentLatitude = msg2->latitude;
    currentLongitude = msg2->longitude;
    RobotLocalization::NavsatConversions::LLtoUTM(currentLatitude, currentLongitude, currentutm_y, currentutm_x, currentutm_zone);

    std::complex<double> complexNumber(currentutm_x - initialutm_x, currentutm_y - initialutm_y);
    initial_angle = -arg(complexNumber);

    transform.header.frame_id = "world";
    transform.child_frame_id = "map";
    transform.transform.translation.x = -initialutm_x;
    transform.transform.translation.y = -initialutm_y;
    ROS_INFO("initial X: %f initial Y: %f", -initialutm_x, -initialutm_y);
    transform.transform.translation.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0,0,initial_angle);

    transform.transform.rotation = tf2::toMsg(quaternion);





    ROS_INFO("testing");

   

    //ROS_INFO("TRANSFORMATION: %f %f %f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);   
}

void sendGoal(double goalLatitude, double goalLongitude, geometry_msgs::TransformStamped transform, ros::NodeHandle& node)
{
   
    RobotLocalization::NavsatConversions::LLtoUTM(goalLatitude, goalLongitude, goalutm_y, goalutm_x, goalutm_zone);

    geometry_msgs::PointStamped goalUtm;
    goalUtm.header.frame_id = "world";
    goalUtm.point.x = goalutm_x;
    goalUtm.point.y = goalutm_y;
    goalUtm.point.z = 0.0;

    geometry_msgs::PointStamped mapPoint;

    tf2::doTransform(goalUtm, mapPoint, transform);

    ROS_INFO("This is in the transform X: %f, Y: %f, Z: %f", transform.transform.translation.x,transform.transform.translation.y, transform.transform.translation.z);
    ROS_INFO("World frame: moving to X: %f, and Y:, %f", goalutm_x, goalutm_y);  
    ROS_INFO("Map frame: moving to X: %f, and Y:, %f", mapPoint.point.x, mapPoint.point.y);
    
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
       
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = mapPoint.point.x;
    goal.target_pose.pose.position.y = mapPoint.point.y;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)

    ROS_INFO("Hooray, the base moved 1 meter forward");
    else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

}



geometry_msgs::PointStamped testing_transforms()
{
    geometry_msgs::TransformStamped transform2;
    transform2.header.frame_id = "frame1";
    transform2.child_frame_id = "frame2";
    transform2.transform.translation.x = -293446.985454;
    transform2.transform.translation.y = -4138896.479295;
    transform2.transform.translation.z= 0.0;

    double initial_angle =0.0;// M_PI/4.0;
     tf2::Quaternion quaternion;
    quaternion.setRPY(0,0,initial_angle);

    transform2.transform.rotation = tf2::toMsg(quaternion);


    geometry_msgs::PointStamped goalUtm;
    goalUtm.header.frame_id = "frame1";
    goalUtm.point.x = 293437.865213;
    goalUtm.point.y = 4138903.922681;
    goalUtm.point.z = 0.0;

    geometry_msgs::PointStamped mapPoint;

    tf2::doTransform(goalUtm, mapPoint, transform2);

    ROS_INFO("x: %f y: %f z: %f", mapPoint.point.x, mapPoint.point.y,mapPoint.point.z);

    return mapPoint;
}




void sendGoal2(double goalLatitude, double goalLongitude, ros::NodeHandle& node)
{

    RobotLocalization::NavsatConversions::LLtoUTM(goalLatitude, goalLongitude, goalutm_y, goalutm_x, goalutm_zone);
    geometry_msgs::PointStamped point_world;
    point_world.header.frame_id = "world";
    point_world.point.x = goalutm_x;
    point_world.point.y = goalutm_y;
    point_world.point.z = 0.0;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);
    geometry_msgs::PointStamped point_map;

 while (ros::ok())
    {
        try
        {
             // Look up the transform from "world" to "map"
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", "world", ros::Time(0));
            // Transform the point from "world" to "map" coordinate frame
            
            tf2::doTransform(point_world, point_map, transformStamped);


            // Use the transformed point in the "map" coordinate frame as needed

            break;  // Once the transformation is obtained, exit the loop
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        ros::spinOnce();
        rate.sleep();
    }
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
       
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = point_map.point.x;
    goal.target_pose.pose.position.y = point_map.point.y;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)

    ROS_INFO("Hooray, the base moved got to destiny");
    else
    ROS_INFO("The base failed to move forward");

}




int main(int argc, char** argv)

{   
    ros::init(argc, argv,"waypoint_node");
    ros::NodeHandle nh;
  

    //testing_transforms();
    //get_initial_angle_and_frame(nh, initialangle, globalTomap);

    //sendGoal(goalLatitude, goalLongitude,globalTomap, nh);
    
    sendGoal2(goalLatitude1, goalLongitude1, nh);
    sendGoal2(goalLatitude2, goalLongitude2, nh);
    sendGoal2(goalLatitude3, goalLongitude3, nh);
    sendGoal2(goalLatitude4, goalLongitude4, nh);
    sendGoal2(goalLatitude5, goalLongitude5, nh);
    sendGoal2(goalLatitude6, goalLongitude6, nh);

   
    return 0;
}



