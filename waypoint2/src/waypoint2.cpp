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
#include <std_msgs/Bool.h>
#include <ros/package.h>
#include <fstream>

int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints = 0;
std::string goalutm_zone;
geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;

std::string path_local, path_abs;

std::vector <std::pair<double, double>> waypointVect;
std::vector<std::pair < double, double> > ::iterator iter; //init. iterator
double latiGoal, longiGoal, latiNext, longiNext, UTM_pointx, UTM_pointy, UTM_pointx_next, UTM_pointy_next;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int countWaypointsInFile(std::string path_local)
{
    path_abs = ros::package::getPath("waypoint2") + path_local;
    std::ifstream fileCount(path_abs.c_str());
    if(fileCount.is_open())
    {
        double lati = 0;
        while(!fileCount.eof())
        {
            fileCount >> lati;
            ++count;
        }
        count = count - 1;
        numWaypoints = count / 2;
        ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
        fileCount.close();
    }
    else
    {
        std::cout << "Unable to open waypoint file" << std::endl;
        ROS_ERROR("Unable to open waypoint file");
    }
    return numWaypoints;
}



std::vector <std::pair<double, double>> getWaypoints(std::string path_local)
{
    double lati = 0, longi = 0;

    path_abs = ros::package::getPath("waypoint2") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    for(int i = 0; i < numWaypoints; i++)
    {
        fileRead >> lati;
        fileRead >> longi;
        waypointVect.push_back(std::make_pair(lati, longi));
    }
    fileRead.close();

    //Outputting vector
    ROS_INFO("The following GPS Waypoints have been set:");
    for(std::vector < std::pair < double, double >> ::iterator iterDisp = waypointVect.begin(); iterDisp != waypointVect.end();
    iterDisp++)
    {
        ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
    }
    return waypointVect;
}















move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next, bool last_point)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    if(last_point == false)
    {
        tf::Matrix3x3 rot_euler;
        tf::Quaternion rot_quat;

        // Calculate quaternion
        float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
        float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
        float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
        float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
        yaw_curr = atan2(delta_y, delta_x);

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
        rot_euler.getRotation(rot_quat);

        goal.target_pose.pose.orientation.x = rot_quat.getX();
        goal.target_pose.pose.orientation.y = rot_quat.getY();
        goal.target_pose.pose.orientation.z = rot_quat.getZ();
        goal.target_pose.pose.orientation.w = rot_quat.getW();
    }
    else
    {
        goal.target_pose.pose.orientation.w = 1.0;
    }

    return goal;
}




int main(int argc, char** argv)

{   
    ros::init(argc, argv,"waypoint_node");
    ros::NodeHandle nh;
    ROS_INFO("Initiated gps_waypoint node");
     MoveBaseClient ac("move_base", true);
    ros::Publisher pubWaypointNodeEnded = nh.advertise<std_msgs::Bool>("/outdoor_waypoint_nav/waypoint_following_status", 100);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        wait_count++;
        if(wait_count > 3)
        {
            ROS_ERROR("move_base action server did not come up, killing gps_waypoint node...");
            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }
        ROS_INFO("Waiting for the move_base action server to come up");
    }
     //Get Longitude and Latitude goals from text file

    //Count number of waypoints
    path_local = "/waypoint_files/points_outdoor.txt";
    
    numWaypoints = countWaypointsInFile(path_local);

    //Reading waypoints from text file and output results
    waypointVect = getWaypoints(path_local);





        // Iterate through vector of waypoints for setting goals
    for(iter = waypointVect.begin(); iter < waypointVect.end(); iter++)
    {
        //Setting goal:
        latiGoal = iter->first;
        longiGoal = iter->second;
        bool final_point = false;

        //set next goal point if not at last waypoint
        if(iter < (waypointVect.end() - 1))
        {
            iter++;
            latiNext = iter->first;
            longiNext = iter->second;
            iter--;
        }
        else //set to current
        {
            latiNext = iter->first;
            longiNext = iter->second;
            final_point = true;
        }

        ROS_INFO("Received Latitude goal:%.8f", latiGoal);
        ROS_INFO("Received longitude goal:%.8f", longiGoal);
        RobotLocalization::NavsatConversions::LLtoUTM(latiGoal, longiGoal, UTM_pointx, UTM_pointy, goalutm_zone);
        RobotLocalization::NavsatConversions::LLtoUTM(latiNext, longiNext, UTM_pointx_next, UTM_pointy_next, goalutm_zone);

        geometry_msgs::PointStamped UTM_point;
        UTM_point.header.frame_id = "world";
        UTM_point.point.x = UTM_pointx;
        UTM_point.point.y = UTM_pointy;
        UTM_point.point.z = 0.0;

        geometry_msgs::PointStamped UTM_next;
        UTM_next.header.frame_id = "world";
        UTM_next.point.x = UTM_pointx_next;
        UTM_next.point.y = UTM_pointy_next;
        UTM_next.point.z = 0.0;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(10.0);
        while (ros::ok())
    {
        try
        {
             // Look up the transform from "world" to "map"
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", "world", ros::Time(0));
            // Transform the point from "world" to "map" coordinate frame
            
            tf2::doTransform(UTM_point, map_point, transformStamped);
            tf2::doTransform(UTM_next, map_next, transformStamped);


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

        //Build goal to send to move_base
        move_base_msgs::MoveBaseGoal goal = buildGoal(map_point, map_next, final_point); //initiate a move_base_msg called goal

        // Send Goal
        ROS_INFO("Sending goal");
        ac.sendGoal(goal); //push goal to move_base node

        //Wait for result
        ac.waitForResult(); //waiting to see if move_base was able to reach goal

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Husky has reached its goal!");
            //switch to next waypoint and repeat
        }
        else
        {
            ROS_ERROR("Husky was unable to reach its goal. GPS Waypoint unreachable.");
            ROS_INFO("Exiting node...");
            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }
    } 


    ROS_INFO("Husky has reached all of its goals!!!\n");
    ROS_INFO("Ending node...");

    // Notify joy_launch_control that waypoint following is complete
    std_msgs::Bool node_ended;
    node_ended.data = true;
    pubWaypointNodeEnded.publish(node_ended);

    ros::shutdown();
    ros::spin();
    return 0;
}


