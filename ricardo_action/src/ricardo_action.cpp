#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
//#include <rosbag/recorder.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float increment = 1.0;

std::string message = "Hooray, the base moved %f meters forward";
move_base_msgs::MoveBaseGoal goal;




int main (int argc, char **argv)
    {
        ros::init(argc,argv, "simple_navigation_goals");

        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        goal.target_pose.header.frame_id = "base_link";
        //We'll send a goal to the robot to move 1 meter forward
        goal.target_pose.pose.position.x = increment;
        goal.target_pose.pose.position.y = increment;
        goal.target_pose.pose.orientation.w = 1.0;

        // create a rosbag recorder object to record all topics
        //rosbag::RecorderOptions options;
        //options.record_all = true;
        //options.prefix = "my_bag";
        
        //rosbag::Recorder recorder(options);


        //wait for the action server to come up

        while (!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
        for (unsigned int i = 0; i < 3; ++i)
            {
                
                goal.target_pose.header.stamp = ros::Time::now();
                ROS_INFO("Sending goal");
                ac.sendGoal(goal);
                ac.waitForResult();

                if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO(message.c_str(),increment);
                else
                    ROS_INFO("The base failed to move forward 1 meter for some reason");
            }

             // stop recording and close the bag file
        //recorder.requestStop();
        //recorder.waitForComplete();
        //recorder.close();
        return 0;
    }

   