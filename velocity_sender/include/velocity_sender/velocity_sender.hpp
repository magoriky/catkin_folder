// _VELOCITY_SENDER_
#ifndef _VELOCITY_SENDER_
#define _VELOCITY_SENDER_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>

ros::Publisher velPub;
geometry_msgs::Twist msg;




class velocity_node
{
    private:
        ros::NodeHandle n;
        ros::Publisher velPub;
    public:
        // constructor
        velocity_node(ros::NodeHandle n){ }

        void driving()
        {
            velPub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

            while(n.ok() & ros::ok())
            {
                msg.linear.x = 1;
                msg.angular.z = 1;
                velPub.publish(msg);
            }

        }
}; //velocity_node

#endif // _VELOCITY_SENDER_
