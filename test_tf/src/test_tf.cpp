#include <ros/ros.h>
#include <robot_localization/navsat_conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_transformer");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Assuming you have a point in the "world" coordinate frame
    geometry_msgs::PointStamped point_world;
    point_world.header.frame_id = "world";
    point_world.point.x = 293437.865213;
    point_world.point.y = 4138903.922681;
    point_world.point.z = 0.0;
    ros::Rate rate(10.0);
    while (ros::ok())
    {
        try
        {
            // Look up the transform from "world" to "map"
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", "world", ros::Time(0));

            // Transform the point from "world" to "map" coordinate frame
            geometry_msgs::PointStamped point_map;
            tf2::doTransform(point_world, point_map, transformStamped);

            // Access the transformed point in the "map" coordinate frame
            /*double x_map = point_map.point.x;
            double y_map = point_map.point.y;
            double z_map = point_map.point.z;*/
            ROS_INFO("x: %f y: %f z: %f", point_map.point.x, point_map.point.y, point_map.point.z);

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

    return 0;
}