#!/usr/bin/env python2.7

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import tf
# Global variable to store the received odometry data
received_odometry = None

# Define the callback function to handle incoming Odometry messages
def odometry_callback(msg):
    global received_odometry
    received_odometry = msg

def rotate_to_goal(desired_angle, angular_velocity):
    # Initialize the ROS node with a unique name
    rospy.init_node('rotate_to_goal', anonymous=True)
    rospy.loginfo("initializing node")

    # Subscribe to the "odometry/filtered" topic, specifying the message type and the callback function
    rospy.Subscriber('odometry/filtered', Odometry, odometry_callback)

    # Create a ROS publisher for the Twist message
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Set the rate at which to publish the Twist message
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Check if we have received odometry data
        if received_odometry is not None:
            # Extract the orientation information from the Odometry message
            orientation = received_odometry.pose.pose.orientation

            # Convert the quaternion to Euler angles (roll, pitch, yaw)
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])

            # Calculate the difference between the desired angle and the current angle
            angle_difference = desired_angle - yaw

            # Normalize the angle to be in the range [-pi, pi]
            while angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            while angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            # Check if the robot is not in the necessary angle
            if abs(angle_difference) > math.radians(1):  # Adjust the threshold as needed
                # Create a Twist message with the desired angular velocity
                twist = Twist()
                twist.angular.z = angular_velocity

                # Publish the Twist message to rotate the robot
                twist_pub.publish(twist)
            else:
                # If the robot is in the necessary angle, stop the rotation
                twist = Twist()
                twist.angular.z = 0.0
                twist_pub.publish(twist)
                break  # Exit the loop as the goal angle is reached

        # Sleep for a short duration before publishing the next Twist message
        rate.sleep()

if __name__ == '__main__':
    try:
        # Set the desired angle in radians and the desired angular velocity in rad/s
        desired_angle = math.radians(90)  # For example, rotate to 90 degrees
        angular_velocity = 0.5  # 0.5 rad/s

        # Call the function to rotate the robot to the desired angle
        rotate_to_goal(desired_angle, angular_velocity)
    except rospy.ROSInterruptException:
        pass
