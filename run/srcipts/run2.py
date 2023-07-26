#!/usr/bin/env python2.7

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math

# Global variables to store the received odometry data and goal position
received_odometry = None
goal_position = Point()

# Define the callback function to handle incoming Odometry messages
def odometry_callback(msg):
    global received_odometry
    received_odometry = msg

def set_goal_position(x, y):
    global goal_position
    goal_position.x = x
    goal_position.y = y

def get_distance_to_goal(current_position):
    return math.sqrt((current_position.x - goal_position.x)**2 + (current_position.y - goal_position.y)**2)

def calculate_desired_angle(current_position):
    # Calculate the desired angle between the robot's current position and the goal position
    desired_angle = math.atan2(goal_position.y - current_position.y, goal_position.x - current_position.x)
    return desired_angle

def rotate_to_goal_and_move_forward(angular_velocity, forward_velocity):
    # Initialize the ROS node with a unique name
    rospy.init_node('rotate_to_goal_and_move_forward', anonymous=True)
    rospy.loginfo("initializing")

    # Subscribe to the "odometry/filtered" topic, specifying the message type and the callback function
    rospy.Subscriber('odometry/filtered', Odometry, odometry_callback)

    # Create a ROS publisher for the Twist message
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Set the rate at which to publish the Twist message
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Check if we have received odometry data and goal position
        if received_odometry is not None:
            rospy.loginfo("received odometry")
            # Extract the orientation information from the Odometry message
            orientation = received_odometry.pose.pose.orientation

            # Convert the quaternion to Euler angles (roll, pitch, yaw)
            quaternion = (
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            )
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

            # Calculate the desired angle based on the robot's current position
            current_position = received_odometry.pose.pose.position
            desired_angle = calculate_desired_angle(current_position)

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
                twist.linear.x = 0.0  # Stop forward movement
            else:
                # If the robot is in the necessary angle, stop the rotation
                twist = Twist()
                twist.angular.z = 0.0

                # Check the distance to the goal position
                distance_to_goal = get_distance_to_goal(current_position)

                if distance_to_goal > 0.05:  # 5 cm distance threshold (adjust as needed)
                    # If the distance to the goal is more than 5 cm, move forward
                    twist.linear.x = forward_velocity
                else:
                    # If the distance to the goal is within 5 cm, stop forward movement
                    twist.linear.x = 0.0

            # Publish the Twist message
            twist_pub.publish(twist)

        # Sleep for a short duration before publishing the next Twist message
        rate.sleep()

if __name__ == '__main__':
    try:
        # Set the desired angular velocity in rad/s and the forward velocity in m/s
        angular_velocity = 0.5  # 0.5 rad/s
        forward_velocity = 0.2  # 0.2 m/s

        # Set the goal position (x, y) in meters
        goal_x = 0.0
        goal_y = 5.0
        set_goal_position(goal_x, goal_y)

        # Call the function to rotate the robot to align with the goal x and y coordinates, then move forward to the goal
        rotate_to_goal_and_move_forward(angular_velocity, forward_velocity)
    except rospy.ROSInterruptException:
        pass
