#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(frame,x,y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()



if __name__ == '__main__':
    frame = "map"
    #points = [(2,0),(3,0.5),(4.5,2),(5.5,1),(6.5,0),(6.5,1.5)]
    #points = [(1*2/3,0) , (2*2/3,0.5*2/3), (3*2/3,1*2/3), (2*2/3,2.5*2/3), (4*2/3,3*2/3)]
    #points = [(1,0) , (2,0)]
    #points = [(4,0)]
    #points = [(8/3,0)]
    #points = [(0,-3*2/3)]
    #points = [(0,0)]
    try:

        rospy.init_node('movebase_client_py')
        for j,i in enumerate(points):
            result = movebase_client(frame,i[0]*2/3,i[1]*2/3)
            if result:

                rospy.loginfo("Point:{} ({},{}) reached".format(j,i[0],i[1]))

    except rospy.ROSInterruptException:

        rospy.loginfo("Navigation test finished.")
