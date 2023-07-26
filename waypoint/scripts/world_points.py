#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pyproj import Proj
import geonav_transform.geonav_conversions as gc

def get_xy_based_on_lat_long(lat,lon):
    olon = 126.667260
    olat = 37.373825
    xg2, yg2 = gc.ll2xy(lat,lon,olat,olon)
    return xg2, yg2

def movebase_client(frame,lati,longi):
    
    x, y =get_xy_based_on_lat_long(lati,longi)
    print("Moving {} forward (x) and {} left (y), distance {} from origin".format(y, -x,np.sqrt((y)**2 + (x)**2) ))

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = round(y,2)   #The output of "get_xy_based_on_lat_long" assumes north is in Y direction
    goal.target_pose.pose.position.y = round(-x,2) #The output of "get_xy_based_on_lat_long" assumes east is in X direction
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
    points = [(37.373818, 126.667251), (37.373811, 126.667246), (37.373803, 126.667237),(37.373795, 126.667228), 
    (37.373791, 126.667222), (37.373783, 126.667216), (37.373774, 126.667209), (37.373767, 126.667202),
    (37.373759, 126.667196), (37.373751, 126.667190)]
    #points = [(37.2225,126.402)]
    #points = [(37.373561,126.667138),(37.373623,126.667061)]
    #points = [ (37.373683,126.667119), (37.373623,126.667061)]
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
            rospy.loginfo("Sending point:{} ({},{}) , latitud, longitud".format(j,i[0],i[1]))
            result = movebase_client(frame,i[0],i[1])
            if result:

                rospy.loginfo("Point:{} ({},{}) reached".format(j,i[0],i[1]))

    except rospy.ROSInterruptException:

        rospy.loginfo("Navigation test finished.")
