#!/usr/bin/env python
import rospy
from mavros_msgs.msg import WaypointList

def callback(data):
    waypoint_list = []
    for waypoint in data.waypoints:
        if (waypoint.command == 16) and (waypoint.x_lat != 0) and (waypoint.y_long != 0):
            waypoint_list.append([waypoint.x_lat, waypoint.y_long])
    
    rospy.loginfo(waypoint_list)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("mavros/mission/waypoints", WaypointList, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()