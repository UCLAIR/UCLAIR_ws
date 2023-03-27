#!/usr/bin/env python3

import rospy

if __name__ == "__main__":
    rospy.init_node("payload_dropping_node")


    while (not rospy.is_shutdown()):
        rospy.loginfo("working")