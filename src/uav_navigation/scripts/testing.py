#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def callback(msg):
    rospy.loginfo(msg)

def listener():
    rospy.init_node("testing")

    rospy.Subscriber(
        name="/osbtacles_r",
        data_class=Float64MultiArray,
        callback=callback
    )

    rospy.spin()

if __name__ == "__main__":
    listener()



