#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def callback(msg):
    rospy.loginfo(msg)

def listener():
    rospy.init_node("testing")

    rospy.Subscriber(
        name="mavros/global_position/global",
        data_class=NavSatFix,
        callback=callback
    )

    rospy.spin()

if __name__ == "__main__":
    listener()



