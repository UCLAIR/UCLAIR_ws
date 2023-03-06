#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import TerrainReport

def callback(msg):
    rospy.loginfo(msg.current_height)

def listener():
    rospy.init_node("testing")

    rospy.Subscriber(
        name="mavros/terrain/report",
        data_class=TerrainReport,
        callback=callback
    )

    rospy.spin()

if __name__ == "__main__":
    listener()



