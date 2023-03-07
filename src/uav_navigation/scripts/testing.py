#!/usr/bin/env python3

import rospy
from navigation_functions import *
from std_msgs.msg import Float64MultiArray, Bool


if __name__ == "__main__":

    try:
        rospy.init_node("uav_execute_node")

        uav = Navigation()

        uav.wait4connect()

        uav.set_mode("GUIDED")

        uav.wait4connect()

        uav.takeoff(10)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            



