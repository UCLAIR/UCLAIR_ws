#!/usr/bin/env python3

"""
This node is to execute mission in Cuffley

The first mission is to check 

"""

import rospy
from navigation_functions import *
from std_msgs.msg import Float64MultiArray, Bool
import time

class Execution:
    def __init__(self):
        self.current_global_waypoint = Float64MultiArray()
        self.in_global_navigation = Bool()
        self.in_air_drop_navigation = Bool()
        self.execute_drop = Bool()
    
        # ROS Publishers
        self.execute_drop_pub = rospy.Publisher(
            name="execute_drop_bottle",
            data_class=Bool,
            queue_size=10
        )

        # ROS Subscribers

        self.global_waypoint_sub = rospy.Subscriber(
            name="global_waypoint",
            data_class=Float64MultiArray,
            callback=self.global_waypoint_sub_cb
        )

        self.global_navigation_sub = rospy.Subscriber(
            name="global_navigation_mission",
            data_class=Bool,
            callback=self.global_navigation_sub_cb
        )

        self.in_air_drop_navigation_sub = rospy.Subscriber(
            name="in_air_drop_navigation",
            data_class=Bool,
            callback=self.in_air_drop_navigation_sub_cb
        )

    # Call back functions

    def global_waypoint_sub_cb(self, msg):
        self.current_global_waypoint = msg.data

    def global_navigation_sub_cb(self, msg):
        self.in_global_navigation = msg.data

    def in_air_drop_navigation_sub_cb(self, msg):
        self.in_air_drop_navigation = msg.data


if __name__ == "__main__":
    try:

        rospy.init_node("uav_guided_cuffley_test_execute_node")

        uav = Navigation()

        mission = Execution()

        uav.wait4connect()

        uav.set_mode("GUIDED")

        while (uav.current_state.armed) == False:
            rospy.loginfo("Waiting for the UAV to be armed")
            time.sleep(2)

        uav.takeoff(5)

        rate = rospy.Rate(10)

        while (not rospy.is_shutdown()) and mission.in_global_navigation:
            rate.sleep()

            rospy.loginfo("Executing waypoint mission")

            # uav.set_speed(5)

            uav.set_global_destination(
                lat=mission.current_global_waypoint[0], lon=mission.current_global_waypoint[1],
                alt=(uav.current_global_position.altitude) - uav.geoid_height(
                    uav.current_global_position.latitude,
                    uav.current_global_position.longitude
                )
            )
        

        rospy.loginfo("All global waypoints have successfully been reached")
        uav.set_mode("RTL")
        rospy.loginfo("Proceeding to RTL")

    except KeyboardInterrupt:
        exit()

