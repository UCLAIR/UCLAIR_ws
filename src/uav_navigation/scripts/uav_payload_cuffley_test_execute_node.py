#!/usr/bin/env python3

"""
This node is to execute guided payload mission in Cuffley

This mission is to put the UAV in GUIDED mode.

Firstly, it autonomously takeoff and navigate 5 waypoints. 
Secondly, the UAV navigates to a predefined waypoint to 
    execute payload dropping.
Laslty, the UAV is set to RTL and land autonomously.
"""

import rospy
from navigation_functions import *
from std_msgs.msg import Float64MultiArray, Bool, Int16


class Execution:
    def __init__(self):
        self.current_global_waypoint = Float64MultiArray()
        self.in_global_navigation = Bool()

        self.current_air_drop_waypoint = Float64MultiArray()
        self.in_air_drop_navigation = Bool()

        self.drop_number_counter = Int16()
        self.drop_checker = 0

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

        self.air_drop_waypoint_sub = rospy.Subscriber(
            name="air_drop_waypoint",
            data_class=Float64MultiArray,
            callback=self.air_drop_waypoint_sub_cb
        )

        self.in_air_drop_navigation_sub = rospy.Subscriber(
            name="in_air_drop_navigation",
            data_class=Bool,
            callback=self.in_air_drop_navigation_sub_cb
        )

        self.drop_number_counter_sub = rospy.Subscriber(
            name="drop_number",
            data_class=Int16,
            callback=self.drop_number_counter_sub_cb
        )


    # Call back functions

    def global_waypoint_sub_cb(self, msg):
        self.current_global_waypoint = msg.data

    def global_navigation_sub_cb(self, msg):
        self.in_global_navigation = msg.data

    def air_drop_waypoint_sub_cb(self, msg):
        self.current_air_drop_waypoint = msg.data

    def in_air_drop_navigation_sub_cb(self, msg):
        self.in_air_drop_navigation = msg.data

    def drop_number_counter_sub_cb(self, msg):
        self.drop_number_counter = msg


if __name__ == "__main__":
    try:

        rospy.init_node("uav_payload_cuffley_test_execute_node")

        uav = Navigation()

        mission = Execution()

        uav.wait4connect()

        while True:
            if uav.current_state.armed == True:
                rospy.loginfo("Armed")

                if uav.current_state.mode == "GUIDED":
                    rospy.loginfo("GUIDED MODE")
                    break

        uav.takeoff(25)

        rate = rospy.Rate(10)

        while (not rospy.is_shutdown()) and (mission.in_global_navigation):
            rate.sleep()

            rospy.loginfo("Executing waypoint mission")

            uav.set_speed(20) #cuufleyyyyy

            uav.set_global_destination(
                lat=mission.current_global_waypoint[0], lon=mission.current_global_waypoint[1],
                alt=(uav.current_global_position.altitude) - uav.geoid_height(
                    uav.current_global_position.latitude,
                    uav.current_global_position.longitude
                )
            )
        
        rospy.loginfo("All global waypoints have successfully been reached")

        while (not rospy.is_shutdown()) and (mission.in_air_drop_navigation):

            rate.sleep()

            rospy.loginfo("Executing air drop mission")

            if mission.in_air_drop_navigation == False:
                break

            uav.set_global_destination(
                lat=mission.current_air_drop_waypoint[0], lon=mission.current_air_drop_waypoint[1],
                alt=(uav.current_global_position.altitude) - uav.geoid_height(
                    uav.current_global_position.latitude,
                    uav.current_global_position.longitude
                )
            )

            uav.set_speed(10) #cufffffffleyyyy

            if uav.check_waypoint_reached() == False:

                mission.execute_drop_pub.publish(False)

                rospy.loginfo("Navigating to air drop target")

            else:
                while mission.drop_checker == mission.drop_number_counter.data:
                    rate.sleep()

                    if mission.drop_checker != mission.drop_number_counter.data:
                        break

                    rospy.loginfo(f"mission.drop_checker: {mission.drop_checker}")
                    rospy.loginfo(f"mission.drop_number_counter.data: {mission.drop_number_counter.data}")
                    rospy.loginfo("Payload dropping")
                    mission.execute_drop_pub.publish(True)
                
                mission.drop_checker += 1

        
        rospy.loginfo("All air drop targets have successfully been reached and dropped")

        uav.set_mode("RTL")

        rospy.loginfo("Returning to Launch")
            

    except KeyboardInterrupt:
        exit()