#!/usr/bin/env python3

import rospy
from navigation_functions import *
from std_msgs.msg import Float64MultiArray, Bool


class Execution:
    def __init__(self):
        self.current_global_waypoint = Float64MultiArray()
        self.in_global_navigation = Bool()
        self.in_local_navigation = Bool()
        self.current_local_waypoint = Float64MultiArray()

        # ROS Publishers

    
        # ROS Subscribers

        # Subscribing global_waypoint topic from global_navigation_node
        self.global_waypoint_sub = rospy.Subscriber(
            name="global_waypoint",
            data_class=Float64MultiArray,
            callback=self.global_waypoint_sub_cb
        )

        # Subcribing global_navigation_mission topic from global_navigation_node
        self.global_navigation_sub = rospy.Subscriber(
            name="global_navigation_mission",
            data_class=Bool,
            callback=self.global_navigation_sub_cb
            )

        # Subscribing obstacle_avoiding topic from local_navigation_node
        self.obstacle_avoiding_sub = rospy.Subscriber(
            name="obstacle_avoiding",
            data_class=Bool,
            callback=self.obstacle_avoiding_sub_cb
        )

        # Subscribing local_waypoints topic with current local navigation waypoints from local_navigation_node
        self.local_waypoints_sub = rospy.Subscriber(
            name="local_waypoints",
            data_class=Float64MultiArray,
            callback=self.local_waypoints_sub_cb
        )


    # Call back functions

    def global_waypoint_sub_cb(self, msg):
        self.current_global_waypoint = msg.data

    def global_navigation_sub_cb(self, msg):
        self.in_global_navigation = msg.data

    def obstacle_avoiding_sub_cb(self, msg):
        self.in_local_navigation = msg.data

    def local_waypoints_sub_cb(self, msg):
        self.current_local_waypoint = msg.data


    # Convert the data into publisherable data for Float64MultiArray()
    def publish_float64multiarray_data(self, d):
        publishing_data = Float64MultiArray()

        publishing_data.data = d

        return publishing_data


if __name__ == "__main__":

    try:
        # Initialise UAV ROS node
        rospy.init_node("uav_execute_node")

        # Initiatlise the Navigation class from navigation_functions.py
        uav = Navigation()

        # Initialise the Execution class
        mission = Execution()

        # Establish the connection between FCU and Drone
        uav.wait4connect()

        # Set mode to GUIDED
        uav.set_mode("GUIDED")

        # Wait for GUIDED mode to start
        uav.wait4start()

        # Take off
        uav.takeoff(10)

        rate = rospy.Rate(10)

        # This loop ensure that the global navigation mission is True
        while (mission.in_global_navigation) and (not rospy.is_shutdown()):
            
            rate.sleep()

            if mission.in_local_navigation:
                rospy.loginfo("In Local Waypoint Navigation...")
                next_local_wayppint = mission.current_local_waypoint
                uav.set_global_destination(
                    lat=next_local_wayppint[0], lon=next_local_wayppint[1],
                    alt= (uav.current_global_position.altitude) - uav.geoid_height(
                        uav.current_global_position.latitude, 
                        uav.current_global_position.longitude)
                )
                continue
            else:
                rospy.loginfo("In Global Waypoint Navigation Mission...")

                # Set speed of 5m/s
                uav.set_speed(5)

                # Setting global destination to desired global waypoint
                uav.set_global_destination(
                    lat=mission.current_global_waypoint[0], lon=mission.current_global_waypoint[1],
                    alt= (uav.current_global_position.altitude) - uav.geoid_height(
                        uav.current_global_position.latitude, 
                        uav.current_global_position.longitude)
                )
        
        rospy.loginfo("All gloabl waypoints have successfully been reached")
        uav.land()
        rospy.loginfo("Proceeding to Landing")

    except KeyboardInterrupt:
        exit()


    

