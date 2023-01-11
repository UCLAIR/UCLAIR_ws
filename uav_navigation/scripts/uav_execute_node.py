#!/usr/bin/env python

import rospy
from navigation_functions import *
from std_msgs.msg import Float64MultiArray, Bool


class Execution:
    def __init__(self):
        self.current_global_waypoint = Float64MultiArray()
        self.in_global_navigation = Bool()

        # ROS Publishers

        # Publishing current_global_location topic to global_navigation_node
        self.current_global_location_pub = rospy.Publisher(
            name="current_global_location",
            data_class=Float64MultiArray,
            queue_size=10
        )

    
        # ROS Subscribers

        # Subscribing global_waypoint topic from global_navigation_node
        self.global_waypoint_sub = rospy.Subscriber(
            name="global_waypoint",
            data_class=Float64MultiArray,
            callback=self.global_waypoint_sub_cb,
        )

        # Subcribing global_navigation_mission topic from global_navigation_node
        self.global_navigation_sub = rospy.Subscriber(
            name="global_navigation_mission",
            data_class=Bool,
            callback=self.global_navigation_sub_cb
            )


    # Call back functions

    def global_waypoint_sub_cb(self, msg):
        self.current_global_waypoint = msg.data


    def global_navigation_sub_cb(self, msg):
        self.in_global_navigation = msg.data


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

        while not rospy.is_shutdown():

            # Establish the connection between FCU and Drone
            uav.wait4connect()
        
            # Set mode to GUIDED
            uav.set_mode("GUIDED")
        
            # Wait for GUIDED mode to start
            uav.wait4start()

            # Take off
            uav.takeoff(3)

            # Publishing current location to global_navigation_node
            data = mission.publish_float64multiarray_data(uav.get_current_location())
            mission.current_global_location_pub.publish(data)

            # # This loop ensure that the global navigation mission is True
            # while mission.in_global_navigation:
        
            #     # Set speed of 20m/s
            #     uav.set_speed(20)

            #     # Setting global destination to desired global waypoint
            #     uav.set_global_destination(
            #         lat=mission.current_global_waypoint[0], lon=mission.current_global_waypoint[1],
            #         alt= (uav.current_global_position.altitude) - uav.geoid_height(
            #         uav.current_global_position.latitude, 
            #         uav.current_global_position.longitude)
            #     )

            #     rospy.loginfo("In Global Waypoing Navigation Mission...")

            
            # rospy.loginfo("All gloabl waypoints have successfully been reached")
            # uav.land()
            # rospy.loginfo("Proceeding to Landing")

    except KeyboardInterrupt:
        exit()


    

