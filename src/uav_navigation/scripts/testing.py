#!/usr/bin/env python
import rospy
from navigation_functions import *


def main():
    # Initialise UAV ROS node
    rospy.init_node("global_navigation_node")

    # Initiatlise the Navigation class from navigation_functions.py
    uav = Navigation()

    # Establish the connection between FCU and Drone
    uav.wait4connect()
    
    # Set mode to GUIDED
    uav.set_mode("GUIDED")
    
    # Wait for GUIDED mode to start
    uav.wait4start()

    # Take off 
    uav.takeoff(3)

    # Keeping the loop at 5hz
    rate = rospy.Rate(5)

    # Global Waypoints
    # The final waypoint is added twice to ensure the final destination is arrived
    waypoints = [
        [-35.361016, 149.165306], [-35.360995, 149.167744],
        [-35.360995, 149.167744]
    ]

    # Waypoint Counter
    waypoint_count = 0

    # Global Navigation Control Loop
    while waypoint_count < len(waypoints):
        
        # Setting Speed
        uav.set_speed(20)

        # Calculating the distance to waypoints
        distance = uav.distance_to_location_from_gps(
            uav.current_global_position.latitude,
            uav.current_global_position.longitude,
            uav.waypoint_global_frame.pose.position.latitude,
            uav.waypoint_global_frame.pose.position.longitude
        )

        rospy.loginfo("Distance left to Waypoint {}: {}m".format(str(waypoint_count+1), distance))

        # Setting the next waypoint
        uav.set_global_destination(
            lat=waypoints[waypoint_count][0], lon=waypoints[waypoint_count][1],
            alt= (uav.current_global_position.altitude) - uav.geoid_height(
                uav.current_global_position.latitude, 
                uav.current_global_position.longitude)
        )

        rate.sleep()

        # Increase wyapoint counter
        if uav.check_waypoint_reached():
            waypoint_count += 1
        
    rospy.loginfo("All gloabl waypoints have successfully been reached")
    uav.land()
    rospy.loginfo("Proceeding to Landing")


if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        exit()
