#!/usr/bin/env python3

import rospy
from navigation_functions import *
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import NavSatFix


class GlobalNavigation:

    # Global Waypoints from competitions (the last waypoint is repeated)
    # waypoints = [
    #     [51.7054566, -0.1298779], [51.7055563, -0.1298591],
    #     [51.7055380, -0.1292449], [51.7054267, -0.1292717],
    #     [51.7054267, -0.1292717]
    # ]

    waypoints = [
        [-35.3624856, 149.16446636], [-35.3624856, 149.16446636]
    ]


    def __init__(self):
        self.current_waypoint_counter = 0
        self.current_global_location = NavSatFix()


        # ROS Publishers

        # Publishing global_waypoint topic with current waypoint dstination to uav_execute_node
        self.global_waypoint_pub = rospy.Publisher(
            name="global_waypoint",
            data_class=Float64MultiArray,
            queue_size=10
        )

        # Publishing global_navigation_mission with a boolean variable to uav_execute_node
        self.global_navigation_pub = rospy.Publisher(
            name="global_navigation_mission",
            data_class=Bool,
            queue_size=10
        )


        # ROS Subcribers

        # Subscribing current_global_location topic from uav_execute_node on current GPS coordinate
        self.current_global_location_sub = rospy.Subscriber(
            name="mavros/global_position/global",
            data_class=NavSatFix,
            callback=self.current_global_location_sub_cb
        )


    # Call back functions

    def current_global_location_sub_cb(self, msg):
        self.current_global_location = msg


    # Getting distance between current location to destination 
    def get_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculates the distance in metres from current location to desired location

        Args:
            lat1 (Float): latitude of current location
            lon2 (Float): longitude of current location
            lat2 (Float): latitude of desired location
            lon2 (Float): longitude of desired location
        """
        
        dlat = abs(lat1 - lat2)
        dlon = abs(lon1 - lon2)
        
        return sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5

    # Convert the data into publisherable data for Float64MultiArray()
    def publish_float64multiarray_data(self, d):
        publishing_data = Float64MultiArray()

        publishing_data.data = d

        return publishing_data


if __name__ == "__main__":
    try:
        # Initalialise global_navigation_node
        rospy.init_node("global_navigation_node")

        # Keey rospy at 10hz
        rate = rospy.Rate(10)

        # Initialise the GlobalNavigation class
        global_path = GlobalNavigation()

        # Keeps global navigation running until it reaches the last waypoint
        while (not rospy.is_shutdown()) and (global_path.current_waypoint_counter < len(global_path.waypoints)):

            # Publishing to the uav_execute_node that the UAV is in global navigation mode
            global_path.global_navigation_pub.publish(True)

            # Publishing current waypoint target to uav_execute_node
            current_waypoint = global_path.publish_float64multiarray_data(global_path.waypoints[global_path.current_waypoint_counter])
            global_path.global_waypoint_pub.publish(current_waypoint)

            # Ensure we are receiving current global location from uav_execute_node
            if (global_path.current_global_location.latitude == None) and (global_path.current_global_location.longitude == None):
                rospy.loginfo("No GPS Coordinates from FCU")
                rate.sleep()

            else:
                
                # Calculating distance
                distance = global_path.get_distance(
                    lat1=global_path.current_global_location.latitude,
                    lon1=global_path.current_global_location.longitude,
                    lat2=global_path.waypoints[global_path.current_waypoint_counter][0],
                    lon2=global_path.waypoints[global_path.current_waypoint_counter][1]
                )

                rospy.loginfo(
                    "Distance left to Waypoint {}: {}m".format(str(global_path.current_waypoint_counter+1), distance)
                    )
                
                rate.sleep()

                # Determining the next waypoint
                if distance < 0.5:
                    global_path.current_waypoint_counter += 1


        # Publishing to the uav_execute_node that the UAV is not in global navigation mode
        global_path.global_navigation_pub.publish(False)

        rospy.loginfo("Stopping Global Waypoint Navigation")


    except KeyboardInterrupt:
        exit()



