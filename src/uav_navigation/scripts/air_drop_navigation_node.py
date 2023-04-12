#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64MultiArray, Int16


class AirDropNavigation:
    def __init__(self):
        self.in_air_drop_navigation = Bool()
        self.GPS_drop_1 = [-35.36366699, 149.16395691]
        self.GPS_drop_2 = [-35.36328456, 149.16439751]
        self.GPS_drop_3 = []
        self.GPS_drop_4 = []
        self.GPS_drop_5 = []

        self.drop_number_counter = Int16()
        self.GPS_drop = [self.GPS_drop_1, self.GPS_drop_2]

        # ROS Publishers
        self.in_air_drop_navigation_pub = rospy.Publisher(
            name="in_air_drop_navigation",
            data_class=Bool,
            queue_size=10
        )

        self.air_drop_waypoint_pub = rospy.Publisher(
            name="air_drop_waypoint",
            data_class=Float64MultiArray,
            queue_size=10
        )

        # ROS Subscribers
        self.in_global_navigation_sub = rospy.Subscriber(
            name="global_navigation_mission",
            data_class=Bool,
            callback=self.global_navigation_sub_cb
        )

        self.drop_number_counter_sub = rospy.Subscriber(
            name="drop_number",
            data_class=Int16,
            callback=self.drop_number_counter_sub_cb
        )

    # Call back functions

    def global_navigation_sub_cb(self, msg):
        if msg.data == True:
            self.in_air_drop_navigation = False
        else:
            self.in_air_drop_navigation = True

    def current_global_location_sub_cb(self, msg):
        self.current_global_location = msg

    def drop_number_counter_sub_cb(self, msg):
        self.drop_number_counter = msg


    # Convert the data into publisherable data for Float64MultiArray()
    def publish_float64multiarray_data(self, d):
        publishing_data = Float64MultiArray()

        publishing_data.data = d

        return publishing_data


if __name__ == "__main__":
    try:
        rospy.init_node("air_drop_navigation_node")

        air_drop = AirDropNavigation()

        rate = rospy.Rate(10)

        while (not rospy.is_shutdown()):
            rate.sleep()

            if air_drop.drop_number_counter.data == len(air_drop.GPS_drop):
                break

            if air_drop.in_air_drop_navigation:
                rospy.loginfo("In Air Drop Navigation")

                rospy.loginfo(f"Air Drop Number: {air_drop.drop_number_counter.data}")
            else:
                rospy.loginfo("In Global Navigation")

            air_drop.in_air_drop_navigation_pub.publish(air_drop.in_air_drop_navigation)

            current_air_drop_waypoint = air_drop.publish_float64multiarray_data(air_drop.GPS_drop[air_drop.drop_number_counter.data])
            air_drop.air_drop_waypoint_pub.publish(current_air_drop_waypoint)

        air_drop.in_air_drop_navigation_pub.publish(False)
        rospy.loginfo("Air Drop Completed")


    except rospy.ROSInterruptException:
        exit()