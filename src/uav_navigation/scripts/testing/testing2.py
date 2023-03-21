#!/usr/bin/env python3

import rospy
from navigation_functions import *

if __name__ == "__main__":
    try:
        rospy.init_node("uav_execute_node")

        uav = Navigation()

        uav.wait4connect()

        uav.set_mode("GUIDED")

        uav.wait4start()

        uav.takeoff(10)

        rate = rospy.Rate(10)
# [-35.36217293, 149.16497581, 10], [-35.36217293, 149.16497581, 10],
        goals = [
                 [50, 0, 0], [50, 50, 0],
                 [0, 50, 0], [0, 0, 0], [0, 0, 0]
                 ]
        
        i = 0

        while (not rospy.is_shutdown()) and (i < len(goals)):
            rate.sleep()
            
            # while i < 2:
            #     rospy.loginfo("global frame")
            #     uav.set_speed(10)

            #     uav.set_global_destination(
            #         lat = goals[i][0], lon=goals[i][1], 
            #         alt= (uav.current_global_position.altitude) - uav.geoid_height(
            #                 uav.current_global_position.latitude, 
            #                 uav.current_global_position.longitude)
            #     )

            #     if uav.check_waypoint_reached():
            #         i += 1
            
            
            while i < len(goals):
                uav.set_new_local_reference_frame()
                rospy.loginfo("local frame")
                
                uav.set_speed(5)

                uav.set_local_destination_offset_ned(goals[i][0], goals[i][1], 0)

                rospy.loginfo(f"x: {abs(goals[i][0] - uav.current_local_position.pose.pose.position.x)}")
                rospy.loginfo(f"y: {abs(goals[i][1] - uav.current_local_position.pose.pose.position.y)}")
                rospy.loginfo(f"z: {abs(goals[i][2] - uav.current_local_position.pose.pose.position.z)}")

                rospy.loginfo(f"x: {abs(goals[i][0] - uav.current_local_local_position.pose.position.x)}")
                rospy.loginfo(f"y: {abs(goals[i][1] - uav.current_local_local_position.pose.position.y)}")
                rospy.loginfo(f"z: {abs(goals[i][2] - uav.current_local_local_position.pose.position.z)}")


                if (abs(goals[i][0] - uav.current_local_position.pose.pose.position.x) < 1) and \
                    (abs(goals[i][1] - uav.current_local_position.pose.pose.position.y) < 1):

                    i += 1

        uav.land()

    except KeyboardInterrupt:
        exit()

    


