#!/usr/bin/env python3

import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from navigation_functions import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray, Bool, Float64

class LocalNavigation:
    def __init__(self):
        self.current_global_location = NavSatFix()
        self.current_compass_heading = Float64()
        self.current_global_waypoint = Float64MultiArray()
        self.obstacle_locations = [np.array([-35.36241997, 149.16487945])]
        self.local_waypoint_counter = 0
        self.in_global_navigation = Bool()

        '''
        -35.36241997, 149.16487945
        '''

        # Obstacle Avoidance constants
        self.K_ATTRACT = 0.1
        self.K_REPULSE = 5000

        # ROS Publishers

        # Publishing obstacle_avoiding topic with a boolean variable to uav_execute_node
        self.obstacle_avoiding_pub = rospy.Publisher(
            name="obstacle_avoiding",
            data_class=Bool,
            queue_size=10
        )
        
        # Publishing local_waypoints topic with current local navigation waypoint to uav_execute_node
        self.local_waypoints_pub = rospy.Publisher(
            name="local_waypoints",
            data_class=Float64MultiArray,
            queue_size=10
        )

        # ROS Subscribers

        # Subscribing current_global_location topic from uav_execute_node on current GPS coordinate
        self.current_global_location_sub = rospy.Subscriber(
            name="mavros/global_position/global",
            data_class=NavSatFix,
            callback=self.current_global_location_sub_cb
        )

        # Subscribing global_wyapoint topic from global_navigation_node on current global waypoint destination
        self.global_waypoint_sub = rospy.Subscriber(
            name="global_waypoint",
            data_class=Float64MultiArray,
            callback=self.global_waypoint_sub_cb
        )

        # Subscribing the global_position/compass_hdg topic to know the compass heading
        self.current_compass_heading_sub = rospy.Subscriber(
            name="mavros/global_position/compass_hdg",
            data_class=Float64,
            callback=self.current_compass_heading_cb
        )

        # Subcribing global_navigation_mission topic from global_navigation_node
        self.global_navigation_sub = rospy.Subscriber(
            name="global_navigation_mission",
            data_class=Bool,
            callback=self.global_navigation_sub_cb
            )


    # Call back functions

    def current_global_location_sub_cb(self, msg):
        self.current_global_location = msg

    def global_waypoint_sub_cb(self, msg):
        self.current_global_waypoint = msg.data

    def current_compass_heading_cb(self, msg):
        self.current_compass_heading = msg

    def global_navigation_sub_cb(self, msg):
        self.in_global_navigation = msg.data

    # Obstacle Avoidance Algorithm

    def get_attractive_force(self, position, goal):

        return self.K_ATTRACT * (goal - position)


    def get_repulsive_force(self, position, obstacle, radius=200):

        distance = np.linalg.norm(obstacle - position)

        if distance <= radius:
            
            repulsive = self.K_REPULSE * ((1.0 / distance) - (1.0 / radius)) * ((position - obstacle) / (distance ** 2))

            return repulsive
        
        else:

            return np.zeros_like(position)


    def get_next_waypoint(self, position, goal, obstacles, radius=200):

        attractive_force = self.get_attractive_force(position, goal)

        repulsive_forces = [self.get_repulsive_force(position, obs, radius) for obs in obstacles]

        net_force = attractive_force

        for force in repulsive_forces:
            net_force += force

        next_position = position + net_force

        return next_position


    def get_path(self, start, goal,obstacles, radius=200, num_waypoints=50):
        
        waypoints = [start]

        current_position = start

        for i in range(num_waypoints):

            next_waypoint = self.get_next_waypoint(current_position, goal, obstacles, radius)

            waypoints.append(next_waypoint)

            if (np.linalg.norm(waypoints[-1] - goal) < 10):
                waypoints.append(goal)
                break

            if len(waypoints) >= 2:
                if (abs(waypoints[-1][0] - waypoints[-2][0]) / waypoints[-1][0]) < 0.0000001 \
                    and (abs(waypoints[-1][1] - waypoints[-2][1])/waypoints[-1][1]) < 0.0000001:

                    obstacles_distance = [((obs[0] - waypoints[-1][0]) ** 2 + (obs[1] - waypoints[-1][1]) ** 2) for obs in obstacles]

                    closest_obstacle_index = obstacles_distance.index(min(obstacles_distance))
                    obstacles.append(np.array([obstacles[closest_obstacle_index][0], obstacles[closest_obstacle_index][1]]))

            current_position = next_waypoint

        minimum_waypoints = []

        for waypoint in waypoints:
            for obstacle in obstacles:

                if np.linalg.norm(waypoint - obstacle) < 50:
                    a = waypoint.tolist()
                    minimum_waypoints.append(a)
                    break

        minimum_waypoints.append(goal.tolist())

        return minimum_waypoints

    def local_xy_to_GPS_coordinates(self, localXY):
        
        earth_radius = 6378137.0

        localXY_x = localXY[0]
        localXY_y = localXY[1]

        localPolar_r = math.sqrt(localXY_x ** 2 + localXY_y ** 2)
        localPolar_alpha = math.degrees(math.atan2(localXY_x, localXY_y))

        NEPolar_r = localPolar_r
        NEPolar_theta = localPolar_alpha + self.current_compass_heading.data

        dNorth = NEPolar_r * math.cos(NEPolar_theta * math.pi/180)
        dEast = NEPolar_r * math.sin(NEPolar_theta * math.pi/180)

        current_lat = self.current_global_location.latitude
        current_lon = self.current_global_location.longitude
        
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius * math.cos(math.pi * current_lat/180))       

        newlat = current_lat + (dLat * 180/math.pi)
        newlon = current_lon + (dLon * 180/math.pi)

        return np.array([newlat, newlon])

    def GPS_coordinates_to_local_xy(self, GPS):

        earth_radius = 6378137.0
        
        GPS_lat = GPS[0]
        GPS_lon = GPS[1]

        current_lat = self.current_global_location.latitude
        current_lon = self.current_global_location.longitude

        dLat = (GPS_lat - current_lat) * math.pi/180
        dLon = (GPS_lon - current_lon) * math.pi/180

        dNorth = dLat * earth_radius
        dEast = dLon * (earth_radius * math.cos(math.pi * current_lat/180))  

        NEPolar_r = math.sqrt((abs(current_lat - GPS_lat) ** 2) + (abs(current_lon - GPS_lon) ** 2)) * 1.113195e5 
        NEPolar_theta = math.acos(dNorth / NEPolar_r) * (180/math.pi)

        localPolar_r = NEPolar_r
        localPolar_alpha = NEPolar_theta - self.current_compass_heading.data

        localXY_y = localPolar_r * math.cos(localPolar_alpha * math.pi/180)
        localXY_x = localPolar_r * math.sin(localPolar_alpha * math.pi/180)

        return np.array([localXY_x, localXY_y])

    def get_local_waypoints(self):

        goal_GPS_lat, goal_GPS_lon = self.current_global_waypoint

        goal_GPS = np.array([goal_GPS_lat, goal_GPS_lon])

        start_local_frame = np.array([0, 0])
        goal_local_frame = self.GPS_coordinates_to_local_xy(goal_GPS)

        # testing
        obstacles_GPS = self.obstacle_locations
        obstacles_local_frame = [self.GPS_coordinates_to_local_xy(obstacle_GPS) for obstacle_GPS in obstacles_GPS]

        local_waypoints_local_frame = self.get_path(start_local_frame, goal_local_frame, obstacles_local_frame)
        local_waypoints_GPS = [self.local_xy_to_GPS_coordinates(local_waypoint_local_frame) for local_waypoint_local_frame in local_waypoints_local_frame]

        return local_waypoints_GPS
    
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
        rospy.init_node("local_navigation_node")

        local_path = LocalNavigation()

        rate = rospy.Rate(10)

        while (not rospy.is_shutdown()) and (local_path.in_global_navigation):

            if not (type(local_path.current_global_waypoint) == tuple):
                rospy.loginfo("No Global Waypoints from global_navigation_node")
                rate.sleep()
                continue

            rate.sleep()

            if len(local_path.obstacle_locations) > 0:

                local_waypoints = local_path.get_local_waypoints()

                rospy.loginfo("Number of Local Waypoints: {}".format(len(local_waypoints)))

                if local_path.local_waypoint_counter + 1 > len(local_waypoints):
                    rospy.loginfo("Successfully Navigated All Local Waypoints")
                    local_path.obstacle_avoiding_pub.publish(False)
                    continue

                local_path.obstacle_avoiding_pub.publish(True)
                publihsing_local_waypoints = local_path.publish_float64multiarray_data(local_waypoints[local_path.local_waypoint_counter])

                local_path.local_waypoints_pub.publish(publihsing_local_waypoints)

                distance_to_next_local_waypoint = local_path.get_distance(
                    local_path.current_global_location.latitude, local_path.current_global_location.longitude,
                    local_waypoints[local_path.local_waypoint_counter][0], local_waypoints[local_path.local_waypoint_counter][1]
                    )
                rospy.loginfo("Distance to local waypoint {}: {}m".format(local_path.local_waypoint_counter+1, distance_to_next_local_waypoint))
                
                if distance_to_next_local_waypoint < 0.5:
                    local_path.local_waypoint_counter += 1

            else:
                rospy.loginfo("No Obstacles")
                local_path.obstacle_avoiding_pub.publish(False)
                local_path.local_waypoint_counter = 0

        rospy.loginfo("Stopping Global Waypoint Navigation")

    except KeyboardInterrupt:
        exit()




