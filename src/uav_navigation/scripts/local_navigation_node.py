#!/usr/bin/env python

import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from navigation_functions import *
from std_msgs.msg import Float64MultiArray, Bool

class LocalNavigation:
    def __init__(self):
        self.current_global_location = Float64MultiArray()
        self.obstacle_avoiding = Bool()
        self.current_global_waypoint = Float64MultiArray()

        '''
        self.obstacles = Float64MultiArray()
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
        self.local_navigation_pub = rospy.Publisher(
            name="local_waypoints",
            data_class=Float64MultiArray,
            queue_size=10
        )

        # ROS Subscribers

        # Subscribing current_global_location topic from uav_execute_node on current GPS coordinate
        self.current_global_location_sub = rospy.Subscriber(
            name="current_global_location",
            data_class=Float64MultiArray,
            queue_size=10,
            callback=self.current_global_location_sub_cb
        )

        # Subscribing global_wyapoint topic from global_navigation_node on current global waypoint destination
        self.global_waypoint_sub = rospy.Subscriber(
            name="global_waypoint",
            data_class=Float64MultiArray,
            queue_size=10,
            callback=self.global_waypoint_sub_cb
        )

        # Subscribing

        # (For future purpose)
        '''
        self.obstacles_sub = rospy.Subscriber(
            name="obstacle_position",
            data_class=Float64MultiArray,
            queue_size=10,
            callback=obstacles_sub_cb
        )
        '''

    # Call back functions

    def current_global_location_sub_cb(self, msg):
        self.current_global_location = msg

    def global_waypoint_sub_cb(self, msg):
        self.current_global_waypoint = msg.data

    '''
    def obstacles_sub_cb(self, msg):
        self.obstacles = msg.data
    '''

    # Obstacle Avoidance Algorithm

    def get_attractive_force(self, position, goal):

        return self.K_ATTRACT * (goal - position)


    def get_repulsive_force(self, position, obstacle, radius=100):

        distance = np.linalg.norm(obstacle - position)

        if distance <= radius:
            
            repulsive = self.K_REPULSE * ((1.0 / distance) - (1.0 / radius)) * ((position - obstacle) / (distance ** 2))

            return repulsive
        
        else:

            return np.zeros_like(position)


    def get_next_waypoint(self, position, goal, obstacles, radius):

        attractive_force = self.get_attractive_force(position, goal)

        repulsive_forces = [self.get_repulsive_force(position, obs, radius) for obs in obstacles]

        net_force = attractive_force

        for force in repulsive_forces:
            net_force += force

        next_position = position + net_force

        return next_position


    def get_path(self, start, goal,obstacles, radius, num_waypoints=50):
        
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

        return waypoints
    

    def convert_obstacles_local_to_global_frame(self, obstacles_local_frame):

        earth_radius = 6378137.0

        obstacles_global_frame = []

        for obs in obstacles_local_frame:

            # Coordinate offsets in radians
            dLat = obs[0]/earth_radius
            dLon = obs[1]/(earth_radius*math.cos(math.pi*self.current_global_location[0]/180))

            # new position in decimal degrees
            newLat = self.current_global_location[0] + (dLat*180/math.pi)
            newLon = self.current_global_location[1] + (dLon*180/math.pi)

            obstacles_global_frame.append([newLat, newLon])

        return obstacles_global_frame


if __name__ == "__main__":
    try:
        rospy.init_node("local_navigation_node")

        rate = rospy.Rate(10)

        testing_obstacles = [np.array([20, 20])]

        local_path = LocalNavigation()

        while not rospy.is_shutdown():

            if len(local_path.current_global_location.data) == 0:

                rospy.loginfo("No GPS Coordinates from FCU")
                rate.sleep()

            else:
                rospy.loginfo(local_path.current_global_location.data)





    except KeyboardInterrupt:
        exit()




