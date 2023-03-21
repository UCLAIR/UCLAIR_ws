#!/usr/bin/env python3

import rospy
from navigation_functions import *
from std_msgs.msg import Float64MultiArray
import numpy as np

class GazeboExecution:
    def __init__(self):
        self.obstacles_r = []
        self.obstacles_theta = []

        self.obstacles_r_sub = rospy.Subscriber(
            name="osbtacles_r",
            data_class=Float64MultiArray,
            callback=self.obstacles_r_sub_cb
        )

        self.obstacles_theta_sub = rospy.Subscriber(
            name="obstacles_theta",
            data_class=Float64MultiArray,
            callback=self.obstacles_theta_sub_cb
        )

    def obstacles_r_sub_cb(self, msg):
        self.obstacles_r = msg.data

    def obstacles_theta_sub_cb(self, msg):
        self.obstacles_theta = msg.data

class LocalNavigation:
    def __init__(self):
        self.K_ATTRACT = 0.1
        self.K_REPULSE = 5000

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


if __name__ == "__main__":
    
    try:
        rospy.init_node("uav_gazebo_test_node")

        rate = rospy.Rate(10)

        local_navigation = LocalNavigation()

        mission = GazeboExecution()

        start = np.array([0, 0])
        goal = np.array([100, 100])
        obstacles = [np.array([10, 30])]

        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo("Obstacle R")
            rospy.loginfo(len(mission.obstacles_r))

            rospy.loginfo("Obstacle Theta")
            rospy.loginfo(mission.obstacles_theta)

        # uav = Navigation()

        # mission = GazeboExecution()

        # uav.wait4connect()

        # uav.set_mode("GUIDED")

        # uav.wait4start()

        # uav.takeoff(5)

        # rate = rospy.Rate(10)

        # global_navigation_status = True

        # global_navigation_waypoints = [[10, 0, 5], [10, 10, 5], [10, 10, 5]]

        # while (global_navigation_status) and (not rospy.is_shutdown()):
        #     pass

    except KeyboardInterrupt:
        exit()