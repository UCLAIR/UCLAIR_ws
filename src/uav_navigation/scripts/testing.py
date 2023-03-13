import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math


# Constants
K_ATTRACT = 0.1  # Attractive force coefficient
K_REPULSE = 10000  # Repulsive force coefficient

"""
normal - 0.1 5000
modified - 0.001 10000
other - 0.01 5000
"""


def get_attractive_force(start, position, goal):
    distance_goal = np.linalg.norm(goal - position)
    distance_star = np.linalg.norm(goal - start) * 0.5

    if distance_goal <= distance_star:

        return K_ATTRACT * (goal - position)
    
    else:
        return distance_star * K_ATTRACT * (goal - position) / distance_goal
    
    # return K_ATTRACT * (goal - position)



def get_repulsive_force(position, goal, obstacle, radius):
    # #modified version
    # distance_obstacle = np.linalg.norm(obstacle - position)
    # distance_goal_x = position[0] - goal[0]
    # distance_goal_y = position[1] - goal[1]
    # drone_radius = 2

    # if distance_obstacle <= radius:
    #     repulsive = K_REPULSE/distance_obstacle**2 * (1/distance_obstacle - 1/radius) * (1 - math.exp(-((goal[0] - position[0])**2 + (goal[1] - position[1])**2)/drone_radius**2))

    #     return repulsive
    
    # else:
    #     return np.zeros_like(position)


    #Krogh's potential function
    distance_obstacle = np.linalg.norm(obstacle - position)

    if distance_obstacle <= radius:
        repulsive = K_REPULSE/(distance_obstacle**2) * ((1.0 / distance_obstacle) - (1.0 / radius)) * (position - obstacle)

        return repulsive
    else:
        return np.zeros_like(position)

    # #Other potential function
    # distance_obstacle = np.linalg.norm(position - obstacle)

    # if distance_obstacle <= radius:
    #     return K_REPULSE * (1/(distance_obstacle - 100)**2 - 1/(radius - 100)**2)
    # else:
    #     return np.zeros_like(position)


def get_next_waypoint(start, position, goal, obstacles, radius):
    # Attractive force from current position to goal
    attractive_force = get_attractive_force(start, position, goal)

    # Repulsive forces from current position to obstacles
    repulsive_forces = [get_repulsive_force(position, goal, obs, radius) for obs in obstacles]

    #print(f"Attractive Force: {attractive_force}, Repulsive Forces: {repulsive_forces}")

    # Calculate the net force
    net_force = attractive_force
    for force in repulsive_forces:
        net_force += force

    # Calculate the next position based on the net force
    next_position = position + net_force

    return next_position

def get_path(start, goal, obstacles, radius, num_waypoints):
    waypoints = [start]

    current_position = start

    for i in range(num_waypoints):

        next_waypoint = get_next_waypoint(start, current_position, goal, obstacles, radius)

        waypoints.append(next_waypoint)

        # Close to the Goal
        if (np.linalg.norm(waypoints[-1] - goal) < 10):
            waypoints.append(goal)
            break

        # Creating virtual obstacle if stuck at local minima
        if len(waypoints) >= 2:
            if (abs(waypoints[-1][0] - waypoints[-2][0]) / waypoints[-1][0]) < 0.0000001 \
                and (abs(waypoints[-1][1] - waypoints[-2][1])/waypoints[-1][1]) < 0.0000001:

                obstacles_distance = [((obs[0] - waypoints[-1][0]) ** 2 + (obs[1] - waypoints[-1][1]) ** 2) for obs in obstacles]

                closest_obstacle_index = obstacles_distance.index(min(obstacles_distance))
                obstacles.append(np.array([obstacles[closest_obstacle_index][0] + 0.0001, obstacles[closest_obstacle_index][1]]))

        current_position = next_waypoint

    minimum_waypoints = []

    for waypoint in waypoints:
        for obstacle in obstacles:

            if np.linalg.norm(waypoint - obstacle) < 50:
                a = waypoint.tolist()
                minimum_waypoints.append(a)
                break

    minimum_waypoints.append(goal.tolist())
    minimum_waypoints.insert(0, start.tolist())

    return minimum_waypoints


# Example usage
start = np.array([0, 0])
goal = np.array([100, 125])
obstacles = [np.array([35, 35])]
radius = 200
num_waypoints = 1000

path = get_path(start, goal, obstacles, radius, num_waypoints)
print(path)
print(len(path))
datax = [p[0] for p in path]
datay = [p[1] for p in path]

# create the figure and axes object
fig, ax = plt.subplots()

ax.plot([obs[0] for obs in obstacles], [obs[1] for obs in obstacles], 'ko')
ax.plot([goal[0]], [goal[1]], 'ro')

# create an empty line object
line, = ax.plot([], [])

# define the animation function
num_frames = len(datax)
frame_interval = 10
def animate(frame):
    # get the x and y coordinates of the line for the current frame
    x = datax[:frame+1]
    y = datay[:frame+1]
    # update the data of the line object
    line.set_data(x, y)
    # return the updated line
    return line,

# create the animation object
animation = FuncAnimation(fig, animate, frames=num_frames, interval=frame_interval)

# set the x and y limits of the axes
ax.set_xlim([-100, 300])
ax.set_ylim([-100, 300])

# show the animation
plt.show()
