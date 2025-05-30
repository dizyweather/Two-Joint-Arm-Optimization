from Arm import Arm
from Simulation import Simulation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import math
import random

# Set if you want to visualize the distance to goal function in 3d
function_viz = False

# Initalize and randomize the number of linkeages, their angles, and lengths in the arm
arm = Arm((0, 0))

if function_viz:
    # Can only really visualize a 2 variable function in 3d so we set the number of linkeages to 2
    number_of_linkeages = 2
else:
    number_of_linkeages = random.randint(2, 10)

for i in range(number_of_linkeages):
    arm.add_linkeage(np.random.uniform(-np.pi, np.pi), np.random.uniform(0.1, 1))

# generate random point within the area of the arm to set as the goal
# NOTE: This may result in a goal that is unreachable depending on the arm's lengths
bounds = 0
for link_length in arm.linkeage_lengths:
    bounds += link_length

radius = bounds # max distance arm can reach from origin

theta = 2 * math.pi * random.random() # random angle
length = random.random() * radius # random distance from origin
x = length * math.cos(theta)
y = length * math.sin(theta)

goal = (x, y)

# Initalize simulation and dummy values to start the while loop
sim = Simulation(goal, arm, 0.1, use_newton=False) # set if we want to use newton's method or not
diff = 1 
distance_from_goal =  1

# If we are visualizing the function, we need to set up the 3d plot
if function_viz and len(arm.linkeage_angles) == 2:
    fig = plt.figure()
    ax = fig.add_subplot( 111, projection='3d' )
    ax.mouserotationstyle = 'azel'
    plt.ion()

# set up count to track the number of iterations    
count = 1

# While the change in end effector position is greater than the threshold (aka unstable)
# and the distance from the goal is greater than the threshold (aka not at goal)
while diff > 0.001 and distance_from_goal > 0.01:
    # update simulation and count
    diff = sim.update()
    count += 1
    
    # if we are visualizing the function
    if function_viz and len(arm.linkeage_angles) == 2:
        # Keep view the same
        elev, azim = ax.elev, ax.azim # Stores the current view

        plt.clf()  # When clearing, graph resets to default view

        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(elev=elev, azim=azim)  # Restore previous view

        # Graph settings
        ax.set_xlabel('Angle 1 (rad)')
        ax.set_ylabel('Angle 2 (rad)')
        ax.set_zlabel('Distance from Goal')
        ax.set_title('3D Plot of Angles vs Distance from Goal')
        ax.set_xlim(-np.pi, np.pi)
        ax.set_ylim(-np.pi, np.pi)
        
        # Analytically calculate the function
        function = lambda theta1, theta2: np.sqrt(
            (np.cos(theta1) * arm.linkeage_lengths[0] + np.cos(theta1 + theta2) * arm.linkeage_lengths[1] - goal[0])**2 + 
            (np.sin(theta1) * arm.linkeage_lengths[0] + np.sin(theta1 + theta2) * arm.linkeage_lengths[1]- goal[1])**2)
        
        # Create a 100x100 grid of different angles for theta1 and theta2
        x = np.linspace(-np.pi, np.pi, 100)
        y = np.linspace(-np.pi, np.pi, 100)
        X, Y = np.meshgrid(x, y)
        
        # Calculate the distance values for each point in the grid & Plot it
        Z = function(X, Y)
        ax.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha = 0.4)

        # plot current angles and distance from goal
        end_effector_position = arm.calculate_end_position()
        distance_from_goal = np.sqrt((end_effector_position[0] - sim.goal[0])**2 + (end_effector_position[1] - sim.goal[1])**2)

        # Plot the highlight ring (larger, lighter color)
        ax.scatter3D(
            arm.linkeage_angles[0],
            arm.linkeage_angles[1],
            distance_from_goal + 0.01,
            c='white',          # Light color for the ring
            marker='o',
            s=300,              # BIGGER size
            depthshade=False,
            alpha=0.5,          # Partially transparent
            edgecolors='black', # Add a black edge
            linewidths=1
        )

        # Plot the actual dot (smaller, bright red)
        ax.scatter3D(
            arm.linkeage_angles[0],
            arm.linkeage_angles[1],
            distance_from_goal + 0.01,
            c='red',            # Bright color
            marker='o',
            s=100,              # SMALLER size
            depthshade=False,
            edgecolors='black', # Optional strong edge
            linewidths=1.5
        )

        plt.pause(0.2)
        
    else: # if we are just visualizing the arm and goal
        # Graph settings
        plt.xlim(-bounds, bounds)
        plt.ylim(-bounds, bounds)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid()
        plt.title("Arm Simulation")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")

        plt.pause(0.1)  # Pause to update the plot
        plt.clf()  # Clear the figure for the next frame
        sim.draw(plt)  # Redraw the arm and goal

        # calculate distance from goal and plot alongside goal
        end_effector_position = arm.calculate_end_position()
        distance_from_goal = np.sqrt((end_effector_position[0] - sim.goal[0])**2 + (end_effector_position[1] - sim.goal[1])**2)
        plt.text(sim.goal[0], sim.goal[1], f"Distance: {distance_from_goal:.2f}", fontsize=10, ha='right', va='bottom')
    
    

print(count, " iterations")
plt.ioff() 
plt.show()


