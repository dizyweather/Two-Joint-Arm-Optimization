
from Arm import Arm
from Simulation import Simulation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import math
import random

grapH_viz = True
# Initalize and randomize the number of linkeages and their angles and lengths in the arm
arm = Arm((0, 0))

number_of_linkeages = 2

for i in range(number_of_linkeages):
    arm.add_linkeage(np.random.uniform(-np.pi, np.pi), np.random.uniform(0.1, 1))

# generate random point within the area of the arm to set as the goal
bounds = 0
for link_length in arm.linkeage_lengths:
    bounds += link_length

radius = bounds

theta = 2 * math.pi * random.random()
length = random.random() * radius
x = length * math.cos(theta)
y = length * math.sin(theta)

goal = (x, y)

# Initalize simulation and difference)
sim = Simulation(goal, arm)
diff = sim.perform_newtons_method()
plt.ion()

if grapH_viz and len(arm.linkeage_angles) == 2:
    fig = plt.figure()
    ax = fig.add_subplot( 111, projection='3d' )
    ax.mouserotationstyle = 'azel'

# While the change in end effector position is greater than the threshold
while diff > 0.01:
    if grapH_viz and len(arm.linkeage_angles) == 2:
        # Store current view
        elev, azim = ax.elev, ax.azim

        plt.clf()  # Clear the figure for the next frame

        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(elev=elev, azim=azim)  # Restore previous view

        ax.set_xlabel('Angle 1 (rad)')
        ax.set_ylabel('Angle 2 (rad)')
        ax.set_zlabel('Distance from Goal')
        ax.set_title('3D Plot of Angles vs Distance from Goal')
        ax.set_xlim(-np.pi * 5/4, np.pi * 5/4)
        ax.set_ylim(-np.pi * 5/4, np.pi * 5/4)
        
        
        function = lambda theta1, theta2: np.sqrt(
            (np.cos(theta1) * arm.linkeage_lengths[0] + np.cos(theta2) * arm.linkeage_lengths[1] - goal[0])**2 + 
            (np.sin(theta1) * arm.linkeage_lengths[0] + np.sin(theta2) * arm.linkeage_lengths[1]- goal[1])**2)
        
        x = np.linspace(-np.pi, np.pi, 100)
        y = np.linspace(-np.pi, np.pi, 100)
        X, Y = np.meshgrid(x, y)
        Z = function(X, Y)
        ax.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha = 0.4)

        plt.plot(goal[0], goal[1], 'g*')

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

        plt.pause(0.25)
        
        
    # Plot arm and goal with proper bounds
    else:
        plt.xlim(-bounds, bounds)
        plt.ylim(-bounds, bounds)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid()
        plt.title("Arm Simulation")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        
        plt.clf()  # Clear the figure for the next frame
        sim.draw(plt)  # Redraw the arm and goal

        # calculate distance from goal and add it the the goal point
        end_effector_position = arm.calculate_end_position()
        distance_from_goal = np.sqrt((end_effector_position[0] - sim.goal[0])**2 + (end_effector_position[1] - sim.goal[1])**2)
        plt.text(sim.goal[0], sim.goal[1], f"Distance: {distance_from_goal:.2f}", fontsize=10, ha='right', va='bottom')
    
        plt.pause(0.1)  # Pause to update the plot
    print("Change in position: ", diff)
    diff = sim.perform_newtons_method()

plt.show()


