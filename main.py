
from Arm import Arm
from Simulation import Simulation
import matplotlib.pyplot as plt
import numpy as np
import math
import random

arm = Arm((0, 0))

number_of_linkeages = random.randint(2, 5)  # Random number of linkeages between 1 and 10
# add n linkeages to the arm of various lengths and inital angles
for i in range(number_of_linkeages):
    arm.add_linkeage(np.random.uniform(-np.pi, np.pi), np.random.uniform(0.1, 1))

bounds = 0
for link_length in arm.linkeage_lengths:
    bounds += link_length

# generate a random point inside the area of a radius
radius = bounds

theta = 2 * math.pi * random.random()
length = random.random() * radius
x = length * math.cos(theta)
y = length * math.sin(theta)

goal = (x, y)
sim = Simulation(goal, arm)
sim.draw(plt)

diff = sim.perform_newtons_method()

while diff > 0.01:
    # using matplotlib to draw the arm and goal
    # keeping x and y limits the same
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

    # calculate distance from goal and plot alongside as a error vs time graph
    end_effector_position = arm.calculate_end_position()
    distance_from_goal = np.sqrt((end_effector_position[0] - sim.goal[0])**2 + (end_effector_position[1] - sim.goal[1])**2)
    plt.text(sim.goal[0], sim.goal[1], f"Distance: {distance_from_goal:.2f}", fontsize=10, ha='right', va='bottom')
    
    print("Change in position: ", diff)
    diff = sim.perform_newtons_method()

plt.show()


