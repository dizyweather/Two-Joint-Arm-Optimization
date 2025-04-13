# Two-Joint-Arm-Optimization

## Description
This project simulates a two-joint robotic arm tasked with navigating to a goal position from a given starting configuration.
To determine an optimal path, the arm will employ a modified version of Newton's Method--in which the program controlling the arm will use derivatives to iteratively adjust the angles found on the arm. Using this process, the arm will simulate the most efficient path to reach the goal position under the following restrictions:
* Arm Lengths - Lengths of each arm segment
* Maximum Rotation - Max rotation distance of each joint/linkage
* Obstacles - Barriers that the arm must path around
* Maximum Angle Change - How far the entire arm (angle from base to end of arm) may rotate in one iteration
This project will be simulating the resulting, realistic movement of the robotic arm using Matplotlib--allowing a clear simulation of how the arm behaves under varying environments.
