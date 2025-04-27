# Two-Joint-Arm-Optimization

## Description
This project simulates a two-joint robotic arm tasked with navigating to a goal position from a given starting configuration.
To determine an optimal path, the arm will employ a modified version of Newton's Method--in which the program controlling the arm will use derivatives to iteratively adjust the angles found on the arm. Using this process, the arm will simulate the most efficient path to reach the goal position under the following restrictions:
* Arm Lengths - Lengths of each arm segment
* Maximum Rotation - Max rotation distance of each joint/linkage
* Obstacles - Barriers that the arm must path around
* Maximum Angle Change - How far the entire arm (angle from base to end of arm) may rotate in one iteration
This project will be simulating the resulting, realistic movement of the robotic arm using Matplotlib--allowing a clear simulation of how the arm behaves under varying environments.

## Key Terms
**End_effector**: The position of the very end of the robotic arm, represented by a point in the 2D space.
**Vector**: The direction of movement and magnitude needed to reach the goal. In this case, we use error vectors:
* **Error vector**: The difference in the position of the end_effector - position of the goal. This creates a 2D vector that points from the end_effector to the goal.
**Goal**: The desired (x,y) point at which we want to the end_effector to lie on
**Linkage**: A straight segment of the robotic arm. Each linkage has:
* A fixed length
* Rotation angle from its base
**Self**: The entire robotic arm of n linkages

## Research Questions
Given a robotic arm of n linkeages and &theta; degrees of freedom:
* Is it possible for the arm to reach the goal state from any starting configuration?
* Can the arm reach the goal state with any static barriers blocking its path?
* What is the most optimal route for the arm to reach the goal state?
* How many different configurations can the arm make to reach the goal state?
* How long does it take the arm to reach the goal state?
* Does increasing the rotation angle/number of linkages always improve the likelihood of finding the goal state?

## Methods-data description
calculate_jacobian()
 * **Overview**: Computes how changes to the angles of the linkeages affects the end_effector's position. This matrix is essential for Newton's Method to determining the most efficient path to the goal.

## Methods-numerical-Methods and software
perform_newtons_method(self)
* **Overview** Calculates the most optimal path of the robotic arm to the goal state. This will be done by iteratively adjusting the angles of the robotic arm to minimize the positional error between its end effector and the goal.

Matplotlib
* **Description**: Matplotlib is used to create animated visualizations of the robotic arm as it moves towards the goal. To maintain smooth animations, we have limited the linkage's rotation per iteration to .05 radians.

## Results (done last)

## Tables figures (done last)

## Conclusion-findings (done last)

## Conclusion-limitations & future research

## Citations (ask for citations)
