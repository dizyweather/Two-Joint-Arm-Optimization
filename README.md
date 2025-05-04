# Two-Joint-Arm-Optimization


## Description
This project simulates a two-joint robotic arm tasked with navigating to a goal in a 2D space from any starting configuration.
To determine an optimal path, the arm will employ Newton's Method--in which the program controlling the arm will alter the angles found on the arm using derivatives (CHANGE ONCE WE SETTLE ON NEW NEWTON’S METHOD). 
Iteratively using this process, the arm will simulate an efficient path to the goal using the following restrictions:
* Linkage Length - Length of each individual linkage
* Maximum Rotation - Max rotation distance of each joint/linkage
* Maximum Angle Change - How far the entire arm (angle from base to end of arm) may rotate in one iteration
This project will attempt to simulate realistic movement using Matplotlib to visualize this arm.


## Key Terms
**End_effector**: The position at the very end of the robotic arm, represented as a point in a 2D space.
**Vector**: The direction of movement and its magnitude needed to reach the goal.
* **Error vector**: The difference in the position of the end_effector minus the position of the goal. This creates a 2D vector that points from the end_effector to the goal.
**Goal**: Described as a (x,y) point at which we want to the end_effector to lie on
**Linkage**: A straight segment of the robotic arm. Each linkage has:
* A fixed length
* Rotation angle from its' base
**Self**: The entire robotic arm made up of n linkages


## Research Questions
Given a robotic arm of n linkages and &theta; degrees of freedom:
* Is it possible for the arm to reach the goal from any starting configuration?
* What is the most optimal route for the arm to reach the goal?
* How many different ways can the arm reach the goal?
* How long would it take for the arm to reach the goal?
* Does increasing the freedom of rotation or # of linkages improve the arm's ability to reach the goal state?


## Methods-data description
calculate_jacobian()
 * **Description:** Computes and stores in a 2D array how changes to the angles of the linkages affects the end_effector's position.


## Methods-numerical-Methods and software
perform_newtons_method(self)
* **Description:** Newton's method is used to analyze the partial derivatives created from Jacobian's matrix & calculate the new positions of the end_effectors using this data.


Matplotlib
* **Description:**: Matplotlib is being used to create visual representation of how the robotic arm moves from the start to the goal point.


## Results (done last)


## Tables figures (done last)


## Conclusion-findings (done last)


## Conclusion-limitations & future research


## Citations (ask for citations)
