# Two-Joint-Arm-Optimization

## Description
This project will simulate a robotic arm of n linkages with the task of moving to a goal--as represented by a point (x,y) in a 2D space from any starting configuration.
To find the optimal path, the employ will be using Newton's Method--a root finding algorithm that will label the goal as the 'root' and attempt to navigate the robotic arm to it using the equation:
$ x_{n+1} = x_n - \frac{f(x_n)/f'(x_n)} $
By repeating this process until we reach the goal--we will discover the most efficient path to the goal while under the additional restrictions:
* Linkage Length - Length of each individual linkage
* Max Rotation - Max rotation a linkage joint
* Max Angle Change - Max rotation a linkage joint may move per iteration
This project will visualize this arm using Matplotlib.


## Key Terms
**End_effector:** The position at the very end of the robotic arm, represented as a point in a 2D space.
**Vector**: Direction and magnitude of a object.
* **Error vector:** The difference in the position of the end_effector minus the position of the goal. This'll create a 2D vector that points from the end_effector to the goal.
**Goal:** Described as a point at which we want to the end_effector to lie on once the function stops iterating.
**Linkage:** A straight segment of the robotic arm. Each linkage has:
* A fixed length
* Angle from its' base
**Self:** The robotic arm, 

## Research Questions
Given a robotic arm of n linkages and &theta; degrees of freedom:
* Is it possible for the arm to reach the goal from any starting postion?
* What is the most optimal route for the arm to reach the goal?
* How many different ways can the arm reach the goal?
* How long will it take for the arm to reach the goal?
* Will increasing the arm's max rotation, angle change, or # of linkages improve its' ability to reach the goal state?

## Methods-data description
calculate_jacobian()
 * **Description:** Computes and stores partial derivates in a 2D array telling how changes to the angles of the linkages will affect the end_effector's position. 

## Methods-numerical-Methods and software
perform_newtons_method(self)
* **Description:** (CHANE HOW NEWTON'S METHOD WORKS HERE)

Matplotlib
* **Description:**: Matplotlib & Jupyter notebook will be used to create an environment & simulation as to how the robotic arm moves.

## Results (done last)

## Tables figures (done last)

## Conclusion-findings (done last)

## Conclusion-limitations & future research

## Citations (ask for citations)
