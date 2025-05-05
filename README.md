# Two-Joint-Arm-Optimization

## Description
This project simulates a two-joint robotic arm tasked with navigating to a goal in a 2D space from a random starting configuration.
To determine an optimal path, the simulation will employ Newton's Method to calculate how the angles should change in the arm based on the first and second partial derivatives to get closer to the goal.
Iteratively using this process, the arm will simulate an efficient path to the goal using the following adjustable restrictions:
* Number of Linkages - How many linkages are in the arm
* Linkage Length - The physical length of each individual linkage
* Maximum Angle Change - How far each joint may rotate in one iteration

## Key Terms
**End_effector**: The position at the very end of the robotic arm, represented as a point in a 2D space.
**Vector**: The direction of movement and its magnitude needed to reach the goal.
* **Error vector**: The difference in the position of the goal minus the end_effector. This creates a 2D vector that points from the end_effector to the goal.
**Goal**: Described as a (x,y) point at which we want to the end_effector to reach
**Linkage**: A straight segment of the robotic arm. Each linkage has:
   * A fixed length
   * Rotation angle relative to its base / previous linkage

## Research Questions
Given a robotic arm of n linkages:
* Is it possible for the arm to reach the goal from any starting configuration?
* What is the most optimal route for the arm to reach the goal?
* How long would it take for the arm to reach the goal?
* Does increasing the rotation per iteration limit or number of linkages improve the arm's ability to reach the goal state?
* Does Newton’s method converge faster than the Pseudo Inverse Jacobian method?

## Methods-data description & numerical methods
Our program is made up of 2 primary classes and 2 runner files. 
### ```Arm.py```:
* Class representing our robotic arm
* Arm contains multiple joints, each with its own length and angle offset
* ```calculate_jacobian()```
   * **Description:** Computes and returns the Jacobian of how changes to the angles of each linkage affect the end_effector's position at this current orientation of the arm. Aka the slope of the current arm's state.

### ```Simulation.py```:
* Performs numerical methods on the arm class to get closer to the set goal
* ```jacobian_pseudoinverse_ik()```
   * **Description:** A method using the pseudo-inverse of the arm’s Jacobian matrix (first derivative) to calculate new joint angles to get the end effector closer to the goal.
* ```perform_newtons_method()```
   * **Description:** Using the current arm’s state, calculates the Jacobian (first derivative) and approximates the Hessian (second derivative) to calculate new joint angles to get the end effector closer to the goal.	


### ```main.py``` 
* The main file to run simulations of the arm
* Set number of joints in the arm, what function you want to perform on the arm, etc.
* Displays the progression of the arm's position after each iteration using matplotlib
* Able to display the distance to goal function of a 2 joint arm

### ```test.iypnb```
* The notebook version of our main.py file
* But unable to display the distance to goal functioni
* Uses FuncAnimation and HTML to display the animations in the output
## Results


## Tables figures (done last)

## Conclusion-findings (done last)

## Conclusion-limitations & future research

## Citations (ask for citations)
