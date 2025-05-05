# Two-Joint-Arm-Optimization

## Description
This project simulates an n-joint robotic arm tasked with navigating to a goal in a 2D space from a random starting configuration.
For a simple 2-joint system in 2d space, it’s pretty easy to calculate what angles the joints have to be to reach the goal. However, as we move into 3d and with more joints, it becomes very difficult to calculate the joint angles analytically. Instead, we can use numerical methods to help calculate them.
This simulation will employ the Pseudo Inverse Jacobian and Newton's Method to calculate how the angles should change in the arm based on the first and second partial derivatives to get closer to the goal.
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

## Goal & Research Questions
Implement numerical methods to solve for joint angles to reach the goal.
Given a robotic arm of n linkages:
* Is it possible for the arm to reach the goal from any starting configuration?
* Does increasing the rotation per iteration limit or number of linkages improve the arm's ability to reach the goal state?
* Does Newton’s method converge faster than the Pseudo Inverse Jacobian method?

## Methods & Setup
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
* Set the number of joints in the arm, what function you want to perform on the arm, etc.
* Displays the progression of the arm's position after each iteration using matplotlib
* Able to display the distance to goal function of a 2-joint arm

### ```test.iypnb```
* The notebook version of our main.py file
* But unable to display the distance to goal function
* Uses FuncAnimation and HTML to display the animations in the output

## Results

### Motion of the robotic arm
- https://github.com/user-attachments/assets/1cfa17c6-080c-41bb-8b2e-7928e9209ea9

### Test Case 1: Goal lies on (0,0), base of the robotic arm.
Videos: animation_jac_base.mp4, animation_new_base.mp4
- https://github.com/user-attachments/assets/127e2e66-00e9-4641-acf4-56fa3ad3010a
- https://github.com/user-attachments/assets/389b6526-00a7-4bdb-a592-c5bc8e3dc698

### Test Case 2: Goal lies on (0,.1), right above the base of the robotic arm.
Videos: animation_jac_closegoal.mp4, animation_new_closegoal.mp4
- https://github.com/user-attachments/assets/8ffaa5b5-c619-4423-a291-763e825670c1
- https://github.com/user-attachments/assets/7255113b-104c-47d0-ac41-b418cf3fd358

### Test Case 3: Goal doesn’t exist, but points to somewhere nearby the base.
Videos: animation_jac_nogoal.mp4, animation_new_nogoal.mp4
- https://github.com/user-attachments/assets/9d000bbd-b3d1-46ce-8c99-355f22a028b4
- https://github.com/user-attachments/assets/cf724c6e-cf2d-4b94-b9d4-213ba4739518

### Test Case 4: Goal lies on (2, 1), just out of reach of the robotic arm.
Videos: animation_jac_noreach_2link.mp4, animation_new_noreach_2link.mp4
- https://github.com/user-attachments/assets/98325379-b9f7-45cc-818c-746ddb8d8d1c
- https://github.com/user-attachments/assets/11234d55-d657-4641-9801-3290fd782b82

### Test Case 5: Goal lies on (-30, -20), 50 link arm
Videos: animation_jac_normal.mp4, animation_new_normal.mp4
- https://github.com/user-attachments/assets/f2c4c6d9-5bc7-4bce-a910-f9454eb7c80a
- https://github.com/user-attachments/assets/5009505d-1e8d-4d57-9f7b-305bb8ce295f

### Test Case 6: Goal lies on (30, 0), overlaid inside the robotic arm. 50 link arm
Videos: animation_jac_layover.mp4, animation_new_layover.mp4
- https://github.com/user-attachments/assets/bdeba6d0-7314-4598-aa16-08bb56fe3875
- https://github.com/user-attachments/assets/a689c841-f357-4e1d-b5c9-4f628bcf91a9

### Test Case 7: Goal lies on (-30, 0), 50 link arm
Videos: animation_jac_opposite.mp4, animation_new_opposite.mp4
- https://github.com/user-attachments/assets/ef8d8cac-1414-4cd5-b7e7-a61f70c1ec22
- https://github.com/user-attachments/assets/58c9330c-6313-40d2-b1fa-db794b676cf5

### Test Case 8: Goal lies on (-30, 40), just out of reach of the robotic arm, 50 link arm
Videos: animation_jac_noreach.mp4, animation_new_noreach.mp4
- https://github.com/user-attachments/assets/d869c573-8897-449f-a65d-225e39bcd296
- https://github.com/user-attachments/assets/439ecbcf-33a9-43ff-9ca3-7c7872aa81b4

### Test Case 9: Goal lies on (0, .5), 2 link arm
Videos: animation_jac_imposs.mp4, animation_new_imposs.mp4
- https://github.com/user-attachments/assets/e4174272-145f-43ba-801c-2cbb77e07560
- https://github.com/user-attachments/assets/93c31801-16b0-45ab-8403-e57153bb4db5

<br>

| Test Case #   | Numerical Method | Iterations    | Converged? | Distance to goal |
|---------------|------------------|---------------|------------|------------------|
| 1             | Jacobian         | 2             | Y          | 10               |
| 1             | Newton’s         | 2             | Y          | 10               |
| 2             | Jacobian         | 3             | Y          | 2                | 
| 2             | Newton’s         | 3             | Y          | 2                | 
| 3             | Jacobian         | 999           | N          | Null             |
| 3             | Newton’s         | 999           | N          | Null             |
| 4             | Jacobian         | 265           | N          | .26              |
| 4             | Newton’s         | 999           | N          | .26              |
| 5             | Jacobian         | 28            | Y          | 72.89            |
| 5             | Newton’s         | 27            | Y          | 72.89            |
| 6             | Jacobian         | 61            | Y          | 21.38            |
| 6             | Newton’s         | 38            | Y          | 21.38            |
| 7             | Jacobian         | 159           | Y          | 77.19            |
| 7             | Newton’s         | 52            | Y          | 77.19            |
| 8             | Jacobian         | 999           | N          | 66.54            |
| 8             | Newton’s         | 999           | N          | 66.54            |
| 9             | Jacobian         | 0             | N          | 1.5              |
| 9             | Newton’s         | 0             | N          | 1.5              |


## Conclusion
So, did we achieve our goal? Yes, we did! As we can see above in the results, multiple instances of the arm successfully reaching the goal.
But what about our questions? Well:
* **Is it possible for the arm to reach the goal from any starting configuration?**
   * No! There are some configurations that make it mathematically impossible for the arm to get to the goal, and there are some scenarios where the arm gets into a non-converging oscillation. The worst thing is that since some of these configurations don’t settle down at a point, we can’t detect that the arm is in a non-converging state except by having an iteration limit.
     
* **Does Newton’s method converge faster than the Pseudo Inverse Jacobian method?**
   * Almost always. For low joint systems, we can see that the Pseudo Inverse Jacobian method takes the same iterations as Newton’s method to converge. However, we can see in our 50 joint systems that Newton often converges much faster than the Pseudo Inverse Jacobian. So in higher-order systems, Newton’s method converges faster
     
* **Does increasing the rotation per iteration limit or number of linkages improve the arm's ability to reach the goal state?**
   * Yes, it can. We often get into a non-converging oscillation when our rotation limit is relatively high, so damping how far the arm can move can help converge the system.

## Limitations & Future Research
But what are some limitations of our simulation?
   * **Computation Time**
      * We did mention that Newton’s method often converges faster than Pseudo Inverse in high-order systems, but we did not take into account the computation time between the two methods. Newton’s method takes much more time to compute/approximate the Hessian matrix, so in some scenarios, Pseudo Inverse could calculate results faster, but in our simulation, we only tracked the number of iterations of each method.
   * **Local Minima**
      * Although they don’t exist in our simulation, there are often local minima that do occur in real life and are a problem for both methods we used. For instance, if we included walls that the arm could not pass through in our simulation, we’d introduce local minima, and if our arm gets trapped in one of them, we’d improperly stop thinking we’ve converged.
   * **Robot abstractions**
      * We simplified our robot arm to a straight linkage with an unbounded angle offset to implement these methods more easily. But we lose out on the real-world challenges that people often face. Arms aren’t always straight, arms can’t overlap each other, robots aren’t 2d, joints sometimes are bounded, and barriers may exist.  

These limitations could be expanded into our future research. Comparing the time of computation between the two methods, researching how people solve the local minima problem, and how to better simulate real-world challenges.

## References
* Correll, N., Heckman, C., & Roncone, A. (2022). Introduction to autonomous robots: Mechanisms, sensors, actuators, and algorithms. The MIT Press.
* Beuken, L. (2025, February 28). Unconstrained Optimization 2 [PowerPoint slides]. Department of Robotics, University of Colorado Boulder. 

