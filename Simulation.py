import numpy as np
import Arm


# The thing that performs Newton's method on the arm
# and draws
class Simulation:
    # takes in an arm and goal point
    def __init__(self, goal, arm:Arm):
        self.goal = goal # (x, t) tuple
        self.arm = arm
        
    # Draw (Takes arm and goal, plots on plt)
    def draw(self, plt):
        self.arm.draw(plt)
        plt.plot(self.goal[0], self.goal[1], 'g*')

    # performs newton's method in optimization (Newton–Raphson) to get the arm to the goal
    # returns the change in position of the end effector
    def perform_newtons_method(self):
        # Calculate the end effector position
        end_effector_position = self.arm.calculate_end_position()

        # Calculate the error vector
        error_vector = np.array([self.goal[0] - end_effector_position[0], self.goal[1] - end_effector_position[1]])

        # Calculate the Jacobian matrix
        J = self.arm.calculate_jacobian()

        # Calculate the change in angles using the pseudo-inverse of the Jacobian
        delta_angles = np.linalg.pinv(J).dot(error_vector)
        # bound delta_angles to a maximum of 0.05 radians so it doesn't jump around too much
        delta_angles = np.clip(delta_angles, -0.1, 0.1)

        # Update the arm's angles
        new_angles = np.array(self.arm.linkeage_angles) + delta_angles
        self.arm.change_angles(new_angles)

        # Return the magnitude of the error vector as a measure of convergence
        return np.linalg.norm(error_vector)
    
    
       

    