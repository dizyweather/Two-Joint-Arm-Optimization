import numpy as np


# The thing that performs Newton's method on the arm
# and draws
class Simulation:
    # takes in an arm and goal point
    def __init__(self, goal, arm):
        self.goal = goal # (x, t) tuple
        self.arm = arm
        
    # Draw (Takes arm and goal, plots on plt)
    def draw(self, plt):
        self.arm.draw(plt)
        plt.plot(self.goal[0], self.goal[1], 'g*')

    # performs newton's method in optimization to get the arm to the goal
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
        # bound delta_angles to a maximum of 0.05 radians
        delta_angles = np.clip(delta_angles, -0.1, 0.1)

        # Update the arm's angles
        new_angles = np.array(self.arm.linkeage_angles) + delta_angles
        self.arm.change_angles(new_angles)

        # Return the magnitude of the error vector as a measure of convergence
        return np.linalg.norm(error_vector)
    

#  # caluclate the jacobian (partial derivatives of the change in joint angles with respect to the end effector position)
#         J = self.arm.calculate_jacobian()

#         # calculate the end effector position
#         end_effector_position = self.arm.calculate_end_position()

#         # calculate the position error
#         error_vector = np.array([self.goal[0] - end_effector_position[0], self.goal[1] - end_effector_position[1]])
        
#         # calculate the change in angles using the pseudo-inverse of the Jacobian
#         J_pseudo_inverse = np.linalg.pinv(J)
#         delta_angles = J_pseudo_inverse @ error_vector

#         # bound delta_angles to a maximum of 0.1 radians
#         delta_angles = np.clip(delta_angles, -0.05, 0.05)

#         # update the arm's angles
#         new_angles = self.arm.linkeage_angles + delta_angles
#         self.arm.change_angles(new_angles)

#         # recalculate the end effector position
#         new_end_effector_position = self.arm.calculate_end_position()

#         # calculate the change in position
#         change_in_position = np.sqrt((new_end_effector_position[0] - end_effector_position[0])**2 + (new_end_effector_position[1] - end_effector_position[1])**2)
        
#         # return the change in position
#         return change_in_position
       
    
    
       

    