import numpy as np
import Arm

# The thing that performs Newton's method on the arm
class Simulation:
    # takes in an arm, goal point, angle change limit, and if we want to use newton's method or not
    def __init__(self, goal, arm:Arm, angle_change_limit = 0.1, use_newton = True):
        self.use_newton = use_newton
        self.angle_change_limit = angle_change_limit
        self.goal = goal # (x, y) tuple
        self.arm = arm
        
    # Draw (Takes arm and goal, plots on plt)
    def draw(self, plt):
        self.arm.draw(plt)
        plt.plot(self.goal[0], self.goal[1], 'g*')

    # Depending on use_newton, either performs newton's method or the jacobian pseudoinverse method
    def update(self):
        if self.use_newton:
            diff = self.perform_newtons_method()
        else:
            diff = self.jacobian_pseudoinverse_ik()
        return diff

    # performs jacobian_pseudoinverse inverse kineamtics method on the arm (uses first derivative (slope) to find the next angle)
    def jacobian_pseudoinverse_ik(self):
        # Calculate the end effector position
        end_effector_position = self.arm.calculate_end_position()

        # Calculate the error vector
        error_vector = np.array([self.goal[0] - end_effector_position[0], self.goal[1] - end_effector_position[1]])

        # Calculate the Jacobian matrix (matrix of first order partial derivatives)
        # aka the rate of change of the end effector position with respect to a given joint angles
        # 2 x n matrix where n is the number of linkeages
        J = self.arm.calculate_jacobian()

        # Calculate the change in angles using the pseudo-inverse of the Jacobian
        delta_angles = np.linalg.pinv(J).dot(error_vector)

        # bound delta_angles to the set maximum so it doesn't jump around too much
        delta_angles = np.clip(delta_angles, -self.angle_change_limit, self.angle_change_limit)

        # Update the arm's angles
        new_angles = np.array(self.arm.linkeage_angles) + delta_angles
        self.arm.change_angles(new_angles)

        # Return the magnitude of the error vector
        new_end_effector_position = self.arm.calculate_end_position()
        return np.linalg.norm(np.array(new_end_effector_position) - np.array(end_effector_position))
    
    # performs newton's method on the arm (uses first (slope) and second (curvature) derivatives to find the next angle)
    def perform_newtons_method(self):
        # calculate the error vector from where the end effector is now to the goal
        end_effector = np.array(self.arm.calculate_end_position())
        error_vector = np.array(self.goal) - end_effector
        
        # same as above
        J = self.arm.calculate_jacobian()

        # damping factor to prevent overshooting
        # this is a small value that is added to the diagonal of the Hessian matrix to make it invertible
        lambda_damping = 1e-5  

        try:
            # calculate a approximated Hessian matrix (second partial derivative) using the Jacobian
            # calculating the true Hessian matrix of a high dimensional function is expensive
            # so we use the Jacobian to approximate it
            H = J.T @ J + lambda_damping * np.eye(J.shape[1])

            # solve for the change in angles using the approximated Hessian matrix
            delta_angles = np.linalg.solve(H, J.T @ error_vector)
        except np.linalg.LinAlgError:
            print("Damped solve failed, falling back to pseudoinverse.")
            delta_angles = np.linalg.pinv(J) @ error_vector

        # Clip and update angles
        delta_angles = np.clip(delta_angles, -self.angle_change_limit, self.angle_change_limit)
        new_angles = np.array(self.arm.linkeage_angles) + delta_angles
        self.arm.change_angles(new_angles)

        # Return the magnitude of the error vector
        new_end_effector_position = self.arm.calculate_end_position()
        return np.linalg.norm(np.array(new_end_effector_position) - np.array(end_effector))



    
    
       

    