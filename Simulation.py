import numpy as np
import Arm


# The thing that performs Newton's method on the arm
class Simulation:
    # takes in an arm and goal point
    def __init__(self, goal, arm:Arm, angle_change_limit = 0.1, use_newton = True):
        self.use_newton = use_newton
        self.angle_change_limit = angle_change_limit
        self.goal = goal # (x, y) tuple
        self.arm = arm
        
    # Draw (Takes arm and goal, plots on plt)
    def draw(self, plt):
        self.arm.draw(plt)
        plt.plot(self.goal[0], self.goal[1], 'g*')

    def update(self):
        if self.use_newton:
            diff = self.perform_newtons_method()
        else:
            diff = self.jacobian_pseudoinverse_ik()
        return diff

    # performs newton's method in optimization (Newton–Raphson) to get the arm to the goal
    # returns the change in position of the end effector
    def jacobian_pseudoinverse_ik(self):
        # Calculate the end effector position
        end_effector_position = self.arm.calculate_end_position()

        # Calculate the error vector
        error_vector = np.array([self.goal[0] - end_effector_position[0], self.goal[1] - end_effector_position[1]])

        # Calculate the Jacobian matrix
        J = self.arm.calculate_jacobian()

        # Note, is this actually newton's method?
        # Calculate the change in angles using the pseudo-inverse of the Jacobian
        delta_angles = np.linalg.pinv(J).dot(error_vector)
        # bound delta_angles to a maximum of 0.05 radians so it doesn't jump around too much
        delta_angles = np.clip(delta_angles, -self.angle_change_limit, self.angle_change_limit)

        # Update the arm's angles
        new_angles = np.array(self.arm.linkeage_angles) + delta_angles
        self.arm.change_angles(new_angles)

        new_end_effector_position = self.arm.calculate_end_position()
        # Return the magnitude of the error vector as a measure of convergence
        return np.linalg.norm(np.array(new_end_effector_position) - np.array(end_effector_position))
    
    def perform_newtons_method(self):
        end_effector = np.array(self.arm.calculate_end_position())
        error_vector = np.array(self.goal) - end_effector
        J = self.arm.calculate_jacobian()


        lambda_damping = 1e-1  # damping factor

        try:
            H = J.T @ J + lambda_damping * np.eye(J.shape[1])
            delta_angles = np.linalg.solve(H, J.T @ error_vector)
        except np.linalg.LinAlgError:
            print("⚠️ Damped solve failed, falling back to pseudoinverse.")
            delta_angles = np.linalg.pinv(J) @ error_vector

        # Clip and update angles
        delta_angles = np.clip(delta_angles, -self.angle_change_limit, self.angle_change_limit)
        new_angles = np.array(self.arm.linkeage_angles) + delta_angles
        self.arm.change_angles(new_angles)

        new_end_effector_position = self.arm.calculate_end_position()
        return np.linalg.norm(np.array(new_end_effector_position) - np.array(end_effector))



    
    
       

    