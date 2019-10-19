import numpy as np

class PathFollower:
    def __init__(self, fuzzy_controller, path):
        self.fuzzy_controller = fuzzy_controller
        self.path = path

    def get_turn_torque(self, robot_state, robot_state_dot, reference_point):
        closest_point, reference, error, error_dot = self.get_cross_track_error(self.path, robot_state, robot_state_dot,
                                                                                reference_point)

        torque = self.fuzzy_controller.crisp_output(error, error_dot)

        return torque, closest_point, reference

    def get_cross_track_error(self, path, robot_state, robot_state_dot, reference_point):
        position = robot_state[:2]  # global position
        theta = robot_state[2, 0]  # angle

        # rotation matrices to and from global frame
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
        rotation_matrix_reverse = np.array([[np.cos(theta), np.sin(theta)],
                                  [-np.sin(theta), np.cos(theta)]])

        x0 = reference_point[0, 0]  # reference point coordinates in local frame
        y0 = reference_point[1, 0]

        path = np.dot(rotation_matrix_reverse, (path-position.T).T).T  # convert path to local coordinates

        # for now we assume path is linear
        p1 = path[2]
        p2 = path[3]

        m = (p2[1]-p1[1])/(p2[0]-p1[0])  # calculate slope of path
        b = - m * p1[0] + p1[1]  # and intercept in local coords

        x = (x0 - m * b + m * y0) / (1 + m**2)  # x coord of closest point
        y = m * x + b  # y coord of closest point

        x_dot = robot_state_dot[0, 0]  # get robot's velocity and angular velocity
        theta_dot = robot_state_dot[2, 0]

        # get cross track error and the derivative of the cross track error
        error = ((x - x0)**2 + (y - y0)**2)**0.5 * np.sign(y)
        error_dot = x_dot * np.sin(np.arctan(m)) - reference_point[0, 0] * theta_dot * np.cos(np.arctan(m))

        # find the global coordinates of the closest point
        closest_point = np.array([[x, y]]).T
        closest_point = np.dot(rotation_matrix, closest_point) + position

        # find global coords of reference point
        reference = robot_state[:2] + np.dot(rotation_matrix, reference_point)

        return closest_point, reference, error, error_dot


