import numpy as np


class PathFollower:
    def __init__(self, fuzzy_controller, path, reference_point):
        self.fuzzy_controller = fuzzy_controller
        self.global_path = path
        self.local_path = path
        self.current_index = 0  # Fractional index of where robot's reference point is along path

        self.a = 0  # Coeffecients of quadratic equation for the approximate path the robot is trying to follow
        self.b = 0
        self.c = 0

        self.robot_state = None
        self.robot_state_dot = None
        self.reference_point = reference_point

        self.turn_torque = 0
        self.closest_point = None

    def update(self, robot_state, robot_state_dot):
        self.robot_state = robot_state
        self.robot_state_dot = robot_state_dot

    def get_turn_torque(self, robot_state, robot_state_dot):
        self.update(robot_state, robot_state_dot)

        error, error_dot, = self.get_cross_track_error()

        torque = self.fuzzy_controller.crisp_output(error, error_dot)

        self.turn_torque = torque

        return torque

    def get_cross_track_error(self):
        position = self.robot_state[:2]  # global position
        theta = self.robot_state[2, 0]  # angle

        # rotation matrices from global to local frame
        rotation_matrix_reverse = np.array([[np.cos(theta), np.sin(theta)],
                                  [-np.sin(theta), np.cos(theta)]])

        x0 = self.reference_point[0, 0]  # reference point coordinates in local frame
        y0 = self.reference_point[1, 0]

        self.local_path = np.dot(rotation_matrix_reverse, (self.global_path-position.T).T).T  # convert path to local coordinates

        index = min(max(int(self.current_index-0.5), 0), len(self.local_path) - 2)

        is_linear = (index == len(self.local_path) - 2)
        # is_linear = False
        self.a, self.b, self.c, phi, x, y, r = self.calculate_path_and_closest_point(index, is_linear, x0, y0)

        self.closest_point = np.array([[x], [y]])

        x_dot = self.robot_state_dot[0, 0]  # get robot's velocity and angular velocity
        theta_dot = self.robot_state_dot[2, 0]

        # get cross track error and the derivative of the cross track error
        error = ((x - x0)**2 + (y - y0)**2)**0.5 * np.sign(y)
        error_dot = x_dot * np.sin(phi) - self.reference_point[0, 0] * theta_dot * np.cos(phi)

        self.current_index = max(self.current_index, index + (((self.local_path[index][0] - x)**2 + (self.local_path[index][1]-y)**2)**0.5) /
                                 ((self.local_path[index][0] - self.local_path[index+1][0])**2 + (self.local_path[index][1]-self.local_path[index+1][1])**2)**0.5)

        return error, error_dot

    def calculate_path_and_closest_point(self, index, is_linear, x0, y0):
        a, b, c, m, x, y, r = 0, 0, 0, 0, 0, 0, 0  # coefficients of path, slope of tangent at closet point, x and y at point

        if is_linear:
            p1 = self.local_path[index]
            p2 = self.local_path[index + 1]

            a = 0
            b = (p2[1] - p1[1]) / (p2[0] - p1[0])  # calculate slope of path
            c = - b * p1[0] + p1[1]  # and intercept in local coords

            x = (x0 - b * c + b * y0) / (1 + b ** 2)  # x coord of closest point
            y = b * x + c  # y coord of closest point

            m = b
        else:
            m, n = self.local_path[index]
            p, r = self.local_path[index + 1]
            s, w = self.local_path[index + 2]

            d = m * m * p + s * s * m + s * p * p - s * s * p - s * m * m - p * p * m
            a = (n * p + m * w + r * s - w * p - s * n - r * m) / d
            b = (m * m * r + n * s * s + p * p * w - s * s * r - m * m * w - p * p * n) / d
            c = (m * m * p * w + m * r * s * s + n * s * p * p - s * s * p * n - s * r * m * m - w * m * p * p) / d

            coeff = [2 * a * a, 3 * a * b, 1 + 2 * a * c + b * b - 2 * a * y0, b * c - b * y0 - x0]

            # Find the furthest to the right real solution
            roots = np.roots(coeff)
            reals = np.real(roots[np.imag(roots) == 0])

            x = max(reals)
            y = a * x * x + b * x + c

            m = 2 * a * x + b  # tangent line to closest point

        if a != 0:
            r = ((1 + m**2)**1.5)/abs(2 * a)  # radius of curvature
        else:
            r = 100

        phi = np.arctan(m)

        return a, b, c, phi, x, y, r

    def draw_path_info(self):
        path = []

        theta = self.robot_state[2, 0]  # angle
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])

        index = min(max(int(self.current_index-0.5), 0), len(self.local_path) - 2)

        for i in range(10 + 1):
            start = self.local_path[index][0] - 1
            end = self.local_path[index + (1 if self.a == 0 else 2)][0] + 1
            x = (end - start) / 10 * i + start
            y = self.a * x * x + self.b * x + self.c
            path.append([x, y])

        path = np.array(path)
        path = (self.robot_state[:2] + np.dot(rotation_matrix, path.T)).T

        # find the global coordinates of the closest point
        closest_point = np.dot(rotation_matrix, self.closest_point) + self.robot_state[:2]

        # find global coords of reference point
        reference = self.robot_state[:2] + np.dot(rotation_matrix, self.reference_point)

        return path, closest_point, reference

