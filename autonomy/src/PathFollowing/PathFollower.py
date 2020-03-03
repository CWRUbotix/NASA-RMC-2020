import numpy as np
import PathFollowing.config as config
from PathPlanning.ThetaStar import create_path, checkBlocked
from PathPlanning.PathPlanningUtils import Position, constrain_angle


class PathFollower:
    def __init__(self, reference_point_x, goal=None, path=None):
        self.global_path = path
        self.local_path = None

        if path is None:
            self.goal_pos = goal
        else:
            self.goal_pos = Position(self.global_path[-1, 0], self.global_path[-1, 1])  # Used if path is manually set

        self.grid = None
        self.current_index = 0  # Fractional index of where robot's reference point is along path
        self.index = 0

        self.a = 0  # Coefficients of quadratic equation for the approximate path the robot is trying to follow
        self.b = 0
        self.c = 0

        self.state = np.zeros((3, 1))
        self.state_dot = np.zeros((3, 1))
        self.reference_point_x = reference_point_x

        self.target_angular_vel = 0
        self.closest_point = None

        self.alpha = 1
        self.r = 100

        self.last_s = 0

        self.errors = []
        self.error_dots = []

        self.done = False
        self.drive_backwards = -1  # -1 for backwards, 1 for forwards

    def update(self, state, state_dot):
        self.state = state
        self.state_dot = state_dot

        # If driving backwards reference point needs to be on other side
        self.reference_point_x = abs(self.reference_point_x) * np.sign(self.drive_backwards)

    def check_if_done(self):
        # Take dot product of vector going from last point to robot and from last point to second to last point
        # When this dot product is less than 0 the robot has "passed" the last point
        dx_r = self.state[0, 0] - self.global_path[-1, 0]
        dy_r = self.state[1, 0] - self.global_path[-1, 1]
        dx = self.global_path[-2, 0] - self.global_path[-1, 0]
        dy = self.global_path[-2, 1] - self.global_path[-1, 1]

        if dx * dx_r + dy * dy_r < 0 and self.current_index > len(self.global_path) - 2:
            self.done = True

    def calculate_path(self):
        # Only update path if robot is more than a 0.5m from the target
        dist = Position(self.state[0, 0], self.state[1, 0]).distanceTo(self.goal_pos)
        if not dist < 0.5 or self.global_path is None:
            path = create_path(Position(self.state[0, 0], self.state[1, 0]), self.goal_pos, self.grid)
            if path:
                self.global_path = np.array(path)
                self.current_index = 0  # new paths are created from robot's current position, so robot is at index 0
            else:
                print("Path could not be found, using old path")

    def set_path(self, path):
        self.global_path = path

    def get_path(self):
        return self.global_path

    def set_goal(self, goal):
        self.goal_pos = goal

    def update_grid(self, grid):
        self.grid = grid

    def set_drive_backwards(self, drive_backwards):
        self.drive_backwards = -1 if drive_backwards else 1

    def reset(self):
        self.done = False  # Reset everything that will not automatically be reset by the creation of a new path
        self.last_s = 0
        self.current_index = 0
        self.errors = []
        self.error_dots = []

    def is_path_blocked(self):
        for i in range(len(self.global_path) - 1):
            if checkBlocked(Position(*self.global_path[i]), Position(*self.global_path[i+1]), self.grid):
                return True
        return False

    def get_target_vels(self, state, state_dot, dt):
        self.update(state, state_dot)

        error, error_dot, = self.get_cross_track_error()

        self.errors.append(error)
        self.error_dots.append(error_dot)

        s = error_dot + config.lambda_e * error
        s_dot = (s - self.last_s) / dt
        self.last_s = s

        S = s * config.G_s
        S_dot = s_dot * config.G_s_dot

        offset = self.get_angular_offset()

        # position = self.state[:2]
        # line = self.global_path[self.index + 1] - position
        # dist = np.sqrt(line[0]**2 + line[1]**2)
        #
        # if dist * np.tan(abs(offset)) < 0.1: # If offset results in less than 10cm deviation
        #     offset = 0

        self.alpha = config.slowdown_controller.crisp_output(abs(offset))

        target_vel = 0
        target_angular_vel = 0

        self.check_if_done()  # Check if done, if we are, set vels to 0

        if not self.done:
            if abs(offset) < np.pi / 3:
                target_vel = config.target_velocity * self.alpha
                target_angular_vel = config.G_u * config.sliding_controller.crisp_output(S, S_dot)
            else:
                target_vel = 0
                target_angular_vel = -self.drive_backwards * 0.2 * np.sign(offset)  # + or - rad/s to turn around if facing wrong way

        # When driving backwards flip velocity and angular velcity
        # TODO: is this mathematically equivalent to changing measured vel and angular vel?
        target_vel *= self.drive_backwards
        target_angular_vel *= self.drive_backwards

        self.target_angular_vel = target_angular_vel  # Store value

        return target_vel, target_angular_vel

    def get_cross_track_error(self):
        position = self.state[:2]  # global position
        theta = self.state[2, 0]  # angle

        # rotation matrices from global to local frame
        rotation_matrix_reverse = np.array([[np.cos(theta), np.sin(theta)],
                                  [-np.sin(theta), np.cos(theta)]])

        x0 = self.reference_point_x  # reference point coordinates in local frame
        y0 = 0

        self.local_path = np.dot(rotation_matrix_reverse, (self.global_path-position.T).T).T  # convert path to local coordinates

        early_turn = 0.0
        self.index = min(max(int(self.current_index + early_turn), 0), len(self.local_path) - 2)

        is_linear = True
        self.a, self.b, self.c, phi, x, y, self.r = self._calculate_path_and_closest_point(self.index, is_linear, x0, y0)

        self.closest_point = np.array([[x], [y]])

        x_dot = self.state_dot[0, 0]  # get robot's velocity and angular velocity
        theta_dot = self.state_dot[2, 0]

        # get cross track error and the derivative of the cross track error
        error = ((x - x0)**2 + (y - y0)**2)**0.5 * np.sign(y)
        error_dot = x_dot * np.sin(phi) - self.reference_point_x * theta_dot * np.cos(phi)

        # Find projection of closest point onto line segment to determine how far along the line segment the robot is
        index = min(int(self.current_index), len(self.local_path)-2)
        start = self.local_path[index]
        segment = self.local_path[index + 1] - start
        fractional_index = (segment[0] * (x - start[0]) + segment[1] * (y-start[1]))/(segment[0]**2 + segment[1]**2)

        self.current_index = max(self.current_index, index + fractional_index)

        return error, error_dot

    def get_angular_offset(self):
        segment = self.global_path[self.index + 1] - self.global_path[self.index]
        angle = np.arctan2(segment[1], segment[0])

        if self.drive_backwards == -1:  # trick robot into thinking it is facing other way
            angle += np.pi

        return constrain_angle(self.state[2, 0] - angle)

    def _calculate_path_and_closest_point(self, index, is_linear, x0, y0):
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

        theta = self.state[2, 0]  # angle

        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])

        for i in range(4 + 1):
            start = self.local_path[self.index][0] - 1
            end = self.local_path[self.index + (1 if self.a == 0 else 2)][0] + 1
            x = (end - start) / 4 * i + start
            y = self.a * x * x + self.b * x + self.c
            path.append([x, y])

        path = np.array(path)
        path = (self.state[:2] + np.dot(rotation_matrix, path.T)).T

        # find the global coordinates of the closest point
        closest_point = np.dot(rotation_matrix, self.closest_point) + self.state[:2]

        # find global coords of reference point
        reference = self.state[:2] + np.dot(rotation_matrix, np.array([[self.reference_point_x], [0]]))

        return path, closest_point, reference
