import numpy as np


class SkidSteerSimulator:
    def __init__(self, X, Y, theta):
        self.X = X  # Global x position
        self.Y = Y  # Global y position
        self.theta = theta  # Robot angle

        self.X_dot = 0  # X velocity in robot local frame
        self.Y_dot = 0  # Y velocity in robot local frame
        self.theta_dot = 0  # angular velocity

        self.width = 0.75  # robot width
        self.length = 1  # robot length (between wheels)
        self.mass = 50  # robot mass
        self.r = 0.2  # wheel radius
        self.moment_inertia = 4.5  # robot moment of inertia around center of gravity
        self.friction_long = 0.6  # coefficient of friction when the robot moves forward
        self.friction_lat = 0.6  # coefficient of friction for the robot moving sideways

        # Distance from horizontal centerline to back and front wheels
        self.a = 1
        self.b = 1

        self.g = 9.8  # gravity

        self.state = np.array([[self.X, self.Y, self.theta]]).T  # Robot's global state
        self.state_dot = np.array([[0, 0, 0]], dtype='float').T  # Rate of change of robot's local state
        self.M = np.array([[1 / self.mass, 0, 0],
                           [0, 1 / self.mass, 0],
                           [0, 0, 1 / self.moment_inertia]])  # Divide by mass matrix for translation and rotation

        self.reference_point_x = self.length/2  # Coords of point robot measures path error from

    def update(self, torque_right, torque_left, dt):
        theta = self.state[2, 0]  # get robot's direction

        direction_vectors, _ = self.get_direction_vectors() # Get the direction each wheel of the robot is going
        direction_vectors = np.dot(np.array([[np.cos(theta), np.sin(theta)],  # rotate direction vectors to local frame
                                             [-np.sin(theta), np.cos(theta)]]), direction_vectors.T).T

        # The direction the wheels are going determines the friction forces on the robot
        wheel_1 = direction_vectors[0]
        wheel_2 = direction_vectors[1]
        wheel_3 = direction_vectors[2]
        wheel_4 = direction_vectors[3]

        wheel_1_x = wheel_1[0]
        wheel_2_x = wheel_2[0]

        wheel_1_y = wheel_1[1]
        wheel_3_y = wheel_3[1]

        # if not (wheel_1_y==0 or wheel_4[1] == 0):
        #     slope1 = wheel_1_x/wheel_1_y
        #     slope2 = -wheel_4[0]/wheel_4[1]
        #     a = slope2/(slope1 + slope2) * self.length
        #     b = self.length - a
        #
        #     self.a = a
        #     self.b = b

        # R_x is friction in the longitudinal direction (x)
        R_x = self.friction_long * (self.mass * self.g / 2) * (np.sign(wheel_1_x) + np.sign(wheel_2_x))

        # F_y is friction in the y direction (sideways)
        F_y = self.friction_lat * self.mass * self.g / (self.a + self.b) * \
              (self.b * np.sign(wheel_1_y) + self.a * np.sign(wheel_3_y))

        # M_r is rotating friction
        M_r = self.friction_lat * self.mass * self.g * self.a * self.b / (self.a + self.b) * \
              (np.sign(wheel_1_y) - np.sign(wheel_3_y)) + \
              self.friction_long * self.width * (self.mass * self.g / 4) * \
              (np.sign(wheel_2_x) - np.sign(wheel_1_x))

        # Rotation matrix to convert robot's local state dot to a change in global state (x, y, theta)
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                   [np.sin(theta), np.cos(theta), 0],
                                   [0, 0, 1]])
        # C = np.array([[R_x * np.cos(theta) - F_y * np.sin(theta)],
        #               [R_x * np.sin(theta) + F_y * np.cos(theta)],
        #               [M_r]])
        # E = np.array([[np.cos(theta) / self.r, np.cos(theta) / self.r],
        #               [np.sin(theta) / self.r, np.sin(theta) / self.r],
        #               [self.width / (2 * self.r), -self.width / (2 * self.r)]])

        # F_y = 0
        M_r = 0  # TODO M_r is set to 0 here because I was having trouble with rotating friction
        # R_x = 0

        # print(R_x, F_y, M_r)

        # E describes how torques are transfered into forces in the 3 directions (x, y, theta)
        E = np.array([[1 / self.r, 1 / self.r],
                      [0, 0],
                      [self.width / (2 * self.r), -self.width / (2 * self.r)]])

        # C is the resistance in those directions
        C = np.array([[R_x],
                      [-F_y],
                      [M_r]])

        T = np.array([[torque_right, torque_left]]).T  # Input torques


        force = np.dot(E, T)  # Calculate forces
        resistance = C

        state_dot_dot = np.dot(self.M, force - resistance)  # acceleration is force - resistance * 1/M
        # state_dot_dot = np.dot(self.M, np.dot(E, T) - C)
        # self.state_dot = self.state_dot + state_dot_dot * dt

        # The idea with this for loop is that a resistance force should not be able to start the robot moving
        # in the other direction. This loop tries to look at each force and cap the resulting velocity.
        # I think it ignores static friction. This is most apparent when turning
        for i in range(len(force)):
            f = force[i, 0]
            r = resistance[i, 0]
            if abs(f) < abs(r):
                if self.state_dot[i, 0] > 0:
                    self.state_dot[i, 0] = max(0, self.state_dot[i, 0] + state_dot_dot[i, 0] * dt)
                elif self.state_dot[i, 0] < 0:
                    self.state_dot[i, 0] = min(0, self.state_dot[i, 0] + state_dot_dot[i, 0] * dt)
            # elif abs(f) < abs([294, 294, 400][i]):
            #     # print(f, i)
            #     pass
            else:
                self.state_dot[i, 0] = self.state_dot[i, 0] + state_dot_dot[i, 0] * dt

        # print(self.state_dot[2, 0], force[2, 0], resistance[2, 0])

        # Update global state by rotating local state to convert it to change in global state
        self.state = self.state + np.dot(rotation_matrix, self.state_dot) * dt
        # self.state = self.state + self.state_dot * dt

    @staticmethod
    def draw(state, width, length):
        # get x y and theta in global coords
        x = state[0, 0]
        y = state[1, 0]
        theta = state[2, 0]

        # A rotation matrix to convert local coords to global
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                  [np.sin(theta), np.cos(theta)]])

        points = np.array([[1, 0.8], [1, -0.8], [-1, -1], [-1, 1]])  # robot edges
        points = points * np.array([length / 2, width / 2])  # multiply by width and length
        points = np.dot(rotation_matrix, points.T)  # rotate the points to the global frame

        points = points.T + np.array([x, y])  # Translate by the robot's center

        return points

    # Returns list of vectors that are which direction each wheel is traveling
    def get_direction_vectors(self):
        theta = self.state[2, 0]  # get angle

        x_dot = self.state_dot[0, 0]  # get x y and theta velocity in local frame
        y_dot = self.state_dot[1, 0]
        theta_dot = self.state_dot[2, 0]

        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],  # convert from local to global
                                    [np.sin(theta), np.cos(theta)]])

        velocity_vectors = np.array([[x_dot, y_dot]])  # translational component of wheel velocities
        velocity_vectors = np.dot(rotation_matrix, velocity_vectors.T).T  # rotate the vectors to global frame

        # each rotation vector is perpendicular to the line drawn from the center of the robot to the corner.
        # its velocity is equal to the angular velocity/radius
        rotation_vectors = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]]) * np.array([self.width/2, self.length/2]) * \
                           np.sqrt((self.width/2) ** 2 + (self.length/2) ** 2) * theta_dot
        rotation_vectors = np.dot(rotation_matrix, rotation_vectors.T).T  # Rotate these vectors to match global frame

        direction_vectors = velocity_vectors + rotation_vectors  # direction is sum of rotation and velocity

        perp_vectors = np.dot(np.array([[0, -1],  # These are the vectors perpendicular to the direction vectors,
                                  [1, 0]]), direction_vectors.T).T * 5  # used to find instantaneous center of rotation
                                                                        # (I'm not doing this, but you could)

        return direction_vectors, perp_vectors


