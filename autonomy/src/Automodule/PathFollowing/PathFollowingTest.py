from PathFollowing.SkidSteerSimulator import SkidSteerSimulator
from PathFollowing.FuzzyLogic import FuzzyLogic, FuzzySet
from PathFollowing.PathFollower import PathFollower
import matplotlib.pyplot as plt
import numpy as np

SCALE = 70
arena_width = 8
arena_height = 8

dt = 0.01
robot = SkidSteerSimulator(0, 2, 0)

# path = np.array([[0, 0], [2, 0.3], [4, 0.6], [5, 1.5], [6, 2.5], [9, 3.3]])
# path = np.array([[1, -1]]) * path
path = np.array([[0.5249999999999999, 2.025 ], [1.4249999999999998, 1.125], [2.775, 0.6749999999999999 ],
                 [3.975, 1.275 ], [5.025, 1.8749999999999998 ], [ 6.225, 1.4249999999999998], [6.5, 0.5]])

forward_torque = 29.7

error_centers = np.array([-0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6])
error_left_widths = np.array([1000, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
error_right_widths = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1000])

error_dot_centers = np.array([-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3])
error_dot_left_widths = np.array([1000, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
error_dot_right_widths = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1000])

error_set = FuzzySet(error_centers, error_left_widths, error_right_widths)
error_dot_set = FuzzySet(error_dot_centers, error_dot_left_widths, error_dot_right_widths)

NB, NM, NS, ZO, PS, PM, PB = -1.5 * np.pi, -1 * np.pi, -0.5 * np.pi, 0, 0.5 * np.pi, 1 * np.pi, 1.5 * np.pi
rules = np.array([[NB, NB, NM, NB, NM, NS, ZO],
                  [NB, NB, NM, NM, NS, ZO, PS],
                  [NB, NM, NS, NS, ZO, PS, PM],
                  [NB, NM, NS, ZO, PS, PM, PB],
                  [NM, NS, ZO, PS, PS, PM, PB],
                  [NS, ZO, PS, PM, PM, PB, PB],
                  [ZO, PS, PM, PB, PM, PB, PB]])

fuzzy_controller = FuzzyLogic(rules, error_set, error_dot_set)

controller = PathFollower(fuzzy_controller, path, robot.reference_point)

torques = []
for i in range(2000):
    torque = controller.get_turn_torque(robot.state, robot.state_dot)
    torque = 0.3 * torque

    torques.append(torque)

    robot.update(forward_torque + torque, forward_torque - torque, dt)
    n = 50
    if i % n == 0:
        plt.clf()
        plt.axis('equal')
        axes = plt.gca()
        axes.set_xlim([-5, 10])
        axes.set_ylim([-5, 10])

        points, direction_vectors, perp_vectors = robot.draw()
        path_aprox, closest_point, reference = controller.draw_path_info()

        plt.scatter(points[:, 0], points[:, 1])
        plt.quiver(points[:, 0], points[:, 1], direction_vectors[:, 0], direction_vectors[:, 1], angles='xy', scale_units='xy', scale=1)
        plt.plot(path[:, 0], path[:, 1])
        plt.scatter(reference[0, 0], reference[1, 0])
        plt.scatter(closest_point[0], closest_point[1])
        plt.plot(path_aprox[:, 0], path_aprox[:, 1])

        plt.savefig('fig_'+str(int(i/n)))

# plt.figure()
# plt.plot(torques)
# plt.show()
