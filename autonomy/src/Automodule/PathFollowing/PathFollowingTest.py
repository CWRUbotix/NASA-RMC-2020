from PathFollowing.SkidSteerSimulator import SkidSteerSimulator
from PathFollowing.PathFollower import PathFollower
from PathFollowing import config
import matplotlib.pyplot as plt
import numpy as np

dt = 0.01
robot = SkidSteerSimulator(0, 0, 0)

# path = np.array([[0, 0], [2, 0.3], [4, 0.6], [5, 1.5], [6, 2.5], [9, 3.3]])
# path = np.array([[1, -1]]) * path
path = np.array([[0.5249999999999999, 2.025 ], [1.4249999999999998, 1.125], [2.775, 0.6749999999999999 ],
                 [3.975, 1.275 ], [5.025, 1.8 ], [ 6.225, 1.4249999999999998], [6.5, 0.5]])

controller = PathFollower(config.fuzzy_controller, config.slowdown_controller, path, robot.reference_point)
controller.set_forward_torque(config.forward_torque)

torques = []
for i in range(2000):
    right_torque, left_torque = controller.get_wheel_torques(robot.state, robot.state_dot)

    torques.append(controller.turn_torque)

    robot.update(right_torque, left_torque, dt)
    n = 50
    if i % n == 0:
        if robot.state_dot[0, 0] > 0.5:
            forward_torque = 29
        else:
            forward_torque = 30

        plt.clf()
        plt.axis('equal')
        axes = plt.gca()
        axes.set_xlim([-2, 8])
        axes.set_ylim([-3, 7])

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
