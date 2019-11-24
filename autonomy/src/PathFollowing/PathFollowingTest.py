from SkidSteerSimulator import SkidSteerSimulator
from PathFollower import PathFollower
from PathPlanningUtils import Grid
import matplotlib.pyplot as plt
import numpy as np
import os
import glob

dt = 0.01
robot = SkidSteerSimulator(0, 2, 0)

# path = np.array([[0, 0], [2, 0.3], [4, 0.6], [5, 1.5], [6, 2.5], [9, 3.3]])
# path = np.array([[1, -1]]) * path
path = np.array([[0.5249999999999999, 2.025], [1.4249999999999998, 1.125], [2.775, 0.6749999999999999],
                 [3.975, 1.275], [5.025, 1.8], [6.225, 1.4249999999999998], [6.5, 0.5]])

controller = PathFollower(robot.reference_point_x, path=path)
controller.set_path(path)

os.makedirs("viz_dir", exist_ok=True)
try:
    files = glob.glob('%s/*' % "viz_dir")
    for f in files:
        os.remove(f)
except Exception as e:
    print(e)

draw_step = 30

target_vels = []
target_angular_vels = []
angular_vels = []
vels = []

for i in range(2000):
    target_vel, target_angular_vel = controller.get_target_vels(robot.state, robot.state_dot, dt)

    if robot.state_dot[0, 0] < target_vel:
        forward_torque = 30
    else:
        forward_torque = 20

    turn_torque = 7 * (target_angular_vel - robot.state_dot[2, 0])

    right_torque = forward_torque + turn_torque
    left_torque = forward_torque - turn_torque

    robot.update(right_torque, left_torque, dt)

    vels.append(robot.state_dot[0, 0])
    angular_vels.append(robot.state_dot[2, 0])
    target_vels.append(target_vel)
    target_angular_vels.append(target_angular_vel)

    if i % draw_step == 0:
        plt.clf()
        plt.axis('equal')
        axes = plt.gca()
        axes.set_xlim([-2, 8])
        axes.set_ylim([-3, 7])

        points = robot.draw(robot.state, robot.width, robot.length)
        direction_vectors, perp_vectors = robot.get_direction_vectors()
        path_aprox, closest_point, reference = controller.draw_path_info()

        plt.scatter(points[:, 0], points[:, 1])
        plt.quiver(points[:, 0], points[:, 1], direction_vectors[:, 0], direction_vectors[:, 1], angles='xy', scale_units='xy', scale=1)
        plt.plot(path[:, 0], path[:, 1])
        plt.scatter(reference[0, 0], reference[1, 0])
        plt.scatter(closest_point[0], closest_point[1])
        plt.plot(path_aprox[:, 0], path_aprox[:, 1])

        plt.savefig('viz_dir/fig_'+str(int(i/draw_step)))

# plt.figure()
# plt.plot(target_vels)
# plt.plot(vels)
# plt.plot(target_angular_vels)
# plt.plot(angular_vels, label='angular vel')
# plt.legend()
# plt.show()
#
# plt.figure()
# plt.plot(controller.errors)
# plt.show()
#
# plt.figure()
# plt.plot(controller.errors, controller.error_dots)
# plt.show()
