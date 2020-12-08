#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np

bag_path = "/home/edf42001/sensor_noise.bag"
bag = rosbag.Bag(bag_path)


obstacle_data = np.array((0, 2))
dist_angle_data = np.array((0, 2))

for topic, msg, t in bag.read_messages(topics=['/obstacles/coefficients', '/output_models_test']):
    if topic == "/obstacles/coefficients":
        if len(msg.values) > 0:
            x = msg.values[0]
            y = msg.values[1]

            # dist = x**2 + y**2
            # angle = np.arctan2(y, x)

            # obstacle_data = np.vstack((obstacle_data, [x, y]))
            # dist_angle_data = np.vstack((dist_angle_data, [dist, angle]))
    elif topic == "/output_models_test":
        x = msg.point.x
        y = msg.point.y

        # dist = x**2 + y**2
        # angle = np.arctan2(y, x)

        obstacle_data = np.vstack((obstacle_data, [x, y]))
        # dist_angle_data = np.vstack((dist_angle_data, [dist, angle]))


# plt.plot(obstacle_data[:, 0])
# plt.plot(obstacle_data[:, 1])
plt.xlim(-0.1, 0.1)
plt.ylim(1.4, 1.6)
plt.scatter(obstacle_data[:, 0], obstacle_data[:, 1])

print(np.cov(obstacle_data.T))


# plt.scatter(dist_angle_data[:, 0], dist_angle_data[:, 1])
# plt.plot(dist_angle_data[:, 0])
# plt.plot(dist_angle_data[:, 1])

plt.show()

bag.close()