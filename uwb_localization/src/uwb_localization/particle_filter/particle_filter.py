#!/usr/bin/env python3 

import signal
import sys
import os

import matplotlib.pyplot as plt
import numpy as np

from particle import Particle, MeasurementMode, euclidean_distance
from weighted_distribution import WeightedDistribution


def signal_handler(sig, frame):
    plt.close()
    sys.exit(0)


def create_folder(folder_name):
    if not os.path.exists(folder_name):
        os.mkdir(folder_name)


class ParticleFilter:
    def __init__(self, world, std_dev, n_particles=1000):
        self.n_particles = n_particles
        self.particles = [Particle(0, 0, 0, randomize=True, std_dev=[1, 1, 0.1, 0.2, 0.2, 0.2],
                        weight=(1.0 / self.n_particles)) for i in range(self.n_particles)]

        self.robot = Particle(0, 0, 0, std_dev=[0, 0, 0, 0.0, 0, 0])
        self.measurement = []  # Store measurement for visualization purposes

        self.world = [[1, 1], [2, 3], [-4, 2]]  # 2D array of landmark X, Y locations
        # self.world = [[2, 3]]  # 2D array of landmark X, Y locations

        self.measurement_mode = MeasurementMode.DISTANCE_FOV  # What sensor data the robot can recieve

        self.robot_position_history = []
        self.estimated_position_history = []
        self.localization_error_history = []

    def motion_step(self, vel, angular_vel, dt):
        for p in self.particles:
            p.move(vel, angular_vel, dt, noisy=True)

    def plot(self, plot):
        if not plot:
            return

        n_particles = len(self.particles)
        xs = [p.x for p in self.particles]
        ys = [p.y for p in self.particles]
        weights = [min(200, max(4, 30 * n_particles * p.weight)) for p in self.particles]

        x_est, y_est, theta_est = self.estimate_robot_position()

        plt.xlim(-5, 5)
        plt.ylim(-5, 5)

        plt.scatter(xs, ys, s=weights, alpha=0.2)  # Particles
        plt.scatter(self.robot.x, self.robot.y, s=25, alpha=1)  # Robot
        plt.scatter([mark[0] for mark in self.world], [mark[1] for mark in self.world])  # landmarks
        plt.scatter(x_est, y_est, s=16, alpha=1)  # Estimated robot

        # Draw sensor measurements
        for z in self.measurement:
            angle = z[1] + self.robot.theta
            x = self.robot.x + z[0] * np.cos(angle)
            y = self.robot.y + z[0] * np.sin(angle)
            plt.plot([self.robot.x, x], [self.robot.y, y], color='C0', alpha=0.5)

        plt.show()

    def measurement_step(self):
        self.measurement = self.robot.read_sensor(self.world, self.measurement_mode, noisy=True)

        total = 0
        for p in self.particles:
            z_particle = p.read_sensor(self.world, self.measurement_mode, noisy=False)
            p.weight = p.probability_sensor_hit(z_particle, self.measurement, self.measurement_mode)
            total += p.weight

        normalizer = 1.0 / total
        for p in self.particles:
            p.weight *= normalizer

    def resample(self):
        sampler = WeightedDistribution(self.particles)

        sample_type = "min_var"
        if sample_type == "random":
            self.particles = sampler.random_sample_particles(self.n_particles)
        elif sample_type == "min_var":
            self.particles = sampler.min_variance_sample(self.n_particles)
        else:
            print("ERROR: Unknown sample type " + sample_type)

    def record_history(self):
        x_est, y_est, theta_est = self.estimate_robot_position()
        distance = euclidean_distance(x_est, y_est, self.robot.x, self.robot.y)

        self.estimated_position_history.append([x_est, y_est, theta_est])
        self.robot_position_history.append([self.robot.x, self.robot.y, self.robot.theta])
        self.localization_error_history.append([distance])

    def save_history(self, folder):
        filename = folder + "/" + self.measurement_mode.name + ".txt"
        with open(filename, "a") as f:
            for robot, est, dist, in zip(self.robot_position_history,
                    self.estimated_position_history, self.localization_error_history):
                f.write("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n" %
                    (robot[0], robot[1], robot[2], est[0], est[1], est[2], dist[0]))
            f.write("\n")

    def estimate_robot_position(self):
        x_total = 0
        y_total = 0
        theta_total = 0
        weight_total = 0

        for p in self.particles:
            weight_total += p.weight
            x_total += p.x * p.weight
            y_total += p.y * p.weight
            theta_total += p.theta * p.weight

        if weight_total == 0:
            return 0, 0, 0

        x = x_total / weight_total
        y = y_total / weight_total
        theta = theta_total / weight_total

        return x, y, theta


if __name__ == "__main__":
    np.random.seed(0)
    create_folder("results")

    # Stop matplotlib for loop from running
    signal.signal(signal.SIGINT, signal_handler)

    robot_motions = np.array([[1, 0.5, 1],
                              [1, 0.5, 1],
                              [1, 0.5, 1],
                              [1, 0.5, 1],
                              [1, 0.5, 1],
                              [1, 0.5, 1],
                              [1, 0.5, 1],
                              [1, 0.5, 1],
                              [1, 0.5, 1]])

    # robot_motions = np.array([[0, 1, 1],
    #                           [0, 1, 1],
    #                           [0, -1, 1],
    #                           [0, -1, 1],
    #                           [1, 0.5, 1],
    #                           [1, 0.5, 1],
    #                           [1, 0.5, 1]])

    particle_filter = ParticleFilter()
    plot = True
    for move in robot_motions:
        particle_filter.plot(plot)

        particle_filter.robot.move(move[0], move[1], move[2], noisy=True)
        particle_filter.motion_step(move[0], move[1], move[2])

        particle_filter.measurement = []  # After motion the old measurement isn't valid
        print("After motion")
        particle_filter.plot(plot)

        particle_filter.measurement_step()
        
        print("Weights updated")
        particle_filter.plot(plot)

        particle_filter.record_history()

        particle_filter.resample()

        print("Resampled")

    particle_filter.save_history("results")
