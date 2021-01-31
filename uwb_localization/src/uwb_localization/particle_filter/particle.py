#!/usr/bin/env python3
import numpy as np
from enum import Enum


def euclidean_distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))


class MeasurementMode(Enum):
    DISTANCE = 1  # Distance to any landmark
    DISTANCE_FOV = 2  # Distance to any landmark in the field of view
    DISTANCE_ANGLE = 3  # Distance and angle to any landmark
    DISTANCE_ANGLE_FOV = 4  # Distance and angle to any landmark in the field of view


class Particle:
    def __init__(self, x, y, theta, randomize=False, std_dev=None, weight=1.0):
        """
        std_dev: array of length 6, std deviations of noise for 
            x, y
            theta
            linear_vel
            angular_vel
            sensor_distance
            sensor_angle
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.std_dev = std_dev

        self.weight = weight

        # Initialize particles randomly
        if randomize:
            self.x = np.random.uniform(-self.std_dev[0], self.std_dev[0])
            self.y = np.random.uniform(-self.std_dev[1], self.std_dev[1])
            self.theta = np.random.uniform(0, 2 * np.pi)

    def copy(self):
        # Do not randomize particle location when copying
        p = Particle(self.x, self.y, self.theta,
                randomize=False, std_dev=self.std_dev, weight=self.weight)

        return p

    def move(self, vel, angular_vel, dt, noisy=False):
        if noisy:
            vel = self.add_noise(vel, self.std_dev[2])
            angular_vel = self.add_noise(angular_vel, self.std_dev[3])

        dtheta = angular_vel * dt

        if abs(dtheta) < 0.0001:
            dx = vel * dt
            dy = 0
        else:
            # theta * r = distance around circle
            radius = vel * dt / dtheta
            dx = radius * np.sin(dtheta)
            dy = radius * (1 - np.cos(dtheta))

        # rotate translation in base_frame to global_frame
        self.x += dx * np.cos(self.theta) - dy * np.sin(self.theta)
        self.y += dx * np.sin(self.theta) + dy * np.cos(self.theta)
        self.theta += dtheta

    def add_noise(self, x, std_dev):
        return x + np.random.normal(0, std_dev)

    def read_sensor(self, world, measurement_mode, noisy=False, fov=1.57):
        """
        Find sensor measurement given particle location
        Will be compared to robot's noisy measurement

        world: 2D array of X, Y locations of landmarks
        """

        readings = []

        for i in range(len(world)):
            # Always need distance angle to identify landmark,
            # some modes will ignore certain measurements though
            distance = euclidean_distance(self.x, self.y, world[i][0], world[i][1])
            angle = np.arctan2(world[i][1] - self.y, world[i][0] - self.x)
            angle = self.wrap_angle(angle - self.theta)  # Convert to relative to robot

            if measurement_mode == MeasurementMode.DISTANCE:
                # Measure distance to the landmarks, add noise if simulating real sensor
                if noisy:
                    distance = self.add_noise(distance, self.std_dev[4])

                readings.append([distance, angle])

            elif measurement_mode == MeasurementMode.DISTANCE_FOV:
                # Measure distance to landmarks, but only ones inside the FOV
                if abs(angle) < 0.5 * fov:
                    if noisy:  # Simulate sensor noise
                        distance = self.add_noise(distance, self.std_dev[4])
                        angle = self.add_noise(angle, self.std_dev[5])

                    readings.append([distance, angle])

            elif measurement_mode == MeasurementMode.DISTANCE_ANGLE:
                # Measure distance and angle to landmarks
                if noisy:  # Simulate sensor noise
                    distance = self.add_noise(distance, self.std_dev[4])
                    angle = self.add_noise(angle, self.std_dev[5])

                readings.append([distance, angle])

            elif measurement_mode == MeasurementMode.DISTANCE_ANGLE_FOV:
                # Measure distance and angle to landmarks, only inside FOV
                if abs(angle) < 0.5 * fov:
                    if noisy:  # Simulate sensor noise
                        distance = self.add_noise(distance, self.std_dev[4])
                        angle = self.add_noise(angle, self.std_dev[5])

                    readings.append([distance, angle])

            else:
                print("ERROR: Unknown measurement mode " + str(measurement_mode))

        return readings

    def probability_sensor_hit(self, z_particle, z_robot, measurement_mode):
        """z and z_actual: 2D arrays of measured and perfect landmark measurements"""

        if measurement_mode == MeasurementMode.DISTANCE:
            # Sum distances sensor distance errors to all landmarks
            total_dist_err = 0
            for i in range(len(z_particle)):
                total_dist_err += abs(z_particle[i][0] - z_robot[i][0])

            # Normal distribution is probability
            return self.normal_dist(total_dist_err, self.std_dev[4])

        elif measurement_mode == MeasurementMode.DISTANCE_FOV:
            # Note: only landmarks within fov are seen
            probability = 1

            # Save how many measurements we got because
            # we will be removing from this list later
            num_z_particle = len(z_particle)
            num_tp = 0  # Number of true positive matches

            # Find true postive landmark matches
            for z_r in z_robot:
                for z_p in z_particle:
                    # A match is found if distance less than 3 std_dev
                    dist_err = abs(z_r[0] - z_p[0])
                    if dist_err < 3 * self.std_dev[4]:
                        num_tp += 1
                        z_particle.remove(z_p)  # Remove this measurement from the list
                        probability *= self.normal_dist(dist_err, self.std_dev[4])
            
            if num_z_particle > num_tp:
                # This means we have false positives
                probability *= 0.0
            elif len(z_robot) > num_tp:
                # This means there are false negatives
                probability *= 0.0

            # probability += 0.05  # Account for general errors in measurement

            return probability

        elif measurement_mode == MeasurementMode.DISTANCE_ANGLE:
            # Sum distances sensor distance errors to all landmarks
            total_dist_err = 0
            total_angle_err = 0
            for i in range(len(z_particle)):
                total_dist_err += abs(z_particle[i][0] - z_robot[i][0])
                total_angle_err += abs(self.wrap_angle(z_particle[i][1] - z_robot[i][1]))

            # Normal distribution is probability
            return self.normal_dist(total_dist_err, self.std_dev[4]) * self.normal_dist(total_angle_err, self.std_dev[5])

        elif measurement_mode == MeasurementMode.DISTANCE_ANGLE_FOV:
            # Note: only landmarks within fov are seen
            probability = 1

            # Save how many measurements we got because
            # we will be removing from this list later
            num_z_particle = len(z_particle)
            num_tp = 0  # Number of true positive matches

            # Find true postive landmark matches
            for z_r in z_robot:
                for z_p in z_particle:
                    # A match is found if distance err less than 3 std_dev
                    dist_err = abs(z_r[0] - z_p[0])
                    angle_err = abs(self.wrap_angle(z_r[1] - z_p[1]))
                    if dist_err < 3 * self.std_dev[4] and angle_err < 3 * self.std_dev[5]:
                        num_tp += 1
                        z_particle.remove(z_p)  # Remove this measurement from the list
                        probability *= self.normal_dist(dist_err, self.std_dev[4]) * \
                            self.normal_dist(angle_err, self.std_dev[5])
            
            if num_z_particle > num_tp:
                # This means we have false positives
                probability *= 0.0
            elif len(z_robot) > num_tp:
                # This means there are false negatives
                probability *= 0.0

            # probability += 0.05  # Account for general errors in measurement

            return probability

        else:
            print("ERROR: Unknown measurement mode " + str(measurement_mode))

    def wrap_angle(self, x):
        """Convert angles to be within -pi to pi"""
        return (x + 3.1416) % 6.283 - 3.1416

    def normal_dist(self, x, std):
        """Normal distribution pdf"""
        return 1 / (np.sqrt(2 * np.pi) * std) * \
                np.exp(-0.5 * (x / std) ** 2) 


if __name__ == "__main__":
    np.random.seed(0)

    particle = Particle(0, 0, 0, randomize=True, std_dev=[0, 0, 0, 0, 0, 0])

    for i in range(10):
        particle.move(2, 0.1, 0.5)
        print("%.3f, %.3f, %.3f" % (particle.x, particle.y, particle.theta))
