#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from localization.localization.geoProject import Project


class UltraWideBandNode:
    def __init__(self, id, relative_x, relative_y, type, sensors):
        self.id = id
        self.x = None
        self.y = None
        self.relative_x = relative_x
        self.relative_y = relative_y
        self.type = type
        self.confidence = None
        self.sensors = sensors
        self.measurements = {}
        self.x_plot = []
        self.y_plot = []
        self.distance_plot = []

    def get_robot_position(self):
        sensor = self.sensors[self.sensors['id'] == self.id]
        return float(sensor['x']), float(sensor['y'])

    def add_measurement(self, anchor_id, distance, confidence):
        if distance >= 0:
            self.measurements[anchor_id] = distance
            if int(anchor_id) == 51:
                self.distance_plot.append(distance)
            self.confidence = confidence
        else:
            self.measurements[anchor_id] = np.nan  # set to invalid value to be ignored

    def plot_position(self, ax, moving_average=False):
        if moving_average:
            if len(self.x_plot) >= 3 and len(self.y_plot) >= 3:
                avg_x = self.moving_average(self.x_plot)
                avg_y = self.moving_average(self.y_plot)
                ax.scatter(avg_x, avg_y, label=str(self.id))
        else:
            ax.scatter(self.x_plot, self.y_plot, label=str(self.id))

    def get_position(self):
        if len(list(self.measurements.keys())) >= 3:
            P = Project(mode='2D', solver='LSE_GC')

            for i, sensor in self.sensors.iterrows():
                if sensor['type'] == 'anchor':
                    P.add_anchor(sensor['id'], (sensor['x'], sensor['y']))

            t,label=P.add_target()

            for sensor in self.measurements.keys():
                t.add_measure(sensor, self.measurements[sensor])

            P.solve()
            # Then the target location is:
            position = t.loc

            print('id: %d, X: %.2f, Y: %.2f' % (self.id, position.x, position.y))
            if position.x > 0 and position.y > 0:
                self.x = position.x
                self.y = position.y
                self.x_plot.append(position.x)
                self.y_plot.append(position.y)
        else:
            print('Not enough points to triangulate node...')

    @staticmethod
    def moving_average(a, n=3):
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n
