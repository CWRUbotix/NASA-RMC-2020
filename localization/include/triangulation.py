#!/usr/bin/env python3
import pandas as pd
import localization as lx

class UltraWideBandNode:
    def __init__(self):
        self.x = None
        self.y = None
        self.sensors =
        self.measurements = {}

    def add_measurement(anchor_id, distance):
        self.measurements[anchor_id] = distance

    def get_position():
        P=lx.Project(mode='2D',solver='LSE')

        for i, sensor in self.sensors.iterrows():
            if sensor['type'] == 'anchor':
                P.add_anchor(sensor['id'], (sensor['x'], sensor['y']))

        t,label=P.add_target()

        for sensor in self.measurements.keys():
            t.add_measure(sensor, measurements[sensor])

        P.solve()
        # Then the target location is:
        print(t.loc)
