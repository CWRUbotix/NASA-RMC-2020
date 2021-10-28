#!/usr/bin/env python3

import localization.geoProject as Project
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt


class UltraWideBandNode:

    def __init__(self, id, relative_x, relative_y, anchors):
        # list of anchors, contains  the id number, x, and y
        self.anchors = anchors
        # map of measurements from each anchor
        self.measurements = {}

        self.relative_x = relative_x
        self.relative_y = relative_y

        self.x = None
        self.y = None
        self.condidence = None

        self.id = id

    def is_valid(self) -> bool:
        return self.x is not None and self.y is not None


    def get_position(self):
        if len(self.measurements.keys()) < 3:
            pass
        else:
            P = Project(mode="2D", solver="LSE")
            for id, x, y in self.anchors:
                P.add_anchor(id, (x, y))

            t, lbl = P.add_target()

            for anchor_id in self.measurements.keys():
                t.add_measurement(anchor_id, self.measurements[anchor_id])

            P.solve()

            self.x = t.loc[0]
            self.y = t.loc[1]

            return t.loc

    def add_measurement(self, anchor_id, distance, condidence):
        if distance > 0:
            self.measurements[anchor_id] = distance
            self.confidence = condidence
        else:
            self.measurements[anchor_id] = np.nan
            self.confidence = np.nan
