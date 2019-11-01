import numpy as np
from PathFollowing.FuzzyLogic import FuzzyLogic, FuzzySet

error_set = [[1e99, -0.6, 0.2], [0.2, -0.4, 0.2], [0.2, -0.2, 0.2],
             [0.2, 0, 0.2], [0.2, 0.2, 0.2], [0.2, 0.4, 0.2], [0.2, 0.6, 1e99]]

error_dot_set = [[1e99, -0.3, 0.1], [0.1, -0.2, 0.1], [0.1, -0.1, 0.1],
                 [0.1, 0, 0.1], [0.1, 0.1, 0.1], [0.1, 0.2, 0.1], [0.1, 0.3, 1e99]]

curvature_set = [[1e99, 0, 5], [5, 5, 5], [5, 10, 5], [5, 15, 5], [5, 20, 1e99]]

NB, NM, NS, ZO, PS, PM, PB = -1.5, -1, -0.5, 0, 0.5, 1, 1.5
rules = np.array([[NB, NB, NM, NB, NM, NS, ZO],
                  [NB, NB, NM, NM, NS, ZO, PS],
                  [NB, NM, NS, NS, ZO, PS, PM],
                  [NB, NM, NS, ZO, PS, PM, PB],
                  [NM, NS, ZO, PS, PS, PM, PB],
                  [NS, ZO, PS, PM, PM, PB, PB],
                  [ZO, PS, PM, PB, PM, PB, PB]])

curvature_rules = np.array([1, 1, 1, 1, 1])

error_set = FuzzySet(error_set)
error_dot_set = FuzzySet(error_dot_set)
curvature_set = FuzzySet(curvature_set)

fuzzy_controller = FuzzyLogic(rules, error_set, error_dot_set)
slowdown_controller = FuzzyLogic(curvature_rules, curvature_set)

forward_torque = 30
