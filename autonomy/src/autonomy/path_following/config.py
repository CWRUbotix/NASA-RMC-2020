import numpy as np
from autonomy.path_following.fuzzy_logic import FuzzyLogic, FuzzySet

s_set = [[1e99, -0.4, 0.2], [0.2, -0.2, 0.2], [0.2, 0, 0.2], [0.2, 0.2, 0.2], [0.2, 0.4, 1e99]]
s_dot_set = s_set
curvature_set = [[1e99, 0, 0.75], [0.75, 0.75, 0.75], [0.75, 1.5, 1e99]]

NL, NM, NS, ZO, PS, PM, PL = -1, -0.66, -0.33, 0, 0.33, 0.66, 1
rules = np.array([[NL, NL, NM, NS, ZO],
                  [NL, NM, NS, ZO, PS],
                  [NM, NS, ZO, PS, PM],
                  [NS, ZO, PS, PM, PL],
                  [ZO, PS, PM, PL, PL]])
curvature_rules = np.array([1, 0.7, 0.6])

s_set = FuzzySet(s_set)
s_dot_set = FuzzySet(s_dot_set)
curvature_set = FuzzySet(curvature_set)

sliding_controller = FuzzyLogic(rules, s_set, s_dot_set)
slowdown_controller = FuzzyLogic(curvature_rules, curvature_set)

target_velocity = 0.15

lambda_e = 0.85
G_s = 0.4
G_s_dot = 0.02
G_u = 3

turn_speed = 0.9
