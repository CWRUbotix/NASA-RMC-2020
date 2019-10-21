import numpy as np
import matplotlib.pyplot as plt


class FuzzySet:
    def __init__(self, centers, left_widths, right_widths):
        self.centers = centers
        self.left_widths = left_widths
        self.right_widths = right_widths

    def get_membership(self, x):
        memberships = []
        for center, left, right in zip(self.centers, self.left_widths, self.right_widths):
            if x > center:
                membership = max((center + right - x) / right, 0)
            else:
                membership = max((x - center + left) / left, 0)

            memberships.append(membership)

        return np.array(memberships)

    def draw(self):
        members = []
        for center, left, right in zip(self.centers, self.left_widths, self.right_widths):
            members.append([[center - left, 0], [center, 1], [center + right, 0]])

        return np.array(members)


class FuzzyLogic:
    def __init__(self, rules, *sets):
        self.set1 = sets[0]

        if len(sets) == 2:
            self.set2 = sets[1]
        else:
            self.set2 = None

        self.rules = rules

        if len(sets) != len(rules.shape):
            raise Exception("Wrong number of sets for rule base")

    def crisp_output(self, *xs):
        x1 = xs[0]
        y1 = self.set1.get_membership(x1)

        if self.set2:
            x2 = xs[1]
            y2 = self.set2.get_membership(x2)

            crisp_output = np.dot(np.dot(y1, self.rules), y2.T)

            return crisp_output
        else:
            crisp_output = np.sum(np.dot(y1, self.rules))

            return crisp_output

if __name__ == "__main__":
    error_centers = np.array([-0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6])
    error_left_widths = np.array([1000, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
    error_right_widths = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1000])

    error_dot_centers = np.array([-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3])
    error_dot_left_widths = np.array([1000, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    error_dot_right_widths = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1000])

    error_set = FuzzySet(error_centers, error_left_widths, error_right_widths)
    error_dot_set = FuzzySet(error_dot_centers, error_dot_left_widths, error_dot_right_widths)
    members = error_set.draw()

    NB, NM, NS, ZO, PS, PM, PB = -1.5 * np.pi, -1 * np.pi, -0.5 * np.pi, 0, 0.5 * np.pi, 1 * np.pi, 1.5 * np.pi
    rules = np.array([[NB, NB, NM, NB, NM, NS, ZO],
                      [NB, NB, NM, NM, NS, ZO, PS],
                      [NB, NM, NS, NS, ZO, PS, PM],
                      [NB, NM, NS, ZO, PS, PM, PB],
                      [NM, NS, ZO, PS, PS, PM, PB],
                      [NS, ZO, PS, PM, PM, PB, PB],
                      [ZO, PS, PM, PB, PM, PB, PB]])

    fuzzy_controller = FuzzyLogic(rules, error_set, error_dot_set)
    fuzzy_controller_2 = FuzzyLogic(rules[3], error_set)

    print(fuzzy_controller.crisp_output(0, 0))
    print(fuzzy_controller_2.crisp_output(0))

    # x = np.arange(-1, 1, 0.1)
    # y = np.array([-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3])
    #
    # for value in y:
    #     plt.plot(x, [fuzzy_controller.crisp_output(x1, value) for x1 in x])
    # plt.show()
