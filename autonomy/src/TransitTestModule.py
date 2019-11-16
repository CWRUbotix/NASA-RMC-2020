
import sys
# import rospy
from PathPlanningUtils import Obstacle, Position
from InteractivePathTester import InteractivePathTester

ROBOT_WIDTH = 0.75
ROBOT_LENGTH = 1.5
ARENA_WIDTH = 3.78
ARENA_HEIGHT = 7.38


def rosTest():
    pass


def softwareTest():
    goal = Position(0.5, 6.5)

    obstacles = [Obstacle(3, 2.0, 0.35), Obstacle(0.5, 5, 0.35), Obstacle(0.5, 1.5, 0.35)]

    tester = InteractivePathTester(goal, 3.78, 7.38, obstacles)
    tester.update_path_and_draw()


def main():
    if sys.argv[1] == '0':
        rosTest()
    elif sys.argv[1] == '1':
        softwareTest()
    else:
        print('unsupported')
        exit(0)


if __name__ == '__main__':
    main()
