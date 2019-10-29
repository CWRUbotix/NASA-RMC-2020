
import sys
import math
#import rospy
from PathPlanning.PathPlanning import Grid, Obstacle, Position
from PathPlanning.ThetaStar import create_path, checkBlocked
from PathPlanning.InteractivePathTester import InteractivePathTester

#from TestModule import turn_algo_2, conservative_drive, testShutdown

ROBOT_WIDTH = 0.75
ROBOT_LENGTH = 1.5
ARENA_WIDTH = 3.78
ARENA_HEIGHT = 7.38


def rosTest():
    pass


def softwareTest():
    p1 = Position(0, 0)
    p2 = Position(0.5, 6.5)
    grid = Grid(p1, p2, 3.78, 7.38)

    obstacles = [Obstacle(3, 2.0, 0.35), Obstacle(0.5, 5, 0.35), Obstacle(0.5, 1.5, 0.35)]

    tester = InteractivePathTester(p1, p2, 3.78, 7.38, obstacles)
    tester.update_path_and_draw()


def toDegree(rad):
    return rad * 180 / math.pi


def toRadian(deg):
    return deg * math.pi / 180


def convertToCommands(path, currentPos):
    commands = []
    for position in path.path:
        angle_to_face = currentPos.angleToFace(position)
        print(toDegree(angle_to_face))
        pos = Position(currentPos.getX(), currentPos.getY(), angle_to_face)
        angle_turn = currentPos.angleTurnTo(pos)
        distance = currentPos.distanceTo(position)
        commands.append((toDegree(angle_turn), distance))
        currentPos = Position(pos.getX() + distance * math.cos(angle_to_face), pos.getY() + distance * math.sin(angle_to_face), angle_to_face)
        currentPos.orientation = angle_to_face
    return commands


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
