import math
import collections
from collections import deque
import sys

# Global Variables
ERROR_BOUND = 0.05
CLEARANCE = 0.0
GRID_SIZE = 0.15


# angle constrained to [-pi, pi]
def constrain_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Position(object):
    def __init__(self, x_pos, y_pos, orientation=0.0):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.orientation = orientation

    def getX(self):
        return self.x_pos

    def getY(self):
        return self.y_pos

    def getOrientation(self):
        return self.orientation

    def setOrientation(self, o):
        self.orientation = o

    def __str__(self):
        return " x:" + str(self.x_pos) + " y:" + str(self.y_pos) + " angle:" + str(self.orientation)

    def __eq__(self, p):
        if math.fabs(self.getX() - p.getX()) < ERROR_BOUND and math.fabs(self.getY() - p.getY()) < ERROR_BOUND:
            return True
        return False

    def distanceTo(self, p):
        x = abs(p.getX() - self.x_pos)
        y = abs(p.getY() - self.y_pos)

        dist = (x ** 2 + y ** 2) ** .5
        if dist < GRID_SIZE:  # TODO: Changing this value drastically affects performance because doesn't penalize distance
            return 0
        else:
            return dist

    def angleTurnTo(self, p):
        change = p.getOrientation() - self.orientation
        change = constrain_angle(change)  # Constrain to [-180, 180]

        if math.fabs(change) < math.pi / 36:  # TODO If less than 5 degrees return 0 for some reason
            return 0
        else:
            return change

    def angleToFace(self, p):
        # print(self)
        # print(p)
        angle = math.atan2(p.getY() - self.getY(), p.getX() - self.getX())

        angle = constrain_angle(angle) # Constrain to [-180, 180]
        return angle


class Grid(object):
    def __init__(self, p1, p2, width, height, grid_width=GRID_SIZE):
        self.p1 = p1
        self.p2 = p2
        self.width = width
        self.height = height
        self.num_cols = int(math.floor(width / grid_width))
        self.num_rows = int(math.floor(height / grid_width))
        self.unit_width = grid_width
        self.unit_height = grid_width
        self.vertices = []

        for i in range(self.num_rows):
            row = []
            for j in range(self.num_cols):
                row.append(Vertex(j * grid_width + grid_width/2, i * grid_width + grid_width/2,
                                  i, j))
            self.vertices.append(row)

    def getVertex(self, row_index, col_index):
        return self.vertices[row_index][col_index]

    def getNeighbors(self, row_index, col_index):
        neighbors = []
        x_coords = [col_index - 1, col_index, col_index + 1]
        y_coords = [row_index - 1, row_index, row_index + 1]

        for r in y_coords:
            for c in x_coords:
                if not self.blocked(r, c) and not (r == row_index and c == col_index):
                    neighbors.append(self.getVertex(r, c))
        return neighbors

    def blocked(self, row_index, col_index):
        if row_index < 0 or row_index > self.num_rows-1:
            return True
        if col_index < 0 or col_index > self.num_cols-1:
            return True

        return self.vertices[int(row_index)][int(col_index)].get_blocked()

    def addObstacle(self, obs):
        o1 = self.getGridIndices(obs.getCenter()[0] - obs.getRadius() - CLEARANCE, obs.getCenter()[1] - obs.getRadius() - CLEARANCE)
        o2 = self.getGridIndices(obs.getCenter()[0] + obs.getRadius() + CLEARANCE, obs.getCenter()[1] + obs.getRadius() + CLEARANCE)
        # print(str(o1), str(o2))
        for i in range(int(o1[0]), int(o2[0])+1):
            for j in range(int(o1[1]), int(o2[1])+1):
                self.vertices[i][j].set_blocked(True)

    def getGridIndices(self, x_pos, y_pos):
        col_index = min(self.num_cols - 1, max(int(x_pos / self.unit_width), 0))
        row_index = min(self.num_rows - 1, max(int(y_pos / self.unit_height), 0))

        return int(row_index), int(col_index)


class Obstacle:
    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius

    def getCenter(self):
        return [self.center_x, self.center_y]

    def getRadius(self):
        return self.radius

    def setRadius(self, radius):
        self.radius = radius

    def setCenter(self, center_x, center_y):
        self.center_x = center_x
        self.center_y = center_y

    def mergeIfEqual(self, other, maximum_overlap):
        distance_x = self.center_x - other.center_x
        distance_y = self.center_y - other.center_y
        distance = math.sqrt(math.pow(distance_x, 2) + math.pow(distance_y, 2))
        sum_radius = self.radius + other.radius
        if (sum_radius - distance) >= maximum_overlap * distance:
            self.center_x = (self.center_x + other.center_x) / 2
            self.center_y = (self.center_y + other.center_y) / 2
            self.radius = distance / 2
            return True
        return False


# acts as a sequence of instances of the Position class
class Path(collections.Sequence):
    def __init__(self, positions):
        self.path = deque(positions)

    def insert(self, newPositions):
        self.path.append(newPositions)

    def getPosition(self):
        return self.path.popleft()

    def __getitem__(self, item):
        return item

    def __len__(self):
        return len(self.path)

    def delete(self, position):
        return self.path.remove(position)

    def printPath(self):
        for position in self.path:
            print(("X: %s" %(position.getX_pos())))
            print(("Y: %s" %(position.getY_pos())))
            print(("Orientation: %s\n" % (position.getOrientation())))

    def get_angles(self):
        angles = []

        last_direction = self.path[0].angleToFace(self.path[1])
        for i in range(1, len(self)-1):
            direction = self.path[i].angleToFace(self.path[i+1])
            angle = constrain_angle(direction - last_direction)
            angles.append(angle)
            last_direction = direction

        return angles


class Vertex(Position):
    def __init__(self, x_pos, y_pos, row, col):
        super(Vertex, self).__init__(x_pos, y_pos, 0)
        self.parent = None
        self.dist = float('inf')
        self.heuristic = float('inf')
        self.row = row
        self.col = col
        self.blocked = False

    def __lt__(self, other):
        return self.dist + self.heuristic < other.getDistance() + other.getHeuristic()

    def __gt__(self, other):
        return self.dist + self.heuristic > other.getDistance() + other.getHeuristic()

    def __le__(self, other):
        return self.dist + self.heuristic <= other.getDistance() + other.getHeuristic()

    def __ge__(self, other):
        return self.dist + self.heuristic >= other.getDistance() + other.getHeuristic()

    def __eq__(self, other):
        return self.getX() == other.getX() and self.getY() == other.getY()

    def __str__(self):
        return 'x: ' + '%.4f' % self.getX() + ' y :' + '%.4f' % self.getY()

    def getParent(self):
        return self.parent

    def setParent(self, parent):
        self.parent = parent

    def getDistance(self):
        return self.dist

    def setDistance(self, dist, reset=False):
        if reset:
            self.dist = float('inf')
        else:
            self.dist = dist

    def getHeuristic(self):
        return self.heuristic

    def setHeuristic(self, dest, reset=False):
        if reset:
            self.heuristic = math.inf
        else:
            self.heuristic = dest

    def get_indices(self):
        return self.row, self.col

    def get_blocked(self):
        return self.blocked

    def set_blocked(self, blocked):
        self.blocked = blocked


