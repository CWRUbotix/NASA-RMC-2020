import math

try:
    from nav_msgs.msg import OccupancyGrid
except:
    pass

# Global Variables
ERROR_BOUND = 0.05
CLEARANCE = 0.3
GRID_SIZE = 0.15


# angle constrained to [-pi, pi]
def constrain_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def distanceTo(self, p):
        x = p.getX() - self.getX()
        y = p.getY() - self.getY()

        return (x ** 2 + y ** 2) ** .5

    def __str__(self):
        return 'x: ' + '%.3f' % self.getX() + ' y :' + '%.3f' % self.getY()


class Vertex(Position):
    def __init__(self, x, y, row, col, prob_blocked=0):
        super().__init__(x, y)
        self.parent = None
        self.dist = float('inf')
        self.heuristic = float('inf')
        self.row = row
        self.col = col
        self.prob_blocked = prob_blocked

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

    def angleToFace(self, p):
        angle = math.atan2(p.getY() - self.getY(), p.getX() - self.getX())
        angle = constrain_angle(angle)  # Constrain to [-180, 180]
        return angle

    def getPos(self):
        return [self.x, self.y]

    def getParent(self):
        return self.parent

    def setParent(self, parent):
        self.parent = parent

    def getDistance(self):
        return self.dist

    def setDistance(self, dist):
        self.dist = dist

    def getHeuristic(self):
        return self.heuristic

    def setHeuristic(self, h):
        self.heuristic = h

    def get_indices(self):
        return self.row, self.col

    def get_blocked(self):
        return self.prob_blocked >= 0.5

    def set_prob_blocked(self, prob_blocked):
        self.prob_blocked = prob_blocked

    def get_prob_blocked(self):
        return self.prob_blocked


class Grid(object):
    def __init__(self, width, height, grid_width=GRID_SIZE, occupancies=None):
        if occupancies:
            self.num_cols = occupancies.info.width
            self.num_rows = occupancies.info.height
            self.unit_width = occupancies.info.resolution
            self.unit_height = occupancies.info.resolution
            self.width = self.num_cols * self.unit_width
            self.height = self.num_rows * self.unit_height
        else:
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
                prob, x, y = 0, 0, 0
                if occupancies:
                    x = occupancies.info.origin.position.x
                    y = occupancies.info.origin.position.y
                    prob = occupancies.data[i * self.num_cols + j] / 100
                vertex = Vertex(x + (j + 0.5) * grid_width, y + (i + 0.5) * grid_width, i, j, prob_blocked=prob)
                row.append(vertex)
            self.vertices.append(row)

    def getVertex(self, row_index, col_index):
        return self.vertices[row_index][col_index]

    def getNeighbors(self, row_index, col_index):
        neighbors = []
        x_coords = [col_index - 1, col_index, col_index + 1]
        y_coords = [row_index - 1, row_index, row_index + 1]

        for r in y_coords:
            for c in x_coords:
                if not self.out_of_bounds(r, c) and not (r == row_index and c == col_index) and not \
                   self.is_probably_blocked(r, c):
                    neighbors.append(self.getVertex(r, c))
        return neighbors

    def out_of_bounds(self, row_index, col_index):
        if row_index < 0 or row_index > self.num_rows-1:
            return True
        if col_index < 0 or col_index > self.num_cols-1:
            return True

        return False

    def is_probably_blocked(self, row_index, col_index):
        return self.vertices[row_index][col_index].get_blocked()

    def get_prob_blocked(self, row_index, col_index):
        return self.vertices[row_index][col_index].get_prob_blocked()

    def addObstacle(self, obs):
        extra_space = 0.5
        o1 = self.getGridIndices(obs.getCenter()[0] - obs.getRadius() - CLEARANCE - extra_space, obs.getCenter()[1] - obs.getRadius() - CLEARANCE - extra_space)
        o2 = self.getGridIndices(obs.getCenter()[0] + obs.getRadius() + CLEARANCE + extra_space, obs.getCenter()[1] + obs.getRadius() + CLEARANCE + extra_space)
        for i in range(o1[0], o2[0]+1):
            for j in range(o1[1], o2[1]+1):
                centerx = self.getGridIndices(obs.getCenter()[0], obs.getCenter()[1])[0]
                centery = self.getGridIndices(obs.getCenter()[0], obs.getCenter()[1])[1]
                dist_sq = (i-centerx)**2 + (j-centery)**2
                radius_sq = ((obs.getRadius())/self.unit_width) ** 2
                if dist_sq <= radius_sq + (CLEARANCE/self.unit_width)**2:
                    self.vertices[i][j].set_prob_blocked(1)
                else:
                    guassian = 50 * 1/(2 * math.pi * radius_sq) * math.exp(-0.2 * dist_sq / radius_sq)
                    self.vertices[i][j].set_prob_blocked(max(guassian, self.vertices[i][j].get_prob_blocked()))

    def getGridIndices(self, x_pos, y_pos):
        col_index = min(self.num_cols - 1, max(int(x_pos / self.unit_width), 0))
        row_index = min(self.num_rows - 1, max(int(y_pos / self.unit_height), 0))

        return int(row_index), int(col_index)

    def __str__(self):
        string = ""
        for i in range(self.num_rows):
            for j in range(self.num_cols):
                string += str(int(self.get_prob_blocked(i, j) * 100)) + " "
            string += '\n'

        return string


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


# acts as a sequence of positions
class Path:
    def printPath(self):
        for position in self.path:
            print(("X: %s" %(position.getX_pos())))
            print(("Y: %s" %(position.getY_pos())))

    def get_angles(self):
        angles = []

        last_direction = self.path[0].angleToFace(self.path[1])
        for i in range(1, len(self)-1):
            direction = self.path[i].angleToFace(self.path[i+1])
            angle = constrain_angle(direction - last_direction)
            angles.append(angle)
            last_direction = direction

        return angles
