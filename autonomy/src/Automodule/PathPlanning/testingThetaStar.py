from PathTesting import drawPath
from ThetaStar import thetaStar
from PathPlanning import Position, Vertex, Grid, Obstacle
import random

startpos = Position(0, 0, 0,)
endpos = Position(400, 400, 0)
start = Vertex(0, 0, 565, [], startpos)
start.setParent(start)
end = Vertex(None, 565, 0, [], endpos)


obstacles = []
positions = []
vertices=[[None]*10 for y in range(10)]
for i in range(10):
    for j in range(10):
        if (i == 4 and j == 4):
            vertices[i][j] = end
        elif(i==0 and j==0):
            vertices[i][j] = start
        else:
            point = Vertex(None, (((i ** 2) + (j ** 2)) ** 1 / 2) * 100, (((4 - i) ** 2 + (4 - j) ** 2) ** 1 / 2) * 100, [], Position(j * 100, i * 100, 0))
            vertices[i][j] = point
for i in range(10):
    for j in range(10):
        elem = vertices[i][j]
        if (i + 1 <= 9):
            elem.addNeighbor(vertices[i + 1][j])
        if (j + 1 <= 9):
                #vertices[i][j].addNeighbor(vertices[i + 1][j + 1])
            elem.addNeighbor(vertices[i][j + 1])
        if (j - 1 >= 0):
                #vertices[i][j].addNeighbor(vertices[i + 1][j - 1])
            elem.addNeighbor(vertices[i][j - 1])
        if (i - 1 >= 0):
            elem.addNeighbor(vertices[i - 1][j])
            #if (j + 1 <= 9):
                #vertices[i][j].addNeighbor(vertices[i - 1][j + 1])
             #   None
            #if (j - 1 >= 0):
                #vertices[i][j].addNeighbor(vertices[i - 1][j - 1])
             #   None
        vertices[i][j]=elem
        positions.append(Position(j * 100, i * 100, 0))

#vertices[0][0].setParent(start)
for i in range(15):
    r1 = random.randint(150, 600)
    r2 = random.randint(150, 600)
    r3 = random.randint(5, 50)
    obstacles.append(Obstacle(r1, r2, r3))
obstacles.append(Obstacle(200, 200, 5))
path = thetaStar(start, end, obstacles, 1)
path.printPath()
drawPath(path, obstacles, positions)