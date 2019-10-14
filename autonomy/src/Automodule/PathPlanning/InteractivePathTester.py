from graphics import *
import collections
from collections import deque
from PathPlanning.ThetaStar import create_path
from PathPlanning.PathPlanning import Position


class InteractivePathTester:
    SCALE = 75  # px/meter

    def __init__(self, start, end, arena_width, arena_height, obstacles):
        self.positions = [start, end]
        self.arena_width = arena_width
        self.arena_height = arena_height
        self.obstacles = obstacles

        self.win = GraphWin("Path", self.arena_width*self.SCALE, self.arena_height*self.SCALE, autoflush=False)

        self.win.bind("<ButtonPress-1>", self.on_click)
        self.win.bind("<B1-Motion>", self.update_drag_position)
        self.win.bind("<ButtonRelease-1>", self.update_drag_position)

        self.object_id = -1

        self.update_image()
        update(1000)
        self.win.getMouse()

    def on_click(self, event):
        self.object_id = -1

        x, y = event.widget.winfo_pointerxy()
        x, y = (x-self.win.winfo_rootx()) / self.SCALE, (y-self.win.winfo_rooty()) / self.SCALE
        x, y = x, self.arena_height - y

        for i, obstacle in enumerate(self.obstacles):
            if Position(x, y).distanceTo(Position(obstacle.center_x, obstacle.center_y)) < obstacle.getRadius():
                self.object_id = i
                break
        for i, position in enumerate(self.positions):
            if Position(x, y).distanceTo(position) < 0.1:
                self.object_id = i + len(self.obstacles)
                break

    def update_drag_position(self, event):
        x, y = event.widget.winfo_pointerxy()
        x, y = (x - self.win.winfo_rootx()) / self.SCALE, (y - self.win.winfo_rooty()) / self.SCALE
        x, y = x, self.arena_height - y

        if 0 <= self.object_id < len(self.obstacles):
            self.obstacles[self.object_id].setCenter(x, y)
            self.update_image()
        elif self.object_id > 0:
            self.positions[self.object_id - len(self.obstacles)] = Position(x, y)
            self.update_image()

    def update_image(self):
        # t = time.time_ns()
        path, grid = create_path(self.positions[0], self.positions[1], self.arena_width, self.arena_height, self.obstacles)
        # print((time.time_ns()-t)*10**-9)
        self.drawPath(self.win, path, self.obstacles, self.positions, grid)

    def drawPath(self, win, path, obstacles, positions, grid):
        for item in win.items[:]:
            item.undraw()
        lastX = 0
        lastY = 0
        lines = deque()
        points = deque()
        for i in range(len(path)):
            p = path.getPosition()
            if lastX != 0 or lastY != 0:
                l = Line(Point(lastX*self.SCALE, (self.arena_height-lastY)*self.SCALE),
                         Point(p.getX()*self.SCALE, (self.arena_height-p.getY())*self.SCALE))
                lines.append(l)
            c = Circle(Point(p.getX()*self.SCALE, (self.arena_height-p.getY())*self.SCALE), 0.06*self.SCALE)
            c.setFill("blue")
            c.setOutline("blue")
            points.append(c)
            lastX = p.getX()
            lastY = p.getY()
        for obstacle in obstacles:
            c = Circle(Point(obstacle.center_x*self.SCALE, (self.arena_height-obstacle.center_y)*self.SCALE), obstacle.getRadius()*self.SCALE)
            c.setFill("red")
            c.setOutline("red")
            points.append(c)
        for x in lines:
            x.draw(win)
        for x in points:
            x.draw(win)
        for r in range(grid.num_rows):
            for c in range(grid.num_cols):
                if grid.blocked(r, c):
                    ci = Circle(Point(((c+0.5) * grid.unit_width) * self.SCALE,
                                      (self.arena_height-(r+0.5) * grid.unit_height) * self.SCALE), 0.05 * self.SCALE)
                    ci.setFill("green")
                    ci.setOutline("green")
                    ci.draw(win)

