#!/usr/bin/env python

import heapq as heap
import math
from PathPlanning.PathPlanning import Grid, Path


def create_path(start, end, areana_width, arena_height, obstacles):
    grid = Grid(start, end, areana_width, arena_height)  # Create a grid and add the obstacles to it
    for obs in obstacles:
        grid.addObstacle(obs)

    path = a_star(start, end, grid)  # a star algorithm to find path

    # return path, grid
    return post_process(path, grid), grid  # Do theta star algorithm


def a_star(start, end, grid):
    # get start and end indices
    start_coord = grid.getGridIndices(start.getX(), start.getY())
    end_coord = grid.getGridIndices(end.getX(), end.getY())
    # print("Start", start_coord, end_coord)

    startVertex = grid.getVertex(start_coord[0], start_coord[1])
    endVertex = grid.getVertex(end_coord[0], end_coord[1])

    endVertex.x_pos = end.getX()
    endVertex.y_pos = end.getY()

    startVertex.setDistance(0)
    updateHeuristic(endVertex, grid)

#    print(str(startVertex))
#    print(str(endVertex))
    if grid.blocked(*startVertex.get_indices()) or grid.blocked(*endVertex.get_indices()):
        print("Start or end is blocked")
        return

    openList = []
    closedList = []

    heap.heappush(openList, startVertex)  # Add start vertex to start list
    while len(openList) != 0:
        currentVertex = heap.heappop(openList)  # Get the node with the smallest f cost (heap does this for us)

        if currentVertex == endVertex:
            return reconstructPath(currentVertex)  # If we made it return the path we took

        closedList.append(currentVertex)  # Add the looked at vertex to the closed list
        currentCoordinate = grid.getGridIndices(currentVertex.getX(), currentVertex.getY())  # Get the indices

        for neighbor in grid.getNeighbors(currentCoordinate[0], currentCoordinate[1]):  # Check all neighbors
            if neighbor not in closedList:  # Unless they were already checked
                dist = currentVertex.getDistance() + currentVertex.distanceTo(neighbor)
                heuristic = currentVertex.getHeuristic()  # Get current node's heuristic

                # if the neighbor has not been evaluated yet or has just received a better evaluation (dist + heuristic)
                if neighbor not in openList or dist + heuristic < neighbor.getDistance() + neighbor.getHeuristic():
                    neighbor.setDistance(dist)  # Set neighbors cumulative distance from origin
                    neighbor.setParent(currentVertex)  # Tell the neighbor that we came from the current node
                    if neighbor in openList:  # if it's already in the list
                        heap.heapify(openList)  # sort the list
                    else:
                        heap.heappush(openList, neighbor)  # Otherwise add it to the list
    print('could not find a path')
    return None


def reconstructPath(v):
    path = [v]
    while v.getParent():  # Loop through parents backwards to reconstruct path
        path.append(v.getParent())
        v = v.getParent()
    path.reverse()
    return Path(path)


def updateHeuristic(end, grid):
    for i in range(grid.num_rows):
        for j in range(grid.num_cols):
            v = grid.getVertex(i, j)
            v.setHeuristic(v.distanceTo(end))


def post_process(path, grid):
    path = theta_star(path, grid)  # Do theta star

    # remove any nodes that are right next to each other
    path = remove_adjacents(path, 1.01*(grid.unit_height**2 + grid.unit_width**2)**.5)
    path.get_angles()  # TODO use to get optimal path with smallest angle change
    return path


def theta_star(path, grid):
    index = 0
    while True:
        if index < len(path.path) - 2:  # If not at end of path - the nodes that will be joined
            if not checkBlocked(path.path[index], path.path[index + 2], grid):  # if there is a line of sigh
                path.path[index + 2].setParent(path.path[index])  # Join the two end nodes
                path.delete(path.path[index + 1])  # and cut out the middle node
            else:
                index += 1  # check the next node
        else:
            break  # Stop when end of path reached

    return path


def remove_adjacents(path, unit_dist):
    index = 0
    while True:
        pos = path.path[index]  # Get current node
        if index < len(path) - 1:  # If not last node in path
            if pos.distanceTo(path.path[index + 1]) <= unit_dist:  # If next node is right next to current
                if index < len(path) - 2:  # If this is not the second to last node in the path
                    path.path[index + 2].setParent(pos)  # Skip over that adjacent node
                    path.delete(path.path[index + 1])  # Delete it
                else:  # Can't delete next node because it is last in path
                    path.path[index - 1].setParent(path.path[index + 1])  # So ignore the current one instead
                    path.delete(pos)
                    break  # End of path reached
            else:
                index += 1  # Check next node
        else:
            break  # end of path reached

    return path


def checkBlocked(p1, p2, grid):
    v1 = grid.getGridIndices(p1.getX(), p1.getY())  # Get indices of the two points
    v2 = grid.getGridIndices(p2.getX(), p2.getY())
    for i in range(int(min(v1[0], v2[0])), int(max(v1[0], v2[0])) + 1):  # In the whole block (not line?) see if blocked
        for j in range(int(min(v1[1], v2[1])), int(max(v1[1], v2[1])) + 1):
            if grid.blocked(i, j):
                return True
    return False

