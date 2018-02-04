import math
import numpy as np

class Node:
    def __init__(self, point, h):
        self.point = point
        self.g = math.inf
        self.h = h
        self.parent = None

    def __eq__(self, other):
        return self.point == other.point


def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def get_neighbors(point, grid):
    """
    Returns a list of all neighbor points
    """
    points = [(point[0] - 1, point[1]),
              (point[0] + 1, point[1]),
              (point[0], point[1] - 1),
              (point[0], point[1] + 1)]
    points = [p for p in points if (0 <= p[0] < len(grid)) and (0 <= p[1] < len(grid[0]))]
    return [point for point in points if grid[point[0]][point[1]] != np.inf]


def astar(grid, start, goal):
    """
    Given a matrix of costs and start/end points, find the shortest path.
    Considers a cost of None to be an obstacle
    :param grid: np matrix of costs for each square
    :param start: (x, y) tuple coordinates of start point
    :param goal: (x, y) tuple coordinates of end point
    :return: list of (x, y) coordinates that define the path from start to goal
    """

    open = {}
    closed = {}
    start_node = Node(start, distance(start, goal))
    start_node.g = 0
    open[start] = start_node
    while open:
        current = sorted(list(open.values()), key=lambda node: node.g + node.h)[0]
        open.pop(current.point)

        if current.point == goal:
            path = [current.point]
            while current.parent:
                path.append(current.parent)
                current = closed[current.parent]
            path.reverse()
            return path

        closed[current.point] = current
        for neighbor_point in get_neighbors(current.point, grid):
            if neighbor_point in closed:
                continue

            if neighbor_point not in open:
                neighbor = Node(neighbor_point, distance(neighbor_point, goal))
            else:
                neighbor = open[neighbor_point]

            if neighbor.g > current.g + distance(current.point, neighbor_point):
                # cost = grid[neighbor_point[0]][neighbor_point[1]]
                neighbor.g = current.g + distance(current.point, neighbor_point)
                neighbor.parent = current.point
                open[neighbor_point] = neighbor

    return "Oh no"


if __name__ == '__main__':
    costs = [[1, 3, 3, None],
             [1, None, 3, 1],
             [1, None, 1, None],
             [1, 3, 1, 1]]

    start = (0, 0)
    goal = (3, 3)
    print(astar(costs, start, goal))
