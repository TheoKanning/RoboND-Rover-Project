import math
import heapq

# based on https://github.com/mdeyo/d-star-lite

class Node:
    def __init__(self, point):
        self.point = point
        self.g = math.inf
        self.rhs = math.inf


class NodeDict(dict):
    def __missing__(self, key):
        node = Node(key)
        self.__setitem__(key, node)
        return node


class DStarNavigator:
    def __init__(self):
        self.queue = None
        self.nodes = None
        self.costs = None
        self.old_costs = None
        self.k_m = 0
        self.start_point = None
        self.goal_point = None
        self.last_goal = None

    def initialize(self, start_point, goal_point, costs):
        """
        Wipe away everything and prepare for a new search
        """
        self.nodes = NodeDict()
        self.queue = []
        self.costs = costs
        self.old_costs = costs
        self.k_m = 0
        self.start_point = start_point
        self.goal_point = goal_point
        self.last_goal = goal_point

        goal = Node(goal_point)
        goal.rhs = 0
        self.nodes[goal_point] = goal
        heapq.heappush(self.queue, (heuristic(goal, start_point), 0) + (goal_point,))

    def calculate_key(self, point):
        node = self.nodes[point]
        return min(node.g, node.rhs) + heuristic(node, self.start_point) + self.k_m, min(node.g, node.rhs)

    def top_key(self):
        self.queue.sort()
        if len(self.queue) > 0:
            return self.queue[0][:2]
        else:
            return math.inf, math.inf

    def travel_cost(self, point1, point2):
        return self.costs[point2[0]][point2[1]] + distance(point1, point2)

    def get_neighbors(self, point):
        # return all valid neighbors as (x, y) for a given point
        points = [(point[0] + 1, point[1]),
                  (point[0] + 1, point[1] + 1),
                  (point[0], point[1] + 1),
                  (point[0] - 1, point[1] + 1),
                  (point[0] - 1, point[1]),
                  (point[0] - 1, point[1] - 1),
                  (point[0], point[1] - 1),
                  (point[0] + 1, point[1] - 1)]
        return [p for p in points if (0 <= p[0] < len(self.costs)) and (0 <= p[1] < len(self.costs[0]))]

    def update_vertex(self, vertex_point):
        vertex = self.nodes[vertex_point]
        if vertex_point != self.goal_point:
            min_rhs = math.inf
            for neighbor in self.get_neighbors(vertex_point):
                min_rhs = min(min_rhs, self.nodes[neighbor].g + self.travel_cost(vertex_point, neighbor))
            vertex.rhs = min_rhs

        id_in_queue = [item for item in self.queue if vertex_point == item[2]]
        if id_in_queue:
            if len(id_in_queue) != 1:
                raise ValueError('more than one ' + vertex_point + ' in the queue!')
            self.queue.remove(id_in_queue[0])
        if vertex.rhs != vertex.g:
            heapq.heappush(self.queue, self.calculate_key(vertex_point) + (vertex_point,))

    def compute_shortest_path(self):
        while self.top_key() < self.calculate_key(self.start_point) or \
                        self.nodes[self.start_point].rhs > self.nodes[self.start_point].g:
            k_old = self.top_key()
            u = heapq.heappop(self.queue)[2]
            if k_old < self.calculate_key(u):
                # key has changed, add back into queue
                heapq.heappush(self.queue, self.calculate_key(u) + (u,))
            elif self.nodes[u].g > self.nodes[u].rhs:
                # rhs is better than existing g, update all parents
                self.nodes[u].g = self.nodes[u].rhs
                for neighbor in self.get_neighbors(u):
                    self.update_vertex(neighbor)
            else:
                self.nodes[u].g = math.inf
                self.update_vertex(u)
                for neighbor in self.get_neighbors(u):
                    self.update_vertex(neighbor)

    def update_costs(self, costs):
        change_x, change_y = (self.costs - costs).nonzero()
        self.costs = costs
        for x, y in zip(change_x, change_y):
            if distance((x,y), self.start_point) < 4 and (x,y) != self.goal_point:
                self.update_vertex((x,y))

    def extract_path(self):
        current = self.start_point
        path = []
        while current != self.goal_point:
            min_rhs = math.inf
            next_point = None
            for neighbor in self.get_neighbors(current):
                neighbor_cost = self.nodes[neighbor].g + self.travel_cost(current, neighbor)
                if neighbor_cost < min_rhs:
                    min_rhs = neighbor_cost
                    next_point = neighbor
            if next_point:
                path.append(next_point)
                current = next_point
            else:
                raise ValueError('Could not find path')

        return path

    def find_path(self, start_point, goal_point, costs):
        if goal_point != self.last_goal:
            print("\n\n\n New Goal \n\n\n")
            # starting a new search, wipe everything
            self.initialize(start_point, goal_point, costs)
            self.compute_shortest_path()
        else:
            self.update_costs(costs)
            self.k_m += heuristic(self.nodes[self.start_point], start_point)
            self.start_point = start_point
            self.compute_shortest_path()

        return self.extract_path()


def heuristic(node, start):
    return distance(node.point, start)


def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


if __name__ == '__main__':
    test_costs = [[1, 1, 3, 1000],
                  [1, 1000, 10, 1],
                  [1, 1000, 1, 1000],
                  [1, 3, 1, 1]]

    test_start = (0, 1)
    test_goal = (3, 3)
    solver = DStarNavigator()
    print(solver.find_path(test_start, test_goal, test_costs))
