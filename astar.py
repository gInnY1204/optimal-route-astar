import math
import heapq
from typing import Dict, Tuple, List
import csv


class Vertex:
    def __init__(self, node_id: int, coord: Tuple[float, float]):
        self.id = node_id
        self.coord = coord
        self.visited = False
        self.adjacent: Dict['Vertex', float] = {}
        self.distance = float('inf')
        self.heuristic = float('inf')
        self.cost = float('inf')
        self.prev: 'Vertex' = None

    def initialize(self):
        self.visited = False
        self.distance = float('inf')
        self.heuristic = float('inf')
        self.cost = float('inf')

    def get_id(self):
        return self.id

    def add_neighbor(self, neighbor: 'Vertex', weight: float = 0):
        self.adjacent[neighbor] = weight

    def get_connection(self):
        return set(self.adjacent.keys())

    def get_weight(self, neighbor):
        return self.adjacent.get(neighbor, None)

    def set_distance(self, dist: float):
        self.distance = dist
        self.set_cost()

    def set_cost(self):
        self.cost = self.distance + self.heuristic

    def set_visited(self):
        self.visited = True

    def set_previous(self, prev: 'Vertex'):
        self.prev = prev

    def __lt__(self, other: 'Vertex'):
        return (self.cost, self.id) < (other.cost, other.id)


class Graph:
    def __init__(self, alpha: float, beta: float, gamma: float):
        self.graph: Dict[int, Vertex] = {}
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.max_speed_limit = 22.352
        self.min_carbon_emission = -3.609

    @staticmethod
    def degree_to_radian(degree: float) -> float:
        return degree * math.pi / 180

    def haversine(self, frm_coord: Tuple[float, float], to_coord: Tuple[float, float]) -> float:
        long1, lat1 = map(self.degree_to_radian, frm_coord)
        long2, lat2 = map(self.degree_to_radian, to_coord)
        dlong, dlat = long2 - long1, lat2 - lat1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlong / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))
        return 6371000 * c  # meters

    def heu_carbon_emission(self, frm_coord, to_coord):
        distance = self.haversine(frm_coord, to_coord)
        return distance * self.min_carbon_emission  # Carbon emission estimation

    def heu_travel_time(self, frm_coord, to_coord):
        distance = self.haversine(frm_coord, to_coord)
        return distance / self.max_speed_limit  # Travel time assuming max speed

    def heuristic(self, frm_coord: Tuple[float, float], to_coord: Tuple[float, float]) -> float:
        return (self.alpha * self.haversine(frm_coord, to_coord) +
                self.beta * self.heu_travel_time(frm_coord, to_coord) +
                self.gamma * self.heu_carbon_emission(frm_coord, to_coord))

    def add_vertex(self, node_id: int, longitude: float, latitude: float):
        self.graph[node_id] = Vertex(node_id, (longitude, latitude))

    def add_edge(self, frm: int, to: int, weight: float):
        self.graph[frm].add_neighbor(self.graph[to], weight)

    def route_planning(self, start_node: int, end_node: int) -> float:
        start, end = self.graph[start_node], self.graph[end_node]
        start.set_distance(0.0)
        start.heuristic = self.heuristic(start.coord, end.coord)
        start.set_cost()

        heap = [start]
        heapq.heapify(heap)

        while heap:
            current = heapq.heappop(heap)
            if current == end:
                return end.cost
            current.set_visited()

            for neighbor, weight in current.adjacent.items():
                if not neighbor.visited:
                    new_cost = current.distance + weight
                    if new_cost < neighbor.distance:
                        neighbor.set_distance(new_cost)
                        neighbor.heuristic = self.heuristic(neighbor.coord, end.coord)
                        neighbor.set_cost()
                        neighbor.set_previous(current)
                        heapq.heappush(heap, neighbor)

        return float('inf')  # No path found

    def get_vertex(self, id):
        return self.graph[id]

    def add_heuristic(self, node, end_node):
        node_coord = self.get_coordinate(node)
        end_coord = self.get_coordinate(end_node)
        self.graph[node].heuristic = self.heuristic(node_coord, end_coord)
        self.graph[node].set_cost()

    def get_coordinate(self, node_id):
        return self.graph[node_id].coord

    def route_planning_details(self, start_node, end_node):
        current = None
        heap = []
        unvisited_queue = set()
        longitude = []
        latitude = []

        start = self.get_vertex(start_node)
        end = self.get_vertex(end_node)

        start.set_distance(0.0)
        self.add_heuristic(start.get_id(), end_node)
        heapq.heappush(heap, (start.cost, start))
        unvisited_queue.add(start)

        while current != end:
            pop_cost, current = heapq.heappop(heap)
            unvisited_queue.discard(current)
            current.set_visited()

            for adj in current.get_connection():
                if not adj.visited:
                    self.add_heuristic(adj.get_id(), end_node)
                    unvisited_queue.add(adj)

                    new_cost = current.distance + current.get_weight(adj) + adj.heuristic
                    if new_cost < adj.cost:
                        adj.set_distance(current.distance + current.get_weight(adj))
                        adj.set_previous(current)

                    heapq.heappush(heap, (adj.cost, adj))

        # Find the optimal path
        cause = end
        while cause != start:
            longitude.append(cause.coord[0])
            latitude.append(cause.coord[1])
            cause = cause.prev

        longitude.append(start.coord[0])
        latitude.append(start.coord[1])

        longitude.reverse()
        latitude.reverse()

        if self.alpha == 1 and self.beta == 0 and self.gamma == 0:
            path_type = "shortest"
        elif self.alpha == 0 and self.beta == 1 and self.gamma == 0:
            path_type = "fastest"
        elif self.alpha == 0 and self.beta == 0 and self.gamma == 1:
            path_type = "eco-friendly"
        else:
            path_type = "error"

        with open("./result/optimal_" + path_type + "_path.csv", "w", newline='') as result:
            writer = csv.writer(result)
            writer.writerow(["longitude", "latitude"])
            for lon, lat in zip(longitude, latitude):
                writer.writerow([lon, lat])
                
    def get_optimal_path(self, start_node: int, end_node: int) -> List[Tuple[float, float]]:
        path = []
        current = self.graph[end_node]
        while current:
            path.append(current.coord)
            current = current.prev
        return path[::-1]  # Reverse to get start -> end
