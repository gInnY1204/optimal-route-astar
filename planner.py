from astar import *

class RoutingPlanner:
    def __init__(self, alpha, beta, gamma):
        self.pGraph = Graph(alpha, beta, gamma)

        # Read graph information from CSV files
        with open("../../OptimalRouteAstar/optimal-route-astar/data/nodes_N96570.csv", 'r') as fnode:
            next(fnode)  # Skip the header
            for line in fnode:
                row = line.strip().split(',')
                node_id = int(float(row[1]))
                longitude = float(row[2])
                latitude = float(row[3])
                self.pGraph.add_vertex(node_id, longitude, latitude)

        with open("../../OptimalRouteAstar/optimal-route-astar/data/links_L260855.csv", 'r') as flink:
            next(flink)  # Skip the header
            for line in flink:
                row = line.strip().split(',')
                frm = int(row[1])
                to = int(row[2])
                street_length = float(row[7])
                travel_time = float(row[8])
                speed_limit = float(row[9])
                elevation = float(row[11])
                cost = alpha * street_length + beta * travel_time + gamma * elevation * street_length
                self.pGraph.add_edge(frm, to, cost)

    def astar(self, frm, to):
        return self.pGraph.route_planning(frm, to)

    def astar_path(self, frm, to):
        return self.pGraph.route_planning_details(frm, to)
