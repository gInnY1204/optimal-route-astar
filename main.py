import pickle
import argparse
from planner import *
from sklearn.neighbors import BallTree

def nearest_node(coord):
    with open("../../OptimalRouteAstar/optimal-route-astar/data/nodes.pickle", "rb") as file:
        nodes = pickle.load(file)

    node_ids = list(nodes.keys())
    node_coords = list(nodes.values())

    # find the nearest node
    tree = BallTree(node_coords, leaf_size=3)
    dist, ind = tree.query(coord, k=1)

    return node_ids[ind[0][0]]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Write the coordinates of the starting point and ending point in order of latitude and longitude.")
    parser.add_argument("-f" , "--frm", required=True, nargs="+", help="start coordinates")
    parser.add_argument("-t", "--to", required=True, nargs="+", help="end coordinates")
    parser.add_argument("-w", "--weight", required=True, nargs="+", help="alpha, beta, gamma")
    parser.add_argument("-s", "--save", required=True, help="save or not")

    args = parser.parse_args()

    frm_lat, frm_lng = args.frm
    to_lat, to_lng = args.to
    alpha, beta, gamma = args.weight
    alpha, beta, gamma = int(alpha), int(beta), int(gamma)

    if alpha == 1 and beta == 0 and gamma == 0:
        path_type = "shortest"
    elif alpha == 0 and beta == 1 and gamma == 0:
        path_type = "fastest"
    elif alpha == 0 and beta == 0 and gamma == 1:
        path_type = "eco-friendly"
    else:
        path_type = "error"
        
    save = args.save
    save = int(save)

    planner = RoutingPlanner(alpha, beta, gamma)
    if save:
        planner.astar_path(nearest_node([(frm_lat, frm_lng)]), nearest_node([(to_lat, to_lng)]))
        print("save in ./result/optimal_" + path_type + "_path.csv")
        print("save")
    else:
        print(planner.astar(nearest_node([(frm_lat, frm_lng)]), nearest_node([(to_lat, to_lng)])))
        print("not_save")
