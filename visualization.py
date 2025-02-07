import folium
import numpy as np
import pandas as pd

if __name__ == "__main__":
    # central coordinate of New York City
    #central_lat, central_lng = 40.78554, -73.95956

    parser = argparse.ArgumentParser(
        description="Write the coordinates of the starting point and ending point in order of latitude and longitude.")
    parser.add_argument("-c", "--ctl", required=True, nargs="+", help="central_coordinate")
    parser.add_argument("-w", "--weight", required=True, nargs="+", help="alpha, beta, gamma")

    args = parser.parse_args()

    central_lat, central_lng = args.ctl
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


    # get an optimal path
    optimal_path = pd.read_csv("./result/optimal_" + path_type + "_path.csv")

    # visualization
    m = folium.Map(location=(central_lat, central_lng), zoom_start=10)
    lat_path = list(optimal_path['latitude'])
    lng_path = list(optimal_path['longitude'])

    folium.PolyLine(locations=np.array([lat_path, lng_path]).T, smooth_factor=1.0, weight=2.0, color='blue').add_to(m)
    folium.Marker([lat_path[0], lng_path[0]], icon=folium.Icon(icon='none', color='blue')).add_to(m)
    folium.Marker([lat_path[-1], lng_path[-1]], icon=folium.Icon(icon='none', color='red')).add_to(m)

    m.save("./result/"+ path_type + "path.html")
