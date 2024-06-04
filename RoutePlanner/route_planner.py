import math
import haversine as hs
from haversine import Unit
from .scan_area import ScanArea
from UI.helpers import invert_coordinates
from .route_division import route_planning


def plan_route(coords, drone_capacities, altitude, intersection_ratio=0.8, route_angle_deg=0, rotated_route_angle_deg=20):
    paths = []

    # route_angle = (math.pi * route_angle_deg / 180)
    # rotated_route_angle = (math.pi * rotated_route_angle_deg / 180)

    # nodes = []
    # for coord in coords:
    #     nodes.append((coord[0], coord[1]))

    # polygon = ScanArea(coords=nodes)
    # optimal_route_nodes, rotated_route_nodes = polygon.create_route(
    #     altitude, intersection_ratio, route_angle, rotated_route_angle)

    results = route_planning(coordinates=coords, drone_capacities=drone_capacities, altitude=altitude, intersection_ratio=intersection_ratio, angle_offset=route_angle_deg, rotate_angle=rotated_route_angle_deg)


    # optimal_route_coords = []
    # previous_node = optimal_route_nodes[0]
    # optimal_path_length = 0
    # for node in optimal_route_nodes:
    #     dist_btw_nodes = hs.haversine(point1=(
    #         previous_node[1], previous_node[0]), point2=(node[1], node[0]), unit=Unit.METERS)
    #     optimal_path_length += dist_btw_nodes
    #     optimal_route_coords.append((node[1], node[0]))

    #     previous_node = node

    # rotated_route_coords = []
    # previous_node = rotated_route_nodes[0]
    # rotated_path_length = 0
    # for node in rotated_route_nodes:
    #     dist_btw_nodes = hs.haversine(point1=(
    #         previous_node[1], previous_node[0]), point2=(node[1], node[0]), unit=Unit.METERS)
    #     rotated_path_length += dist_btw_nodes
    #     rotated_route_coords.append((node[1], node[0]))

    #     previous_node = node

    # DEMO PATH FOR 2nd DRONE (SIMULATION REGION)
    # paths.append((invert_coordinates(coords), optimal_route_coords,
    #              optimal_path_length, rotated_route_coords, rotated_path_length))

    # if len(drone_capacities) == 2:
    #     bound1 = [[48.879725, 2.370499], [48.880868, 2.371733],
    #               [48.880349, 2.37295], [48.87916, 2.371786]]
    #     opt1 = [[48.87972499500485, 2.370499011378331], [48.880867995102534, 2.371733011484041], [48.88068598300662, 2.3721598105605843], [48.87953935264712, 2.3709218816693047], [
    #         48.87935370573297, 2.3713447623392336], [48.88050396642586, 2.3725866201536223], [48.87942880071144, 2.3720491488882356], [48.87916805925754, 2.3717676420098233]]
    #     opt1l = 705
    #     rot1 = [[48.8793557926286, 2.371977675878631], [48.88051063999924, 2.372570971331269], [48.880698305943156, 2.3721309145803136], [48.87931953712735, 2.371422594189551], [
    #         48.879511704536355, 2.3709848606401915], [48.88077720976348, 2.3716349814944273], [48.87982922014526, 2.370611517637139], [48.879703872901786, 2.3705471249122247]]
    #     rot1l = 667
        # paths.append((bound1, opt1, opt1l, rot1, rot1l))

    # for i in range(len(drone_capacities)):
    #     paths.append((invert_coordinates(coords), optimal_route_coords, optimal_path_length, rotated_route_coords, rotated_path_length))

    if len(drone_capacities) > 1:
        for i in range(len(results.polygons)):
            xx, yy = results.polygons[i].exterior.coords.xy
            bounds = list(zip(xx.tolist(), yy.tolist()))

            paths.append((invert_coordinates(bounds), invert_coordinates(results.optimal_routes[i]), results.optimal_routes_length[i][1]
                        , invert_coordinates(results.rotated_routes[i]), results.rotated_routes_length[i][1]))
    else:
        xx, yy = results.polygons.exterior.coords.xy
        bounds = list(zip(xx.tolist(), yy.tolist()))
        paths.append((invert_coordinates(bounds), invert_coordinates(results.optimal_routes), results.optimal_routes_length
                        , invert_coordinates(results.rotated_routes), results.rotated_routes_length))
        
    return paths
 
def plot():
    x_list = []
    y_list = []
    x_list_trans = []
    y_list_trans = []
    ### Plot the route: sorted now
    for i in range(len(route)):
        x_list.append(route[i].coordinates[0])
        y_list.append(route[i].coordinates[1])
    for i in range(len(transformed_list)):
        x_list_trans.append(transformed_list[i].coordinates[0])
        y_list_trans.append(transformed_list[i].coordinates[1])
    
    plt.plot(x_list, y_list, color='green', label='Optimal Route')
    plt.plot(x_list_trans, y_list_trans, color='orange',
             label=f'Route Rotated {int(round(angle * 180 / math.pi))} degress')
    plt.scatter(x_list_trans, y_list_trans, color='olive')
    plt.scatter(x_list, y_list, color='purple')
    plt.scatter(take_off_node.coordinates[0], take_off_node.coordinates[1], color="blue")
    
    x_vertices = []
    y_vertices = []
    
    for node in nodes:
        x_vertices.append(node.coordinates[0])
        y_vertices.append(node.coordinates[1])
    
    plt.scatter(x_vertices, y_vertices, color='red')
    plt.legend()
    plt.show()