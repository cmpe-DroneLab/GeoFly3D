import math
import haversine as hs
from haversine import Unit
from .scan_area import ScanArea


def plan_route(coords, altitude, intersection_ratio=0.8, route_angle_deg=0, rotated_route_angle_deg=20):
    route_angle = (math.pi * route_angle_deg / 180)
    rotated_route_angle = (math.pi * rotated_route_angle_deg / 180)

    nodes = []
    for coord in coords:
        nodes.append((coord[0], coord[1]))

    polygon = ScanArea(coords=nodes)
    optimal_route_nodes, rotated_route_nodes = polygon.create_route(altitude, intersection_ratio, route_angle, rotated_route_angle)

    # x_list = []
    # y_list = []
    # x_list_trans = []
    # y_list_trans = []
    # ### Plot the route: sorted now
    # for i in range(len(route)):
    #     x_list.append(route[i].coordinates[0])
    #     y_list.append(route[i].coordinates[1])
    # for i in range(len(transformed_list)):
    #     x_list_trans.append(transformed_list[i].coordinates[0])
    #     y_list_trans.append(transformed_list[i].coordinates[1])
    #
    # plt.plot(x_list, y_list, color='green', label='Optimal Route')
    # plt.plot(x_list_trans, y_list_trans, color='orange',
    #          label=f'Route Rotated {int(round(angle * 180 / math.pi))} degress')
    # plt.scatter(x_list_trans, y_list_trans, color='olive')
    # plt.scatter(x_list, y_list, color='purple')
    # plt.scatter(take_off_node.coordinates[0], take_off_node.coordinates[1], color="blue")
    #
    # x_vertices = []
    # y_vertices = []
    #
    # for node in nodes:
    #     x_vertices.append(node.coordinates[0])
    #     y_vertices.append(node.coordinates[1])
    #
    # plt.scatter(x_vertices, y_vertices, color='red')
    # plt.legend()
    # plt.show()

    optimal_route_coords = []
    previous_node = optimal_route_nodes[0]
    optimal_path_length = 0
    for node in optimal_route_nodes:
        dist_btw_nodes = hs.haversine(point1=(previous_node[1], previous_node[0]), point2=(node[1], node[0]), unit=Unit.METERS)
        optimal_path_length += dist_btw_nodes
        optimal_route_coords.append((node[1], node[0]))

        previous_node = node

    rotated_route_coords = []
    previous_node = rotated_route_nodes[0]
    rotated_path_length = 0
    for node in rotated_route_nodes:
        dist_btw_nodes = hs.haversine(point1=(previous_node[1], previous_node[0]), point2=(node[1], node[0]), unit=Unit.METERS)
        rotated_path_length += dist_btw_nodes
        rotated_route_coords.append((node[1], node[0]))

        previous_node = node


    return optimal_route_coords, optimal_path_length, rotated_route_coords, rotated_path_length
