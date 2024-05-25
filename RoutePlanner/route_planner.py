import math
import haversine as hs
from haversine import Unit
from .scan_area import ScanArea


def plan_route(coords, drone_capacities, altitude, intersection_ratio=0.8, route_angle_deg=0, rotated_route_angle_deg=20):
    paths = []

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


    # DEMO PATH FOR 2nd DRONE
    # paths.append((coords, optimal_route_coords, optimal_path_length, rotated_route_coords, rotated_path_length))
    #
    # if len(drone_capacities) == 2:
    #     bound1 = [[41.085857, 29.040856], [41.085101, 29.04061], [41.084972, 29.041243], [41.085756, 29.041489],
    #               [41.085857, 29.040856]]
    #     opt1 = [[41.084972002348444, 29.041242988476228], [41.08575600186303, 29.041488988323774],
    #             [41.08577908020056, 29.041344348842028], [41.0850010937875, 29.041100237461357],
    #             [41.08503018758864, 29.040957474855748], [41.08580216040795, 29.041199697641275],
    #             [41.08582524062216, 29.041055046397755], [41.08505928140343, 29.040814712183185],
    #             [41.08508837523186, 29.04067194944366], [41.08584832084319, 29.04091039511146]]
    #     opt1l = 975 - 400
    #     rot1 = [[41.085760325573645, 29.041461890216674], [41.08568098860741, 29.041465463262018],
    #             [41.085268796062806, 29.04133612733603], [41.085784095071986, 29.0413129190043],
    #             [41.085807864588865, 29.041163947675724], [41.08498051601051, 29.041201212134492],
    #             [41.085010935786244, 29.0410519430024], [41.08583163412429, 29.041014976230944],
    #             [41.08585540367825, 29.040866004669965], [41.085041355591855, 29.040902673723703],
    #             [41.085071775427345, 29.04075340429839], [41.085484560411224, 29.04073480934016]]
    #     rot1l = 400
    #     paths.append((bound1, opt1, opt1l, rot1, rot1l))

    for i in range(len(drone_capacities)):
        paths.append((coords, optimal_route_coords, optimal_path_length, rotated_route_coords, rotated_path_length))
    return paths
