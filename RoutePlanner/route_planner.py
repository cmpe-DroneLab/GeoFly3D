import math

from matplotlib import pyplot as plt

from node import Node
from scan_area import Scan_area


def plan_route(coords, altitude, intersection_ratio=0.8, angle_deg=15):
    angle = (math.pi * angle_deg / 180)

    nodes = []
    for coord in coords:
        nodes.append(Node(coord[0], coord[1], 0))

    take_off_node = nodes[0]
    edges = []
    len_nodes = len(nodes)
    for i in range(len_nodes):
        edges.append([nodes[i], nodes[(i + 1) % len_nodes]])

    area = Scan_area(edges)
    route, transformed_list = area.create_route(take_off_node, altitude, intersection_ratio, True, angle, None, None)

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
