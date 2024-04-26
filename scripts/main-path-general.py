from node import Node
from scan_area import Scan_area
import matplotlib.pyplot as plt
import math

height = 70  # Scanning resolution
intersection_ratio = 0.8  # The intersection_ratio is configurable
angle = (math.pi * 15 / 180)
#################################### Sample polygon
node_1 = Node(2.37, 48.8802, 0)
node_2 = Node(2.37, 48.8810, 0)
node_3 = Node(2.372, 48.8810, 0)
node_4 = Node(2.372, 48.8802, 0)

# node_1 = Node(29.043099, 41.086961, 0)
# node_2 = Node(29.043839, 41.087066, 0)
# node_3 = Node(29.045663, 41.08671, 0)
# node_4 = Node(29.045727, 41.085441, 0)
# node_5 = Node(29.043775, 41.085085, 0)
take_off_node = Node(2.371, 48.8801, 0)

nodes = [node_1, node_2, node_3, node_4]
edges = []
len_nodes = len(nodes)
for i in range(len_nodes):
    edges.append([nodes[i], nodes[(i + 1) % len_nodes]])

####################################
area = Scan_area(edges)
route, transformed_list, increment = area.create_route(take_off_node, height, intersection_ratio, True, angle, None, None)


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
plt.plot(x_list_trans, y_list_trans, color='orange', label=f'Route Rotated {int(round(angle * 180 / math.pi))} degress')
plt.scatter(x_list_trans, y_list_trans, color='olive')
plt.scatter(x_list, y_list, color='purple')
plt.scatter(take_off_node.coordinates[0], take_off_node.coordinates[1], color="blue")

x_vertices = []
y_vertices = []

for node in nodes:
    x_vertices.append(node.coordinates[0])
    y_vertices.append(node.coordinates[1])

# plt.scatter(x_vertices, y_vertices, color='red')
plt.legend()
plt.show()
