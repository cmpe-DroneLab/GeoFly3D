from node import Node
from scan_area import Scan_area
import matplotlib.pyplot as plt

height              = 0.5
intersection_ratio  = 0.8
node_1              = Node(1,3,0)
node_2              = Node(2,4,0)
node_3              = Node(4,4,0)
node_4              = Node(0,0,0)
take_off_node       = Node(-1,-1,0)
edge_1              = [node_4,node_3]
edge_2              = [node_4,node_1]
edge_3              = [node_1,node_2]
edge_4              = [node_2,node_3]
edges               = [edge_1,edge_2,edge_3,edge_4]
x_vertices          = [1,2,4,0]
y_vertices          = [3,4,4,0]

area  = Scan_area(edges)
#print("Area is", area.edges)
route  = area.create_route(take_off_node,height,intersection_ratio)
x_list = []
y_list = []

for i in range(len(route)):
    for j in range(len(route[i][1])):
        x_list.append(route[i][1][j].coordinates[0])
        y_list.append(route[i][1][j].coordinates[1])

#print("X:",x_list)
#print("Y:",y_list)
plt.scatter(x_list,y_list)
plt.scatter(x_vertices,y_vertices)
plt.show() 
