from node import Node
from scan_area import Scan_area
import matplotlib.pyplot as plt
import math

height              = 0.5            # Scanning resolution
intersection_ratio  = 0.8            # The intersection_ratio is configurable
angle               = math.pi * 15/ 180
#################################### Sample polygon
node_1 = Node(0, 0, 0)
node_2 = Node(1, 6, 0)
node_3 = Node(6, 6, 0)
node_4 = Node(7, 0, 0)
take_off_node       = Node(-1,-1,0)
edge_1              = [node_1,node_2]
edge_2              = [node_2,node_3]
edge_3              = [node_3,node_4]
edge_4              = [node_4,node_1]
edges               = [edge_1,edge_2,edge_3,edge_4]
x_vertices          = [node_1.coordinates[0],node_2.coordinates[0],node_3.coordinates[0],node_4.coordinates[0]]
y_vertices          = [node_1.coordinates[1],node_2.coordinates[1],node_3.coordinates[1],node_4.coordinates[1]]
####################################
area                  = Scan_area(edges)
route,transformed_list   = area.create_route(take_off_node,height,intersection_ratio,True,angle,None,None)


x_list       = []
y_list       = []
x_list_trans = []
y_list_trans = []
### Plot the route: sorted now
for i in range(len(route)):
    x_list.append(route[i].coordinates[0])
    y_list.append(route[i].coordinates[1])
for i in range(len(transformed_list)):
    x_list_trans.append(transformed_list[i].coordinates[0])
    y_list_trans.append(transformed_list[i].coordinates[1])

plt.plot(x_list,y_list,color='green',label='Optimal Route')
plt.plot(x_list_trans,y_list_trans,color='orange',label=f'Route Rotated {int(round(angle*180/math.pi))} degress')
plt.scatter(x_list_trans,y_list_trans,color='olive')
plt.scatter(x_list,y_list,color='purple')
plt.scatter(take_off_node.coordinates[0],take_off_node.coordinates[1],color="blue")
plt.scatter(x_vertices,y_vertices,color='red')
plt.legend()
plt.show()
