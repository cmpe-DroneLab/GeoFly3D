from node import Node
from scan_area import Scan_area
import matplotlib.pyplot as plt

height              = 0.5            # Scanning resolution
intersection_ratio  = 0.8            # The intersection_ratio is configurable
#################################### Sample polygon
node_1 = Node(0, 0, 0)
node_2 = Node(6, 0, 0)
node_3 = Node(7, 8, 0)
node_4 = Node(0, 7, 0)
node_5 = Node(-2, 4, 0)
take_off_node       = Node(-1,-1,0)
edge_1              = [node_4,node_3]
edge_2              = [node_5,node_1]
edge_3              = [node_4,node_5]
edge_4              = [node_2,node_3]
edge_5              = [node_1,node_2]
edges               = [edge_1,edge_2,edge_3,edge_4,edge_5]
x_vertices          = [node_1.coordinates[0],node_2.coordinates[0],node_3.coordinates[0],node_4.coordinates[0],node_5.coordinates[0]]
y_vertices          = [node_1.coordinates[1],node_2.coordinates[1],node_3.coordinates[1],node_4.coordinates[1],node_5.coordinates[1]]
####################################
area    = Scan_area(edges)
#print("Area is", area.edges)
route   = area.create_route(take_off_node,height,intersection_ratio)
#for i in range(len(route)):
#    print("Angle:",route[i][2])
x_list       = []
y_list       = []

### Plot the route: sorted now
for i in range(len(route)):
    x_list.append(route[i].coordinates[0])
    y_list.append(route[i].coordinates[1])

#print("X:",x_list)
#print("Y:",y_list)
plt.plot(x_list,y_list,color='green')
plt.scatter(x_list,y_list,color='purple')
plt.scatter(take_off_node.coordinates[0],take_off_node.coordinates[1],color="blue")
plt.scatter(x_vertices,y_vertices,color='red')
plt.show() 
