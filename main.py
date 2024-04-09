from node import Node
from scan_area import Scan_area

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

area  = Scan_area(edges=edges)
route = area.create_route(take_off_node,height,intersection_ratio)
print("Route",route)
