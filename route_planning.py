import math
from node import Node
import matplotlib.pyplot as plt

HFOV_degree   = 75.5                            # Check https://www.parrot.com/en/drones/anafi/technical-specifications
HFOV_rad      = (HFOV_degree*math.pi)/180
     
counter = 0

# Calculate the increment value 
def calculate_increment(height):
    increment = math.tan(HFOV_rad/2) * height * 2 * 0.75 * 0.2  # Increment by 0.2 of the height of the taken picture
    return abs(increment)

# Calculate the closest boundary vertex to the start point
def calculate_start_vertex(start_node,vertex_list):
    min_dist             = 10000000000
    closest_vertex_index = 0
    for i in range(len(vertex_list)):
        dist = start_node.calculate_transposed_distance(vertex_list[i])
        if dist < min_dist:
            min_dist = dist
            closest_vertex_index = i
    return vertex_list[closest_vertex_index]

# Calculate the horizontal and vertical displacements with respect to the closest vertex
def calculate_boundaries(start_vertex,vertex_list):
    distance_horizontal = 0
    distance_vertical   = 0
    for i in range(len(vertex_list)):
        if vertex_list[i].coordinates[0] == start_vertex.coordinates[0] and vertex_list[i].coordinates[1] != start_vertex.coordinates[1]:
            distance_vertical = vertex_list[i].coordinates[1] - start_vertex.coordinates[1]
            if  distance_vertical > 0:
                sign_vertical = 1
            else:
                sign_vertical = -1            
        if vertex_list[i].coordinates[1] == start_vertex.coordinates[1] and vertex_list[i].coordinates[0] != start_vertex.coordinates[0]:
            distance_horizontal = vertex_list[i].coordinates[0] - start_vertex.coordinates[0]
            if  distance_horizontal > 0:
                sign_horizontal = 1
            else:
                sign_horizontal = -1
    return sign_horizontal,sign_vertical,abs(distance_horizontal),abs(distance_vertical)

# Create a route as a list of nodes with a given height and vertex list
def create_route(start_node,height,vertex_list):
    route           = []
    increment       = calculate_increment(height)
    print(increment)
    start_vertex    = calculate_start_vertex(start_node,vertex_list)
    Sh, Sv, Dh, Dv  = calculate_boundaries(start_vertex,vertex_list)
    start_node      = Node(start_node.coordinates[0],start_node.coordinates[1],start_vertex.coordinates[2])
    route.append(start_node)
    route.append(start_vertex)
    counter_horizontal = 0
    counter_vertical   = 0
    start_horizontal   = start_vertex.coordinates[0]
    start_vertical     = start_vertex.coordinates[1]

    # The vertical displacement will be alternating at every turn
    while counter_horizontal <= Dh:
        while counter_vertical <= Dv:
            altitude          = height
            start_vertical    = start_vertical + Sv * increment 
            new_coordinates   = [start_horizontal,start_vertical, altitude]
            route.append(Node(new_coordinates[0],new_coordinates[1],new_coordinates[2]))
            counter_vertical += increment
        counter_horizontal = counter_horizontal + increment
        start_horizontal   = start_horizontal + Sh * increment
        start_vertical    = start_vertical + Sv * increment
        counter_vertical   = 0
        Sv                 = Sv * -1
    return route

corner_list = [Node(5,5,0), Node(15,5,0), Node(5,15,0), Node(15,15,0)]
route = create_route(Node(0,0,0), 2, corner_list)
x_list = []
y_list = []
for node in route:
    x_list.append(node.coordinates[0])
    y_list.append(node.coordinates[1])
plt.title("Route")
plt.xlabel("X")
plt.ylabel("Y")
plt.scatter(x_list, y_list)
plt.show()

