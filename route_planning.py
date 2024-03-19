import math
from node import Node
import matplotlib.pyplot as plt

import haversine as hs 
from haversine import Unit, Direction

HFOV_degree   = 75.5                            # Check https://www.parrot.com/en/drones/anafi/technical-specifications
HFOV_rad      = (HFOV_degree*math.pi)/180
     
counter = 0

# Calculate the increment value 
def calculate_increment(height):
    increment = math.tan(HFOV_rad/2) * height * 2 * 0.2  # Increment by 0.2 of the height of the taken picture
    return abs(increment)

# Calculate the closest boundary vertex to the start point
def calculate_start_vertex(start_node,vertex_list):
    min_dist             = 10000000000
    closest_vertex_index = 0
    for i in range(len(vertex_list)):
        # dist = start_node.calculate_transposed_distance(vertex_list[i])
        
        dist = start_node.calculate_geographic_distance(vertex_list[i])
        print(dist)
        if dist < min_dist:
            min_dist = dist
            closest_vertex_index = i
    return vertex_list[closest_vertex_index]

# Calculate the horizontal and vertical displacements with respect to the closest vertex
# def calculate_boundaries(start_vertex,vertex_list):
#     distance_horizontal = 0
#     distance_vertical   = 0
#     for i in range(len(vertex_list)):
#         if vertex_list[i].coordinates[0] == start_vertex.coordinates[0] and vertex_list[i].coordinates[1] != start_vertex.coordinates[1]:
#             distance_vertical = vertex_list[i].coordinates[1] - start_vertex.coordinates[1]
#             if  distance_vertical > 0:
#                 sign_vertical = 1
#             else:
#                 sign_vertical = -1            
#         if vertex_list[i].coordinates[1] == start_vertex.coordinates[1] and vertex_list[i].coordinates[0] != start_vertex.coordinates[0]:
#             distance_horizontal = vertex_list[i].coordinates[0] - start_vertex.coordinates[0]
#             if  distance_horizontal > 0:
#                 sign_horizontal = 1
#             else:
#                 sign_horizontal = -1
#     return sign_horizontal,sign_vertical,abs(distance_horizontal),abs(distance_vertical)


def calculate_boundaries(start_vertex,vertex_list):
    distance_horizontal = 0
    distance_vertical   = 0


    for i in range(len(vertex_list)):

        if vertex_list[i].coordinates[1] == start_vertex.coordinates[1] and vertex_list[i].coordinates[0] != start_vertex.coordinates[0]:
            if  vertex_list[i].coordinates[0] - start_vertex.coordinates[0] > 0:
                direction_vertical = Direction.NORTH
            else:
                direction_vertical = Direction.SOUTH
            
            distance_vertical = hs.haversine(vertex_list[i].geoloc, start_vertex.geoloc, unit=Unit.METERS)

        if vertex_list[i].coordinates[0] == start_vertex.coordinates[0] and vertex_list[i].coordinates[1] != start_vertex.coordinates[1]:
            if vertex_list[i].coordinates[1] - start_vertex.coordinates[1] > 0:
                direction_horizontal = Direction.EAST
            else:
                direction_horizontal = Direction.WEST

            distance_horizontal = hs.haversine(vertex_list[i].geoloc, start_vertex.geoloc, unit=Unit.METERS)

    return direction_horizontal,direction_vertical,abs(distance_horizontal),abs(distance_vertical)

    

# Create a route as a list of nodes with a given height and vertex list
def create_route(start_node,height,vertex_list):
    route           = []
    # Vertical Horizontal ratio is 0.75
    h_increment = calculate_increment(height)
    v_increment = h_increment * 0.75 

    print("Horizontal Increment: " ,h_increment)
    print("Vertical Increment: " ,v_increment)

    start_vertex    = calculate_start_vertex(start_node,vertex_list)
    direction_horizontal, direction_vertical, Dh, Dv  = calculate_boundaries(start_vertex,vertex_list)
    # Sh, Sv, Dh, Dv  = calculate_boundaries(start_vertex,vertex_list)

    start_node      = Node(start_node.coordinates[0],start_node.coordinates[1],start_vertex.coordinates[2])
    route.append(start_node)
    # route.append(start_vertex)
    counter_horizontal = 0
    counter_vertical   = 0
    # start_horizontal   = start_vertex.coordinates[0]
    # start_vertical     = start_vertex.coordinates[1]

    current_latitude = start_vertex.coordinates[0]
    current_longitude = start_vertex.coordinates[1]

    # The vertical displacement will be alternating at every turn
    while counter_horizontal <= Dh:
        while counter_vertical <= Dv:
            altitude          = height
            # start_vertical    = start_vertical + Sv * v_increment 
            # new_coordinates   = [start_horizontal,start_vertical, altitude]
            # route.append(Node(new_coordinates[0],new_coordinates[1],new_coordinates[2]))

            route.append(Node(current_latitude, current_longitude, 0))
            # print(next_node.geoloc)


            current_latitude, current_longitude = hs.inverse_haversine((current_latitude, current_longitude), v_increment, direction_vertical, unit=Unit.METERS)
            # start_vertical = loc[1]

            route.append(Node(current_latitude, current_longitude, 0))
            # print(next_node.geoloc)

            counter_vertical += v_increment
            
        counter_horizontal = counter_horizontal + h_increment

        # start_horizontal   = start_horizontal + Sh * h_increment
        # start_vertical    = start_vertical + Sv * v_increment

        current_latitude, current_longitude = hs.inverse_haversine((current_latitude, current_longitude), h_increment, direction_horizontal, unit=Unit.METERS)

        counter_vertical   = 0
        # Sv = Sv * -1

        if direction_vertical == Direction.NORTH:
            direction_vertical = Direction.SOUTH
        else:
            direction_vertical = Direction.NORTH

    return route

# corner_list = [Node(5,5,0), Node(15,5,0), Node(5,15,0), Node(15,15,0)]


# takeoff = Node(41.0859275, 29.044429, 0)

# loc00=  Node(41.085695, 29.044161, 0)
# loc01= Node(41.085695, 29.044697, 0)
# loc10= Node(41.086160, 29.044161, 0)
# loc11= Node(41.086160, 29.044697, 0)


takeoff = Node(52.29145808851995, 4.939366446653248, 0)

loc00=  Node(52.2912133088, 4.9390984466, 0)
loc01= Node(52.2912133088, 4.9396344466, 0)
loc10= Node(52.2916783088, 4.9390984466, 0)
loc11= Node(52.2916783088, 4.9396344466, 0)

corner_list = [loc00, loc01, loc10, loc11]

route = create_route(takeoff, 15, corner_list)
print(len(route))
x_list = []
y_list = []
for node in route:
    print(node.geoloc)
    # x_list.append(node.coordinates[0])
    # y_list.append(node.coordinates[1])
    x_list.append(node.coordinates[1])
    y_list.append(node.coordinates[0])
plt.title("Route")
# plt.xlabel("X")
# plt.ylabel("Y")

plt.xlabel("Longitude")
plt.ylabel("Latitude")

plt.scatter(x_list, y_list)
plt.show()
