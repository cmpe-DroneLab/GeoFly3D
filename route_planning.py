import math
import node

HFOV_degree   = 75.5                            # Check https://www.parrot.com/en/drones/anafi/technical-specifications
HFOV_rad      = (HFOV_degree*math.pi)/180     

def calculate_increment(height):
    increment = math.tan(HFOV_rad/2) * height * 2 * 0.75 * 0.2  # Increment by 0.2 of the height of the taken picture
    return increment

def calculate_start_vertex(start_node,vertex_list):
    min_dist             = 10000000000
    closest_vertex_index = 0
    for i in range(len(vertex_list)):
        dist = start_node.calculate_transposed_distance(vertex_list[i])
        if dist < min_dist:
            min_dist = dist
            closest_vertex_index = i
    return vertex_list[closest_vertex_index]

