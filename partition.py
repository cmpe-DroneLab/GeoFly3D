from shapely.geometry import LineString,Polygon, Point
import math
import matplotlib.pyplot as plt


def furthest_vertex(polygon, reference_vertex):
    max_distance = float('-inf')
    furthest_vertex  = None
    # Iterate over each vertex of the polygon
    for vertex in polygon.exterior.coords:
        # Calculate the distance between the reference vertex and the current vertex
        distance = reference_vertex.distance(Point(vertex))
        
        # Update furthest vertex if distance is greater than max_distance
        if distance > max_distance:
            max_distance = distance
            furthest_vertex = vertex
    
    return furthest_vertex,max_distance

def orientation_decider(polygon,partition_line):
    center_polygon       = polygon.centroid
    line_start, line_end = partition_line.coords
    
    # Calculate the angles between the line and the vectors from the polygon center to each line endpoint
    angle_start = math.atan2(line_start[1] - center_polygon.y, line_start[0] - center_polygon.x)
    angle_end   = math.atan2(line_end[1] - center_polygon.y, line_end[0] - center_polygon.x)
    
    # Calculate the difference in angles
    angle_diff = angle_end - angle_start
    
    # Determine if the line is to the right or left of the center of the polygon
    if angle_diff > math.pi:
        return "Right"
    elif angle_diff < -math.pi:
        return "Left"
    elif angle_diff > 0:
        return "Right"
    elif angle_diff < 0:
        return "Left"
    else:
        pass

def rotate_line(line, angle):
    # Calculate sine and cosine of the angle
    cos_angle        = math.cos(angle)
    sin_angle        = math.sin(angle)
    rotation_point,_ = line.coords
    # Function to rotate a single point
    def rotate_point(point):
        x, y = point
        dx = x - rotation_point[0]
        dy = y - rotation_point[1]
        x_rotated = dx * cos_angle - dy * sin_angle + rotation_point[0]
        y_rotated = dx * sin_angle + dy * cos_angle + rotation_point[1]
        return x_rotated, y_rotated

    # Rotate each point of the line
    rotated_points = [rotate_point(point) for point in line.coords]

    # Create a new LineString from the rotated points
    rotated_line = LineString(rotated_points)

    return rotated_line


def optimization_function(polygon_1,polygon_2):
    polygon_area_1 = polygon_1.area
    polygon_area_2 = polygon_1.area
    return abs(polygon_area_1-polygon_area_2)

def partition_polygon(polygon,division_line):
    # Split the shape along the line
    divided_shapes = polygon.difference(division_line.buffer(0.000001))
    print(divided_shapes)
    return divided_shapes



def parallel_partitions(polygon, partition_line,start_vertex):
    rotation_point       = start_vertex
    far_vertex,distance  = furthest_vertex(polygon,start_vertex)
    number_partition     = 20    
    interpartition_disp  = distance / (number_partition + 1)
    partition_direction  = orientation_decider(polygon,partition_line)
    partition_list       = []
    for counter in range(number_partition):
        line_start, _  = partition_line.coords
        rotation_point = Point(line_start)
        for i in range(1,23):
            angle_radians      = math.radians(i)  
            rotated_line       = rotate_line(partition_line,angle_radians)
            partitioned_shapes = partition_polygon(polygon,rotated_line)
            print(partitioned_shapes)
            value              = optimization_function(partitioned_shapes[0],partitioned_shapes[1])
            partition_list.append([partitioned_shapes[0],partitioned_shapes[1],value])
        partition_line = partition_line.parallel_offset(interpartition_disp, side=partition_direction)
    return len(partition_list)



polygon_coords = [(0, 0), (0, 3), (3, 3), (3, 0)]
# Define the line that partitions the polygon
partition_line = LineString([(0, 10), (0, -10)])
# Create a Polygon
polygon        = Polygon(polygon_coords)
line_start, _  = partition_line.coords
part_len       = parallel_partitions(polygon,partition_line,Point(line_start))
print(part_len)
# Extract the x and y coordinates of the polygon exterior
x, y = polygon.exterior.xy

# Plot the polygon
plt.plot(x, y, color='blue')  # Plot the exterior of the polygon in blue
plt.fill(x, y, color='lightblue', alpha=0.5)  # Fill the interior of the polygon in light blue with transparency
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Polygon Visualization')
plt.grid(True)
plt.axis('equal')
plt.show()