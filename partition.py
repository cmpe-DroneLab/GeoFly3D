from shapely.geometry import LineString,Polygon, Point,MultiPolygon
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
        return "right"
    elif angle_diff < -math.pi:
        return "left"
    elif angle_diff > 0:
        return "right"
    elif angle_diff < 0:
        return "left"
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


def plot_partition(polygon):
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


def partition_polygon(polygon,division_line):
    separator        = division_line.buffer(0.000001)  #is polygon
    # the `difference` operation between 2 polygons
    partitions       = polygon.difference(separator)
    geom_type_check  = partitions.geom_type
    if geom_type_check == 'MultiPolygon':
        polygons = list(partitions.geoms)
        return polygons
    elif geom_type_check == 'Polygon':
        return [partitions]
    else:
        return []

def stretch_line(line,distance):
    # Get the coordinates of the endpoints of the original LineString
    start_point = Point(line.coords[0])
    end_point = Point(line.coords[-1])
    
    # Calculate the unit vector along the original LineString
    dx = end_point.x - start_point.x
    dy = end_point.y - start_point.y
    length = line.length
    unit_vector = (dx / length, dy / length)
    
    # Create new points by extending from the original endpoints
    extended_start_point = Point(start_point.x - distance * unit_vector[0], start_point.y - distance * unit_vector[1])
    extended_end_point = Point(end_point.x + distance * unit_vector[0], end_point.y + distance * unit_vector[1])
    
    # Create a new LineString using the extended points
    extended_line = LineString([extended_start_point, extended_end_point])
    
    return extended_line


def parallel_partitions(polygon, partition_line,start_vertex):
    rotation_point       = start_vertex
    far_vertex,distance  = furthest_vertex(polygon,start_vertex)
    number_partition     = 20    
    interpartition_disp  = distance / (number_partition + 1)
    partition_direction  = orientation_decider(polygon,partition_line)
    partition_list       = []
    rot_sign             = None
    if partition_direction == "left":
        rot_sign = 1
    else:
        rot_sign = -1
    partition_line  = stretch_line(partition_line,distance)
    ideal_partition = None
    min_val         = float("inf")
    for counter in range(number_partition):
        line_start, _  = partition_line.coords
        rotation_point = Point(line_start)
        for i in range(1,23):
            angle_radians      = math.radians(i*rot_sign)  
            rotated_line       = rotate_line(partition_line,angle_radians)
            partitioned_shapes = partition_polygon(polygon,rotated_line)
            if len(partitioned_shapes) == 2:
                value              = optimization_function(partitioned_shapes[0],partitioned_shapes[1])
                if min_val > value:
                    min_val = value
                    ideal_partition = [partitioned_shapes[0],partitioned_shapes[1]]            
            #partition_list.append([partitioned_shapes[0],partitioned_shapes[1],value])
        partition_line = partition_line.parallel_offset(interpartition_disp, side=partition_direction)
        #plt.show()
    return ideal_partition


def get_partitions(polygon_coords,line_start,line_end):
    # Create a Polygon
    polygon        = Polygon(polygon_coords)
    partition_line = LineString([line_start, line_end])
    ideal_partition= parallel_partitions(polygon,partition_line,Point(line_start))
    #print(ideal_partition)
    plot_partition(ideal_partition[0])
    plot_partition(ideal_partition[1])


# Extract the x and y coordinates of the polygon exterior
polygon_coords = [(0, 0), (0, 3), (3, 3), (3, 0)]
get_partitions(polygon_coords,(0,0),(0,3))