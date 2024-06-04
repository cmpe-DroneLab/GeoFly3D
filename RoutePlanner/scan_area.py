import haversine as hs
from haversine import Unit
from shapely import affinity
from shapely.geometry import *
import matplotlib.pyplot as plt
from math import cos, sin, radians
import math


def calculate_geographic_distance(point1, point2):
    return hs.haversine((point1[1], point1[0]), (point2[1], point2[0]), unit=Unit.METERS)


def calculate_bearing_angle(point1, point2):
    lon_a, lat_a = point1
    lon_b, lat_b = point2

    X = cos(radians(lat_b)) * sin(radians(abs(lon_a - lon_b)))
    Y = cos(radians(lat_a)) * sin(radians(lat_b)) - sin(radians(lat_a)) * cos(radians(lat_b)) * cos(
        radians(abs(lon_a - lon_b)))

    sign = 1 if lon_a < lon_b else -1
    return sign * math.atan2(X, Y)


def calculate_coordinate_with_distance(point, distance, bearing_angle):
    lat, lon = hs.inverse_haversine(
        (point[1], point[0]), distance, bearing_angle, unit=Unit.METERS)
    return lon, lat

def get_edges(coords):
    edges = []
    for i in range(len(coords) - 1):
        edges.append(LineString([coords[i], coords[i + 1]]))
    edges.append(LineString([coords[-1], coords[0]]))  # Closing the polygon
    return edges

def calculate_increment(height, intersection_ratio):
    HFOV_degree = 75.5  # Check https://www.parrot.com/en/drones/anafi/technical-specifications
    HFOV_rad = (HFOV_degree * math.pi) / 180
    # Horizontal increment
    increment = math.tan(HFOV_rad / 2) * height * 2 * (
        1 - intersection_ratio)  # Increment by variable intersection rates of the height of the taken picture
    return abs(increment)


class ScanArea:
    def __init__(self, coords):
        self.polygon = Polygon(coords)

    def find_extension_amount(self):
        minx, miny, maxx, maxy = self.polygon.bounds
        diagonal = LineString([(minx, miny), (maxx, maxy)])
        return diagonal.length

    def find_normal_direction(self, longest_edge):
        center = self.polygon.centroid

        vector_1 = [longest_edge.coords[0][i] -
                    longest_edge.coords[1][i] for i in range(2)]
        vector_2 = [center.coords[0][i] - longest_edge.coords[0][i]
                    for i in range(2)]

        cross_multiple = (vector_1[0] * vector_2[1]) - \
            (vector_1[1] * vector_2[0])

        return -1 if cross_multiple < 0 else 1

    def scan_polygon(self, longest_edge, distance, angle=0):

        minx, miny, maxx, maxy = self.polygon.bounds
        bounding_box = box(minx, miny, maxx, maxy)

        if angle == math.pi:
            # Reverse the coordinates
            longest_edge = LineString(
                [longest_edge.coords[1], longest_edge.coords[0]])
        elif angle != 0:
            # Rotate the longest edge
            longest_edge = affinity.rotate(
                longest_edge, angle, origin='center', use_radians=True)
            coords = longest_edge.coords
            slope_radian = math.atan2(
                coords[0][1] - coords[1][1], coords[0][0] - coords[1][0])
            if slope_radian < 0:
                slope_radian += math.pi
            if slope_radian < math.pi / 2:
                xoff = maxx - longest_edge.centroid.coords[0][0]
                yoff = miny - longest_edge.centroid.coords[0][1]
            else:
                xoff = minx - longest_edge.centroid.coords[0][0]
                yoff = miny - longest_edge.centroid.coords[0][1]

            longest_edge = affinity.translate(longest_edge, xoff, yoff)

        # Calculate the normal vector (perpendicular) to the line
        dx = longest_edge.coords[1][0] - longest_edge.coords[0][0]
        dy = longest_edge.coords[1][1] - longest_edge.coords[0][1]
        magnitude = longest_edge.length

        normal_direction = self.find_normal_direction(longest_edge)
        distance *= normal_direction
        vector = (dx / magnitude, dy / magnitude)

        extension_amount = self.find_extension_amount()
        extension_vector = (vector[0] * extension_amount,
                            vector[1] * extension_amount)

        extended_coords = [
            (longest_edge.coords[0][0] - extension_vector[0],
             longest_edge.coords[0][1] - extension_vector[1]),
            (longest_edge.coords[1][0] + extension_vector[0], longest_edge.coords[1][1] + extension_vector[1])]

        normal_bearing_angle = calculate_bearing_angle(
            extended_coords[0], extended_coords[1]) + math.pi / 2

        parallel_lines = []
        intersects = []

        i = 0
        # for i in range(5):
        while True:
            if i > 0:
                # Move the line along the normal vector
                new_line_coords = [
                    calculate_coordinate_with_distance(
                        extended_coords[0], distance * i, normal_bearing_angle),
                    calculate_coordinate_with_distance(
                        extended_coords[1], distance * i, normal_bearing_angle)
                ]
                new_line = LineString(new_line_coords)
            else:
                new_line_coords = [
                    calculate_coordinate_with_distance(extended_coords[0], 0.001 * normal_direction,
                                                       normal_bearing_angle),
                    calculate_coordinate_with_distance(extended_coords[1], 0.001 * normal_direction,
                                                       normal_bearing_angle)
                ]
                new_line = LineString(new_line_coords)

            intersection = self.polygon.intersection(new_line)

            if bounding_box.intersection(new_line).is_empty:
                break

            parallel_lines.append(new_line)
            if not intersection.is_empty:
                intersects.append(intersection)

            i += 1

        return parallel_lines, intersects, bounding_box

    def find_the_longest_edge(self):
        b = self.polygon.boundary.coords
        line_strings = [LineString(b[k:k + 2]) for k in range(len(b) - 1)]
        return max(line_strings, key=lambda edge: edge.length)

    def create_route(self, altitude, intersection_ratio, route_angle, rotated_route_angle,edge_idx = None):
        if edge_idx != None:
            longest_edge = get_edges(list(self.polygon.exterior.coords))[edge_idx]
        else:
            longest_edge = self.find_the_longest_edge()
            
        # Distance between parallel lines
        dist = calculate_increment(altitude, intersection_ratio)

        parallels, intersections, bb = self.scan_polygon(
            longest_edge, dist, route_angle)

        # self.plot_path(parallels, intersections, bb, 'Optimal Route')

        optimal_route = []
        for i in range(len(intersections)):
            if intersections[i].is_empty:
                continue
            if i % 2 == 0:
                optimal_route.append(intersections[i].coords[0])
                optimal_route.append(intersections[i].coords[1])
            else:
                optimal_route.append(intersections[i].coords[1])
                optimal_route.append(intersections[i].coords[0])

        # Second Route

        parallels, intersections, bb = self.scan_polygon(
            longest_edge, dist, route_angle + rotated_route_angle)

        rotated_route = []

        dist1 = calculate_geographic_distance(
            optimal_route[-1], intersections[0].coords[0])
        dist2 = calculate_geographic_distance(
            optimal_route[-1], intersections[-1].coords[1])

        print(dist1, dist2)

        if dist2 < dist1:
            for i in range(len(intersections)-1, -1, -1):
                if intersections[i].is_empty:
                    continue
                if i % 2 == 0:
                    rotated_route.append(intersections[i].coords[0])
                    rotated_route.append(intersections[i].coords[1])
                else:
                    rotated_route.append(intersections[i].coords[1])
                    rotated_route.append(intersections[i].coords[0])
        else:

            for i in range(len(intersections)):
                if intersections[i].is_empty:
                    continue
                if i % 2 == 0:
                    rotated_route.append(intersections[i].coords[0])
                    rotated_route.append(intersections[i].coords[1])
                else:
                    rotated_route.append(intersections[i].coords[1])
                    rotated_route.append(intersections[i].coords[0])

        # self.plot_path(parallels, intersections, bb, 'Rotated Route')

        return optimal_route, rotated_route

    def plot_path(self, parallels, intersections, bounding_box, label):
        # Plotting
        x, y = self.polygon.exterior.xy
        plt.plot(x, y, color='black', label='Boundary')

        x, y = bounding_box.exterior.xy
        plt.plot(x, y, 'blue')  # Plot polygon boundary

        for line_string in parallels:
            x = [x for x, _ in line_string.coords]
            y = [y for _, y in line_string.coords]
            plt.plot(x, y, 'r--')  # Plot parallel lines
        route = []

        for i in range(len(intersections)):
            if intersections[i].is_empty:
                continue
            if i % 2 == 0:
                route.append(intersections[i].coords[0])
                route.append(intersections[i].coords[1])
            else:
                route.append(intersections[i].coords[1])
                route.append(intersections[i].coords[0])

        # for line_string in intersections:
        #     x = [x for x, _ in line_string.coords]
        #     y = [y for _, y in line_string.coords]
        #     plt.plot(x, y, 'b')  # Plot intersection points

        # plt.show()
        x_list = []
        y_list = []

        for i in range(len(route)):
            x_list.append(route[i][0])
            y_list.append(route[i][1])

        plt.plot(x_list, y_list, color='green',
                 linestyle='dashed', label=label)
        plt.scatter(x_list, y_list, color='purple')
        plt.legend()
        plt.show()