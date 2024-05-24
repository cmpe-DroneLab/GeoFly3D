import haversine as hs
from haversine import Unit
from scan_area import ScanArea
import math
import matplotlib.pyplot as plt
import numpy as np
import copy
from tqdm import tqdm
from shapely.ops import linemerge, unary_union, polygonize
from shapely.geometry import LineString, Polygon, Point
from matplotlib import pyplot as plt
import geopandas as gpd


def est_duration(route_nodes):
    node_count = len(route_nodes)
    len_lines = []
    for i in range(len(route_nodes)-1):
        if i%2 == 0:
            temp_node1 = route_nodes[i]
            temp_node2 = route_nodes[i+1]
            len_lines.append(hs.haversine(temp_node1, temp_node2, unit=Unit.METERS))
    # print(len_lines)
    distance = sum(len_lines)
    # print(node_count, distance)
    est_time = node_count * 4 + (distance/5)
    return est_time



def get_route(coordinates):
    nodes = []
    for coord in coordinates:
        nodes.append((coord[0], coord[1]))
    route_angle = (math.pi * 0 / 180)
    rotated_route_angle = (math.pi * 20 / 180)
    polygon = ScanArea(coords=nodes)
    optimal_route_nodes, rotated_route_nodes = polygon.create_route(40, 0.8, route_angle, rotated_route_angle)

    return(optimal_route_nodes)

def cut_polygon_by_line(polygon, line):
    merged = linemerge([polygon.boundary, line])
    borders = unary_union(merged)
    polygons = polygonize(borders)
    return list(polygons)

def plot(shapely_objects, figure_path='fig.png'):
    boundary = gpd.GeoSeries(shapely_objects)
    boundary.plot(color=['red', 'green', 'blue', 'yellow', 'yellow'])
    plt.show()
    
    
    

def find_midpoint(coordinates,angle_offset,m = 1,rec_quit= False):
    nodes = []
    for coord in coordinates:
        nodes.append((coord[0], coord[1]))
    
    polygon = ScanArea(coords=nodes)
    # plot(polygon.polygon)
    longest = polygon.find_the_longest_edge()
    
    max_len = 0
    max_len_idx = 0
    min_x = 1000
    min_y = 1000
    max_x = -1000
    max_y = -1000
    for  coord in coordinates:
        min_x = coord[0] if coord[0] < min_x else min_x
        max_x = coord[0] if coord[0] > max_x else max_x
        min_y = coord[1] if coord[1] < min_y else min_y
        max_y = coord[1] if coord[1] > max_y else max_y

    if m > 0:
        start_point = [min_x, min_y]
        end_point = [max_x, max_y]
    else:
        start_point = [min_x,max_y]
        end_point = [max_x,min_y]
    m1 = (longest.coords[1][1]-longest.coords[0][1])/(longest.coords[1][0]-longest.coords[0][0])
    angle = math.degrees(math.atan(m1)) + angle_offset
    m1 = math.tan(math.radians(angle))
    while_check = True
    counter = 0
    while while_check:
        counter+=1
        if counter >= 30:
            if rec_quit:
                # plot([polygons[0],polygons[1],check_line])
                return 0
            return find_midpoint(coordinates,angle_offset,-m,True)
        check_line = LineString([start_point,end_point])
        mid_point = ((start_point[0]+end_point[0])/2,(start_point[1]+end_point[1])/2)
        mid_point_extension0 = (mid_point[0]-0.003,mid_point[1]-0.003*m1)
        mid_point_extension1 = (mid_point[0]+0.003,mid_point[1]+m1*0.003)

        mid_line = LineString([[mid_point_extension0[0],mid_point_extension0[1]],[mid_point_extension1[0],mid_point_extension1[1]]])
        polygons = cut_polygon_by_line(polygon.polygon,mid_line)
        if len(polygons) == 1:
            if smaller:
                end_point = mid_point
                smaller = False
            else:
                start_point = mid_point
                smaller = True
            continue

            
        p1 = polygons[0]
        p2 = polygons[1]

        coor1= list(p1.exterior.coords)
        coor2= list(p2.exterior.coords)
        r1 = get_route(coor1)
        r2 = get_route(coor2)
        est1 = est_duration(r1)
        est2 = est_duration(r2)
        if abs(est2 - est1) < 4:
            while_check = False
            print(f"estimateds {angle_offset}: {est1} , {est2}")
            break
        if est2 < est1:
            if p1.centroid.x > p2.centroid.x:
                start_point = mid_point
                smaller = True
            else:
                end_point = mid_point
                smaller = False
        else:
            if p1.centroid.x > p2.centroid.x:
                end_point = mid_point
                smaller = False
            else:
                start_point = mid_point
                smaller = True
        # print(f"estimateds {counter}: {est1} , {est2}")
        # plot([polygons[0],polygons[1],check_line])
    # plot(polygons)
    return polygons
# for i in range(-180,180,1):
#     polygons = find_midpoint(coordinates2,angle_offset = i,m= 1,rec_quit=False)
