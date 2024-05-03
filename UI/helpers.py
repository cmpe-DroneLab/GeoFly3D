import json
import os
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler

import folium
from PyQt6 import QtWebEngineCore
from PyQt6.QtCore import pyqtSignal, QThread

from RoutePlanner import route_planner


class RouteDrawer:
    @staticmethod
    def draw_route(map_obj, mission):
        if mission.coordinates == 'null' or mission.coordinates is None:
            return

        coords = json.loads(mission.coordinates)
        coords_lon_lat = invert_coordinates(coords)

        # Create feature group for Selected Area
        fg_selected = folium.FeatureGroup(name="Selected Area")

        # Draw Selected Area
        fg_selected.add_child(folium.Polygon(locations=coords,
                                             weight=0,
                                             fill_color="red",
                                             fill_opacity=0.1,
                                             fill=True, ))

        # Add Selected Area feature group to the Map
        map_obj.add_child(fg_selected)

        # Calculate Optimal and Rotated Route
        (optimal_route, rotated_route) = route_planner.plan_route(coords=coords_lon_lat[:-1],
                                                                  altitude=mission.altitude,
                                                                  intersection_ratio=0.8,
                                                                  route_angle_deg=mission.route_angle,
                                                                  rotated_route_angle_deg=mission.rotated_route_angle)

        # Create feature group for Optimal Route
        fg_optimal = folium.FeatureGroup(name="Optimal Route")

        # Add marker to Start point of Optimal Route
        optimal_start_point = optimal_route[0]
        fg_optimal.add_child(folium.Marker(location=[optimal_start_point[0], optimal_start_point[1]],
                                           tooltip="Starting point of Optimal Route",
                                           icon=folium.Icon(color="darkgreen", icon='play')))

        # Add marker to End point of Optimal Route
        optimal_end_point = optimal_route[-1]
        fg_optimal.add_child(folium.Marker(location=[optimal_end_point[0], optimal_end_point[1]],
                                           tooltip="Ending point of Optimal Route",
                                           icon=folium.Icon(color="darkgreen", icon='stop')))

        # Draw Optimal Route
        fg_optimal.add_child(folium.PolyLine(locations=optimal_route,
                                             weight=2,
                                             color="darkgreen",
                                             opacity=0.6))

        # Add Optimal Route feature group to the Map
        map_obj.add_child(fg_optimal)

        # Create feature group for Rotated Route
        fg_rotated = folium.FeatureGroup(name="Rotated Route")

        # Add marker to Start point of Rotated Route
        rotated_start_point = rotated_route[0]
        fg_rotated.add_child(folium.Marker(location=[rotated_start_point[0], rotated_start_point[1]],
                                           tooltip="Starting point of Rotated Route",
                                           icon=folium.Icon(color="darkred", icon='play')))

        # Add marker to End point of Rotated Route
        rotated_end_point = rotated_route[-1]
        fg_rotated.add_child(folium.Marker(location=[rotated_end_point[0], rotated_end_point[1]],
                                           tooltip="Ending point of Rotated Route",
                                           icon=folium.Icon(color="darkred", icon='stop')))

        # Draw Rotated Route
        fg_rotated.add_child(folium.PolyLine(locations=rotated_route,
                                             weight=2,
                                             color="darkred",
                                             opacity=0.6))

        # Add Rotated Route feature group to the Map
        map_obj.add_child(fg_rotated)

        # Add Layer Control
        map_obj.add_child(folium.LayerControl())

        # Save and Show Map
        sw_point, ne_point = calculate_sw_ne_points(coords)
        map_obj.fit_bounds([sw_point, ne_point])


class WebEnginePage(QtWebEngineCore.QWebEnginePage):
    coords_printed = pyqtSignal(list)  # Signal emitted when coordinates are printed
    shapes_deleted = pyqtSignal()  # Signal emitted when shapes are deleted

    def javaScriptConsoleMessage(self, level, msg, line, sourceID):
        # Check if the message is a non-empty string
        if msg and isinstance(msg, str):
            try:

                # Check if the message is "drawings deleted"
                if msg == "drawings deleted":
                    # Emit the shapes deleted signal
                    self.shapes_deleted.emit()

                # Attempt to parse the message as JSON
                else:
                    coords_dict = json.loads(msg)
                    # Check if the JSON structure contains the expected keys
                    if 'geometry' in coords_dict and 'coordinates' in coords_dict['geometry']:
                        # Extract coordinates from the parsed JSON
                        coords = coords_dict['geometry']['coordinates'][0]
                        # Emit the coordinates signal with the extracted coordinates
                        self.coords_printed.emit(coords)
                    else:
                        # Print an error message if the expected keys are not found
                        print("Invalid JSON structure: 'geometry' or 'coordinates' key not found.")
            except json.JSONDecodeError as e:
                # Print an error message if JSON decoding fails
                print("Error decoding JSON:", e)
                print(msg)
        else:
            # Print an error message for invalid messages
            print("Invalid message:", msg)


class CORSRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        SimpleHTTPRequestHandler.end_headers(self)

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        if self.path == '/drone_realtime.geojson':
            # Load your GeoJSON file here
            with open('UI/Midflight/drone_realtime.geojson', 'r') as f:
                data = f.read()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(data.encode())
        else:
            # Serve other files as usual
            super().do_GET()


class ServerThread(QThread):
    def __init__(self):
        super().__init__()
        self.httpd = None

    def run(self):
        self.start_server()

    def terminate(self):
        self.terminate_server()

    def start_server(self, server_class=HTTPServer, handler_class=CORSRequestHandler, port=9000):
        server_address = ('', port)
        self.httpd = server_class(server_address, handler_class)
        print(f'Starting server on port {port}...')
        self.httpd.serve_forever()

    def terminate_server(self):
        if self.httpd:
            print('Shutting down server...')
            self.httpd.shutdown()  # Shutdown the server
            self.httpd.server_close()


def update_drone_position_on_map(latitude, longitude):
    # Read the GeoJSON file
    with open(resource_path('rt_drone_info.geojson'), 'r') as fr:
        data = json.load(fr)

    # Update the coordinates
    data['features'][0]['geometry']['coordinates'] = [longitude, latitude]

    # Write back the modified data
    with open(resource_path('rt_drone_info.geojson'), 'w') as fw:
        json.dump(data, fw)


# Calculates SW and NE points given coordinates
def calculate_sw_ne_points(coords):
    if len(coords):
        lats = [c[0] for c in coords]
        lons = [c[1] for c in coords]

        sw_point = (min(lats), min(lons))
        ne_point = (max(lats), max(lons))

        return sw_point, ne_point
    return None, None


# Calculates center point coordinates of given coordinates
def calculate_center_point(coords):
    if not coords:
        return None

    total_lat = 0
    total_lon = 0
    num_coords = len(coords)

    # Iterate over all coordinates to calculate the total lat and lon values
    for lat, lon in coords:
        total_lat += lat
        total_lon += lon

    # Calculate the average point by dividing the total lat and lon values by the number of coordinates
    center_lat = total_lat / num_coords
    center_lon = total_lon / num_coords

    return center_lat, center_lon


# Changes positions of longitudes and latitudes
def invert_coordinates(coordinates):
    inverted_coordinates = []
    for coord_pair in coordinates:
        inverted_coord = [coord_pair[1], coord_pair[0]]
        inverted_coordinates.append(inverted_coord)
    return inverted_coordinates


def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath("./UI/Midflight")

    return os.path.join(base_path, relative_path)