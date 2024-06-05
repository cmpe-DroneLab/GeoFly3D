import json
import os
import sys
import folium
import haversine as hs

from haversine import Unit
from datetime import datetime
from http.server import HTTPServer, SimpleHTTPRequestHandler
from PyQt6 import QtWebEngineCore
from PyQt6.QtCore import pyqtSignal, QThread


def draw_route(map_obj, mission_paths=None, mission_boundary=None, gcs_node=None, draw_coverage=False):
    if mission_boundary == 'null' or mission_boundary is None:
        return

    if (draw_coverage is True) and (gcs_node is not None):
        # Create feature group for GCS (Ground Control Station)
        fg_coverage = folium.FeatureGroup(name="Coverage Area")

        # Add marker for GCS
        fg_coverage.add_child(folium.Marker([gcs_node.latitude, gcs_node.longitude], tooltip="GCS Point"))

        # Draw coverage area
        fg_coverage.add_child(
            folium.Circle(
                location=[gcs_node.latitude, gcs_node.longitude],
                radius=2000,
                fill=False,
                fill_opacity=0,
                fill_color="white",
                weight=2,
                opacity=1,
                color="black",
                dash_array='5',
                tooltip="Within Coverage Area"
            )
        )

        # Add GCS feature group to the Map
        map_obj.add_child(fg_coverage)

    if len(mission_paths):
        colors = ["red", "blue", "orange", "purple", "green"]
        color_idx = 0
        for i, path in enumerate(mission_paths):
            path_id = path.path_id
            drone_id = -1
            for drone in path.mission.mission_drones:
                if drone.path_id == path_id:
                    drone_id = drone.drone_id
                    break

            path_boundary = json.loads(path.path_boundary)
            opt_route = json.loads(path.opt_route)
            rot_route = json.loads(path.rot_route)
            color = colors[color_idx % len(colors)]
            color_idx += 1

            # Create feature group for Path Boundary
            fg_path_boundary = folium.FeatureGroup(name="Drone #{} | Scanning Area".format(drone_id))

            # Draw Path Boundary Area
            fg_path_boundary.add_child(folium.Polygon(locations=path_boundary,
                                                      weight=0,
                                                      fill_color=color,
                                                      fill_opacity=0.1,
                                                      fill=True, ))

            # Add Path Boundary feature group to the Map
            map_obj.add_child(fg_path_boundary)

            # Create feature group for Optimal Route
            fg_optimal = folium.FeatureGroup(name="Drone #{} | 1st Route".format(drone_id))

            # Add marker to Start point of Optimal Route
            optimal_start_point = opt_route[0]
            fg_optimal.add_child(folium.Marker(location=[optimal_start_point[0], optimal_start_point[1]],
                                               tooltip="Starting point of 1st Route of Drone #{}".format(drone_id),
                                               icon=folium.Icon(color="darkgreen", icon='play')))

            # Add marker to End point of Optimal Route
            optimal_end_point = opt_route[-1]
            fg_optimal.add_child(folium.Marker(location=[optimal_end_point[0], optimal_end_point[1]],
                                               tooltip="Ending point of 1st Route of Drone #{}".format(drone_id),
                                               icon=folium.Icon(color="darkgreen", icon='stop')))

            # Draw Optimal Route
            fg_optimal.add_child(folium.PolyLine(locations=opt_route,
                                                 weight=2,
                                                 color="darkgreen",
                                                 opacity=0.6))

            # Add Optimal Route feature group to the Map
            map_obj.add_child(fg_optimal)

            # Create feature group for Rotated Route
            fg_rotated = folium.FeatureGroup(name="Drone #{} | 2nd Route".format(drone_id))

            # Add marker to Start point of Rotated Route
            rotated_start_point = rot_route[0]
            fg_rotated.add_child(folium.Marker(location=[rotated_start_point[0], rotated_start_point[1]],
                                               tooltip="Starting point of 2nd Route of Drone #{}".format(drone_id),
                                               icon=folium.Icon(color="darkred", icon='play')))

            # Add marker to End point of Rotated Route
            rotated_end_point = rot_route[-1]
            fg_rotated.add_child(folium.Marker(location=[rotated_end_point[0], rotated_end_point[1]],
                                               tooltip="Ending point of 2nd Route of Drone #{}".format(drone_id),
                                               icon=folium.Icon(color="darkred", icon='stop')))

            # Draw Rotated Route
            fg_rotated.add_child(folium.PolyLine(locations=rot_route,
                                                 weight=2,
                                                 color="darkred",
                                                 opacity=0.6))

            # Add Rotated Route feature group to the Map
            map_obj.add_child(fg_rotated)

    else:
        # Draw Mission Boundary Area
        fg_mission_boundary = folium.FeatureGroup(name="Selected Area")
        fg_mission_boundary.add_child(folium.Polygon(locations=mission_boundary, fill=True))
        map_obj.add_child(fg_mission_boundary)

    # Add Layer Control
    map_obj.add_child(folium.LayerControl())

    # Show Map
    if (draw_coverage is True) and (gcs_node is not None):
        gcs = [gcs_node.latitude, gcs_node.longitude]
        mission_boundary.append(gcs)
    sw_point, ne_point = calculate_sw_ne_points(mission_boundary)
    map_obj.fit_bounds([sw_point, ne_point])


class WebEnginePage(QtWebEngineCore.QWebEnginePage):
    polygon_coords_printed = pyqtSignal(list)
    point_coords_printed = pyqtSignal(list)
    drawings_deleted = pyqtSignal()

    def javaScriptConsoleMessage(self, level, msg, line, sourceID):
        # Check if the message is a non-empty string
        if msg and isinstance(msg, str):
            try:
                # Attempt to parse the message as JSON
                coords_dict = json.loads(msg)

                # Check if the JSON structure contains the expected keys
                if 'geometry' in coords_dict:
                    geometry_type = coords_dict['geometry']['type']

                    if geometry_type == 'Polygon':
                        # Extract coordinates from the parsed JSON for Polygon
                        coords = coords_dict['geometry']['coordinates'][0]
                        # Emit the coordinates signal for Polygon
                        self.polygon_coords_printed.emit(coords)

                    elif geometry_type == 'Point':
                        # Extract coordinates from the parsed JSON for Point
                        coords = coords_dict['geometry']['coordinates']
                        # Emit the coordinates signal for Point
                        self.point_coords_printed.emit(coords)

                    else:
                        # Print an error message if the geometry type is not supported
                        print(f"Unsupported geometry type: {geometry_type}")

                elif msg == "drawings deleted":
                    # Emit the drawings deleted signal
                    self.drawings_deleted.emit()

                else:
                    # Print an error message if the 'geometry' key is not found
                    print("Invalid JSON structure: 'geometry' key not found.")

            except json.JSONDecodeError as e:
                # Print an error message if JSON decoding fails
                # print("Error decoding JSON:", e)
                # print("Message:", msg)
                pass

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
        if self.path == f'mission_0_drones.geojson':
            # Load your GeoJSON file here
            filename = f"./UI/LiveData/mission_0_drones.geojson"
            with open(filename, 'r') as f:
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


def update_drone_position_on_map(latitude, longitude, mission_id, drone_id):
    # Read the GeoJSON file
    filename = f"./UI/LiveData/mission_{mission_id}_drones.geojson"
    with open(filename, 'r') as fr:
        data = json.load(fr)
    fr.close()
    # Update the coordinates
    for i, feature in enumerate(data['features']):
        if feature['properties']['drone_id'] == int(drone_id):
            data['features'][i]['geometry']['coordinates'] = [longitude, latitude]
            break
    # Write back the modified data
    with open(filename, 'w') as fw:
        json.dump(data, fw)
    fw.close()


def update_drone_battery(battery_percent, mission_id, drone_id):
    # Read the GeoJSON file
    filename = f"./UI/LiveData/mission_{mission_id}_drones.geojson"
    with open(filename, 'r') as fr:
        data = json.load(fr)
    fr.close()
    # Update the battery
    for i, feature in enumerate(data['features']):
        if feature['properties']['drone_id'] == int(drone_id):
            data['features'][i]['properties']['battery'] = battery_percent
            break
    # Write back the modified data
    with open(filename, 'w') as fw:
        json.dump(data, fw)
    fw.close()


def update_drone_status(status: str, mission_id, drone_id):
    # Read the GeoJSON file
    filename = f"./UI/LiveData/mission_{mission_id}_drones.geojson"
    with open(filename, 'r') as fr:
        data = json.load(fr)
    fr.close()
    # Update the status
    for i, feature in enumerate(data['features']):
        if feature['properties']['drone_id'] == int(drone_id):
            data['features'][i]['properties']['status'] = status
            break
    # Write back the modified data
    with open(filename, 'w') as fw:
        json.dump(data, fw)
    fw.close()


def calculate_sw_ne_points(coords):
    if len(coords):
        lats = [c[0] for c in coords]
        lons = [c[1] for c in coords]

        sw_point = (min(lats), min(lons))
        ne_point = (max(lats), max(lons))

        return sw_point, ne_point
    return None, None


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


def get_current_time():
    current_time = datetime.now()
    time_format = "%H:%M:%S %d/%m/%y"
    return current_time.strftime(time_format)


def calculate_geographic_distance(first, second):
    return hs.haversine((first[0], first[1]), (second[0], second[1]), unit=Unit.METERS)
