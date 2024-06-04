import json
import folium
import UI.Midflight.mid_design
import math
from folium import JsCode
from folium.plugins import MousePosition, Realtime
from PyQt6.QtCore import QSize, QDateTime, QTimer, pyqtSignal, QMutex
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import QWidget, QListWidgetItem, QLabel, QMessageBox
from UI.database import get_mission_by_id, get_drone_by_id
from UI.ListItems.drone_mid import Ui_Form
from UI.helpers import WebEnginePage, draw_route, update_drone_position_on_map, update_drone_battery, update_drone_status, calculate_geographic_distance
from drone_controller import DroneController
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

CRITICAL_BATTERY_LEVEL = 15
WARNING_DISTANCE_THRESHOLD = 3     # in meters


class Mid(QWidget):
    emergency_rth_clicked = pyqtSignal(int)
    emergency_land_clicked = pyqtSignal(int)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Midflight.mid_design.Ui_Form()
        self.ui.setupUi(self)

        self.mission = None
        self.map = None
        self.webView = QWebEngineView()
        self.ui.v_lay_right.addWidget(self.webView)

        self.has_taken_off = False
        self.mission_threads = {}
        self.timer = QTimer(self)

        self.total_length = 0
        self.stop_progress = {}
        self.progress_mutex = QMutex()
        self.live_data_mutex = QMutex()

        self.ui.btn_land.setVisible(False)
        self.ui.btn_return_to_home.setVisible(False)

        self.battery_popup_permission = {}
        self.area_popup_permission = {}

    # Loads mission information from database into relevant fields
    def load_mission(self, mission_id):
        if mission_id == 0:
            exit(-1)
        else:
            self.mission = get_mission_by_id(mission_id)

        # Set mission information box
        self.ui.selected_area_value.setText(str(self.mission.selected_area))
        self.ui.estimated_mission_time_value.setText(str(self.mission.estimated_mission_time))
        self.ui.progress_bar.setValue(0)

        # Load drones
        self.refresh_drone_list()
        self.create_geojson()

        # Set mission id in the header box
        self.ui.gb_mission.setTitle("Mission # " + str(self.mission.mission_id))

        # Set up the Map
        self.setup_map(self.mission.center_node.latitude, self.mission.center_node.longitude, 10)

        # Draw route
        self.update_map()

        # Start timer for update elapsed time
        self.ui.elapsed_time_value.setText("")
        self.timer.timeout.connect(self.update_elapsed_time)
        self.timer.start(1000)  # Update every second

        # Initialize alerts and progress for each drone
        for drone in self.mission.mission_drones:
            self.stop_progress = {drone.drone_id: False}
            self.battery_popup_permission = {drone.drone_id: True}
            self.area_popup_permission = {drone.drone_id: None}


        # Calculate total length of the mission paths
        self.calculate_total_length()

    # Gets all matching drones from the database, adds them to the Drone List
    def refresh_drone_list(self):
        self.ui.listWidget.clear()
        for drone in self.mission.mission_drones:
            self.add_drone_to_list(drone)

    # Adds given drone to the Drone List
    def add_drone_to_list(self, drone):
        new_drone_item = QListWidgetItem()
        new_drone_widget = QWidget()
        new_drone_ui = Ui_Form()
        new_drone_ui.setupUi(new_drone_widget)
        new_drone_ui.id_text.setText(str(drone.drone_id))
        new_drone_ui.model_text.setText(drone.model)
        new_drone_ui.battery_text.setText("Waiting for live data...")
        new_drone_ui.status_text.setText("Waiting for live data...")

        # Calculate the height of the new_drone_widget
        new_drone_widget.adjustSize()
        widget_height = new_drone_widget.sizeHint().height()

        # Set the size hint for the item
        new_drone_item.setSizeHint(QSize(self.ui.listWidget.lineWidth(), widget_height))

        self.ui.listWidget.addItem(new_drone_item)
        self.ui.listWidget.setItemWidget(new_drone_item, new_drone_widget)

    # Creates a map given center point and zoom level
    def setup_map(self, lat, lon, zoom):
        self.map = folium.Map(location=[lat, lon], zoom_start=zoom, control_scale=True, )
        self.map.add_child(MousePosition(position="topright", separator=" | ", empty_string="NaN", lng_first=False, ))
        self.add_drone_markers()
        self.save_map()

    # Saves and shows the map
    def save_map(self):
        self.map.save('./UI/Midflight/mid_map.html')
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(open('./UI/Midflight/mid_map.html').read())
        self.webView.show()

    def update_map(self):
        if self.mission:
            draw_route(
                map_obj=self.map,
                mission_paths=self.mission.mission_paths,
                mission_boundary=json.loads(self.mission.mission_boundary),
                gcs_node=self.mission.gcs_node,
                draw_coverage=True
            )
            self.save_map()

    def create_geojson(self):
        # Create an empty GeoJSON structure
        geojson_data = {
            "type": "FeatureCollection",
            "features": []
        }

        # Add each drone as a feature to the GeoJSON
        for drone in self.mission.mission_drones:
            feature = {
                "type": "Feature",
                "properties": {
                    "objectid": str(drone.drone_id),
                    "model": drone.model,
                    "drone_id": drone.drone_id,
                    "ip_address": drone.ip_address,
                    "battery": None,
                    "status": None,
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": None
                }
            }
            geojson_data["features"].append(feature)

        # Convert the GeoJSON data to a string
        geojson_str = json.dumps(geojson_data, indent=2)

        # Write the GeoJSON string to a file with mission_id in the filename
        filename = f"./UI/LiveData/mission_{self.mission.mission_id}_drones.geojson"
        with open(filename, "w") as geojson_file:
            geojson_file.write(geojson_str)

    def add_drone_markers(self):
        source = f"http://localhost:9000/UI/LiveData/mission_{self.mission.mission_id}_drones.geojson"
        Realtime(
            source,
            get_feature_id=JsCode("(f) => { return f.properties.objectid }"),
            point_to_layer=JsCode("""
                        (f, latlng) => { 
                            return L.marker(latlng, {
                                icon: L.icon({
                                    iconUrl: 'http://localhost:9000/UI/Images/drone.gif', 
                                    iconSize:[50, 48], 
                                    iconAnchor:[25, 24]
                                })
                            }).bindPopup(
                                '<h5>DRONE #' + f.properties.drone_id + '</h5>' + 
                                 f.properties.model
                            ).openPopup();
                        }
                    """),
            interval=2000,
        ).add_to(self.map)

    def update_live_data(self):
        for row in range(self.ui.listWidget.count()):
            item = self.ui.listWidget.item(row)
            drone_id = item.listWidget().itemWidget(item).findChild(QLabel, "id_text").text()
            battery_field = item.listWidget().itemWidget(item).findChild(QLabel, "battery_text")
            status_field = item.listWidget().itemWidget(item).findChild(QLabel, "status_text")

            # Read the GeoJSON file
            filename = f"./UI/LiveData/mission_{self.mission.mission_id}_drones.geojson"
            with open(filename, 'r') as fr:
                data = json.load(fr)
            fr.close()
            for feature in data['features']:
                if feature['properties']['drone_id'] == int(drone_id):
                    battery_level = feature['properties'].get('battery')
                    status = feature['properties'].get('status')

                    if battery_level is not None:
                        battery_field.setText(str(int(feature['properties'].get('battery'))))
                        if int(battery_level) <= CRITICAL_BATTERY_LEVEL:
                            self.emergency_alarm("battery", int(drone_id), f"Drone {drone_id} battery level is critical!")
                        elif int(battery_level) > CRITICAL_BATTERY_LEVEL:
                            self.battery_popup_permission[int(drone_id)] = True

                    if status is not None:
                        status_field.setText(feature['properties'].get('status'))

    def update_elapsed_time(self):
        current_time = QDateTime.currentDateTime()
        flight_start_time = QDateTime.fromString(self.mission.flight_start_time)
        elapsed_seconds = flight_start_time.secsTo(current_time)
        hours = elapsed_seconds // 3600
        minutes = (elapsed_seconds % 3600) // 60
        seconds = elapsed_seconds % 60
        elapsed_time_string = "{:02d}:{:02d}:{:02d}".format(hours, minutes, seconds)
        self.ui.elapsed_time_value.setText(elapsed_time_string)

    def calculate_total_length(self):
        self.total_length = 0
        for path in self.mission.mission_paths:
            self.total_length += path.opt_route_length
            self.total_length += path.rot_route_length
        self.ui.progress_value.setText(f"0m / {int(self.total_length)}m")

    def update_progress_bar(self, increment, drone_id):
        self.progress_mutex.lock()
        try:
            path = get_drone_by_id(drone_id).path
            path.set_increment(increment)

            total_actual_length = sum(path.actual_flown + path.increment for path in self.mission.mission_paths)

            progress = math.ceil(total_actual_length / self.total_length * 100)
            self.ui.progress_bar.setValue(progress)
            self.ui.progress_value.setText(f"{int(total_actual_length)}m / {int(self.total_length)}m")
        finally:
            self.progress_mutex.unlock()

    def drone_position_changed(self, lat, lon, drone_id):
        self.live_data_mutex.lock()
        try:
            update_drone_position_on_map(lat, lon, self.mission.mission_id, drone_id)
            drone = get_drone_by_id(drone_id)

            # Check if the drone is within the path boundary
            path_boundary = json.loads(drone.path.path_boundary)
            point = Point(lat, lon)
            polygon = Polygon(path_boundary)

            nearest_point = nearest_points(polygon, point)[0]
            distance = calculate_geographic_distance((nearest_point.x, nearest_point.y), (point.x, point.y))

            if distance >= WARNING_DISTANCE_THRESHOLD and self.area_popup_permission[drone_id] is True:
                self.emergency_alarm("area", int(drone_id), f"Drone {drone_id} is outside the scanning area more than {distance:.2f} meters!")
                self.area_popup_permission[drone_id] = False
            if distance < WARNING_DISTANCE_THRESHOLD:
                self.area_popup_permission[drone_id] = True

            last_node_lat = drone.path.last_visited_node.latitude
            last_node_lon = drone.path.last_visited_node.longitude
            if drone.path.last_visited_node.latitude != 500 and self.stop_progress[drone_id] is False:
                length_increment = calculate_geographic_distance((lat, lon), (last_node_lat, last_node_lon))
                self.update_progress_bar(length_increment, drone_id)
        finally:
            self.live_data_mutex.unlock()

    def last_visited_node_changed(self, coords, drone_id):
        lat, lon = coords
        path = get_drone_by_id(drone_id).path

        last_node_lat = path.last_visited_node.latitude
        last_node_lon = path.last_visited_node.longitude
        if last_node_lat != 500:
            if self.stop_progress[drone_id] is False:
                distance = calculate_geographic_distance((last_node_lat, last_node_lon), (lat, lon))

                actual = path.actual_flown
                path.set_actual_flown(actual + distance)
                self.update_progress_bar(0, drone_id)

                if path.actual_flown == path.opt_route_length:
                    self.stop_progress[drone_id] = True
                elif path.actual_flown == self.total_length:
                    self.stop_progress[drone_id] = True
            else:
                self.stop_progress[drone_id] = False
        else:
            self.stop_progress[drone_id] = False

        path.last_visited_node.latitude = lat
        path.last_visited_node.longitude = lon

    def emergency_alarm(self, alarm_type, drone_id, message):
        if alarm_type == "area":
            if self.area_popup_permission[drone_id] is not True:
                return
            self.area_popup_permission[drone_id] = False

        elif alarm_type == "battery":
            if self.battery_popup_permission[drone_id] is not True:
                return
            self.battery_popup_permission[drone_id] = False

        def show_message_box():
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Icon.Critical)
            msg.setText(message)
            msg.setWindowTitle("Drone Alert")
            land_button = msg.addButton("Land the Drone", QMessageBox.ButtonRole.ActionRole)
            rth_button = msg.addButton("Return to Home (RTH)", QMessageBox.ButtonRole.ActionRole)
            cancel_button = msg.addButton(QMessageBox.StandardButton.Cancel)
            msg.exec()

            if msg.clickedButton() == land_button:
                self.emergency_land_clicked.emit(drone_id)
            elif msg.clickedButton() == rth_button:
                self.emergency_rth_clicked.emit(drone_id)

        QTimer.singleShot(0, show_message_box)

    def take_off(self, vertices, flight_altitude, mission_id, gimbal_angle, route_angle, rotated_route_angle):
        if mission_id in self.mission_threads:
            return

        drone_controller_thread = DroneController(vertices=vertices, mission_id=mission_id, flight_altitude=flight_altitude,
                                                  gimbal_angle=gimbal_angle, route_angle=route_angle,
                                                  rotated_route_angle=rotated_route_angle)

        drone_controller_thread.started.connect(print)
        drone_controller_thread.progress_text.connect(print)
        drone_controller_thread.progress.connect(self.last_visited_node_changed)
        drone_controller_thread.update_coord.connect(self.drone_position_changed)
        drone_controller_thread.update_coord.connect(self.update_live_data)
        drone_controller_thread.update_status.connect(update_drone_status)
        drone_controller_thread.update_status.connect(self.update_live_data)
        drone_controller_thread.update_battery.connect(update_drone_battery)
        drone_controller_thread.update_battery.connect(self.update_live_data)

        self.mission_threads[mission_id] = drone_controller_thread
        # drone_controller_thread.start()
        self.has_taken_off = True
        mission = get_mission_by_id(mission_id)
        mission.set_status("Mid Flight")
        mission.set_flight_start_time(QDateTime.currentDateTime().toString())
        return drone_controller_thread
