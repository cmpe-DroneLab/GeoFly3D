import json
import math

import folium
from PyQt6.QtCore import QSize
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import QDialog, QListWidgetItem, QWidget, QLabel

from UI import draw
from UI.database import Drone, session, Mission
from UI.drone_dialog import Ui_drone_dialog
from UI.drone import Ui_Form
import UI.Preflight2.pre2_design

from UI.web_engine_page import WebEnginePage


class Pre2(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Preflight2.pre2_design.Ui_Form()
        self.ui.setupUi(self)

        self.mission = None
        self.mission_id = None
        self.mission_drones = []
        self.coords = None
        self.coords_lon_lat = None
        self.map2 = None
        self.webView = QWebEngineView()
        self.setup_map(39, 35, 5)
        self.ui.v_lay_right.addWidget(self.webView)

        self.ui.slider_altitude.valueChanged.connect(self.slider_altitude_changed)
        self.ui.spinbox_altitude.valueChanged.connect(self.spinbox_altitude_changed)
        self.ui.btn_add_drone.clicked.connect(self.create_drone)
        self.ui.btn_delete_drone.clicked.connect(self.delete_drone)
        self.ui.listWidget.itemSelectionChanged.connect(self.enable_delete_button)
        self.ui.btn_save.clicked.connect(self.save_mission)

    # Loads mission information from database into relevant fields
    def load_mission(self, mission_id):
        self.mission_id = mission_id
        self.mission = session.query(Mission).filter_by(mission_id=mission_id).first()

        # Set mission id in the header box
        self.ui.id_label.setText(str(self.mission.mission_id))

        # Set mission information box
        self.ui.selected_area_value.setText(str(self.mission.selected_area))
        self.ui.mission_time_value.setText(str(self.mission.estimated_mission_time))
        self.ui.batt_required_value.setText(str(self.mission.required_battery_capacity))

        # Load drones
        self.refresh_drone_list()
        self.calculate_provided_capacity()

        # Set altitude box values
        self.ui.slider_altitude.setValue(self.mission.altitude)
        self.ui.spinbox_altitude.setValue(self.mission.altitude)

        # Set up the Map
        self.setup_map(self.mission.center_lat, self.mission.center_lon, 10)

        # Draw previously selected area if available in the database
        if self.mission.coordinates is not None:
            self.coords = json.loads(self.mission.coordinates)
            self.draw_polygon(self.coords)

    # Gets all matching drones from the database, adds them to the Drone List
    def refresh_drone_list(self):
        self.ui.listWidget.clear()

        # Find all drones matching the Mission
        drones = session.query(Drone).filter_by(mission_id=self.mission_id).all()
        for drone in drones:
            self.add_drone_to_list(drone)

        # Disable Delete Drone button
        self.disable_delete_button()

        # Update drone related metrics
        self.update_drone_metrics()

    # Creates a Drone
    def create_drone(self):
        # Open add drone dialog
        dialog_win = QDialog()
        dialog_ui = Ui_drone_dialog()
        dialog_ui.setupUi(dialog_win)
        dialog_win.setWindowTitle("Add Drone")
        dialog_ui.btn_save.setText("Add")
        dialog_ui.spin_spare_batt.setValue(0)
        response = dialog_win.exec()

        if response == 1:
            # Get values from dialog
            new_drone_model = dialog_ui.combo_model.currentText()
            new_drone_spare = dialog_ui.spin_spare_batt.value()

            draft_drone = Drone(model=new_drone_model,
                                battery_no=new_drone_spare,
                                flight_status=None,
                                gps_status=False,
                                connection_status=False,
                                mission_id=self.mission_id)
            session.add(draft_drone)
            session.commit()

            # Refresh the Drone List
            self.refresh_drone_list()

    # Adds given drone to the Drone List
    def add_drone_to_list(self, drone):
        new_drone_item = QListWidgetItem()
        new_drone_item.setSizeHint(QSize(self.ui.listWidget.lineWidth(), 80))
        new_drone_widget = QWidget()
        new_drone_ui = Ui_Form()
        new_drone_ui.setupUi(new_drone_widget)
        new_drone_ui.id_text.setText(str(drone.drone_id))
        new_drone_ui.model_text.setText(drone.model)
        new_drone_ui.spare_batt_text.setText(str(drone.battery_no))
        self.ui.listWidget.addItem(new_drone_item)
        self.ui.listWidget.setItemWidget(new_drone_item, new_drone_widget)

    # Deletes the drone selected from the list from the database
    def delete_drone(self):
        selected_items = self.ui.listWidget.selectedItems()

        # Find selected drone(s)
        for item in selected_items:
            # Get the widget associated with the item
            drone_widget = self.ui.listWidget.itemWidget(item)
            # Find the QLabel that holds the drone id
            id_label = drone_widget.findChild(QLabel, "id_text")
            # Get the drone id
            drone_id = int(id_label.text())
            # Query the database for the drone with the given id
            drone = session.query(Drone).filter_by(drone_id=drone_id).first()

            # Delete the drone from the database and refresh the Drone List
            if drone:
                session.delete(drone)
                session.commit()

        # Refresh the Drone List
        self.refresh_drone_list()

    # Enables Delete Drone button
    def enable_delete_button(self):
        if len(self.ui.listWidget.selectedIndexes()):
            self.ui.btn_delete_drone.setEnabled(True)

    # Disables Delete Drone button
    def disable_delete_button(self):
        self.ui.btn_delete_drone.setEnabled(False)

    # Saves mission to the database
    def save_mission(self):
        self.mission.coordinates = json.dumps(self.coords)
        self.mission.estimated_mission_time = int(self.ui.mission_time_value.text())
        self.mission.required_battery_capacity = int(self.ui.batt_required_value.text())
        self.mission.selected_area = int(self.ui.selected_area_value.text())
        self.mission.altitude = self.ui.spinbox_altitude.value()
        session.commit()

    # Creates a map given center point and zoom level
    def setup_map(self, lat, lon, zoom):

        if lat is None:
            lat = 41.0859528
            lon = 29.0443435

        self.map2 = folium.Map(location=[lat, lon],
                               zoom_start=zoom,
                               control_scale=True, )

        drw = draw.Draw(export=True,
                        show_geometry_on_click=False,
                        filename='my_data.geojson',
                        draw_options={'polyline': False,
                                      'polygone': True,
                                      'rectangle': False,
                                      'circle': False,
                                      'marker': False,
                                      'circlemarker': False})
        self.map2.add_child(drw)
        self.save_map()

    # Saves and shows the map
    def save_map(self):
        self.map2.save('./UI/Preflight2/pre2map.html')
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(open('./UI/Preflight2/pre2map.html').read())
        self.webView.show()

        # Listen for any drawings on the Map
        page.coords_printed.connect(self.selected_area_changed)

    # Draws a polygon with given coordinates
    def draw_polygon(self, coords):
        # Draw polygon and add to the map
        fg = folium.FeatureGroup(name="ScanArea")
        fg.add_child(folium.Polygon(coords))
        self.map2.add_child(fg)

        # Set the map to show the polygon
        sw_point, ne_point = self.calculate_sw_ne_points(coords)
        self.map2.fit_bounds([sw_point, ne_point])
        self.save_map()

    # Captures changes in the altitude slider and makes necessary updates
    def slider_altitude_changed(self):
        value = self.ui.slider_altitude.value()
        self.ui.spinbox_altitude.setValue(value)
        self.update_altitude_metrics()

    # Captures changes in the altitude spinbox and makes necessary updates
    def spinbox_altitude_changed(self):
        value = self.ui.spinbox_altitude.value()
        self.ui.slider_altitude.setValue(value)
        self.update_altitude_metrics()

    # Captures changes in the selected area and makes necessary updates
    def selected_area_changed(self, coords_lon_lat):
        self.coords_lon_lat = coords_lon_lat
        self.coords = self.invert_coordinates(coords_lon_lat)
        self.update_area_metrics()

    # Updates drone related metrics
    def update_drone_metrics(self):
        self.calculate_mission_time()
        self.calculate_provided_capacity()

    # Updates altitude related metrics
    def update_altitude_metrics(self):
        self.calculate_mission_time()
        self.calculate_required_capacity()

    # Updates selected area related metrics
    def update_area_metrics(self):

        # Update the label_area text with the coordinates
        area = self.calculate_selected_area()
        self.ui.selected_area_value.setText(f"{area:.0f}")

        self.calculate_required_capacity()
        self.calculate_mission_time()

    # Calculates selected area from coordinates
    def calculate_selected_area(self):
        area = 0.0
        if len(self.coords) > 2:
            for i in range(len(self.coords) - 1):
                p1 = self.coords[i]
                p2 = self.coords[i + 1]
                area += math.radians(p2[0] - p1[0]) * (
                        2 + math.sin(math.radians(p1[1])) + math.sin(math.radians(p2[1])))
            area = area * 6378137.0 * 6378137.0 / 2.0
        return abs(area)

    # Calculates mission time
    def calculate_mission_time(self):
        num_of_drones = self.ui.listWidget.count()
        required_capacity = self.ui.batt_required_value.text()
        if len(required_capacity) and num_of_drones:
            mission_time = int(required_capacity) / num_of_drones
            self.ui.mission_time_value.setText(f"{mission_time:.0f}")
        else:
            self.ui.mission_time_value.setText('0')

    # Calculates required battery capacity for the mission
    def calculate_required_capacity(self):
        altitude = self.ui.slider_altitude.value()
        selected_area = int(self.ui.selected_area_value.text())
        required_capacity = selected_area / altitude ** 2
        self.ui.batt_required_value.setText(f"{required_capacity:.0f}")

    # Calculates provided battery capacity from drones in the Drone List
    def calculate_provided_capacity(self):
        num_of_drones = self.ui.listWidget.count()
        battery_count = num_of_drones
        if num_of_drones:
            for i in range(num_of_drones):
                drone_item = self.ui.listWidget.item(i)
                drone_widget = self.ui.listWidget.itemWidget(drone_item)
                spare_batt_field = drone_widget.findChild(QLabel, "spare_batt_text")
                spare_battery_count = int(spare_batt_field.text())
                battery_count += spare_battery_count
            self.ui.batt_provided_value.setText(str(battery_count * 15))
        else:
            self.ui.batt_provided_value.setText('0')

    # Calculates SW and NE points given coordinates
    def calculate_sw_ne_points(self, coords):
        if len(coords):
            lats = [c[0] for c in coords]
            lons = [c[1] for c in coords]

            min_lat = min(lats)
            min_lon = min(lons)
            max_lat = max(lats)
            max_lon = max(lons)

            sw_point = (min_lat, min_lon)
            ne_point = (max_lat, max_lon)

            return sw_point, ne_point
        return None, None

    def invert_coordinates(self, coordinates):
        inverted_coordinates = []
        for coord_pair in coordinates:
            inverted_coord = [coord_pair[1], coord_pair[0]]
            inverted_coordinates.append(inverted_coord)
        return inverted_coordinates
