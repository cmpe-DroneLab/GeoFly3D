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

        self.mission_id = None
        self.mission = None

        self.webView = QWebEngineView()
        self.load_map(48.88, 2.37, 5)

        self.ui.slider_altitude.valueChanged.connect(self.slider_altitude_changed)
        self.ui.spinbox_altitude.valueChanged.connect(self.spinbox_altitude_changed)
        self.ui.btn_add_drone.clicked.connect(self.add_drone)
        self.ui.btn_delete_drone.clicked.connect(self.delete_drone)
        self.ui.btn_save.clicked.connect(self.save_mission)



    def load_mission(self, mission_id):
        self.mission_id = mission_id
        self.mission = session.query(Mission).filter_by(mission_id=mission_id).first()

        # set mission header box
        self.ui.id_label.setText(str(self.mission.mission_id))

        # set altitude box
        self.ui.slider_altitude.setValue(self.mission.altitude)
        self.ui.spinbox_altitude.setValue(self.mission.altitude)

        # set mission information box
        self.ui.mission_time_value.setText(str(self.mission.estimated_mission_time))
        self.ui.selected_area_value.setText(str(self.mission.selected_area))
        self.ui.batt_required_value.setText(str(self.mission.required_battery_capacity))
        self.ui.batt_provided_value.setText("to do")

        # load drones
        self.load_drones()

        # set map
        self.load_map(self.mission.center_lat, self.mission.center_lon, 15)

        # print coordinates
        if self.mission.coordinates is not None:
            self.coords = json.loads(self.mission.coordinates)
            self.draw_polygon(self.coords)
        else:
            # Handle the case where coordinates are None
            self.coords = []  # Or any other default value you want to assign


    def load_drones(self):
        # Görev ID'sine göre dronları yükle
        drones = session.query(Drone).filter_by(mission_id=self.mission_id).all()
        self.ui.listWidget.clear()
        for drone in drones:
            self.add_drone_to_list(drone)

    def save_mission(self):
        self.mission.altitude = self.ui.spinbox_altitude.value()
        self.mission.estimated_mission_time = int(self.ui.mission_time_value.text())
        self.mission.selected_area = int(self.ui.selected_area_value.text())
        self.mission.required_battery_capacity = int(self.ui.batt_required_value.text())
        self.mission.coordinates = json.dumps(self.coords)

    def load_map(self, lat, lon, zoom):

        if lat is None:
            lon = 2.37
            lat = 48.88
            zoom = 5

        self.m = folium.Map(location=[lat, lon],
                            zoom_start=zoom,
                            control_scale=True,
                            )
        drw = draw.Draw(export=True,
                        show_geometry_on_click=False,
                        filename='my_data.geojson',
                        draw_options={'polyline': False,
                                      'polygone': True,
                                      'rectangle': False,
                                      'circle': False,
                                      'marker': False,
                                      'circlemarker': False})
        self.m.add_child(drw)
        self.save_map()

    def save_map(self):
        self.m.save('./UI/Preflight2/pre2map.html')
        self.web_engine_page = WebEnginePage(self.webView)
        self.webView.setPage(self.web_engine_page)
        self.webView.setHtml(open('./UI/Preflight2/pre2map.html').read())
        self.webView.show()

        self.ui.v_lay_right.addWidget(self.webView)

        # Connect the signal from WebEnginePage to slot in Pre2
        self.web_engine_page.coords_printed.connect(self.update_label_area)

    def draw_polygon(self, coords):

        fg = folium.FeatureGroup(name="ScanArea")
        fg.add_child(folium.Polygon(coords))
        self.m.add_child(fg)
        sw_point, ne_point = self.calculate_sw_ne_points()
        self.m.fit_bounds([sw_point, ne_point])
        self.save_map()


    def add_drone(self):
        # Open Dialog
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
            self.add_drone_to_list(draft_drone)

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
        self.update_metrics()

    def delete_drone(self):
        selected_items = self.ui.listWidget.selectedItems()
        for item in selected_items:
            # Get the widget associated with the item
            drone_widget = self.ui.listWidget.itemWidget(item)
            # Find the QLabel that holds the drone id
            id_label = drone_widget.findChild(QLabel, "id_text")
            # Get the drone id
            drone_id = int(id_label.text())
            # Query the database for the drone with the given id
            drone = session.query(Drone).filter_by(drone_id=drone_id).first()
            if drone:
                # Remove the drone from the database
                session.delete(drone)
                session.commit()
                # Remove the item from the list widget
                self.ui.listWidget.removeItemWidget(item)
                self.ui.listWidget.takeItem(self.ui.listWidget.row(item))
        self.update_metrics()

    def slider_altitude_changed(self):
        value = self.ui.slider_altitude.value()
        self.ui.spinbox_altitude.setValue(value)
        self.update_metrics()

    def spinbox_altitude_changed(self):
        value = self.ui.spinbox_altitude.value()
        self.ui.slider_altitude.setValue(value)
        self.update_metrics()

    def update_label_area(self, coords_lon_lat):

        self.coords_lon_lat = coords_lon_lat
        self.coords = self.invert_coordinates(coords_lon_lat)

        # Update the label_area text with the coordinates
        area = self.calculate_area(self.coords)
        self.ui.selected_area_value.setText(f"{area:.0f}")

        print("Calculated area:", area)  # Debugging

    def update_metrics(self):
        self.calculate_required_capacity()
        self.calculate_mission_time()
        self.calculate_provided_capacity()

    def start_mission(self):
        altitude_val = self.ui.slider_altitude.value()
        # (optimal_route, rotated_route) = route_planner.plan_route(self.coords, altitude=altitude_val,
        #                                                           intersection_ratio=0.8, angle_deg=20)

    @staticmethod
    def calculate_area(coords):
        area = 0.0
        if len(coords) > 2:
            for i in range(len(coords) - 1):
                p1 = coords[i]
                p2 = coords[i + 1]
                area += math.radians(p2[0] - p1[0]) * (
                        2 + math.sin(math.radians(p1[1])) + math.sin(math.radians(p2[1])))
            area = area * 6378137.0 * 6378137.0 / 2.0
        return abs(area)

    def calculate_mission_time(self):
        num_of_drones = self.ui.listWidget.count()
        required_capacity = self.ui.batt_required_value.text()
        if len(required_capacity) and num_of_drones:
            mission_time = int(required_capacity) / num_of_drones
            self.ui.mission_time_value.setText(f"{mission_time:.0f}")
        else:
            self.ui.mission_time_value.setText('0')

    def calculate_required_capacity(self):
        altitude = self.ui.slider_altitude.value()
        if len(self.ui.selected_area_value.text()):
            area = float(self.ui.selected_area_value.text())
            required_capacity = area / altitude ** 2
            self.ui.batt_required_value.setText(f"{required_capacity:.0f}")
        else:
            self.ui.batt_required_value.setText('0')

    def calculate_provided_capacity(self):
        total_battery = 0
        num_of_drones = self.ui.listWidget.count()
        if num_of_drones:
            for i in range(num_of_drones):
                drone_item = self.ui.listWidget.item(i)
                drone_widget = self.ui.listWidget.itemWidget(drone_item)
                spare_batt_field = drone_widget.findChild(QLabel, "spare_batt_text")
                spare_battery_count = int(spare_batt_field.text())
                total_battery = total_battery + spare_battery_count
            self.ui.batt_provided_value.setText(str(15 * total_battery))
        else:
            self.ui.batt_provided_value.setText('0')

    def invert_coordinates(self, coordinates):
        inverted_coordinates = []
        for coord_pair in coordinates:
            inverted_coord = [coord_pair[1], coord_pair[0]]
            inverted_coordinates.append(inverted_coord)
        return inverted_coordinates

    def calculate_sw_ne_points(self):
        if not self.coords:
            return None, None

        lats = [coord[0] for coord in self.coords]
        lons = [coord[1] for coord in self.coords]

        min_lat = min(lats)
        min_lon = min(lons)
        max_lat = max(lats)
        max_lon = max(lons)

        sw_point = (min_lat, min_lon)  # Longitude comes first for southwest point
        ne_point = (max_lat, max_lon)  # Longitude comes first for northeast point

        return sw_point, ne_point

