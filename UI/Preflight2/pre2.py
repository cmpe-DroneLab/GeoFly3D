import json
import math
import folium
import UI.Preflight2.pre2_design

from folium.plugins import MousePosition
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import QDialog, QListWidgetItem, QWidget, QLabel
from UI import draw
from UI.database import Drone, session, Mission, get_mission_drones
from UI.Dialogs.drone_dialog import Ui_Dialog
from UI.ListItems.drone_pre import Ui_Form
from UI.helpers import RouteDrawer, WebEnginePage, calculate_center_point, invert_coordinates, get_current_time


class Pre2(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Preflight2.pre2_design.Ui_Form()
        self.ui.setupUi(self)

        self.mission = None
        self.map = None
        self.webView = QWebEngineView()
        self.setup_map(39, 35, 5)
        self.ui.v_lay_right.addWidget(self.webView)
        self.total_path_length = 0
        self.total_vertex_count = 0

        self.ui.spinbox_altitude.valueChanged.connect(self.altitude_changed)
        self.ui.spinbox_gimbal.valueChanged.connect(self.gimbal_angle_changed)
        self.ui.spinbox_route_angle.valueChanged.connect(self.route_angle_changed)
        self.ui.spinbox_rotated_route_angle.valueChanged.connect(self.rotated_route_angle_changed)
        self.ui.btn_add_drone.clicked.connect(self.create_drone)
        self.ui.btn_edit_drone.clicked.connect(self.edit_drone)
        self.ui.listWidget.doubleClicked.connect(self.edit_drone)
        self.ui.btn_delete_drone.clicked.connect(self.delete_drone)
        self.ui.listWidget.itemSelectionChanged.connect(self.enable_buttons)
        self.ui.btn_save.clicked.connect(self.save_mission)
        self.ui.btn_cancel.clicked.connect(self.cancel_mission)

    # Loads mission information from database into relevant fields
    def load_mission(self, mission_id):
        if mission_id == 0:
            self.mission = Mission(
                creation_time=get_current_time(),
                mission_status="Draft",
                estimated_mission_time=0,
                actual_mission_time=0,
                provided_battery_capacity=0,
                required_battery_capacity=0,
                selected_area=0,
                scanned_area=0,
                altitude=100,
                gimbal_angle=-90,
                route_angle=0,
                rotated_route_angle=20,
                last_visited_node_lat=500,
                last_visited_node_lon=500,
            )
            session.add(self.mission)
        else:
            self.mission = session.query(Mission).filter_by(mission_id=mission_id).first()
            self.mission.last_visited_node_lat = 500
            self.mission.last_visited_node_lon = 500

        self.total_path_length = 0
        self.total_vertex_count = 0

        # Set mission information box
        self.ui.selected_area_value.setText(str(self.mission.selected_area))
        self.ui.estimated_mission_time_value.setText(str(self.mission.estimated_mission_time))
        self.ui.batt_required_value.setText(str(self.mission.required_battery_capacity))

        # Load drones
        self.refresh_drone_list()
        self.calculate_provided_capacity()

        # Set mission id in the header box
        self.ui.gb_mission.setTitle("Mission # " + str(self.mission.mission_id))

        # Set up the Map
        if self.mission.center_lat is None:
            self.setup_map(41.0859528, 29.0443435, 10)
        else:
            self.setup_map(self.mission.center_lat, self.mission.center_lon, 10)

        # Set altitude value
        self.ui.spinbox_altitude.setValue(self.mission.altitude)

        # Set gimbal angle value
        self.ui.spinbox_gimbal.setValue(self.mission.gimbal_angle)

        # Set route angle value
        self.ui.spinbox_route_angle.setValue(self.mission.route_angle)

        # Set rotated route angle value
        self.ui.spinbox_rotated_route_angle.setValue(self.mission.rotated_route_angle)

        # Draw route
        self.draw_route()

        # Set Start Mission button
        self.update_start_button()

    # Gets all matching drones from the database, adds them to the Drone List
    def refresh_drone_list(self):
        self.ui.listWidget.clear()
        for drone in get_mission_drones(self.mission.mission_id):
            self.add_drone_to_list(drone)

        # Disable Edit Drone and Delete Drone buttons
        self.disable_buttons()

        # Update metrics
        self.update_metrics()

    # Creates a Drone
    def create_drone(self):
        # Open create drone dialog
        dialog_win = QDialog()
        dialog_ui = Ui_Dialog()
        dialog_ui.setupUi(dialog_win)
        response = dialog_win.exec()

        if response == 1:
            # Create the drone object and add it to the database session
            draft_drone = Drone(model=dialog_ui.model_combo.currentText(),
                                ip_address=dialog_ui.ip_text.text(),
                                battery_no=dialog_ui.spare_spin.value(),
                                mission_id=self.mission.mission_id)
            session.add(draft_drone)

            # Refresh drone list after creating the drone
            self.refresh_drone_list()

    # Adds given drone to the Drone List
    def add_drone_to_list(self, drone):
        new_drone_item = QListWidgetItem()
        new_drone_widget = QWidget()
        new_drone_ui = Ui_Form()
        new_drone_ui.setupUi(new_drone_widget)
        new_drone_ui.id_text.setText(str(drone.drone_id))
        new_drone_ui.model_text.setText(drone.model)
        new_drone_ui.ip_text.setText(drone.ip_address)
        new_drone_ui.spare_batt_text.setText(str(drone.battery_no))

        # Calculate the height of the new_drone_widget
        new_drone_widget.adjustSize()
        widget_height = new_drone_widget.sizeHint().height()

        # Set the size hint for the item
        new_drone_item.setSizeHint(QSize(self.ui.listWidget.lineWidth(), widget_height))

        self.ui.listWidget.addItem(new_drone_item)
        self.ui.listWidget.setItemWidget(new_drone_item, new_drone_widget)

    # Edits selected Drone
    def edit_drone(self):
        # Get the selected item and widget associated with it
        selected_item = self.ui.listWidget.selectedItems()[0]
        selected_drone_widget = self.ui.listWidget.itemWidget(selected_item)

        drone_id = int(selected_drone_widget.findChild(QLabel, "id_text").text())
        # Query the database for the drone with the given id
        drone = session.query(Drone).filter_by(drone_id=drone_id).first()

        # Open edit drone dialog
        dialog_win = QDialog()
        dialog_ui = Ui_Dialog()
        dialog_ui.setupUi(dialog_win)
        dialog_win.setWindowTitle("Edit Drone #" + str(drone_id))
        dialog_ui.model_combo.setCurrentText(drone.model)
        dialog_ui.ip_text.setText(drone.ip_address)
        dialog_ui.spare_spin.setValue(drone.battery_no)
        response = dialog_win.exec()

        if response == 1:
            # Edit the drone object and add it to the database session
            drone.model = dialog_ui.model_combo.currentText()
            drone.ip_address = dialog_ui.ip_text.text()
            drone.battery_no = dialog_ui.spare_spin.value()

            # Refresh drone list after updating the drone
            self.refresh_drone_list()

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
            # Query the database for the drone with the given id and delete it
            drone = session.query(Drone).filter_by(drone_id=drone_id).first()

            # Delete the drone object from the database session
            if drone:
                session.delete(drone)
                # Refresh drone list after deleting the drone
                self.refresh_drone_list()

    # Enables Edit Drone and Delete Drone buttons
    def enable_buttons(self):
        if len(self.ui.listWidget.selectedIndexes()):
            self.ui.btn_delete_drone.setEnabled(True)
            self.ui.btn_edit_drone.setEnabled(True)

    # Disables Edit Drone and Delete Drone buttons
    def disable_buttons(self):
        self.ui.btn_delete_drone.setEnabled(False)
        self.ui.btn_edit_drone.setEnabled(False)

    # Captures button pressings and performs necessary actions
    def keyPressEvent(self, event):
        # Clear selection when ESC button is pressed
        if event.key() == Qt.Key.Key_Escape:
            self.ui.listWidget.clearSelection()
            self.disable_buttons()
        # Delete selected drone when DELETE/BACKSPACE button is pressed
        if event.key() == Qt.Key.Key_Delete or event.key() == Qt.Key.Key_Backspace:
            self.delete_drone()
        else:
            super().keyPressEvent(event)

    # Update Start Mission button
    def update_start_button(self):
        if (self.mission.coordinates != 'null') and (self.mission.coordinates is not None) and (
                int(self.ui.batt_provided_value.text()) > int(self.ui.batt_required_value.text())):
            self.ui.btn_start.setEnabled(True)
        else:
            self.ui.btn_start.setEnabled(False)

    # Saves mission to the database
    def save_mission(self):

        self.draw_route()
        self.mission.last_update_time = get_current_time()

        # Commit changes to the database
        session.commit()

        # Refresh drone list after saving mission
        self.refresh_drone_list()

    # Cancels mission changes and rolls back to the state before the last commit
    def cancel_mission(self):
        # Rollback to the state before the last commit
        session.rollback()

    # Creates a map given center point and zoom level
    def setup_map(self, lat, lon, zoom):

        self.map = folium.Map(location=[lat, lon],
                              zoom_start=zoom,
                              control_scale=True, )

        drw = draw.Draw(export=False,
                        show_geometry_on_click=True,
                        filename='my_data.geojson',
                        draw_options={'polyline': False,
                                      'polygon': True,
                                      'rectangle': True,
                                      'circle': False,
                                      'marker': False,
                                      'circlemarker': False})
        self.map.add_child(drw)
        self.map.add_child(MousePosition(position="topright", separator=" | ", empty_string="NaN", lng_first=False,))

        self.save_map()

    # Saves and shows the map
    def save_map(self):
        self.map.save('./UI/Preflight2/pre2_map.html')
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(open('./UI/Preflight2/pre2_map.html').read())
        self.webView.show()

        # Listen for any events on the Map
        page.polygon_coords_printed.connect(self.selected_area_changed)
        page.drawings_deleted.connect(self.selected_area_deleted)

    # Captures changes in the altitude spinbox and makes necessary updates
    def altitude_changed(self):
        self.mission.altitude = self.ui.spinbox_altitude.value()
        self.draw_route()
        self.update_metrics()

    # Captures changes in the gimbal angle spinbox and makes necessary updates
    def gimbal_angle_changed(self):
        self.mission.gimbal_angle = self.ui.spinbox_gimbal.value()

    # Captures changes in the routing angle spinbox and makes necessary updates
    def route_angle_changed(self):
        self.mission.route_angle = self.ui.spinbox_route_angle.value()
        self.draw_route()
        self.update_metrics()

    # Captures changes in the rotated routing angle spinbox and makes necessary updates
    def rotated_route_angle_changed(self):
        self.mission.rotated_route_angle = self.ui.spinbox_rotated_route_angle.value()
        self.draw_route()
        self.update_metrics()

    # Captures changes in the selected area and makes necessary updates
    def selected_area_changed(self, coords_lon_lat):
        self.mission.coordinates = json.dumps(invert_coordinates(coords_lon_lat))
        self.mission.center_lat, self.mission.center_lon = calculate_center_point(json.loads(self.mission.coordinates))
        self.update_area_metrics()

    # Captures changes in the selected area and makes necessary updates
    def selected_area_deleted(self):
        self.mission.coordinates = None
        self.mission.center_lat = None
        self.mission.center_lon = None
        self.update_area_metrics()

    def update_metrics(self):
        self.calculate_provided_capacity()
        self.calculate_required_capacity()
        self.calculate_mission_time()
        self.update_start_button()

    # Updates selected area related metrics
    def update_area_metrics(self):
        # Update the label_area text with the coordinates
        area = self.calculate_selected_area()
        self.ui.selected_area_value.setText(f"{area:.0f}")
        self.mission.selected_area = area
        self.update_metrics()

    # Calculates selected area from coordinates
    def calculate_selected_area(self):

        if self.mission.coordinates == 'null' or self.mission.coordinates is None:
            return 0

        area = 0
        coords = json.loads(self.mission.coordinates)
        if len(coords) > 2:
            for i in range(len(coords) - 1):
                p1 = coords[i]
                p2 = coords[i + 1]
                area += math.radians(p2[0] - p1[0]) * (
                        2 + math.sin(math.radians(p1[1])) + math.sin(math.radians(p2[1])))
            area = area * 6378137.0 * 6378137.0 / 2.0
        return int(abs(area))

    # Calculates mission time
    def calculate_mission_time(self):
        num_of_drones = self.ui.listWidget.count()
        required_capacity = self.ui.batt_required_value.text()
        if len(required_capacity) and num_of_drones:
            mission_time = int(required_capacity) / num_of_drones
            self.ui.estimated_mission_time_value.setText(f"{mission_time:.0f}")
        else:
            self.ui.estimated_mission_time_value.setText('0')
        self.mission.estimated_mission_time = int(self.ui.estimated_mission_time_value.text())

    # Calculates required battery capacity for the mission
    def calculate_required_capacity(self):
        altitude = self.ui.spinbox_altitude.value()
        required_capacity = int((altitude * 0.155126 + self.total_vertex_count * 5.594083 + self.total_path_length * 0.232189 + 5.199370)/60)
        self.ui.batt_required_value.setText(str(required_capacity))
        self.mission.required_battery_capacity = required_capacity

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
        self.mission.provided_battery_capacity = int(self.ui.batt_provided_value.text())

    def draw_route(self):
        if (self.mission.coordinates == 'null') or (self.mission.coordinates is None):
            return

        if self.mission:
            self.setup_map(self.mission.center_lat, self.mission.center_lon, 10)
            optimal_route_length, rotated_route_length, self.total_vertex_count = RouteDrawer.draw_route(self.map, self.mission)
            self.total_path_length = optimal_route_length + rotated_route_length
            self.update_metrics()
            self.save_map()
