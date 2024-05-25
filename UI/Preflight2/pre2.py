import json
import math
import folium
import UI.Preflight2.pre2_design

from folium.plugins import MousePosition
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import QDialog, QListWidgetItem, QWidget, QLabel
from RoutePlanner import route_planner
from UI import draw
from UI.database import Mission, Drone, Session, Node, Path
from UI.Dialogs.drone_dialog import Ui_Dialog
from UI.ListItems.drone_pre import Ui_Form
from UI.helpers import WebEnginePage, draw_route, calculate_center_point, invert_coordinates, get_current_time


class Pre2(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Preflight2.pre2_design.Ui_Form()
        self.ui.setupUi(self)

        self.session = Session()
        self.mission = None
        self.map = None
        self.webView = QWebEngineView()
        self.setup_map(39, 35, 5)
        self.ui.v_lay_right.addWidget(self.webView)

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
                altitude=100,
                gimbal_angle=-90,
                route_angle=0,
                rotated_route_angle=20
            )
            self.session.add(self.mission)
            self.session.flush()
            self.mission.center_node = None
        else:
            self.mission = self.session.query(Mission).filter_by(mission_id=mission_id).first()

        # Set mission information box
        self.ui.gb_mission.setTitle("Mission # " + str(self.mission.mission_id))
        self.ui.selected_area_value.setText(str(self.mission.selected_area))
        self.ui.estimated_mission_time_value.setText(str(self.mission.estimated_mission_time))
        self.ui.batt_required_value.setText(str(self.mission.required_battery_capacity))

        # Load drones
        self.refresh_drone_list()
        self.calculate_provided_capacity()

        # Set up the Map
        if self.mission.center_node is None:
            self.setup_map(41.0859528, 29.0443435, 10)
        else:
            self.setup_map(self.mission.center_node.latitude, self.mission.center_node.longitude, 10)

        # Set up the flight parameters
        self.ui.spinbox_altitude.setValue(self.mission.altitude)
        self.ui.spinbox_gimbal.setValue(self.mission.gimbal_angle)
        self.ui.spinbox_route_angle.setValue(self.mission.route_angle)
        self.ui.spinbox_rotated_route_angle.setValue(self.mission.rotated_route_angle)

        # Draw route
        self.update_map()

        # Set Start Mission button
        self.update_start_button()

    # Gets all matching drones from the database, adds them to the Drone List
    def refresh_drone_list(self):
        self.ui.listWidget.clear()
        drones = self.session.query(Drone).filter_by(mission_id=self.mission.mission_id).all()
        for drone in drones:
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
            draft_path = Path(opt_route_length=0,
                              rot_route_length=0,
                              vertex_count=0,
                              mission_id=self.mission.mission_id)
            self.session.add(draft_path)
            self.session.flush()

            draft_drone = Drone(model=dialog_ui.model_combo.currentText(),
                                ip_address=dialog_ui.ip_text.text(),
                                battery_no=dialog_ui.spare_spin.value(),
                                mission_id=self.mission.mission_id,
                                path_id=draft_path.path_id)
            self.session.add(draft_drone)
            self.session.flush()

            # Refresh drone list after creating the drone
            self.refresh_drone_list()
            self.update_map()

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
        
        # Query the database for the drone with the given id
        drone_id = int(selected_drone_widget.findChild(QLabel, "id_text").text())
        drone = self.session.query(Drone).filter_by(drone_id=drone_id).first()

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
            self.refresh_drone_list()
            self.update_map()

    # Deletes the drone selected from the list from the database
    def delete_drone(self):
        selected_items = self.ui.listWidget.selectedItems()

        # Find selected drone(s)
        for item in selected_items:
            # Get the drone id
            drone_widget = self.ui.listWidget.itemWidget(item)
            id_label = drone_widget.findChild(QLabel, "id_text")
            drone_id = int(id_label.text())

            # Delete the drone object from the database session
            drone = self.session.query(Drone).filter_by(drone_id=drone_id).first()
            self.session.delete(drone)
            self.session.flush()
        self.refresh_drone_list()
        self.update_map()

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
        if (self.mission.mission_boundary != 'null') and (self.mission.mission_boundary is not None) and (
                int(self.ui.batt_provided_value.text()) > int(self.ui.batt_required_value.text())):
            self.ui.btn_start.setEnabled(True)
        else:
            self.ui.btn_start.setEnabled(False)

    # Saves mission to the database
    def save_mission(self):
        self.mission.gimbal_angle = self.ui.spinbox_gimbal.value()
        self.mission.last_update_time = get_current_time()
        self.session.commit()
        self.refresh_drone_list()
        self.update_map()

    # Cancels mission changes and rolls back to the state before the last commit
    def cancel_mission(self):
        self.session.rollback()

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

    def altitude_changed(self):
        self.mission.altitude = self.ui.spinbox_altitude.value()
        self.update_map()
        self.update_metrics()

    def gimbal_angle_changed(self):
        self.mission.gimbal_angle = self.ui.spinbox_gimbal.value()

    def route_angle_changed(self):
        self.mission.route_angle = self.ui.spinbox_route_angle.value()
        self.update_map()
        self.update_metrics()

    def rotated_route_angle_changed(self):
        self.mission.rotated_route_angle = self.ui.spinbox_rotated_route_angle.value()
        self.update_map()
        self.update_metrics()

    def selected_area_changed(self, coords_lon_lat):
        self.mission.mission_boundary = json.dumps(invert_coordinates(coords_lon_lat))
        if self.mission.center_node is None:
            center_node = Node()
            self.session.add(center_node)
            self.session.flush()
            self.mission.center_node = center_node

        if self.mission.mission_boundary is not None:
            self.mission.center_node.latitude, self.mission.center_node.longitude = calculate_center_point(json.loads(self.mission.mission_boundary))
        self.update_area_metrics()

    # TODO: Fix needed
    def selected_area_deleted(self):
        # Delete the center_node object from the database session
        if self.mission.center_node is not None:
            self.session.delete(self.mission.center_node)
            self.session.flush()

        self.mission.mission_boundary = None
        self.update_area_metrics()

    def update_metrics(self):
        self.calculate_provided_capacity()
        self.calculate_req_capacity_est_time()
        self.update_start_button()

    def update_area_metrics(self):
        # Update the label_area text with the coordinates
        area = self.calculate_selected_area()
        self.ui.selected_area_value.setText(f"{area:.0f}")
        self.mission.selected_area = area
        self.update_metrics()

    def calculate_selected_area(self):
        if self.mission.mission_boundary == 'null' or self.mission.mission_boundary is None:
            return 0

        area = 0
        coords = json.loads(self.mission.mission_boundary)
        if len(coords) > 2:
            for i in range(len(coords) - 1):
                p1 = coords[i]
                p2 = coords[i + 1]
                area += math.radians(p2[0] - p1[0]) * (
                        2 + math.sin(math.radians(p1[1])) + math.sin(math.radians(p2[1])))
            area = area * 6378137.0 * 6378137.0 / 2.0
        return int(abs(area))

    def calculate_req_capacity_est_time(self):
        altitude = self.ui.spinbox_altitude.value()
        total_required_capacity = 0
        max_required_capacity = 0
        for path in self.session.query(Path).filter_by(mission_id=self.mission.mission_id).all():
            vertex_count = path.vertex_count
            path_length = path.opt_route_length + path.rot_route_length
            required_capacity = (altitude * 0.155126 + vertex_count * 5.594083 + path_length * 0.232189 + 5.199370)/60
            total_required_capacity += required_capacity
            if required_capacity > max_required_capacity:
                max_required_capacity = required_capacity
        self.ui.batt_required_value.setText(str(round(total_required_capacity)))
        self.mission.required_battery_capacity = total_required_capacity
        self.ui.estimated_mission_time_value.setText(str(round(max_required_capacity)))
        self.mission.estimated_mission_time = round(max_required_capacity)

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

    def update_map(self):
        if (self.mission.mission_boundary == 'null') or (self.mission.mission_boundary is None):
            return

        if self.mission.center_node is None:
            self.setup_map(41.0859528, 29.0443435, 10)
        else:
            self.setup_map(self.mission.center_node.latitude, self.mission.center_node.longitude, 10)

        self.plan_route()
        mission_paths = self.session.query(Path).filter_by(mission_id=self.mission.mission_id).all()
        draw_route(
            map_obj=self.map,
            mission_paths=mission_paths,
            mission_boundary=json.loads(self.mission.mission_boundary),
            gcs_node=self.mission.gcs_node
        )
        self.update_metrics()
        self.save_map()

    def plan_route(self):
        drones = self.session.query(Drone).filter_by(mission_id=self.mission.mission_id).all()
        if (self.mission.mission_boundary == 'null'
                or self.mission.mission_boundary is None
                or len(drones) == 0):
            return

        drone_capacities = []
        for drone in drones:
            drone_capacities.append(15 * (drone.battery_no + 1))

        mission_boundary_lon_lat = invert_coordinates(json.loads(self.mission.mission_boundary))
        paths = route_planner.plan_route(
            coords=mission_boundary_lon_lat[:-1],
            drone_capacities=drone_capacities,
            altitude=self.mission.altitude,
            intersection_ratio=0.8,
            route_angle_deg=self.mission.route_angle,
            rotated_route_angle_deg=self.mission.rotated_route_angle,
        )

        for i, calc_path in enumerate(paths):
            drone = drones[i]

            drone.path.path_boundary = json.dumps(calc_path[0])
            drone.path.opt_route = json.dumps(calc_path[1])
            drone.path.rot_route = json.dumps(calc_path[3])
            drone.path.opt_route_length = round(calc_path[2])
            drone.path.rot_route_length = round(calc_path[4])
            drone.path.vertex_count = len(calc_path[1]) + len(calc_path[3])
            self.session.add(drone)
            self.session.flush()


