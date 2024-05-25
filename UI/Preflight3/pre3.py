import json
import folium
import UI.Preflight3.pre3_design

from folium.plugins import MousePosition
from PyQt6.QtCore import QSize, pyqtSignal
from PyQt6.QtWidgets import QWidget, QListWidgetItem, QMessageBox, QPushButton
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtGui import QIcon
from UI import draw
from UI.database import get_mission_by_id, Node
from UI.ListItems.drone_pre3 import Ui_Form
from UI.helpers import WebEnginePage, draw_route, calculate_geographic_distance


class Pre3(QWidget):

    drone_connect_click_signal = pyqtSignal(int)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Preflight3.pre3_design.Ui_Form()
        self.ui.setupUi(self)

        self.mission = None
        self.map = None
        self.webView = QWebEngineView()
        self.ui.v_lay_right.addWidget(self.webView)
        self.is_area_in_coverage = True

    # Loads mission information from database into relevant fields
    def load_mission(self, mission_id):

        self.mission = get_mission_by_id(mission_id)

        # Set mission information box
        self.ui.gb_mission.setTitle("Mission # " + str(self.mission.mission_id))
        self.ui.selected_area_value.setText(str(self.mission.selected_area))
        self.ui.estimated_mission_time_value.setText(str(self.mission.estimated_mission_time))
        self.ui.batt_required_value.setText(str(self.mission.required_battery_capacity))
        self.ui.batt_provided_value.setText(str(self.mission.provided_battery_capacity))

        # Load drones
        self.refresh_drone_list()

        # Set up the Map
        self.setup_map(self.mission.center_node.latitude, self.mission.center_node.longitude, 10)

        # Draw route
        self.update_map()

        # Update Takeoff Button
        self.update_takeoff_button()

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
        new_drone_ui.ip_text.setText(drone.ip_address)
        new_drone_ui.spare_batt_text.setText(str(drone.battery_no))
        new_drone_ui.btn_connect.clicked.connect(
            lambda is_checked, button=new_drone_ui.btn_connect, did=drone.drone_id: self.connect_drone_clicked(button, is_checked, did))

        # Calculate the height of the new_drone_widget
        new_drone_widget.adjustSize()
        widget_height = new_drone_widget.sizeHint().height()

        # Set the size hint for the item
        new_drone_item.setSizeHint(QSize(self.ui.listWidget.lineWidth(), widget_height))

        self.ui.listWidget.addItem(new_drone_item)
        self.ui.listWidget.setItemWidget(new_drone_item, new_drone_widget)

    # Creates a map given center point and zoom level
    def setup_map(self, lat, lon, zoom):

        self.map = folium.Map(location=[lat, lon],
                              zoom_start=zoom,
                              control_scale=True, )

        drw = draw.Draw(export=False,
                        show_geometry_on_click=True,
                        filename='my_data.geojson',
                        draw_options={'polyline': False,
                                      'polygon': False,
                                      'rectangle': False,
                                      'circle': False,
                                      'marker': True,
                                      'circlemarker': False},
                        edit_options={'edit': False,
                                      'remove': False})
        self.map.add_child(drw)

        self.map.add_child(MousePosition(position="topright", separator=" | ", empty_string="NaN", lng_first=False,))

        self.save_map()

    # Saves and shows the map
    def save_map(self):
        self.map.save('./UI/Preflight3/pre3_map.html')
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(open('./UI/Preflight3/pre3_map.html').read())
        self.webView.show()

        # Listen for any events on the Map
        page.point_coords_printed.connect(self.gcs_changed)

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

    # Captures changes in the Ground Control Station marker and makes necessary updates
    def gcs_changed(self, coords_lon_lat):
        if self.mission.gcs_node is None:
            gcs_node = Node()
            gcs_node.add_to_db()
            self.mission.gcs_node_id = gcs_node.node_id

        self.mission.gcs_node.latitude = coords_lon_lat[1]
        self.mission.gcs_node.longitude = coords_lon_lat[0]

        # Check if the selected area is within coverage
        self.is_area_in_coverage = True
        for vertex in json.loads(self.mission.mission_boundary):
            gcs = [self.mission.gcs_node.latitude, self.mission.gcs_node.longitude]
            distance = calculate_geographic_distance(vertex, gcs)
            if distance > 2000:
                self.is_area_in_coverage = False
                break

        if not self.is_area_in_coverage:
            QMessageBox().warning(self, "Coverage Error!", "The area to be scanned is out of coverage.\n"
                                                           "Select a new GCS (Ground Control Station) point so that "
                                                           "the area to be scanned is within coverage.!")

        self.update_takeoff_button()
        self.setup_map(self.mission.center_node.latitude, self.mission.center_node.longitude, 10)
        self.update_map()

    # Update (Enable/Disable) Takeoff Button
    def update_takeoff_button(self):
        self.ui.btn_take_off.setEnabled(False)

        # Check if the GCS is selected
        if (self.mission.gcs_node == 'null') or (self.mission.gcs_node is None):
            self.ui.btn_take_off.setEnabled(False)
            return

        # Check if the selected area is within coverage
        if not self.is_area_in_coverage:
            return

        # Check if all drones connected
        for i in range(self.ui.listWidget.count()):
            list_item = self.ui.listWidget.item(i)
            drone_widget = self.ui.listWidget.itemWidget(list_item)
            connect_button = drone_widget.findChild(QPushButton, "btn_connect")
            if not connect_button.isChecked():
                return

        self.ui.btn_take_off.setEnabled(True)

    def connect_drone_clicked(self, button, is_checked, drone_id):
        if is_checked:
            button.setText("Connected, Click to Disconnect")
            button.setIcon(QIcon('./UI/Images/disconnected.png'))
            print("Connecting to Drone #", drone_id)
            self.update_takeoff_button()
        else:
            button.setText("Click to Connect")
            button.setIcon(QIcon('./UI/Images/connected.png'))
            print("Disconnecting from Drone #", drone_id)
            self.update_takeoff_button()

        self.drone_connect_click_signal.emit(drone_id)
