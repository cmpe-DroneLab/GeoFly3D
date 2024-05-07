import json
import folium
import UI.Midflight.mid_design

from folium import JsCode
from folium.plugins import MousePosition, Realtime
from PyQt6.QtCore import QSize, pyqtSignal, QTimer
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import QWidget, QListWidgetItem, QLabel
from UI.database import session, Mission, Drone
from UI.ListItems.drone_mid import Ui_Form
from UI.helpers import RouteDrawer, WebEnginePage, update_drone_position_on_map, resource_path
from drone_controller import DroneController


class Mid(QWidget):
    drone_position_updated = pyqtSignal(float, float)  # Define a signal to indicate updated drone position

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Midflight.mid_design.Ui_Form()
        self.ui.setupUi(self)

        self.mission = None
        self.map = None
        self.webView = QWebEngineView()
        self.ui.v_lay_right.addWidget(self.webView)

        self.has_taken_off = False
        self.threads = {}

        self.drone_position_updated.connect(update_drone_position_on_map)
        self.drone_position_updated.connect(self.update_live_data)

    # Loads mission information from database into relevant fields
    def load_mission(self, mission_id):
        if mission_id == 0:
            exit(-1)
        else:
            self.mission = session.query(Mission).filter_by(mission_id=mission_id).first()

        # Set mission information box
        self.ui.selected_area_value.setText(str(self.mission.selected_area))
        self.ui.mission_time_value.setText(str(self.mission.estimated_mission_time))
        self.ui.scanned_area_value.setText(str(self.mission.scanned_area))

        # Load drones
        self.refresh_drone_list()

        # Set mission id in the header box
        self.ui.id_label.setText(str(self.mission.mission_id))

        # Set up the Map
        self.setup_map(self.mission.center_lat, self.mission.center_lon, 10)

        # Draw route
        self.draw_route()

    # Gets all matching drones from the database, adds them to the Drone List
    def refresh_drone_list(self):
        self.ui.listWidget.clear()

        # Find all drones matching the Mission
        drones = session.query(Drone).filter_by(mission_id=self.mission.mission_id).all()
        for drone in drones:
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

    def draw_route(self):
        if self.mission:
            RouteDrawer.draw_route(self.map, self.mission)
            self.save_map()

    def add_drone_markers(self):
        source = "http://localhost:9000/UI/Midflight/rt_drone_info.geojson"
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
                                '<b>ID:</b> ' + f.properties.droneId + '<br>' +
                                '<b>Model:</b> ' + f.properties.model + '<br>' +
                                '<b>Battery:</b> ' + f.properties.battery + '<br>' +
                                '<b>Status:</b> ' + f.properties.status
                            ).openPopup();
                        }
                    """),
            interval=10000,
        ).add_to(self.map)

    def update_live_data(self):
        for row in range(self.ui.listWidget.count()):
            item = self.ui.listWidget.item(row)
            drone_id = item.listWidget().itemWidget(item).findChild(QLabel, "id_text").text()
            battery_field = item.listWidget().itemWidget(item).findChild(QLabel, "battery_text")
            status_field = item.listWidget().itemWidget(item).findChild(QLabel, "status_text")

            # Read the GeoJSON file
            with open(resource_path('rt_drone_info.geojson'), 'r') as fr:
                data = json.load(fr)
            fr.close()
            for feature in data['features']:
                if feature['properties']['droneId'] == int(drone_id):
                    battery_field.setText(str(feature['properties'].get('battery')))
                    status_field.setText(feature['properties'].get('status'))

    def take_off(self, vertices, flight_altitude, mission_id, gimbal_angle, route_angle, rotated_route_angle):
        if mission_id in self.threads:
            return

        drone_controller_thread = DroneController(vertices=vertices, mission_id=mission_id, flight_altitude=flight_altitude,
                                                  gimbal_angle=gimbal_angle, route_angle=route_angle,
                                                  rotated_route_angle=rotated_route_angle)

        drone_controller_thread.started.connect(print)
        drone_controller_thread.progress_text.connect(print)
        drone_controller_thread.update_coord.connect(update_drone_position_on_map)

        self.threads[mission_id] = drone_controller_thread
        drone_controller_thread.start()
        self.has_taken_off = True
        mission = session.query(Mission).filter_by(mission_id=mission_id).first()
        mission.mission_status = "Mid Flight"
        session.commit()

        return drone_controller_thread