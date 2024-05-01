import json

import folium
from PyQt6.QtCore import QSize
from PyQt6.QtWidgets import QWidget, QListWidgetItem
from PyQt6.QtWebEngineWidgets import QWebEngineView

from RoutePlanner import route_planner
from UI.database import session, Mission, Drone
from UI.web_engine_page import WebEnginePage
import UI.Preflight3.pre3_design
from UI.drone import Ui_Form
from UI.Preflight2.pre2 import calculate_sw_ne_points, invert_coordinates


class Pre3(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Preflight3.pre3_design.Ui_Form()
        self.ui.setupUi(self)

        self.mission = None
        self.mission_id = None
        self.coords = None
        self.coords_lon_lat = None
        self.optimal_route = None
        self.rotated_route = None
        self.map = None
        self.webView = QWebEngineView()
        self.ui.v_lay_right.addWidget(self.webView)

    # Loads mission information from database into relevant fields
    def load_mission(self, mission_id):

        if mission_id == 0:
            exit(-1)
        else:
            self.mission = session.query(Mission).filter_by(mission_id=mission_id).first()

        self.mission_id = self.mission.mission_id
        self.coords = json.loads(self.mission.coordinates)
        self.coords_lon_lat = invert_coordinates(self.coords)

        # Set mission id in the header box
        self.ui.id_label.setText(str(self.mission_id))

        # Set mission information box
        self.ui.selected_area_value.setText(str(self.mission.selected_area))
        self.ui.mission_time_value.setText(str(self.mission.estimated_mission_time))
        self.ui.batt_required_value.setText(str(self.mission.required_battery_capacity))

        # Load drones
        self.refresh_drone_list()

        # Set up the Map
        self.setup_map()
        self.calculate_route()
        self.draw_optimal_route(self.optimal_route)
        self.draw_rotated_route(self.rotated_route)

    # Gets all matching drones from the database, adds them to the Drone List
    def refresh_drone_list(self):
        self.ui.listWidget.clear()

        # Find all drones matching the Mission
        drones = session.query(Drone).filter_by(mission_id=self.mission_id).all()
        for drone in drones:
            self.add_drone_to_list(drone)

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

    # Creates a map
    def setup_map(self):

        self.map = folium.Map(location=[35, 39],
                              zoom_start=5,
                              control_scale=True, )
        self.save_map()

        sw_point, ne_point = calculate_sw_ne_points(self.coords)
        self.map.fit_bounds([sw_point, ne_point])

    # Saves and shows the map
    def save_map(self):
        self.map.save('./UI/Preflight3/pre3_map.html')
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(open('./UI/Preflight3/pre3_map.html').read())
        self.webView.show()

    # Calculates route
    def calculate_route(self):
        (self.optimal_route, self.rotated_route) = route_planner.plan_route(coords=self.coords_lon_lat[:-1],
                                                                            altitude=self.mission.altitude,
                                                                            intersection_ratio=0.8,
                                                                            route_angle_deg=self.mission.route_angle,
                                                                            rotated_route_angle_deg=self.mission.rotated_route_angle)

    # Draws optimal route
    def draw_optimal_route(self, route):

        # draw path nodes
        for point in route:
            folium.CircleMarker(point,
                                radius=5,
                                color="green",
                                weight=3,
                                opacity=0.8,
                                fill=False).add_to(self.map)

        # draw path edges
        route_line = folium.PolyLine(locations=route,
                                     color='green',
                                     weight=2.5,
                                     opacity=0.8)
        self.map.add_child(route_line)
        self.save_map()

    # Draws rotated route
    def draw_rotated_route(self, route):

        # draw path nodes
        for point in route:
            folium.CircleMarker(point,
                                radius=5,
                                stroke=False,
                                fill=True,
                                fill_color="orange",
                                fill_opacity=0.8).add_to(self.map)

        # draw path edges
        route_line = folium.PolyLine(locations=route,
                                     color='orange',
                                     weight=2.5,
                                     opacity=0.8)
        self.map.add_child(route_line)
        self.save_map()
