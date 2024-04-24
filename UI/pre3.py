import folium
from PyQt6.QtWidgets import QWidget
from PyQt6.QtWebEngineWidgets import QWebEngineView

from RoutePlanner import route_planner
from UI.web_engine_page import WebEnginePage
import UI.pre3_design


class Pre3(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.m = None
        self.ui = UI.pre3_design.Ui_Form()
        self.ui.setupUi(self)
        self.webView = QWebEngineView()
        self.ui.v_lay_right.addWidget(self.webView)

    def setup_map(self, coords, altitude):

        self.m = folium.Map(location=[35, 39],
                            zoom_start=5,
                            control_scale=True,
                            )
        self.save_map()
        print("Coords: ", coords)
        (optimal_route, rotated_route) = route_planner.plan_route(coords, altitude=altitude, intersection_ratio=0.8, angle_deg=20)
        sw_point, ne_point = self.calculate_sw_ne_points(coords)
        self.m.fit_bounds([sw_point, ne_point])
        self.draw_optimal_route(optimal_route)
        self.draw_rotated_route(rotated_route)

    def save_map(self):
        self.m.save('./UI/pre3map.html')
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(open('./UI/pre3map.html').read())
        self.webView.show()

    def add_marker(self, lat, lon, popup_text):
        folium.Marker([lat, lon], popup=popup_text).add_to(self.m)
        self.save_map()

    def draw_optimal_route(self, route):
        print(route)

        # draw path nodes
        for point in route:
            folium.CircleMarker(point,
                                radius=2,
                                fill=True,
                                fill_color="blue",
                                fill_opacity=1).add_to(self.m)

        # draw path edges
        route_line = folium.PolyLine(locations=route, color='green', weight=2.5, opacity=0.8)
        self.m.add_child(route_line)
        self.save_map()

    def draw_rotated_route(self, route):

        # draw path nodes
        for point in route:
            folium.CircleMarker(point,
                                radius=2,
                                fill=True,
                                fill_color="orange",
                                fill_opacity=1).add_to(self.m)

        # draw path edges
        route_line = folium.PolyLine(locations=route, color='orange', weight=2.5, opacity=0.8)
        self.m.add_child(route_line)
        self.save_map()

    def calculate_sw_ne_points(self, coords):
        if not coords:
            return None, None

        lats = [coord[0] for coord in coords]
        lons = [coord[1] for coord in coords]

        min_lat = min(lats)
        min_lon = min(lons)
        max_lat = max(lats)
        max_lon = max(lons)

        sw_point = (min_lon, min_lat)  # Longitude comes first for southwest point
        ne_point = (max_lon, max_lat)  # Longitude comes first for northeast point

        return sw_point, ne_point
