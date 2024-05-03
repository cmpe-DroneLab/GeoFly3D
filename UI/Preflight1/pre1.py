import json
import folium
import UI.Preflight1.pre1_design

from folium.plugins import MousePosition, MarkerCluster
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QWidget, QListWidgetItem
from PyQt6.QtWebEngineWidgets import QWebEngineView
from UI.database import Mission, session, get_all_missions, Drone
from UI.helpers import WebEnginePage, calculate_sw_ne_points


class Pre1(QWidget):
    mission_deleted = pyqtSignal(int)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Preflight1.pre1_design.Ui_Form()
        self.ui.setupUi(self)

        self.map = None
        self.marker_cluster = MarkerCluster()
        self.webView = QWebEngineView()
        self.setup_map(39, 35, 5)
        self.ui.v_lay_right.addWidget(self.webView)

        self.ui.btn_test_missions.clicked.connect(self.create_test_missions)
        self.ui.btn_delete_mission.clicked.connect(self.delete_mission)
        self.ui.btn_duplicate_mission.clicked.connect(self.duplicate_mission)
        self.ui.listWidget.itemSelectionChanged.connect(self.enable_buttons)
        self.refresh_mission_list()
        self.refresh_general_map()

    # Creates a map given center point and zoom level
    def setup_map(self, lat, lon, zoom):
        self.map = folium.Map(location=[lat, lon],
                              zoom_start=zoom,
                              control_scale=True, )

        self.map.add_child(MousePosition(position="topright", separator=" | ", empty_string="NaN", lng_first=False, ))
        self.map.add_child(self.marker_cluster)

        self.save_map()

    # Saves and shows the Map
    def save_map(self):
        self.map.save('./UI/Preflight1/pre1_map.html')
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(open('./UI/Preflight1/pre1_map.html').read())
        self.webView.show()

    # Adds marker to the Map
    def add_marker(self, lat, lon, popup_text):
        folium.Marker([lat, lon], popup=popup_text).add_to(self.marker_cluster)
        self.save_map()

    # Deletes the mission selected from the list from the database
    # TODO: Related thread will be removed
    def delete_mission(self):
        selected_items = self.ui.listWidget.selectedItems()

        # Find selected mission(s)
        for item in selected_items:
            mission_id = int(item.text().split(":")[1].split(",")[0].strip())
            mission = session.query(Mission).filter_by(mission_id=mission_id).first()

            # Delete the mission from the database
            if mission:
                self.mission_deleted.emit(mission_id)
                session.delete(mission)
                session.commit()

        # Refresh the Mission List and the Map
        self.refresh_mission_list()
        self.refresh_general_map()

    # Duplicates the mission selected from the list
    def duplicate_mission(self):
        selected_items = self.ui.listWidget.selectedItems()

        # Find selected mission(s)
        for item in selected_items:
            old_mission_id = int(item.text().split(":")[1].split(",")[0].strip())
            old_mission = session.query(Mission).filter_by(mission_id=old_mission_id).first()

            # Create a new "Draft" mission
            new_mission = Mission(
                center_lat=old_mission.center_lat,
                center_lon=old_mission.center_lon,
                coordinates=old_mission.coordinates,
                mission_status="Draft",
                estimated_mission_time=old_mission.estimated_mission_time,
                actual_mission_time=old_mission.actual_mission_time,
                required_battery_capacity=old_mission.required_battery_capacity,
                selected_area=old_mission.selected_area,
                scanned_area=old_mission.scanned_area,
                altitude=old_mission.altitude,
                gimbal_angle=old_mission.gimbal_angle,
                route_angle=old_mission.route_angle,
                rotated_route_angle=old_mission.rotated_route_angle,
            )

            session.add(new_mission)
            session.commit()

            # Find all drones matching the old Mission
            old_drones = session.query(Drone).filter_by(mission_id=old_mission.mission_id).all()
            for old_drone in old_drones:
                # Create a new Drone with same features as the old one
                new_drone = Drone(
                    model=old_drone.model,
                    battery_no=old_drone.battery_no,
                    flight_status=old_drone.flight_status,
                    gps_status=old_drone.gps_status,
                    connection_status=old_drone.connection_status,
                    mission_id=new_mission.mission_id
                )

                session.add(new_drone)
                session.commit()

        # Refresh the Mission List and the Map
        self.refresh_mission_list()
        self.refresh_general_map()

    # Creates test missions
    def create_test_missions(self):

        # Test Mission 1 RECTANGLE
        test_mission_1 = Mission(
            center_lat=41.0855452,
            center_lon=29.0406428,
            coordinates="[[41.085815, 29.040274], [41.085334, 29.040121], [41.085137, 29.041192], [41.085625, 29.041353], [41.085815, 29.040274]]",
            mission_status="Draft",
            required_battery_capacity=0,
            selected_area=5972,
            altitude=20,
            gimbal_angle=-85,
            route_angle=0,
            rotated_route_angle=20,
        )

        session.add(test_mission_1)
        session.commit()

        test_drone_1 = Drone(
            model="Parrot Anafi 4k",
            battery_no=3,
            mission_id=test_mission_1.mission_id
        )

        session.add(test_drone_1)
        session.commit()

        # Test Mission 2 SQUARE
        test_mission_2 = Mission(
            center_lat=41.0854778,
            center_lon=29.040432199999998,
            coordinates="[[41.085321, 29.040092], [41.085839, 29.040256], [41.085714, 29.040945], [41.085194, 29.040776], [41.085321, 29.040092]]",
            mission_status="Draft",
            required_battery_capacity=0,
            selected_area=4087,
            altitude=20,
            gimbal_angle=-85,
            route_angle=0,
            rotated_route_angle=20,
        )

        session.add(test_mission_2)
        session.commit()

        test_drone_2 = Drone(
            model="Parrot Anafi 4k",
            battery_no=3,
            mission_id=test_mission_2.mission_id
        )

        session.add(test_drone_2)
        session.commit()

        self.refresh_mission_list()
        self.refresh_general_map()

    # Gets all missions from the database, adds them to the Mission List and adds markers to the Map
    def refresh_mission_list(self):
        self.ui.listWidget.clear()

        # Get all missions from the database
        missions = get_all_missions()
        for mission in missions:
            # Add missions to the Mission List
            item = QListWidgetItem(f"Mission ID: {mission.mission_id}, Status: {mission.mission_status}")
            self.ui.listWidget.addItem(item)

        # Disable Edit Mission and Delete Mission buttons
        self.disable_buttons()

    def refresh_general_map(self):

        # Generate a new map in order to clear all markers
        self.setup_map(39, 35, 5)

        # Get all missions from the database
        missions = get_all_missions()
        for mission in missions:
            # Get center points of the missions and put markers for them to the Map
            lat = mission.center_lat
            lon = mission.center_lon
            if lat is not None:
                popup_text = f"Mission ID: {mission.mission_id}, Status: {mission.mission_status}"
                self.add_marker(lat, lon, popup_text)

        # Save and display the updated map
        self.save_map()

    # Enables Edit Mission, Duplicate Mission and Delete Mission buttons
    def enable_buttons(self):
        if len(self.ui.listWidget.selectedIndexes()):
            self.ui.btn_edit_mission.setEnabled(True)
            self.ui.btn_delete_mission.setEnabled(True)
            self.ui.btn_duplicate_mission.setEnabled(True)
            self.zoom_selected_mission()

    # Zooms into the selected mission on the map
    def zoom_selected_mission(self):
        if len(self.ui.listWidget.selectedIndexes()):
            selected_item = self.ui.listWidget.selectedItems()[0]
            mission_id = int(selected_item.text().split(":")[1].split(",")[0].strip())
            mission = session.query(Mission).filter_by(mission_id=mission_id).first()

            # Draw previously selected area if available in the database
            if not (mission.coordinates == 'null' or mission.coordinates is None):

                # Generate a new map in order to clear all markers
                self.map = folium.Map(location=[39, 35], zoom_start=5, control_scale=True)

                coords = json.loads(mission.coordinates)

                # Draw polygon and add to the map
                fg = folium.FeatureGroup(name="ScanArea")
                fg.add_child(folium.Polygon(coords))
                self.map.add_child(fg)

                # Set the map to show the polygon
                sw_point, ne_point = calculate_sw_ne_points(coords)
                self.map.fit_bounds([sw_point, ne_point])
                self.save_map()
            else:
                self.refresh_general_map()

    # Captures button pressings and performs necessary actions
    def keyPressEvent(self, event):
        # Clear selection and zooms out the map when ESC button is pressed
        if event.key() == Qt.Key.Key_Escape:
            self.ui.listWidget.clearSelection()
            self.refresh_mission_list()
            self.refresh_general_map()
        # Delete selected mission when DELETE/BACKSPACE button is pressed
        if event.key() == Qt.Key.Key_Delete or event.key() == Qt.Key.Key_Backspace:
            self.delete_mission()
        else:
            super().keyPressEvent(event)

    # Disables Edit Mission and Delete Mission buttons
    def disable_buttons(self):
        self.ui.btn_edit_mission.setEnabled(False)
        self.ui.btn_delete_mission.setEnabled(False)
        self.ui.btn_duplicate_mission.setEnabled(False)