import folium
from PyQt6.QtWidgets import QWidget, QListWidgetItem
from PyQt6.QtWebEngineWidgets import QWebEngineView

from UI.web_engine_page import WebEnginePage
import UI.Preflight1.pre1_design

from UI.database import Mission, session, get_all_missions, Drone


class Pre1(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Preflight1.pre1_design.Ui_Form()
        self.ui.setupUi(self)

        self.map = None
        self.webView = QWebEngineView()
        self.setup_map(39, 35, 5)
        self.ui.v_lay_right.addWidget(self.webView)

        self.ui.btn_delete_mission.clicked.connect(self.delete_mission)
        self.ui.btn_duplicate_mission.clicked.connect(self.duplicate_mission)
        self.ui.listWidget.itemSelectionChanged.connect(self.enable_buttons)
        self.refresh_mission_list()

    # Creates a map given center point and zoom level
    def setup_map(self, lat, lon, zoom):
        self.map = folium.Map(location=[lat, lon],
                              zoom_start=zoom,
                              control_scale=True, )
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
        folium.Marker([lat, lon], popup=popup_text).add_to(self.map)
        self.save_map()

    # Deletes the mission selected from the list from the database
    def delete_mission(self):
        selected_items = self.ui.listWidget.selectedItems()

        # Find selected mission(s)
        for item in selected_items:
            mission_id = int(item.text().split(":")[1].split(",")[0].strip())
            mission = session.query(Mission).filter_by(mission_id=mission_id).first()

            # Delete the mission from the database
            if mission:
                session.delete(mission)
                session.commit()
        # Refresh the Mission List
        self.refresh_mission_list()

    # Duplicate the mission selected from the list
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

        # Refresh the Mission List
        self.refresh_mission_list()

    # Gets all missions from the database, adds them to the Mission List and adds markers to the Map
    def refresh_mission_list(self):
        self.ui.listWidget.clear()

        # Get all missions from the database
        missions = get_all_missions()

        # Generate a new map in order to clear all markers
        self.map = folium.Map(location=[39, 35], zoom_start=5, control_scale=True)

        for mission in missions:
            # Add missions to the Mission List
            item = QListWidgetItem(f"Mission ID: {mission.mission_id}, Status: {mission.mission_status}")
            self.ui.listWidget.addItem(item)

            # Get center points of the missions and put markers for them to the Map
            lat = mission.center_lat
            lon = mission.center_lon
            if lat is not None:
                popup_text = f"Mission ID: {mission.mission_id}, Status: {mission.mission_status}"
                self.add_marker(lat, lon, popup_text)

        # Save and display the updated map
        self.save_map()

        # Disable Edit Mission and Delete Mission buttons
        self.disable_buttons()

    # Enables Edit Mission and Delete Mission buttons
    def enable_buttons(self):
        if len(self.ui.listWidget.selectedIndexes()):
            self.ui.btn_edit_mission.setEnabled(True)
            self.ui.btn_delete_mission.setEnabled(True)
            self.ui.btn_duplicate_mission.setEnabled(True)


    # Disables Edit Mission and Delete Mission buttons
    def disable_buttons(self):
        self.ui.btn_edit_mission.setEnabled(False)
        self.ui.btn_delete_mission.setEnabled(False)
        self.ui.btn_duplicate_mission.setEnabled(False)
