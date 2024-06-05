import json
import folium
import UI.Preflight1.pre1_design

from folium.plugins import MousePosition, MarkerCluster
from PyQt6.QtCore import Qt, pyqtSignal, QSize
from PyQt6.QtWidgets import QWidget, QListWidgetItem, QLabel
from PyQt6.QtWebEngineWidgets import QWebEngineView
from UI.database import Mission, Drone, get_mission_by_id, delete_mission_by_id, Node, get_all_missions, Path, session
from UI.ListItems.mission import Ui_Form
from UI.helpers import WebEnginePage, calculate_sw_ne_points, get_current_time


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
        self.marker_cluster = None
        self.marker_cluster = MarkerCluster()
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
    def add_marker(self, node, popup_text):
        folium.Marker([node.latitude, node.longitude], popup=popup_text).add_to(self.marker_cluster)
        self.save_map()

    # Deletes the mission selected from the list from the database
    # TODO: Related thread will be removed
    def delete_mission(self):
        selected_items = self.ui.listWidget.selectedItems()

        # Find selected mission(s)
        for item in selected_items:
            mission_id = int(item.listWidget().itemWidget(item).findChild(QLabel, "id_text").text())
            delete_mission_by_id(mission_id)
            self.mission_deleted.emit(mission_id)

        # Refresh the Mission List and the Map
        self.refresh_mission_list()
        self.refresh_general_map()

    # Duplicates the mission selected from the list
    def duplicate_mission(self):
        selected_items = self.ui.listWidget.selectedItems()

        # Find selected mission(s)
        for item in selected_items:
            old_mission_id = int(item.listWidget().itemWidget(item).findChild(QLabel, "id_text").text())
            old_mission = get_mission_by_id(old_mission_id)

            # Create a new "Draft" mission
            new_mission = Mission(
                creation_time=get_current_time(),
                last_update_time=get_current_time(),
                mission_boundary=old_mission.mission_boundary,
                mission_status="Draft",
                estimated_mission_time=old_mission.estimated_mission_time,
                provided_battery_capacity=old_mission.provided_battery_capacity,
                required_battery_capacity=old_mission.required_battery_capacity,
                selected_area=old_mission.selected_area,
                altitude=old_mission.altitude,
                gimbal_angle=old_mission.gimbal_angle,
                route_angle=old_mission.route_angle,
                rotated_route_angle=old_mission.rotated_route_angle,
            )
            new_mission.add_to_db()

            if old_mission.center_node is not None:
                center_node = Node(
                    latitude=old_mission.center_node.latitude,
                    longitude=old_mission.center_node.longitude)
                center_node.add_to_db()
                new_mission.center_node = center_node
                session.commit()

            if old_mission.gcs_node is not None:
                gcs_node = Node(
                    latitude=old_mission.gcs_node.latitude,
                    longitude=old_mission.gcs_node.longitude)
                gcs_node.add_to_db()
                new_mission.gcs_node = gcs_node
                session.commit()

            # Find all drones matching the old Mission
            for old_drone in old_mission.mission_drones:
                old_path = old_drone.path

                lv_node = Node(latitude=500, longitude=500)
                lv_node.add_to_db()

                # Create a new Path with same features as the old one
                new_path = Path(
                    path_boundary=old_path.path_boundary,
                    vertex_count=old_path.vertex_count,
                    opt_route=old_path.opt_route,
                    rot_route=old_path.rot_route,
                    opt_route_length=old_path.opt_route_length,
                    rot_route_length=old_path.rot_route_length,
                    mission_id=new_mission.mission_id,
                    last_visited_node_id=lv_node.node_id,
                )
                new_path.add_to_db()

                # Create a new Drone with same features as the old one
                new_drone = Drone(
                    model=old_drone.model,
                    ip_address=old_drone.ip_address,
                    battery_no=old_drone.battery_no,
                    mission_id=new_mission.mission_id,
                    path_id=new_path.path_id,
                )
                new_drone.add_to_db()

        # Refresh the Mission List and the Map
        self.refresh_mission_list()
        self.refresh_general_map()

    # Creates test missions
    def create_test_missions(self):
        # Test Mission 1 RECTANGLE
        lv_node_1 = Node(latitude=500, longitude=500)
        lv_node_1.add_to_db()

        center_node_1 = Node(latitude=41.0854276, longitude=29.0406022)
        center_node_1.add_to_db()

        gcs_node_1 = Node(latitude=41.085089, longitude=29.040599)
        gcs_node_1.add_to_db()

        test_mission_1 = Mission(
            creation_time=get_current_time(),
            last_update_time=get_current_time(),
            center_node_id=center_node_1.node_id,
            gcs_node_id=gcs_node_1.node_id,
            mission_boundary="[[41.085344, 29.040127], [41.085148, 29.041186], [41.085556, 29.041315], [41.085746, 29.040256], [41.085344, 29.040127]]",
            mission_status="Draft",
            required_battery_capacity=0,
            selected_area=5972,
            altitude=20,
            gimbal_angle=-85,
            route_angle=0,
            rotated_route_angle=20
        )
        test_mission_1.add_to_db()

        test_path_1 = Path(
            path_boundary=test_mission_1.mission_boundary,
            opt_route_length=774,
            rot_route_length=811,
            vertex_count=42,
            mission_id=test_mission_1.mission_id,
            last_visited_node_id=lv_node_1.node_id,
        )
        test_path_1.add_to_db()

        test_drone_1 = Drone(
            model="Parrot Anafi 4k",
            ip_address="192.168.53.1",
            battery_no=3,
            mission_id=test_mission_1.mission_id,
            path_id=test_path_1.path_id,
        )
        test_drone_1.add_to_db()

        # Test Mission 2 SQUARE
        lv_node_2 = Node(latitude=500, longitude=500)
        lv_node_2.add_to_db()

        center_node_2 = Node(latitude=41.0854778, longitude=29.0404322)
        center_node_2.add_to_db()

        gcs_node_2 = Node(latitude=41.085089, longitude=29.040599)
        gcs_node_2.add_to_db()

        test_mission_2 = Mission(
            creation_time=get_current_time(),
            last_update_time=get_current_time(),
            center_node_id=center_node_2.node_id,
            gcs_node_id=gcs_node_2.node_id,
            mission_boundary="[[41.085273, 29.040307], [41.085148, 29.040999], [41.085598, 29.041143], [41.085718, 29.040454], [41.085273, 29.040307]]",
            mission_status="Draft",
            required_battery_capacity=0,
            selected_area=4087,
            altitude=20,
            gimbal_angle=-85,
            route_angle=0,
            rotated_route_angle=20
        )
        test_mission_2.add_to_db()

        test_path_2 = Path(
            path_boundary=test_mission_2.mission_boundary,
            opt_route_length=585,
            rot_route_length=592,
            vertex_count=42,
            mission_id=test_mission_2.mission_id,
            last_visited_node_id=lv_node_2.node_id,
            )
        test_path_2.add_to_db()

        test_drone_2 = Drone(
            model="Parrot Anafi 4k",
            ip_address="192.168.53.1",
            battery_no=3,
            mission_id=test_mission_2.mission_id,
            path_id=test_path_2.path_id,
        )
        test_drone_2.add_to_db()

        # Test Mission 3 for SIMULATION
        lv_node_3 = Node(latitude=500, longitude=500)
        lv_node_3.add_to_db()

        center_node_3 = Node(latitude=48.8806686, longitude=2.3706136)
        center_node_3.add_to_db()

        gcs_node_3 = Node(latitude=48.880744, longitude=2.369775)
        gcs_node_3.add_to_db()

        test_mission_3 = Mission(
            creation_time=get_current_time(),
            last_update_time=get_current_time(),
            center_node_id=center_node_3.node_id,
            gcs_node_id=gcs_node_3.node_id,
            mission_boundary="[[48.880896, 2.371609], [48.879753, 2.370397], [48.880317, 2.369131], [48.881481, 2.370322], [48.880896, 2.371609]]",
            mission_status="Draft",
            required_battery_capacity=2,
            selected_area=26777,
            altitude=120,
            gimbal_angle=-85,
            route_angle=0,
            rotated_route_angle=20
        )
        test_mission_3.add_to_db()

        test_path_3 = Path(
            path_boundary=test_mission_3.mission_boundary,
            opt_route_length=733,
            rot_route_length=595,
            vertex_count=16,
            mission_id=test_mission_3.mission_id,
            last_visited_node_id=lv_node_3.node_id,
        )
        test_path_3.add_to_db()

        test_drone_3 = Drone(
            model="Parrot Anafi 4k",
            ip_address="10.202.0.1",
            battery_no=99,
            mission_id=test_mission_3.mission_id,
            path_id=test_path_3.path_id,
        )
        test_drone_3.add_to_db()

        self.refresh_mission_list()
        self.refresh_general_map()

    # Gets all missions from the database, adds them to the Mission List and adds markers to the Map
    def refresh_mission_list(self):
        self.ui.listWidget.clear()

        # Get all missions from the database
        for mission in get_all_missions():
            # Add missions to the Mission List
            self.add_mission_to_list(mission)

        # Disable Edit Mission and Delete Mission buttons
        self.disable_buttons()

    # Adds given mission to the Mission List
    def add_mission_to_list(self, mission):
        new_mission_item = QListWidgetItem()
        new_mission_widget = QWidget()
        new_mission_ui = Ui_Form()
        new_mission_ui.setupUi(new_mission_widget)
        new_mission_ui.id_text.setText(str(mission.mission_id))
        new_mission_ui.creation_time_text.setText(mission.creation_time)
        new_mission_ui.last_update_time_text.setText(mission.last_update_time)
        new_mission_ui.status_text.setText(mission.mission_status)

        # Calculate the height of the new_mission_widget
        new_mission_widget.adjustSize()
        widget_height = new_mission_widget.sizeHint().height()

        # Set the size hint for the item
        new_mission_item.setSizeHint(QSize(self.ui.listWidget.lineWidth(), widget_height))

        self.ui.listWidget.addItem(new_mission_item)
        self.ui.listWidget.setItemWidget(new_mission_item, new_mission_widget)

    def refresh_general_map(self):
        # Generate a new map in order to clear all markers
        self.setup_map(39, 35, 5)

        # Get all missions from the database
        for mission in get_all_missions():
            # Get center points of the missions and put markers for them to the Map
            if mission.center_node is not None:
                popup_text = (f"<h5 style='text-align:center'>MISSION #{mission.mission_id}</h5>"
                              f"<b>Mission Status:</b> {mission.mission_status}<br>"
                              f"<b>Selected Area:</b> {mission.selected_area}m<sup>2</sup><br>")
                popup = folium.Popup(popup_text, max_width=150, min_width=150)
                self.add_marker(mission.center_node, popup)
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
            item = self.ui.listWidget.selectedItems()[0]
            mission_id = int(item.listWidget().itemWidget(item).findChild(QLabel, "id_text").text())
            mission = get_mission_by_id(mission_id)

            # Draw previously selected area if available in the database
            if not (mission.mission_boundary == 'null' or mission.mission_boundary is None):

                # Generate a new map in order to clear all markers
                self.map = folium.Map(location=[39, 35], zoom_start=5, control_scale=True)

                coords = json.loads(mission.mission_boundary)

                # Draw polygon and add to the map
                fg = folium.FeatureGroup(name="ScanArea")
                fg.add_child(folium.Polygon(coords, fill='True'))
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
